#!/usr/bin/env bash
#
# Headless mission run with telemetry capture.
#
# Starts Gazebo (no GUI), the bridge, the thruster mixer and the mission
# FSM, records the topics that matter for diagnosing the run, then shuts
# everything down cleanly and prints a summary.
#
#   ./run_mission_log.sh [seconds]        default 120
#
# Everything lands in ./mission_logs/<timestamp>/
#
# NOTE: uses /usr/bin/python3 explicitly. The bluerov2_gz/.venv on PATH
# does not expose the Gazebo bindings and the bridge dies under it.

set -u

DURATION="${1:-200}"
PROJECT_DIR="/home/aryan/niot"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${PROJECT_DIR}/mission_logs/${STAMP}"
PY=/usr/bin/python3

mkdir -p "${LOG_DIR}"
cd "${PROJECT_DIR}" || exit 1

# ---------------------------------------------------------------- setup
set +u
source /opt/ros/humble/setup.bash || true
set -u
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/bluerov2_gz/models:${PROJECT_DIR}/bluerov2_gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Own ROS domain so a stray node from another session cannot join in and
# publish thruster commands alongside ours.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

PIDS=()

cleanup() {
    echo ""
    echo "[run] shutting down..."
    # Reverse order: mission first so it stops commanding, gz last.
    for (( i=${#PIDS[@]}-1 ; i>=0 ; i-- )); do
        kill -INT "${PIDS[i]}" 2>/dev/null
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -KILL "$pid" 2>/dev/null
    done
    # Anything that escaped the process group.
    pkill -f 'gz sim'                     2>/dev/null
    pkill -f bluerov2_native_bridge.py    2>/dev/null
    pkill -f thruster_mixer.py            2>/dev/null
    pkill -f line_buoy.py                 2>/dev/null
    wait 2>/dev/null
    echo "[run] done."
}
trap cleanup EXIT INT TERM

# ------------------------------------------------------------- launch
echo "[run] logging to ${LOG_DIR}  (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"

echo "[run] 1/4 gazebo (headless)"
gz sim -v 3 -r -s save_arena.sdf > "${LOG_DIR}/gazebo.log" 2>&1 &
PIDS+=($!)
sleep 8

echo "[run] 2/4 bridge"
$PY bluerov2_native_bridge.py > "${LOG_DIR}/bridge.log" 2>&1 &
PIDS+=($!)
sleep 4

echo "[run] 3/4 thruster mixer"
$PY thruster_mixer.py > "${LOG_DIR}/mixer.log" 2>&1 &
PIDS+=($!)
sleep 2

echo "[run] 4/4 mission FSM"
# Headless: no X display, so the OpenCV debug windows must not be opened.
QT_QPA_PLATFORM=offscreen $PY line_buoy.py > "${LOG_DIR}/mission.log" 2>&1 &
PIDS+=($!)
sleep 2

# ------------------------------------------------------------ recording
# Ground truth pose is what tells us whether the vehicle surfaced or
# stalled; the mission log gives the state machine's own view.
echo "[run] recording ${DURATION}s ..."
ros2 topic echo /bluerov2/odom      --csv > "${LOG_DIR}/odom.csv"    2>/dev/null &
PIDS+=($!)
ros2 topic echo /mission/state      --csv > "${LOG_DIR}/state.csv"   2>/dev/null &
PIDS+=($!)
ros2 topic echo /cmd_surge          --csv > "${LOG_DIR}/surge.csv"   2>/dev/null &
PIDS+=($!)
ros2 topic echo /cmd_yaw            --csv > "${LOG_DIR}/yaw.csv"     2>/dev/null &
PIDS+=($!)
ros2 topic echo /cmd_heave          --csv > "${LOG_DIR}/heave.csv"   2>/dev/null &
PIDS+=($!)

for (( t=0; t<DURATION; t+=10 )); do
    sleep 10
    printf '[run]   %3ds / %ds\n' "$((t+10))" "${DURATION}"
done

# --------------------------------------------------------------- report
echo ""
echo "=============== SUMMARY ==============="
echo "--- state transitions ---"
grep -E '^\[INFO\].*->' "${LOG_DIR}/mission.log" | sed 's/\[INFO\] \[[0-9.]*\] \[mission_control\]: //' | head -40

echo ""
echo "--- touches / drops ---"
grep -E '\*\*\*' "${LOG_DIR}/mission.log" | head -20

echo ""
echo "--- warnings / errors ---"
grep -E '^\[WARN\]|^\[ERROR\]' "${LOG_DIR}/mission.log" | sed 's/\[[0-9.]*\] \[mission_control\]: //' | sort -u | head -20

echo ""
echo "--- depth envelope (from ground truth) ---"
$PY - "${LOG_DIR}/odom.csv" <<'PYEOF'
import sys, csv
path = sys.argv[1]
zs, xs, ys = [], [], []
# Layout: sec, nanosec, frame_id, child_frame_id, pos.x, pos.y, pos.z, ...
# Columns 2 and 3 are STRINGS, so filtering the row to floats first
# shifts every index and silently yields nothing.
try:
    with open(path) as f:
        for row in csv.reader(f):
            if len(row) < 7:
                continue
            try:
                xs.append(float(row[4]))
                ys.append(float(row[5]))
                zs.append(float(row[6]))
            except ValueError:
                continue
except FileNotFoundError:
    print("  (no odom captured)"); sys.exit()

if not zs:
    print("  (no odom rows parsed)"); sys.exit()

SURFACE = 2.5
print(f"  samples        : {len(zs)}")
print(f"  z range        : {min(zs):.3f} .. {max(zs):.3f}")
print(f"  depth range    : {SURFACE-max(zs):.3f} .. {SURFACE-min(zs):.3f} m")
print(f"  shallowest     : {SURFACE-max(zs):.3f} m depth", end="")
print("   *** SURFACED ***" if max(zs) > 2.40 else "   (stayed submerged)")
print(f"  x travel       : {min(xs):.2f} .. {max(xs):.2f}")
print(f"  y travel       : {min(ys):.2f} .. {max(ys):.2f}")
PYEOF

echo ""
echo "--- state dwell (where the run actually spent its time) ---"
$PY - "${LOG_DIR}/state.csv" <<'PYEOF'
import sys, collections
try:
    rows = [l.strip().strip('"') for l in open(sys.argv[1]) if l.strip()]
except FileNotFoundError:
    print("  (no state data)"); sys.exit()
if not rows:
    print("  (no state data)"); sys.exit()
c = collections.Counter(rows)
total = sum(c.values())
for st, n in c.most_common():
    print(f"  {st:16s} {100.0*n/total:5.1f}%  ({n})")
PYEOF

echo ""
echo "--- stall check (surge near zero while yaw active) ---"
echo "    note: BUOY_SEARCH sweeps deliberately, so some low-surge is normal"
$PY - "${LOG_DIR}/surge.csv" "${LOG_DIR}/yaw.csv" <<'PYEOF'
import sys
def load(p):
    out=[]
    try:
        for line in open(p):
            line=line.strip()
            if not line: continue
            try: out.append(float(line.split(',')[-1]))
            except ValueError: pass
    except FileNotFoundError: pass
    return out
s=load(sys.argv[1]); y=load(sys.argv[2])
if not s or not y:
    print("  (no command data)")
else:
    n=min(len(s),len(y))
    stalled=sum(1 for i in range(n) if abs(s[i])<0.15 and abs(y[i])>1.0)
    print(f"  samples {n}, stalled frames {stalled} ({100.0*stalled/max(n,1):.1f}%)")
    print("  " + ("*** STALL DETECTED - turning without advancing ***" if stalled>0.10*n
                  else "no significant stall"))
PYEOF

echo ""
echo "--- MISSION VERDICT ---"
# Machine-readable pass/fail per task, so an iterating wrapper can decide
# whether to keep going without a human reading the prose above.
$PY - "${LOG_DIR}" <<'PYEOF'
import sys, os, csv, math, re

log_dir = sys.argv[1]
mission = os.path.join(log_dir, 'mission.log')
odom = os.path.join(log_dir, 'odom.csv')

text = open(mission).read() if os.path.exists(mission) else ''

# Ground truth end state.
rows = []
if os.path.exists(odom):
    with open(odom) as f:
        for r in csv.reader(f):
            if len(r) < 7:
                continue
            try:
                rows.append((float(r[4]), float(r[5]), float(r[6])))
            except ValueError:
                pass

SURFACE = 2.5
OCT = {'starboard': (8.0, -5.0), 'port': (8.0, 5.0)}
APOTHEM = 3.26

fx = fy = fz = None
if rows:
    fx, fy, fz = rows[-1]

checks = []
checks.append(('dive', 'DIVE -> LINE_FOLLOW' in text))
checks.append(('buoy touched', 'BUOY TOUCHED' in text))
checks.append(('fork chosen', 'FORK:' in text))

n_drop = len(re.findall(r'MARKER \d+/\d+ RELEASED', text))
checks.append((f'markers dropped ({n_drop}/2)', n_drop >= 2))

# Shallowest depth REACHED, not the depth at the last sample.
#
# Using the final row reports a failure when the vehicle surfaced
# correctly and then drifted back down - and it also picks up the next
# attempt's startup if the topic recorders are still draining. One run
# reached 0.071 m (clearly surfaced) but ended at 1.635 m and was scored
# FAIL on a check it had passed.
min_depth = min((SURFACE - z for _, _, z in rows), default=None)
surfaced = min_depth is not None and min_depth <= 0.20
checks.append((f'surfaced (min {min_depth:.2f}m)' if min_depth is not None
               else 'surfaced', surfaced))

inside = None
if fx is not None:
    d = min(math.hypot(fx - cx, fy - cy) for cx, cy in OCT.values())
    inside = d < APOTHEM
    checks.append((f'inside octagon (d={d:.2f}m)', inside))

# What the vehicle itself believed, which is the thing under test.
m = re.search(r'SURFACED at ([\d.]+) m depth - (\d+)/(\d+) frames', text)
if m:
    votes, total = int(m.group(2)), int(m.group(3))
    checks.append((f'containment detected ({votes}/{total})', votes > 0))
else:
    checks.append(('containment detected', False))

for name, ok in checks:
    print(f'  [{"PASS" if ok else "FAIL"}] {name}')

n_ok = sum(1 for _, ok in checks if ok)
print(f'  ---> {n_ok}/{len(checks)} checks passed')
print('MISSION_COMPLETE' if n_ok == len(checks) else 'MISSION_INCOMPLETE')
PYEOF

echo ""
echo "======================================="
echo "logs: ${LOG_DIR}"
