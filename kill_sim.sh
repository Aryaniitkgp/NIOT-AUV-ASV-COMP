#!/usr/bin/env bash
#
# Kill everything from a simulation run - Gazebo, the bridge, the mixer,
# the mission FSM, and any ros2 topic recorders left behind.
#
# Safe to run at any time, including when nothing is running.
#
#   ./kill_sim.sh

echo "[kill] stopping simulation processes..."

# SIGINT first so nodes shut down cleanly and release their topics.
for pat in line_buoy.py thruster_mixer.py bluerov2_native_bridge.py \
           'gz sim' 'ros2 topic echo' line_follow.py buoy.py; do
    if pkill -INT -f "$pat" 2>/dev/null; then
        echo "  INT  $pat"
    fi
done

sleep 2

# Anything still alive gets SIGKILL.
for pat in line_buoy.py thruster_mixer.py bluerov2_native_bridge.py \
           'gz sim' 'ros2 topic echo' line_follow.py buoy.py; do
    if pkill -KILL -f "$pat" 2>/dev/null; then
        echo "  KILL $pat"
    fi
done

# gz spawns helper processes (server/gui) that do not always match
# 'gz sim' - clear those too.
pkill -KILL -f 'gz-sim'     2>/dev/null
pkill -KILL -f ruby.*gz     2>/dev/null

sleep 1

echo ""
echo "[kill] survivors (should be empty):"
pgrep -af 'gz sim|gz-sim|bluerov2_native_bridge|thruster_mixer|line_buoy' \
    | grep -v 'kill_sim' || echo "  none - all clear"
