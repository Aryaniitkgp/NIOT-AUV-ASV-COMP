#!/usr/bin/env bash
#
# Run the mission repeatedly until every check passes, or the attempt
# budget runs out.
#
#   ./run_until_complete.sh [seconds] [max_attempts]      default 200 5
#
# Each attempt is a full headless run via run_mission_log.sh. After every
# attempt the verdict block is read back and, if anything failed, the
# specific failure is reported so the next attempt can be made against a
# fix rather than blindly repeated.
#
# This does NOT edit the code between attempts - a script cannot diagnose
# a perception bug. It surfaces exactly which check failed and where the
# vehicle ended up, which is what a fix needs.

set -u

DURATION="${1:-200}"
MAX_ATTEMPTS="${2:-5}"
PROJECT_DIR="/home/aryan/niot"

cd "${PROJECT_DIR}" || exit 1

for (( attempt=1; attempt<=MAX_ATTEMPTS; attempt++ )); do
    echo ""
    echo "################################################################"
    echo "# ATTEMPT ${attempt} / ${MAX_ATTEMPTS}   (${DURATION}s)"
    echo "################################################################"

    # Make sure nothing survived from the previous attempt. A leftover
    # gz server keeps the world in whatever state the last run left it,
    # so the vehicle starts from where it drifted to rather than from
    # spawn. Attempt 1 of a real 3-run sweep began at 1.36 m depth
    # instead of 0.08 m for exactly this reason, chased the wrong buoy
    # just under the surface, and finished 7.78 m from the octagon.
    ./kill_sim.sh > /dev/null 2>&1
    # ros2 topic echo recorders from the previous attempt can still be
    # draining their buffers when the next one starts, which appends the
    # new run's samples to the old attempt's CSVs. One attempt's state.csv
    # ended "SURFACED, DIVE, SURFACED" - the next run's startup leaking in.
    pkill -KILL -f 'ros2 topic echo' > /dev/null 2>&1
    sleep 3

    out="/tmp/mission_attempt_${attempt}.txt"
    ./run_mission_log.sh "${DURATION}" > "${out}" 2>&1

    # Progress bar lines are noise; everything else is the report.
    grep -vE "^\[run\] +[0-9]+s" "${out}" | sed -n '/=============== SUMMARY/,$p'

    if grep -q '^MISSION_COMPLETE' "${out}"; then
        echo ""
        echo "################################################################"
        echo "# MISSION COMPLETE on attempt ${attempt}"
        echo "################################################################"
        exit 0
    fi

    echo ""
    echo "--- attempt ${attempt} failed these checks ---"
    grep '^  \[FAIL\]' "${out}" || echo "  (verdict block missing - run died early?)"

    # A run that never reached the verdict block usually means a crash or
    # a startup race, which is worth separating from a genuine mission
    # failure.
    if ! grep -q 'MISSION_' "${out}"; then
        echo ""
        echo "  !! no verdict produced. Last mission log lines:"
        latest=$(ls -td "${PROJECT_DIR}"/mission_logs/*/ 2>/dev/null | head -1)
        [ -n "${latest}" ] && tail -5 "${latest}/mission.log" 2>/dev/null | sed 's/^/     /'
    fi

    sleep 3
done

echo ""
echo "################################################################"
echo "# Gave up after ${MAX_ATTEMPTS} attempts."
echo "# Per-attempt output: /tmp/mission_attempt_*.txt"
echo "################################################################"
exit 1
