#!/usr/bin/env bash
# acceptance_control.sh — end-to-end acceptance of the control stack (no hardware).
#
#   bring up the stack (daemon dry-run + ros2_control + JSB + JTC)
#     → send a two-point trajectory → read back /joint_states and check the
#       tracking error → tear the stack down
#
# Why this is a separate script rather than a bash one-liner: when the command
# line contains a string like "litearm_control.launch", pkill -f also matches
# the shell running that very command and kills it (the self-match trap).
# Inside a script the process name is "bash acceptance_control.sh", which does
# not self-match.
#
# Usage:
#   scripts/acceptance_control.sh              # dry-run (no hardware)
#   scripts/acceptance_control.sh --real       # real hardware (board connected,
#                                              # license activated)
#
# Environment caveat: ROS 2 defaults to domain 0 and multicast discovery spans
# the whole network. If other robots/experiments on the same subnet are running
# ROS 2, their nodes (and same-named topics such as /move_action,
# /joint_states) get discovered, so goals are taken over by unfamiliar nodes
# and state is corrupted, which shows up as "the trajectory is sent but never
# executed" or as random failures. Hence a dedicated domain here + localhost
# only.
#
# ⚠ The next two lines are `${VAR:-default}` — the default is **only taken when
#   you have not set the variable yourself**. If the shell already exported
#   ROS_LOCALHOST_ONLY=0 (say, to interface with another system), this script
#   keeps 0 and discovery goes out over every NIC. In that case **the probe
#   reports "action server did not appear"**, while the truth is that JTC is
#   activated (the log says Configured and activated) — the probe simply
#   cannot discover it. If you hit this, run `echo $ROS_LOCALHOST_ONLY` first,
#   then rerun with ROS_LOCALHOST_ONLY=1.

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Workspace root: the script may run from the source tree
# (<ws>/src/<pkg>/scripts/) or from the install tree
# (<ws>/install/<pkg>/lib/<pkg>/), which sit at different depths. So walk up
# looking for a directory that holds both src/ and install/, instead of
# assuming a fixed number of levels.
_find_ws() {
    local d="$SCRIPT_DIR"
    while [ "$d" != "/" ]; do
        if [ -f "$d/install/setup.bash" ] && [ -d "$d/src" ]; then
            echo "$d"
            return 0
        fi
        d="$(dirname "$d")"
    done
    return 1
}

if ! WS_DIR="$(_find_ws)"; then
    echo "cannot find the workspace root (needs both src/ and install/setup.bash): $SCRIPT_DIR" >&2
    exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

LOG="${TMPDIR:-/tmp}/litearm_acceptance_control.log"
PROBE="$SCRIPT_DIR/e2e_probe.py"

cd "$WS_DIR"
set +u
# ROS environment: do not hard-code the distro; locate the install prefix via
# $ROS_DISTRO, defaulting to humble. If you have already sourced another
# distro, export ROS_DISTRO explicitly before running this script.
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
set -u

REAL_ARGS=("dry_run:=true")
if [ "${1:-}" = "--real" ]; then
    REAL_ARGS=("dry_run:=false")
    echo "[accept] ★ real hardware mode: the arm will actually move — make sure the workspace is clear and keep a hand on the emergency stop"
fi

rm -f "$LOG"
setsid ros2 launch litearm_ros2_control litearm_control.launch.py \
    "${REAL_ARGS[@]}" >"$LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "[accept] control stack started pid=$LAUNCH_PID domain=$ROS_DOMAIN_ID log=$LOG"

cleanup() {
    local pgid
    pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
    if [ -n "$pgid" ]; then
        kill -TERM -"$pgid" 2>/dev/null
        sleep 4
        kill -KILL -"$pgid" 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
    echo "[accept] stack torn down (the daemon holds position with high MIT stiffness on exit, so the arm stays where it is)"
}
trap cleanup EXIT

deadline=$((SECONDS + 90))
while (( SECONDS < deadline )); do
    if grep -q "Configured and activated .*joint_trajectory_controller" "$LOG" 2>/dev/null; then
        echo "[accept] stack is ready"
        break
    fi
    if grep -qiE "\[FATAL\]|\[ERROR\].*hardware|Failed" "$LOG" 2>/dev/null; then
        echo "[accept] startup failed:"
        grep -iE "\[FATAL\]|\[ERROR\]" "$LOG" | head -20
        exit 1
    fi
    sleep 1
done
if ! grep -q "Configured and activated .*joint_trajectory_controller" "$LOG" 2>/dev/null; then
    echo "[accept] timed out before becoming ready, tail of the log:"
    tail -30 "$LOG"
    exit 1
fi

python3 "$PROBE"
RC=$?

echo "[accept] ===== warnings/errors during startup ====="
grep -iE "\[ERROR\]|\[FATAL\]" "$LOG" | head -10 || echo "(none)"
echo "[accept] probe exit code=$RC"
exit $RC
