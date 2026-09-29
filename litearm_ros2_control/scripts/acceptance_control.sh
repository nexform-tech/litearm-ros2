#!/usr/bin/env bash
# acceptance_control.sh — 控制栈全链路验收（无硬件）。
#
#   起栈（守护进程 dry-run + ros2_control + JSB + JTC）
#     → 下发一条两点轨迹 → 回读 /joint_states 校验跟踪误差 → 收栈
#
# 为什么单独成脚本而不是一行 bash：命令行里含 "litearm_control.launch" 这类
# 字符串时，pkill -f 会把执行该命令的 shell 自己也匹配上并杀掉（自匹配陷阱）。
# 放进脚本后进程名是 "bash acceptance_control.sh"，不会自匹配。
#
# 用法:
#   scripts/acceptance_control.sh              # dry-run（无硬件）
#   scripts/acceptance_control.sh --real       # 真机（需板子已连、license 已激活）
#
# 环境注意：ROS 2 默认域 0 且组播发现是全网的。若同网段还有其他机器人/实验在跑
# ROS 2，它们的节点（以及 /move_action、/joint_states 这类同名话题）会被发现到，
# 导致目标被陌生节点接管、状态被污染，表现为"轨迹发出去却不执行"或随机失败。
# 因此这里用独立域名 + 只走本机。
#
# ⚠ 下面两行是 `${VAR:-默认}` —— **只在你没设过时才取默认值**。
#   如果 shell 里已经 export 了 ROS_LOCALHOST_ONLY=0（比如为了对接别的系统设过），
#   本脚本会沿用 0，发现走全部网卡。此时**探针会报 "action server 未出现"**，而真相
#   是 JTC 明明已激活（日志里有 Configured and activated）——只是探针发现不到它。
#   碰上这个现象先 `echo $ROS_LOCALHOST_ONLY`，再用 ROS_LOCALHOST_ONLY=1 重跑。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 工作区根：脚本可能从源码树（<ws>/src/<pkg>/scripts/）运行，也可能从安装树
# （<ws>/install/<pkg>/lib/<pkg>/）运行，两种深度不同。这里向上找同时含
# src/ 与 install/ 的目录，而不是靠固定层数推算。
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
    echo "找不到工作区根（需要同时存在 src/ 与 install/setup.bash）：$SCRIPT_DIR" >&2
    exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

LOG="${TMPDIR:-/tmp}/litearm_acceptance_control.log"
PROBE="$SCRIPT_DIR/e2e_probe.py"

cd "$WS_DIR"
set +u
# ROS 环境：不写死发行版，用 $ROS_DISTRO 或默认 humble 定位安装前缀。
# 若已 source 过其他发行版，请显式导出 ROS_DISTRO 再运行本脚本。
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
set -u

REAL_ARGS=("dry_run:=true")
if [ "${1:-}" = "--real" ]; then
    REAL_ARGS=("dry_run:=false")
    echo "[accept] ★ 真机模式：机械臂将实际运动，确认空间开阔、已扶好急停"
fi

rm -f "$LOG"
setsid ros2 launch litearm_ros2_control litearm_control.launch.py \
    "${REAL_ARGS[@]}" >"$LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "[accept] 控制栈已启动 pid=$LAUNCH_PID domain=$ROS_DOMAIN_ID log=$LOG"

cleanup() {
    local pgid
    pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
    if [ -n "$pgid" ]; then
        kill -TERM -"$pgid" 2>/dev/null
        sleep 4
        kill -KILL -"$pgid" 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
    echo "[accept] 已收栈（守护进程退出时会做 MIT 高刚度持位，臂保持原位）"
}
trap cleanup EXIT

deadline=$((SECONDS + 90))
while (( SECONDS < deadline )); do
    if grep -q "Configured and activated .*joint_trajectory_controller" "$LOG" 2>/dev/null; then
        echo "[accept] 栈已就绪"
        break
    fi
    if grep -qiE "\[FATAL\]|\[ERROR\].*hardware|Failed" "$LOG" 2>/dev/null; then
        echo "[accept] 启动失败："
        grep -iE "\[FATAL\]|\[ERROR\]" "$LOG" | head -20
        exit 1
    fi
    sleep 1
done
if ! grep -q "Configured and activated .*joint_trajectory_controller" "$LOG" 2>/dev/null; then
    echo "[accept] 超时未就绪，日志尾部："
    tail -30 "$LOG"
    exit 1
fi

python3 "$PROBE"
RC=$?

echo "[accept] ===== 启动期告警/错误 ====="
grep -iE "\[ERROR\]|\[FATAL\]" "$LOG" | head -10 || echo "（无）"
echo "[accept] 探针退出码=$RC"
exit $RC
