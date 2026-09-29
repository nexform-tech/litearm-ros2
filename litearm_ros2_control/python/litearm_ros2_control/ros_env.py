#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ros_env.py — launch 共用的 ROS 发现域隔离。

**故意不 import launch**：本模块会被 launch 文件使用，但 ``litearm_ros2_control``
包的 ``__init__`` 还要给守护进程 / 共享内存绑定用 —— 那里不该依赖 launch。
所以这里只提供纯数据与纯函数，LaunchDescription 的构造留在各 launch 文件里。

为什么必须做域隔离
------------------
ROS 2 默认 ``ROS_DOMAIN_ID=0`` 且组播发现是**全网段**的。同网段只要还有另一台
机器在跑 ROS 2（本工程实测遇到过一台 twoarm/rail 设备），就会互相发现，症状是：

* ``/robot_description`` 被对方的模型覆盖 —— RViz 去加载
  ``package://litearm_description/meshes/...``，本机没有这个包，于是机器人
  在界面上直接消失，日志里是 ``Package [litearm_description] does not exist``；
* ``/joint_states`` 混入别人的关节；
* ``/move_action`` 出现**多个 action server**，MoveIt 的规划目标被陌生节点的
  move_group 接管（它没有本机控制器，立刻返回 FAILURE），而本机 move_group
  同时也在执行 —— 表现为"报失败但臂确实动了"这种极难定位的现象。

修法只有一个：把域固定下来 + 只走本机。要对接外部系统时显式关掉。
"""

DEFAULT_DOMAIN_ID = "42"
"""默认 ROS 域。刻意避开 0：域 0 是最容易被同网段其他设备占用的。"""


def parse_bool(value: str, fallback: bool = False) -> bool:
    """宽容地解析 launch 传来的布尔字符串。"""
    if value is None:
        return fallback
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return fallback


def ros_isolation_env(domain_id: str, localhost_only: str) -> "dict[str, str]":
    """构造传给各节点的 additional_env：锁定 ROS 域、默认只走本机。"""
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    return {
        "ROS_DOMAIN_ID": domain,
        "ROS_LOCALHOST_ONLY": "1" if parse_bool(localhost_only, True) else "0",
    }


def isolation_hint(domain_id: str, localhost_only: str) -> str:
    """给用户看的提示：终端里要设同样的变量，否则 ros2 命令行看不到本栈话题。"""
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    if parse_bool(localhost_only, True):
        return (f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY=1")
    return (f"export ROS_DOMAIN_ID={domain}    "
            f"（已关闭本机隔离，注意跨机串扰）")


__all__ = ["DEFAULT_DOMAIN_ID", "isolation_hint", "parse_bool",
           "ros_isolation_env"]
