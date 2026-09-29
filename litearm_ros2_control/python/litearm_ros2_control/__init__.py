#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_ros2_control — litearm 的 ros2_control 适配层。

对外暴露：

* :mod:`litearm_ros2_control.shm_bridge` —— 共享内存契约的 Python 侧绑定。
* :mod:`litearm_ros2_control.hw_daemon`  —— 硬件守护进程（独占硬件、写状态块）。
* :mod:`litearm_ros2_control.stm32_proto` —— litearm-stm32 USB CDC 协议的字节层。
* :mod:`litearm_ros2_control.stm32_link`  —— 该协议的串口链路与请求/应答。
* :mod:`litearm_ros2_control.fake_firmware` —— 固件行为替身（无硬件演练与测试）。

后三个是新的硬件后端：litearm-stm32（STM32 独占 CAN，PC 走 USB CDC），
正在替掉原来的 pylitearm 后端（PC 端 SocketCAN 直驱达妙电机）。
SHM 契约不随后端变化，所以 C++ 插件与 MoveIt 那侧无需改动。
"""

from litearm_ros2_control.ros_env import (DEFAULT_DOMAIN_ID, isolation_hint,
                                          parse_bool, ros_isolation_env)
from litearm_ros2_control.shm_bridge import (
    NUM_JOINTS,
    LitearmCommand,
    LitearmHeader,
    LitearmState,
    SharedMemory,
    ShmError,
    describe_layout,
)

__all__ = [
    "DEFAULT_DOMAIN_ID",
    "isolation_hint",
    "parse_bool",
    "ros_isolation_env",
    "NUM_JOINTS",
    "LitearmCommand",
    "LitearmHeader",
    "LitearmState",
    "SharedMemory",
    "ShmError",
    "describe_layout",
]
