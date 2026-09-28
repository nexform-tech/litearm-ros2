#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_ros2_control — the ros2_control adapter layer for litearm.

Public surface:

* :mod:`litearm_ros2_control.shm_bridge` — Python-side bindings for the shared
  memory contract.
* :mod:`litearm_ros2_control.hw_daemon`  — the hardware daemon (owns the hardware,
  writes the state block).
* :mod:`litearm_ros2_control.stm32_proto` — byte layer of the litearm-stm32 USB CDC
  protocol.
* :mod:`litearm_ros2_control.stm32_link`  — serial link and request/response for that
  protocol.
* :mod:`litearm_ros2_control.fake_firmware` — stand-in for the firmware's behaviour
  (hardware-free dry runs and tests).

The last three are the new hardware backend: litearm-stm32 (the STM32 owns the CAN bus,
the PC talks over USB CDC), which is replacing the old pylitearm backend (PC-side
SocketCAN driving Damiao motors directly). The SHM contract does not change with the
backend, so the C++ plugin and the MoveIt side need no changes.
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
