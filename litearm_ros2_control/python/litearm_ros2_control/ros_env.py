#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ros_env.py — shared ROS discovery-domain isolation for launch files.

**Deliberately does not import launch**: this module is used by launch files, but the
``litearm_ros2_control`` package ``__init__`` also serves the daemon / shared memory
bindings — and that side must not depend on launch. So only plain data and pure
functions live here; LaunchDescription construction stays in the individual launch
files.

Why domain isolation is mandatory
---------------------------------
ROS 2 defaults to ``ROS_DOMAIN_ID=0`` and multicast discovery is **subnet-wide**.
If any other machine on the same subnet is running ROS 2 (we did hit a twoarm/rail
rig during this project), the two discover each other. The symptoms:

* ``/robot_description`` gets overwritten by the other machine's model — RViz tries to
  load ``package://litearm_description/meshes/...``, this machine has no such package,
  so the robot simply disappears from the display, with
  ``Package [litearm_description] does not exist`` in the log;
* ``/joint_states`` gets mixed with another robot's joints;
* ``/move_action`` shows **multiple action servers**: MoveIt's planning goal is taken
  over by a stranger node's move_group (which has no local controllers and immediately
  returns FAILURE) while the local move_group is executing it at the same time — which
  shows up as "it reports failure, but the arm really does move", a phenomenon that is
  extremely hard to pin down.

There is only one fix: pin the domain and stay localhost-only. Turn it off explicitly
when you need to talk to an external system.
"""

DEFAULT_DOMAIN_ID = "42"
"""Default ROS domain. Deliberately avoids 0 — domain 0 is the one other devices on the
same subnet are most likely to occupy."""


def parse_bool(value: str, fallback: bool = False) -> bool:
    """Leniently parse a boolean string handed over by a launch file."""
    if value is None:
        return fallback
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return fallback


def ros_isolation_env(domain_id: str, localhost_only: str) -> "dict[str, str]":
    """Additional env for each node: pin the ROS domain, default to localhost only."""
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    return {
        "ROS_DOMAIN_ID": domain,
        "ROS_LOCALHOST_ONLY": "1" if parse_bool(localhost_only, True) else "0",
    }


def isolation_hint(domain_id: str, localhost_only: str) -> str:
    """Hint shown to the user: set the same variables in the terminal, otherwise the
    ros2 CLI tools will not see this stack's topics."""
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    if parse_bool(localhost_only, True):
        return (f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY=1")
    return (f"export ROS_DOMAIN_ID={domain}    "
            f"(localhost isolation is off — watch out for cross-machine crosstalk)")


__all__ = ["DEFAULT_DOMAIN_ID", "isolation_hint", "parse_bool",
           "ros_isolation_env"]
