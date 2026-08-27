"""Shared fixtures: a fake litearm.Arm used to exercise the bridge without hardware."""
from __future__ import annotations

import pytest

_IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


class FakeArm:
    def __init__(self):
        self.q = [0.0] * 7
        self.dq = [0.0] * 7
        self.tau = [0.0] * 7
        self.state = "idle"
        self.fault = False
        self.stopped = False
        self.enabled = True
        self.calls = []

    def get_state(self):
        return {"q": self.q, "dq": self.dq, "tau": self.tau,
                "state": self.state, "fault": self.fault}

    def get_tcp_pose(self):
        # litearm pose format: [position(3), rotation(3x3 row-major)]
        return ([0.3, 0.0, 0.4], _IDENTITY)

    def movej(self, q, speed=0.5, settle_s=0.5):
        self.calls.append(("movej", list(q)))
        self.q = [float(v) for v in q]

    def movel(self, pose, speed=0.5, settle_s=0.5):
        self.calls.append(("movel", pose))

    def fk(self, q):
        self.calls.append(("fk", list(q)))
        return ([0.3, 0.0, 0.4], _IDENTITY)

    def ik(self, pos_d, R_d, q_seed=None):
        self.calls.append(("ik", pos_d, R_d, q_seed))
        return [0.1] * 7, True

    def request_stop(self):
        self.stopped = True

    def clear_stop(self):
        self.stopped = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def close(self):
        self.calls.append(("close",))


@pytest.fixture
def fake_arm():
    return FakeArm()
