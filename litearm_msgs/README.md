# litearm_msgs — the message and service definitions of the LiteArm maintenance driver

Interfaces used by `litearm_driver`, the standalone ROS 2 node that owns the arm's USB
link while the ros2_control stack is not running. Read this when you write a client
against the driver, or add an interface the driver must expose.

## Messages

| Message | Purpose |
| --- | --- |
| `LitearmStatus` | One snapshot of the arm: link, enable, fault, mode, per-axis diagnostics, licence. Published on `/litearm/status` and returned by the `get_status` service. |
| `JointParam` | One axis of the firmware's joint parameter table: `kp`, `kd`, `tau_max`, `q_min`, `q_max`. |

## Services

Forty-one services carry values; the rest of the driver's surface uses `std_srvs`. There is no licence service: the
driver reads the licence record once while configuring and reports it in the status message.

| Service | Purpose |
| --- | --- |
| `GetStatus` | On-demand status snapshot with a timeout, instead of waiting for the topic. |
| `GetJointParams` | Read one axis or the whole joint parameter table. |
| `SetJointGains` | Write `kp`/`kd`/`tau_max` for one axis. |
| `SetJointLimits` | Write the soft limits of one axis. |
| `SetMotionMode` | Write the firmware motion mode (mode 0 only in the current firmware). |
| `SetSpeedScaling` | Write the global speed governor, in percent. |
| `SetPayload` | Declare the end-effector mass and centre of mass. |
| `SetFeedforwardMask` | Write the feedforward enable mask. |
| `SetFeedforwardPreset` | Select a feedforward preset (off, factory, all on). |
| `SetFeedforwardScalar` | Write one feedforward scalar. |
| `SetFeedforwardVector` | Write one feedforward vector. |
| `GetFeedforwardScalar` | Read one feedforward scalar or the feedforward mask back. |
| `Trigger` | Re-open the link after an unplugged cable. |
| `Trigger` | Walk to the URDF zero pose at the firmware's low speed. |
| `GetTcp` | Read the firmware's current tool pose. |
| `GetDiagnostics` | Host counters, per-id message rates, link and capability flags. |
| `KinBench` | Run the firmware's kinematics benchmark and parse the reply. |
| `MoveJ` | Joint move on the firmware's per-axis S-curve. |
| `MoveJSync` | Joint move with every axis on one synchronised curve. |
| `MoveP` | Pose move: the firmware solves the IK and walks its own curve. |
| `MoveJs` | Send one MOVE_JS frame, the streaming primitive. |
| `SendMit` | One axis of raw MIT passthrough. |
| `SendMitAll` | Whole-arm MIT passthrough, one frame. |
| `MoveL` | Straight-line cartesian move, planned by the firmware. |
| `MoveC` | Circular cartesian move through a via pose. |
| `MovePath` | Multi-waypoint cartesian path. |
| `PollCart` | Outcome of an in-flight cartesian request. |
| `InverseKinematics` | Solve a pose into joint angles; does not move. |
| `GetFeedforwardVector` | Read one feedforward vector back. |
| `GetFeedforwardMask` | Read the feedforward enable mask back. |
| `GetFeedforwardCatalog` | The SDK's item tables, answered without a session. |
| `SetGravityScale` | Scale the gravity feedforward per axis. |
| `SetInertiaScale` | Scale the inertia feedforward per axis. |
| `SetGravityVector` | Write the gravity vector the model uses. |
| `ProbeModel` | Ask whether the firmware has a dynamics model store. |
| `GetModelBody` | Read one body of the dynamics model. |
| `SetModelBody` | Stage one body of the dynamics model in RAM. |
| `GetModelJm` | Read the joint-space model terms. |
| `SetModelJm` | Stage the joint-space model terms in RAM. |
| `CommitModel` | Write the staged model to flash. |
| `Trigger` | Drop the staged model. |
| `GetModelStatus` | Override level, staged mask and the dirty flag. |
| `LogStart` | Start recording the firmware's 300 Hz control ticks. |
| `Trigger` | Stop recording. |
| `LogDump` | Read the recording back and write it to a file. |

The full command set, including the `std_srvs` services and the safety rules behind them,
is in [`docs/command-set.md`](../docs/command-set.md).

## License

Apache-2.0. See [LICENSE](../LICENSE).
