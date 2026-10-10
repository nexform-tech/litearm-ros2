# litearm_driver

A standalone ROS 2 lifecycle node that owns the LiteArm arm's USB link and exposes the
SDK's administrative command set over services, for maintaining, licensing or tuning an
arm without a controller_manager in the loop.

English · [简体中文](README.zh-CN.md)

The node talks to the firmware through the [litearm-cpp](https://github.com/nexform-tech/litearm-cpp)
SDK. It creates its services on activation and destroys them on deactivation, and it
serialises every SDK call through one callback group.

**This node is mutually exclusive with the ros2_control stack.** Both open the same serial
port and the SDK takes an exclusive `flock` on it, so the second process fails to open the
port. Stop one before starting the other.

## Run

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

The launch file puts the node in the `litearm` namespace, so its relative service names
become `/litearm/enable`, `/litearm/clear_faults` and so on. Every service name below is
written relative to that namespace.

For the whole bring-up — the launch arguments, the read-only start and the calls that do not
move the arm — see [`docs/quickstart.md`](../docs/quickstart.md). The ros2_control stack is a
separate repository, [litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control).

## Command set

The node exposes 55 services in ten groups, three topics and 14 parameters. Every service
returns `success` plus a `message`; on failure the message carries the SDK's own words, and
when a gate refused the call before the frame was built, the reason for the refusal.

Three rules shape the whole surface:

- **Gates run before the frame.** A refused argument puts nothing on the wire. The node
  refuses what the firmware would silently turn into a different request: a payload mass
  outside `[0, 20]` kg, a speed percent outside `0..100`, a zero-gravity keep-alive period
  past the 100 ms command watchdog, a feedforward value that is not finite.
- **One command at a time.** Every SDK call runs in one mutually exclusive callback group,
  so two commands can never interleave on the wire. A command waits for the firmware ACK,
  so a stalled link can hold a service call for up to the SDK's own 1.2 s timeout.
- **Reads are cached.** Only the on-demand services (`get_status`, `get_tcp`, the
  `get_feedforward_*` reads, `get_joint_params`, the model reads, `kin_bench`, the log
  services and `poll_cart`) talk to the firmware on demand; `get_feedforward_catalog`
  answers from the SDK's static tables and needs no link at all. `/litearm/status` never
  does, so subscribing cannot disturb a command in flight.

### Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `port` | `""` | USB CDC device path. Empty means auto-discovery by VID:PID `1d50:606f`. |
| `joint_names` | unset | Axis names in firmware axis order. Unset derives `joint1`..`jointN` from the axes the firmware reports; a list of the wrong length fails configure. |
| `auto_enable` | `true` | Enable the motors on activation. A refused ENABLE fails the activation instead of leaving an "active" node with dead motors. |
| `enable_attempts` | `12` | Retries for the `enable` service. Only the code the firmware marks retryable consumes them. |
| `status_rate_hz` | `10.0` | `/litearm/status` publication rate. |
| `publish_joint_states` | `false` | Publish `joint_states_topic`. Keep it false whenever a `joint_state_broadcaster` runs. |
| `joint_states_topic` | `/joint_states` | Topic for the optional joint states. |
| `publish_diagnostics` | `true` | Publish `/diagnostics`. |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` publication rate. |
| `zero_g_keepalive_period_s` | `0.04` | Zero-gravity keep-alive period, `[0.005, 0.10)`. Read on every `zero_g` call. |
| `allow_dfu` | `false` | Gate for `enter_dfu`. Read on every call. |
| `allow_motion` | `false` | Gate for every motion service (`move_j`, `move_p`, `move_js`, `send_mit*`, `home`, `move_l`, `move_c`, `move_path`). Read on every call. |
| `log_dir` | `""` | Directory `log_dump` writes into. Empty means `$HOME/.ros/litearm`. |
| `frame_id` | `""` | Header frame id of the status and joint state messages. |

### Topics

| Topic | Type | Contents |
| --- | --- | --- |
| `/litearm/status` | `litearm_msgs/msg/LitearmStatus` | Cached and local SDK state at `status_rate_hz`. Never sends a frame. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | One status: link, enable, fault, drop-hold inference, licence, feedback age. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Only when `publish_joint_states:=true`. Off by default so it cannot compete with `joint_state_broadcaster`. |

`LitearmStatus` carries, in `joint_names` order: `temperature_mos`, `temperature_coil`,
`joint_error_code` and the `joint_fault` bitmap. An axis the current frame does not cover
reads `NaN` for temperature and `0` for the error code; `feedback_age` is `-1` until the
first frame arrives. `speed_scaling` and the `zero_g_*` fields are host-side views, not
read-back values.

### Service index

| Service | Type | Purpose |
| --- | --- | --- |
| `enable` | `std_srvs/srv/Trigger` | Enable every axis. |
| `disable` | `std_srvs/srv/Trigger` | Disable every axis; the arm stops being held. |
| `reset` | `std_srvs/srv/Trigger` | Clear the fault and re-anchor the pose. |
| `clear_faults` | `std_srvs/srv/Trigger` | Clear the RAM fault bits only. |
| `emergency_stop` | `std_srvs/srv/Trigger` | Request a stop. |
| `park` | `std_srvs/srv/Trigger` | Declare a static hold at full stiffness. |
| `zero_g` | `std_srvs/srv/SetBool` | Enter or leave zero gravity (hand-guided motion). |
| `get_status` | `litearm_msgs/srv/GetStatus` | Send `GET_STATUS` and return one snapshot. |
| `reconnect` | `litearm_msgs/srv/Trigger` | Re-open the link after an unplugged cable. |
| `home` | `litearm_msgs/srv/Trigger` | Walk to the URDF zero pose at the firmware's low speed. |
| `get_tcp` | `litearm_msgs/srv/GetTcp` | Read the firmware's current tool pose. |
| `get_diagnostics` | `litearm_msgs/srv/GetDiagnostics` | Host counters, per-id message rates, link and capability flags. |
| `kin_bench` | `litearm_msgs/srv/KinBench` | Run the firmware's kinematics benchmark and parse the reply. |
| `move_j` | `litearm_msgs/srv/MoveJ` | Joint move on the firmware's per-axis S-curve. |
| `move_j_sync` | `litearm_msgs/srv/MoveJSync` | Joint move with every axis on one synchronised curve. |
| `move_p` | `litearm_msgs/srv/MoveP` | Pose move: the firmware solves the IK and walks its own curve. |
| `move_js` | `litearm_msgs/srv/MoveJs` | Send one MOVE_JS frame, the streaming primitive. |
| `send_mit` | `litearm_msgs/srv/SendMit` | One axis of raw MIT passthrough. |
| `send_mit_all` | `litearm_msgs/srv/SendMitAll` | Whole-arm MIT passthrough, one frame. |
| `move_l` | `litearm_msgs/srv/MoveL` | Straight-line cartesian move, planned by the firmware. |
| `move_c` | `litearm_msgs/srv/MoveC` | Circular cartesian move through a via pose. |
| `move_path` | `litearm_msgs/srv/MovePath` | Multi-waypoint cartesian path. |
| `poll_cart` | `litearm_msgs/srv/PollCart` | Outcome of an in-flight cartesian request. |
| `inverse_kinematics` | `litearm_msgs/srv/InverseKinematics` | Solve a pose into joint angles; does not move. |
| `get_feedforward_vector` | `litearm_msgs/srv/GetFeedforwardVector` | Read one feedforward vector back. |
| `get_feedforward_mask` | `litearm_msgs/srv/GetFeedforwardMask` | Read the feedforward enable mask back. |
| `get_feedforward_catalog` | `litearm_msgs/srv/GetFeedforwardCatalog` | The SDK's item tables, answered without a session. |
| `set_gravity_scale` | `litearm_msgs/srv/SetGravityScale` | Scale the gravity feedforward per axis. |
| `set_inertia_scale` | `litearm_msgs/srv/SetInertiaScale` | Scale the inertia feedforward per axis. |
| `set_gravity_vector` | `litearm_msgs/srv/SetGravityVector` | Write the gravity vector the model uses. |
| `probe_model` | `litearm_msgs/srv/ProbeModel` | Ask whether the firmware has a dynamics model store. |
| `get_model_body` | `litearm_msgs/srv/GetModelBody` | Read one body of the dynamics model. |
| `set_model_body` | `litearm_msgs/srv/SetModelBody` | Stage one body of the dynamics model in RAM. |
| `get_model_jm` | `litearm_msgs/srv/GetModelJm` | Read the joint-space model terms. |
| `set_model_jm` | `litearm_msgs/srv/SetModelJm` | Stage the joint-space model terms in RAM. |
| `commit_model` | `litearm_msgs/srv/CommitModel` | Write the staged model to flash. |
| `revert_model` | `litearm_msgs/srv/Trigger` | Drop the staged model. |
| `get_model_status` | `litearm_msgs/srv/GetModelStatus` | Override level, staged mask and the dirty flag. |
| `log_start` | `litearm_msgs/srv/LogStart` | Start recording the firmware's 300 Hz control ticks. |
| `log_stop` | `litearm_msgs/srv/Trigger` | Stop recording. |
| `log_dump` | `litearm_msgs/srv/LogDump` | Read the recording back and write it to a file. |
| `set_speed_scaling` | `litearm_msgs/srv/SetSpeedScaling` | Write the global speed governor, in percent. |
| `set_motion_mode` | `litearm_msgs/srv/SetMotionMode` | Write the firmware motion mode. |
| `set_payload` | `litearm_msgs/srv/SetPayload` | Declare the end-effector mass and centre of mass. |
| `set_feedforward_mask` | `litearm_msgs/srv/SetFeedforwardMask` | Write the feedforward enable mask. |
| `set_feedforward_preset` | `litearm_msgs/srv/SetFeedforwardPreset` | Select a feedforward preset. |
| `set_feedforward_scalar` | `litearm_msgs/srv/SetFeedforwardScalar` | Write one feedforward scalar. |
| `set_feedforward_vector` | `litearm_msgs/srv/SetFeedforwardVector` | Write one feedforward vector. |
| `get_feedforward_scalar` | `litearm_msgs/srv/GetFeedforwardScalar` | Read one scalar back. |
| `get_joint_params` | `litearm_msgs/srv/GetJointParams` | Read the joint parameter table. |
| `set_joint_gains` | `litearm_msgs/srv/SetJointGains` | Write `kp`/`kd`/`tau_max` of one axis. |
| `set_joint_limits` | `litearm_msgs/srv/SetJointLimits` | Write the soft limits of one axis. |
| `reset_factory_params` | `std_srvs/srv/Trigger` | Restore the factory joint parameters. |
| `save_params` | `std_srvs/srv/Trigger` | Write the runtime parameters to flash. |
| `enter_dfu` | `std_srvs/srv/Trigger` | Mark the device for the ROM bootloader. |

### State and safety

#### `enable`

`Arm::enable(enable_attempts)`. Only the error code the firmware marks retryable is
retried; a latched fault or a missing licence fails immediately.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. The licence must be activated, otherwise the firmware refuses ENABLE with `ERR{0x10,0x08}`. |

```bash
ros2 service call /litearm/enable std_srvs/srv/Trigger "{}"
```

#### `disable`

`Arm::disable()`. Bypasses the zero-gravity guard, because losing energy is safe.

**Do not** call `disable` on an arm that is not supported: the position loop stops holding
and a load will fall.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/disable std_srvs/srv/Trigger "{}"
```

#### `reset`

`Arm::reset()`: clear the fault and re-anchor the pose. Not an MCU reset.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/reset std_srvs/srv/Trigger "{}"
```

#### `clear_faults`

`Arm::clear_faults()`: clear the RAM fault bits only. Use `reset` when the pose reference
also has to be re-anchored.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/clear_faults std_srvs/srv/Trigger "{}"
```

#### `emergency_stop`

`Arm::emergency_stop()`. A software request: the hardware emergency stop remains the
authority, and the firmware's own watchdog holds the arm if the link dies.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/emergency_stop std_srvs/srv/Trigger "{}"
```

#### `park`

`Arm::park()`: declare a static hold at full stiffness, which does not depend on the 100 ms
command watchdog.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up. |

```bash
ros2 service call /litearm/park std_srvs/srv/Trigger "{}"
```

#### `zero_g`

`data=true` calls `Arm::zero_g_start(zero_g_keepalive_period_s)`; `data=false` calls
`Arm::zero_g_stop(true)`, so a keep-alive thread that died is reported instead of swallowed.
The SDK owns the keep-alive thread.

**Do not** send any other motion command while zero gravity is active: the firmware rejects
it, and the arm is under human guidance at that moment. Leave zero gravity first.

| Field | Type | Meaning |
| --- | --- | --- |
| `data` (request) | `bool` | `true` enters zero gravity, `false` leaves it. |
| `success` (response) | `bool` | `false` when the gate or the SDK refused. |
| `message` (response) | `string` | On leave, carries the keep-alive failure if one happened. |

```bash
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: false}"
```

### Status

#### `get_status`

Sends `GET_STATUS` (`Arm::get_status_now(timeout)`) and returns one `LitearmStatus`. Use it
when the passive stream is silent and you need proof of life with a deadline.

| Field | Type | Meaning |
| --- | --- | --- |
| `timeout` (request) | `float64` | Seconds to wait for the reply; `<= 0` returns the cached frame immediately. |
| `status` (response) | `litearm_msgs/LitearmStatus` | The snapshot, same fields as the topic. |
| `success` (response) | `bool` | `false` when the arm is not connected or the firmware did not answer. |

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

### Motion configuration

#### `set_speed_scaling`

`Arm::set_speed(percent)`: the firmware's global speed governor. It is global and persists
until changed, unlike the per-move `speed` argument of the SDK's `movej`.

| Field | Type | Meaning |
| --- | --- | --- |
| `percent` (request) | `uint8` | 0..100. It is an integer percentage, so 1 is one percent of full speed, not full speed. |
| Gate | | Out-of-range values are refused before any frame is built. |

```bash
ros2 service call /litearm/set_speed_scaling litearm_msgs/srv/SetSpeedScaling "{percent: 40}"
```

#### `set_motion_mode`

`Arm::set_motion_mode(mode)`. This firmware recognises mode 0 only, which is the same thing
as `park()`; the SDK rejects any other value rather than ACKing a mode change that does not
happen. The response message says so.

| Field | Type | Meaning |
| --- | --- | --- |
| `mode` (request) | `int32` | Mode number. 0 is the only accepted value today. |

```bash
ros2 service call /litearm/set_motion_mode litearm_msgs/srv/SetMotionMode "{mode: 0}"
```

#### `set_payload`

`Arm::set_payload(mass, com)`: declare the load the firmware compensates for. The firmware
clamps mass into `[0, 20]` kg silently, so the node refuses a value outside that range
rather than reporting success for a different number.

| Field | Type | Meaning |
| --- | --- | --- |
| `mass` (request) | `float64` | Kilograms, `[0, 20]`. |
| `com` (request) | `float64[3]` | Centre of mass in metres, flange frame, all components finite. |
| Gate | | Range and finiteness are checked here; the SDK and the firmware enforce the wire rule too. |

Read it back with `get_feedforward_scalar` (`item: 4, sub: 0`), which returns the clamped
truth.

```bash
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload \
  "{mass: 0.8, com: [0.0, 0.0, 0.05]}"
```

### Dynamics and feedforward

The firmware clamps feedforward values silently, so a successful write means the clamped
value was stored, not the requested one. **Read back what you wrote.**

#### `set_feedforward_mask`

`Arm::set_ff_mask(mask)`: the feedforward enable mask. Bits outside `FF_ALL` are rejected
by the SDK rather than folded into "all off", which would silently remove gravity
compensation.

| Field | Type | Meaning |
| --- | --- | --- |
| `mask` (request) | `uint32` | Only bits inside `FF_ALL` are accepted. |

```bash
ros2 service call /litearm/set_feedforward_mask litearm_msgs/srv/SetFeedforwardMask "{mask: 511}"
```

#### `set_feedforward_preset`

`Arm::ff_preset(preset)`.

| Field | Type | Meaning |
| --- | --- | --- |
| `preset` (request) | `int32` | 0 all off, 1 factory, 2 all on. |
| Gate | | Any other value is refused before any frame is built. |

```bash
ros2 service call /litearm/set_feedforward_preset litearm_msgs/srv/SetFeedforwardPreset "{preset: 1}"
```

#### `set_feedforward_scalar`

`Arm::set_ff_scalar(item, sub, value)`. Item numbering is
`litearm::Arm::ff_scalar_items()`; `sub` is meaningful for items 5 and 6 only.

| Field | Type | Meaning |
| --- | --- | --- |
| `item` (request) | `int32` | Scalar item number. |
| `sub` (request) | `int32` | Sub-index; meaningful for items 5 and 6 (0..2). |
| `value` (request) | `float64` | The value to store; the firmware clamps it. |

```bash
ros2 service call /litearm/set_feedforward_scalar litearm_msgs/srv/SetFeedforwardScalar \
  "{item: 4, sub: 0, value: 0.8}"
```

#### `set_feedforward_vector`

`Arm::set_ff_vec(item, values)`. Item numbering is `litearm::Arm::ff_vec_items()`: 7 values
for items 1..5, 7 and 8; 3 for the gravity vector (item 6). The SDK rejects a wrong length;
the node refuses an empty list or a value that is not finite.

| Field | Type | Meaning |
| --- | --- | --- |
| `item` (request) | `int32` | Vector item number. |
| `values` (request) | `float64[]` | Non-empty, all finite; the length has to match the item. |

```bash
ros2 service call /litearm/set_feedforward_vector litearm_msgs/srv/SetFeedforwardVector \
  "{item: 7, values: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

#### `get_feedforward_scalar`

`Arm::get_ff_scalar(item, sub)`: the read-back path. Item 9 is the read-only extension that
returns `ff_mask`.

| Field | Type | Meaning |
| --- | --- | --- |
| `item` (request) | `int32` | Scalar item number; 9 reads `ff_mask`. |
| `sub` (request) | `int32` | Sub-index; meaningful for items 5 and 6. |
| `value` (response) | `float64` | The value the firmware actually holds. |

```bash
ros2 service call /litearm/get_feedforward_scalar litearm_msgs/srv/GetFeedforwardScalar \
  "{item: 9, sub: 0}"
```

### Joint parameters

#### `get_joint_params`

`JointParams::get_joint_param(joint)` or `all_joint_params()`. One axis or the whole table.

| Field | Type | Meaning |
| --- | --- | --- |
| `joint` (request) | `int32` | Axis index, or `-1` for every axis. |
| `params` (response) | `litearm_msgs/JointParam[]` | One entry per requested axis. |
| Gate | | An index outside the reported axes is refused. |

Each `JointParam` carries `index`, `kp`, `kd`, `tau_max`, `q_min`, `q_max`.

```bash
ros2 service call /litearm/get_joint_params litearm_msgs/srv/GetJointParams "{joint: -1}"
```

#### `set_joint_gains`

`JointParams::set_joint_param(joint, kp, kd, tau_max)`. These are the gains of the
firmware's own position loop; the ros2_control component sends none of its own.

| Field | Type | Meaning |
| --- | --- | --- |
| `joint` (request) | `int32` | Axis index. |
| `kp`, `kd`, `tau_max` (request) | `float64` | Position gain, damping gain and torque ceiling for that axis. |

```bash
ros2 service call /litearm/set_joint_gains litearm_msgs/srv/SetJointGains \
  "{joint: 0, kp: 12.0, kd: 1.5, tau_max: 9.0}"
```

#### `set_joint_limits`

`JointParams::set_joint_limits(joint, q_min, q_max)`. The host caches the limits at connect
time, so the SDK's local pre-check picks the new values up on the next connect, not
immediately.

| Field | Type | Meaning |
| --- | --- | --- |
| `joint` (request) | `int32` | Axis index. |
| `q_min`, `q_max` (request) | `float64` | Soft limits in radians. |

```bash
ros2 service call /litearm/set_joint_limits litearm_msgs/srv/SetJointLimits \
  "{joint: 0, q_min: -1.0, q_max: 1.0}"
```

#### `reset_factory_params`

`JointParams::reset_factory()`. Requires disabled motors.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up and motors disabled. |

```bash
ros2 service call /litearm/reset_factory_params std_srvs/srv/Trigger "{}"
```

#### `save_params`

`Arm::save_params()`: write the runtime parameters to flash. Requires disabled motors.
A flash write is not reversible.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | Link up and motors disabled. |

```bash
ros2 service call /litearm/save_params std_srvs/srv/Trigger "{}"
```

### Motion (`allow_motion`)

Every service here can energise the motors, so all of them are refused unless `allow_motion:=true` is set (`ros2 param
set` works without a restart; the parameter is read on every call, like `allow_dfu`). The driver owns the serial port
exclusively, so nothing else is streaming commands while one of these runs.

⚠ `move_js`, `send_mit` and `send_mit_all` are **single frames**. The firmware's 100 ms command watchdog takes the arm
back to its hold unless a caller keeps re-sending, which is what `litearm_ros2_control` does and what this driver does
not. `speed` is a fraction of full speed (`0 < speed <= 1`); `set_speed_scaling` is the percentage governor.

| Service | Type | What it does |
| --- | --- | --- |
| `home` | `litearm_msgs/srv/Trigger` | Walk to the URDF zero pose at the firmware's low speed. |
| `move_j` | `litearm_msgs/srv/MoveJ` | Joint move on the firmware's per-axis S-curve. |
| `move_j_sync` | `litearm_msgs/srv/MoveJSync` | Joint move with every axis on one synchronised curve. |
| `move_p` | `litearm_msgs/srv/MoveP` | Pose move: the firmware solves the IK and walks its own curve. |
| `move_js` | `litearm_msgs/srv/MoveJs` | Send one MOVE_JS frame, the streaming primitive. |
| `send_mit` | `litearm_msgs/srv/SendMit` | One axis of raw MIT passthrough. |
| `send_mit_all` | `litearm_msgs/srv/SendMitAll` | Whole-arm MIT passthrough, one frame. |
| `move_l` | `litearm_msgs/srv/MoveL` | Straight-line cartesian move, planned by the firmware. |
| `move_c` | `litearm_msgs/srv/MoveC` | Circular cartesian move through a via pose. |
| `move_path` | `litearm_msgs/srv/MovePath` | Multi-waypoint cartesian path. |

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
ros2 service call /litearm/move_l litearm_msgs/srv/MoveL "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], speed: 0.2, wait: true}"
```

### Computing

`inverse_kinematics` only computes, so it is **not** behind `allow_motion`. `poll_cart` reports the outcome of a
`wait=false` cartesian request: `pending` stays true until the firmware answers, and `err` is the firmware's own
cartesian error code.

| Service | Type | What it does |
| --- | --- | --- |
| `poll_cart` | `litearm_msgs/srv/PollCart` | Outcome of an in-flight cartesian request. |
| `inverse_kinematics` | `litearm_msgs/srv/InverseKinematics` | Solve a pose into joint angles; does not move. |

```bash
ros2 service call /litearm/inverse_kinematics litearm_msgs/srv/InverseKinematics "{pose: [0.3, 0.0, 0.4, 3.14, 0.0,
0.0], seed: [], timeout: 0.5}"
```

### Feedforward reads and gravity variables

The firmware clamps silently, so the reads are how a write is confirmed. `get_feedforward_catalog` answers from the
SDK's static tables and works with no session at all; its three lists are `item: name`, and `scalar_read_only_items` are
the ones `set_feedforward_scalar` cannot write. `set_gravity_scale` and `set_inertia_scale` are the SDK's convenience
wrappers over feedforward vector items 7 and 8 (seven values each); `set_gravity_vector` writes the three scalar
components of item 6.

| Service | Type | What it does |
| --- | --- | --- |
| `get_feedforward_vector` | `litearm_msgs/srv/GetFeedforwardVector` | Read one feedforward vector back. |
| `get_feedforward_mask` | `litearm_msgs/srv/GetFeedforwardMask` | Read the feedforward enable mask back. |
| `get_feedforward_catalog` | `litearm_msgs/srv/GetFeedforwardCatalog` | The SDK's item tables, answered without a session. |
| `set_gravity_scale` | `litearm_msgs/srv/SetGravityScale` | Scale the gravity feedforward per axis. |
| `set_inertia_scale` | `litearm_msgs/srv/SetInertiaScale` | Scale the inertia feedforward per axis. |
| `set_gravity_vector` | `litearm_msgs/srv/SetGravityVector` | Write the gravity vector the model uses. |

```bash
ros2 service call /litearm/get_feedforward_catalog litearm_msgs/srv/GetFeedforwardCatalog "{}"
ros2 service call /litearm/get_feedforward_mask litearm_msgs/srv/GetFeedforwardMask "{}"
```

### Dynamics model store

The firmware can import its dynamics model over the wire (`probe_model` says whether this build answers those commands).
Writes are **staged in RAM**: `commit_model` stores them and needs the motors disabled, `revert_model` drops them.
`expected_mask` must match what the firmware has staged or the commit is refused — read it from `get_model_status`.

| Service | Type | What it does |
| --- | --- | --- |
| `probe_model` | `litearm_msgs/srv/ProbeModel` | Ask whether the firmware has a dynamics model store. |
| `get_model_body` | `litearm_msgs/srv/GetModelBody` | Read one body of the dynamics model. |
| `set_model_body` | `litearm_msgs/srv/SetModelBody` | Stage one body of the dynamics model in RAM. |
| `get_model_jm` | `litearm_msgs/srv/GetModelJm` | Read the joint-space model terms. |
| `set_model_jm` | `litearm_msgs/srv/SetModelJm` | Stage the joint-space model terms in RAM. |
| `commit_model` | `litearm_msgs/srv/CommitModel` | Write the staged model to flash. |
| `revert_model` | `litearm_msgs/srv/Trigger` | Drop the staged model. |
| `get_model_status` | `litearm_msgs/srv/GetModelStatus` | Override level, staged mask and the dirty flag. |

```bash
ros2 service call /litearm/probe_model litearm_msgs/srv/ProbeModel "{}"
ros2 service call /litearm/get_model_status litearm_msgs/srv/GetModelStatus "{}"
```

### Diagnostics and the link

`get_diagnostics` is what separates "the link is quiet" from "the two ends disagree about the frame format": `dropped`
counts frames nobody wanted, `bad_status_frames` counts CRC-valid frames this decoder could not read. `kin_bench`
returns the firmware's own text plus the counters it names; 0 there means "not reported", not "no errors". `reconnect`
rebuilds the session after an unplugged cable, and re-reads the licence record because a different device could answer.

| Service | Type | What it does |
| --- | --- | --- |
| `reconnect` | `litearm_msgs/srv/Trigger` | Re-open the link after an unplugged cable. |
| `get_tcp` | `litearm_msgs/srv/GetTcp` | Read the firmware's current tool pose. |
| `get_diagnostics` | `litearm_msgs/srv/GetDiagnostics` | Host counters, per-id message rates, link and capability flags. |
| `kin_bench` | `litearm_msgs/srv/KinBench` | Run the firmware's kinematics benchmark and parse the reply. |

```bash
ros2 service call /litearm/get_diagnostics litearm_msgs/srv/GetDiagnostics "{}"
ros2 service call /litearm/kin_bench litearm_msgs/srv/KinBench "{timeout: 8.0}"
```

### Control tick log

The firmware records its own 300 Hz control ticks (`tick`, `q_ref`, `dq`, `tau` per axis) — the log is how a
control-loop problem is looked at after the fact. `log_dump` writes the raw blob into the node's `log_dir`; the
`filename` is a plain name, never a path, because where files land is not the caller's choice.

| Service | Type | What it does |
| --- | --- | --- |
| `log_start` | `litearm_msgs/srv/LogStart` | Start recording the firmware's 300 Hz control ticks. |
| `log_stop` | `litearm_msgs/srv/Trigger` | Stop recording. |
| `log_dump` | `litearm_msgs/srv/LogDump` | Read the recording back and write it to a file. |

```bash
ros2 service call /litearm/log_start litearm_msgs/srv/LogStart "{ticks: 600}"
ros2 service call /litearm/log_dump litearm_msgs/srv/LogDump "{filename: tick_log.bin, wait: true, timeout: 3.0}"
```

### Maintenance

#### `enter_dfu`

`Arm::enter_dfu()`: mark the device for the ROM bootloader. After the ACK the CDC device
disappears, this node is unusable until it is reconfigured with a new link, and the port
re-enumerates as `0483:DF11` until a firmware image is flashed.

**Do not** call `enter_dfu` to try it out.

| Item | Value |
| --- | --- |
| Type | `std_srvs/srv/Trigger` |
| Request | none |
| Response | `success`, `message` |
| Preconditions | `allow_dfu:=true`, motors disabled, valid ROM vector table. |

```bash
ros2 param set /litearm/driver allow_dfu true
ros2 service call /litearm/enter_dfu std_srvs/srv/Trigger "{}"
```

### Refusals

Each gate refuses before any frame is built, so a refused call puts nothing on the wire.
The test `RefusedArgumentsSendNoFrame` covers that.

| Refusal message | Trigger | Why the node refuses instead of forwarding |
| --- | --- | --- |
| `percent must be 0..100` | `set_speed_scaling` outside the range | The firmware takes an integer percent: `1` is one percent, not full speed. |
| `preset must be 0 ... 2` | `set_feedforward_preset` outside the range | Only three presets exist. |
| `period must be in [0.005, 0.10)` | `zero_g` with a bad keep-alive period | The firmware command watchdog trips at 0.10 s, so a slower keep-alive would silently drop out of zero gravity. |
| `mass must be in [0, 20] kg` | `set_payload` outside the range | The firmware clamps silently; the stored value would differ from the requested one. |
| `requires the motors to be disabled` | `save_params`, `reset_factory_params`, `commit_model` while enabled | Flash writes are only safe with no torque authority. |
| `set the parameter ... to true` | `enter_dfu` (`allow_dfu`), every motion service (`allow_motion`) while its opt-in parameter is false | One is irreversible, the other energises the arm; both default to off. |
| `not connected` | any service with no live link | Names the usual causes: USB cable, 24 V supply, or the ros2_control stack holding the port. |
| `axis index must be 0..N-1` | a joint service with an index outside the reported axes | A typo would otherwise address the wrong axis. |

The pre-checks are best effort: `save_params` reads the cached state frame to decide whether
the motors are enabled. The SDK and the firmware enforce the same rule on the wire, so a
stale cache can only turn a clean refusal into the firmware's own error message, never into
an unsafe write.

### Not exposed

Cartesian servo and teleoperation have no service here: servo needs a controller in the
command path rather than a service that hands over one pose at a time, and MoveIt Servo plus
`litearm_servo_control` are where that lives. Trajectory execution, firmware cartesian
planning, the model store and the tick log are exposed above, behind `allow_motion` where
they can move the arm. The ownership rules, the error semantics and the compatibility table
are in [`docs/command-set.md`](../docs/command-set.md).

## Tests

The suite is offline. It injects the SDK's `FakeTransport` and drives the whole lifecycle
and every service handler without an arm attached:

```bash
colcon test --packages-select litearm_driver --event-handlers console_direct+
colcon test-result --verbose
```

## License

Apache-2.0. See [LICENSE](../LICENSE).
