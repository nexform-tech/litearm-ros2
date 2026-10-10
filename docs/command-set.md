# litearm command set

This is the reference for every ROS 2 interface `litearm_driver` exposes and what each one
does to the arm; read it when you operate an arm without the ros2_control stack, or when
you write a client against the driver. The Chinese reference is
[`docs/command-set.zh-CN.md`](command-set.zh-CN.md); the English version here is canonical.
The field-level interface reference is [`litearm_driver/README.md`](../litearm_driver/README.md).

Verification status: the service names, types, request handling, refusals and the status
message are covered by the offline test suite (`colcon test --packages-select litearm_driver`).
The `ros2` command lines below are the interface contract as tested; running them against
a real arm was not part of this repository's test run.

## 1. One process per arm

The driver opens the arm's USB CDC port and the SDK takes an exclusive `flock` on it. Two
processes therefore cannot drive one arm:

- **Do not run `litearm_driver` while the ros2_control stack is running.** The second
  process fails to open the port, and the driver reports it at configure time.
- The same rule applies in reverse: stop the driver before you start
  `ros2_control_node`.

The driver is the maintenance path: licensing, fault handling, parameter tuning and
bring-up checks, with no `controller_manager` in the loop. Motion control belongs to
`joint_trajectory_controller` (via MoveIt 2) or to a custom controller such as the servo
controller in `litearm_servo_control`.

## 2. Start and stop

```bash
ros2 launch litearm_driver litearm_driver.launch.py
```

The launch file starts the node as `/litearm/driver` in the `litearm` namespace, so every
service below is `/litearm/<name>`. It configures and activates the node unless you pass
`autostart:=false`.

```bash
ros2 launch litearm_driver litearm_driver.launch.py autostart:=false
ros2 lifecycle set /litearm/driver configure
ros2 lifecycle set /litearm/driver activate
ros2 lifecycle get /litearm/driver
```

Deactivating (`ros2 lifecycle set /litearm/driver deactivate`) removes every service,
stops zero gravity if it was active, and leaves the motors as they are: `park()` and the
firmware hold keep the arm where it is. Disabling the motors on deactivate would let the
arm fall.

Run the executable directly only when you set the namespace yourself; without a namespace
the relative service names resolve to `/enable`, `/status` and so on.

### 2.1 Parameters

Every parameter has a default; the node reads them at configure time, except
`allow_dfu`, `allow_motion` and `zero_g_keepalive_period_s`, which are read
again on each call so `ros2 param set` takes effect immediately.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `port` | `""` | USB CDC device path. Empty means auto-discovery by VID:PID `1d50:606f`. |
| `joint_names` | unset | Axis names in firmware axis order. Unset derives `joint1`..`jointN` from the axes the firmware reports; a list of the wrong length fails configure. |
| `auto_enable` | `true` | Enable the motors on activation. A refused ENABLE fails the activation instead of leaving an "active" node with dead motors. |
| `enable_attempts` | `12` | `enable()` retries; only the firmware's retryable code consumes them. |
| `status_rate_hz` | `10.0` | `/litearm/status` publication rate. |
| `publish_joint_states` | `false` | Publish `joint_states_topic`. Keep it false whenever a `joint_state_broadcaster` runs. |
| `joint_states_topic` | `/joint_states` | Topic for the optional joint states. |
| `publish_diagnostics` | `true` | Publish `/diagnostics`. |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` publication rate. |
| `zero_g_keepalive_period_s` | `0.04` | Zero-gravity keep-alive period, `[0.005, 0.10)`. |
| `allow_dfu` | `false` | Gate for `enter_dfu`. |
| `allow_motion` | `false` | Gate for every motion service: `move_j`, `move_j_sync`, `move_p`, `move_js`, `send_mit`, `send_mit_all`, `home`, `move_l`, `move_c`, `move_path`. |
| `log_dir` | `""` | Directory `log_dump` writes into. Empty means `$HOME/.ros/litearm`. |
| `frame_id` | `""` | Header frame id of the status and joint state messages. |

## 3. Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/litearm/status` | `litearm_msgs/msg/LitearmStatus` | Cached and local SDK state only; the publisher never sends a frame, so subscribing cannot disturb a command. `speed_scaling` and the `zero_g_*` fields are host-side views, not read-back values. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | One status: link, enable, fault, drop-hold inference, licence, feedback age. Published by the node itself rather than through `diagnostic_updater`, to keep the dependency set to what the ros2_control stack already needs. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Only when `publish_joint_states:=true`. |

`LitearmStatus` carries the per-axis temperatures, error codes and `joint_fault` bitmap
indexed in `joint_names` order. An axis the current frame does not cover reads `NaN` for
temperature and `0` for the error code; `feedback_age` is `-1` before the first frame.

## 4. Services

All service names are relative to the node's namespace, so with the launch file they are
`/litearm/<name>`. `success=false` always comes with a `message` that names the reason,
including, where a gate refused the call, why it refused.

### 4.1 State and safety

| Service | Type | What it does |
| --- | --- | --- |
| `enable` | `std_srvs/srv/Trigger` | `Arm::enable(enable_attempts)`. |
| `disable` | `std_srvs/srv/Trigger` | `Arm::disable()`. The arm stops being held; support it first. |
| `reset` | `std_srvs/srv/Trigger` | `Arm::reset()`: clear the fault and re-anchor the pose. Not an MCU reset. |
| `clear_faults` | `std_srvs/srv/Trigger` | `Arm::clear_faults()`: clear the RAM fault bits only. |
| `emergency_stop` | `std_srvs/srv/Trigger` | `Arm::emergency_stop()`. A software request; the hardware emergency stop remains the authority. |
| `park` | `std_srvs/srv/Trigger` | `Arm::park()`: declare a static hold at full stiffness. |
| `zero_g` | `std_srvs/srv/SetBool` | `data=true` enters zero gravity and starts the keep-alive; `data=false` leaves it. The arm is then free to move by hand, so run it with the workspace clear. |

```bash
ros2 service call /litearm/clear_faults std_srvs/srv/Trigger "{}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
```

### 4.2 Status

| Service | Type | What it does |
| --- | --- | --- |
| `get_status` | `litearm_msgs/srv/GetStatus` | Sends `GET_STATUS` and returns one `LitearmStatus`. `timeout <= 0` returns the cached frame immediately. Use it when the passive stream is silent and you need proof of life with a deadline. |

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
ros2 service call /litearm/get_tcp litearm_msgs/srv/GetTcp "{}"
```

The licence record is read once while the node configures and is reported in the status
message's licence fields; there is no service for it, and no way to activate a licence
through this node.

### 4.3 Motion configuration

| Service | Type | What it does |
| --- | --- | --- |
| `set_speed_scaling` | `litearm_msgs/srv/SetSpeedScaling` | Writes the firmware's global speed governor, an integer percent 0..100. It is global and persists until changed, unlike the per-move `speed` argument of the SDK's `movej`. |
| `set_motion_mode` | `litearm_msgs/srv/SetMotionMode` | Writes `SET_MOTION_MODE`. The current firmware recognises mode 0 only, which is `park()`; the SDK rejects any other value rather than ACKing a mode change that does not happen. |
| `set_payload` | `litearm_msgs/srv/SetPayload` | Declares the end-effector mass (0..20 kg) and centre of mass (metres, flange frame). |

```bash
ros2 service call /litearm/set_speed_scaling litearm_msgs/srv/SetSpeedScaling "{percent: 40}"
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload \
  "{mass: 0.8, com: [0.0, 0.0, 0.05]}"
```

### 4.4 Dynamics and feedforward

The firmware clamps feedforward values silently, so a write reports success for the
clamped value, not the requested one. Read back what you wrote.

| Service | Type | What it does |
| --- | --- | --- |
| `set_feedforward_mask` | `litearm_msgs/srv/SetFeedforwardMask` | Writes the feedforward enable mask. Bits outside `FF_ALL` are refused instead of being folded into "all off", which would remove gravity compensation. |
| `set_feedforward_preset` | `litearm_msgs/srv/SetFeedforwardPreset` | 0 all off, 1 factory, 2 all on. |
| `set_feedforward_scalar` | `litearm_msgs/srv/SetFeedforwardScalar` | Writes one scalar (`item`, `sub`, `value`). `sub` is meaningful for items 5 and 6 only. Item numbering is `litearm::Arm::ff_scalar_items()`. |
| `set_feedforward_vector` | `litearm_msgs/srv/SetFeedforwardVector` | Writes one vector (`item`, `values`): 7 values for items 1..5, 7 and 8; 3 for the gravity vector (item 6). The SDK rejects a wrong length; non-finite values are refused here. |
| `get_feedforward_scalar` | `litearm_msgs/srv/GetFeedforwardScalar` | Reads one scalar back. Item 9 is the read-only extension that returns `ff_mask`. |

```bash
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload "{mass: 0.8, com: [0, 0, 0.05]}"
ros2 service call /litearm/get_feedforward_scalar litearm_msgs/srv/GetFeedforwardScalar \
  "{item: 4, sub: 0}"
```

The feedforward mask and item numbering come from the firmware's own parameter table; the
SDK's `ff_scalar_items()` and `ff_vec_items()` are the authoritative lists.

### 4.5 Joint parameters

| Service | Type | What it does |
| --- | --- | --- |
| `get_joint_params` | `litearm_msgs/srv/GetJointParams` | Reads one axis (`joint >= 0`) or every axis (`joint: -1`). |
| `set_joint_gains` | `litearm_msgs/srv/SetJointGains` | Writes `kp`, `kd`, `tau_max` for one axis. These are the gains of the firmware's own position loop. |
| `set_joint_limits` | `litearm_msgs/srv/SetJointLimits` | Writes the soft limits of one axis. The host caches the limits at connect time, so the SDK's local pre-check picks them up on the next connect. |
| `reset_factory_params` | `std_srvs/srv/Trigger` | `JointParams::reset_factory()`. Requires disabled motors. |
| `save_params` | `std_srvs/srv/Trigger` | Writes the runtime parameters to flash. Requires disabled motors. Flash writes are not reversible. |

```bash
ros2 service call /litearm/get_joint_params litearm_msgs/srv/GetJointParams "{joint: -1}"
ros2 service call /litearm/set_joint_gains litearm_msgs/srv/SetJointGains \
  "{joint: 0, kp: 12.0, kd: 1.5, tau_max: 9.0}"
ros2 service call /litearm/save_params std_srvs/srv/Trigger "{}"
```

### 4.6 Maintenance

| Service | Type | What it does |
| --- | --- | --- |
| `enter_dfu` | `std_srvs/srv/Trigger` | Marks the device for the ROM bootloader. Requires `allow_dfu:=true`, disabled motors and a valid ROM vector table. After an ACK the CDC device disappears and this node is unusable until it is reconfigured with a new link. |

```bash
ros2 param set /litearm/driver allow_dfu true
ros2 service call /litearm/enter_dfu std_srvs/srv/Trigger "{}"
```

**Do not** call `enter_dfu` to "try it out": the device re-enumerates as `0483:DF11` and
needs a firmware image to come back.

### 4.7 Motion (`allow_motion`)

| Service | Type | What it does |
| --- | --- | --- |
| `move_j` | `litearm_msgs/srv/MoveJ` | Joint move on the firmware's per-axis S-curve. Blocks until the firmware reports the move finished. `speed` is a fraction, `0 < speed <= 1`; `30` means 30x, not 30 percent. |
| `move_j_sync` | `litearm_msgs/srv/MoveJSync` | The same target, every axis on one synchronised curve. |
| `move_p` | `litearm_msgs/srv/MoveP` | Pose move: the firmware solves the IK and walks its own curve. `wait=false` leaves the plan in flight for `poll_cart`. |
| `move_js` | `litearm_msgs/srv/MoveJs` | One MOVE_JS frame. `dq` is the slew limit of the position reference; an all-zero `dq` freezes the firmware's own reference, and supplying `tau_ff` switches the built-in feedforward off for that frame. |
| `send_mit`, `send_mit_all` | `litearm_msgs/srv/SendMit`, `SendMitAll` | Raw MIT passthrough: the gains go to the motors as given, with no built-in feedforward. |
| `home` | `std_srvs/srv/Trigger` | Walk to the URDF zero pose at the firmware's own low speed. |
| `move_l`, `move_c`, `move_path` | `litearm_msgs/srv/MoveL`, `MoveC`, `MovePath` | The firmware's own cartesian planner: a line, an arc through a via pose, or a multi-waypoint path. The reply carries the plan report (`settled`, `err`, `waypoints`, `plan_us`, `settle_err_rad`, `q_final`). |

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
```

⚠ Every service in this group is refused unless `allow_motion:=true`, and the parameter is
read on every call. `move_js`, `send_mit` and `send_mit_all` are **single frames**: the
firmware's 100 ms watchdog takes the arm back to its hold unless a caller keeps re-sending,
which is what the ros2_control component does and what this driver does not.

### 4.8 Computing

| Service | Type | What it does |
| --- | --- | --- |
| `inverse_kinematics` | `litearm_msgs/srv/InverseKinematics` | Solves a pose into joint angles. It does not move the arm and is **not** behind `allow_motion`. |
| `poll_cart` | `litearm_msgs/srv/PollCart` | The outcome of a `wait=false` cartesian request: `pending` until the firmware answers, then the plan report. |

### 4.9 Dynamics model store

| Service | Type | What it does |
| --- | --- | --- |
| `probe_model` | `litearm_msgs/srv/ProbeModel` | Whether this firmware answers the model store commands at all. |
| `get_model_body`, `set_model_body`, `get_model_jm`, `set_model_jm` | `litearm_msgs/srv/GetModelBody`, `SetModelBody`, `GetModelJm`, `SetModelJm` | Read a body or the joint-space terms, or stage a write in RAM. |
| `commit_model` | `litearm_msgs/srv/CommitModel` | Writes the staged model to flash. Requires disabled motors, and `expected_mask` must match what the firmware has staged. |
| `revert_model` | `std_srvs/srv/Trigger` | Drops the staged model. |
| `get_model_status` | `litearm_msgs/srv/GetModelStatus` | Override level, staged mask and the dirty flag — the mask `commit_model` needs. |

### 4.10 Diagnostics and the link

| Service | Type | What it does |
| --- | --- | --- |
| `reconnect` | `std_srvs/srv/Trigger` | Rebuilds the session after an unplugged cable and re-reads the licence record. |
| `get_diagnostics` | `litearm_msgs/srv/GetDiagnostics` | Host counters (`dropped`, `bad_status_frames`, `flush_failures`, the cartesian pairing counters), the per-id message rates, `cart_supported` and the identity fields. |
| `kin_bench` | `litearm_msgs/srv/KinBench` | The firmware's kinematics benchmark: its own text, the timings it printed, and the counters it names. A counter of 0 means "not reported", not "no errors". |

### 4.11 Control tick log

| Service | Type | What it does |
| --- | --- | --- |
| `log_start` | `litearm_msgs/srv/LogStart` | Records that many 300 Hz control ticks (`tick`, `q_ref`, `dq`, `tau` per axis); the firmware stops on its own when its buffer is full. |
| `log_stop` | `std_srvs/srv/Trigger` | Stops the recording. |
| `log_dump` | `litearm_msgs/srv/LogDump` | Reads the recording back and writes the raw blob into the node's `log_dir`. `filename` is a plain name; a path is refused. |

## 5. Refusals and what they mean

Every refusal names the parameter, the value or the operation it refused. The gates run
before any frame is built, so a refused call puts nothing on the wire (covered by the
test `RefusedArgumentsSendNoFrame`).

| Refusal | Trigger | Why the driver refuses instead of forwarding |
| --- | --- | --- |
| `percent must be 0..100` | `set_speed_scaling` outside the range | The firmware takes an integer percent: `1` is one percent, not full speed. |
| `preset must be 0 ... 2` | `set_feedforward_preset` outside the range | Only three presets exist. |
| `period must be in [0.005, 0.10)` | `zero_g` with a bad `zero_g_keepalive_period_s` | The firmware command watchdog trips at 0.10 s, so a slower keep-alive would silently drop out of zero gravity. |
| `mass must be in [0, 20] kg` | `set_payload` outside the range | The firmware clamps silently; the stored value would differ from the requested one. |
| `requires the motors to be disabled` | `save_params`, `reset_factory_params`, `activate_license` while enabled | Flash writes and activation are only safe with no torque authority. |
| `set the parameter ... to true` | `enter_dfu`, `activate_license` while the opt-in parameter is false | Both are irreversible or trust-relevant; the default is off. |
| `not connected` | any service with no live link | Names the usual causes: USB cable, 24 V supply, or the ros2_control stack holding the port. |
| `axis index must be 0..N-1` | a joint service with an index outside the reported axes | A typo would otherwise address the wrong axis. |

The pre-checks are best effort: `save_params` reads the cached state frame to decide
whether the motors are enabled. The SDK and the firmware enforce the same rule on the
wire, so a stale cache can only turn a clean refusal into the firmware's own error
message, never into an unsafe write.

## 6. Deliberately not exposed

| Capability | Where it lives instead | Why |
| --- | --- | --- |
| Cartesian servo and teleoperation | MoveIt Servo plus a custom controller (`litearm_servo_control` in the working workspace) | Servo needs a controller in the command path, not a service that hands over one pose at a time, and it must not share the arm with a one-shot planner. |
| Licence activation | The vendor's signing tool, outside ROS | The UID a credential is signed for is readable from the status message; submitting the credential is a provisioning step, not a maintenance call. |
| Joint state publication while the control stack runs | `joint_state_broadcaster` | Two publishers on one `joint_states` topic make the arm's state ambiguous; the driver keeps its own publisher off by default. |

Trajectory execution, the firmware's cartesian planner, the model store and the control
tick log **are** exposed (section 4.7 to 4.11). The motion group is behind
`allow_motion:=true` because one of them can move the arm, and the driver is the only
process holding the port while it runs.

## 7. Errors, timeouts and the wire

- One mutually exclusive callback group serialises every SDK call. A command waits for the
  firmware ACK, so a stalled USB link can hold a service call for up to the SDK's own
  timeout (1.2 s); the status publication pauses for that long, which is the price of
  never interleaving two commands on the wire.
- A service that fails returns `success=false` and the SDK's own message, for example a
  clamped register, the firmware's "reset first" refusal for a latched fault, or a refusal
  of `ENABLE` when the licence is not activated.
- The arm is not a real-time system from the host's point of view: this node is for
  maintenance and diagnostics, not for control loops.

## 8. Compatibility

| Item | Requirement |
| --- | --- |
| ROS 2 | Humble |
| Firmware | `Litearm1.5.0` or newer for the status layout and `joint_fault`; `Litearm1.8.0` or newer for the licence record and activation |
| SDK | `litearm-cpp`, found as an installed package or built from the sibling source tree |
| Hardware | USB CDC `VID:PID 1d50:606f`, 24 V supply for motion |
