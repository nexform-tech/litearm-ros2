# litearm driver quickstart

The commands that bring `litearm_driver` up on a real arm, for anyone doing bring-up,
licensing or parameter tuning without the ros2_control stack. Each section is a step you run
in order; section 5 is what to read when one of them fails.

The ros2_control stack is not in this repository. If you want controllers, MoveIt and RViz
instead of the SDK's administrative calls, use
[litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control). The two entry
points are mutually exclusive — both open the same USB CDC port and the SDK takes an
exclusive `flock` on it — so run one at a time.

**Do not run the driver while the ros2_control stack is running.** The symptom is a driver
whose configure fails: neither process can share the port.

## 0. Before anything

```bash
source /opt/ros/humble/setup.bash
source ~/litearm_ws/install/setup.bash

lsusb | grep 1d50          # expect 1d50:606f
ls /dev/ttyACM*            # expect /dev/ttyACM0
```

## 1. Start the driver

```bash
ros2 launch litearm_driver litearm_driver.launch.py
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `namespace` | `litearm` | Namespace of the services; they live directly in it. |
| `autostart` | `true` | Configure and activate the node. `false` starts it inactive for manual lifecycle driving. |
| `config` | packaged `config/litearm_driver.yaml` | Parameter file. |
| `log_level` | `info` | Log level of the node. |

The driver does not pin a ROS domain, so a normal shell reaches it directly.

### Read-only bring-up

`auto_enable` lives in the parameter file and defaults to `true`. To look at an arm without
energising it:

```bash
cp install/litearm_driver/share/litearm_driver/config/litearm_driver.yaml /tmp/driver_no_enable.yaml
sed -i 's/auto_enable: true/auto_enable: false/' /tmp/driver_no_enable.yaml
ros2 launch litearm_driver litearm_driver.launch.py config:=/tmp/driver_no_enable.yaml
```

## 2. Calls that do not move the arm

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{}"
ros2 service call /litearm/get_diagnostics litearm_msgs/srv/GetDiagnostics "{}"
ros2 service call /litearm/get_tcp litearm_msgs/srv/GetTcp "{}"
ros2 service call /litearm/get_feedforward_mask litearm_msgs/srv/GetFeedforwardMask "{}"
ros2 service call /litearm/get_feedforward_catalog litearm_msgs/srv/GetFeedforwardCatalog "{}"
ros2 service call /litearm/inverse_kinematics litearm_msgs/srv/InverseKinematics \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], seed: [], timeout: 0.5}"
ros2 service call /litearm/log_start litearm_msgs/srv/LogStart "{ticks: 600}"
ros2 service call /litearm/log_dump litearm_msgs/srv/LogDump \
  "{filename: tick_log.bin, wait: true, timeout: 3.0}"
```

`get_feedforward_catalog` answers from the SDK's static tables and needs no session at all;
`log_dump` writes inside the node's `log_dir` (`$HOME/.ros/litearm` by default) and refuses a
path as `filename`.

## 3. Moving the arm

Every service that can energise the motors is refused until `allow_motion` is set. It is read
on every call, like `allow_dfu`, so no restart is needed:

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
ros2 service call /litearm/move_l litearm_msgs/srv/MoveL \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], speed: 0.2, wait: true}"
```

`speed` is a fraction of full speed, `0 < speed <= 1`; `30` means 30×, not 30 percent. The
percentage governor is `/litearm/set_speed_scaling`, and the firmware's own `speed_scaling`
field in the status message is a host-side record of the last write.

**Support the arm before enabling anything that moves it, and keep the emergency stop within
reach.** `move_js`, `send_mit` and `send_mit_all` are single frames: the firmware's 100 ms
watchdog takes the arm back to its hold unless a caller keeps re-sending.

## 4. Handing the arm back

```bash
# In the terminal running the driver: Ctrl-C, and wait for the process to exit.
fuser -v /dev/ttyACM0 2>/dev/null      # who still holds the port, if anything
```

The two deployments cannot overlap because of the port, not because of a ROS resource: the
driver does not take the shared-memory lock of any other deployment.

## 5. When it does not come up

| Symptom | Cause and what to do |
| --- | --- |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | The board is unplugged, off, or in DFU. Check `lsusb`; a board in DFU enumerates as `0483:DF11` and needs a firmware image. |
| Configure fails while the ros2_control stack ran a moment ago | The port is still held. Stop the other process and retry; nothing else can share it. |
| `ENABLE was refused: ... ERR{0x10,0x06}` | The firmware has a latched fault (EMERGENCY or `joint_fault`). The driver reports the licence fields in its status message if that is the cause; clear it with `/litearm/reset`. |
| A motion service returns a refusal about `allow_motion` | The gate is closed. Set it with `ros2 param set /litearm/driver allow_motion true`. |
| `enter_dfu` is refused | `allow_dfu` defaults to `false`. Set it with `ros2 param set /litearm/driver allow_dfu true`. |

## 6. Where to go next

- [`docs/command-set.md`](command-set.md) — every driver service, its refusals and the wire
  semantics behind them (Chinese: [`docs/command-set.zh-CN.md`](command-set.zh-CN.md)).
- [`litearm_driver/README.md`](../litearm_driver/README.md) — the driver's parameters, topics
  and field-level service reference.
- [`litearm_msgs/README.md`](../litearm_msgs/README.md) — the message and service definitions
  a client compiles against.
