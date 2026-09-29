// litearm_system.cpp — LitearmSystem implementation (system hardware interface,
// shared-memory bridge).

#include "litearm_ros2_control/litearm_system.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace litearm_ros2_control {

namespace {

/** Convert a daemon suppression reason code into readable text (mirrors the
 * LITEARM_DAEMON_* codes in litearm_shm.h). */
const char *daemon_status_text(double code) {
  switch (static_cast<int>(code)) {
    case LITEARM_DAEMON_OK:
      return "tracking commands normally";
    case LITEARM_DAEMON_CONNECTING:
      return "hardware not connected (starting up, port not found, or license "
             "not activated)";
    case LITEARM_DAEMON_HOLDING_STALE_COMMAND:
      return "command frames are stale: the ROS control loop stopped publishing";
    case LITEARM_DAEMON_HOLDING_ESTOP:
      return "soft emergency stop in progress";
    case LITEARM_DAEMON_HOLDING_MOTOR_FAULT:
      return "a joint has a non-healthy error code";
    case LITEARM_DAEMON_HOLDING_FEEDBACK_STALE:
      return "joint feedback is missing or has timed out";
    case LITEARM_DAEMON_HOLDING_OVERTEMP:
      return "motor temperature reached the software protection threshold";
    case LITEARM_DAEMON_DISABLED:
      return "disabled on request (motors have no torque)";
    case LITEARM_DAEMON_HOLDING_WATCHDOG:
      return "the litearm-stm32 firmware watchdog took over";
    case LITEARM_DAEMON_SHUTTING_DOWN:
      return "the daemon is shutting down";
    case LITEARM_DAEMON_HOLDING_BAD_COMMAND:
      return "the command frame held non-finite values; rejected";
    default:
      return "unknown status code";
  }
}

std::string trim(const std::string &value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool strict_bool(const std::string &raw, bool fallback) {
  const std::string value = to_lower(trim(raw));
  if (value.empty()) {
    return fallback;
  }
  if (value == "true" || value == "1" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off") {
    return false;
  }
  return fallback;
}

/** Parse a URDF joint name into a shm array index: joint1..joint7 → 0..6. */
bool joint_name_to_index(const std::string &name, std::size_t *out) {
  constexpr const char *kPrefix = "joint";
  if (name.rfind(kPrefix, 0) != 0) {
    return false;
  }
  const std::string digits = name.substr(std::strlen(kPrefix));
  if (digits.empty() || digits.size() > 2) {
    return false;
  }
  for (const char c : digits) {
    if (std::isdigit(static_cast<unsigned char>(c)) == 0) {
      return false;
    }
  }
  const int number = std::stoi(digits);
  if (number < 1 || number > LITEARM_SHM_NUM_JOINTS) {
    return false;
  }
  *out = static_cast<std::size_t>(number - 1);
  return true;
}

/**
 * Parse a comma-separated list of 7 doubles (e.g. "400,400,300,300,50,50,50").
 *
 * A single scalar is also accepted (broadcast to all joints), handy for ad-hoc
 * debugging.
 */
bool parse_double_list(const std::string &raw,
                       std::array<double, LITEARM_SHM_NUM_JOINTS> *out) {
  std::vector<double> values;
  std::stringstream stream(raw);
  std::string token;
  while (std::getline(stream, token, ',')) {
    const std::string item = trim(token);
    if (item.empty()) {
      continue;
    }
    char *end = nullptr;
    const double value = std::strtod(item.c_str(), &end);
    if (end == item.c_str() || *end != '\0' || !std::isfinite(value)) {
      return false;
    }
    values.push_back(value);
  }
  if (values.size() == 1) {
    out->fill(values.front());
    return true;
  }
  if (values.size() != LITEARM_SHM_NUM_JOINTS) {
    return false;
  }
  std::copy(values.begin(), values.end(), out->begin());
  return true;
}

}  // namespace

double LitearmSystem::monotonic_seconds() {
  // Explicitly use CLOCK_MONOTONIC: the same time base as Python's
  // time.monotonic(), so timestamps from both sides compare directly
  // (heartbeat_s / stamp_s / command_age_s).
  struct timespec ts {};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

hardware_interface::CallbackReturn LitearmSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger(info_.name.empty() ? "LitearmSystem" : info_.name);

  if (info_.joints.size() != static_cast<std::size_t>(LITEARM_SHM_NUM_JOINTS)) {
    RCLCPP_FATAL(logger_,
                 "The URDF declares %zu joints, but the litearm shared-memory "
                 "contract is fixed at %d (joint1..joint7). Check the <joint> "
                 "list in <ros2_control>.",
                 info_.joints.size(), LITEARM_SHM_NUM_JOINTS);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Build the URDF order → shm index mapping. The state/command arrays in shared
  // memory are always laid out as joint1..joint7, while the order of <joint> in
  // the URDF is decided by the description file; without this mapping, a
  // different joint order would silently write J3's position into J5's command.
  joint_to_shm_.assign(info_.joints.size(), 0);
  std::vector<bool> seen(LITEARM_SHM_NUM_JOINTS, false);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    std::size_t index = 0;
    if (!joint_name_to_index(info_.joints[i].name, &index)) {
      RCLCPP_FATAL(logger_,
                   "Joint name '%s' cannot be mapped to a litearm axis index "
                   "(valid range is joint1..joint%d).",
                   info_.joints[i].name.c_str(), LITEARM_SHM_NUM_JOINTS);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (seen[index]) {
      RCLCPP_FATAL(logger_, "Joint %s is declared more than once in the URDF.",
                   info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    seen[index] = true;
    joint_to_shm_[i] = index;
  }

  // ── parameter parsing (URDF <hardware><param name="..">value</param>) ──
  const auto param = [this](const std::string &key, const std::string &fallback) {
    const auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? fallback : it->second;
  };

  shm_name_ = trim(param("shm_name", LITEARM_SHM_DEFAULT_NAME));
  if (shm_name_.empty() || shm_name_.front() != '/') {
    RCLCPP_FATAL(logger_,
                 "shm_name must be a POSIX shared memory name starting with '/'; "
                 "it is currently '%s'.",
                 shm_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto parse_double_param = [&](const std::string &key, double fallback) {
    const std::string raw = trim(param(key, ""));
    if (raw.empty()) {
      return fallback;
    }
    try {
      return std::stod(raw);
    } catch (const std::exception &) {
      RCLCPP_WARN(logger_,
                  "Parameter %s='%s' is not a valid number; using default %g.",
                  key.c_str(), raw.c_str(), fallback);
      return fallback;
    }
  };

  connect_timeout_s_ = std::max(0.1, parse_double_param("connect_timeout_s", 10.0));
  heartbeat_timeout_s_ =
      std::max(0.05, parse_double_param("heartbeat_timeout_s", 1.0));
  state_read_retries_ = std::max(
      0, static_cast<int>(parse_double_param("state_read_retries", 8.0)));
  configure_read_retries_ = std::max(
      1, static_cast<int>(parse_double_param("configure_read_retries", 512.0)));
  export_diagnostics_ =
      strict_bool(param("export_diagnostic_interfaces", "true"), true);
  disable_on_shutdown_ =
      strict_bool(param("disable_on_shutdown", "false"), false);

  // Default MIT gains: used as the initial values of the kp/kd command
  // interfaces when nothing claims them (which is what makes "position control
  // with only the JTC attached" work), and as the stiffness for latching
  // position at activation.
  default_kp_.fill(0.0);
  default_kd_.fill(0.0);
  const std::string kp_raw = param("default_kp", "400,400,300,300,50,50,50");
  if (!parse_double_list(kp_raw, &default_kp_)) {
    RCLCPP_FATAL(logger_,
                 "default_kp='%s' is invalid: expected 7 comma-separated "
                 "numbers (or a single scalar).",
                 kp_raw.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string kd_raw = param("default_kd", "5,5,4,5,2.5,2.5,2.5");
  if (!parse_double_list(kd_raw, &default_kd_)) {
    RCLCPP_FATAL(logger_,
                 "default_kd='%s' is invalid: expected 7 comma-separated "
                 "numbers (or a single scalar).",
                 kd_raw.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Hard limits of the DM (Damiao) MIT frame: kp∈[0,500], kd∈[0,5].
  // Out-of-range values get truncated by the motor driver; rather than letting
  // the upper layer believe it has high stiffness, clamp and warn at init time.
  for (std::size_t i = 0; i < LITEARM_SHM_NUM_JOINTS; ++i) {
    if (default_kp_[i] < 0.0 || default_kp_[i] > 500.0) {
      RCLCPP_WARN(logger_,
                  "default_kp[%zu]=%g is outside the MIT range [0,500]; clamped.",
                  i, default_kp_[i]);
    }
    if (default_kd_[i] < 0.0 || default_kd_[i] > 5.0) {
      RCLCPP_WARN(logger_,
                  "default_kd[%zu]=%g is outside the MIT range [0,5]; clamped.",
                  i, default_kd_[i]);
    }
    default_kp_[i] = std::clamp(default_kp_[i], 0.0, 500.0);
    default_kd_[i] = std::clamp(default_kd_[i], 0.0, 5.0);
  }
  command_kp_ = default_kp_;
  command_kd_ = default_kd_;

  // Command positions start at 0; on_activate overwrites them with the measured
  // positions, so there is no jump at activation.
  command_position_.fill(0.0);
  command_velocity_.fill(0.0);
  command_acceleration_.fill(0.0);
  command_effort_.fill(0.0);

  map_ready_ = true;
  RCLCPP_INFO(logger_,
              "litearm hardware interface initialized: shm=%s joints=%zu "
              "default kp/kd=%g/%g (tail) diagnostic interfaces=%s",
              shm_name_.c_str(), info_.joints.size(), default_kp_.back(),
              default_kd_.back(), export_diagnostics_ ? "on" : "off");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // The segment must already have been created by the daemon: create=false. If
  // it is missing, the daemon is not up, and failing right away with an
  // actionable message beats waiting until activation to fail.
  const int code = litearm_shm_open(shm_name_.c_str(), 0, &shm_);
  if (code != LITEARM_SHM_OK) {
    RCLCPP_FATAL(logger_,
                 "Failed to open shared memory '%s' (code=%d). Start the "
                 "hardware daemon first: ros2 run litearm_ros2_control "
                 "litearm_hw_daemon (the port is auto-discovered from the "
                 "VID:PID 1d50:606f by default); with no hardware attached, add "
                 "--dry-run to debug.",
                 shm_name_.c_str(), code);
    shm_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Wait for the daemon to reach the "connected" state. Two failure modes are
  // distinguished: the segment exists but there is no heartbeat (the daemon is
  // not running), and there is a heartbeat but the hardware cannot be reached
  // (CAN not up / bus busy / motor fault).
  const double deadline = monotonic_seconds() + connect_timeout_s_;
  bool saw_heartbeat = false;
  double last_status = LITEARM_DAEMON_CONNECTING;
  double last_heartbeat = 0.0;
  while (monotonic_seconds() < deadline) {
    LitearmState state{};
    if (litearm_shm_read_state(shm_, &state, configure_read_retries_) ==
        LITEARM_SHM_OK) {
      last_heartbeat = state.heartbeat_s;
      last_status = state.last_error;
      if (state.heartbeat_s > 0.0) {
        saw_heartbeat = true;
      }
      // ⚠ The condition must include **a fresh heartbeat + not shutting down**,
      //   not just the connected bit: when the daemon dies abnormally, shared
      //   memory keeps a stale state block that still reports connected=1 (the
      //   one it wrote just before dying). Looking only at connected would
      //   treat it as "daemon ready" and go on to activate — while read() is
      //   returning **frozen joint positions** that the controller keeps
      //   commanding against. Seen on real hardware: no serial port permission
      //   → the daemon exits immediately → the plugin reports "daemon ready
      //   (status: the daemon is shutting down)" and activates.
      const double heartbeat_age =
          state.heartbeat_s > 0.0
              ? monotonic_seconds() - state.heartbeat_s
              : std::numeric_limits<double>::infinity();
      const bool daemon_shutting_down =
          static_cast<int>(state.last_error) == LITEARM_DAEMON_SHUTTING_DOWN;
      if (state.connected != 0.0 && !daemon_shutting_down &&
          heartbeat_age < heartbeat_timeout_s_) {
        state_buffer_ = state;
        have_state_ = true;
        last_heartbeat_s_ = state.heartbeat_s;
        daemon_alive_ = true;
        mode_ = Mode::kConfigured;
        RCLCPP_INFO(logger_,
                    "Daemon ready (dry_run=%s, heartbeat %.0fms ago, "
                    "status: %s).",
                    state.dry_run != 0.0 ? "yes" : "no", heartbeat_age * 1e3,
                    daemon_status_text(state.last_error));
        return hardware_interface::CallbackReturn::SUCCESS;
      }
      if (state.connected != 0.0 && daemon_shutting_down) {
        // The segment holds a relic of "the daemon from last time": it exited
        // cleanly, so stop waiting. The specific reason has been reported
        // already, go straight to the cleanup below (do not bury it under a
        // generic error).
        RCLCPP_FATAL(logger_,
                     "Shared memory '%s' holds a **stale** state (the previous "
                     "daemon exited cleanly, last status: %s). Restart the "
                     "hardware daemon.",
                     shm_name_.c_str(), daemon_status_text(state.last_error));
        litearm_shm_close(shm_);
        shm_ = nullptr;
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  litearm_shm_close(shm_);
  shm_ = nullptr;
  if (saw_heartbeat) {
    RCLCPP_FATAL(logger_,
                 "The daemon is running but the hardware is not connected "
                 "(%.1fs timeout, last heartbeat %.2fs ago, reason: %s). Check "
                 "whether the board is powered and the USB is plugged in "
                 "(lsusb | grep 1d50:606f), the serial port permissions "
                 "(dialout group), whether another daemon already holds that "
                 "serial port, and whether the firmware license is activated.",
                 connect_timeout_s_,
                 last_heartbeat > 0.0 ? monotonic_seconds() - last_heartbeat : -1.0,
                 daemon_status_text(last_status));
  } else {
    RCLCPP_FATAL(logger_,
                 "Shared memory '%s' exists but the daemon has no heartbeat "
                 "(%.1fs timeout). Make sure litearm_hw_daemon is running and "
                 "uses the same shm_name as this plugin.",
                 shm_name_.c_str(), connect_timeout_s_);
  }
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn LitearmSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!map_ready_ || shm_ == nullptr) {
    RCLCPP_ERROR(logger_, "Not initialized/configured yet; cannot activate.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Align the command positions with the measured positions: if a controller
  // does not write the command interfaces in the cycle right after activation,
  // the MIT frame's q_ref is the current angle — the arm stays in place instead
  // of jumping toward the zero position.
  latch_command_to_measured();
  mode_ = Mode::kActive;
  publish_command();
  RCLCPP_INFO(logger_,
              "Activated: command positions aligned with the measured "
              "positions, kp/kd use the default values (unless overridden).");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (mode_ == Mode::kActive && shm_ != nullptr) {
    // Hand back control: send one "in place + current kp/kd" hold command. If
    // it is not sent, the command frame goes stale on its own and the daemon
    // enters hold anyway — sending one explicitly just makes the handover
    // smoother (the daemon's slew limit can anchor from the correct position).
    latch_command_to_measured();
    publish_command();
    RCLCPP_INFO(logger_,
                "Deactivated: published the final hold frame (arm keeps its "
                "position, motors stay enabled).");
  }
  mode_ = Mode::kStopped;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (shm_ != nullptr) {
    litearm_shm_close(shm_);
    shm_ = nullptr;
  }
  have_state_ = false;
  daemon_alive_ = false;
  mode_ = Mode::kUnconfigured;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (shm_ != nullptr) {
    if (mode_ == Mode::kActive) {
      latch_command_to_measured();
      publish_command();
    }
    if (disable_on_shutdown_) {
      // Not done by default: disabling would cut motor torque and let the arm
      // sag. The daemon's own shutdown path also parks / holds position.
      LitearmCommand command{};
      if (litearm_shm_read_command(shm_, &command, configure_read_retries_) ==
          LITEARM_SHM_OK) {
        command.enable = 0.0;
        command.stamp_s = monotonic_seconds();
        command.cycle_count = ++command_cycle_;
        litearm_shm_publish_command(shm_, &command);
        RCLCPP_WARN(logger_,
                    "disable_on_shutdown=true: asked the daemon to disable the "
                    "motors — the arm will lose torque, make sure it is "
                    "supported.");
      }
    }
    litearm_shm_close(shm_);
    shm_ = nullptr;
  }
  mode_ = Mode::kUnconfigured;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // ⚠ Do not unconditionally promise "it will hold position automatically"
  // here: if the daemon never started (most commonly no serial port permission
  // / board not plugged in), **no process is holding position** and the arm is
  // free. The old implementation said flatly "it will switch to a
  // high-stiffness hold automatically because the command frames are stale",
  // which was a false guarantee in that scenario.
  if (daemon_alive_) {
    RCLCPP_ERROR(logger_,
                 "Hardware entered an error state. The daemon will switch to a "
                 "high-stiffness hold automatically because the command frames "
                 "are stale, and the arm stays in place; once the problem is "
                 "resolved, configure/activate again to recover.");
  } else {
    RCLCPP_ERROR(logger_,
                 "Hardware entered an error state (**the daemon is not "
                 "ready**). ⚠ No process is holding position right now, the arm "
                 "is free — if it is staying up by gravity rather than by a "
                 "support, support it first. Once the problem is resolved, "
                 "configure/activate again to recover.");
  }
  mode_ = Mode::kStopped;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LitearmSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(info_.joints.size() * (export_diagnostics_ ? 7u : 3u));
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string &name = info_.joints[i].name;
    interfaces.emplace_back(name, kStatePosition, &state_position_[i]);
    interfaces.emplace_back(name, kStateVelocity, &state_velocity_[i]);
    interfaces.emplace_back(name, kStateEffort, &state_effort_[i]);
    if (!export_diagnostics_) {
      continue;
    }
    // Non-standard interfaces: joint_state_broadcaster will not claim them, and
    // controller_manager will warn at activation that "these interfaces are
    // unclaimed" — that is expected.
    interfaces.emplace_back(name, kStateTemperatureMos, &state_temperature_mos_[i]);
    interfaces.emplace_back(name, kStateTemperatureCoil, &state_temperature_coil_[i]);
    interfaces.emplace_back(name, kStateErrorCode, &state_error_code_[i]);
    interfaces.emplace_back(name, kStateFeedbackAge, &state_feedback_age_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
LitearmSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(info_.joints.size() * 6u);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string &name = info_.joints[i].name;
    interfaces.emplace_back(name, kCommandPosition, &command_position_[i]);
    interfaces.emplace_back(name, kCommandVelocity, &command_velocity_[i]);
    // acceleration is only there for the daemon's M(q)·q̈ feedforward; leaving
    // it unwritten makes it 0 (that feedforward term degenerates to C·q̇).
    interfaces.emplace_back(name, kCommandAcceleration, &command_acceleration_[i]);
    interfaces.emplace_back(name, kCommandEffort, &command_effort_[i]);
    // kp/kd are the "optional runtime tuning" entry point: when no controller
    // writes them, the values stay at default_kp/default_kd (see on_init), so
    // pure position control works out of the box.
    interfaces.emplace_back(name, kCommandKp, &command_kp_[i]);
    interfaces.emplace_back(name, kCommandKd, &command_kd_[i]);
  }
  return interfaces;
}

void LitearmSystem::apply_state(const LitearmState &state) {
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const std::size_t index = joint_to_shm_[i];
    state_position_[i] = state.position[index];
    state_velocity_[i] = state.velocity[index];
    state_effort_[i] = state.effort[index];
    state_temperature_mos_[i] = state.temperature_mos[index];
    state_temperature_coil_[i] = state.temperature_coil[index];
    state_error_code_[i] = state.error_code[index];
    state_feedback_age_[i] = state.feedback_age_s[index];
  }
}

void LitearmSystem::latch_command_to_measured() {
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const std::size_t index = joint_to_shm_[i];
    command_position_[i] = have_state_ ? state_buffer_.position[index] : 0.0;
    command_velocity_[i] = 0.0;
    command_acceleration_[i] = 0.0;
    command_effort_[i] = 0.0;
    // kp/kd keep their current values: they may be the defaults, or an
    // impedance tuned by the upper layer.
  }
}

void LitearmSystem::publish_command() {
  if (shm_ == nullptr) {
    return;
  }
  LitearmCommand command{};
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    const std::size_t index = joint_to_shm_[i];
    command.position[index] = command_position_[i];
    command.velocity[index] = command_velocity_[i];
    command.acceleration[index] = command_acceleration_[i];
    command.effort[index] = command_effort_[i];
    command.kp[index] = command_kp_[i];
    command.kd[index] = command_kd_[i];
  }
  const double now = monotonic_seconds();
  command.enable = 1.0;
  command.estop = 0.0;
  command.stamp_s = now;
  command.cycle_count = ++command_cycle_;
  if (litearm_shm_publish_command(shm_, &command) == LITEARM_SHM_OK) {
    last_command_stamp_s_ = now;
  }
}

hardware_interface::return_type LitearmSystem::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (mode_ != Mode::kActive || shm_ == nullptr) {
    return hardware_interface::return_type::OK;
  }

  LitearmState state{};
  const int code = litearm_shm_read_state(shm_, &state, state_read_retries_);
  if (code == LITEARM_SHM_TORN) {
    // Rare tearing: this cycle reuses the previous frame's values. Never let a
    // single torn read stop the controller.
    ++torn_state_reads_;
    return hardware_interface::return_type::OK;
  }
  if (code != LITEARM_SHM_OK) {
    RCLCPP_ERROR(logger_, "Failed to read shared memory state (code=%d).", code);
    return hardware_interface::return_type::ERROR;
  }

  state_buffer_ = state;
  have_state_ = true;
  last_heartbeat_s_ = state.heartbeat_s;
  last_applied_cycle_ = state.applied_command_cycle;
  apply_state(state);

  const double now = monotonic_seconds();

  // ── daemon liveness check ──
  // Once the daemon dies, the state block freezes at its last frame. Without
  // this check the controller would keep planning into thin air (while the arm
  // is in fact stopped in the hold pose from when the daemon exited).
  const double heartbeat_age = now - state.heartbeat_s;
  const bool alive = state.heartbeat_s > 0.0 && heartbeat_age <= heartbeat_timeout_s_;
  if (!alive) {
    if (!reported_daemon_loss_) {
      reported_daemon_loss_ = true;
      RCLCPP_ERROR(logger_,
                   "Lost the hardware daemon heartbeat (last heartbeat %.2fs "
                   "ago > threshold %.2fs). The hardware interface exits with an "
                   "error; the arm should be stopped in the hold pose from when "
                   "the daemon exited.",
                   heartbeat_age, heartbeat_timeout_s_);
    }
    daemon_alive_ = false;
    return hardware_interface::return_type::ERROR;
  }
  if (!daemon_alive_) {
    RCLCPP_INFO(logger_, "Hardware daemon heartbeat recovered.");
    daemon_alive_ = true;
    reported_daemon_loss_ = false;
  }

  // ── diagnostics for abnormal conditions (do not block control, but say what
  // is going on) ──
  // Throttling is computed here by hand rather than with RCLCPP_*_THROTTLE:
  // those macros hold a static rclcpp::Clock at the call site that is only
  // constructed/allocated on first use — and this function runs on the
  // real-time thread.
  if (state.faulted != 0.0) {
    if (now - last_fault_log_s_ >= 1.0) {
      last_fault_log_s_ = now;
      RCLCPP_ERROR(logger_, "Motors report a fault code: %s.",
                   daemon_status_text(state.last_error));
    }
  } else {
    last_fault_log_s_ = 0.0;
  }
  if (state.watchdog_tripped != 0.0 && !reported_watchdog_trip_) {
    reported_watchdog_trip_ = true;
    RCLCPP_WARN(logger_,
                "The litearm-stm32 firmware command watchdog has taken over "
                "after more than 100ms without a command (the firmware switched "
                "to a fail-soft hold). This plugin never blocks itself; if this "
                "keeps happening, check CPU preemption / other process load and "
                "whether the daemon is being starved.");
  }
  if (state.last_error == LITEARM_DAEMON_HOLDING_STALE_COMMAND) {
    // This means the daemon considers the commands stale — i.e. this plugin
    // has not published for longer than command_watchdog_timeout_s, which
    // usually means the update cycle was stretched badly or the controller was
    // paused.
    if (now - last_stale_log_s_ >= 2.0) {
      last_stale_log_s_ = now;
      RCLCPP_WARN(logger_,
                  "The daemon considers the command frames stale (age=%.3fs) "
                  "and has entered hold.",
                  state.command_age_s);
    }
  } else {
    last_stale_log_s_ = 0.0;
  }

  // Closed-loop command-path diagnostic: if the daemon does not apply the
  // command counter for a long time, it has been holding all along.
  if (last_applied_cycle_ >= 0.0 && command_cycle_ > 0.0 &&
      state.last_error != LITEARM_DAEMON_OK &&
      state.last_error != LITEARM_DAEMON_HOLDING_STALE_COMMAND) {
    if (!reported_daemon_command_mismatch_) {
      reported_daemon_command_mismatch_ = true;
      RCLCPP_WARN(logger_,
                  "The daemon is not following commands, reason: %s. The arm "
                  "is currently holding position.",
                  daemon_status_text(state.last_error));
    }
  } else if (state.last_error == LITEARM_DAEMON_OK) {
    reported_daemon_command_mismatch_ = false;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LitearmSystem::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (mode_ != Mode::kActive || shm_ == nullptr) {
    return hardware_interface::return_type::OK;
  }
  publish_command();
  return hardware_interface::return_type::OK;
}

}  // namespace litearm_ros2_control

PLUGINLIB_EXPORT_CLASS(litearm_ros2_control::LitearmSystem,
                       hardware_interface::SystemInterface)
