// litearm_driver_node.cpp — implementation of the LiteArm maintenance driver.

#include "litearm_driver/litearm_driver_node.hpp"
#include "litearm_driver/service_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rmw/qos_profiles.h>
#include <rclcpp/logging.hpp>

#include "litearm/clock.hpp"
#include "litearm/errors.hpp"
#include "litearm/state.hpp"

namespace litearm_driver
{

LitearmDriverNode::LitearmDriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("litearm_driver", options)
{
  // Declared here so `ros2 param set` works before configure; read again in on_configure
  // because a launch file sets them between construction and configuration.
  declare_parameter<std::string>("port", "");
  declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
  declare_parameter<bool>("auto_enable", true);
  declare_parameter<int>("enable_attempts", 12);
  declare_parameter<double>("status_rate_hz", 10.0);
  declare_parameter<bool>("publish_joint_states", false);
  declare_parameter<std::string>("joint_states_topic", "/joint_states");
  declare_parameter<bool>("publish_diagnostics", true);
  declare_parameter<double>("diagnostics_rate_hz", 1.0);
  declare_parameter<double>("zero_g_keepalive_period_s", litearm::ZG_KEEPALIVE_S);
  declare_parameter<bool>("allow_dfu", false);
  declare_parameter<bool>("allow_motion", false);
  declare_parameter<std::string>("log_dir", "");
  declare_parameter<std::string>("frame_id", "");

  sdk_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

LitearmDriverNode::~LitearmDriverNode()
{
  // The Arm destructor closes the link; nothing else is required here.
}

std::unique_ptr<litearm::Arm> LitearmDriverNode::create_arm(const litearm::ArmOptions & options)
{
  return std::make_unique<litearm::Arm>(options);
}

// ────────────────────────────────────────────────────────────────────────────────
// parameters
// ────────────────────────────────────────────────────────────────────────────────

void LitearmDriverNode::read_parameters()
{
  port_ = get_parameter("port").as_string();
  joint_names_ = get_parameter("joint_names").as_string_array();
  auto_enable_ = get_parameter("auto_enable").as_bool();
  enable_attempts_ = static_cast<int>(get_parameter("enable_attempts").as_int());
  status_rate_hz_ = get_parameter("status_rate_hz").as_double();
  publish_joint_states_ = get_parameter("publish_joint_states").as_bool();
  joint_states_topic_ = get_parameter("joint_states_topic").as_string();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
  diagnostics_rate_hz_ = get_parameter("diagnostics_rate_hz").as_double();
  frame_id_ = get_parameter("frame_id").as_string();
  log_dir_ = get_parameter("log_dir").as_string();
  if (log_dir_.empty()) {
    const char * home = std::getenv("HOME");
    log_dir_ = std::string(home != nullptr ? home : ".") + "/.ros/litearm";
  }
}

// ────────────────────────────────────────────────────────────────────────────────
// lifecycle
// ────────────────────────────────────────────────────────────────────────────────

LitearmDriverNode::CallbackReturn LitearmDriverNode::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  read_parameters();

  have_state_ = false;
  license_valid_ = false;
  speed_scaling_ = 100;
  last_state_stamp_ = 0.0;

  litearm::ArmOptions options;
  if (!port_.empty()) {
    options.port = port_;
  }
  arm_ = create_arm(options);

  if (arm_ == nullptr) {
    RCLCPP_FATAL(get_logger(), "The arm factory returned no object.");
    return CallbackReturn::ERROR;
  }

  try {
    arm_->connect();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      get_logger(),
      "Could not open the arm: %s\n"
      "Check the USB cable, the 24 V supply and the firmware version. If the ros2_control "
      "stack is running, stop it first: it holds an exclusive lock on the serial port.",
      error.what());
    arm_.reset();
    return CallbackReturn::ERROR;
  }

  const int axis_count = arm_->n();
  if (joint_names_.empty()) {
    joint_names_.reserve(static_cast<std::size_t>(axis_count));
    for (int axis = 1; axis <= axis_count; ++axis) {
      joint_names_.push_back("joint" + std::to_string(axis));
    }
  } else if (static_cast<int>(joint_names_.size()) != axis_count) {
    RCLCPP_FATAL(
      get_logger(),
      "The joint_names parameter lists %zu names, but the firmware reports %d axes. "
      "Either leave joint_names empty or list exactly one name per axis.",
      joint_names_.size(), axis_count);
    arm_.reset();
    return CallbackReturn::ERROR;
  }

  try {
    refresh_license_record();
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      get_logger(), "Could not read the licence record yet (%s). It is not fatal: the "
      "status message's licence fields stay empty until the next attempt.", error.what());
  }

  // Publishers are created here and activated in on_activate, which is the lifecycle
  // convention: an inactive node advertises nothing.
  status_publisher_ = create_publisher<litearm_msgs::msg::LitearmStatus>(
    "status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  if (publish_joint_states_) {
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  }
  if (publish_diagnostics_) {
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  }

  RCLCPP_INFO(
    get_logger(), "Configured: %s, %d axes, firmware %s.", arm_->is_connected() ? "connected" : "link down",
    axis_count, arm_->firmware().c_str());
  return CallbackReturn::SUCCESS;
}

LitearmDriverNode::CallbackReturn LitearmDriverNode::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (arm_ == nullptr) {
    RCLCPP_FATAL(get_logger(), "Cannot activate: the arm was never configured.");
    return CallbackReturn::ERROR;
  }

  if (auto_enable_) {
    try {
      arm_->enable(enable_attempts_);
    } catch (const std::exception & error) {
      // Failing the transition is deliberate: a refused ENABLE usually means the licence
      // is not activated or a fault is latched, and an "activated" node whose motors are
      // off is the silent failure this node exists to prevent.
      RCLCPP_FATAL(
        get_logger(), "ENABLE was refused: %s\n"
        "The node stays inactive. Check the licence fields of the status message, or "
        "clear the fault and try again.", error.what());
      return CallbackReturn::ERROR;
    }
  }

  create_service_servers();
  activate_publishers();
  start_timers();

  RCLCPP_INFO(
    get_logger(), "Active: %zu services, status on %s.",
    services_.size(), status_publisher_->get_topic_name());
  return CallbackReturn::SUCCESS;
}

LitearmDriverNode::CallbackReturn LitearmDriverNode::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Leave zero gravity before anything else: it is the only mode whose keep-alive thread
  // would keep writing to a port this node is about to stop servicing.
  if (arm_ != nullptr && arm_->zero_g_active()) {
    try {
      arm_->zero_g_stop(true);
      RCLCPP_INFO(get_logger(), "Left zero gravity on deactivate.");
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Could not leave zero gravity cleanly (%s). The firmware watchdog "
        "will drop the mode within 0.1 s.", error.what());
    }
  }

  stop_timers();
  destroy_service_servers();
  deactivate_publishers();

  // The motors are deliberately left as they are. Disabling them would let the arm fall;
  // park() and the firmware hold keep it where it is.
  RCLCPP_INFO(get_logger(), "Inactive: the arm keeps holding its position.");
  return CallbackReturn::SUCCESS;
}

LitearmDriverNode::CallbackReturn LitearmDriverNode::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_timers();
  destroy_service_servers();
  deactivate_publishers();
  status_publisher_.reset();
  joint_state_publisher_.reset();
  diagnostics_publisher_.reset();

  if (arm_ != nullptr) {
    try {
      arm_->close();
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Ignoring an error while closing the link: %s", error.what());
    }
    arm_.reset();
  }

  have_state_ = false;
  license_valid_ = false;
  last_state_stamp_ = 0.0;
  return CallbackReturn::SUCCESS;
}

LitearmDriverNode::CallbackReturn LitearmDriverNode::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return on_cleanup(rclcpp_lifecycle::State());
}

LitearmDriverNode::CallbackReturn LitearmDriverNode::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (arm_ != nullptr && arm_->is_connected()) {
    RCLCPP_ERROR(
      get_logger(), "Error state with the link up: the firmware holds the arm. "
      "Reconfigure and reactivate once the cause is cleared.");
  } else {
    RCLCPP_ERROR(
      get_logger(), "Error state with the link down: only the firmware's last latched "
      "hold keeps the arm up. Support it if gravity is what is holding the load.");
  }
  return CallbackReturn::SUCCESS;
}

// ────────────────────────────────────────────────────────────────────────────────
// services
// ────────────────────────────────────────────────────────────────────────────────

void LitearmDriverNode::create_service_servers()
{
  using std_srvs::srv::SetBool;
  using std_srvs::srv::Trigger;

  const auto & qos = rmw_qos_profile_services_default;

  const auto trigger = [this, &qos](
                         const std::string & name,
                         void (LitearmDriverNode::*handler)(
                           const Trigger::Request::SharedPtr,
                           Trigger::Response::SharedPtr)) {
      services_.push_back(create_service<Trigger>(
        name,
        [this, handler](
          const Trigger::Request::SharedPtr request,
          Trigger::Response::SharedPtr response) { (this->*handler)(request, response); },
        qos, sdk_group_));
    };

  trigger("reconnect", &LitearmDriverNode::handle_reconnect);
  trigger("home", &LitearmDriverNode::handle_home);
  trigger("revert_model", &LitearmDriverNode::handle_revert_model);
  trigger("log_stop", &LitearmDriverNode::handle_log_stop);
  trigger("enable", &LitearmDriverNode::handle_enable);
  trigger("disable", &LitearmDriverNode::handle_disable);
  trigger("reset", &LitearmDriverNode::handle_reset);
  trigger("clear_faults", &LitearmDriverNode::handle_clear_faults);
  trigger("emergency_stop", &LitearmDriverNode::handle_emergency_stop);
  trigger("park", &LitearmDriverNode::handle_park);
  trigger("save_params", &LitearmDriverNode::handle_save_params);
  trigger("reset_factory_params", &LitearmDriverNode::handle_reset_factory_params);
  trigger("enter_dfu", &LitearmDriverNode::handle_enter_dfu);

  services_.push_back(create_service<SetBool>(
    "zero_g",
    [this](
      const SetBool::Request::SharedPtr request,
      SetBool::Response::SharedPtr response) { handle_zero_g(request, response); },
    qos, sdk_group_));

  register_service<litearm_msgs::srv::GetTcp>("get_tcp", &LitearmDriverNode::handle_get_tcp);
  register_service<litearm_msgs::srv::GetDiagnostics>("get_diagnostics", &LitearmDriverNode::handle_get_diagnostics);
  register_service<litearm_msgs::srv::KinBench>("kin_bench", &LitearmDriverNode::handle_kin_bench);
  register_service<litearm_msgs::srv::MoveJ>("move_j", &LitearmDriverNode::handle_move_j);
  register_service<litearm_msgs::srv::MoveJSync>("move_j_sync", &LitearmDriverNode::handle_move_j_sync);
  register_service<litearm_msgs::srv::MoveP>("move_p", &LitearmDriverNode::handle_move_p);
  register_service<litearm_msgs::srv::MoveJs>("move_js", &LitearmDriverNode::handle_move_js);
  register_service<litearm_msgs::srv::SendMit>("send_mit", &LitearmDriverNode::handle_send_mit);
  register_service<litearm_msgs::srv::SendMitAll>("send_mit_all", &LitearmDriverNode::handle_send_mit_all);
  register_service<litearm_msgs::srv::MoveL>("move_l", &LitearmDriverNode::handle_move_l);
  register_service<litearm_msgs::srv::MoveC>("move_c", &LitearmDriverNode::handle_move_c);
  register_service<litearm_msgs::srv::MovePath>("move_path", &LitearmDriverNode::handle_move_path);
  register_service<litearm_msgs::srv::PollCart>("poll_cart", &LitearmDriverNode::handle_poll_cart);
  register_service<litearm_msgs::srv::InverseKinematics>("inverse_kinematics", &LitearmDriverNode::handle_inverse_kinematics);
  register_service<litearm_msgs::srv::GetFeedforwardVector>("get_feedforward_vector", &LitearmDriverNode::handle_get_feedforward_vector);
  register_service<litearm_msgs::srv::GetFeedforwardMask>("get_feedforward_mask", &LitearmDriverNode::handle_get_feedforward_mask);
  register_service<litearm_msgs::srv::GetFeedforwardCatalog>("get_feedforward_catalog", &LitearmDriverNode::handle_get_feedforward_catalog);
  register_service<litearm_msgs::srv::SetGravityScale>("set_gravity_scale", &LitearmDriverNode::handle_set_gravity_scale);
  register_service<litearm_msgs::srv::SetInertiaScale>("set_inertia_scale", &LitearmDriverNode::handle_set_inertia_scale);
  register_service<litearm_msgs::srv::SetGravityVector>("set_gravity_vector", &LitearmDriverNode::handle_set_gravity_vector);
  register_service<litearm_msgs::srv::ProbeModel>("probe_model", &LitearmDriverNode::handle_probe_model);
  register_service<litearm_msgs::srv::GetModelBody>("get_model_body", &LitearmDriverNode::handle_get_model_body);
  register_service<litearm_msgs::srv::SetModelBody>("set_model_body", &LitearmDriverNode::handle_set_model_body);
  register_service<litearm_msgs::srv::GetModelJm>("get_model_jm", &LitearmDriverNode::handle_get_model_jm);
  register_service<litearm_msgs::srv::SetModelJm>("set_model_jm", &LitearmDriverNode::handle_set_model_jm);
  register_service<litearm_msgs::srv::CommitModel>("commit_model", &LitearmDriverNode::handle_commit_model);
  register_service<litearm_msgs::srv::GetModelStatus>("get_model_status", &LitearmDriverNode::handle_get_model_status);
  register_service<litearm_msgs::srv::LogStart>("log_start", &LitearmDriverNode::handle_log_start);
  register_service<litearm_msgs::srv::LogDump>("log_dump", &LitearmDriverNode::handle_log_dump);
  services_.push_back(create_service<litearm_msgs::srv::GetStatus>(
    "get_status",
    [this](
      const litearm_msgs::srv::GetStatus::Request::SharedPtr request,
      litearm_msgs::srv::GetStatus::Response::SharedPtr response) {
      handle_get_status(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetSpeedScaling>(
    "set_speed_scaling",
    [this](
      const litearm_msgs::srv::SetSpeedScaling::Request::SharedPtr request,
      litearm_msgs::srv::SetSpeedScaling::Response::SharedPtr response) {
      handle_set_speed_scaling(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetMotionMode>(
    "set_motion_mode",
    [this](
      const litearm_msgs::srv::SetMotionMode::Request::SharedPtr request,
      litearm_msgs::srv::SetMotionMode::Response::SharedPtr response) {
      handle_set_motion_mode(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetPayload>(
    "set_payload",
    [this](
      const litearm_msgs::srv::SetPayload::Request::SharedPtr request,
      litearm_msgs::srv::SetPayload::Response::SharedPtr response) {
      handle_set_payload(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetFeedforwardMask>(
    "set_feedforward_mask",
    [this](
      const litearm_msgs::srv::SetFeedforwardMask::Request::SharedPtr request,
      litearm_msgs::srv::SetFeedforwardMask::Response::SharedPtr response) {
      handle_set_feedforward_mask(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetFeedforwardPreset>(
    "set_feedforward_preset",
    [this](
      const litearm_msgs::srv::SetFeedforwardPreset::Request::SharedPtr request,
      litearm_msgs::srv::SetFeedforwardPreset::Response::SharedPtr response) {
      handle_set_feedforward_preset(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetFeedforwardScalar>(
    "set_feedforward_scalar",
    [this](
      const litearm_msgs::srv::SetFeedforwardScalar::Request::SharedPtr request,
      litearm_msgs::srv::SetFeedforwardScalar::Response::SharedPtr response) {
      handle_set_feedforward_scalar(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetFeedforwardVector>(
    "set_feedforward_vector",
    [this](
      const litearm_msgs::srv::SetFeedforwardVector::Request::SharedPtr request,
      litearm_msgs::srv::SetFeedforwardVector::Response::SharedPtr response) {
      handle_set_feedforward_vector(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::GetFeedforwardScalar>(
    "get_feedforward_scalar",
    [this](
      const litearm_msgs::srv::GetFeedforwardScalar::Request::SharedPtr request,
      litearm_msgs::srv::GetFeedforwardScalar::Response::SharedPtr response) {
      handle_get_feedforward_scalar(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::GetJointParams>(
    "get_joint_params",
    [this](
      const litearm_msgs::srv::GetJointParams::Request::SharedPtr request,
      litearm_msgs::srv::GetJointParams::Response::SharedPtr response) {
      handle_get_joint_params(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetJointGains>(
    "set_joint_gains",
    [this](
      const litearm_msgs::srv::SetJointGains::Request::SharedPtr request,
      litearm_msgs::srv::SetJointGains::Response::SharedPtr response) {
      handle_set_joint_gains(request, response);
    },
    qos, sdk_group_));

  services_.push_back(create_service<litearm_msgs::srv::SetJointLimits>(
    "set_joint_limits",
    [this](
      const litearm_msgs::srv::SetJointLimits::Request::SharedPtr request,
      litearm_msgs::srv::SetJointLimits::Response::SharedPtr response) {
      handle_set_joint_limits(request, response);
    },
    qos, sdk_group_));
}

void LitearmDriverNode::destroy_service_servers()
{
  services_.clear();
}

void LitearmDriverNode::activate_publishers()
{
  if (status_publisher_) {
    status_publisher_->on_activate();
  }
  if (joint_state_publisher_) {
    joint_state_publisher_->on_activate();
  }
  if (diagnostics_publisher_) {
    diagnostics_publisher_->on_activate();
  }
}

void LitearmDriverNode::deactivate_publishers()
{
  if (status_publisher_) {
    status_publisher_->on_deactivate();
  }
  if (joint_state_publisher_) {
    joint_state_publisher_->on_deactivate();
  }
  if (diagnostics_publisher_) {
    diagnostics_publisher_->on_deactivate();
  }
}

void LitearmDriverNode::start_timers()
{
  if (status_rate_hz_ > 0.0) {
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_rate_hz_),
      [this]() {publish_status();}, sdk_group_);
  }
  if (diagnostics_publisher_ && diagnostics_rate_hz_ > 0.0) {
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / diagnostics_rate_hz_),
      [this]() {publish_diagnostics();}, sdk_group_);
  }
}

void LitearmDriverNode::stop_timers()
{
  status_timer_.reset();
  diagnostics_timer_.reset();
}

// ────────────────────────────────────────────────────────────────────────────────
// helpers
// ────────────────────────────────────────────────────────────────────────────────

bool LitearmDriverNode::connected() const
{
  return arm_ != nullptr && arm_->is_connected();
}

bool LitearmDriverNode::motors_enabled() const
{
  if (arm_ == nullptr) {
    return false;
  }
  try {
    const auto message = arm_->get_state(false);
    if (message.value) {
      return message.value->enabled();
    }
  } catch (const std::exception &) {
    // A state read that fails is not evidence of enablement; the SDK and the firmware both
    // enforce "disabled" on the write itself, so a false negative is safe here.
  }
  return false;
}

const litearm::RobotState * LitearmDriverNode::cached_state()
{
  if (arm_ != nullptr) {
    try {
      const auto message = arm_->get_state(false);
      if (message.value) {
        last_state_ = *message.value;
        last_state_stamp_ = message.timestamp;
        have_state_ = true;
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Could not read the cached state: %s", error.what());
    }
  }
  return have_state_ ? &last_state_ : nullptr;
}

void LitearmDriverNode::refresh_license_record()
{
  if (arm_ == nullptr) {
    return;
  }
  license_ = arm_->license();
  license_valid_ = true;
}

// ────────────────────────────────────────────────────────────────────────────────
// publishing
// ────────────────────────────────────────────────────────────────────────────────

litearm_msgs::msg::LitearmStatus LitearmDriverNode::build_status(
  const litearm::RobotState * fresh)
{
  litearm_msgs::msg::LitearmStatus status;
  status.header.stamp = now();
  status.header.frame_id = frame_id_;

  status.connected = connected();
  status.in_dfu = arm_ != nullptr && arm_->is_in_dfu();
  status.firmware = arm_ != nullptr ? arm_->firmware() : std::string();
  status.last_reset_reason = arm_ != nullptr ? arm_->last_reset_reason() : std::string();
  status.speed_scaling = speed_scaling_;
  status.zero_g_active = arm_ != nullptr && arm_->zero_g_active();
  status.zero_g_error = arm_ != nullptr ? arm_->zero_g_error_text() : std::string();

  const litearm::RobotState * state = fresh;
  if (state == nullptr && have_state_) {
    state = &last_state_;
  }
  if (state != nullptr) {
    status.enabled = state->enabled();
    status.faulted = state->faulted();
    status.cart_busy = state->cart_busy();
    status.joint_fault = state->joint_fault;
    status.drop_hold_inferred = state->drop_hold_inferred();
    status.mode = static_cast<uint16_t>(state->mode);
    status.mode_name = state->mode_name;
    status.flags = state->flags;
  }

  status.joint_names = joint_names_;
  const std::size_t count = joint_names_.size();
  status.temperature_mos.assign(count, std::nan(""));
  status.temperature_coil.assign(count, std::nan(""));
  status.joint_error_code.assign(count, 0);
  status.feedback_age = frame_age(last_state_stamp_);
  if (state != nullptr) {
    const std::size_t axes = std::min(count, state->joints.size());
    for (std::size_t i = 0; i < axes; ++i) {
      status.temperature_mos[i] = state->joints[i].t_mos;
      status.temperature_coil[i] = state->joints[i].t_coil;
      status.joint_error_code[i] = state->joints[i].err;
    }
  }

  status.license_valid = license_valid_;
  if (license_valid_) {
    status.license_state = static_cast<uint8_t>(license_.state);
    status.license_state_name = license_.state_name();
    status.license_uid = license_.uid_hex();
  }
  return status;
}

void LitearmDriverNode::publish_status()
{
  const litearm::RobotState * state = cached_state();
  if (status_publisher_) {
    status_publisher_->publish(build_status(state));
  }

  if (joint_state_publisher_ && state != nullptr) {
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    joints.header.frame_id = frame_id_;
    joints.name = joint_names_;
    const std::size_t axes = std::min(joint_names_.size(), state->joints.size());
    joints.position.reserve(axes);
    joints.velocity.reserve(axes);
    joints.effort.reserve(axes);
    for (std::size_t i = 0; i < axes; ++i) {
      joints.position.push_back(state->joints[i].q);
      joints.velocity.push_back(state->joints[i].dq);
      joints.effort.push_back(state->joints[i].tau);
    }
    joint_state_publisher_->publish(joints);
  }
}

void LitearmDriverNode::publish_diagnostics()
{
  if (!diagnostics_publisher_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_name()) + ": litearm link";
  status.hardware_id = arm_ != nullptr ? arm_->firmware() : std::string();
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

  std::string problems;
  const auto note = [&problems](const std::string & text) {
      if (!problems.empty()) {
        problems += "; ";
      }
      problems += text;
    };

  if (!connected()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    note("link down");
  }
  if (arm_ != nullptr && arm_->is_in_dfu()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    note("in DFU");
  }
  if (have_state_ && last_state_.faulted()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    note("faulted: " + last_state_.fault_detail());
  }
  if (have_state_ && last_state_.drop_hold_inferred()) {
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    note("rigid-hold latch inferred (reset or clear_faults to release)");
  }
  if (license_valid_ && !license_.activated()) {
    if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    note("licence not activated: ENABLE will be refused");
  }
  if (arm_ != nullptr && arm_->zero_g_active()) {
    note("zero gravity active");
  }
  status.message = problems.empty() ? "ok" : problems;

  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue pair;
      pair.key = key;
      pair.value = value;
      status.values.push_back(pair);
    };
  add("connected", connected() ? "true" : "false");
  add("enabled", have_state_ && last_state_.enabled() ? "true" : "false");
  add("mode", have_state_ ? last_state_.mode_name : std::string("unknown"));
  add("joint_fault", std::to_string(have_state_ ? last_state_.joint_fault : 0));
  add("speed_scaling_percent", std::to_string(speed_scaling_));
  add("zero_g_active", (arm_ != nullptr && arm_->zero_g_active()) ? "true" : "false");
  add("feedback_age_s", std::to_string(frame_age(last_state_stamp_)));
  add(
    "license_state",
    license_valid_ ? license_.state_name() : std::string("unknown"));

  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

// ────────────────────────────────────────────────────────────────────────────────
// zero gravity
// ────────────────────────────────────────────────────────────────────────────────

void LitearmDriverNode::start_zero_g(double period_s)
{
  if (arm_ == nullptr) {
    return;
  }
  // The SDK owns the keep-alive thread; the node only decides when it runs.
  arm_->zero_g_start(period_s);
}

void LitearmDriverNode::stop_zero_g()
{
  if (arm_ == nullptr) {
    return;
  }
  // raise_on_lost=true: a keep-alive that died is not silence, it means the arm already
  // fell out of zero gravity, and the caller has to know.
  arm_->zero_g_stop(true);
}

// ────────────────────────────────────────────────────────────────────────────────
// service handlers: state and safety
// ────────────────────────────────────────────────────────────────────────────────

void LitearmDriverNode::handle_enable(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  run_command(arm_.get(), "enable", *response, [this](litearm::Arm & arm) {
    arm.enable(enable_attempts_);
  });
}

void LitearmDriverNode::handle_disable(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  run_command(arm_.get(), "disable", *response, [](litearm::Arm & arm) {
    arm.disable();
  });
}

void LitearmDriverNode::handle_reset(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  run_command(arm_.get(), "reset", *response, [](litearm::Arm & arm) {
    arm.reset();
  });
}

void LitearmDriverNode::handle_clear_faults(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  run_command(arm_.get(), "clear_faults", *response, [](litearm::Arm & arm) {
    arm.clear_faults();
  });
}

void LitearmDriverNode::handle_emergency_stop(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  run_command(arm_.get(), "emergency_stop", *response, [](litearm::Arm & arm) {
    arm.emergency_stop();
  });
  if (response->success) {
    response->message += " (software request; the hardware emergency stop remains the authority)";
  }
}

void LitearmDriverNode::handle_park(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  run_command(arm_.get(), "park", *response, [](litearm::Arm & arm) {
    arm.park();
  });
}

void LitearmDriverNode::handle_save_params(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  const GateResult gate = check_requires_disabled(motors_enabled(), "save_params");
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "save_params", *response, [](litearm::Arm & arm) {
    arm.save_params();
  });
}

void LitearmDriverNode::handle_reset_factory_params(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  const GateResult gate = check_requires_disabled(motors_enabled(), "reset_factory_params");
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "reset_factory_params", *response, [](litearm::Arm & arm) {
    arm.params().reset_factory();
  });
}

void LitearmDriverNode::handle_enter_dfu(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  const bool allowed = get_parameter("allow_dfu").as_bool();
  const GateResult gate = check_parameter_allows(allowed, "allow_dfu");
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "enter_dfu", *response, [](litearm::Arm & arm) {
    arm.enter_dfu();
  });
  if (response->success) {
    response->message =
      "device entered DFU: the CDC link is gone and this node is unusable until it is "
      "reconfigured with a new link";
  }
}

void LitearmDriverNode::handle_zero_g(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  if (request->data) {
    // Read the period at call time so `ros2 param set` can retune it without a restart.
    const double period_s = get_parameter("zero_g_keepalive_period_s").as_double();
    const GateResult gate = check_zero_g_period(period_s);
    if (!gate.ok) {
      fill_failure(*response, gate);
      return;
    }
    run_command(arm_.get(), "zero_g", *response, [this, period_s](litearm::Arm &) {
      start_zero_g(period_s);
    });
    if (response->success) {
      response->message =
        "zero gravity entered; push the arm by hand, and leave the service (data=false) "
        "before any other motion command";
    }
  } else {
    run_command(arm_.get(), "zero_g", *response, [this](litearm::Arm &) {
      stop_zero_g();
    });
    if (response->success) {
      response->message = "zero gravity left";
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────────
// service handlers: status
// ────────────────────────────────────────────────────────────────────────────────

void LitearmDriverNode::handle_get_status(
  const litearm_msgs::srv::GetStatus::Request::SharedPtr request,
  litearm_msgs::srv::GetStatus::Response::SharedPtr response)
{
  run_command(arm_.get(), "get_status", *response, [this, &request, &response](litearm::Arm & arm) {
    const auto message = arm.get_status_now(request->timeout);
    last_state_ = message.value;
    last_state_stamp_ = message.timestamp;
    have_state_ = true;
    response->status = build_status(&last_state_);
    response->message = "status taken from the firmware";
  });
}

// ────────────────────────────────────────────────────────────────────────────────
// service handlers: motion configuration
// ────────────────────────────────────────────────────────────────────────────────

void LitearmDriverNode::handle_set_speed_scaling(
  const litearm_msgs::srv::SetSpeedScaling::Request::SharedPtr request,
  litearm_msgs::srv::SetSpeedScaling::Response::SharedPtr response)
{
  const GateResult gate = check_speed_percent(static_cast<int>(request->percent));
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_speed_scaling", *response, [this, &request, &response](
      litearm::Arm & arm) {
    arm.set_speed(static_cast<int>(request->percent));
    speed_scaling_ = request->percent;
    response->message =
      "global speed governor set to " + std::to_string(request->percent) + "%";
  });
}

void LitearmDriverNode::handle_set_motion_mode(
  const litearm_msgs::srv::SetMotionMode::Request::SharedPtr request,
  litearm_msgs::srv::SetMotionMode::Response::SharedPtr response)
{
  run_command(arm_.get(), "set_motion_mode", *response, [&request](litearm::Arm & arm) {
    arm.set_motion_mode(static_cast<int>(request->mode));
  });
  if (response->success) {
    response->message +=
      " (this firmware recognises mode 0 only, which is park(); the SDK rejects other "
      "values rather than ACKing a mode change that does not happen)";
  }
}

void LitearmDriverNode::handle_set_payload(
  const litearm_msgs::srv::SetPayload::Request::SharedPtr request,
  litearm_msgs::srv::SetPayload::Response::SharedPtr response)
{
  const GateResult mass = check_mass(request->mass);
  if (!mass.ok) {
    fill_failure(*response, mass);
    return;
  }
  const std::array<double, 3> com{request->com[0], request->com[1], request->com[2]};
  const GateResult centre = check_com(com);
  if (!centre.ok) {
    fill_failure(*response, centre);
    return;
  }
  run_command(arm_.get(), "set_payload", *response, [&request, &com](litearm::Arm & arm) {
    arm.set_payload(request->mass, com);
  });
  if (response->success) {
    response->message +=
      " (read it back with get_feedforward_scalar item 4 before trusting it: the "
      "firmware clamps silently)";
  }
}

// ────────────────────────────────────────────────────────────────────────────────
// service handlers: feedforward and joint parameters
// ────────────────────────────────────────────────────────────────────────────────

void LitearmDriverNode::handle_set_feedforward_mask(
  const litearm_msgs::srv::SetFeedforwardMask::Request::SharedPtr request,
  litearm_msgs::srv::SetFeedforwardMask::Response::SharedPtr response)
{
  run_command(arm_.get(), "set_feedforward_mask", *response, [&request](litearm::Arm & arm) {
    arm.set_ff_mask(request->mask);
  });
}

void LitearmDriverNode::handle_set_feedforward_preset(
  const litearm_msgs::srv::SetFeedforwardPreset::Request::SharedPtr request,
  litearm_msgs::srv::SetFeedforwardPreset::Response::SharedPtr response)
{
  const GateResult gate = check_ff_preset(static_cast<int>(request->preset));
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_feedforward_preset", *response, [&request](litearm::Arm & arm) {
    arm.ff_preset(static_cast<int>(request->preset));
  });
}

void LitearmDriverNode::handle_set_feedforward_scalar(
  const litearm_msgs::srv::SetFeedforwardScalar::Request::SharedPtr request,
  litearm_msgs::srv::SetFeedforwardScalar::Response::SharedPtr response)
{
  run_command(arm_.get(), "set_feedforward_scalar", *response, [&request, &response](
      litearm::Arm & arm) {
    arm.set_ff_scalar(
      static_cast<int>(request->item), static_cast<int>(request->sub), request->value);
    response->message =
      "scalar written; read it back with get_feedforward_scalar, because the firmware "
      "clamps silently";
  });
}

void LitearmDriverNode::handle_set_feedforward_vector(
  const litearm_msgs::srv::SetFeedforwardVector::Request::SharedPtr request,
  litearm_msgs::srv::SetFeedforwardVector::Response::SharedPtr response)
{
  const GateResult gate = check_ff_values(request->values);
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_feedforward_vector", *response, [&request](litearm::Arm & arm) {
    arm.set_ff_vec(static_cast<int>(request->item), request->values);
  });
}

void LitearmDriverNode::handle_get_feedforward_scalar(
  const litearm_msgs::srv::GetFeedforwardScalar::Request::SharedPtr request,
  litearm_msgs::srv::GetFeedforwardScalar::Response::SharedPtr response)
{
  run_command(arm_.get(), "get_feedforward_scalar", *response, [&request, &response](
      litearm::Arm & arm) {
    const auto message =
      arm.get_ff_scalar(static_cast<int>(request->item), static_cast<int>(request->sub));
    response->value = message.value;
    response->message = "value read back from the firmware";
  });
}

void LitearmDriverNode::handle_get_joint_params(
  const litearm_msgs::srv::GetJointParams::Request::SharedPtr request,
  litearm_msgs::srv::GetJointParams::Response::SharedPtr response)
{
  if (!connected()) {
    fill_failure(*response, check_connected(false));
    return;
  }
  if (request->joint >= 0) {
    const GateResult gate = check_joint_index(static_cast<int>(request->joint), arm_->n());
    if (!gate.ok) {
      fill_failure(*response, gate);
      return;
    }
  }
  run_command(arm_.get(), "get_joint_params", *response, [&request, &response](
      litearm::Arm & arm) {
    std::vector<litearm::JointParam> params;
    if (request->joint < 0) {
      params = arm.params().all_joint_params();
    } else {
      params.push_back(arm.params().get_joint_param(static_cast<int>(request->joint)).value);
    }
    response->params.reserve(params.size());
    for (const litearm::JointParam & param : params) {
      litearm_msgs::msg::JointParam out;
      out.index = param.idx;
      out.kp = param.kp;
      out.kd = param.kd;
      out.tau_max = param.tau_max;
      out.q_min = param.q_min;
      out.q_max = param.q_max;
      response->params.push_back(out);
    }
  });
}

void LitearmDriverNode::handle_set_joint_gains(
  const litearm_msgs::srv::SetJointGains::Request::SharedPtr request,
  litearm_msgs::srv::SetJointGains::Response::SharedPtr response)
{
  if (!connected()) {
    fill_failure(*response, check_connected(false));
    return;
  }
  const GateResult gate = check_joint_index(static_cast<int>(request->joint), arm_->n());
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_joint_gains", *response, [&request](litearm::Arm & arm) {
    arm.params().set_joint_param(
      static_cast<int>(request->joint), request->kp, request->kd, request->tau_max);
  });
}

void LitearmDriverNode::handle_set_joint_limits(
  const litearm_msgs::srv::SetJointLimits::Request::SharedPtr request,
  litearm_msgs::srv::SetJointLimits::Response::SharedPtr response)
{
  if (!connected()) {
    fill_failure(*response, check_connected(false));
    return;
  }
  const GateResult gate = check_joint_index(static_cast<int>(request->joint), arm_->n());
  if (!gate.ok) {
    fill_failure(*response, gate);
    return;
  }
  run_command(arm_.get(), "set_joint_limits", *response, [&request](litearm::Arm & arm) {
    arm.params().set_joint_limits(
      static_cast<int>(request->joint), request->q_min, request->q_max);
  });
}

}  // namespace litearm_driver
