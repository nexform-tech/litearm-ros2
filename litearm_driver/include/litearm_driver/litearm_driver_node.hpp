// litearm_driver_node.hpp — the standalone maintenance driver for the LiteArm arm.
//
// This node owns the arm's USB CDC link itself, through the litearm C++ SDK, and exposes
// the SDK's administrative command set as ROS 2 services plus a status topic. It is the
// counterpart of the ros2_control hardware component in this repository, not a companion:
// both open the same serial port, and the SDK takes an exclusive flock on it, so running
// the two at once is impossible by construction. Start this node when you want to
// diagnose, calibrate, license or tune the arm without a controller_manager in the loop.
//
// Design notes
// ------------
// * Services are created on activation and destroyed on deactivation, so an inactive node
//   advertises nothing it cannot serve.
// * Every SDK call is serialised through one mutually exclusive callback group. The SDK
//   blocks on the firmware ACK for up to its own timeout, so a service call can delay the
//   status publication; that is the price of never interleaving two commands on the wire.
// * The status publisher reads only cached or local SDK state. It never sends a frame, so
//   subscribing cannot disturb a command in flight.

#ifndef LITEARM_DRIVER__LITEARM_DRIVER_NODE_HPP_
#define LITEARM_DRIVER__LITEARM_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <litearm_msgs/msg/litearm_status.hpp>
#include <litearm_msgs/srv/get_feedforward_scalar.hpp>
#include <litearm_msgs/srv/get_joint_params.hpp>
#include <litearm_msgs/srv/get_status.hpp>
#include <litearm_msgs/srv/set_feedforward_mask.hpp>
#include <litearm_msgs/srv/set_feedforward_preset.hpp>
#include <litearm_msgs/srv/set_feedforward_scalar.hpp>
#include <litearm_msgs/srv/set_feedforward_vector.hpp>
#include <litearm_msgs/srv/set_joint_gains.hpp>
#include <litearm_msgs/srv/set_joint_limits.hpp>
#include <litearm_msgs/srv/set_motion_mode.hpp>
#include <litearm_msgs/srv/set_payload.hpp>
#include <litearm_msgs/srv/set_speed_scaling.hpp>
#include <litearm_msgs/srv/get_tcp.hpp>
#include <litearm_msgs/srv/get_diagnostics.hpp>
#include <litearm_msgs/srv/kin_bench.hpp>
#include <litearm_msgs/srv/move_j.hpp>
#include <litearm_msgs/srv/move_j_sync.hpp>
#include <litearm_msgs/srv/move_p.hpp>
#include <litearm_msgs/srv/move_js.hpp>
#include <litearm_msgs/srv/send_mit.hpp>
#include <litearm_msgs/srv/send_mit_all.hpp>
#include <litearm_msgs/srv/move_l.hpp>
#include <litearm_msgs/srv/move_c.hpp>
#include <litearm_msgs/srv/move_path.hpp>
#include <litearm_msgs/srv/poll_cart.hpp>
#include <litearm_msgs/srv/inverse_kinematics.hpp>
#include <litearm_msgs/srv/get_feedforward_vector.hpp>
#include <litearm_msgs/srv/get_feedforward_mask.hpp>
#include <litearm_msgs/srv/get_feedforward_catalog.hpp>
#include <litearm_msgs/srv/set_gravity_scale.hpp>
#include <litearm_msgs/srv/set_inertia_scale.hpp>
#include <litearm_msgs/srv/set_gravity_vector.hpp>
#include <litearm_msgs/srv/probe_model.hpp>
#include <litearm_msgs/srv/get_model_body.hpp>
#include <litearm_msgs/srv/set_model_body.hpp>
#include <litearm_msgs/srv/get_model_jm.hpp>
#include <litearm_msgs/srv/set_model_jm.hpp>
#include <litearm_msgs/srv/commit_model.hpp>
#include <litearm_msgs/srv/get_model_status.hpp>
#include <litearm_msgs/srv/log_start.hpp>
#include <litearm_msgs/srv/log_dump.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <cstddef>
#include <utility>

#include "litearm/arm.hpp"
#include "litearm_driver/safety_gates.hpp"
#include "litearm_driver/service_helpers.hpp"

namespace litearm_driver
{

/// Lifecycle node that exposes the litearm SDK's administrative command set over ROS 2.
///
/// Parameters (all optional):
///
///   port                         USB CDC device path. Empty (the default) triggers the
///                                SDK's auto-discovery by VID:PID 1d50:606f.
///   joint_names                  Axis names, in firmware axis order. Empty (the default)
///                                derives joint1..jointN from what the firmware reports.
///                                A non-empty list of the wrong length fails configure.
///   auto_enable                  Enable the motors on activation. Default true. A failed
///                                enable fails the activation, which is what keeps a
///                                refused ENABLE visible instead of silently inactive.
///   enable_attempts              enable() retries. Default 12.
///   status_rate_hz               /litearm/status publication rate. Default 10.
///   publish_joint_states         Publish the optional joint_states topic. Default false:
///                                when the ros2_control stack runs, its
///                                joint_state_broadcaster owns that topic, and this node
///                                must not compete with it.
///   joint_states_topic           Topic for the optional joint states. Default
///                                /joint_states.
///   publish_diagnostics          Publish /diagnostics. Default true.
///   diagnostics_rate_hz          /diagnostics publication rate. Default 1.
///   zero_g_keepalive_period_s    Zero-gravity keep-alive period, [0.005, 0.10). Default
///                                0.04.
///   allow_dfu                    Allow the enter_dfu service. Default false.
///   allow_motion                 Allow the services that can energise the motors:
///                                move_j, move_j_sync, move_p, move_js, send_mit,
///                                send_mit_all, home, move_l, move_c, move_path.
///                                Default false: this node is a maintenance driver,
///                                and nothing about diagnosing the arm requires it to
///                                be able to move it. inverse_kinematics only solves
///                                and is not gated.
///   log_dir                      Directory the log_dump service writes into. Empty
///                                (the default) means $HOME/.ros/litearm.
///   frame_id                     Header frame id of the status and joint state messages.
///                                Default empty.
class LitearmDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn;

  explicit LitearmDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LitearmDriverNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

protected:
  /// Factory seam for the SDK object.
  ///
  /// Production builds a real `litearm::Arm`; the offline test overrides this to inject
  /// `litearm::testing::FakeTransport` and run the whole lifecycle without hardware.
  virtual std::unique_ptr<litearm::Arm> create_arm(const litearm::ArmOptions & options);

  // ── service handlers ─────────────────────────────────────────────────────────────
  // Protected rather than private so the offline test can call them directly: no client,
  // no executor, no spin.

  void handle_enable(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_disable(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_clear_faults(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_emergency_stop(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_park(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_save_params(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_reset_factory_params(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_enter_dfu(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  void handle_zero_g(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  void handle_get_status(
    const litearm_msgs::srv::GetStatus::Request::SharedPtr request,
    litearm_msgs::srv::GetStatus::Response::SharedPtr response);
  void handle_set_speed_scaling(
    const litearm_msgs::srv::SetSpeedScaling::Request::SharedPtr request,
    litearm_msgs::srv::SetSpeedScaling::Response::SharedPtr response);
  void handle_set_motion_mode(
    const litearm_msgs::srv::SetMotionMode::Request::SharedPtr request,
    litearm_msgs::srv::SetMotionMode::Response::SharedPtr response);
  void handle_set_payload(
    const litearm_msgs::srv::SetPayload::Request::SharedPtr request,
    litearm_msgs::srv::SetPayload::Response::SharedPtr response);
  void handle_set_feedforward_mask(
    const litearm_msgs::srv::SetFeedforwardMask::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardMask::Response::SharedPtr response);
  void handle_set_feedforward_preset(
    const litearm_msgs::srv::SetFeedforwardPreset::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardPreset::Response::SharedPtr response);
  void handle_set_feedforward_scalar(
    const litearm_msgs::srv::SetFeedforwardScalar::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardScalar::Response::SharedPtr response);
  void handle_set_feedforward_vector(
    const litearm_msgs::srv::SetFeedforwardVector::Request::SharedPtr request,
    litearm_msgs::srv::SetFeedforwardVector::Response::SharedPtr response);
  void handle_get_feedforward_scalar(
    const litearm_msgs::srv::GetFeedforwardScalar::Request::SharedPtr request,
    litearm_msgs::srv::GetFeedforwardScalar::Response::SharedPtr response);
  void handle_get_joint_params(
    const litearm_msgs::srv::GetJointParams::Request::SharedPtr request,
    litearm_msgs::srv::GetJointParams::Response::SharedPtr response);
  void handle_set_joint_gains(
    const litearm_msgs::srv::SetJointGains::Request::SharedPtr request,
    litearm_msgs::srv::SetJointGains::Response::SharedPtr response);
  void handle_set_joint_limits(
    const litearm_msgs::srv::SetJointLimits::Request::SharedPtr request,
    litearm_msgs::srv::SetJointLimits::Response::SharedPtr response);

  // Motion, cartesian, inverse kinematics, model store, diagnostics and the tick log.
  // The implementations live in litearm_driver_motion.cpp, litearm_driver_model.cpp and
  // litearm_driver_diagnostics.cpp; the split is by area, not by visibility.
  void handle_get_tcp(
    const litearm_msgs::srv::GetTcp::Request::SharedPtr request,
    litearm_msgs::srv::GetTcp::Response::SharedPtr response);
  void handle_get_diagnostics(
    const litearm_msgs::srv::GetDiagnostics::Request::SharedPtr request,
    litearm_msgs::srv::GetDiagnostics::Response::SharedPtr response);
  void handle_kin_bench(
    const litearm_msgs::srv::KinBench::Request::SharedPtr request,
    litearm_msgs::srv::KinBench::Response::SharedPtr response);
  void handle_move_j(
    const litearm_msgs::srv::MoveJ::Request::SharedPtr request,
    litearm_msgs::srv::MoveJ::Response::SharedPtr response);
  void handle_move_j_sync(
    const litearm_msgs::srv::MoveJSync::Request::SharedPtr request,
    litearm_msgs::srv::MoveJSync::Response::SharedPtr response);
  void handle_move_p(
    const litearm_msgs::srv::MoveP::Request::SharedPtr request,
    litearm_msgs::srv::MoveP::Response::SharedPtr response);
  void handle_move_js(
    const litearm_msgs::srv::MoveJs::Request::SharedPtr request,
    litearm_msgs::srv::MoveJs::Response::SharedPtr response);
  void handle_send_mit(
    const litearm_msgs::srv::SendMit::Request::SharedPtr request,
    litearm_msgs::srv::SendMit::Response::SharedPtr response);
  void handle_send_mit_all(
    const litearm_msgs::srv::SendMitAll::Request::SharedPtr request,
    litearm_msgs::srv::SendMitAll::Response::SharedPtr response);
  void handle_move_l(
    const litearm_msgs::srv::MoveL::Request::SharedPtr request,
    litearm_msgs::srv::MoveL::Response::SharedPtr response);
  void handle_move_c(
    const litearm_msgs::srv::MoveC::Request::SharedPtr request,
    litearm_msgs::srv::MoveC::Response::SharedPtr response);
  void handle_move_path(
    const litearm_msgs::srv::MovePath::Request::SharedPtr request,
    litearm_msgs::srv::MovePath::Response::SharedPtr response);
  void handle_poll_cart(
    const litearm_msgs::srv::PollCart::Request::SharedPtr request,
    litearm_msgs::srv::PollCart::Response::SharedPtr response);
  void handle_inverse_kinematics(
    const litearm_msgs::srv::InverseKinematics::Request::SharedPtr request,
    litearm_msgs::srv::InverseKinematics::Response::SharedPtr response);
  void handle_get_feedforward_vector(
    const litearm_msgs::srv::GetFeedforwardVector::Request::SharedPtr request,
    litearm_msgs::srv::GetFeedforwardVector::Response::SharedPtr response);
  void handle_get_feedforward_mask(
    const litearm_msgs::srv::GetFeedforwardMask::Request::SharedPtr request,
    litearm_msgs::srv::GetFeedforwardMask::Response::SharedPtr response);
  void handle_get_feedforward_catalog(
    const litearm_msgs::srv::GetFeedforwardCatalog::Request::SharedPtr request,
    litearm_msgs::srv::GetFeedforwardCatalog::Response::SharedPtr response);
  void handle_set_gravity_scale(
    const litearm_msgs::srv::SetGravityScale::Request::SharedPtr request,
    litearm_msgs::srv::SetGravityScale::Response::SharedPtr response);
  void handle_set_inertia_scale(
    const litearm_msgs::srv::SetInertiaScale::Request::SharedPtr request,
    litearm_msgs::srv::SetInertiaScale::Response::SharedPtr response);
  void handle_set_gravity_vector(
    const litearm_msgs::srv::SetGravityVector::Request::SharedPtr request,
    litearm_msgs::srv::SetGravityVector::Response::SharedPtr response);
  void handle_probe_model(
    const litearm_msgs::srv::ProbeModel::Request::SharedPtr request,
    litearm_msgs::srv::ProbeModel::Response::SharedPtr response);
  void handle_get_model_body(
    const litearm_msgs::srv::GetModelBody::Request::SharedPtr request,
    litearm_msgs::srv::GetModelBody::Response::SharedPtr response);
  void handle_set_model_body(
    const litearm_msgs::srv::SetModelBody::Request::SharedPtr request,
    litearm_msgs::srv::SetModelBody::Response::SharedPtr response);
  void handle_get_model_jm(
    const litearm_msgs::srv::GetModelJm::Request::SharedPtr request,
    litearm_msgs::srv::GetModelJm::Response::SharedPtr response);
  void handle_set_model_jm(
    const litearm_msgs::srv::SetModelJm::Request::SharedPtr request,
    litearm_msgs::srv::SetModelJm::Response::SharedPtr response);
  void handle_commit_model(
    const litearm_msgs::srv::CommitModel::Request::SharedPtr request,
    litearm_msgs::srv::CommitModel::Response::SharedPtr response);
  void handle_get_model_status(
    const litearm_msgs::srv::GetModelStatus::Request::SharedPtr request,
    litearm_msgs::srv::GetModelStatus::Response::SharedPtr response);
  void handle_log_start(
    const litearm_msgs::srv::LogStart::Request::SharedPtr request,
    litearm_msgs::srv::LogStart::Response::SharedPtr response);
  void handle_log_dump(
    const litearm_msgs::srv::LogDump::Request::SharedPtr request,
    litearm_msgs::srv::LogDump::Response::SharedPtr response);
  void handle_reconnect(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_home(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_revert_model(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void handle_log_stop(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  /// The SDK is reachable and must be used now.
  bool connected() const;

  /// True when the firmware currently reports the motors enabled.
  bool motors_enabled() const;

  /// Build a status message. `fresh` is used when it is set (the on-demand service), and
  /// the cached frame otherwise (the publisher).
  litearm_msgs::msg::LitearmStatus build_status(const litearm::RobotState * fresh);

  /// Fill the cached licence record by reading the device. Blocking, so it is called on
  /// configure, on activation and by the licence services — never by a timer.
  void refresh_license_record();

  /// Cached frame, or nullopt when this session has not seen one yet.
  const litearm::RobotState * cached_state();

  /// Number of axes this session was configured with (the firmware's axis count).
  std::size_t joint_count() const { return joint_names_.size(); }

  /// Every motion service sits behind allow_motion. Returns false and fills the response
  /// when the switch is off, so a handler is one line: if (!require_motion_allowed(...)).
  template<class Response>
  bool require_motion_allowed(Response & response)
  {
    // Read at call time, like allow_dfu, so `ros2 param set`
    // can arm or disarm the motion services without a restart.
    const GateResult gate = check_parameter_allows(
      get_parameter("allow_motion").as_bool(), "allow_motion");
    if (!gate.ok) {
      fill_failure(response, gate);
      return false;
    }
    return true;
  }

  /// Register one service whose handler is a plain member function.
  template<class Srv>
  void register_service(
    const std::string & name,
    void (LitearmDriverNode::*handler)(
      const typename Srv::Request::SharedPtr,
      typename Srv::Response::SharedPtr))
  {
    services_.push_back(create_service<Srv>(
      name,
      [this, handler](
        const typename Srv::Request::SharedPtr request,
        typename Srv::Response::SharedPtr response) {
        (this->*handler)(request, response);
      },
      rclcpp::ServicesQoS().get_rmw_qos_profile(), sdk_group_));
  }

private:
  void read_parameters();
  void create_service_servers();
  void destroy_service_servers();
  void activate_publishers();
  void deactivate_publishers();
  void start_timers();
  void stop_timers();
  void publish_status();
  void publish_diagnostics();
  void start_zero_g(double period_s);
  void stop_zero_g();

  // Parameters, declared in the constructor and read in on_configure. The two opt-in
  // switches are read again at call time, so `ros2 param set` can disable a service
  // without a restart.
  std::string port_;
  std::vector<std::string> joint_names_;
  bool auto_enable_ = true;
  int enable_attempts_ = 12;
  double status_rate_hz_ = 10.0;
  bool publish_joint_states_ = false;
  std::string joint_states_topic_ = "/joint_states";
  bool publish_diagnostics_ = true;
  double diagnostics_rate_hz_ = 1.0;
  std::string frame_id_;
  std::string log_dir_;

  std::unique_ptr<litearm::Arm> arm_;

  // Advertised while active; destroyed on deactivate. The typed handles are not needed
  // after creation, so the base-class handle is enough to keep them alive.
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  rclcpp_lifecycle::LifecyclePublisher<litearm_msgs::msg::LitearmStatus>::SharedPtr
    status_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  /// One mutually exclusive group for every SDK call: services and timers never run
  /// concurrently, so two commands can never interleave on the wire.
  rclcpp::CallbackGroup::SharedPtr sdk_group_;

  // Cached state. Nothing here is read from the firmware by a timer.
  bool license_valid_ = false;
  litearm::LicenseInfo license_;
  uint8_t speed_scaling_ = 100;
  litearm::RobotState last_state_;
  bool have_state_ = false;
  /// SDK arrival time of the cached frame, in the SDK's own seconds (monotonic).
  double last_state_stamp_ = 0.0;
};

}  // namespace litearm_driver

#endif  // LITEARM_DRIVER__LITEARM_DRIVER_NODE_HPP_
