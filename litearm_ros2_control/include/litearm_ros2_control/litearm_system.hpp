// litearm_system.hpp — ros2_control SystemInterface for litearm.
//
// The plugin never touches USB/CAN directly: it only reads and writes one
// seqlock-protected block of shared memory, while a separate Python daemon
// (litearm_hw_daemon) owns the USB CDC device and talks to the litearm-stm32
// firmware. That makes read()/write() a pure memcpy: no Python, no serial port,
// no locks in the real-time loop.
//
// Interface mapping
// -----------------
// All six command interfaces are **exported**, but the daemon consumes only some
// of them — which ones take effect is decided by its command channel (see
// hw_daemon.py):
//
//   command interface (per joint)    default channel (MOVE_JS)  --mit-passthrough
//   position                         ✅ q_ref                   ✅ q_ref
//   velocity                         ✅ dq_ref                  ✅ dq_ref
//   effort                           ❌ not forwarded           ✅ tau_ff
//   kp / kd                          ❌ firmware param table    ✅ per-frame MIT gains
//   acceleration                     ❌ not forwarded           ❌ no accel channel in firmware
//
// Interfaces not claimed by a sub-controller are inert, so exporting extra ones
// has no side effects; they are kept so the fallback channels stay usable. The
// default stack claims only position + velocity.
//
// Under the default channel the firmware runs
//   tau = kp·(q_ref − q) + kd·(dq_ref − dq) + G(q) + friction + ki·∫e + kd_extra·Δdq
// where kp/kd come from the firmware parameter table and the feedforward terms
// are summed by the firmware according to ff_mask.
//
// State interfaces (per joint)
//   position, velocity, effort      —— the standard trio (joint_state_broadcaster)
//   temperature_mos, temperature_coil, error_code, feedback_age
//                                   —— diagnostics, see export_diagnostic_interfaces

#ifndef LITEARM_ROS2_CONTROL__LITEARM_SYSTEM_HPP_
#define LITEARM_ROS2_CONTROL__LITEARM_SYSTEM_HPP_

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "litearm_ros2_control/litearm_shm.h"

namespace litearm_ros2_control {

/**
 * Standard state interface names (matching the hardware_interface constants).
 */
inline constexpr char kStatePosition[] = "position";
inline constexpr char kStateVelocity[] = "velocity";
inline constexpr char kStateEffort[] = "effort";
/** Diagnostic state interface names. */
inline constexpr char kStateTemperatureMos[] = "temperature_mos";
inline constexpr char kStateTemperatureCoil[] = "temperature_coil";
inline constexpr char kStateErrorCode[] = "error_code";
inline constexpr char kStateFeedbackAge[] = "feedback_age";

/** Command interface names. */
inline constexpr char kCommandPosition[] = "position";
inline constexpr char kCommandVelocity[] = "velocity";
inline constexpr char kCommandAcceleration[] = "acceleration";
inline constexpr char kCommandEffort[] = "effort";
inline constexpr char kCommandKp[] = "kp";
inline constexpr char kCommandKd[] = "kd";

/**
 * litearm seven-axis hardware, bridged to the Python daemon through shared
 * memory.
 *
 * Lifecycle
 * ---------
 * on_init       parse the URDF parameters and joint names, build the
 *               URDF order → shm array index mapping
 * on_configure  open the shared memory segment with create=false (the segment
 *               must already have been created by the daemon), wait for the
 *               daemon heartbeat/connection to become ready, and fail with an
 *               actionable message on timeout
 * on_activate   initialize the command positions to the measured positions and
 *               publish a first hold frame (avoids a jump right at activation)
 * read          take one frame from the state block (seqlock, limited retries)
 * write         pack the command interfaces into one frame and write it back to
 *               the command block
 * on_deactivate publish one "hold in place" frame, then stop
 */
class LitearmSystem : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LitearmSystem)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

 private:
  /**
   * Active control mode; decides whether read()/write() actually exchange data.
   */
  enum class Mode { kUnconfigured, kConfigured, kActive, kStopped };

  /**
   * Pack the current command interface values into a command frame and publish
   * them.
   */
  void publish_command();
  /**
   * Copy the latest state from shared memory into the state interface storage.
   */
  void apply_state(const LitearmState &state);
  /**
   * Fill the command interfaces with "measured position + current kp/kd" to form
   * a hold-in-place command frame.
   *
   * Used by on_activate (avoids a jump right at activation) and by on_deactivate
   * (when handing back control, lets the daemon stop where it is instead of
   * continuing to chase a long-expired setpoint).
   */
  void latch_command_to_measured();
  /**
   * Read CLOCK_MONOTONIC in seconds with millisecond resolution; same time base
   * as Python time.monotonic().
   */
  static double monotonic_seconds();

  Mode mode_ = Mode::kUnconfigured;
  litearm_shm_handle_t shm_ = nullptr;

  // Joint mapping: the shm arrays are always laid out as joint1..joint7, while
  // the order of <joint> in the URDF is decided by the description file.
  // joint_to_shm_[urdf_index] = shm index.
  std::vector<std::size_t> joint_to_shm_;
  bool map_ready_ = false;

  // Parameters (URDF <hardware><param>).
  std::string shm_name_ = LITEARM_SHM_DEFAULT_NAME;
  double connect_timeout_s_ = 10.0;
  double heartbeat_timeout_s_ = 1.0;
  int state_read_retries_ = 8;
  int configure_read_retries_ = 512;
  bool export_diagnostics_ = true;
  bool disable_on_shutdown_ = false;
  std::array<double, LITEARM_SHM_NUM_JOINTS> default_kp_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> default_kd_{};

  // Interface storage (in URDF order).
  std::array<double, LITEARM_SHM_NUM_JOINTS> state_position_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> state_velocity_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> state_effort_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> state_temperature_mos_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> state_temperature_coil_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> state_error_code_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> state_feedback_age_{};

  std::array<double, LITEARM_SHM_NUM_JOINTS> command_position_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> command_velocity_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> command_acceleration_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> command_effort_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> command_kp_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> command_kd_{};

  // Most recently read state plus daemon health (used for heartbeat timeout
  // checks and diagnostics).
  LitearmState state_buffer_{};
  bool have_state_ = false;
  double last_heartbeat_s_ = 0.0;
  double command_cycle_ = 0.0;
  double last_applied_cycle_ = -1.0;
  double last_command_stamp_s_ = 0.0;
  bool daemon_alive_ = false;
  bool reported_daemon_loss_ = false;
  bool reported_daemon_command_mismatch_ = false;
  bool reported_watchdog_trip_ = false;
  // Self-managed log throttle timestamps. RCLCPP_*_THROTTLE is not used here
  // because those macros hold a static rclcpp::Clock at the call site that is
  // only constructed on first use — and this path runs on the real-time thread.
  double last_fault_log_s_ = 0.0;
  double last_stale_log_s_ = 0.0;
  std::uint64_t torn_state_reads_ = 0;

  rclcpp::Logger logger_ = rclcpp::get_logger("LitearmSystem");
};

}  // namespace litearm_ros2_control

#endif  // LITEARM_ROS2_CONTROL__LITEARM_SYSTEM_HPP_
