// litearm_system.hpp — litearm 的 ros2_control SystemInterface。
//
// 插件不直接碰 USB/CAN：它只读写一段 seqlock 保护的共享内存，由独立的 Python
// 守护进程（litearm_hw_daemon）独占 USB CDC 并与 litearm-stm32 固件交互。
// 因此 read()/write() 是纯 memcpy，实时环里没有 Python、没有串口、没有锁。
//
// 接口映射
// --------
// 六条命令接口**全部导出**，但守护进程只消费其中一部分——具体哪些生效由它的
// 命令通道决定（见 hw_daemon.py）：
//
//   命令接口（每个 joint）           默认通道（MOVE_JS）  --mit-passthrough
//   position                         ✅ q_ref            ✅ q_ref
//   velocity                         ✅ dq_ref           ✅ dq_ref
//   effort                           ❌ 不转发           ✅ tau_ff
//   kp / kd                          ❌ 取固件参数表      ✅ 逐帧 MIT 增益
//   acceleration                     ❌ 不转发           ❌ 固件无加速度通道
//
// 未被子控制器 claim 的接口是惰性的，所以"多导出"不产生副作用；保留它们是为了
// 让回退通道可用。默认栈只 claim position + velocity。
//
// 默认通道下固件内部执行
//   tau = kp·(q_ref − q) + kd·(dq_ref − dq) + G(q) + 摩擦 + ki·∫e + kd_extra·Δdq
// 其中 kp/kd 来自固件参数表，前馈由固件按 ff_mask 叠加。
//
// 状态接口（每个 joint）
//   position, velocity, effort      —— 标准三件套（joint_state_broadcaster 认）
//   temperature_mos, temperature_coil, error_code, feedback_age
//                                   —— 诊断量，见 export_diagnostic_interfaces

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

/** 标准状态接口名（与 hardware_interface 的常量对应）。 */
inline constexpr char kStatePosition[] = "position";
inline constexpr char kStateVelocity[] = "velocity";
inline constexpr char kStateEffort[] = "effort";
/** 诊断状态接口名。 */
inline constexpr char kStateTemperatureMos[] = "temperature_mos";
inline constexpr char kStateTemperatureCoil[] = "temperature_coil";
inline constexpr char kStateErrorCode[] = "error_code";
inline constexpr char kStateFeedbackAge[] = "feedback_age";

/** 命令接口名。 */
inline constexpr char kCommandPosition[] = "position";
inline constexpr char kCommandVelocity[] = "velocity";
inline constexpr char kCommandAcceleration[] = "acceleration";
inline constexpr char kCommandEffort[] = "effort";
inline constexpr char kCommandKp[] = "kp";
inline constexpr char kCommandKd[] = "kd";

/**
 * litearm 七轴硬件，经共享内存桥接到 Python 守护进程。
 *
 * 生命周期
 * --------
 * on_init     解析 URDF 参数与关节名，建立 URDF 顺序 → shm 数组下标 的映射
 * on_configure 以 create=false 打开共享内存段（段必须已由守护进程创建），
 *              并等待守护进程心跳/连接就绪，超时则报错（给出可操作的提示）
 * on_activate  把命令位置初始化为实测位置，发布首帧持位命令（避免激活瞬间跳变）
 * read         从状态块取一帧（seqlock，重试有限）
 * write        把命令接口打包成一帧写回命令块
 * on_deactivate 发布一帧"原位持位"命令后停止
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
  /** 生效的控制模式；决定 read()/write() 是否真正交换数据。 */
  enum class Mode { kUnconfigured, kConfigured, kActive, kStopped };

  /** 打包当前命令接口值为一帧命令并发布。 */
  void publish_command();
  /** 把共享内存里的最新状态拷贝到状态接口存储。 */
  void apply_state(const LitearmState &state);
  /**
   * 用"实测位置 + 当前 kp/kd"填充命令接口，作为一帧原位持位命令。
   *
   * 用于 on_activate（避免激活瞬间跳变）与 on_deactivate（交还控制权时
   * 让守护进程停在原地而不是继续追一个早已过期的设定值）。
   */
  void latch_command_to_measured();
  /** 以毫秒精度读取 CLOCK_MONOTONIC 秒，与 Python time.monotonic() 同源。 */
  static double monotonic_seconds();

  Mode mode_ = Mode::kUnconfigured;
  litearm_shm_handle_t shm_ = nullptr;

  // 关节映射：shm 数组固定按 joint1..joint7 排布，而 URDF 里 <joint> 的顺序
  // 由描述文件决定。joint_to_shm_[urdf_index] = shm 下标。
  std::vector<std::size_t> joint_to_shm_;
  bool map_ready_ = false;

  // 参数（URDF <hardware><param>）。
  std::string shm_name_ = LITEARM_SHM_DEFAULT_NAME;
  double connect_timeout_s_ = 10.0;
  double heartbeat_timeout_s_ = 1.0;
  int state_read_retries_ = 8;
  int configure_read_retries_ = 512;
  bool export_diagnostics_ = true;
  bool disable_on_shutdown_ = false;
  std::array<double, LITEARM_SHM_NUM_JOINTS> default_kp_{};
  std::array<double, LITEARM_SHM_NUM_JOINTS> default_kd_{};

  // 接口存储（按 URDF 顺序）。
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

  // 最近一次读到的状态与守护进程健康度（用于心跳超时判定与诊断）。
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
  // 自管理的日志节流时刻。不用 RCLCPP_*_THROTTLE 是因为它们在调用点持有
  // 静态 rclcpp::Clock，首次使用才构造 —— 而这条路径跑在实时线程里。
  double last_fault_log_s_ = 0.0;
  double last_stale_log_s_ = 0.0;
  std::uint64_t torn_state_reads_ = 0;

  rclcpp::Logger logger_ = rclcpp::get_logger("LitearmSystem");
};

}  // namespace litearm_ros2_control

#endif  // LITEARM_ROS2_CONTROL__LITEARM_SYSTEM_HPP_
