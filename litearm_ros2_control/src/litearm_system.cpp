// litearm_system.cpp — LitearmSystem 实现（系统硬件接口，共享内存桥接）。

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

/** 把守护进程抑制原因码转成可读文本（与 litearm_shm.h 的 LITEARM_DAEMON_* 对应）。 */
const char *daemon_status_text(double code) {
  switch (static_cast<int>(code)) {
    case LITEARM_DAEMON_OK:
      return "正常跟踪命令";
    case LITEARM_DAEMON_CONNECTING:
      return "未连接硬件（启动中、端口未找到或 license 未激活）";
    case LITEARM_DAEMON_HOLDING_STALE_COMMAND:
      return "命令帧陈旧：ROS 侧控制环已停止发布";
    case LITEARM_DAEMON_HOLDING_ESTOP:
      return "软急停中";
    case LITEARM_DAEMON_HOLDING_MOTOR_FAULT:
      return "存在非健康码关节";
    case LITEARM_DAEMON_HOLDING_FEEDBACK_STALE:
      return "关节反馈缺失或超时";
    case LITEARM_DAEMON_HOLDING_OVERTEMP:
      return "电机温度达到软件保护阈值";
    case LITEARM_DAEMON_DISABLED:
      return "已按请求失能（电机失力）";
    case LITEARM_DAEMON_HOLDING_WATCHDOG:
      return "litearm-stm32 固件看门狗曾接管";
    case LITEARM_DAEMON_SHUTTING_DOWN:
      return "守护进程退出中";
    case LITEARM_DAEMON_HOLDING_BAD_COMMAND:
      return "命令帧含非有限数，已拒绝";
    default:
      return "未知状态码";
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

/** 按 URDF 关节名解析出 shm 数组下标：joint1..joint7 → 0..6。 */
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
 * 解析逗号分隔的 7 元 double 列表（如 "400,400,300,300,50,50,50"）。
 *
 * 也接受单个标量（广播到全部关节），方便临时调试。
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
  // 显式取 CLOCK_MONOTONIC：与 Python 的 time.monotonic() 同一时基，
  // 两侧时间戳可直接比较（heartbeat_s / stamp_s / command_age_s）。
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
                 "URDF 声明了 %zu 个关节，litearm 共享内存契约固定为 %d 个"
                 "（joint1..joint7）。请检查 <ros2_control> 的 <joint> 列表。",
                 info_.joints.size(), LITEARM_SHM_NUM_JOINTS);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 建立 URDF 顺序 → shm 下标映射。共享内存里状态/命令数组固定按 joint1..joint7
  // 排布，而 URDF 的 <joint> 顺序由描述文件决定；不做映射会在关节顺序不同时
  // 静默地把 J3 的位置写进 J5 的命令里。
  joint_to_shm_.assign(info_.joints.size(), 0);
  std::vector<bool> seen(LITEARM_SHM_NUM_JOINTS, false);
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    std::size_t index = 0;
    if (!joint_name_to_index(info_.joints[i].name, &index)) {
      RCLCPP_FATAL(logger_,
                   "关节名 '%s' 无法映射到 litearm 轴号（合法范围 joint1..joint%d）。",
                   info_.joints[i].name.c_str(), LITEARM_SHM_NUM_JOINTS);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (seen[index]) {
      RCLCPP_FATAL(logger_, "关节 %s 在 URDF 中重复声明。",
                   info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    seen[index] = true;
    joint_to_shm_[i] = index;
  }

  // ── 参数解析（URDF <hardware><param name="..">value</param>）──
  const auto param = [this](const std::string &key, const std::string &fallback) {
    const auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? fallback : it->second;
  };

  shm_name_ = trim(param("shm_name", LITEARM_SHM_DEFAULT_NAME));
  if (shm_name_.empty() || shm_name_.front() != '/') {
    RCLCPP_FATAL(logger_, "shm_name 必须是以 '/' 开头的 POSIX 共享内存名，当前为 '%s'。",
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
      RCLCPP_WARN(logger_, "参数 %s='%s' 不是合法数字，改用默认值 %g。",
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

  // 默认 MIT 增益：既作为未被 claim 时 kp/kd 命令接口的初值（决定"只挂 JTC
  // 也能位置控制"这件事能不能成立），也是激活瞬间锁位用的刚度。
  default_kp_.fill(0.0);
  default_kd_.fill(0.0);
  const std::string kp_raw = param("default_kp", "400,400,300,300,50,50,50");
  if (!parse_double_list(kp_raw, &default_kp_)) {
    RCLCPP_FATAL(logger_,
                 "default_kp='%s' 非法：需要 7 个逗号分隔的数（或单个标量）。",
                 kp_raw.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string kd_raw = param("default_kd", "5,5,4,5,2.5,2.5,2.5");
  if (!parse_double_list(kd_raw, &default_kd_)) {
    RCLCPP_FATAL(logger_,
                 "default_kd='%s' 非法：需要 7 个逗号分隔的数（或单个标量）。",
                 kd_raw.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 达妙 MIT 帧硬性范围：kp∈[0,500]、kd∈[0,5]。超范围会被电调截断，
  // 与其让上层以为拿到了高刚度，不如在初始化时就钳住并告警。
  for (std::size_t i = 0; i < LITEARM_SHM_NUM_JOINTS; ++i) {
    if (default_kp_[i] < 0.0 || default_kp_[i] > 500.0) {
      RCLCPP_WARN(logger_, "default_kp[%zu]=%g 超出 MIT 范围 [0,500]，已钳位。",
                  i, default_kp_[i]);
    }
    if (default_kd_[i] < 0.0 || default_kd_[i] > 5.0) {
      RCLCPP_WARN(logger_, "default_kd[%zu]=%g 超出 MIT 范围 [0,5]，已钳位。",
                  i, default_kd_[i]);
    }
    default_kp_[i] = std::clamp(default_kp_[i], 0.0, 500.0);
    default_kd_[i] = std::clamp(default_kd_[i], 0.0, 5.0);
  }
  command_kp_ = default_kp_;
  command_kd_ = default_kd_;

  // 命令位置初值取 0；on_activate 会用实测位置覆盖，避免激活瞬间的跳变。
  command_position_.fill(0.0);
  command_velocity_.fill(0.0);
  command_acceleration_.fill(0.0);
  command_effort_.fill(0.0);

  map_ready_ = true;
  RCLCPP_INFO(logger_,
              "litearm 硬件接口初始化完成：shm=%s 关节=%zu 默认 kp/kd="
              "%g/%g(尾部) 诊断接口=%s",
              shm_name_.c_str(), info_.joints.size(), default_kp_.back(),
              default_kd_.back(), export_diagnostics_ ? "开" : "关");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // 段必须已由守护进程创建：create=false。若段不存在说明守护进程没起来，
  // 这时直接报错并给出可操作的提示，比空等到激活再失败要好。
  const int code = litearm_shm_open(shm_name_.c_str(), 0, &shm_);
  if (code != LITEARM_SHM_OK) {
    RCLCPP_FATAL(logger_,
                 "打开共享内存 '%s' 失败（code=%d）。请先启动硬件守护进程："
                 "ros2 run litearm_ros2_control litearm_hw_daemon"
                 "（端口默认按 VID:PID 1d50:606f 自动发现）；"
                 "无硬件调试可加 --dry-run。",
                 shm_name_.c_str(), code);
    shm_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 等守护进程进入"已连接"状态。区分两种失败：段在但没有心跳（守护进程没跑）
  // 与 有心跳但连不上硬件（CAN 未 up / 总线被占 / 电机故障）。
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
      // ⚠ 判据必须包含**心跳新鲜 + 不在退出中**，不能只看 connected 位：
      //   守护进程异常退出后，共享内存里会留下一份陈旧的 &connected=1 的状态块
      //   （它临死前写的那一份）。只看 connected 会把它当成"守护进程就绪"并继续
      //   激活 —— 而那时 read() 返回的是**冻结的关节位置**，控制器却会照着它
      //   发命令。真机踩过：串口没权限 → 守护进程立即退出 → 插件报
      //   「守护进程就绪（状态：守护进程退出中）」并激活。
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
                    "守护进程就绪（dry_run=%s，心跳 %.0fms 前，状态：%s）。",
                    state.dry_run != 0.0 ? "是" : "否", heartbeat_age * 1e3,
                    daemon_status_text(state.last_error));
        return hardware_interface::CallbackReturn::SUCCESS;
      }
      if (state.connected != 0.0 && daemon_shutting_down) {
        // 段里是"上一次那台守护进程"的遗物：它是正常退出的，别再等了。
        // 已经给出具体原因，直接走后面的收尾（别再加一条泛化错误盖住它）。
        RCLCPP_FATAL(logger_,
                     "共享内存 '%s' 里是一份**陈旧**状态（上一次守护进程已正常"
                     "退出，最后状态：%s）。请重新启动硬件守护进程。",
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
                 "守护进程在运行但硬件未连接（%.1fs 超时，最近心跳 %.2fs 前，"
                 "原因：%s）。请检查：板子是否上电且 USB 已连"
                 "（lsusb | grep 1d50:606f）、串口权限（dialout 组）、"
                 "是否已有别的守护进程占用该串口、以及固件 license 是否已激活。",
                 connect_timeout_s_,
                 last_heartbeat > 0.0 ? monotonic_seconds() - last_heartbeat : -1.0,
                 daemon_status_text(last_status));
  } else {
    RCLCPP_FATAL(logger_,
                 "共享内存 '%s' 存在但守护进程无心跳（%.1fs 超时）。"
                 "请确认 litearm_hw_daemon 正在运行，且与本插件用同一个 shm_name。",
                 shm_name_.c_str(), connect_timeout_s_);
  }
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn LitearmSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!map_ready_ || shm_ == nullptr) {
    RCLCPP_ERROR(logger_, "尚未完成初始化/配置，无法激活。");
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 命令位置对齐实测位置：控制器激活后若本周期没写命令接口，
  // MIT 帧的 q_ref 就是当前角——臂原地不动，而不是窜向 0 位。
  latch_command_to_measured();
  mode_ = Mode::kActive;
  publish_command();
  RCLCPP_INFO(logger_,
              "已激活：命令位置已对齐实测位置，kp/kd 使用默认值（未被 override 时）。");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LitearmSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (mode_ == Mode::kActive && shm_ != nullptr) {
    // 交还控制权：发一帧"原位 + 当前 kp/kd"的持位命令。
    // 若不发，命令帧会自然陈旧，守护进程同样会转持位——显式发一帧只是让
    // 交接更平滑（守护进程的 slew 限幅能从正确的位置起锚）。
    latch_command_to_measured();
    publish_command();
    RCLCPP_INFO(logger_, "已停用：已发布最终持位帧（臂保持原位，电机仍使能）。");
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
      // 默认不做：失能会让臂失力下坠。守护进程退出时走的也是 park 持位路径。
      LitearmCommand command{};
      if (litearm_shm_read_command(shm_, &command, configure_read_retries_) ==
          LITEARM_SHM_OK) {
        command.enable = 0.0;
        command.stamp_s = monotonic_seconds();
        command.cycle_count = ++command_cycle_;
        litearm_shm_publish_command(shm_, &command);
        RCLCPP_WARN(logger_,
                    "disable_on_shutdown=true：已请求守护进程失能电机——"
                    "臂将失力，请确保已支撑。");
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
  // ⚠ 别在这里无条件承诺"会自动持位"：如果守护进程根本没起来（most 常见：
  // 串口没权限 / 板子没插），**没有任何进程在持位**，臂是自由的。
  // 原实现统一说"会因命令帧陈旧而自动转入高刚度持位"，在那种场景下是假保证。
  if (daemon_alive_) {
    RCLCPP_ERROR(logger_,
                 "硬件进入错误态。守护进程会因命令帧陈旧而自动转入高刚度持位，"
                 "机械臂保持原位；排查后重新 configure/activate 即可恢复。");
  } else {
    RCLCPP_ERROR(logger_,
                 "硬件进入错误态（**守护进程未就绪**）。⚠ 此刻没有任何进程在"
                 "持位，臂是自由的 —— 若它靠重力而不是靠支撑稳着，请先支撑。"
                 "排查后重新 configure/activate 即可恢复。");
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
    // 非标准接口：joint_state_broadcaster 不会 claim 它们，
    // controller_manager 会在激活时提示"这些接口无人认领"——这是预期的。
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
    // acceleration 只供守护进程做 M(q)·q̈ 前馈；不写它就是 0（该前馈项退化为 C·q̇）。
    interfaces.emplace_back(name, kCommandAcceleration, &command_acceleration_[i]);
    interfaces.emplace_back(name, kCommandEffort, &command_effort_[i]);
    // kp/kd 是"可选的运行时整定"入口：没有控制器写它们时，值保持为
    // default_kp/default_kd（见 on_init），于是纯位置控制开箱可用。
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
    // kp/kd 保持当前值：可能是默认值，也可能是上层整定过的阻抗。
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
    // 极少发生的撕裂：本周期沿用上一帧的值，绝不让单次撕裂把控制器打停。
    ++torn_state_reads_;
    return hardware_interface::return_type::OK;
  }
  if (code != LITEARM_SHM_OK) {
    RCLCPP_ERROR(logger_, "读取共享内存状态失败（code=%d）。", code);
    return hardware_interface::return_type::ERROR;
  }

  state_buffer_ = state;
  have_state_ = true;
  last_heartbeat_s_ = state.heartbeat_s;
  last_applied_cycle_ = state.applied_command_cycle;
  apply_state(state);

  const double now = monotonic_seconds();

  // ── 守护进程存活判定 ──
  // 守护进程一死，状态块就冻结在最后一帧。若不检测，控制器会对着空气
  // 一直规划下去（而臂其实已经停在守护进程退出时的持位姿态）。
  const double heartbeat_age = now - state.heartbeat_s;
  const bool alive = state.heartbeat_s > 0.0 && heartbeat_age <= heartbeat_timeout_s_;
  if (!alive) {
    if (!reported_daemon_loss_) {
      reported_daemon_loss_ = true;
      RCLCPP_ERROR(logger_,
                   "硬件守护进程心跳丢失（最近心跳 %.2fs 前 > 阈值 %.2fs）。"
                   "硬件接口报错退出；机械臂应停在守护进程退出时的持位姿态。",
                   heartbeat_age, heartbeat_timeout_s_);
    }
    daemon_alive_ = false;
    return hardware_interface::return_type::ERROR;
  }
  if (!daemon_alive_) {
    RCLCPP_INFO(logger_, "硬件守护进程心跳恢复。");
    daemon_alive_ = true;
    reported_daemon_loss_ = false;
  }

  // ── 非正常状态的诊断（不阻止控制，但要说清楚）──
  // 这里自己算节流，不用 RCLCPP_*_THROTTLE：那些宏会在调用点持有一个静态
  // rclcpp::Clock，首次使用时才构造/分配 —— 而本函数跑在实时线程里。
  if (state.faulted != 0.0) {
    if (now - last_fault_log_s_ >= 1.0) {
      last_fault_log_s_ = now;
      RCLCPP_ERROR(logger_, "电机存在故障码：%s。",
                   daemon_status_text(state.last_error));
    }
  } else {
    last_fault_log_s_ = 0.0;
  }
  if (state.watchdog_tripped != 0.0 && !reported_watchdog_trip_) {
    reported_watchdog_trip_ = true;
    RCLCPP_WARN(logger_,
                "litearm-stm32 固件的命令看门狗曾因超过 100ms 没收到命令而"
                "接管过（固件转入 fail-soft 持位）。本插件自身不阻塞，若反复出现"
                "请检查 CPU 抢占/其他进程负载，以及守护进程是否被拖慢。");
  }
  if (state.last_error == LITEARM_DAEMON_HOLDING_STALE_COMMAND) {
    // 说明守护进程认为命令陈旧——即本插件超过 command_watchdog_timeout_s
    // 没发布命令，通常意味着 update 周期被严重拖长或控制器被暂停。
    if (now - last_stale_log_s_ >= 2.0) {
      last_stale_log_s_ = now;
      RCLCPP_WARN(logger_, "守护进程认为命令帧陈旧（age=%.3fs），已转入持位。",
                  state.command_age_s);
    }
  } else {
    last_stale_log_s_ = 0.0;
  }

  // 命令链路闭环诊断：命令计数长期不被守护进程应用，说明它一直在持位。
  if (last_applied_cycle_ >= 0.0 && command_cycle_ > 0.0 &&
      state.last_error != LITEARM_DAEMON_OK &&
      state.last_error != LITEARM_DAEMON_HOLDING_STALE_COMMAND) {
    if (!reported_daemon_command_mismatch_) {
      reported_daemon_command_mismatch_ = true;
      RCLCPP_WARN(logger_,
                  "守护进程未跟随命令，原因：%s。机械臂当前处于持位状态。",
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
