// litearm_shm.h — litearm 硬件守护进程 ⇄ ros2_control 插件的共享内存契约。
//
// 这是跨语言（C++ 插件 / Python 守护进程）唯一的布局真相：
//   * C++ 侧：直接 include 本头文件。
//   * Python 侧：通过 ctypes 调用 liblitearm_shm.so 的 C API，并用 ctypes.Structure
//     镜像 LitearmState / LitearmCommand（字段顺序必须逐字节一致，
//     test/test_shm_layout.py 会做交叉校验）。
//
// 设计要点
// --------
// 1. 结构体成员全部为 double（8 字节自然对齐）→ 无隐式 padding，
//    跨语言布局无歧义；offsetof 由静态断言锁死。
// 2. 两块数据各用一个 seqlock（单写者 / 单读者，无锁）：
//      state   块：守护进程写，ROS 读  —— RT 读侧永不阻塞
//      command 块：ROS 写，守护进程读  —— RT 写侧永不阻塞
//    读者重试上限由调用方给出，超限返回 LITEARM_SHM_TORN 让上层决定降级策略。
// 3. seqlock 的 acquire/release 语义完全在 C++ 侧实现（见 litearm_shm.cpp），
//    Python 只做“整块结构体进出”的 memcpy，不直接触碰共享内存，
//    因此不需要在 Python 里表达内存序。
// 4. 时间戳统一用 CLOCK_MONOTONIC 秒（Linux 下 Python time.monotonic() 与
//    C++ std::chrono::steady_clock 同源），两侧可直接比较做超时判定。

#ifndef LITEARM_ROS2_CONTROL__LITEARM_SHM_H_
#define LITEARM_ROS2_CONTROL__LITEARM_SHM_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** litearm 轴数（joint1..joint7）。 */
#define LITEARM_SHM_NUM_JOINTS 7

/** 共享内存对象默认名（POSIX shm，需以 '/' 开头）。 */
#define LITEARM_SHM_DEFAULT_NAME "/litearm_hw"

/** 布局魔法值 'L''A''R''M'。 */
#define LITEARM_SHM_MAGIC 0x4C41524Du

/** 布局版本。任何字段增删都必须递增，旧段会被重建。 */
#define LITEARM_SHM_LAYOUT_VERSION 2u

#define LITEARM_SHM_OK 0
#define LITEARM_SHM_ERR_GENERIC (-1)
#define LITEARM_SHM_ERR_OPEN (-2)
#define LITEARM_SHM_ERR_TRUNCATE (-3)
#define LITEARM_SHM_ERR_MAP (-4)
#define LITEARM_SHM_ERR_LAYOUT (-5)
#define LITEARM_SHM_ERR_VERSION (-6)
#define LITEARM_SHM_ERR_INVALID_ARG (-7)
/** 读侧重试 max_retries 次仍检测到撕裂；输出保持调用前内容。 */
#define LITEARM_SHM_TORN 1

/* ─────────────── LitearmState.last_error：守护进程抑制原因码 ─────────────── */
/*
 * 守护进程只有两种“跟随模式”：TRACKING（跟随 shm 命令帧）与 HOLDING（忽略命令帧、
 * 以高刚度 PD 锁在最后位置）。last_error 说明当前为何处于 HOLDING —— 正常跟踪为
 * LITEARM_DAEMON_OK。这些码只用于诊断/上报，不影响 seqlock 布局。
 */
#define LITEARM_DAEMON_OK 0
/** 尚未连接硬件（守护进程启动中、端口没找到或 license 未激活）。 */
#define LITEARM_DAEMON_CONNECTING 1
/** 命令帧陈旧：ROS 侧控制环已停止发布（进程退出/控制器崩了）。 */
#define LITEARM_DAEMON_HOLDING_STALE_COMMAND 2
/** ROS 侧请求软急停。 */
#define LITEARM_DAEMON_HOLDING_ESTOP 3
/** 存在非健康码关节。 */
#define LITEARM_DAEMON_HOLDING_MOTOR_FAULT 4
/** 关节反馈缺失或超时。 */
#define LITEARM_DAEMON_HOLDING_FEEDBACK_STALE 5
/** 电机温度达到软件保护阈值。 */
#define LITEARM_DAEMON_HOLDING_OVERTEMP 6
/** ROS 侧请求失能（enable=0）；此时电机失力，臂会下坠。 */
#define LITEARM_DAEMON_DISABLED 7
/** litearm-stm32 固件的命令看门狗曾因 100ms 无命令而接管。 */
#define LITEARM_DAEMON_HOLDING_WATCHDOG 8
/** 守护进程正在退出流程中（park 持位）。 */
#define LITEARM_DAEMON_SHUTTING_DOWN 9
/** 命令帧含非有限数或超出 MIT 可表示范围，已拒绝本帧。 */
#define LITEARM_DAEMON_HOLDING_BAD_COMMAND 10

/* ───────────────────────── 守护进程 → ROS：状态块 ───────────────────────── */

/**
 * 关节状态 + 守护进程健康度。守护进程每个控制周期发布一次。
 *
 * 所有关节数组下标 0..6 对应 joint1..joint7（与固件 joint frame 一致）。
 */
typedef struct LitearmState {
  /** 关节位置 rad（joint frame，已含 direction/zero_offset 换算）。 */
  double position[LITEARM_SHM_NUM_JOINTS];
  /** 关节速度 rad/s。 */
  double velocity[LITEARM_SHM_NUM_JOINTS];
  /** 实测关节力矩 Nm（DM 电机为电流估计值，含摩擦、噪声较大）。 */
  double effort[LITEARM_SHM_NUM_JOINTS];
  /** MOS 温度 °C。 */
  double temperature_mos[LITEARM_SHM_NUM_JOINTS];
  /** 线圈/转子温度 °C。 */
  double temperature_coil[LITEARM_SHM_NUM_JOINTS];
  /** 原始健康码：0=失能 1=使能 9=UV 10=OC 11=MOS_OT 12=COIL_OT，其余为故障。 */
  double error_code[LITEARM_SHM_NUM_JOINTS];
  /** 距最近一次反馈的时长 s；从未收到为 -1。 */
  double feedback_age_s[LITEARM_SHM_NUM_JOINTS];
  /** 累计收到的反馈帧数（判断反馈链路是否活着）。 */
  double feedback_received[LITEARM_SHM_NUM_JOINTS];

  /** 本帧状态对应的 CLOCK_MONOTONIC 时刻 s。
   *  ⚠ 是**帧的到达时刻**，不是守护进程写出这一刻。守护进程以 250Hz 发布，
   *  而固件只以 100Hz 上报 ⇒ 同一帧会被发布 2~3 次，那几次的 stamp_s 相同。
   *  想判断"守护进程是否还在跑"用 heartbeat_s，不要用这个字段。 */
  double stamp_s;
  /** 守护进程心跳时刻 s（写出这一刻的 CLOCK_MONOTONIC，与 stamp_s **不同源**：
   *  它每拍都前进，用于判定守护进程是否活着）。 */
  double heartbeat_s;
  /** 守护进程已成功连接硬件（含 dry-run）为 1。 */
  double connected;
  /** 电机处于使能状态为 1。 */
  double enabled;
  /** 存在非健康码关节为 1。 */
  double faulted;
  /** litearm-stm32 固件看门狗已接管为 1（命令超过 100ms 未下发）。 */
  double watchdog_tripped;
  /** 守护进程运行在 dry-run（无硬件）模式为 1。 */
  double dry_run;
  /** 守护进程控制周期累计计数。 */
  double cycle_count;
  /** 守护进程最近一次实际下发的命令 cycle（用于 ROS 侧确认命令生效）。 */
  double applied_command_cycle;
  /** 守护进程观测到的命令帧龄 s（> 阈值时已进入 watchdog 持位）。 */
  double command_age_s;
  /** 最近一次错误码（LITEARM_SHM_OK 表示无错误）。 */
  double last_error;
} LitearmState;

/* ───────────────────────── ROS → 守护进程：命令块 ───────────────────────── */

/**
 * 期望关节状态。ROS 侧每个控制周期发布一次。
 *
 * 语义由守护进程的**命令通道**决定（见 hw_daemon.py）：
 *
 * * 默认（MOVE_JS 位置模式）：**只消费 position 与 velocity**，把它们映射到
 *   固件的 (q_ref, dq_ref)。PD 增益与重力/摩擦/积分/kd_extra 前馈由固件算，
 *   所以 kp/kd/effort/acceleration 这四组量在这条通道上不被转发。
 * * --mit-passthrough（MIT_ALL 全透传）：五组量全部原样交给固件，
 *   固件执行 tau = kp*(q_ref-q) + kd*(dq_ref-dq) + tau_ff、不叠任何自家前馈。
 *
 * 结构体本身与通道无关（字段恒在），这样两条通道共用一份共享内存契约。
 * 这正是"换底层不动 C++ 插件"的原因。
 *
 * acceleration 是给**自算前馈**留的通道：期望加速度，绝不对实测位置二次差分。
 * ⚠ 默认通道不用它——MOVE_JS 没有加速度源，固件源码注释原文是
 * "无加速度源(梯形限幅, 非 S 曲线): M·ddq 不猜"，故 M·q̈ 与 C·q̇ 在默认通道下
 * 不参与控制。
 */
typedef struct LitearmCommand {
  /** MIT 位置参考 rad（joint frame）。 */
  double position[LITEARM_SHM_NUM_JOINTS];
  /** MIT 速度参考 rad/s。 */
  double velocity[LITEARM_SHM_NUM_JOINTS];
  /** 期望加速度 rad/s²（默认通道不用；--mit-passthrough 下透传给自算前馈）。 */
  double acceleration[LITEARM_SHM_NUM_JOINTS];
  /** MIT 前馈力矩 Nm（默认通道不转发；透传通道下原样下发）。 */
  double effort[LITEARM_SHM_NUM_JOINTS];
  /** MIT 位置增益（有效范围 [0, 500]；默认通道不转发，取固件参数表）。 */
  double kp[LITEARM_SHM_NUM_JOINTS];
  /** MIT 速度增益（有效范围 [0, 5]；默认通道不转发，取固件参数表）。 */
  double kd[LITEARM_SHM_NUM_JOINTS];

  /** ROS 侧请求使能/保持使能为 1；请求失能为 0。 */
  double enable;
  /** 软急停：非 0 时守护进程停止跟随命令并进入高刚度持位，直至清零且重新使能。 */
  double estop;
  /** 本帧命令的 CLOCK_MONOTONIC 时刻 s（守护进程据此判定命令是否陈旧）。 */
  double stamp_s;
  /** ROS 侧发布计数（每发布一次 +1，用于丢帧/存活诊断）。 */
  double cycle_count;
} LitearmCommand;

/* ───────────────────────────── 头部与句柄 ───────────────────────────── */

/** 段头部信息，供诊断与就绪判定。 */
typedef struct LitearmHeader {
  uint32_t magic;
  uint32_t layout_version;
  uint64_t state_seq;
  uint64_t command_seq;
  uint64_t state_publish_count;
  uint64_t command_publish_count;
  uint64_t state_torn_reads;
  uint64_t command_torn_reads;
} LitearmHeader;

/** 不透明句柄（实际指向内部 mmap 上下文）。 */
typedef void *litearm_shm_handle_t;

/* ───────────────────────────── C API ───────────────────────────── */

/** LitearmState 的字节大小（Python 侧校验布局用）。 */
size_t litearm_shm_state_size(void);
/** LitearmCommand 的字节大小。 */
size_t litearm_shm_command_size(void);
/** LitearmHeader 的字节大小。 */
size_t litearm_shm_header_size(void);
/** 共享内存段总大小。 */
size_t litearm_shm_segment_size(void);

/**
 * 打开（可选创建）共享内存段。
 *
 * create != 0 时：段不存在则创建并初始化；已存在但 magic/版本不匹配（陈旧段）
 * 则删除重建。create == 0 时：段不存在返回 LITEARM_SHM_ERR_LAYOUT。
 *
 * 成功返回 LITEARM_SHM_OK 并写出句柄；失败返回负错误码。
 */
int litearm_shm_open(const char *name, int create, litearm_shm_handle_t *out);

/** 解除映射并关闭句柄（幂等，句柄置空由调用方负责）。 */
void litearm_shm_close(litearm_shm_handle_t handle);

/** 删除共享内存对象（所有使用者退出后调用；不存在返回 LITEARM_SHM_OK）。 */
int litearm_shm_unlink(const char *name);

/**
 * 发布状态（守护进程侧，单写者）。
 *
 * 内部：seqlock 置奇数 → memcpy → release fence → seqlock 置偶数。
 */
int litearm_shm_publish_state(litearm_shm_handle_t handle, const LitearmState *state);

/**
 * 读取状态（ROS 侧，单读者）。
 *
 * max_retries < 0 表示无限重试（RT 控制环推荐 0～少量重试后走降级）。
 * 成功返回 LITEARM_SHM_OK；重试耗尽返回 LITEARM_SHM_TORN（*out 未被修改）。
 */
int litearm_shm_read_state(litearm_shm_handle_t handle, LitearmState *out,
                           int max_retries);

/** 发布命令（ROS 侧，单写者）。 */
int litearm_shm_publish_command(litearm_shm_handle_t handle,
                                const LitearmCommand *command);

/** 读取命令（守护进程侧，单读者）。语义同 litearm_shm_read_state。 */
int litearm_shm_read_command(litearm_shm_handle_t handle, LitearmCommand *out,
                             int max_retries);

/** 读取头部信息（无锁快照，仅用于诊断/就绪判定，不做一致性保证）。 */
int litearm_shm_read_header(litearm_shm_handle_t handle, LitearmHeader *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LITEARM_ROS2_CONTROL__LITEARM_SHM_H_
