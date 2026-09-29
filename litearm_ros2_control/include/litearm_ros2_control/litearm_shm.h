// litearm_shm.h — shared-memory contract between the litearm hardware daemon
// and the ros2_control plugin.
//
// This is the single source of layout truth across languages (C++ plugin /
// Python daemon):
//   * C++ side: include this header directly.
//   * Python side: call the C API of liblitearm_shm.so through ctypes and
//     mirror LitearmState / LitearmCommand with ctypes.Structure (the field
//     order must match byte for byte; test/test_shm_layout.py cross-checks it).
//
// Design notes
// ------------
// 1. All struct members are double (8-byte natural alignment) → no implicit
//    padding, so the layout is unambiguous across languages; offsetof is
//    pinned down by static assertions.
// 2. Each of the two blocks has its own seqlock (single writer / single reader,
//    lock-free):
//      state   block: daemon writes, ROS reads   — RT reader never blocks
//      command block: ROS writes, daemon reads   — RT writer never blocks
//    The reader retry limit is given by the caller; when it is exceeded we
//    return LITEARM_SHM_TORN so the upper layer can choose a fallback strategy.
// 3. The seqlock's acquire/release semantics live entirely on the C++ side (see
//    litearm_shm.cpp); Python only memcpys whole structs in and out and never
//    touches shared memory directly, so memory ordering does not need to be
//    expressed in Python.
// 4. Timestamps are uniformly CLOCK_MONOTONIC seconds (on Linux,
//    Python time.monotonic() shares a time base with C++
//    std::chrono::steady_clock), so the two sides can compare them directly for
//    timeout decisions.

#ifndef LITEARM_ROS2_CONTROL__LITEARM_SHM_H_
#define LITEARM_ROS2_CONTROL__LITEARM_SHM_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of litearm axes (joint1..joint7). */
#define LITEARM_SHM_NUM_JOINTS 7

/** Default shared memory object name (POSIX shm, must start with '/'). */
#define LITEARM_SHM_DEFAULT_NAME "/litearm_hw"

/** Layout magic value 'L''A''R''M'. */
#define LITEARM_SHM_MAGIC 0x4C41524Du

/**
 * Layout version. Any added or removed field must bump this; stale segments are
 * rebuilt.
 */
#define LITEARM_SHM_LAYOUT_VERSION 2u

#define LITEARM_SHM_OK 0
#define LITEARM_SHM_ERR_GENERIC (-1)
#define LITEARM_SHM_ERR_OPEN (-2)
#define LITEARM_SHM_ERR_TRUNCATE (-3)
#define LITEARM_SHM_ERR_MAP (-4)
#define LITEARM_SHM_ERR_LAYOUT (-5)
#define LITEARM_SHM_ERR_VERSION (-6)
#define LITEARM_SHM_ERR_INVALID_ARG (-7)
/**
 * The reader retried max_retries times and still saw tearing; the output keeps
 * its pre-call contents.
 */
#define LITEARM_SHM_TORN 1

/* ─────────────── LitearmState.last_error: daemon suppression codes ─────────────── */
/*
 * The daemon has only two "following modes": TRACKING (follow the shm command
 * frames) and HOLDING (ignore command frames and lock onto the last position
 * with a high-stiffness PD loop). last_error says why it is currently HOLDING —
 * normal tracking reports LITEARM_DAEMON_OK. These codes are for
 * diagnostics/reporting only; they do not affect the seqlock layout.
 */
#define LITEARM_DAEMON_OK 0
/**
 * Not connected to the hardware yet (daemon starting up, port not found, or
 * license not activated).
 */
#define LITEARM_DAEMON_CONNECTING 1
/**
 * Command frames are stale: the ROS control loop stopped publishing (process
 * exited / controller crashed).
 */
#define LITEARM_DAEMON_HOLDING_STALE_COMMAND 2
/** The ROS side requested a soft emergency stop. */
#define LITEARM_DAEMON_HOLDING_ESTOP 3
/** At least one joint has a non-healthy error code. */
#define LITEARM_DAEMON_HOLDING_MOTOR_FAULT 4
/** Joint feedback is missing or has timed out. */
#define LITEARM_DAEMON_HOLDING_FEEDBACK_STALE 5
/** Motor temperature reached the software protection threshold. */
#define LITEARM_DAEMON_HOLDING_OVERTEMP 6
/**
 * The ROS side requested disable (enable=0); the motors lose torque and the arm
 * will sag.
 */
#define LITEARM_DAEMON_DISABLED 7
/**
 * The litearm-stm32 firmware command watchdog took over after 100ms with no
 * command.
 */
#define LITEARM_DAEMON_HOLDING_WATCHDOG 8
/** The daemon is in its shutdown sequence (park / hold position). */
#define LITEARM_DAEMON_SHUTTING_DOWN 9
/**
 * The command frame held non-finite numbers or values outside the MIT
 * representable range; the frame was rejected.
 */
#define LITEARM_DAEMON_HOLDING_BAD_COMMAND 10

/* ───────────────────────── daemon → ROS: state block ───────────────────────── */

/**
 * Joint state plus daemon health. The daemon publishes it once per control
 * cycle.
 *
 * All joint array indices 0..6 correspond to joint1..joint7 (matching the
 * firmware joint frame).
 */
typedef struct LitearmState {
  /**
   * Joint position in rad (joint frame, with direction/zero_offset already
   * applied).
   */
  double position[LITEARM_SHM_NUM_JOINTS];
  /** Joint velocity in rad/s. */
  double velocity[LITEARM_SHM_NUM_JOINTS];
  /**
   * Measured joint torque in Nm (for DM motors this is a current estimate; it
   * includes friction and is fairly noisy).
   */
  double effort[LITEARM_SHM_NUM_JOINTS];
  /** MOS temperature in °C. */
  double temperature_mos[LITEARM_SHM_NUM_JOINTS];
  /** Coil/rotor temperature in °C. */
  double temperature_coil[LITEARM_SHM_NUM_JOINTS];
  /**
   * Raw error code: 0=disabled 1=enabled 9=UV 10=OC 11=MOS_OT 12=COIL_OT;
   * anything else is a fault.
   */
  double error_code[LITEARM_SHM_NUM_JOINTS];
  /**
   * Time since the most recent feedback in s; -1 if none was ever received.
   */
  double feedback_age_s[LITEARM_SHM_NUM_JOINTS];
  /**
   * Cumulative count of feedback frames received (tells whether the feedback link
   * is alive).
   */
  double feedback_received[LITEARM_SHM_NUM_JOINTS];

  /**
   * CLOCK_MONOTONIC instant in s that this state frame corresponds to.
   */
  double stamp_s;
  /**
   * Daemon heartbeat instant in s (same time base as stamp_s; used to tell
   * whether the daemon is alive).
   */
  double heartbeat_s;
  /**
   * 1 when the daemon has successfully connected to the hardware (dry-run
   * included).
   */
  double connected;
  /** 1 when the motors are enabled. */
  double enabled;
  /** 1 when any joint has a non-healthy error code. */
  double faulted;
  /**
   * 1 when the litearm-stm32 firmware watchdog has taken over (no command sent
   * for over 100ms).
   */
  double watchdog_tripped;
  /**
   * 1 when the daemon is running in dry-run (no hardware) mode.
   */
  double dry_run;
  /** Cumulative daemon control cycle count. */
  double cycle_count;
  /**
   * Cycle of the last command the daemon actually applied (lets the ROS side
   * confirm that commands took effect).
   */
  double applied_command_cycle;
  /**
   * Command frame age as observed by the daemon, in s (above the threshold it has
   * entered a watchdog hold).
   */
  double command_age_s;
  /**
   * Most recent error code (LITEARM_SHM_OK means no error).
   */
  double last_error;
} LitearmState;

/* ───────────────────────── ROS → daemon: command block ───────────────────────── */

/**
 * Desired joint state. Published once per ROS control cycle.
 *
 * The semantics are decided by the daemon's **command channel** (see
 * hw_daemon.py):
 *
 * * Default (MOVE_JS position mode): **only position and velocity are
 *   consumed**, and they are mapped onto the firmware's (q_ref, dq_ref). The
 *   PD gains and the gravity/friction/integral/kd_extra feedforward are
 *   computed by the firmware, so the kp/kd/effort/acceleration quantities are
 *   not forwarded on this channel.
 * * --mit-passthrough (MIT_ALL full passthrough): all five quantities are
 *   handed to the firmware as-is, and the firmware runs
 *   tau = kp*(q_ref-q) + kd*(dq_ref-dq) + tau_ff without stacking any
 *   feedforward of its own.
 *
 * The struct itself is channel-independent (the fields are always present), so
 * both channels share one shared-memory contract. That is exactly why the lower
 * layer can be swapped without touching the C++ plugin.
 *
 * acceleration is the channel reserved for a **self-computed feedforward**: a
 * desired acceleration, never a second difference of the measured position.
 * ⚠ The default channel does not use it — MOVE_JS has no acceleration source.
 * The firmware source comment reads, translated: "no acceleration source
 * (trapezoidal limiting, not S-curve): M·ddq is not guessed", so under the
 * default channel M·q̈ and C·q̇ take no part in control.
 */
typedef struct LitearmCommand {
  /** MIT position reference in rad (joint frame). */
  double position[LITEARM_SHM_NUM_JOINTS];
  /** MIT velocity reference in rad/s. */
  double velocity[LITEARM_SHM_NUM_JOINTS];
  /**
   * Desired acceleration in rad/s² (unused on the default channel; forwarded to
   * the self-computed feedforward under --mit-passthrough).
   */
  double acceleration[LITEARM_SHM_NUM_JOINTS];
  /**
   * MIT feedforward torque in Nm (not forwarded on the default channel; sent
   * as-is on the passthrough channel).
   */
  double effort[LITEARM_SHM_NUM_JOINTS];
  /**
   * MIT position gain (valid range [0, 500]; not forwarded on the default
   * channel, which takes it from the firmware parameter table).
   */
  double kp[LITEARM_SHM_NUM_JOINTS];
  /**
   * MIT velocity gain (valid range [0, 5]; not forwarded on the default channel,
   * which takes it from the firmware parameter table).
   */
  double kd[LITEARM_SHM_NUM_JOINTS];

  /**
   * 1 when the ROS side requests enable/keep-enabled; 0 requests disable.
   */
  double enable;
  /**
   * Soft emergency stop: while non-zero the daemon stops following commands and
   * enters a high-stiffness hold, until it is cleared and enable is requested
   * again.
   */
  double estop;
  /**
   * CLOCK_MONOTONIC instant of this command frame in s (the daemon uses it to
   * decide whether the command is stale).
   */
  double stamp_s;
  /**
   * ROS-side publish counter (+1 on every publish; for dropped-frame/liveness
   * diagnostics).
   */
  double cycle_count;
} LitearmCommand;

/* ───────────────────────────── header and handle ───────────────────────────── */

/**
 * Segment header info, for diagnostics and readiness checks.
 */
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

/**
 * Opaque handle (actually points at the internal mmap context).
 */
typedef void *litearm_shm_handle_t;

/* ───────────────────────────── C API ───────────────────────────── */

/**
 * Byte size of LitearmState (used by the Python side to check the layout).
 */
size_t litearm_shm_state_size(void);
/** Byte size of LitearmCommand. */
size_t litearm_shm_command_size(void);
/** Byte size of LitearmHeader. */
size_t litearm_shm_header_size(void);
/** Total size of the shared memory segment. */
size_t litearm_shm_segment_size(void);

/**
 * Open (and optionally create) the shared memory segment.
 *
 * With create != 0: create and initialize the segment if it does not exist; if
 * it exists but the magic/version does not match (stale segment), unlink and
 * rebuild it. With create == 0: return LITEARM_SHM_ERR_LAYOUT if the segment
 * does not exist.
 *
 * Returns LITEARM_SHM_OK and writes out the handle on success; a negative error
 * code on failure.
 */
int litearm_shm_open(const char *name, int create, litearm_shm_handle_t *out);

/**
 * Unmap and close the handle (idempotent; clearing the handle is the caller's
 * job).
 */
void litearm_shm_close(litearm_shm_handle_t handle);

/**
 * Unlink the shared memory object (call it after all users have exited; returns
 * LITEARM_SHM_OK if it does not exist).
 */
int litearm_shm_unlink(const char *name);

/**
 * Publish state (daemon side, single writer).
 *
 * Internally: set the seqlock odd → memcpy → release fence → set the seqlock
 * even.
 */
int litearm_shm_publish_state(litearm_shm_handle_t handle, const LitearmState *state);

/**
 * Read state (ROS side, single reader).
 *
 * max_retries < 0 means retry forever (an RT control loop should prefer 0 or a
 * small number of retries, then fall back).
 * Returns LITEARM_SHM_OK on success; LITEARM_SHM_TORN when the retries are
 * exhausted (*out is left unmodified).
 */
int litearm_shm_read_state(litearm_shm_handle_t handle, LitearmState *out,
                           int max_retries);

/** Publish a command (ROS side, single writer). */
int litearm_shm_publish_command(litearm_shm_handle_t handle,
                                const LitearmCommand *command);

/**
 * Read a command (daemon side, single reader). Same semantics as
 * litearm_shm_read_state.
 */
int litearm_shm_read_command(litearm_shm_handle_t handle, LitearmCommand *out,
                             int max_retries);

/**
 * Read the header (lock-free snapshot, for diagnostics/readiness checks only;
 * no consistency guarantee).
 */
int litearm_shm_read_header(litearm_shm_handle_t handle, LitearmHeader *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LITEARM_ROS2_CONTROL__LITEARM_SHM_H_
