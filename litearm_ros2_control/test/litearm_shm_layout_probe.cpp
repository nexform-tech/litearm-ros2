// litearm_shm_layout_probe.cpp — 把共享内存结构体的 size/offsetof 打印成 JSON。
//
// 用途：Python 侧（test/test_shm_layout.py）用 ctypes 镜像同一批结构体，
// 再把两边的字段偏移逐项对比。ctypes 的 Structure 在加载 .so 时只能校验
// 总大小，字段级偏移漂移（例如 C 侧插了字段、Python 侧忘了同步）不会被
// 总量校验发现——那个空洞由本探针补上。
//
// 输出示例：
//   {
//     "num_joints": 7,
//     "state_size": 536,
//     "state_offsets": {"position": 0, "velocity": 56, ...},
//     ...
//   }

#include <cstddef>
#include <cstdio>

#include "litearm_ros2_control/litearm_shm.h"

#define LITEARM_STATE_FIELDS(X)  \
  X(position)                    \
  X(velocity)                    \
  X(effort)                      \
  X(temperature_mos)             \
  X(temperature_coil)            \
  X(error_code)                  \
  X(feedback_age_s)              \
  X(feedback_received)           \
  X(stamp_s)                     \
  X(heartbeat_s)                 \
  X(connected)                   \
  X(enabled)                     \
  X(faulted)                     \
  X(watchdog_tripped)            \
  X(dry_run)                     \
  X(cycle_count)                 \
  X(applied_command_cycle)       \
  X(command_age_s)               \
  X(last_error)

#define LITEARM_COMMAND_FIELDS(X)  \
  X(position)                      \
  X(velocity)                      \
  X(acceleration)                  \
  X(effort)                        \
  X(kp)                            \
  X(kd)                            \
  X(enable)                        \
  X(estop)                         \
  X(stamp_s)                       \
  X(cycle_count)

#define LITEARM_HEADER_FIELDS(X)  \
  X(magic)                        \
  X(layout_version)               \
  X(state_seq)                    \
  X(command_seq)                  \
  X(state_publish_count)          \
  X(command_publish_count)        \
  X(state_torn_reads)             \
  X(command_torn_reads)

namespace {

void print_state_offsets() {
  std::printf("\"state_offsets\": {");
  bool first = true;
#define EMIT(field)                                                            \
  std::printf("%s\"%s\": %zu", first ? "" : ", ", #field,                      \
              offsetof(LitearmState, field));                                  \
  first = false;
  LITEARM_STATE_FIELDS(EMIT)
#undef EMIT
  std::printf("}");
}

void print_command_offsets() {
  std::printf("\"command_offsets\": {");
  bool first = true;
#define EMIT(field)                                                            \
  std::printf("%s\"%s\": %zu", first ? "" : ", ", #field,                      \
              offsetof(LitearmCommand, field));                                \
  first = false;
  LITEARM_COMMAND_FIELDS(EMIT)
#undef EMIT
  std::printf("}");
}

void print_header_offsets() {
  std::printf("\"header_offsets\": {");
  bool first = true;
#define EMIT(field)                                                            \
  std::printf("%s\"%s\": %zu", first ? "" : ", ", #field,                      \
              offsetof(LitearmHeader, field));                                 \
  first = false;
  LITEARM_HEADER_FIELDS(EMIT)
#undef EMIT
  std::printf("}");
}

}  // namespace

int main() {
  std::printf("{\n");
  std::printf("  \"num_joints\": %d,\n", LITEARM_SHM_NUM_JOINTS);
  std::printf("  \"magic\": %u,\n", LITEARM_SHM_MAGIC);
  std::printf("  \"layout_version\": %u,\n", LITEARM_SHM_LAYOUT_VERSION);
  std::printf("  \"state_size\": %zu,\n", sizeof(LitearmState));
  std::printf("  \"command_size\": %zu,\n", sizeof(LitearmCommand));
  std::printf("  \"header_size\": %zu,\n", sizeof(LitearmHeader));
  std::printf("  \"segment_size\": %zu,\n", litearm_shm_segment_size());
  std::printf("  ");
  print_state_offsets();
  std::printf(",\n  ");
  print_command_offsets();
  std::printf(",\n  ");
  print_header_offsets();
  std::printf("\n}\n");
  return 0;
}
