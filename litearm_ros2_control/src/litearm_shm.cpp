// litearm_shm.cpp — 共享内存契约实现（seqlock + POSIX shm）。
//
// 内存序说明
// ----------
// 采用内核风格的 seqlock：
//
//   写者：seq++（→奇数）；release fence；写数据；release fence；seq++（→偶数）
//   读者：s0 = seq（acquire）；若 s0 为奇数则重试；读数据；acquire fence；
//         若 seq != s0 则重试（数据可能已撕裂）
//
// acquire/release fence 在弱内存序架构（aarch64 等）上会生成真正的屏障指令，
// 在 x86-64 上退化为编译器屏障 —— 两种情况下语义都正确。
//
// 读者永不阻塞、永不取锁，因此可以安全地放在 ros2_control 的实时线程里；
// 读者重试不会自旋等待写者（写者是非实时的守护进程），只在极小的撕裂窗口内
// 偶尔重试，重试上限由调用方控制。

#include "litearm_ros2_control/litearm_shm.h"

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

// ─────────────────────────── 段内部布局 ───────────────────────────
//
// 与 litearm_shm.h 中的公开结构体一一对应；公开 API 只暴露
// LitearmState / LitearmCommand / LitearmHeader，段内部布局对外不可见。

struct SharedSegment {
  uint32_t magic;
  uint32_t layout_version;
  uint64_t state_seq;             // seqlock 写序号（偶数 = 稳定）
  uint64_t command_seq;           // seqlock 写序号
  uint64_t state_publish_count;   // 诊断：状态发布次数
  uint64_t command_publish_count; // 诊断：命令发布次数
  uint64_t state_torn_reads;      // 诊断：状态读撕裂退避次数
  uint64_t command_torn_reads;    // 诊断：命令读撕裂退避次数
  LitearmState state;
  LitearmCommand command;
};

// 布局锁死：字段全为 double/uint64，自然对齐下不应有任何隐式 padding。
static_assert(sizeof(double) == 8, "double 必须为 8 字节");
static_assert(alignof(LitearmState) == 8, "LitearmState 对齐异常");
static_assert(alignof(LitearmCommand) == 8, "LitearmCommand 对齐异常");
static_assert(sizeof(LitearmState) == 8 * 67, "LitearmState 出现隐式 padding");
static_assert(sizeof(LitearmCommand) == 8 * 46, "LitearmCommand 出现隐式 padding");
// 2 个 uint32（magic + version）恰好打满 8 字节，后接 6 个 uint64。
static_assert(sizeof(LitearmHeader) == 8 * 7, "LitearmHeader 出现隐式 padding");

static_assert(offsetof(SharedSegment, magic) == 0, "布局漂移");
static_assert(offsetof(SharedSegment, layout_version) == 4, "布局漂移");
static_assert(offsetof(SharedSegment, state_seq) == 8, "布局漂移");
static_assert(offsetof(SharedSegment, command_seq) == 16, "布局漂移");
static_assert(offsetof(SharedSegment, state_publish_count) == 24, "布局漂移");
static_assert(offsetof(SharedSegment, command_publish_count) == 32, "布局漂移");
static_assert(offsetof(SharedSegment, state_torn_reads) == 40, "布局漂移");
static_assert(offsetof(SharedSegment, command_torn_reads) == 48, "布局漂移");
static_assert(offsetof(SharedSegment, state) == 56, "布局漂移");
static_assert(offsetof(SharedSegment, command) == 56 + 536, "布局漂移");
static_assert(sizeof(SharedSegment) == 960, "布局漂移");

// seqlock 序号必须能以原子方式无锁访问。
static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "本平台 uint64 原子操作非无锁，seqlock 无法成立");

inline std::atomic<uint64_t> &atom(uint64_t *slot) {
  return *reinterpret_cast<std::atomic<uint64_t> *>(slot);
}

// ─────────────────────────── seqlock 原语 ───────────────────────────

inline void seq_write_begin(std::atomic<uint64_t> &seq) {
  seq.fetch_add(1, std::memory_order_relaxed);          // 进入写临界区（→奇数）
  std::atomic_thread_fence(std::memory_order_release);  // 数据写入不得上浮到此之前
}

inline void seq_write_end(std::atomic<uint64_t> &seq) {
  std::atomic_thread_fence(std::memory_order_release);  // 数据写入必须先于此处可见
  seq.fetch_add(1, std::memory_order_relaxed);          // 离开写临界区（→偶数）
}

inline bool seq_read_begin(std::atomic<uint64_t> &seq, uint64_t *stamp) {
  *stamp = seq.load(std::memory_order_acquire);
  return (*stamp & 1u) == 0u;  // 奇数说明写者正在写
}

inline bool seq_read_retry(std::atomic<uint64_t> &seq, uint64_t stamp) {
  std::atomic_thread_fence(std::memory_order_acquire);  // 数据读取不得下沉到此之后
  return seq.load(std::memory_order_relaxed) != stamp;
}

// 泛化的“读一块受 seqlock 保护的数据”。max_retries < 0 表示无限重试。
// 撕裂重试期间执行 relax 让出流水线，避免与写者抢缓存行。
template <typename T>
int seq_read_block(std::atomic<uint64_t> &seq, std::atomic<uint64_t> &torn_counter,
                   const T *src, T *out, int max_retries) {
  for (int attempt = 0;; ++attempt) {
    uint64_t stamp = 0;
    if (!seq_read_begin(seq, &stamp)) {
      // 写者正在写：不计入 torn，直接重试（不自旋等待，让出调度）。
      if (max_retries >= 0 && attempt >= max_retries) {
        torn_counter.fetch_add(1, std::memory_order_relaxed);
        return LITEARM_SHM_TORN;
      }
      continue;
    }
    std::memcpy(out, src, sizeof(T));
    if (!seq_read_retry(seq, stamp)) {
      return LITEARM_SHM_OK;
    }
    torn_counter.fetch_add(1, std::memory_order_relaxed);
    if (max_retries >= 0 && attempt >= max_retries) {
      return LITEARM_SHM_TORN;
    }
  }
}

template <typename T>
int seq_write_block(std::atomic<uint64_t> &seq, T *dst, const T *src) {
  seq_write_begin(seq);
  std::memcpy(dst, src, sizeof(T));
  seq_write_end(seq);
  return LITEARM_SHM_OK;
}

// ─────────────────────────── 句柄与初始化 ───────────────────────────

struct Handle {
  int fd = -1;
  SharedSegment *segment = nullptr;
  size_t size = 0;
};

inline Handle *as_handle(litearm_shm_handle_t h) { return static_cast<Handle *>(h); }

// 用当前进程身份可读的初始值填充段（不做任何硬件假设）。
void initialize_segment(SharedSegment *segment) {
  std::memset(segment, 0, sizeof(SharedSegment));
  segment->magic = LITEARM_SHM_MAGIC;
  segment->layout_version = LITEARM_SHM_LAYOUT_VERSION;
  // seqlock 起点为偶数（稳定态）。
  atom(&segment->state_seq).store(0, std::memory_order_relaxed);
  atom(&segment->command_seq).store(0, std::memory_order_relaxed);
}

bool layout_matches(const SharedSegment *segment) {
  return segment->magic == LITEARM_SHM_MAGIC &&
         segment->layout_version == LITEARM_SHM_LAYOUT_VERSION;
}

}  // namespace

extern "C" {

size_t litearm_shm_state_size(void) { return sizeof(LitearmState); }
size_t litearm_shm_command_size(void) { return sizeof(LitearmCommand); }
size_t litearm_shm_header_size(void) { return sizeof(LitearmHeader); }
size_t litearm_shm_segment_size(void) { return sizeof(SharedSegment); }

int litearm_shm_open(const char *name, int create, litearm_shm_handle_t *out) {
  if (name == nullptr || out == nullptr) {
    return LITEARM_SHM_ERR_INVALID_ARG;
  }
  *out = nullptr;

  // 已有段以 O_RDWR 打开即可；不存在时（create）才带 O_CREAT。
  // 注意不能直接用 O_CREAT|O_EXCL 判存在性：那会让并发的 create 方互相失败。
  int fd = ::shm_open(name, create ? (O_CREAT | O_RDWR) : O_RDWR, 0666);
  if (fd < 0) {
    return (errno == ENOENT && !create) ? LITEARM_SHM_ERR_LAYOUT
                                        : LITEARM_SHM_ERR_OPEN;
  }

  struct stat st {};
  if (::fstat(fd, &st) != 0) {
    ::close(fd);
    return LITEARM_SHM_ERR_OPEN;
  }

  const bool needs_grow =
      static_cast<size_t>(st.st_size) != sizeof(SharedSegment);
  if (needs_grow) {
    if (!create) {
      ::close(fd);
      return LITEARM_SHM_ERR_LAYOUT;
    }
    if (::ftruncate(fd, static_cast<off_t>(sizeof(SharedSegment))) != 0) {
      ::close(fd);
      return LITEARM_SHM_ERR_TRUNCATE;
    }
  }

  void *base = ::mmap(nullptr, sizeof(SharedSegment), PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
  if (base == MAP_FAILED) {
    ::close(fd);
    return LITEARM_SHM_ERR_MAP;
  }

  auto *segment = static_cast<SharedSegment *>(base);

  const bool fresh =
      needs_grow || st.st_size == 0 || !layout_matches(segment);
  if (fresh) {
    if (!create) {
      ::munmap(base, sizeof(SharedSegment));
      ::close(fd);
      return layout_matches(segment) ? LITEARM_SHM_ERR_GENERIC
                                     : LITEARM_SHM_ERR_VERSION;
    }
    // 陈旧段（上次守护进程异常退出）或版本不符：整体重建。
    // 这会把 seqlock 复位到偶数，避免读者卡在写者的奇数序号上。
    initialize_segment(segment);
  } else if (create) {
    // 段已存在且布局正确，但由 create 方（守护进程）接管：
    // 仍强制复位 seqlock，因为上一次持有者的写临界区可能未闭合。
    initialize_segment(segment);
  }

  auto *handle = new (std::nothrow) Handle();
  if (handle == nullptr) {
    ::munmap(base, sizeof(SharedSegment));
    ::close(fd);
    return LITEARM_SHM_ERR_GENERIC;
  }
  handle->fd = fd;
  handle->segment = segment;
  handle->size = sizeof(SharedSegment);
  *out = handle;
  return LITEARM_SHM_OK;
}

void litearm_shm_close(litearm_shm_handle_t handle) {
  Handle *h = as_handle(handle);
  if (h == nullptr) {
    return;
  }
  if (h->segment != nullptr) {
    ::munmap(h->segment, h->size);
  }
  if (h->fd >= 0) {
    ::close(h->fd);
  }
  delete h;
}

int litearm_shm_unlink(const char *name) {
  if (name == nullptr) {
    return LITEARM_SHM_ERR_INVALID_ARG;
  }
  if (::shm_unlink(name) == 0 || errno == ENOENT) {
    return LITEARM_SHM_OK;
  }
  return LITEARM_SHM_ERR_GENERIC;
}

int litearm_shm_publish_state(litearm_shm_handle_t handle,
                              const LitearmState *state) {
  Handle *h = as_handle(handle);
  if (h == nullptr || state == nullptr) {
    return LITEARM_SHM_ERR_INVALID_ARG;
  }
  seq_write_block(atom(&h->segment->state_seq), &h->segment->state, state);
  atom(&h->segment->state_publish_count)
      .fetch_add(1, std::memory_order_relaxed);
  return LITEARM_SHM_OK;
}

int litearm_shm_read_state(litearm_shm_handle_t handle, LitearmState *out,
                           int max_retries) {
  Handle *h = as_handle(handle);
  if (h == nullptr || out == nullptr) {
    return LITEARM_SHM_ERR_INVALID_ARG;
  }
  return seq_read_block(atom(&h->segment->state_seq),
                        atom(&h->segment->state_torn_reads),
                        &h->segment->state, out, max_retries);
}

int litearm_shm_publish_command(litearm_shm_handle_t handle,
                                const LitearmCommand *command) {
  Handle *h = as_handle(handle);
  if (h == nullptr || command == nullptr) {
    return LITEARM_SHM_ERR_INVALID_ARG;
  }
  seq_write_block(atom(&h->segment->command_seq), &h->segment->command,
                  command);
  atom(&h->segment->command_publish_count)
      .fetch_add(1, std::memory_order_relaxed);
  return LITEARM_SHM_OK;
}

int litearm_shm_read_command(litearm_shm_handle_t handle, LitearmCommand *out,
                             int max_retries) {
  Handle *h = as_handle(handle);
  if (h == nullptr || out == nullptr) {
    return LITEARM_SHM_ERR_INVALID_ARG;
  }
  return seq_read_block(atom(&h->segment->command_seq),
                        atom(&h->segment->command_torn_reads),
                        &h->segment->command, out, max_retries);
}

int litearm_shm_read_header(litearm_shm_handle_t handle, LitearmHeader *out) {
  Handle *h = as_handle(handle);
  if (h == nullptr || out == nullptr) {
    return LITEARM_SHM_ERR_INVALID_ARG;
  }
  SharedSegment *segment = h->segment;
  out->magic = segment->magic;
  out->layout_version = segment->layout_version;
  out->state_seq = atom(&segment->state_seq).load(std::memory_order_relaxed);
  out->command_seq =
      atom(&segment->command_seq).load(std::memory_order_relaxed);
  out->state_publish_count =
      atom(&segment->state_publish_count).load(std::memory_order_relaxed);
  out->command_publish_count =
      atom(&segment->command_publish_count).load(std::memory_order_relaxed);
  out->state_torn_reads =
      atom(&segment->state_torn_reads).load(std::memory_order_relaxed);
  out->command_torn_reads =
      atom(&segment->command_torn_reads).load(std::memory_order_relaxed);
  return LITEARM_SHM_OK;
}

}  // extern "C"
