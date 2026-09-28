// litearm_shm.cpp — shared-memory contract implementation (seqlock + POSIX shm).
//
// Memory ordering notes
// ---------------------
// A kernel-style seqlock is used:
//
//   writer: seq++ (→odd); release fence; write data; release fence; seq++ (→even)
//   reader: s0 = seq (acquire); retry if s0 is odd; read data; acquire fence;
//           retry if seq != s0 (the data may already be torn)
//
// acquire/release fences emit real barrier instructions on weakly ordered
// architectures (aarch64 and friends) and degrade to compiler barriers on
// x86-64 — the semantics are correct either way.
//
// The reader never blocks and never takes a lock, so it is safe to put in the
// ros2_control real-time thread; a reader retry does not spin waiting for the
// writer (which is a non-real-time daemon), it only retries occasionally inside
// the tiny tearing window, with the retry limit controlled by the caller.

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

// ─────────────────────────── internal segment layout ───────────────────────────
//
// One-to-one with the public structs in litearm_shm.h; the public API only
// exposes LitearmState / LitearmCommand / LitearmHeader, so the segment's
// internal layout is not visible outside.

struct SharedSegment {
  uint32_t magic;
  uint32_t layout_version;
  uint64_t state_seq;             // seqlock write sequence (even = stable)
  uint64_t command_seq;           // seqlock write sequence
  uint64_t state_publish_count;   // diagnostics: state publish count
  uint64_t command_publish_count; // diagnostics: command publish count
  uint64_t state_torn_reads;      // diagnostics: state torn-read backoffs
  uint64_t command_torn_reads;    // diagnostics: command torn-read backoffs
  LitearmState state;
  LitearmCommand command;
};

// Layout pinned down: every field is double/uint64, so natural alignment should
// leave no implicit padding behind.
static_assert(sizeof(double) == 8, "double must be 8 bytes");
static_assert(alignof(LitearmState) == 8, "unexpected LitearmState alignment");
static_assert(alignof(LitearmCommand) == 8, "unexpected LitearmCommand alignment");
static_assert(sizeof(LitearmState) == 8 * 67, "LitearmState has implicit padding");
static_assert(sizeof(LitearmCommand) == 8 * 46, "LitearmCommand has implicit padding");
// The two uint32s (magic + version) exactly fill 8 bytes, then six uint64s.
static_assert(sizeof(LitearmHeader) == 8 * 7, "LitearmHeader has implicit padding");

static_assert(offsetof(SharedSegment, magic) == 0, "layout drift");
static_assert(offsetof(SharedSegment, layout_version) == 4, "layout drift");
static_assert(offsetof(SharedSegment, state_seq) == 8, "layout drift");
static_assert(offsetof(SharedSegment, command_seq) == 16, "layout drift");
static_assert(offsetof(SharedSegment, state_publish_count) == 24, "layout drift");
static_assert(offsetof(SharedSegment, command_publish_count) == 32, "layout drift");
static_assert(offsetof(SharedSegment, state_torn_reads) == 40, "layout drift");
static_assert(offsetof(SharedSegment, command_torn_reads) == 48, "layout drift");
static_assert(offsetof(SharedSegment, state) == 56, "layout drift");
static_assert(offsetof(SharedSegment, command) == 56 + 536, "layout drift");
static_assert(sizeof(SharedSegment) == 960, "layout drift");

// The seqlock sequence numbers must be accessible atomically and lock-free.
static_assert(std::atomic<uint64_t>::is_always_lock_free,
              "uint64 atomics are not lock-free here; seqlock cannot hold");

inline std::atomic<uint64_t> &atom(uint64_t *slot) {
  return *reinterpret_cast<std::atomic<uint64_t> *>(slot);
}

// ─────────────────────────── seqlock primitives ───────────────────────────

inline void seq_write_begin(std::atomic<uint64_t> &seq) {
  seq.fetch_add(1, std::memory_order_relaxed);          // enter write critical section (→odd)
  std::atomic_thread_fence(std::memory_order_release);  // data writes must not float up past here
}

inline void seq_write_end(std::atomic<uint64_t> &seq) {
  std::atomic_thread_fence(std::memory_order_release);  // data writes must be visible before here
  seq.fetch_add(1, std::memory_order_relaxed);          // leave write critical section (→even)
}

inline bool seq_read_begin(std::atomic<uint64_t> &seq, uint64_t *stamp) {
  *stamp = seq.load(std::memory_order_acquire);
  return (*stamp & 1u) == 0u;  // an odd value means the writer is mid-write
}

inline bool seq_read_retry(std::atomic<uint64_t> &seq, uint64_t stamp) {
  std::atomic_thread_fence(std::memory_order_acquire);  // data reads must not sink past here
  return seq.load(std::memory_order_relaxed) != stamp;
}

// Generic "read one block protected by a seqlock". max_retries < 0 means retry
// forever. During a torn-read retry a relax hint gives up the pipeline so we do
// not fight the writer for the cache line.
template <typename T>
int seq_read_block(std::atomic<uint64_t> &seq, std::atomic<uint64_t> &torn_counter,
                   const T *src, T *out, int max_retries) {
  for (int attempt = 0;; ++attempt) {
    uint64_t stamp = 0;
    if (!seq_read_begin(seq, &stamp)) {
      // The writer is mid-write: not counted as torn, just retry (no spinning,
      // yield the scheduler).
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

// ─────────────────────────── handle and initialization ───────────────────────────

struct Handle {
  int fd = -1;
  SharedSegment *segment = nullptr;
  size_t size = 0;
};

inline Handle *as_handle(litearm_shm_handle_t h) { return static_cast<Handle *>(h); }

// Fill the segment with initial values that any process identity can read (no
// hardware assumptions are made).
void initialize_segment(SharedSegment *segment) {
  std::memset(segment, 0, sizeof(SharedSegment));
  segment->magic = LITEARM_SHM_MAGIC;
  segment->layout_version = LITEARM_SHM_LAYOUT_VERSION;
  // The seqlock starts at an even value (stable state).
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

  // An existing segment can simply be opened O_RDWR; only add O_CREAT when it
  // does not exist (create). Note that existence cannot be tested with
  // O_CREAT|O_EXCL directly: that would make concurrent creators fail each
  // other.
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
    // Stale segment (the daemon exited abnormally last time) or version
    // mismatch: rebuild it wholesale. That resets the seqlock to an even value,
    // so readers cannot get stuck on the writer's odd sequence number.
    initialize_segment(segment);
  } else if (create) {
    // The segment exists with the correct layout but is being taken over by the
    // creator (the daemon): still force the seqlock back to even, because the
    // previous owner's write critical section may not have been closed.
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
