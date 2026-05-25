// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Real-time scheduling helpers.
//
// None of this is required to use the library — the rest of mks_servo works
// fine under the kernel's default SCHED_OTHER policy with paging enabled.
// These helpers exist for callers that DO want sub-ms determinism:
//
//   #include "mks_servo/rt.hpp"
//
//   int main() {
//       mks_servo::rt::lock_memory();
//       mks_servo::rt::set_realtime_priority(80);   // SCHED_FIFO, prio 80
//       mks_servo::rt::pin_thread_to_core(2);
//       // ... the rest of the program now runs RT-safe.
//   }
//
// Or all at once via install():
//
//   mks_servo::rt::install({.priority = 80, .core = 2});
//
// Each call is independent and returns an enum Status — none of them throw.
// All of them require appropriate privileges (CAP_SYS_NICE / CAP_IPC_LOCK
// or root); without them, calls return a privilege-failure status and the
// program continues at default scheduling.
//
// Why this matters for the mks_servo library specifically:
//   - SCHED_OTHER lets the kernel migrate threads across cores and pre-empt
//     them at any time, so cross-motor synchronisation jitter under load
//     can spike to ms-class on a non-RT kernel.
//   - mlockall prevents demand paging from causing 10-50 ms page-fault
//     stalls during a Rubik's solve or similar deterministic workload.
//   - These are 100% optional. The library doesn't internally call any of
//     them — that decision is left to application code.

#ifndef MKS_SERVO_RT_HPP
#define MKS_SERVO_RT_HPP

#include <cstdint>

#if defined(__linux__)
  #include <cerrno>
  #include <pthread.h>
  #include <sched.h>
  #include <sys/mman.h>
  #include <sys/resource.h>
#endif

namespace mks_servo {
namespace rt {

enum class Status : std::uint8_t {
    OK              = 0,
    NotSupported    = 1,  // not Linux, or feature not available
    PrivilegeDenied = 2,  // EPERM / EACCES
    InvalidArg      = 3,
    OtherError      = 4,
};

// Settings bundle for install(). Members default to safe choices; set to
// -1 to skip an individual step.
struct Config {
    int  priority = 80;   // SCHED_FIFO priority 1-99; -1 = don't change
    int  core     = -1;   // CPU core to pin to; -1 = no pinning
    bool lock_mem = true; // mlockall(MCL_CURRENT|MCL_FUTURE)
};

// Lock all current and future pages of the process address space.
// Must be done before allocating long-lived buffers to avoid soft-faults
// later. Requires CAP_IPC_LOCK or RLIMIT_MEMLOCK relaxation.
inline Status lock_memory() noexcept {
#if defined(__linux__)
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) == 0) return Status::OK;
    if (errno == EPERM || errno == EACCES) return Status::PrivilegeDenied;
    return Status::OtherError;
#else
    return Status::NotSupported;
#endif
}

// Switch the calling thread to SCHED_FIFO at `priority` (Linux RT range
// 1-99; 80 is a sensible default for application threads). Higher = more
// preemptive over normal tasks. Requires CAP_SYS_NICE or root.
inline Status set_realtime_priority(int priority) noexcept {
#if defined(__linux__)
    if (priority < 1 || priority > 99) return Status::InvalidArg;
    struct sched_param sp{};
    sp.sched_priority = priority;
    const int rc = ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &sp);
    if (rc == 0)              return Status::OK;
    if (rc == EPERM)          return Status::PrivilegeDenied;
    if (rc == EINVAL)         return Status::InvalidArg;
    return Status::OtherError;
#else
    (void)priority;
    return Status::NotSupported;
#endif
}

// Pin the calling thread to a single CPU core. Stops the scheduler from
// migrating it, which removes cross-core cache-warmup costs from latency
// spikes. Core index 0..N-1.
inline Status pin_thread_to_core(int core) noexcept {
#if defined(__linux__)
    if (core < 0) return Status::InvalidArg;
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<unsigned>(core), &set);
    const int rc = ::pthread_setaffinity_np(::pthread_self(), sizeof(set), &set);
    if (rc == 0)      return Status::OK;
    if (rc == EPERM)  return Status::PrivilegeDenied;
    if (rc == EINVAL) return Status::InvalidArg;
    return Status::OtherError;
#else
    (void)core;
    return Status::NotSupported;
#endif
}

// Apply a full RT setup in one call. Each step is independent: a failure
// in one step doesn't abort the others. Returns the first non-OK status,
// or OK if everything succeeded. Per-step results can be inspected via
// the optional output struct.
struct InstallResults {
    Status memory   = Status::OK;
    Status priority = Status::OK;
    Status affinity = Status::OK;
};

inline Status install(const Config& cfg, InstallResults* out = nullptr) noexcept {
    InstallResults r;
    Status first_failure = Status::OK;
    auto note = [&](Status s) {
        if (s != Status::OK && first_failure == Status::OK) first_failure = s;
    };

    if (cfg.lock_mem) {
        r.memory = lock_memory();
        note(r.memory);
    }
    if (cfg.priority >= 1) {
        r.priority = set_realtime_priority(cfg.priority);
        note(r.priority);
    }
    if (cfg.core >= 0) {
        r.affinity = pin_thread_to_core(cfg.core);
        note(r.affinity);
    }
    if (out) *out = r;
    return first_failure;
}

}  // namespace rt
}  // namespace mks_servo

#endif  // MKS_SERVO_RT_HPP
