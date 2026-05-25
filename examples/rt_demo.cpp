// SPDX-License-Identifier: Apache-2.0
//
// Demonstrates the optional RT helpers. Run without arguments to see what
// the current process can do. Run with sudo to see successful application.
//
// Typical use in a robot's main loop:
//
//   int main() {
//       mks_servo::rt::install({.priority = 80, .core = 2});
//       // ... rest of the program runs RT-safe.
//   }
//
// install() returns the first failure (if any). Individual steps can be
// inspected via the optional InstallResults output struct.

#include <cstdio>
#include <unistd.h>

#include "mks_servo/rt.hpp"

using mks_servo::rt::Config;
using mks_servo::rt::InstallResults;
using mks_servo::rt::Status;

static const char* sstr(Status s) {
    switch (s) {
        case Status::OK:              return "OK";
        case Status::NotSupported:    return "NotSupported";
        case Status::PrivilegeDenied: return "PrivilegeDenied (need root or CAP_*)";
        case Status::InvalidArg:      return "InvalidArg";
        case Status::OtherError:      return "OtherError";
    }
    return "?";
}

int main() {
    std::printf("running as uid=%d (root=%s)\n",
                static_cast<int>(::geteuid()),
                ::geteuid() == 0 ? "yes" : "no");

    const Config cfg{
        /*priority=*/80,
        /*core=*/0,
        /*lock_mem=*/true,
    };

    InstallResults out;
    const auto overall = mks_servo::rt::install(cfg, &out);
    std::printf("install() overall   : %s\n", sstr(overall));
    std::printf("  lock_memory       : %s\n", sstr(out.memory));
    std::printf("  realtime_priority : %s\n", sstr(out.priority));
    std::printf("  pin_to_core       : %s\n", sstr(out.affinity));
    return overall == Status::OK ? 0 : 1;
}
