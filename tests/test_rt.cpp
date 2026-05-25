// SPDX-License-Identifier: Apache-2.0
//
// Tests for rt.hpp. Most RT operations require privileges (CAP_SYS_NICE,
// CAP_IPC_LOCK, or root). The tests check that:
//   - functions return well-formed statuses regardless of privileges,
//   - InvalidArg is caught before any syscall,
//   - install() with all fields disabled is OK,
//   - install() with all fields enabled returns either OK (privileged
//     run) or PrivilegeDenied (typical CI / non-root run).

#include "doctest/doctest.h"

#include <unistd.h>

#include "mks_servo/rt.hpp"

using mks_servo::rt::Config;
using mks_servo::rt::InstallResults;
using mks_servo::rt::Status;

static bool running_as_root() { return ::geteuid() == 0; }

// ─── argument validation (runs always) ─────────────────────────────
TEST_CASE("rt::set_realtime_priority rejects out-of-range priority") {
    CHECK(mks_servo::rt::set_realtime_priority(0)   == Status::InvalidArg);
    CHECK(mks_servo::rt::set_realtime_priority(100) == Status::InvalidArg);
    CHECK(mks_servo::rt::set_realtime_priority(-5)  == Status::InvalidArg);
}

TEST_CASE("rt::pin_thread_to_core rejects negative core") {
    CHECK(mks_servo::rt::pin_thread_to_core(-1) == Status::InvalidArg);
}

TEST_CASE("rt::install with all-off config is OK") {
    const Config cfg{/*priority=*/-1, /*core=*/-1, /*lock_mem=*/false};
    InstallResults out;
    CHECK(mks_servo::rt::install(cfg, &out) == Status::OK);
    CHECK(out.memory   == Status::OK);
    CHECK(out.priority == Status::OK);
    CHECK(out.affinity == Status::OK);
}

// ─── full install (privilege-dependent outcome) ────────────────────
TEST_CASE("rt::install: full setup returns a well-formed status") {
    const Config cfg{/*priority=*/50, /*core=*/0, /*lock_mem=*/true};
    InstallResults out;
    const auto s = mks_servo::rt::install(cfg, &out);

    if (running_as_root()) {
        CHECK(s == Status::OK);
        CHECK(out.memory   == Status::OK);
        CHECK(out.priority == Status::OK);
        CHECK(out.affinity == Status::OK);
    } else {
        // Non-root user: at least one of these should be PrivilegeDenied.
        // We don't assert WHICH because some kernels permit some ops.
        CHECK((s == Status::OK || s == Status::PrivilegeDenied));
    }
}

// ─── individual helpers don't crash ─────────────────────────────────
TEST_CASE("rt::lock_memory returns OK or PrivilegeDenied") {
    const auto s = mks_servo::rt::lock_memory();
    CHECK((s == Status::OK || s == Status::PrivilegeDenied));
}

TEST_CASE("rt::set_realtime_priority returns OK or PrivilegeDenied for valid prio") {
    const auto s = mks_servo::rt::set_realtime_priority(50);
    CHECK((s == Status::OK || s == Status::PrivilegeDenied));
}

TEST_CASE("rt::pin_thread_to_core: core 0 is always valid; returns OK or PrivilegeDenied") {
    const auto s = mks_servo::rt::pin_thread_to_core(0);
    CHECK((s == Status::OK
        || s == Status::PrivilegeDenied
        || s == Status::OtherError));  // some sandboxes refuse with other errnos
}
