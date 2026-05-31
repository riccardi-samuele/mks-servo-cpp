// SPDX-License-Identifier: Apache-2.0
//
// Compile-only test for envelope.hpp: confirms the header is
// self-contained, compiles cleanly under -Wall -Wextra -Wpedantic
// -Wshadow -Wconversion -Wsign-conversion, and the public surface
// (Envelope struct, auto_calibrate signature) is reachable. No
// runtime behavior — auto_calibrate needs real hardware and is
// validated by examples/hil_auto_calibrate.

#include "doctest/doctest.h"
#include "mks_servo/envelope.hpp"
#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::Envelope;
using mks_servo::Motor;
using mks_servo::RawDriver;
using mks_servo::Transport;

TEST_CASE("envelope.hpp: Envelope default-constructs invalid") {
    Envelope e{};
    CHECK_FALSE(e.valid);
    CHECK(e.recommended_rpm == 0);
    CHECK(e.recommended_acc == 0);
    CHECK(e.soak_n == 0);
}

TEST_CASE("envelope.hpp: auto_calibrate is callable (compile-time check)") {
    // Don't actually call it — needs hardware. Just verify the binding.
    Transport  t;
    RawDriver  raw(t, 1);
    Motor      m(raw);
    using FnPtr = Envelope (*)(Motor&) noexcept;
    FnPtr fn = &mks_servo::auto_calibrate;
    CHECK(fn != nullptr);
}
