// SPDX-License-Identifier: Apache-2.0
//
// HIL example: high-level Motor API doing a 90° quarter turn back and forth.
//
// Demonstrates:
//   - opening the bus
//   - constructing a Motor with mechanical parameters
//   - enabling the driver
//   - setting the current position as origin
//   - blocking writes that poll the encoder for completion
//   - reading the achieved angle
//
// Usage: move_quarter_turn [device] [baud] [slave_addr]
// Defaults: /dev/ttyUSB0  38400  1

#include <cstdio>
#include <cstdlib>
#include <cstdint>

#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::Motor;
using mks_servo::Mechanical;
using mks_servo::MoveParams;
using mks_servo::MotorStatusEx;
using mks_servo::RawDriver;
using mks_servo::Transport;

static const char* tstatus(Transport::Status s) {
    switch (s) {
        case Transport::Status::OK:           return "OK";
        case Transport::Status::OpenFailed:   return "OpenFailed";
        case Transport::Status::ConfigFailed: return "ConfigFailed";
        case Transport::Status::WriteFailed:  return "WriteFailed";
        case Transport::Status::ReadTimeout:  return "ReadTimeout";
        case Transport::Status::ReadFailed:   return "ReadFailed";
        case Transport::Status::NotOpen:      return "NotOpen";
        case Transport::Status::InvalidArg:   return "InvalidArg";
    }
    return "?";
}

static const char* mstatus(MotorStatusEx s) {
    switch (s) {
        case MotorStatusEx::OK:             return "OK";
        case MotorStatusEx::TransportError: return "TransportError";
        case MotorStatusEx::ParseError:     return "ParseError";
        case MotorStatusEx::LimitExceeded:  return "LimitExceeded";
        case MotorStatusEx::LimitWarned:    return "LimitWarned";
        case MotorStatusEx::Timeout:        return "Timeout";
        case MotorStatusEx::NotEnabled:     return "NotEnabled";
    }
    return "?";
}

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   baud = (argc > 2) ? std::atoi(argv[2]) : 38400;
    const int   addr = (argc > 3) ? std::atoi(argv[3]) : 1;

    std::printf("opening %s @ %d baud, slave addr %d…\n", dev, baud, addr);

    Transport t;
    const auto open_s = t.open(dev, baud);
    if (open_s != Transport::Status::OK) {
        std::fprintf(stderr, "open: %s\n", tstatus(open_s));
        return 1;
    }

    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor     m(raw, Mechanical{/*gear_ratio=*/1.0, /*offset=*/0});

    // Enable the driver.
    const auto en = raw.enable(true);
    if (!en.ok() || !en.value) {
        std::fprintf(stderr, "enable failed (t=%s)\n", tstatus(en.t_status));
        return 2;
    }

    // Use the current encoder position as 0°.
    const auto origin = m.set_origin();
    if (!origin.ok()) {
        std::fprintf(stderr, "set_origin: %s\n", mstatus(origin.status));
        raw.enable(false);
        return 3;
    }
    std::printf("origin set; first read = %.3f°\n", m.read().value);

    // Move +90, then -90, then back to 0. Block on encoder for each.
    const MoveParams mp{/*rpm=*/600, /*acc=*/200};
    const double targets[] = {90.0, -90.0, 0.0};
    for (double target : targets) {
        std::printf("→ moving to %+.1f°…\n", target);
        const auto r = m.write(target, mp,
                               /*blocking=*/true,
                               /*tolerance_counts=*/32,    // ~0.7°
                               /*timeout_us=*/3'000'000);  // 3 s
        if (!r.ok()) {
            std::fprintf(stderr, "  write failed: %s\n", mstatus(r.status));
            raw.enable(false);
            return 4;
        }
        const auto actual = m.read();
        std::printf("  reached: %.3f°\n", actual.value);
    }

    raw.enable(false);
    std::printf("done.\n");
    return 0;
}
