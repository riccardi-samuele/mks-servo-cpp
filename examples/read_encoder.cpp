// SPDX-License-Identifier: Apache-2.0
//
// HIL example: open the serial port, read the cumulative encoder counts.
//
// Usage:
//   read_encoder [device] [baud] [slave_addr]
// Defaults: /dev/ttyUSB0  38400  1
//
// This program proves end-to-end:
//   - termios2 configuration works at the requested baud
//   - the wire frame is byte-compatible with the firmware
//   - the parsed response gives a sane encoder value
//
// Build (from build/):  cmake --build . --target read_encoder
// Run:                  ./examples/read_encoder /dev/ttyUSB0 38400 1

#include <cstdio>
#include <cstdlib>
#include <string>

#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::RawDriver;
using mks_servo::Transport;

static const char* status_str(Transport::Status s) {
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

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   baud = (argc > 2) ? std::atoi(argv[2]) : 38400;
    const int   addr = (argc > 3) ? std::atoi(argv[3]) : 1;

    std::printf("opening %s @ %d baud, slave addr %d…\n", dev, baud, addr);

    Transport t;
    const auto open_s = t.open(dev, baud);
    if (open_s != Transport::Status::OK) {
        std::fprintf(stderr, "open failed: %s\n", status_str(open_s));
        return 1;
    }

    RawDriver drv(t, static_cast<std::uint8_t>(addr));

    // Read encoder five times to give a confidence sample.
    for (int i = 0; i < 5; ++i) {
        const auto r = drv.read_encoder_addition();
        if (!r.ok()) {
            std::fprintf(stderr,
                "read_encoder_addition failed: t_status=%s parse_status=%d\n",
                status_str(r.t_status),
                static_cast<int>(r.parse_status));
            return 2;
        }
        // 16384 counts per revolution.
        const double deg = static_cast<double>(r.value) * 360.0 / 16384.0;
        std::printf("  read %d: counts=%lld  angle=%+.3f deg\n",
                    i, static_cast<long long>(r.value), deg);
    }

    return 0;
}
