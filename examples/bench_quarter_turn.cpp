// SPDX-License-Identifier: Apache-2.0
//
// Bench: time N quarter-turns at high baud + tight polling, matching the
// methodology of the Python lib's bench_tmin.py. Direct apples-to-apples
// comparison of the same physical motor at the same firmware floor.
//
// Procedure:
//   1. Switch the motor to 256000 baud (firmware cmd 0x8A).
//   2. Set origin.
//   3. Time N alternating-direction quarter-turns at acc=255, rpm=2000.
//      Polling interval is 1 ms (matches Python bench).
//   4. Restore baud to 38400.
//
// Usage: bench_quarter_turn [device] [start_baud] [slave_addr] [N]

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>

#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::Motor;
using mks_servo::Mechanical;
using mks_servo::MotorStatusEx;
using mks_servo::MoveParams;
using mks_servo::RawDriver;
using mks_servo::Transport;
using mks_servo::op::SET_BAUD;

static std::uint64_t now_us() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
         + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
}

static void sleep_ms(int ms) {
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1'000'000L;
    ::nanosleep(&ts, nullptr);
}

// Send SET_BAUD to switch the motor. Tolerate response timeout (motor may
// switch baud mid-response).
static bool send_set_baud(const char* dev, int current_baud, std::uint8_t code) {
    Transport t;
    if (t.open(dev, current_baud) != Transport::Status::OK) return false;
    std::uint8_t req[16];
    std::uint8_t data[1] = {code};
    const auto n = build_frame(req, sizeof(req), 1, SET_BAUD, data, 1);
    if (t.write_all(req, n) != Transport::Status::OK) return false;
    sleep_ms(200);  // let firmware switch
    return true;
}

int main(int argc, char** argv) {
    const char* dev   = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   sbaud = (argc > 2) ? std::atoi(argv[2]) : 38400;
    const int   addr  = (argc > 3) ? std::atoi(argv[3]) : 1;
    const int   N     = (argc > 4) ? std::atoi(argv[4]) : 8;
    const int   target_baud  = 256000;
    const std::uint8_t code_256k = 0x07;
    const std::uint8_t code_38400 = 0x04;

    std::printf("switching motor to %d baud (current = %d)...\n", target_baud, sbaud);
    if (!send_set_baud(dev, sbaud, code_256k)) {
        std::fprintf(stderr, "set_baud send failed\n");
        return 1;
    }

    Transport t;
    if (t.open(dev, target_baud) != Transport::Status::OK) {
        std::fprintf(stderr, "open at %d failed\n", target_baud);
        return 2;
    }

    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});

    if (!raw.enable(true).ok()) {
        std::fprintf(stderr, "enable failed\n");
        return 3;
    }

    if (!m.set_origin().ok()) {
        std::fprintf(stderr, "set_origin failed\n");
        return 4;
    }

    // Alternating quarter-turns, time each one. Use 1 ms poll interval
    // (matches Python bench_tmin.py), since at 256k baud each encoder
    // round-trip is ~1 ms.
    const MoveParams mp{/*rpm=*/2000, /*acc=*/255};

    std::vector<double> samples_ms;
    samples_ms.reserve(static_cast<std::size_t>(N));

    int direction = +1;
    for (int i = 0; i < N; ++i) {
        const auto cur_r = m.read();
        if (!cur_r.ok()) {
            std::fprintf(stderr, "read before move %d failed\n", i);
            break;
        }
        const double cur_angle = cur_r.value;
        const double target_angle = cur_angle + 90.0 * direction;
        const std::int32_t target_counts = m.angle_to_counts(target_angle);

        // Drive write+wait directly so we can pass a custom poll_interval.
        const std::uint64_t t0 = now_us();
        const auto disp = m.write(target_angle, mp, /*blocking=*/false);
        if (!disp.ok()) {
            std::fprintf(stderr, "dispatch %d failed\n", i);
            break;
        }
        // Match the Python bench_tmin.py settings exactly: tolerance=50
        // counts (~1°), consecutive_in_window=1, no inter-poll sleep.
        const auto wait_s = m.wait_for_position(target_counts,
                                                /*tolerance=*/50,
                                                /*timeout_us=*/2'000'000,
                                                /*consecutive=*/1,
                                                /*poll_interval_us=*/0);
        const std::uint64_t t1 = now_us();
        if (wait_s != MotorStatusEx::OK) {
            std::fprintf(stderr, "wait %d failed\n", i);
            break;
        }
        const double elapsed_ms = static_cast<double>(t1 - t0) / 1000.0;
        samples_ms.push_back(elapsed_ms);
        std::printf("  move %2d: %+.1f° -> %+.1f°  %.2f ms\n",
                    i, cur_angle, target_angle, elapsed_ms);
        direction = -direction;
        sleep_ms(50);
    }

    if (!samples_ms.empty()) {
        double sum = 0;
        double mn = samples_ms[0], mx = samples_ms[0];
        for (double v : samples_ms) {
            sum += v;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        const double mean = sum / static_cast<double>(samples_ms.size());
        std::printf("\nstats over %zu moves @ 256k baud, 1ms polling:\n",
                    static_cast<std::size_t>(samples_ms.size()));
        std::printf("  mean = %.2f ms\n", mean);
        std::printf("  min  = %.2f ms\n", mn);
        std::printf("  max  = %.2f ms\n", mx);
    }

    raw.enable(false);
    t.close();

    std::printf("\nrestoring baud to 38400...\n");
    if (!send_set_baud(dev, target_baud, code_38400)) {
        std::fprintf(stderr, "restore set_baud failed; you may need to power-cycle\n");
        return 5;
    }
    Transport t2;
    if (t2.open(dev, 38400) == Transport::Status::OK) {
        RawDriver raw2(t2, static_cast<std::uint8_t>(addr));
        const auto e = raw2.read_encoder_addition();
        if (e.ok()) std::printf("verify @ 38400: encoder=%lld\n",
                                 static_cast<long long>(e.value));
    }
    return 0;
}
