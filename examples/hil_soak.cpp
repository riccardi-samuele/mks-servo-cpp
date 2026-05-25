// SPDX-License-Identifier: Apache-2.0
//
// Soak test: hammer the motor with alternating quarter-turns until it has
// done N moves or one fails. Reports failure counts and timing stats.
//
// What this catches that small tests don't:
//   - rare comm faults (1-in-a-thousand truncated frames)
//   - encoder drift over many moves
//   - firmware quirks under sustained polling
//   - leaks / state that accumulates
//
// Defaults: 200 moves @ 256k baud + 1 ms polling. Adjust N from CLI.

#include <algorithm>
#include <cstdint>
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
using mks_servo::Mechanical;
using mks_servo::Motor;
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

static bool send_set_baud(const char* dev, int baud, std::uint8_t code) {
    Transport t;
    if (t.open(dev, baud) != Transport::Status::OK) return false;
    std::uint8_t req[16];
    std::uint8_t data[1] = {code};
    const auto n = build_frame(req, sizeof(req), 1, SET_BAUD, data, 1);
    if (t.write_all(req, n) != Transport::Status::OK) return false;
    sleep_ms(200);
    return true;
}

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   N    = (argc > 2) ? std::atoi(argv[2]) : 200;
    const int   addr = (argc > 3) ? std::atoi(argv[3]) : 1;

    std::printf("soak: %d quarter-turns @ 256k baud, 1ms polling\n", N);
    send_set_baud(dev, 38400, 0x07);

    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open failed\n");
        return 1;
    }
    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});

    raw.enable(true);
    if (!m.set_origin().ok()) {
        std::fprintf(stderr, "set_origin failed\n");
        return 2;
    }

    // Use moderate params: at full acc/rpm the unloaded NEMA17 sometimes
    // trips firmware stall protection (transient vibration on an empty
    // shaft looks like a stall). For a soak we want reliability.
    const MoveParams mp{600, 200};

    std::vector<double> sample_ms;
    sample_ms.reserve(static_cast<std::size_t>(N));
    int    failures = 0;
    int    direction = +1;

    const std::uint64_t soak_start = now_us();
    for (int i = 0; i < N; ++i) {
        const auto cur = m.read();
        if (!cur.ok()) {
            std::fprintf(stderr, "move %d: read failed (status=%d)\n",
                         i, static_cast<int>(cur.status));
            ++failures;
            break;
        }

        const double target = cur.value + 90.0 * direction;
        const std::int32_t target_counts = m.angle_to_counts(target);

        const std::uint64_t t0 = now_us();
        const auto d = m.write(target, mp, /*blocking=*/false);
        if (!d.ok()) {
            std::fprintf(stderr, "move %d: dispatch failed (status=%d)\n",
                         i, static_cast<int>(d.status));
            ++failures;
            break;
        }
        const auto w = m.wait_for_position(target_counts,
                                           /*tolerance=*/50,
                                           /*timeout_us=*/2'000'000,
                                           /*consecutive=*/1,
                                           /*poll_interval_us=*/0);
        const std::uint64_t t1 = now_us();
        if (w != MotorStatusEx::OK) {
            std::fprintf(stderr, "move %d: wait failed (status=%d)\n",
                         i, static_cast<int>(w));
            ++failures;
            break;
        }
        sample_ms.push_back(static_cast<double>(t1 - t0) / 1000.0);
        direction = -direction;
        if ((i + 1) % 25 == 0) {
            std::printf("  %d/%d ok, last move %.2f ms\n",
                        i + 1, N, sample_ms.back());
        }
    }
    const std::uint64_t soak_end = now_us();

    raw.enable(false);
    t.close();

    std::printf("\nsoak complete: %zu/%d moves, %d failures, total %.1f s\n",
                static_cast<std::size_t>(sample_ms.size()), N, failures,
                static_cast<double>(soak_end - soak_start) / 1e6);

    if (!sample_ms.empty()) {
        std::sort(sample_ms.begin(), sample_ms.end());
        double sum = 0;
        for (double v : sample_ms) sum += v;
        const double mean = sum / static_cast<double>(sample_ms.size());
        auto pct = [&](double p) {
            const std::size_t i = std::min<std::size_t>(
                static_cast<std::size_t>(static_cast<double>(sample_ms.size()) * p / 100.0),
                sample_ms.size() - 1);
            return sample_ms[i];
        };
        std::printf("per-move ms:  mean=%.2f  min=%.2f  p50=%.2f  p95=%.2f  p99=%.2f  max=%.2f\n",
                    mean, sample_ms.front(), pct(50), pct(95), pct(99), sample_ms.back());
    }

    send_set_baud(dev, 256000, 0x04);
    return failures == 0 ? 0 : 3;
}
