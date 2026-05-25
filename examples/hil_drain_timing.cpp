// SPDX-License-Identifier: Apache-2.0
//
// HIL experiment: measure when the firmware's "complete" ack arrives
// relative to the moment the encoder reaches the target window. The
// answer tells us the minimum safe settle_drain_ms value for the
// adaptive smart-drain optimization.
//
// Methodology: for many move iterations, dispatch a MOVE with the
// inline "started" ack read normally, then poll the encoder tightly.
// At the instant we declare "in window" (target reached), record T0.
// Then do tight reads with a long timeout to capture incoming bytes
// (which should include the "complete" ack frame). At the moment the
// first byte arrives, record T1. Report (T1 - T0) histogram.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>

#include <poll.h>
#include <unistd.h>

#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::Mechanical;
using mks_servo::Motor;
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
    const int   addr = (argc > 2) ? std::atoi(argv[2]) : 1;
    const int   N    = (argc > 3) ? std::atoi(argv[3]) : 30;

    send_set_baud(dev, 38400, 0x07);
    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) return 1;
    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});
    raw.enable(true);
    if (!m.set_origin().ok()) return 2;

    // Conservative move: 90° at acc=255, rpm=2000 (matches bench).
    const MoveParams mp{2000, 255};
    const std::int32_t QTR = 16384 / 4;  // 90°

    std::vector<int> deltas_us;
    deltas_us.reserve(static_cast<std::size_t>(N));
    int timeouts = 0;
    int directions[2] = {+1, -1};

    for (int i = 0; i < N; ++i) {
        const int dir = directions[i % 2];
        // Dispatch (with ack inline — the started ack gets consumed)
        const auto dis = raw.move_relative_axis(QTR * dir, mp.rpm, mp.acc);
        if (!dis.ok() || !dis.value) {
            std::fprintf(stderr, "iter %d dispatch failed\n", i);
            continue;
        }

        // Tight encoder polling: read until current_value within
        // tolerance of "delta hit" relative to baseline.
        const std::int32_t target_counts = m.angle_to_counts(m.read().value + 90.0 * dir);
        // Simpler: poll until speed ~= 0 OR encoder near target.
        // We use raw encoder via raw_driver to avoid Motor's drain logic
        // (which we want to bypass for this experiment).
        bool reached = false;
        for (int p = 0; p < 200; ++p) {
            auto er = raw.read_encoder_addition();
            if (!er.ok()) break;
            const std::int64_t diff  = er.value - target_counts;
            const std::int64_t adiff = diff < 0 ? -diff : diff;
            if (adiff <= 50) { reached = true; break; }
        }
        if (!reached) {
            std::fprintf(stderr, "iter %d never reached target\n", i);
            continue;
        }

        const std::uint64_t t0 = now_us();

        // Now wait for ANY byte to arrive on the bus. The "complete" ack
        // is 5 bytes; the first one arriving is what we time.
        std::uint8_t buf[16];
        struct pollfd pfd{};
        pfd.fd     = t.fd();
        pfd.events = POLLIN;
        const int pr = ::poll(&pfd, 1, 200);  // 200 ms safety
        if (pr <= 0) {
            ++timeouts;
            continue;
        }
        const std::uint64_t t1 = now_us();
        // Drain whatever's there. Capture the return into a local so
        // glibc's warn_unused_result on read() doesn't fire under
        // -Werror on hardened toolchains.
        const ssize_t drained = ::read(t.fd(), buf, sizeof(buf));
        (void)drained;

        deltas_us.push_back(static_cast<int>(t1 - t0));
        sleep_ms(50);
    }

    if (deltas_us.empty()) {
        std::fprintf(stderr, "no samples — all iterations timed out\n");
        return 3;
    }
    std::sort(deltas_us.begin(), deltas_us.end());
    const int n = static_cast<int>(deltas_us.size());
    auto pct = [&](double p) {
        const int i = std::min(static_cast<int>(static_cast<double>(n) * p / 100.0),
                               n - 1);
        return deltas_us[static_cast<std::size_t>(i)];
    };
    double sum = 0;
    for (int v : deltas_us) sum += v;
    std::printf("\nN=%d  timeouts=%d\n", n, timeouts);
    std::printf("Time from 'encoder in window' to 'first byte of complete ack':\n");
    std::printf("  mean  = %.1f us\n", sum / n);
    std::printf("  min   = %d us\n",   deltas_us.front());
    std::printf("  p25   = %d us\n",   pct(25));
    std::printf("  p50   = %d us\n",   pct(50));
    std::printf("  p75   = %d us\n",   pct(75));
    std::printf("  p95   = %d us\n",   pct(95));
    std::printf("  p99   = %d us\n",   pct(99));
    std::printf("  max   = %d us\n",   deltas_us.back());

    raw.enable(false);
    t.close();
    send_set_baud(dev, 256000, 0x04);
    return 0;
}
