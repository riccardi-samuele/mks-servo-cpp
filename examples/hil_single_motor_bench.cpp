// SPDX-License-Identifier: Apache-2.0
//
// Single-motor characterization benchmark. Brings the motor to an
// optimal configuration (SR_CLOSE work_mode + 256000 baud + calibrated)
// and then runs a set of benchmarks to probe the motor's limits:
//
//   1. Firmware version + identity probe (cmd 0x40)
//   2. t_90deg vs (rpm, acc) sweep with N iterations per combo
//   3. Max sustained RPM via MOVE_SPEED ramp
//   4. Sigma / overshoot at the recommended preset
//
// Usage:
//   hil_single_motor_bench [device] [addr]
// Defaults: /dev/ttyUSB4  1
//
// WARNING: this tool writes to flash (SET_WORK_MODE, SET_BAUD, CALIBRATE
// and may run SET_ZERO_POINT). Shaft must be free to rotate during the
// calibration phase (~6 seconds of unrestricted spin).

#include <algorithm>
#include <cmath>
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

using namespace mks_servo;

static std::uint64_t now_us() {
    struct timespec ts;
    ::clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
         + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
}

static void sleep_ms(int ms) {
    struct timespec ts{ms / 1000, (ms % 1000) * 1'000'000L};
    ::nanosleep(&ts, nullptr);
}

// SET_BAUD must be sent at the *current* baud, since the firmware
// switches AFTER ack'ing the frame.
static bool send_set_baud(const char* dev, int from_baud, std::uint8_t code) {
    Transport t;
    if (t.open(dev, from_baud) != Transport::Status::OK) return false;
    std::uint8_t req[16];
    std::uint8_t data[1] = {code};
    const auto n = build_frame(req, sizeof(req), 1, op::SET_BAUD, data, 1);
    if (t.write_all(req, n) != Transport::Status::OK) return false;
    sleep_ms(300);
    return true;
}

static int detect_baud(const char* dev, int addr) {
    for (int b : {256000, 38400}) {
        Transport t;
        if (t.open(dev, b) != Transport::Status::OK) continue;
        RawDriver p(t, static_cast<std::uint8_t>(addr));
        auto e = p.read_encoder_addition();
        if (e.ok()) return b;
    }
    return 0;
}

// Send raw READ_VERSION (cmd 0x40) and dump up to 16 bytes of response.
// Frame format isn't strictly documented in the V1.0.4 manual but
// V1.0.6+ firmware responds with a hardware code + version string.
static void probe_version(const char* dev, int baud, std::uint8_t addr) {
    Transport t;
    if (t.open(dev, baud) != Transport::Status::OK) {
        std::printf("  version probe: open failed\n");
        return;
    }
    std::uint8_t req[16];
    const auto n = build_frame(req, sizeof(req), addr, 0x40, nullptr, 0);
    t.drain_input();
    if (t.write_all(req, n) != Transport::Status::OK) {
        std::printf("  version probe: write failed\n");
        return;
    }
    std::uint8_t buf[64]{};
    std::size_t got = 0;
    // Read up to 16 bytes within 200 ms; firmware responses are short
    // (typically 6-10 bytes including header).
    for (std::size_t i = 0; i < 16; ++i) {
        const auto s = t.read_exact(buf + got, 1, 50'000);
        if (s != Transport::Status::OK) break;
        ++got;
    }
    if (got == 0) {
        std::printf("  version probe: no response\n");
        return;
    }
    std::printf("  version probe raw (%zu B): ", got);
    for (std::size_t i = 0; i < got; ++i) std::printf("%02X ", buf[i]);
    std::printf("\n");
    // Decode best-effort: skip header FA, addr, cmd, then ASCII chars
    if (got >= 5 && buf[0] == 0xFA) {
        std::printf("  version probe ASCII tail: \"");
        for (std::size_t i = 3; i + 1 < got; ++i) {
            const std::uint8_t c = buf[i];
            if (c >= 0x20 && c < 0x7F) std::printf("%c", static_cast<char>(c));
            else std::printf(".");
        }
        std::printf("\"\n");
    }
}

struct Stats {
    double mean{0}, sigma{0}, min{0}, max{0}, p50{0}, p99{0};
    int n{0};
};
static Stats compute_stats(std::vector<double>& v) {
    Stats s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    double sum = 0;
    for (double x : v) sum += x;
    s.mean = sum / static_cast<double>(v.size());
    double var = 0;
    for (double x : v) var += (x - s.mean) * (x - s.mean);
    s.sigma = std::sqrt(var / static_cast<double>(v.size()));
    s.min = v.front();
    s.max = v.back();
    s.p50 = v[v.size() / 2];
    s.p99 = v[v.size() * 99 / 100];
    s.n = static_cast<int>(v.size());
    return s;
}

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB4";
    const int   addr = (argc > 2) ? std::atoi(argv[2]) : 1;
    bool skip_setup = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--skip-setup") == 0) skip_setup = true;
    }

    std::printf("=== single motor characterization bench ===\n");
    std::printf("device:  %s\n", dev);
    std::printf("addr:    %d\n", addr);
    std::printf("setup:   %s\n\n", skip_setup ? "SKIPPED (bench-only)" : "ENABLED");

    // ── 1. Detect current baud ────────────────────────────────────────
    int cur_baud = detect_baud(dev, addr);
    if (!cur_baud) {
        std::fprintf(stderr, "no response on %s at 256000 or 38400 baud\n", dev);
        return 1;
    }
    std::printf("[1] detected baud: %d\n", cur_baud);

    // ── 2. Probe firmware version ─────────────────────────────────────
    std::printf("[2] firmware version probe\n");
    probe_version(dev, cur_baud, static_cast<std::uint8_t>(addr));

    // ── 3. Raise baud to 256000 if needed ─────────────────────────────
    if (skip_setup && cur_baud == 256000) {
        std::printf("[3] skipping setup (--skip-setup, already at 256000)\n");
    } else if (cur_baud != 256000) {
        std::printf("[3] raising baud %d -> 256000\n", cur_baud);
        if (!send_set_baud(dev, cur_baud, 0x07)) {
            std::fprintf(stderr, "SET_BAUD failed\n");
            return 2;
        }
        sleep_ms(500);
        if (detect_baud(dev, addr) != 256000) {
            std::fprintf(stderr, "baud did not switch to 256000\n");
            return 3;
        }
        std::printf("    baud switch verified\n");
    } else {
        std::printf("[3] already at 256000 baud\n");
    }

    // ── 4. Open at 256k and configure ─────────────────────────────────
    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open 256k failed\n");
        return 4;
    }
    RawDriver raw(t, static_cast<std::uint8_t>(addr));

    if (!skip_setup) {
        std::printf("[4] setting work_mode = SR_CLOSE\n");
        auto wm = raw.set_work_mode(WorkMode::SR_CLOSE);
        if (!wm.ok() || !wm.value) {
            std::fprintf(stderr, "    SET_WORK_MODE failed (ok=%d ack=%d)\n",
                         (int)wm.ok(), (int)wm.value);
            return 5;
        }
        sleep_ms(300);

        // ── 5. Calibrate (cmd 0x80 needs a 0x00 data byte per V1.0.4 manual) ─
        std::printf("[5] calibrating (motor spins freely ~6s)\n");
        {
            std::uint8_t req[16];
            std::uint8_t data[1] = {0x00};
            const auto n = build_frame(req, sizeof(req),
                                       static_cast<std::uint8_t>(addr),
                                       op::CALIBRATE, data, 1);
            t.drain_input();
            if (t.write_all(req, n) != Transport::Status::OK) {
                std::fprintf(stderr, "    CALIBRATE write failed\n");
                return 6;
            }
            // Calibration takes ~5-7 seconds. Wait generously, with
            // periodic drains to clear the firmware's calibration ack
            // frame (which is emitted asynchronously).
            for (int i = 0; i < 12; ++i) {
                sleep_ms(1000);
                t.drain_input();
            }
        }
    } else {
        std::printf("[4-5] skipped (assuming SR_CLOSE + calibrated)\n");
    }

    // ── 6. Enable + sanity reads ──────────────────────────────────────
    auto en = raw.enable(true);
    if (!en.ok() || !en.value) {
        std::fprintf(stderr, "[6] enable failed (ok=%d ack=%d)\n",
                     (int)en.ok(), (int)en.value);
        return 7;
    }
    sleep_ms(300);
    auto sanity = raw.read_encoder_addition();
    if (!sanity.ok()) {
        std::fprintf(stderr, "[6] post-calibrate encoder read failed\n");
        return 8;
    }
    std::printf("[6] enabled. encoder=%lld (%+0.3f deg)\n",
                static_cast<long long>(sanity.value),
                static_cast<double>(sanity.value) * 360.0 / 16384.0);

    Motor m(raw, Mechanical{1.0, 0});
    m.set_origin();
    sleep_ms(100);

    // ── 7. t_90deg sweep ──────────────────────────────────────────────
    //
    // We measure REAL 90 deg moves. Pattern per iteration:
    //   measured: 0° -> +90°   (timed)
    //   untimed:  +90° -> 0°   (return at the same params)
    //
    // Direction is alternated only between iterations of *measured* moves
    // (+90° on even, -90° on odd) so we exercise both rotations equally.
    // Each measurement is always a single-direction 90° swing from rest.
    struct Combo { std::uint16_t rpm; std::uint8_t acc; };
    Combo combos[] = {
        {1000, 200},
        {1500, 255},
        {2000, 255},
        {2500, 255},
        {3000, 255},
    };
    constexpr int N = 20;

    std::printf("\n=== t_90deg sweep (N=%d, single-direction 90° from rest) ===\n", N);
    std::printf("  %5s  %3s   %8s  %8s  %8s  %8s  %8s   %s\n",
                "rpm", "acc", "mean_ms", "sigma_ms", "min_ms", "max_ms", "p99_ms",
                "fails");

    Combo best_combo{0,0};
    double best_mean = 1e9;

    for (auto c : combos) {
        std::vector<double> times;
        int failures = 0;
        // Mirror hil_envelope::ec_do_90deg EXACTLY so numbers are
        // directly comparable to the historical 39.90 ms baseline:
        //   - move_relative_axis(4096 counts = 90 deg) — not absolute
        //   - single tolerance check (no consecutive_in_window)
        //   - settle_drain skipped (raw drain at end is post-timing)
        // Direction alternates to balance bus / motor wear.
        constexpr std::int32_t QTR = 4096;  // 16384 / 4
        // Start with the bus quiet.
        raw.transport_drain_settle(20);
        for (int i = 0; i < N; ++i) {
            const int dir = (i % 2 == 0) ? +1 : -1;
            const std::int32_t delta = QTR * dir;
            auto e0 = raw.read_encoder_addition();
            if (!e0.ok()) { ++failures; sleep_ms(120); continue; }
            const std::int64_t target = e0.value + delta;

            const std::uint64_t t0 = now_us();
            auto disp = raw.move_relative_axis(delta, c.rpm, c.acc);
            if (!disp.ok() || !disp.value) {
                ++failures;
                sleep_ms(120);
                continue;
            }
            // Spin-poll: no sleep, single tolerance check.
            const std::uint64_t deadline = t0 + 2'000'000;
            bool reached = false;
            while (now_us() < deadline) {
                auto e = raw.read_encoder_addition();
                if (!e.ok()) { break; }
                const std::int64_t diff = e.value - target;
                const std::int64_t adiff = diff < 0 ? -diff : diff;
                if (adiff <= 50) { reached = true; break; }
            }
            const std::uint64_t t1 = now_us();
            if (reached) {
                times.push_back(static_cast<double>(t1 - t0) / 1000.0);
            } else {
                ++failures;
            }
            raw.transport_drain_settle(15);   // OUTSIDE timing window
            sleep_ms(120);
        }
        const auto s = compute_stats(times);
        std::printf("  %5u  %3u   %8.2f  %8.3f  %8.2f  %8.2f  %8.2f   %d/%d\n",
                    c.rpm, c.acc,
                    s.mean, s.sigma, s.min, s.max, s.p99,
                    failures, N);
        if (s.n > 0 && s.mean < best_mean) {
            best_mean = s.mean;
            best_combo = c;
        }
    }

    if (best_combo.rpm == 0) {
        std::fprintf(stderr, "no combo completed any move — aborting\n");
        raw.enable(false);
        return 9;
    }

    // ── 8. Recovery: return to angle 0 before max RPM test ────────────
    (void)m.write(0.0, MoveParams{1000, 200}, true, 80, 2'000'000);
    sleep_ms(300);

    // ── 9. Max sustained RPM via MOVE_SPEED ───────────────────────────
    std::printf("\n=== max sustained RPM (MOVE_SPEED + encoder count) ===\n");
    std::printf("  %5s   %8s   %s\n", "cmd", "obs_rpm", "tracking");
    const std::uint16_t cmd_rpms[] = {500, 1000, 1500, 2000, 2500, 3000};
    std::uint16_t last_tracking = 0;
    for (std::uint16_t cmd : cmd_rpms) {
        auto mv = raw.move_speed(cmd, /*acc=*/255, Direction::CW);
        if (!mv.ok() || !mv.value) {
            std::printf("  %5u   %8s   refused\n", cmd, "-");
            continue;
        }
        sleep_ms(700);  // ramp + initial settle
        auto e0 = raw.read_encoder_addition();
        const std::uint64_t t0 = now_us();
        sleep_ms(800);
        auto e1 = raw.read_encoder_addition();
        const std::uint64_t t1 = now_us();
        raw.move_speed(0, 255, Direction::CW);
        sleep_ms(600);
        if (!e0.ok() || !e1.ok()) {
            std::printf("  %5u   %8s   encoder read failed\n", cmd, "-");
            continue;
        }
        const double dt = static_cast<double>(t1 - t0) / 1e6;
        double dc = static_cast<double>(e1.value - e0.value);
        if (dc < 0) dc = -dc;
        const double real_rpm = (dc / 16384.0) / dt * 60.0;
        const double ratio = real_rpm / static_cast<double>(cmd);
        const char* tag = (ratio >= 0.97) ? "OK" :
                          (ratio >= 0.90) ? "marginal" : "STALL";
        std::printf("  %5u   %8.1f   %s (ratio=%.3f)\n",
                    cmd, real_rpm, tag, ratio);
        if (ratio >= 0.97) last_tracking = cmd;
    }
    std::printf("  -> max tracking cmd: %u rpm\n", last_tracking);

    // ── 10. Final summary ─────────────────────────────────────────────
    std::printf("\n=== summary ===\n");
    std::printf("  best 90deg combo:  rpm=%u acc=%u  -> mean=%.2fms\n",
                best_combo.rpm, best_combo.acc, best_mean);
    std::printf("  max tracking rpm:  %u\n", last_tracking);

    raw.move_speed(0, 255, Direction::CW);
    sleep_ms(200);
    raw.enable(false);
    return 0;
}
