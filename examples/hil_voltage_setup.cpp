// SPDX-License-Identifier: Apache-2.0
//
// Voltage / work_current tuning helper. Sweeps work_current candidates
// at the current supply voltage and reports which one gives full
// firmware-spec RPM tracking. The recommendation is the LOWEST
// work_current that achieves max steady RPM ~= cmd at cmd=2400+ —
// that's the most power-efficient (and coolest) setting for full
// performance.
//
// Why this matters: the firmware's default work_current=1600 mA is
// fine at 12 V but caps the motor at ~1800 RPM at 24 V (see
// docs/setup_guide.md). This tool finds the right value for YOUR
// supply, instead of guessing.
//
// WARNING:
//   * This tool WRITES TO FLASH (SET_WORK_CURRENT cmd 0x83). The new
//     value persists across reboots. The tool restores the previous
//     value at the end of a successful run; if interrupted, you may
//     need to run it again or manually restore.
//   * The motor will rotate freely for several seconds during each
//     sweep step. Make sure the shaft is unloaded or has clearance.
//   * Higher work_current = more heat. The tool tests up to 3000 mA;
//     under continuous high-duty use, monitor motor temperature.
//
// Usage:
//   hil_voltage_setup [device] [addr]
// Defaults: /dev/ttyUSB0  1
//
// Runtime: ~1-2 minutes (3-4 work_current values tested).

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/envelope.hpp"
#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::build_frame;
using mks_servo::Direction;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::RawDriver;
using mks_servo::Transport;
using mks_servo::op::SET_BAUD;

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

// Cap firmware-spec max RPM. If we see real_rpm within 3% of this AND
// of cmd_rpm at high commands, the work_current is sufficient.
constexpr double FIRMWARE_MAX_RPM = 3000.0;
constexpr double GOOD_TRACKING_RATIO = 0.97;  // real/cmd > 0.97 = tracking

struct SweepResult {
    std::uint16_t work_current_ma = 0;
    double max_observed_rpm       = 0;
    double tracking_ratio_at_3000 = 0;  // real_rpm at cmd=3000 / cmd
    bool   reaches_firmware_cap   = false;
};

// Spin motor at cmd_rpm via MOVE_SPEED for plateau_ms, return real rpm
// measured from encoder addition.
static double measure_rpm(RawDriver& raw, std::uint16_t cmd_rpm, int plateau_ms = 800) {
    auto mv = raw.move_speed(cmd_rpm, /*acc=*/255, Direction::CW);
    if (!mv.ok() || !mv.value) return -1;
    sleep_ms(700);  // ramp + initial settle
    auto e0 = raw.read_encoder_addition();
    struct timespec t0, t1;
    ::clock_gettime(CLOCK_MONOTONIC, &t0);
    sleep_ms(plateau_ms);
    auto e1 = raw.read_encoder_addition();
    ::clock_gettime(CLOCK_MONOTONIC, &t1);
    if (!e0.ok() || !e1.ok()) return -1;
    const double dt = static_cast<double>(t1.tv_sec - t0.tv_sec)
                    + static_cast<double>(t1.tv_nsec - t0.tv_nsec) / 1e9;
    double dc = static_cast<double>(e1.value - e0.value);
    if (dc < 0) dc = -dc;
    return (dc / 16384.0) / dt * 60.0;
}

static SweepResult test_one_current(RawDriver& raw, std::uint16_t ma) {
    SweepResult r;
    r.work_current_ma = ma;

    // Set work current
    auto set = raw.set_work_current_ma(ma);
    if (!set.ok() || !set.value) {
        std::printf("  work_current=%4d mA: SET REJECTED (firmware ack 0x%02x)\n",
                    (int)ma, (int)set.value);
        return r;
    }
    sleep_ms(400);  // flash settle

    // Probe at two high commands
    const std::uint16_t probes[] = {2400, 3000};
    for (std::uint16_t cmd : probes) {
        const double real = measure_rpm(raw, cmd);
        if (real > 0) {
            if (real > r.max_observed_rpm) r.max_observed_rpm = real;
            if (cmd == 3000) r.tracking_ratio_at_3000 = real / cmd;
        }
    }
    // Stop motor between candidates
    raw.move_speed(0, 255, Direction::CW);
    sleep_ms(600);

    // Considered "reaches cap" if real_rpm@cmd=3000 is within 3% of 3000
    r.reaches_firmware_cap = r.tracking_ratio_at_3000 >= GOOD_TRACKING_RATIO;

    std::printf("  work_current=%4d mA: max=%4.0f rpm  ratio@3000=%.3f  %s\n",
                (int)ma, r.max_observed_rpm, r.tracking_ratio_at_3000,
                r.reaches_firmware_cap ? "REACHES FIRMWARE CAP"
                                        : "limited by power budget");
    return r;
}

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   addr = (argc > 2) ? std::atoi(argv[2]) : 1;

    int baud = 0;
    {
        Transport t;
        for (int b : {256000, 38400}) {
            if (t.open(dev, b) != Transport::Status::OK) continue;
            RawDriver p(t, static_cast<std::uint8_t>(addr));
            auto e = p.read_encoder_addition();
            if (e.ok()) { baud = b; break; }
            t.close();
        }
    }
    if (!baud) {
        std::fprintf(stderr, "no response on %s\n", dev);
        return 1;
    }
    if (baud != 256000) send_set_baud(dev, baud, 0x07);

    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open 256k failed\n");
        return 2;
    }
    RawDriver raw(t, static_cast<std::uint8_t>(addr));

    auto en = raw.enable(true);
    if (!en.ok() || !en.value) {
        std::fprintf(stderr, "enable failed (transport_ok=%d firmware_ack=%d)\n",
                     (int)en.ok(), (int)en.value);
        return 3;
    }

    std::printf("=== work_current tuning sweep ===\n");
    std::printf("device:           %s @ 256000 baud\n", dev);
    std::printf("firmware max RPM: %.0f (spec)\n", FIRMWARE_MAX_RPM);
    std::printf("tracking thresh:  %.2f * cmd_rpm (= %.0f at cmd=3000)\n\n",
                GOOD_TRACKING_RATIO, GOOD_TRACKING_RATIO * 3000);

    // Candidates in INCREASING order. We stop at the lowest one that
    // reaches firmware cap.
    const std::uint16_t candidates[] = {1000, 1600, 2000, 2500, 3000};
    constexpr std::size_t N = sizeof(candidates)/sizeof(candidates[0]);
    SweepResult results[N];

    std::printf("Sweeping (each step ~15s):\n");
    std::size_t recommendation_idx = N;  // sentinel "none found"
    for (std::size_t i = 0; i < N; ++i) {
        results[i] = test_one_current(raw, candidates[i]);
        if (results[i].reaches_firmware_cap && recommendation_idx == N) {
            recommendation_idx = i;
            // We could stop here (we found the lowest), but keep going
            // for a few more so the user sees the diminishing returns
            // pattern. Stop after one more step.
            if (i + 1 < N) {
                results[i + 1] = test_one_current(raw, candidates[i + 1]);
            }
            break;
        }
    }

    // Restore default (the safest setting for casual use)
    constexpr std::uint16_t SAFE_DEFAULT = 1600;
    std::printf("\n=== restoring work_current to %d mA (factory default) ===\n",
                (int)SAFE_DEFAULT);
    auto rs = raw.set_work_current_ma(SAFE_DEFAULT);
    std::printf("  ack: ok=%d firmware_ack=%d\n",
                (int)rs.ok(), (int)rs.value);
    sleep_ms(300);
    raw.enable(false);
    t.close();

    // Recommendation
    std::printf("\n=== RECOMMENDATION ===\n");
    if (recommendation_idx < N) {
        std::printf("Lowest work_current that reaches firmware cap (%.0f RPM):\n",
                    FIRMWARE_MAX_RPM);
        std::printf("  -> %d mA  (max RPM observed: %.0f, tracking ratio %.3f)\n",
                    (int)candidates[recommendation_idx],
                    results[recommendation_idx].max_observed_rpm,
                    results[recommendation_idx].tracking_ratio_at_3000);
        std::printf("\nTo apply persistently, run:\n");
        std::printf("  raw.set_work_current_ma(%d);\n", (int)candidates[recommendation_idx]);
        std::printf("Or use the board's menu: Set -> Ma -> %d\n",
                    (int)candidates[recommendation_idx]);
        std::printf("\nNote: this tool restored your motor to the factory default\n"
                    "(%d mA). Apply the recommendation manually when ready.\n",
                    (int)SAFE_DEFAULT);
    } else {
        std::printf("No tested work_current reached the firmware cap.\n");
        std::printf("  Highest max RPM observed: %.0f (at %d mA)\n",
            results[N-1].max_observed_rpm,
            (int)candidates[N-1]);
        std::printf("Possible reasons:\n");
        std::printf("  * Supply voltage too low (try 24 V supply)\n");
        std::printf("  * Heavy mechanical load on the shaft\n");
        std::printf("  * Motor has higher inductance than typical (slower current rise)\n");
    }
    return 0;
}
