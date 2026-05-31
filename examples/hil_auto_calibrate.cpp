// SPDX-License-Identifier: Apache-2.0
//
// HIL example: auto_calibrate() in action. The "library is a toy"
// pattern — on startup, the motor discovers its own limits in a few
// seconds, then user code calls m.write(angle) using the discovered
// recommended params.
//
// What this prints is exactly what env stores: latency, max steady
// RPM, the most aggressive params that pass a mini-soak, observed 90°
// time, overshoot. Persist the resulting Envelope to disk if you want
// instant subsequent startups (the struct is plain-old-data, just
// memcpy it to a file — recharacterise when load/supply/motor changes).
//
// Setup:
//   - Motor enabled-capable (responds to ENABLE with firmware_ack=1)
//   - Shaft FREE or under a load that tolerates ~3 full revolutions
//     during the max-RPM probe
//   - Supply within MKS spec
//
// Usage: hil_auto_calibrate [device] [addr]
// Defaults: /dev/ttyUSB0  1

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "mks_servo/envelope.hpp"
#include "mks_servo/motor.hpp"
#include "mks_servo/protocol.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::auto_calibrate;
using mks_servo::build_frame;
using mks_servo::Envelope;
using mks_servo::Mechanical;
using mks_servo::Motor;
using mks_servo::MoveParams;
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

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   addr = (argc > 2) ? std::atoi(argv[2]) : 1;

    // Probe + normalise to 256k (the lib operating point).
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
    if (!baud) { std::fprintf(stderr, "no response from %s\n", dev); return 1; }
    if (baud != 256000) send_set_baud(dev, baud, 0x07);

    Transport t;
    if (t.open(dev, 256000) != Transport::Status::OK) {
        std::fprintf(stderr, "open at 256k failed\n");
        return 2;
    }
    RawDriver raw(t, static_cast<std::uint8_t>(addr));
    Motor m(raw, Mechanical{1.0, 0});

    auto en = raw.enable(true);
    if (!en.ok() || !en.value) {
        std::fprintf(stderr, "enable failed; aborting\n");
        return 3;
    }
    if (!m.set_origin().ok()) {
        std::fprintf(stderr, "set_origin failed; aborting\n");
        raw.enable(false);
        return 4;
    }

    std::printf("=== auto_calibrate (this takes ~3-6 s)...\n");
    std::fflush(stdout);

    Envelope env = auto_calibrate(m);

    std::printf("\n=== ENVELOPE DISCOVERED ===\n");
    std::printf("valid:                 %s\n", env.valid ? "yes" : "NO");
    std::printf("comms latency:         p50=%.0f us  p99=%.0f us\n",
        env.comms_latency_us_p50, env.comms_latency_us_p99);
    std::printf("max steady RPM:        %d\n", (int)env.max_steady_rpm);
    std::printf("recommended params:    rpm=%d  acc=%d\n",
        (int)env.recommended_rpm, (int)env.recommended_acc);
    std::printf("90deg time:            %.2f ms  (sigma %.2f ms)\n",
        env.t_90deg_ms_mean, env.t_90deg_ms_sigma);
    std::printf("peak overshoot:        %+.2f deg\n", env.overshoot_peak_deg);
    std::printf("mini-soak:             %d/%d ok\n",
        env.soak_successes, env.soak_n);

    if (env.valid) {
        std::printf("\n=== Using discovered params for one extra move ===\n");
        // Enable auto-recovery so a transient firmware refusal (rare
        // but possible right after aggressive testing) doesn't break
        // the demo. This is the recommended pattern for "fire and
        // forget" application code that just wants reliable moves.
        m.set_auto_clear_protection(true);
        if (!m.set_origin().ok()) {
            std::fprintf(stderr, "set_origin failed before demo move\n");
        } else {
            // Use the recommended params: this is the "user code"
            // pattern after auto_calibrate.
            auto wr = m.write(90.0,
                MoveParams{env.recommended_rpm, env.recommended_acc},
                /*blocking=*/true);
            auto pos = m.read();
            std::printf("m.write(90, recommended) -> status=%d  pos=%+.3f deg\n",
                (int)wr.status, pos.ok() ? pos.value : -9999.0);
            // Small inter-move pause: the closed-loop needs ~100ms to
            // fully settle from the overshoot at acc=255 (envelope's
            // overshoot_peak_deg tells you how big the transient is).
            sleep_ms(150);
            auto wr2 = m.write(0.0,
                MoveParams{env.recommended_rpm, env.recommended_acc},
                /*blocking=*/true);
            auto home = m.read();
            std::printf("m.write(0, recommended)  -> status=%d  pos=%+.3f deg\n",
                (int)wr2.status, home.ok() ? home.value : -9999.0);
        }
    }

    raw.enable(false);
    t.close();
    if (baud != 256000) send_set_baud(dev, 256000, 0x04);
    return env.valid ? 0 : 5;
}
