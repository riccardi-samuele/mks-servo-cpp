// SPDX-License-Identifier: Apache-2.0
//
// Compile-only test for envelope.hpp: confirms the header is
// self-contained, compiles cleanly under -Wall -Wextra -Wpedantic
// -Wshadow -Wconversion -Wsign-conversion, and the public surface
// (Envelope struct, auto_calibrate signature) is reachable. No
// runtime behavior — auto_calibrate needs real hardware and is
// validated by examples/hil_auto_calibrate.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "doctest/doctest.h"
#include "mks_servo/envelope.hpp"
#include "mks_servo/motor.hpp"
#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::auto_calibrate;
using mks_servo::Envelope;
using mks_servo::EnvelopeFileHeader;
using mks_servo::ENVELOPE_MAGIC;
using mks_servo::ENVELOPE_VERSION;
using mks_servo::load_envelope;
using mks_servo::Motor;
using mks_servo::RawDriver;
using mks_servo::save_envelope;
using mks_servo::Transport;

TEST_CASE("envelope.hpp: Envelope default-constructs invalid") {
    Envelope e{};
    CHECK_FALSE(e.valid);
    CHECK(e.recommended_rpm == 0);
    CHECK(e.recommended_acc == 0);
    CHECK(e.soak_n == 0);
}

TEST_CASE("envelope.hpp: auto_calibrate is callable (compile-time check)") {
    Transport  t;
    RawDriver  raw(t, 1);
    Motor      m(raw);
    using FnPtr = Envelope (*)(Motor&) noexcept;
    FnPtr fn = &auto_calibrate;
    CHECK(fn != nullptr);
}

// ─── persistence ───────────────────────────────────────────────────

// Generate a unique-ish tmp path per test run so parallel ctest doesn't
// race. Using tmpnam is fine for tests; not security-sensitive.
static std::string tmp_envelope_path(const char* suffix) {
    char buf[L_tmpnam + 32];
    std::strncpy(buf, "/tmp/mks_env_test_XXXXXX", sizeof(buf));
    int fd = ::mkstemp(buf);
    if (fd >= 0) ::close(fd);
    std::string s = buf;
    s += "_";
    s += suffix;
    return s;
}

TEST_CASE("envelope.hpp: save_envelope / load_envelope roundtrip") {
    Envelope e_in{};
    e_in.valid                = true;
    e_in.comms_latency_us_p50 = 995.0;
    e_in.comms_latency_us_p99 = 1043.0;
    e_in.max_steady_rpm       = 1839;
    e_in.recommended_rpm      = 2000;
    e_in.recommended_acc      = 255;
    e_in.t_90deg_ms_mean      = 41.2;
    e_in.t_90deg_ms_sigma     = 0.45;
    e_in.overshoot_peak_deg   = 2.04;
    e_in.soak_n               = 10;
    e_in.soak_successes       = 9;

    const std::string path = tmp_envelope_path("roundtrip");
    REQUIRE(save_envelope(e_in, path.c_str()));

    Envelope e_out = load_envelope(path.c_str());
    CHECK(e_out.valid);
    CHECK(e_out.comms_latency_us_p50 == doctest::Approx(995.0));
    CHECK(e_out.comms_latency_us_p99 == doctest::Approx(1043.0));
    CHECK(e_out.max_steady_rpm       == 1839);
    CHECK(e_out.recommended_rpm      == 2000);
    CHECK(e_out.recommended_acc      == 255);
    CHECK(e_out.t_90deg_ms_mean      == doctest::Approx(41.2));
    CHECK(e_out.t_90deg_ms_sigma     == doctest::Approx(0.45));
    CHECK(e_out.overshoot_peak_deg   == doctest::Approx(2.04));
    CHECK(e_out.soak_n               == 10);
    CHECK(e_out.soak_successes       == 9);

    std::remove(path.c_str());
}

TEST_CASE("envelope.hpp: load_envelope returns invalid on missing file") {
    Envelope e = load_envelope("/tmp/this_file_does_not_exist_abc123xyz");
    CHECK_FALSE(e.valid);
}

TEST_CASE("envelope.hpp: load_envelope rejects wrong magic") {
    const std::string path = tmp_envelope_path("badmagic");
    std::FILE* f = std::fopen(path.c_str(), "wb");
    REQUIRE(f != nullptr);
    EnvelopeFileHeader h{0xDEADBEEF, ENVELOPE_VERSION,
                         static_cast<std::uint32_t>(sizeof(Envelope)), 0};
    Envelope e{}; e.valid = true;  // would look like a valid envelope if magic passed
    std::fwrite(&h, sizeof(h), 1, f);
    std::fwrite(&e, sizeof(e), 1, f);
    std::fclose(f);

    Envelope out = load_envelope(path.c_str());
    CHECK_FALSE(out.valid);
    std::remove(path.c_str());
}

TEST_CASE("envelope.hpp: load_envelope rejects wrong version") {
    const std::string path = tmp_envelope_path("badver");
    std::FILE* f = std::fopen(path.c_str(), "wb");
    REQUIRE(f != nullptr);
    EnvelopeFileHeader h{ENVELOPE_MAGIC, ENVELOPE_VERSION + 1,
                         static_cast<std::uint32_t>(sizeof(Envelope)), 0};
    Envelope e{}; e.valid = true;
    std::fwrite(&h, sizeof(h), 1, f);
    std::fwrite(&e, sizeof(e), 1, f);
    std::fclose(f);

    Envelope out = load_envelope(path.c_str());
    CHECK_FALSE(out.valid);
    std::remove(path.c_str());
}

TEST_CASE("envelope.hpp: load_envelope rejects size mismatch") {
    const std::string path = tmp_envelope_path("badsize");
    std::FILE* f = std::fopen(path.c_str(), "wb");
    REQUIRE(f != nullptr);
    EnvelopeFileHeader h{ENVELOPE_MAGIC, ENVELOPE_VERSION,
                         static_cast<std::uint32_t>(sizeof(Envelope) + 4), 0};
    Envelope e{}; e.valid = true;
    std::fwrite(&h, sizeof(h), 1, f);
    std::fwrite(&e, sizeof(e), 1, f);
    std::fclose(f);

    Envelope out = load_envelope(path.c_str());
    CHECK_FALSE(out.valid);
    std::remove(path.c_str());
}
