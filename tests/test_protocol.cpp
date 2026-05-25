// SPDX-License-Identifier: Apache-2.0
//
// Protocol-layer tests. Vectors are taken verbatim from the Python
// `mks-servo` test suite (tests/test_protocol.py) so a passing build proves
// the two implementations are byte-for-byte interoperable.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <array>
#include <cstdint>

#include "mks_servo/protocol.hpp"

using mks_servo::build_frame;
using mks_servo::checksum8;
using mks_servo::parse_frame;
using mks_servo::ParseStatus;
using mks_servo::HEAD_DOWN;
using mks_servo::HEAD_UP;

namespace {

// Helper: compare a byte buffer against a literal.
bool bytes_eq(const std::uint8_t* a, std::size_t na,
              std::initializer_list<std::uint8_t> b) {
    if (na != b.size()) return false;
    std::size_t i = 0;
    for (auto v : b) {
        if (a[i++] != v) return false;
    }
    return true;
}

}  // namespace

// ─── checksum ─────────────────────────────────────────────────────
TEST_CASE("checksum: manual §4 calibrate example  FA 01 80 00 -> 0x7B") {
    const std::uint8_t buf[] = {0xFA, 0x01, 0x80, 0x00};
    CHECK(checksum8(buf, 4) == 0x7B);
}

TEST_CASE("checksum: manual §7.3 read_encoder  FA 01 30 -> 0x2B") {
    const std::uint8_t buf[] = {0xFA, 0x01, 0x30};
    CHECK(checksum8(buf, 3) == 0x2B);
}

TEST_CASE("checksum: empty input is 0") {
    CHECK(checksum8(nullptr, 0) == 0);
}

TEST_CASE("checksum: overflow wraps to a byte  FF FF -> 0xFE") {
    const std::uint8_t buf[] = {0xFF, 0xFF};
    CHECK(checksum8(buf, 2) == 0xFE);
}

TEST_CASE("checksum: is constexpr-evaluable") {
    constexpr std::uint8_t buf[] = {0xFA, 0x01, 0x80, 0x00};
    constexpr auto crc = checksum8(buf, 4);
    static_assert(crc == 0x7B, "constexpr checksum must match runtime");
    CHECK(crc == 0x7B);
}

// ─── build_frame ──────────────────────────────────────────────────
TEST_CASE("build_frame: manual §4 calibrate  FA 01 80 00 7B") {
    std::array<std::uint8_t, 16> out{};
    const std::uint8_t data[] = {0x00};
    const auto n = build_frame(out, 0x01, 0x80, data, 1);
    REQUIRE(n == 5);
    CHECK(bytes_eq(out.data(), n, {0xFA, 0x01, 0x80, 0x00, 0x7B}));
}

TEST_CASE("build_frame: no payload  FA 01 30 2B") {
    std::array<std::uint8_t, 16> out{};
    const auto n = build_frame(out, 0x01, 0x30);
    REQUIRE(n == 4);
    CHECK(bytes_eq(out.data(), n, {0xFA, 0x01, 0x30, 0x2B}));
}

TEST_CASE("build_frame: manual §7.4 speed mode  FA 01 F6 01 40 02 34") {
    std::array<std::uint8_t, 16> out{};
    const std::uint8_t data[] = {0x01, 0x40, 0x02};
    const auto n = build_frame(out, 0x01, 0xF6, data, 3);
    REQUIRE(n == 7);
    CHECK(bytes_eq(out.data(), n,
                   {0xFA, 0x01, 0xF6, 0x01, 0x40, 0x02, 0x34}));
}

TEST_CASE("build_frame: returns 0 if output buffer too small") {
    std::array<std::uint8_t, 3> tiny{};
    const std::uint8_t data[] = {0x00};
    const auto n = build_frame(tiny, 0x01, 0x80, data, 1);
    CHECK(n == 0);
}

TEST_CASE("build_frame: pointer overload matches array overload") {
    std::array<std::uint8_t, 16> a{}, b{};
    const std::uint8_t data[] = {0x01, 0x40, 0x02};
    const auto na = build_frame(a.data(), a.size(), 0x01, 0xF6, data, 3);
    const auto nb = build_frame(b, 0x01, 0xF6, data, 3);
    REQUIRE(na == nb);
    for (std::size_t i = 0; i < na; ++i) CHECK(a[i] == b[i]);
}

// ─── parse_frame ──────────────────────────────────────────────────
TEST_CASE("parse_frame: manual §7.3 encoder response") {
    // FB 01 30 FF FF FF FF 22 69 B3
    const std::uint8_t buf[] = {
        0xFB, 0x01, 0x30, 0xFF, 0xFF, 0xFF, 0xFF, 0x22, 0x69, 0xB3
    };
    const auto f = parse_frame(buf, sizeof(buf));
    CHECK(f.status == ParseStatus::OK);
    CHECK(f.addr == 0x01);
    CHECK(f.code == 0x30);
    REQUIRE(f.payload_len == 6);
    CHECK(bytes_eq(f.payload, f.payload_len,
                   {0xFF, 0xFF, 0xFF, 0xFF, 0x22, 0x69}));
}

TEST_CASE("parse_frame: status-only calibrate response  FB 01 80 01 7D") {
    const std::uint8_t buf[] = {0xFB, 0x01, 0x80, 0x01, 0x7D};
    const auto f = parse_frame(buf, sizeof(buf));
    CHECK(f.status == ParseStatus::OK);
    CHECK(f.addr == 0x01);
    CHECK(f.code == 0x80);
    REQUIRE(f.payload_len == 1);
    CHECK(f.payload[0] == 0x01);
}

TEST_CASE("parse_frame: bad checksum is rejected") {
    // Last byte should be 0x7D
    const std::uint8_t bad[] = {0xFB, 0x01, 0x80, 0x01, 0x00};
    const auto f = parse_frame(bad, sizeof(bad));
    CHECK(f.status == ParseStatus::BadChecksum);
}

TEST_CASE("parse_frame: bad head is rejected") {
    const std::uint8_t bad[] = {0xAA, 0x01, 0x80, 0x01, 0x00};
    const auto f = parse_frame(bad, sizeof(bad));
    CHECK(f.status == ParseStatus::BadHead);
}

TEST_CASE("parse_frame: too short is rejected") {
    const std::uint8_t bad[] = {0xFB, 0x01};
    const auto f = parse_frame(bad, sizeof(bad));
    CHECK(f.status == ParseStatus::TooShort);
}

TEST_CASE("round-trip: build then parse yields the same fields") {
    // Build an uplink-shaped frame manually (head=FB), pass through parse.
    std::array<std::uint8_t, 16> out{};
    const std::uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    // build_frame writes HEAD_DOWN; for the test we patch the head to HEAD_UP
    // to simulate the driver's response shape.
    const auto n = build_frame(out, 0x07, 0x31, payload, 4);
    REQUIRE(n == 8);
    out[0] = HEAD_UP;
    // Recompute crc with the new head.
    out[n - 1] = checksum8(out.data(), n - 1);
    const auto f = parse_frame(out.data(), n);
    CHECK(f.status == ParseStatus::OK);
    CHECK(f.addr == 0x07);
    CHECK(f.code == 0x31);
    REQUIRE(f.payload_len == 4);
    CHECK(bytes_eq(f.payload, 4, {0xDE, 0xAD, 0xBE, 0xEF}));
}
