// SPDX-License-Identifier: Apache-2.0
//
// Mock-transport tests for RawDriver.
//
// We don't need real hardware: a socketpair gives us a bidirectional fd pair.
// We hand one end to a Transport and play "the motor" on the other end —
// reading what RawDriver sent and feeding back canned responses.

#include "doctest/doctest.h"

#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::MotorStatus;
using mks_servo::RawDriver;
using mks_servo::Transport;
using mks_servo::WorkMode;

namespace {

// Mock side of socketpair: writes are fire-and-forget. Capturing the
// return value (vs a bare `(void)::write(...)`) satisfies glibc's
// warn_unused_result attribute on hardened toolchains.
inline void mock_write(int fd, const void* buf, std::size_t n) noexcept {
    const ssize_t r = ::write(fd, buf, n);
    (void)r;
}

// Create a socketpair and wrap one end in a Transport. Returns {transport, peer_fd}.
struct PairedTransport {
    Transport transport;
    int       peer_fd = -1;
};

PairedTransport make_paired_transport() {
    int fds[2] = {-1, -1};
    const int rc = ::socketpair(AF_UNIX, SOCK_STREAM, 0, fds);
    REQUIRE(rc == 0);
    return PairedTransport{Transport(fds[0]), fds[1]};
}

// Read exactly n bytes from fd (with a short timeout via select-style alarm).
std::vector<std::uint8_t> read_n(int fd, std::size_t n) {
    std::vector<std::uint8_t> out(n);
    std::size_t got = 0;
    while (got < n) {
        const ssize_t r = ::read(fd, out.data() + got, n - got);
        if (r <= 0) break;
        got += static_cast<std::size_t>(r);
    }
    out.resize(got);
    return out;
}

void write_all(int fd, std::initializer_list<std::uint8_t> bytes) {
    const auto* p = bytes.begin();
    std::size_t left = bytes.size();
    while (left > 0) {
        const ssize_t w = ::write(fd, p, left);
        REQUIRE(w > 0);
        p    += w;
        left -= static_cast<std::size_t>(w);
    }
}

}  // namespace

// ─── read_encoder_addition ─────────────────────────────────────────
TEST_CASE("RawDriver::read_encoder_addition emits correct request and decodes int48 BE") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/100'000);

    // Run the driver call on a thread, since it blocks for the response.
    std::thread t([&] {
        // Wait for the request first.
        auto req = read_n(p.peer_fd, 4);
        REQUIRE(req.size() == 4);
        CHECK(req[0] == 0xFA);  // HEAD_DOWN
        CHECK(req[1] == 0x01);
        CHECK(req[2] == 0x31);  // READ_ENCODER_ADDITION
        CHECK(req[3] == mks_servo::checksum8(req.data(), 3));

        // Send a 6-byte (int48 BE) payload = 0x000123456789.
        // Frame: FB 01 31 00 01 23 45 67 89 | CRC
        std::uint8_t body[] = {0xFB, 0x01, 0x31,
                               0x00, 0x01, 0x23, 0x45, 0x67, 0x89};
        const std::uint8_t crc = mks_servo::checksum8(body, sizeof(body));
        std::vector<std::uint8_t> frame(body, body + sizeof(body));
        frame.push_back(crc);
        mock_write(p.peer_fd, frame.data(), frame.size());
    });

    auto r = drv.read_encoder_addition();
    t.join();

    REQUIRE(r.ok());
    CHECK(r.value == 0x000123456789ll);
    ::close(p.peer_fd);
}

// ─── move_relative_axis (with ack) ─────────────────────────────────
TEST_CASE("RawDriver::move_relative_axis emits exact byte pattern") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/100'000);

    std::thread t([&] {
        // Expect: FA 01 F4 07 D0 FF 00 00 10 00 D5
        // rpm=2000 (0x07D0), acc=255 (0xFF), counts=4096 (0x00001000)
        auto req = read_n(p.peer_fd, 11);
        REQUIRE(req.size() == 11);
        const std::array<std::uint8_t, 11> expected = {
            0xFA, 0x01, 0xF4, 0x07, 0xD0, 0xFF, 0x00, 0x00, 0x10, 0x00,
            mks_servo::checksum8(req.data(), 10),  // recompute to match
        };
        // Use bit-exact comparison of the first 10 bytes plus CRC validity.
        for (std::size_t i = 0; i < 10; ++i) CHECK(req[i] == expected[i]);
        CHECK(req[10] == mks_servo::checksum8(req.data(), 10));

        // Send ack: FB 01 F4 01 EA   (CRC = (0xFB + 0x01 + 0xF4 + 0x01) & 0xFF = 0xF1)
        std::uint8_t body[] = {0xFB, 0x01, 0xF4, 0x01};
        write_all(p.peer_fd, {body[0], body[1], body[2], body[3],
                              mks_servo::checksum8(body, 4)});
    });

    auto r = drv.move_relative_axis(/*counts=*/4096, /*rpm=*/2000, /*acc=*/255);
    t.join();

    REQUIRE(r.ok());
    CHECK(r.value == true);  // firmware returned 0x01 = success
    ::close(p.peer_fd);
}

// ─── dispatch_move_relative_axis (fire-and-forget) ─────────────────
TEST_CASE("RawDriver::dispatch_* writes the frame and does not read") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/100'000);

    const auto s = drv.dispatch_move_relative_axis(/*counts=*/4096, /*rpm=*/2000, /*acc=*/255);
    CHECK(s == Transport::Status::OK);

    // The 11-byte frame should be sitting on the peer side, ready to read.
    auto req = read_n(p.peer_fd, 11);
    REQUIRE(req.size() == 11);
    CHECK(req[0] == 0xFA);
    CHECK(req[1] == 0x01);
    CHECK(req[2] == 0xF4);
    CHECK(req[3] == 0x07);
    CHECK(req[4] == 0xD0);
    CHECK(req[5] == 0xFF);  // acc=255
    CHECK(req[10] == mks_servo::checksum8(req.data(), 10));
    ::close(p.peer_fd);
}

// ─── enable ────────────────────────────────────────────────────────
TEST_CASE("RawDriver::enable(true) sends ENABLE 0xF3 with payload 0x01") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/100'000);

    std::thread t([&] {
        auto req = read_n(p.peer_fd, 5);
        REQUIRE(req.size() == 5);
        CHECK(req[0] == 0xFA);
        CHECK(req[1] == 0x01);
        CHECK(req[2] == 0xF3);  // ENABLE
        CHECK(req[3] == 0x01);  // on

        std::uint8_t body[] = {0xFB, 0x01, 0xF3, 0x01};
        write_all(p.peer_fd, {body[0], body[1], body[2], body[3],
                              mks_servo::checksum8(body, 4)});
    });

    auto r = drv.enable(true);
    t.join();
    REQUIRE(r.ok());
    CHECK(r.value == true);
    ::close(p.peer_fd);
}

// ─── read_motor_status ─────────────────────────────────────────────
TEST_CASE("RawDriver::read_motor_status decodes enum correctly") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/100'000);

    std::thread t([&] {
        auto req = read_n(p.peer_fd, 4);
        REQUIRE(req.size() == 4);
        CHECK(req[2] == 0xF1);  // QUERY_STATUS

        // Send status = SpeedUp (2): FB 01 F1 02 CRC
        std::uint8_t body[] = {0xFB, 0x01, 0xF1, 0x02};
        write_all(p.peer_fd, {body[0], body[1], body[2], body[3],
                              mks_servo::checksum8(body, 4)});
    });

    auto r = drv.read_motor_status();
    t.join();
    REQUIRE(r.ok());
    CHECK(r.value == MotorStatus::SpeedUp);
    ::close(p.peer_fd);
}

// ─── set_work_current_ma ───────────────────────────────────────────
TEST_CASE("RawDriver::set_work_current_ma encodes uint16 BE") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/100'000);

    std::thread t([&] {
        // current=1500 = 0x05DC
        auto req = read_n(p.peer_fd, 6);
        REQUIRE(req.size() == 6);
        CHECK(req[2] == 0x83);  // SET_WORK_CURRENT
        CHECK(req[3] == 0x05);
        CHECK(req[4] == 0xDC);

        std::uint8_t body[] = {0xFB, 0x01, 0x83, 0x01};
        write_all(p.peer_fd, {body[0], body[1], body[2], body[3],
                              mks_servo::checksum8(body, 4)});
    });

    auto r = drv.set_work_current_ma(1500);
    t.join();
    REQUIRE(r.ok());
    CHECK(r.value == true);
    ::close(p.peer_fd);
}

// ─── timeout path ──────────────────────────────────────────────────
TEST_CASE("RawDriver returns ReadTimeout if peer sends nothing") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/10'000);  // 10 ms

    // Don't send anything on the peer side.
    auto r = drv.read_encoder_addition();
    CHECK(r.t_status == Transport::Status::ReadTimeout);
    CHECK_FALSE(r.ok());

    // Drain whatever the request was, just to be tidy.
    (void)read_n(p.peer_fd, 4);
    ::close(p.peer_fd);
}

// ─── corrupt response path ─────────────────────────────────────────
//
// transact() is robust to stray/corrupt frames in the bus buffer: it scans
// for HEAD_UP and skips frames that fail CRC. A response with only a single
// bad-CRC frame and nothing valid behind it therefore manifests as a
// ReadTimeout (transact gave up looking for a valid frame), not as a
// BadChecksum returned to the caller. This is the correct behavior on a
// half-duplex RS-485 bus where stray MOVE_* acks routinely sit in the
// input buffer.
TEST_CASE("RawDriver: corrupt-only response causes ReadTimeout (frame is skipped)") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/50'000);  // 50 ms

    std::thread t([&] {
        (void)read_n(p.peer_fd, 4);
        // Send a frame with a deliberately wrong CRC. Nothing else follows.
        std::uint8_t bad[] = {0xFB, 0x01, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        mock_write(p.peer_fd, bad, sizeof(bad));
    });

    auto r = drv.read_encoder_addition();
    t.join();
    CHECK(r.t_status == Transport::Status::ReadTimeout);
    CHECK_FALSE(r.ok());
    ::close(p.peer_fd);
}

// ─── stray-frame robustness path ───────────────────────────────────
//
// The realistic case: a previous MOVE_* response is sitting in the buffer
// when we issue a READ_ENCODER. transact must skip the stray frame and
// return the encoder response.
TEST_CASE("RawDriver skips a stray MOVE ack frame and parses the real response") {
    auto p = make_paired_transport();
    RawDriver drv(p.transport, /*addr=*/0x01, /*timeout=*/100'000);

    std::thread t([&] {
        (void)read_n(p.peer_fd, 4);
        // First: a stray MOVE_* ack (5 bytes) — looks like a valid frame
        // but the code is 0xF5, not 0x31, so transact must skip it.
        std::uint8_t stray_body[] = {0xFB, 0x01, 0xF5, 0x01};
        std::uint8_t stray[5];
        std::memcpy(stray, stray_body, 4);
        stray[4] = mks_servo::checksum8(stray_body, 4);
        mock_write(p.peer_fd, stray, 5);

        // Then: the real encoder response.
        std::uint8_t body[] = {0xFB, 0x01, 0x31,
                               0x00, 0x00, 0x00, 0x00, 0x10, 0x00};  // counts=4096
        std::uint8_t frame[10];
        std::memcpy(frame, body, 9);
        frame[9] = mks_servo::checksum8(body, 9);
        mock_write(p.peer_fd, frame, 10);
    });

    auto r = drv.read_encoder_addition();
    t.join();
    REQUIRE(r.ok());
    CHECK(r.value == 4096);
    ::close(p.peer_fd);
}
