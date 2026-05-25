// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Protocol layer for MKS SERVO42D RS485 stepper drivers.
//
// Frame format (per manufacturer manual §4):
//   downlink (host → driver):  0xFA | addr | code | data... | crc8
//   uplink   (driver → host):  0xFB | addr | code | data... | crc8
// where crc8 is the 8-bit modular sum of all preceding bytes.
//
// This header is single-file, zero-allocation, no exceptions in the hot path,
// constexpr-friendly. Frames write into a caller-provided span; no std::vector,
// no std::string.

#ifndef MKS_SERVO_PROTOCOL_HPP
#define MKS_SERVO_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace mks_servo {

inline constexpr std::uint8_t HEAD_DOWN = 0xFA;
inline constexpr std::uint8_t HEAD_UP   = 0xFB;

// Minimum overhead per frame: head + addr + code + crc = 4 bytes.
inline constexpr std::size_t FRAME_OVERHEAD = 4;

// Maximum frame size we ever emit. The longest opcodes (MOVE_*_PULSES) carry
// at most 7 payload bytes, giving 11 total. We round up to a comfortable 16.
inline constexpr std::size_t MAX_FRAME_SIZE = 16;

enum class ParseStatus : std::uint8_t {
    OK             = 0,
    TooShort       = 1,
    BadHead        = 2,
    BadChecksum    = 3,
};

struct ParsedFrame {
    std::uint8_t  addr;
    std::uint8_t  code;
    const std::uint8_t* payload;       // pointer into the input buffer
    std::size_t   payload_len;
    ParseStatus   status;
};

// 8-bit modular sum. constexpr so it can be evaluated at compile time when
// inputs are known.
constexpr std::uint8_t checksum8(const std::uint8_t* buf, std::size_t n) noexcept {
    std::uint32_t sum = 0;
    for (std::size_t i = 0; i < n; ++i) sum += buf[i];
    return static_cast<std::uint8_t>(sum & 0xFFu);
}

// Build a downlink frame into `out`. Returns total bytes written, or 0 if the
// output buffer is too small. No allocation; no exceptions.
//
//   out:           destination buffer (must hold at least FRAME_OVERHEAD + data_len)
//   out_capacity:  size of `out` in bytes
//   addr:          slave address 0..255
//   code:          opcode 0..255
//   data:          payload (may be nullptr if data_len == 0)
//   data_len:      payload length in bytes
inline std::size_t build_frame(std::uint8_t* out,
                               std::size_t out_capacity,
                               std::uint8_t addr,
                               std::uint8_t code,
                               const std::uint8_t* data,
                               std::size_t data_len) noexcept {
    const std::size_t total = FRAME_OVERHEAD + data_len;
    if (out_capacity < total) return 0;
    out[0] = HEAD_DOWN;
    out[1] = addr;
    out[2] = code;
    for (std::size_t i = 0; i < data_len; ++i) out[3 + i] = data[i];
    out[total - 1] = checksum8(out, total - 1);
    return total;
}

// Convenience overload: fixed-size std::array destination.
template <std::size_t N>
inline std::size_t build_frame(std::array<std::uint8_t, N>& out,
                               std::uint8_t addr,
                               std::uint8_t code,
                               const std::uint8_t* data = nullptr,
                               std::size_t data_len = 0) noexcept {
    return build_frame(out.data(), N, addr, code, data, data_len);
}

// Parse an uplink frame. Does not copy: returned ParsedFrame.payload points
// into the input buffer.
//
//   buf:           input bytes (entire frame including head, crc)
//   len:           number of bytes in buf
inline ParsedFrame parse_frame(const std::uint8_t* buf, std::size_t len) noexcept {
    ParsedFrame f{};
    if (len < FRAME_OVERHEAD) {
        f.status = ParseStatus::TooShort;
        return f;
    }
    if (buf[0] != HEAD_UP) {
        f.status = ParseStatus::BadHead;
        return f;
    }
    const std::uint8_t given_crc    = buf[len - 1];
    const std::uint8_t expected_crc = checksum8(buf, len - 1);
    if (given_crc != expected_crc) {
        f.status = ParseStatus::BadChecksum;
        return f;
    }
    f.addr        = buf[1];
    f.code        = buf[2];
    f.payload     = buf + 3;
    f.payload_len = len - FRAME_OVERHEAD;
    f.status      = ParseStatus::OK;
    return f;
}

}  // namespace mks_servo

#endif  // MKS_SERVO_PROTOCOL_HPP
