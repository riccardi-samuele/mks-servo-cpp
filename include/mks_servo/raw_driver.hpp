// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// RawDriver: Level 3 of the mks-servo API. One method per firmware opcode
// (per MKS SERVO42D RS485 V1.0.6 manual). The same architectural layer as
// the Python lib's `mks_servo.raw.RawDriver`.
//
// Conventions:
//   - All methods are non-blocking (return as soon as the bus exchange
//     finishes) and never throw.
//   - All methods that issue a command and read an acknowledgement return
//     Result<T>; check `.ok()` before using `.value`.
//   - Move commands have an `_async` sibling that does fire-and-forget
//     (writes the frame, doesn't read the ack). Use these for hot paths
//     when you'd rather poll the encoder for completion.
//   - Numeric types are explicit (std::int32_t etc.). RPM is uint16
//     because firmware caps at 3000.

#ifndef MKS_SERVO_RAW_DRIVER_HPP
#define MKS_SERVO_RAW_DRIVER_HPP

#include <cstdint>

#include "mks_servo/protocol.hpp"
#include "mks_servo/transport.hpp"

namespace mks_servo {

// ─── Opcodes (mirror of mks_servo.constants.OpCode) ────────────────
namespace op {
inline constexpr std::uint8_t READ_ENCODER          = 0x30;
inline constexpr std::uint8_t READ_ENCODER_ADDITION = 0x31;
inline constexpr std::uint8_t READ_SPEED_RPM        = 0x32;
inline constexpr std::uint8_t READ_PULSES           = 0x33;
inline constexpr std::uint8_t READ_IO               = 0x34;
inline constexpr std::uint8_t READ_ANGLE_ERROR      = 0x39;
inline constexpr std::uint8_t READ_EN_PIN           = 0x3A;
inline constexpr std::uint8_t READ_HOMING_STATUS    = 0x3B;
inline constexpr std::uint8_t RELEASE_PROTECTION    = 0x3D;
inline constexpr std::uint8_t READ_PROTECT_STATUS   = 0x3E;
inline constexpr std::uint8_t RESTORE_DEFAULTS      = 0x3F;
inline constexpr std::uint8_t RESTART               = 0x41;
inline constexpr std::uint8_t READ_ALL_CONFIG       = 0x47;
inline constexpr std::uint8_t CALIBRATE             = 0x80;
inline constexpr std::uint8_t SET_WORK_MODE         = 0x82;
inline constexpr std::uint8_t SET_WORK_CURRENT      = 0x83;
inline constexpr std::uint8_t SET_SUBDIVISION       = 0x84;
inline constexpr std::uint8_t SET_DIRECTION         = 0x85;
inline constexpr std::uint8_t SET_HOLDING_CURRENT   = 0x86;
inline constexpr std::uint8_t SET_BAUD              = 0x8A;
inline constexpr std::uint8_t SET_SLAVE_ADDR        = 0x8B;
inline constexpr std::uint8_t SET_RESPOND_ACTIVE    = 0x8C;
inline constexpr std::uint8_t SET_ZERO_POINT        = 0x92;
inline constexpr std::uint8_t MOVE_REL_AXIS         = 0xF4;
inline constexpr std::uint8_t MOVE_ABS_AXIS         = 0xF5;
inline constexpr std::uint8_t MOVE_SPEED            = 0xF6;
inline constexpr std::uint8_t EMERGENCY_STOP        = 0xF7;
inline constexpr std::uint8_t QUERY_STATUS          = 0xF1;
inline constexpr std::uint8_t ENABLE                = 0xF3;
inline constexpr std::uint8_t MOVE_REL_PULSES       = 0xFD;
inline constexpr std::uint8_t MOVE_ABS_PULSES       = 0xFE;
inline constexpr std::uint8_t SAVE_SPEED_STATE      = 0xFF;
}  // namespace op

// ─── MotorStatus (mirror of mks_servo.raw.MotorStatus) ─────────────
enum class MotorStatus : std::uint8_t {
    QueryFail   = 0,
    Stopped     = 1,
    SpeedUp     = 2,
    SpeedDown   = 3,
    FullSpeed   = 4,
    Homing      = 5,
    Calibrating = 6,
};

enum class WorkMode : std::uint8_t {
    CR_OPEN  = 0,
    CR_CLOSE = 1,
    CR_vFOC  = 2,
    SR_OPEN  = 3,
    SR_CLOSE = 4,
    SR_vFOC  = 5,
};

enum class Direction : std::uint8_t {
    CW  = 0,
    CCW = 1,
};

// ─── Result<T>: bundles a value with status info ───────────────────
template <typename T>
struct Result {
    T                 value         = T{};
    Transport::Status t_status      = Transport::Status::OK;
    ParseStatus       parse_status  = ParseStatus::OK;

    bool ok() const noexcept {
        return t_status == Transport::Status::OK
            && parse_status == ParseStatus::OK;
    }
};

// ─── RawDriver ─────────────────────────────────────────────────────
class RawDriver {
public:
    explicit RawDriver(Transport&    transport,
                       std::uint8_t  slave_addr        = 1,
                       std::uint64_t default_timeout_us = 500'000) noexcept
        : transport_(transport)
        , addr_(slave_addr)
        , timeout_us_(default_timeout_us) {}

    std::uint8_t  slave_addr() const noexcept { return addr_; }
    void          set_slave_addr_local(std::uint8_t a) noexcept { addr_ = a; }
    std::uint64_t default_timeout_us() const noexcept { return timeout_us_; }
    void          set_default_timeout_us(std::uint64_t us) noexcept { timeout_us_ = us; }

    // Discard any pending bytes on the bus. Exposed for callers (e.g.
    // Motor) that know an asynchronous frame may still be in flight
    // (typically the firmware's second response to a MOVE_*).
    void transport_drain() noexcept { transport_.drain_input(); }

    // Same as transport_drain but waits up to `settle_ms` for in-flight
    // bytes to arrive before declaring the buffer empty. Use when a
    // response is *known* to be coming but timing is uncertain.
    void transport_drain_settle(int settle_ms) noexcept {
        transport_.drain_input(settle_ms);
    }

    // ─── Reads ──────────────────────────────────────────────────
    Result<std::int64_t> read_encoder_addition() noexcept {
        auto r = txn(op::READ_ENCODER_ADDITION, nullptr, 0, 6);
        Result<std::int64_t> out;
        out.t_status     = r.t_status;
        out.parse_status = r.parse_status;
        if (out.ok()) out.value = decode_int_be(r.payload, 6, /*signed=*/true);
        return out;
    }

    Result<std::int16_t> read_speed_rpm() noexcept {
        auto r = txn(op::READ_SPEED_RPM, nullptr, 0, 2);
        Result<std::int16_t> out;
        out.t_status     = r.t_status;
        out.parse_status = r.parse_status;
        if (out.ok()) {
            out.value = static_cast<std::int16_t>(decode_int_be(r.payload, 2, true));
        }
        return out;
    }

    Result<std::int32_t> read_pulses() noexcept {
        auto r = txn(op::READ_PULSES, nullptr, 0, 4);
        Result<std::int32_t> out;
        out.t_status     = r.t_status;
        out.parse_status = r.parse_status;
        if (out.ok()) {
            out.value = static_cast<std::int32_t>(decode_int_be(r.payload, 4, true));
        }
        return out;
    }

    Result<std::int32_t> read_angle_error() noexcept {
        auto r = txn(op::READ_ANGLE_ERROR, nullptr, 0, 4);
        Result<std::int32_t> out;
        out.t_status     = r.t_status;
        out.parse_status = r.parse_status;
        if (out.ok()) {
            out.value = static_cast<std::int32_t>(decode_int_be(r.payload, 4, true));
        }
        return out;
    }

    Result<MotorStatus> read_motor_status() noexcept {
        auto r = txn(op::QUERY_STATUS, nullptr, 0, 1);
        Result<MotorStatus> out;
        out.t_status     = r.t_status;
        out.parse_status = r.parse_status;
        if (out.ok()) out.value = static_cast<MotorStatus>(r.payload[0]);
        return out;
    }

    // ─── Enable / disable ───────────────────────────────────────
    Result<bool> enable(bool on) noexcept {
        const std::uint8_t d = on ? 0x01 : 0x00;
        auto r = txn(op::ENABLE, &d, 1, 1);
        return ack_to_bool(r);
    }

    // ─── Move commands (with ack) ───────────────────────────────
    //
    // Frame format (per manual §7.4):
    //   MOVE_REL_AXIS / MOVE_ABS_AXIS: rpm[2 BE] | acc[1] | counts[4 BE signed]
    //   MOVE_REL_PULSES: dir<<7 | rpm_hi[1] | rpm_lo[1] | acc[1] | pulses[4 BE]
    //   MOVE_ABS_PULSES: rpm[2 BE] | acc[1] | pulses[4 BE signed]
    //   MOVE_SPEED:     dir<<7 | rpm_hi[1] | rpm_lo[1] | acc[1]

    Result<bool> move_relative_axis(std::int32_t  counts,
                                    std::uint16_t rpm,
                                    std::uint8_t  acc) noexcept {
        std::uint8_t d[7];
        encode_move_axis(d, counts, rpm, acc);
        auto r = txn(op::MOVE_REL_AXIS, d, 7, 1);
        return ack_to_bool(r);
    }

    Result<bool> move_absolute_axis(std::int32_t  counts,
                                    std::uint16_t rpm,
                                    std::uint8_t  acc) noexcept {
        std::uint8_t d[7];
        encode_move_axis(d, counts, rpm, acc);
        auto r = txn(op::MOVE_ABS_AXIS, d, 7, 1);
        return ack_to_bool(r);
    }

    Result<bool> emergency_stop() noexcept {
        auto r = txn(op::EMERGENCY_STOP, nullptr, 0, 1);
        return ack_to_bool(r);
    }

    // Cmd 0xF6: velocity-mode move. Motor runs at `rpm` in the given
    // direction with the requested acceleration. Stop by calling this
    // again with rpm=0.
    //
    // Frame: dir<<7 | rpm_hi(4 bits) | rpm_lo(8 bits) | acc(1 byte)
    Result<bool> move_speed(std::uint16_t rpm,
                            std::uint8_t  acc,
                            Direction     direction = Direction::CW) noexcept {
        const std::uint8_t dir_bit = (direction == Direction::CCW) ? 0x80 : 0x00;
        const std::uint8_t d[3] = {
            static_cast<std::uint8_t>(dir_bit | ((rpm >> 8) & 0x0F)),
            static_cast<std::uint8_t>(rpm & 0xFF),
            acc,
        };
        auto r = txn(op::MOVE_SPEED, d, 3, 1);
        return ack_to_bool(r);
    }

    // ─── Move commands (fire-and-forget) ─────────────────────────
    //
    // No ack read. Returns the write-side Transport::Status only. The motor
    // still executes the command; the caller is responsible for verifying
    // completion (e.g., by polling read_encoder_addition).
    //
    // Saves ~850 μs per command at 256 kbaud vs the ack-waiting form.

    Transport::Status dispatch_move_relative_axis(std::int32_t  counts,
                                                  std::uint16_t rpm,
                                                  std::uint8_t  acc) noexcept {
        std::uint8_t d[7];
        encode_move_axis(d, counts, rpm, acc);
        return dispatch(op::MOVE_REL_AXIS, d, 7);
    }

    Transport::Status dispatch_move_absolute_axis(std::int32_t  counts,
                                                  std::uint16_t rpm,
                                                  std::uint8_t  acc) noexcept {
        std::uint8_t d[7];
        encode_move_axis(d, counts, rpm, acc);
        return dispatch(op::MOVE_ABS_AXIS, d, 7);
    }

    // ─── Flash setters (intentionally minimal — extend as needed) ──
    Result<bool> set_work_current_ma(std::uint16_t ma) noexcept {
        const std::uint8_t d[2] = {
            static_cast<std::uint8_t>((ma >> 8) & 0xFF),
            static_cast<std::uint8_t>(ma & 0xFF),
        };
        auto r = txn(op::SET_WORK_CURRENT, d, 2, 1);
        return ack_to_bool(r);
    }

    Result<bool> set_subdivision(std::uint16_t microsteps) noexcept {
        // Firmware encodes 256 microsteps as 0x00 (1-byte field).
        const std::uint8_t v = (microsteps == 256)
            ? 0x00
            : static_cast<std::uint8_t>(microsteps & 0xFF);
        auto r = txn(op::SET_SUBDIVISION, &v, 1, 1);
        return ack_to_bool(r);
    }

    Result<bool> set_work_mode(WorkMode m) noexcept {
        const std::uint8_t v = static_cast<std::uint8_t>(m);
        auto r = txn(op::SET_WORK_MODE, &v, 1, 1);
        return ack_to_bool(r);
    }

    // ─── Maintenance ────────────────────────────────────────────
    Result<bool> restore_defaults() noexcept {
        auto r = txn(op::RESTORE_DEFAULTS, nullptr, 0, 1);
        return ack_to_bool(r);
    }

    Result<bool> restart() noexcept {
        auto r = txn(op::RESTART, nullptr, 0, 1);
        return ack_to_bool(r);
    }

    Result<bool> release_protection() noexcept {
        auto r = txn(op::RELEASE_PROTECTION, nullptr, 0, 1);
        return ack_to_bool(r);
    }

    // Cmd 0x3E: true if stall protection is currently latched.
    Result<bool> read_protect_status() noexcept {
        auto r = txn(op::READ_PROTECT_STATUS, nullptr, 0, 1);
        return ack_to_bool(r);
    }

    // Cmd 0x92: reset the firmware's internal absolute-position counter to
    // the current rotor position. Subsequent MOVE_ABS_AXIS commands are
    // relative to this new zero. Persists in flash.
    Result<bool> set_zero_point() noexcept {
        auto r = txn(op::SET_ZERO_POINT, nullptr, 0, 1);
        return ack_to_bool(r);
    }

private:
    // Issue a synchronous transaction with the configured default timeout.
    TransactResult txn(std::uint8_t opcode,
                       const std::uint8_t* data,
                       std::size_t data_len,
                       std::size_t expected_payload_len) noexcept {
        return transact(transport_, addr_, opcode,
                        data, data_len, expected_payload_len, timeout_us_);
    }

    // Build and write only — no read.
    Transport::Status dispatch(std::uint8_t opcode,
                               const std::uint8_t* data,
                               std::size_t data_len) noexcept {
        std::uint8_t req[MAX_FRAME_SIZE];
        const auto n = build_frame(req, sizeof(req), addr_, opcode, data, data_len);
        if (n == 0) return Transport::Status::InvalidArg;
        // Note: we intentionally do NOT drain_input here. Caller may be
        // pipelining and won't appreciate losing in-flight bytes.
        return transport_.write_all(req, n);
    }

    // Pack rpm (2 BE) + acc (1) + counts (4 BE signed) into a 7-byte buffer.
    static void encode_move_axis(std::uint8_t out[7],
                                 std::int32_t  counts,
                                 std::uint16_t rpm,
                                 std::uint8_t  acc) noexcept {
        out[0] = static_cast<std::uint8_t>((rpm >> 8) & 0xFF);
        out[1] = static_cast<std::uint8_t>(rpm & 0xFF);
        out[2] = acc;
        const std::uint32_t u = static_cast<std::uint32_t>(counts);
        out[3] = static_cast<std::uint8_t>((u >> 24) & 0xFF);
        out[4] = static_cast<std::uint8_t>((u >> 16) & 0xFF);
        out[5] = static_cast<std::uint8_t>((u >> 8)  & 0xFF);
        out[6] = static_cast<std::uint8_t>( u        & 0xFF);
    }

    // Big-endian integer decode, signed or unsigned, 1..8 bytes.
    static std::int64_t decode_int_be(const std::uint8_t* buf,
                                      std::size_t n,
                                      bool is_signed) noexcept {
        std::uint64_t u = 0;
        for (std::size_t i = 0; i < n; ++i) {
            u = (u << 8) | buf[i];
        }
        if (is_signed && n < 8) {
            // Sign-extend from the top bit of the most significant byte.
            const std::uint64_t sign_bit = 1ull << (n * 8 - 1);
            if (u & sign_bit) {
                const std::uint64_t mask = ~((1ull << (n * 8)) - 1);
                u |= mask;
            }
        }
        return static_cast<std::int64_t>(u);
    }

    static Result<bool> ack_to_bool(const TransactResult& r) noexcept {
        Result<bool> out;
        out.t_status     = r.t_status;
        out.parse_status = r.parse_status;
        out.value        = out.ok() && r.payload_len >= 1 && r.payload[0] == 0x01;
        return out;
    }

    Transport&    transport_;
    std::uint8_t  addr_;
    std::uint64_t timeout_us_;
};

}  // namespace mks_servo

#endif  // MKS_SERVO_RAW_DRIVER_HPP
