// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 mks-servo-cpp contributors
//
// Serial transport for MKS SERVO42D over RS-485 (via a USB-RS485 adapter).
//
// Design constraints:
//   - Raw mode (no canonical processing, no echo, 8N1, no flow control).
//   - VMIN=0, VTIME=0: every read() is non-blocking; we use poll() to time
//     out at a precise microsecond granularity.
//   - Non-standard baud rates (notably 256000 — the firmware's fastest) via
//     Linux termios2 + BOTHER.
//   - No exceptions in the hot path; all errors are enum returns.
//   - Header-only, no thread safety at this layer (a SharedTransport wrapper
//     will add a mutex later for multi-driver-on-one-bus scenarios).
//
// Platform: Linux. macOS support (IOSSIOSPEED) and Windows (DCB) can be added
// later without changing this header's public surface.

#ifndef MKS_SERVO_TRANSPORT_HPP
#define MKS_SERVO_TRANSPORT_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

#include <string.h>  // memmove

#include "mks_servo/protocol.hpp"

#if defined(__linux__)
  #include <asm/termbits.h>     // struct termios2, BOTHER, TCGETS2/TCSETS2, CS8, CREAD, CLOCAL
  #include <errno.h>
  #include <fcntl.h>
  #include <poll.h>
  #include <sys/ioctl.h>
  #include <time.h>             // clock_gettime, CLOCK_MONOTONIC
  #include <unistd.h>
#else
  #error "mks_servo::Transport currently supports Linux only. Patches welcome."
#endif

namespace mks_servo {

class Transport {
public:
    enum class Status : std::uint8_t {
        OK             = 0,
        OpenFailed     = 1,
        ConfigFailed   = 2,
        WriteFailed    = 3,
        ReadTimeout    = 4,
        ReadFailed     = 5,
        NotOpen        = 6,
        InvalidArg     = 7,
    };

    Transport() noexcept = default;

    // Adopt an already-open file descriptor. The fd is closed when this
    // Transport is destroyed. Useful for:
    //   - tests using socketpair/pty to simulate a driver
    //   - advanced users who want to configure the fd themselves (e.g. with
    //     a custom termios layout, or on a non-Linux backend not yet
    //     supported by open()).
    // The fd is taken AS-IS — no termios reconfiguration is performed.
    explicit Transport(int adopted_fd) noexcept : fd_(adopted_fd) {}

    ~Transport() noexcept { close(); }

    Transport(const Transport&)            = delete;
    Transport& operator=(const Transport&) = delete;

    Transport(Transport&& other) noexcept : fd_(other.fd_) {
        other.fd_ = -1;
    }

    Transport& operator=(Transport&& other) noexcept {
        if (this != &other) {
            close();
            fd_       = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }

    // Open a serial device and configure it for raw, 8N1, no flow control,
    // at the requested baud rate.
    //
    //   path:  e.g. "/dev/ttyUSB0"
    //   baud:  any rate supported by the underlying USB-serial driver.
    //          Standard rates (38400, 115200) and non-standard (256000) all
    //          work via termios2 + BOTHER.
    Status open(const char* path, int baud) noexcept {
        close();
        if (!path || baud <= 0) return Status::InvalidArg;

        const int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) return Status::OpenFailed;

        struct termios2 tio{};
        if (::ioctl(fd, TCGETS2, &tio) != 0) {
            ::close(fd);
            return Status::ConfigFailed;
        }

        // Raw mode: zero all flags, then enable what we need.
        tio.c_iflag = 0;
        tio.c_oflag = 0;
        tio.c_lflag = 0;

        // Clear the baud bits, then enable BOTHER and explicit ispeed/ospeed.
        // 8N1, receiver on, ignore modem control lines (CLOCAL).
        // The bitwise-NOTs are cast to tcflag_t because some libc headers
        // define these constants as signed ints, producing sign-conversion
        // warnings under -Wsign-conversion.
        using Tcf = tcflag_t;
        tio.c_cflag &= static_cast<Tcf>(~static_cast<Tcf>(CBAUD));
        tio.c_cflag &= static_cast<Tcf>(~static_cast<Tcf>(CSIZE));
        tio.c_cflag &= static_cast<Tcf>(~static_cast<Tcf>(PARENB));
        tio.c_cflag &= static_cast<Tcf>(~static_cast<Tcf>(CSTOPB));
        tio.c_cflag &= static_cast<Tcf>(~static_cast<Tcf>(CRTSCTS));
        tio.c_cflag |= static_cast<Tcf>(BOTHER | CS8 | CREAD | CLOCAL);
        tio.c_ispeed = static_cast<speed_t>(baud);
        tio.c_ospeed = static_cast<speed_t>(baud);

        // VMIN=0, VTIME=0: read() returns immediately with whatever is in the
        // buffer (may be 0 bytes). We use poll() for the actual timeout.
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;

        if (::ioctl(fd, TCSETS2, &tio) != 0) {
            ::close(fd);
            return Status::ConfigFailed;
        }

        // Switch fd back to blocking (poll handles timing); some kernels
        // dislike non-blocking reads on serial devices in raw mode.
        const int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) < 0) {
            ::close(fd);
            return Status::ConfigFailed;
        }

        fd_ = fd;
        return Status::OK;
    }

    void close() noexcept {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool is_open() const noexcept { return fd_ >= 0; }
    int  fd()      const noexcept { return fd_; }

    // Discard any unread bytes in the input buffer.
    //
    // settle_ms: how long to wait for additional in-flight bytes to arrive
    // before considering the buffer drained. Useful immediately after a
    // MOVE_* dispatch where the firmware's "complete" response may still
    // be in transit across USB-FTDI when this is called. settle_ms=0
    // = best-effort drain of what's already buffered.
    //
    // Why a two-stage drain: TCFLSH alone does not always clear bytes
    // that are still sitting in the FTDI chip waiting to be pulled across
    // USB. The MKS firmware emits a "started" then a "complete" response
    // per MOVE_* command — without the wait, the complete response can
    // linger here and corrupt subsequent transactions.
    void drain_input(int settle_ms = 0) noexcept {
        if (fd_ < 0) return;
        std::uint8_t junk[64];
        for (;;) {
            struct pollfd pfd{};
            pfd.fd     = fd_;
            pfd.events = POLLIN;
            const int pr = ::poll(&pfd, 1, settle_ms);
            settle_ms = 0;  // only wait once; subsequent polls are immediate
            if (pr <= 0) break;
            const ssize_t r = ::read(fd_, junk, sizeof(junk));
            if (r <= 0) break;
        }
        ::ioctl(fd_, TCFLSH, TCIFLUSH);
    }

    // Write all bytes. Loops over partial writes; returns WriteFailed on
    // unrecoverable error.
    Status write_all(const std::uint8_t* data, std::size_t n) noexcept {
        if (fd_ < 0) return Status::NotOpen;
        if (n == 0)  return Status::OK;
        std::size_t off = 0;
        while (off < n) {
            const ssize_t w = ::write(fd_, data + off, n - off);
            if (w < 0) {
                if (errno == EINTR) continue;
                return Status::WriteFailed;
            }
            off += static_cast<std::size_t>(w);
        }
        return Status::OK;
    }

    // Read exactly `n` bytes within `timeout_us` total microseconds.
    // Uses poll() between read() calls so we time out at the requested
    // resolution regardless of the kernel's TTY buffering.
    Status read_exact(std::uint8_t* out,
                      std::size_t   n,
                      std::uint64_t timeout_us) noexcept {
        if (fd_ < 0) return Status::NotOpen;
        if (n == 0)  return Status::OK;

        // Convert microseconds to a relative deadline using poll's ms units.
        // We loop because the poll timeout argument is per-call.
        std::size_t got = 0;
        std::uint64_t remaining_us = timeout_us;

        while (got < n) {
            // poll() takes milliseconds; round up so we don't undershoot.
            int timeout_ms;
            if (remaining_us == 0) {
                timeout_ms = 0;  // one last non-blocking check
            } else if (remaining_us >= 1000) {
                timeout_ms = static_cast<int>(remaining_us / 1000);
            } else {
                timeout_ms = 1;
            }

            struct pollfd pfd{};
            pfd.fd     = fd_;
            pfd.events = POLLIN;

            const auto t_before = monotonic_us();
            const int pr = ::poll(&pfd, 1, timeout_ms);
            const auto t_after = monotonic_us();
            const std::uint64_t spent = t_after - t_before;

            if (pr < 0) {
                if (errno == EINTR) {
                    // Recompute remaining time and retry.
                    if (spent >= remaining_us) return Status::ReadTimeout;
                    remaining_us -= spent;
                    continue;
                }
                return Status::ReadFailed;
            }
            if (pr == 0) {
                return Status::ReadTimeout;
            }

            const ssize_t r = ::read(fd_, out + got, n - got);
            if (r < 0) {
                if (errno == EINTR) {
                    if (spent >= remaining_us) return Status::ReadTimeout;
                    remaining_us -= spent;
                    continue;
                }
                return Status::ReadFailed;
            }
            if (r == 0) {
                // EOF on a serial port is unusual but treat as failure.
                return Status::ReadFailed;
            }
            got += static_cast<std::size_t>(r);

            if (spent >= remaining_us) {
                // Out of time. Only OK if we already filled the buffer.
                if (got >= n) return Status::OK;
                return Status::ReadTimeout;
            }
            remaining_us -= spent;
        }
        return Status::OK;
    }

private:
    int fd_ = -1;

    static std::uint64_t monotonic_us() noexcept {
        struct timespec ts;
        ::clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000ull
             + static_cast<std::uint64_t>(ts.tv_nsec) / 1000ull;
    }
};

// ─── transact: build frame, write, read, parse ─────────────────────
//
// One-shot request/response. The response payload is copied into the result
// (so the caller doesn't have to keep a buffer alive). Returns a result with
// a Transport::Status and (if Status==OK) a ParseStatus + payload.
//
// For fire-and-forget moves (no ack wanted), call write_all directly with a
// frame built via build_frame.

struct TransactResult {
    Transport::Status t_status     = Transport::Status::OK;
    ParseStatus       parse_status = ParseStatus::OK;
    std::uint8_t      addr         = 0;
    std::uint8_t      code         = 0;
    std::uint8_t      payload[MAX_FRAME_SIZE]{};
    std::size_t       payload_len  = 0;
};

inline TransactResult transact(Transport&    transport,
                               std::uint8_t  addr,
                               std::uint8_t  code,
                               const std::uint8_t* data,
                               std::size_t   data_len,
                               std::size_t   expected_payload_len,
                               std::uint64_t timeout_us) noexcept {
    TransactResult res;

    std::uint8_t req[MAX_FRAME_SIZE];
    const auto req_n = build_frame(req, sizeof(req), addr, code, data, data_len);
    if (req_n == 0) {
        res.t_status = Transport::Status::InvalidArg;
        return res;
    }

    transport.drain_input();

    res.t_status = transport.write_all(req, req_n);
    if (res.t_status != Transport::Status::OK) return res;

    const std::size_t want = FRAME_OVERHEAD + expected_payload_len;
    if (want > MAX_FRAME_SIZE) {
        res.t_status = Transport::Status::InvalidArg;
        return res;
    }

    // Frame-scanning read loop. The MKS firmware sends *two* responses per
    // MOVE_* command (start + complete), so a previous fire-and-forget MOVE
    // can leave stray ack frames in the bus buffer. We sliding-window over
    // the input: at each scan position, check if a valid `want`-byte frame
    // with the right opcode starts there; if not, advance one byte. This
    // works even when stray frames have a different length than `want`,
    // because we never throw away bytes we haven't tried as a frame start.
    //
    // Bounded by MAX_SCAN_BYTES so line noise can't spin us forever.
    // Allow up to 16 frames worth of garbage before giving up; in practice
    // the firmware emits at most ~2 stray ack frames per pending MOVE_*.
    constexpr std::size_t MAX_SCAN_BYTES = MAX_FRAME_SIZE * 16;
    constexpr std::size_t BUF_CAPACITY   = MAX_FRAME_SIZE * 2;
    std::uint8_t buf[BUF_CAPACITY];
    std::size_t  buf_n    = 0;
    std::size_t  scan_pos = 0;

    while (scan_pos < MAX_SCAN_BYTES) {
        // Ensure buf[scan_pos..scan_pos+want) is filled. Compact when we'd
        // otherwise overflow the buffer.
        while (buf_n - scan_pos < want) {
            if (buf_n + (want - (buf_n - scan_pos)) > BUF_CAPACITY) {
                std::memmove(buf, buf + scan_pos, buf_n - scan_pos);
                buf_n   -= scan_pos;
                scan_pos = 0;
            }
            const std::size_t need = want - (buf_n - scan_pos);
            res.t_status = transport.read_exact(buf + buf_n, need, timeout_us);
            if (res.t_status != Transport::Status::OK) return res;
            buf_n += need;
        }

        if (buf[scan_pos] == HEAD_UP) {
            const auto pf = parse_frame(buf + scan_pos, want);
            if (pf.status == ParseStatus::OK
                && (code == 0 || pf.code == code)) {
                res.t_status     = Transport::Status::OK;
                res.parse_status = ParseStatus::OK;
                res.addr         = pf.addr;
                res.code         = pf.code;
                res.payload_len  = pf.payload_len;
                std::memcpy(res.payload, pf.payload, pf.payload_len);
                return res;
            }
        }
        // Not HEAD_UP or parse/code mismatch — advance one byte. Never
        // discard bytes we haven't tried as frame starts.
        scan_pos += 1;
    }

    res.t_status = Transport::Status::ReadFailed;
    return res;
}

}  // namespace mks_servo

#endif  // MKS_SERVO_TRANSPORT_HPP
