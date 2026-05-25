// SPDX-License-Identifier: Apache-2.0
//
// Stress-read: hammer read_encoder_addition in a tight loop, count failures.
// Use this to isolate "is the bus reliable in isolation?" from "does the
// stream get corrupted after MOVE_*?".

#include <cstdio>
#include <cstdlib>

#include "mks_servo/raw_driver.hpp"
#include "mks_servo/transport.hpp"

using mks_servo::RawDriver;
using mks_servo::Transport;

int main(int argc, char** argv) {
    const char* dev  = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    const int   baud = (argc > 2) ? std::atoi(argv[2]) : 38400;
    const int   N    = (argc > 3) ? std::atoi(argv[3]) : 1000;

    Transport t;
    if (t.open(dev, baud) != Transport::Status::OK) {
        std::fprintf(stderr, "open failed\n");
        return 1;
    }
    RawDriver drv(t);

    int ok = 0, t_fail = 0, p_fail = 0;
    for (int i = 0; i < N; ++i) {
        const auto r = drv.read_encoder_addition();
        if (r.ok()) ok++;
        else if (r.t_status != Transport::Status::OK) t_fail++;
        else p_fail++;
    }
    std::printf("N=%d  ok=%d  t_fail=%d  p_fail=%d\n", N, ok, t_fail, p_fail);
    return (ok == N) ? 0 : 2;
}
