#pragma once
#include <chrono>

namespace TimeUtils {
    // Get elapsed time in seconds from program start
    // Returns time as a double in seconds
    inline double GetTimeNow() {
        using clock = std::chrono::steady_clock;
        static const auto t0 = clock::now(); // one-time zero point
        auto now = clock::now();
        std::chrono::duration<double> elapsed = now - t0;
        return elapsed.count(); // seconds as double
    }
}
