#include "timerutilities.hpp"
#include <chrono>

double GetTimeNow() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - start;
    return elapsed.count();
}