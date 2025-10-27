#pragma once

#include <array>
#include <atomic>
#include <optional>
#include <vector>


// -----------------------------------------------------------------------------
// LockFreeRingBuffer
// -----------------------------------------------------------------------------
// A single-producer, single-consumer (SPSC) lock-free ring buffer.
// Safe for real-time loops: no malloc, no locks, constant-time push/pop. Allows
// For popBatch with gets all elements in the buffer and stores them in a vector
//
// Template parameters:
//   T - type of the stored elements
//   N - buffer capacity (must be > 1)
// -----------------------------------------------------------------------------

template <typename T, size_t N>
class LockFreeRingBuffer {
public: 
    LockFreeRingBuffer() = default;
    LockFreeRingBuffer(const LockFreeRingBuffer&) = delete;
    LockFreeRingBuffer& operator=(const LockFreeRingBuffer&) = delete;

    bool push(const T& items);
    std::optional<T> pop();

    size_t popBatch(std::vector<T>& out);

    bool empty() const;
    bool full() const;
    size_t size() const;

private:
    std::array<T, N> buffer_;
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
};

#include "core/LockFreeRingBuffer.tpp"
