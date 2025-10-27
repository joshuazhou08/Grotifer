#include "core/LockFreeRingBuffer.hpp"

template <typename T, size_t N> 
bool LockFreeRingBuffer<T, N>::push (const T& item) {
    auto head = head_.load(std::memory_order_relaxed);
    auto next = (head + 1) % N;

    if (next == tail_.load(std::memory_order_acquire))
        return false;
    
    buffer_[head] = item;
    head_.store(next, std::memory_order_release);
    return true;
}

template <typename T, size_t N> 
std::optional<T> LockFreeRingBuffer<T, N>::pop() {
    auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire))
        return std::nullopt; // empty

    T item = buffer_[tail];
    tail_.store((tail + 1) % N, std::memory_order_release);
    return item;
}

template <typename T, size_t N>
size_t LockFreeRingBuffer<T, N>::popBatch(std::vector<T>& out) {
    out.clear();

    auto tail = tail_.load(std::memory_order_relaxed);
    auto head = head_.load(std::memory_order_acquire);

    if (tail == head)
        return 0; // empty

    size_t count = 0;

    // First segment: from tail to end or up to head
    while (tail != head) {
        out.push_back(buffer_[tail]);
        tail = (tail + 1) % N;
        ++count;
    }

    tail_.store(tail, std::memory_order_release);
    return count;
}

// -----------------------------------------------------------------------------
// Utility methods
// -----------------------------------------------------------------------------
template <typename T, size_t N>
bool LockFreeRingBuffer<T, N>::empty() const {
    return head_.load(std::memory_order_acquire) ==
           tail_.load(std::memory_order_acquire);
}

template <typename T, size_t N>
bool LockFreeRingBuffer<T, N>::full() const {
    auto next = (head_.load(std::memory_order_relaxed) + 1) % N;
    return next == tail_.load(std::memory_order_acquire);
}

template <typename T, size_t N>
size_t LockFreeRingBuffer<T, N>::size() const {
    auto head = head_.load(std::memory_order_relaxed);
    auto tail = tail_.load(std::memory_order_relaxed);
    return (head >= tail) ? (head - tail) : (N - (tail - head));
}
