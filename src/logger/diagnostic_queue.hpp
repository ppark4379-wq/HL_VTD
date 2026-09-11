#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>

// Single producer (controller), single consumer (writer). Slots are allocated
// before driving. No mutex, allocation, retry, or I/O in reserve/commit.
template <typename T, std::size_t Capacity>
struct DiagnosticQueue {
    std::unique_ptr<std::array<T, Capacity>> slots =
        std::make_unique<std::array<T, Capacity>>();
    alignas(64) std::atomic<std::size_t> written{0};
    alignas(64) std::atomic<std::size_t> read{0};
    std::atomic<std::size_t> dropped{0};
};

static_assert(std::atomic<std::size_t>::is_always_lock_free,
              "diagnostics require lock-free queue indices");

template <typename T, std::size_t Capacity>
T* reserveDiagnostic(DiagnosticQueue<T, Capacity>& queue) noexcept {
    const auto written = queue.written.load(std::memory_order_relaxed);
    if (written - queue.read.load(std::memory_order_acquire) >= Capacity) {
        queue.dropped.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }
    return &(*queue.slots)[written % Capacity];
}

template <typename T, std::size_t Capacity>
void commitDiagnostic(DiagnosticQueue<T, Capacity>& queue) noexcept {
    queue.written.fetch_add(1, std::memory_order_release);
}

template <typename T, std::size_t Capacity>
const T* peekDiagnostic(const DiagnosticQueue<T, Capacity>& queue) noexcept {
    const auto read = queue.read.load(std::memory_order_relaxed);
    if (read == queue.written.load(std::memory_order_acquire)) return nullptr;
    return &(*queue.slots)[read % Capacity];
}

template <typename T, std::size_t Capacity>
void releaseDiagnostic(DiagnosticQueue<T, Capacity>& queue) noexcept {
    queue.read.fetch_add(1, std::memory_order_release);
}
