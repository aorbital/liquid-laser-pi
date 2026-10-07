#pragma once
#include <chrono>
#include <mutex>
#include <memory>

using Clock = std::chrono::steady_clock;

inline double seconds(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
}
inline double millis(Clock::time_point a, Clock::time_point b) {
    return seconds(a, b) * 1000.0;
}

template<class T>
class Latest {
    mutable std::mutex mutex_;
    std::shared_ptr<const T> data_;
public:
    void set(std::shared_ptr<const T> p) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_ = std::move(p);
    }
    std::shared_ptr<const T> get() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_;
    }
};
