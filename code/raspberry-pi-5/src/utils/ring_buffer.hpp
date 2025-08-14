#pragma once
#include <optional>
#include <vector>

template <typename T>
class RingBuffer
{
public:
    explicit RingBuffer(size_t capacity)
        : capacity_(capacity)
        , buffer_(capacity)
        , head_(0)
        , size_(0) {}

    // Add a new element to the buffer
    void push(const T &item) {
        buffer_[head_] = item;
        head_ = (head_ + 1) % capacity_;
        if (size_ < capacity_) ++size_;
    }

    void push(T &&item) {
        buffer_[head_] = std::move(item);
        head_ = (head_ + 1) % capacity_;
        if (size_ < capacity_) ++size_;
    }

    // Get the most recent element
    std::optional<T> latest() const {
        if (size_ == 0) return std::nullopt;
        size_t index = (head_ + capacity_ - 1) % capacity_;
        return buffer_[index];
    }

    // Get a copy of all elements in order from oldest to newest
    std::vector<T> getAll() const {
        std::vector<T> result;
        result.reserve(size_);
        for (size_t i = 0; i < size_; ++i) {
            size_t idx = (head_ + capacity_ - size_ + i) % capacity_;
            result.push_back(buffer_[idx]);
        }
        return result;
    }

    // Get current number of elements
    size_t size() const {
        return size_;
    }

    // Check if buffer is empty
    bool empty() const {
        return size_ == 0;
    }

    // Check if buffer is full
    bool full() const {
        return size_ == capacity_;
    }

private:
    size_t capacity_;
    std::vector<T> buffer_;
    size_t head_;
    size_t size_;
};
