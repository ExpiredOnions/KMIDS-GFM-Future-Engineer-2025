#pragma once
#include <optional>
#include <vector>

/**
 * @brief A fixed-size circular buffer (ring buffer) template class.
 *
 * This class stores a fixed number of elements of type `T` in a circular fashion.
 * When the buffer reaches its capacity, adding a new element overwrites the oldest one.
 *
 * @tparam T The type of elements stored in the buffer.
 *
 * **Example usage:**
 * @code
 * RingBuffer<int> buffer(3);
 * buffer.push(1);
 * buffer.push(2);
 * buffer.push(3);
 * buffer.push(4); // overwrites 1
 *
 * auto latest = buffer.latest(); // returns 4
 * auto all = buffer.getAll();    // returns {2, 3, 4}
 * @endcode
 */
template <typename T>
class RingBuffer
{
public:
    /**
     * @brief Construct a ring buffer with a given capacity.
     * @param capacity The maximum number of elements the buffer can hold.
     */
    explicit RingBuffer(size_t capacity);

    /**
     * @brief Add a new element to the buffer (copy version).
     *
     * If the buffer is full, the oldest element is overwritten.
     *
     * @param item The element to be copied into the buffer.
     */
    void push(const T &item);

    /**
     * @brief Add a new element to the buffer (move version).
     *
     * If the buffer is full, the oldest element is overwritten.
     *
     * @param item The element to be moved into the buffer.
     */
    void push(T &&item);

    /**
     * @brief Retrieve the most recent element added to the buffer.
     * @return The latest element, or `std::nullopt` if the buffer is empty.
     */
    std::optional<T> latest() const;

    /**
     * @brief Get all elements from oldest to newest.
     * @return A vector containing all elements in chronological order.
     */
    std::vector<T> getAll() const;

    /**
     * @brief Get the current number of elements stored in the buffer.
     * @return Number of elements.
     */
    size_t size() const;

    /**
     * @brief Check if the buffer is empty.
     * @return `true` if no elements are stored, otherwise `false`.
     */
    bool empty() const;

    /**
     * @brief Check if the buffer is full.
     * @return `true` if the number of stored elements equals the capacity, otherwise `false`.
     */
    bool full() const;

private:
    size_t capacity_;        ///< Maximum number of elements the buffer can hold.
    std::vector<T> buffer_;  ///< Internal storage for elements.
    size_t head_;            ///< Index where the next element will be written.
    size_t size_;            ///< Current number of elements stored.
};

// ===== Definitions =====

template <typename T>
RingBuffer<T>::RingBuffer(size_t capacity)
    : capacity_(capacity)
    , buffer_(capacity)
    , head_(0)
    , size_(0) {}

template <typename T>
void RingBuffer<T>::push(const T &item) {
    buffer_[head_] = item;
    head_ = (head_ + 1) % capacity_;
    if (size_ < capacity_) ++size_;
}

template <typename T>
void RingBuffer<T>::push(T &&item) {
    buffer_[head_] = std::move(item);
    head_ = (head_ + 1) % capacity_;
    if (size_ < capacity_) ++size_;
}

template <typename T>
std::optional<T> RingBuffer<T>::latest() const {
    if (size_ == 0) return std::nullopt;
    size_t index = (head_ + capacity_ - 1) % capacity_;
    return buffer_[index];
}

template <typename T>
std::vector<T> RingBuffer<T>::getAll() const {
    std::vector<T> result;
    result.reserve(size_);
    for (size_t i = 0; i < size_; ++i) {
        size_t idx = (head_ + capacity_ - size_ + i) % capacity_;
        result.push_back(buffer_[idx]);
    }
    return result;
}

template <typename T>
size_t RingBuffer<T>::size() const {
    return size_;
}

template <typename T>
bool RingBuffer<T>::empty() const {
    return size_ == 0;
}

template <typename T>
bool RingBuffer<T>::full() const {
    return size_ == capacity_;
}
