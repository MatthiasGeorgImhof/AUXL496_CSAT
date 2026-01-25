#ifndef INC_CIRCULARBUFFER_HPP_
#define INC_CIRCULARBUFFER_HPP_

#include <array>
#include <cstddef>
#include <utility>
#include "BufferLikeConcept.hpp"

template <typename T, size_t capacity_>
class CircularBuffer
{
public:
    CircularBuffer() : head_(0), tail_(0), count_(0) {}

    T &next()
    {
        if (is_full())
            drop_tail();
        T &result = data[head_];
        head_ = (head_ + 1) % capacity_;
        ++count_;
        return result;
    }

    void push(T value)
    {
        if (is_full())
            drop_tail();
        data[head_] = value;
        head_ = (head_ + 1) % capacity_;
        ++count_;
    }

    T pop()
    {
        if (is_empty())
        {
            return T{};
        }
        size_t current = tail_;
        tail_ = (tail_ + 1) % capacity_;
        --count_;
        return std::move(data[current]);
    }

    const T &peek() const
    {
        size_t current = tail_;
        return data[current];
    }

    T &peek()
    {
        size_t current = tail_;
        return data[current];
    }

    T& begin_write()
    {
        // If full, drop oldest
        if (is_full()) {
            drop_tail();
        }

        return data[head_];     // caller gives this to DMA
    }

    void commit_write()
    {
        // Move head to write_index_ + 1
        head_ = (head_ + 1) % capacity_;
        ++count_;
    }

    bool is_empty() const
    {
        return count_ == 0;
    }

    bool is_full() const
    {
        return count_ == capacity_;
    }

    size_t size() const
    {
        return count_;
    }

    size_t capacity() const
    {
        return capacity_;
    }

    void clear()
    {
        if constexpr (std::is_default_constructible_v<T>)
        {
            for (size_t i = 0; i < count_; ++i)
            {
                data[i] = T(); // reset shared_ptr → release references
            }
        }
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    void drop_tail()
    {

        if constexpr (std::is_default_constructible_v<T>)
        {
            data[tail_] = T();
        }
        tail_ = (tail_ + 1) % capacity_;
        --count_;
    }

private:
    std::array<T, capacity_> data;
    size_t head_;
    size_t tail_;
    size_t count_;

    static_assert(capacity_ > 0, "CircularBuffer capacity must be greater than zero.");
    static_assert(std::is_default_constructible_v<T>, "CircularBuffer<T>: T must be default-constructible");
    static_assert(std::is_copy_assignable_v<T>, "CircularBuffer<T>: T must be copy-assignable");
    static_assert(std::is_move_assignable_v<T>, "CircularBuffer<T>: T must be move-assignable");
};

static_assert(BufferLike<CircularBuffer<int, 8>, int>);

#endif /* INC_CIRCULARBUFFER_HPP_ */
