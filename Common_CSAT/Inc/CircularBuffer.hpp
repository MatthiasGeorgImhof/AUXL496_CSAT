#ifndef INC_CIRCULARBUFFER_HPP_
#define INC_CIRCULARBUFFER_HPP_

#include <array>
#include <cstddef>
#include <utility>

template <typename T, size_t capacity_>
class CircularBuffer
{
public:
    CircularBuffer() : head_(0), tail_(0), count_(0){}

    T &next()
    {
        if (is_full()) drop_tail();
        T &result = data[head_];
        head_ = (head_ + 1) % capacity_;
        ++count_;
        return result;
    }

    void push(T value)
    {
        if (is_full()) drop_tail();
        data[head_] = value;
        head_ = (head_ + 1) % capacity_;
        ++count_;
    }

    T pop()
    {
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
        head_ = 0;
        tail_ = 0;
        count_ = 0; 
    }

private:
    void drop_tail()
    {
      
        if constexpr (std::is_default_constructible_v<T>) {
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
};

#endif /* INC_CIRCULARBUFFER_HPP_ */
