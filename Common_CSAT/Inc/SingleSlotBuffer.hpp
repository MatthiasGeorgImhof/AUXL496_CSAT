#ifndef INC_SINGLESLOTBUFFER_HPP_
#define INC_SINGLESLOTBUFFER_HPP_

#include <cstddef>
#include <utility>
#include <type_traits>
#include "BufferLikeConcept.hpp"

template <typename T>
class SingleSlotBuffer
{
public:
    SingleSlotBuffer() : has_value_(false) {}

    T& next()
    {
        has_value_ = true;
        return slot_;
    }

    void push(T value)
    {
        slot_ = std::move(value);
        has_value_ = true;
    }

    T pop()
    {
        has_value_ = false;
        return std::move(slot_);
    }

    const T& peek() const
    {
        return slot_;
    }

    T& peek()
    {
        return slot_;
    }

    bool is_empty() const
    {
        return !has_value_;
    }

    bool is_full() const
    {
        return has_value_;
    }

    size_t size() const
    {
        return has_value_ ? 1 : 0;
    }

    size_t capacity() const
    {
        return 1;
    }

    void clear()
    {
        has_value_ = false;
        if constexpr (std::is_default_constructible_v<T>) {
            slot_ = T();
        }
    }

private:
    T slot_;
    bool has_value_;
};

static_assert(BufferLike<SingleSlotBuffer<int>, int>);

#endif /* INC_SINGLESLOTBUFFER_HPP_ */
