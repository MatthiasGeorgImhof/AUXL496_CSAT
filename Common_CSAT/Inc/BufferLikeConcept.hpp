// BufferLikeConcept.hpp

#pragma once

#include <concepts>
#include <cstddef>
#include <utility>


template <typename B, typename T>
concept BufferLike = requires(B buffer, T value) {
    { buffer.push(value) } -> std::same_as<void>;
    { buffer.pop() } -> std::same_as<T>;
    { buffer.peek() } -> std::same_as<T&>;
    { std::as_const(buffer).peek() } -> std::same_as<const T&>;
    { buffer.next() } -> std::same_as<T&>;
    { buffer.is_empty() } -> std::same_as<bool>;
    { buffer.is_full() } -> std::same_as<bool>;
    { buffer.size() } -> std::convertible_to<size_t>;
    { buffer.capacity() } -> std::convertible_to<size_t>;
    { buffer.clear() } -> std::same_as<void>;
};
