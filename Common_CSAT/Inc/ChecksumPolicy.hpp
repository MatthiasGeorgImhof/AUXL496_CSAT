#ifndef __CHECKSUM_POLICY_HPP__
#define __CHECKSUM_POLICY_HPP__

#include <concepts>
#include <cstdint>

template<typename T>
concept ChecksumPolicy =
    requires(T t, const uint8_t* data, size_t size)
{
    { t.reset() } -> std::same_as<void>;
    { t.update(data, size) } -> std::same_as<void>;
    { t.get() } -> std::convertible_to<crc_t>;
};

#endif // __CHECKSUM_POLICY_HPP__