#ifndef IMAGE_BUFFER_ACCESSOR_H
#define IMAGE_BUFFER_ACCESSOR_H

#include <cstdint>
#include <type_traits>
#include "mock_hal.h"

enum class AccessorError : uint16_t
{
    NO_ERROR = 0,
    WRITE_ERROR = 1,
    READ_ERROR = 2,
    OUT_OF_BOUNDS = 3,
    GENERIC_ERROR = 4
};

AccessorError toAccessorError(HAL_StatusTypeDef status)
{
    return (status == HAL_OK) ? AccessorError::NO_ERROR : AccessorError::GENERIC_ERROR;
}

// Structure to hold required member function signatures
template <typename T>
struct AccessorTraits {
    using WriteFunc = AccessorError(T::*)(size_t, const uint8_t*, size_t);
    using ReadFunc = AccessorError(T::*)(size_t, uint8_t*, size_t);
    using EraseFunc = AccessorError(T::*)(size_t);
    using GetAlignmentFunc = size_t (T::*)() const;
    using GetFlashMemorySizeFunc = size_t (T::*)() const;
    using GetFlashStartAddressFunc = size_t (T::*)() const;
};

template <typename T>
concept Accessor = requires(T a, size_t address, const uint8_t* data, uint8_t* read_data, size_t size) {
    // Check for required member functions with correct signatures
    requires std::is_same_v<decltype(a.write(address, data, size)), AccessorError>;
    requires std::is_same_v<decltype(a.read(address, read_data, size)), AccessorError>;
    requires std::is_same_v<decltype(a.erase(address)), AccessorError>;
    { a.getAlignment() } -> std::convertible_to<size_t>;
    { a.getFlashMemorySize() } -> std::convertible_to<size_t>;
    { a.getFlashStartAddress() } -> std::convertible_to<size_t>;
    { a.getEraseBlockSize() } -> std::convertible_to<size_t>;
};

#endif /* IMAGE_BUFFER_ACCESSOR_H */
