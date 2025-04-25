#pragma once
#include <cstdint>

enum class AccessError : uint32_t
{
    NO_ERROR = 0,
    WRITE_ERROR = 1,
    READ_ERROR = 2,
    OUT_OF_BOUNDS = 3,
};
