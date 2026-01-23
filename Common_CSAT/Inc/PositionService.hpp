// PositionService.hpp

#pragma once

#include <array>
#include "au.hpp"

struct PositionSolution
{
    enum class Validity : uint8_t {
        POSITION     = 0b0001,
        VELOCITY     = 0b0010,
        ACCELERATION = 0b0100,
    };

    au::QuantityU64<au::Milli<au::Seconds>> timestamp;

    std::array<au::QuantityF<au::MetersInEcefFrame>, 3> position;
    std::array<au::QuantityF<au::MetersPerSecondInEcefFrame>, 3> velocity;
    std::array<au::QuantityF<au::MetersPerSecondSquaredInEcefFrame>, 3> acceleration;

    uint8_t validity_flags;

    bool has_valid(Validity v) const {
        return validity_flags & static_cast<uint8_t>(v);
    }
};
