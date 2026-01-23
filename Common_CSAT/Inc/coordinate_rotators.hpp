// coordinate_rotators.hpp

#pragma once

#include <concepts>
#include <optional>
#include <cmath>
#include <array>
#include "au.hpp"

#include "IMU.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"


namespace coordinate_rotators   
{
    Eigen::Matrix3f computeNEDtoECEFRotation(const std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &ecef);
} // coordinate_rotators
