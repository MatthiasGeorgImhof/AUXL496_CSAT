// coordinate_rotators.cpp

#include <concepts>
#include <optional>
#include <cmath>
#include <array>
#include "au.hpp"

#include "IMU.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "coordinate_rotators.hpp"
#include "coordinate_transformations.hpp"

namespace coordinate_rotators
{
    Eigen::Matrix3f computeNEDtoECEFRotation(const std::array<au::QuantityF<au::MetersInEcefFrame>, 3> &ecef)
    {
        // Convert ECEF to geodetic (lat, lon)
        coordinate_transformations::ECEF ecef_ = {ecef[0], ecef[1], ecef[2]};
        coordinate_transformations::Geodetic geo = coordinate_transformations::ecefToGeodetic(ecef_);

        float lat = geo.latitude.in(au::radiansInGeodeticFrame);
        float lon = geo.longitude.in(au::radiansInGeodeticFrame);

        float sin_lat = std::sin(lat);
        float cos_lat = std::cos(lat);
        float sin_lon = std::sin(lon);
        float cos_lon = std::cos(lon);

        // NED to ECEF rotation matrix
        Eigen::Matrix3f R;
        R << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,
            -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,
            cos_lat, 0.f, -sin_lat;

        return R;
    }
}

// coordinate_rotators
