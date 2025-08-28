#ifndef COORDINATE_TRANSFORMATIONS_HPP
#define COORDINATE_TRANSFORMATIONS_HPP

#include <cmath>
#include <array>
#include "au.hpp"

namespace coordinate_transformations
{

    // --- Constants ---
    constexpr float DEG_TO_RAD = M_PIf / 180.0f; // Conversion factor from degrees to radians
    constexpr float RAD_TO_DEG = 180.0f / M_PIf; // Conversion factor from radians to degrees

    // --- Constants for WGS84 ---
    constexpr float WGS84_A = 6378137.0f;                       // Semi-major axis
    constexpr float WGS84_F = 1.0f / 298.257223563f;            // Flattening
    constexpr float WGS84_B = WGS84_A * (1.0f - WGS84_F);       // Semi-minor axis
    constexpr float WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // Eccentricity squared

    // --- Coordinate System Structs ---

    // Geodetic Coordinates (Latitude, Longitude, Height)
    struct Geodetic
    {
        au::QuantityF<au::DegreesInGeodeticFrame> latitude;  // Latitude in degrees
        au::QuantityF<au::DegreesInGeodeticFrame> longitude;  // Longitude in degrees
        au::QuantityF<au::MetersInGeodeticFrame> height;     // Height above ellipsoid in meters
    };

    // Geocentric Coordinates (Latitude, Longitude, Radius)
    struct Geocentric
    {
        au::QuantityF<au::DegreesInGeocentricFrame> latitude;  // Latitude in degrees
        au::QuantityF<au::DegreesInGeocentricFrame> longitude; // Longitude in degrees
        au::QuantityF<au::MetersInGeocentricFrame> radius;     // Radius in meters
    };

    // ECEF Coordinates (X, Y, Z)
    struct ECEF
    {
        au::QuantityF<au::MetersInEcefFrame> x; // X coordinate in meters
        au::QuantityF<au::MetersInEcefFrame> y; // Y coordinate in meters
        au::QuantityF<au::MetersInEcefFrame> z; // Z coordinate in meters
    };

    // TEME Coordinates (X, Y, Z)
    struct TEME
    {
        au::QuantityF<au::MetersInTemeFrame> x; // X coordinate in meters
        au::QuantityF<au::MetersInTemeFrame> y; // Y coordinate in meters
        au::QuantityF<au::MetersInTemeFrame> z; // Z coordinate in meters
    };

    // TEME Coordinates (X, Y, Z)
    struct NED
    {
        au::QuantityF<au::MetersInNedFrame> x; // X coordinate in meters
        au::QuantityF<au::MetersInNedFrame> y; // Y coordinate in meters
        au::QuantityF<au::MetersInNedFrame> z; // Z coordinate in meters
    };

    // --- Coordinate Conversion Functions ---

    ECEF geodeticToECEF(Geodetic geodetic);
    Geodetic ecefToGeodetic(ECEF ecef);

    Geocentric geodeticToGeocentric(Geodetic geodetic);
    Geodetic geocentricToGeodetic(Geocentric geocentric);

    ECEF temeToECEF(TEME teme, float jd2000);
    TEME ecefToTEME(ECEF ecef, float jd2000);

    std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>,3> temeToecef(const std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>,3> teme, float jd2000);
    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>,3> temeToecef(const std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>,3> teme, float jd2000);
    std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>,3> ecefToteme(const std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>,3> teme, float jd2000);
    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>,3> ecefToteme(const std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>,3> teme, float jd2000);

    struct PolarMotion { float x, y; };
    PolarMotion polarmMJD2000(float jd2000, float pm[3][3]);

} /* namespace coordinate_transformations */

#endif // COORDINATE_TRANSFORMATIONS_HPP