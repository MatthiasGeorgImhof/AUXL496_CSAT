#ifndef SGP4_TLE_HPP
#define SGP4_TLE_HPP

#include <cstdint>

#ifdef __linux__
#include <string>
#include <optional>
#endif // __linux__

struct SGP4TwoLineElement
{
    int32_t satelliteNumber;           // Satellite Catalog Number: up to 99999 (5 digits). Can be negative for some test satellites.
    uint16_t elementNumber;            // Element Set Number: Sequence number of the TLE.  Can be up to 9999.
    uint8_t ephemerisType;             // Ephemeris Type: Typically 0, but can be other small integers.
    uint8_t epochYear;                 // Epoch Year: Last two digits of the year (00-99).
    float epochDay;                    // Epoch Day: Day of the year (fractional). Max is < 366
    float meanMotionDerivative1;       // First Time Derivative of Mean Motion: Can be positive or negative, and very small.
    float meanMotionDerivative2;       // Second Time Derivative of Mean Motion: Can be positive or negative, and very small
    float bStarDrag;                   // BSTAR drag term: Can be positive or negative, and very small.
    float inclination;                 // Inclination: Angle in degrees (0-180).
    float rightAscensionAscendingNode; // Right Ascension of Ascending Node: Angle in degrees (0-360).
    float eccentricity;                // Eccentricity: 0.0 to < 1.0
    float argumentOfPerigee;           // Argument of Perigee: Angle in degrees (0-360)
    float meanAnomaly;                 // Mean Anomaly: Angle in degrees (0-360).
    float meanMotion;                  // Mean Motion: Revolutions per day.  Can be a wide range of values.
    int32_t revolutionNumberAtEpoch;   // Revolution Number at Epoch: Can be large.
};

#ifdef __linux__
namespace sgp4_utils
{
    std::optional<SGP4TwoLineElement> parseTLE(const std::string &l1, const std::string &l2);
}
#endif // __linux__

#endif // SGP4_TLE_HPP