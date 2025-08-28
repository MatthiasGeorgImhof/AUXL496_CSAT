#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "coordinate_transformations.hpp"
#include "TimeUtils.hpp"
#include <random>
#include <limits> // For NaN
#include <cmath>  // For std::isnan
#include "au.hpp"

using namespace coordinate_transformations;

uint32_t getExponent(float f)
{
    if (std::isnan(f) || std::isinf(f))
    {
        return std::numeric_limits<int32_t>::quiet_NaN(); // Or some other appropriate value for special cases
    }

    uint32_t floatBits = 0;
    static_assert(sizeof(float) == sizeof(uint32_t), "Size mismatch between float and uint32_t");
    std::memcpy(&floatBits, &f, sizeof(float)); // Copy the float's bits to the integer

    // Extract the exponent bits (bits 23-30) and shift them to the right
    uint32_t exponentBits = (floatBits >> 23) & 0xFF;
    return exponentBits;

    //   // Subtract the bias (127) to get the actual exponent
    //   int32_t exponent = static_cast<int32_t>(exponentBits) - 127;
    //   return exponent;
}

float resolution(float f)
{
    uint32_t exponent = getExponent(f);
    if (std::isnan(f) || std::isinf(f))
    {
        return std::numeric_limits<float>::quiet_NaN(); // Or some other appropriate value for special cases
    }

    return std::pow(2.0f, static_cast<float>(exponent - 150));
}

TEST_CASE("ECEF to Geodetic Conversion - Iterative")
{
    SUBCASE("Equator, Sea Level")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(WGS84_A), au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(0.0)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-3));
    }

    SUBCASE("Equator, Sea Level but somewhere else")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(WGS84_A / sqrtf(2.f)), au::make_quantity<au::MetersInEcefFrame>(WGS84_A / sqrtf(2.f)), au::make_quantity<au::MetersInEcefFrame>(0.0)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(45.0).epsilon(1e-8));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-3));
    }

    SUBCASE("Equator, Sea Level + 1000")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(WGS84_A + 1000.f), au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(0.0)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(1000.0).epsilon(1e-3));
    }

    SUBCASE("Equator, Sea Level - 10000")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(WGS84_A - 10000.f), au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(0.0)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(-10000.0).epsilon(1e-3));
    }

    SUBCASE("North Pole, Sea Level")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(WGS84_B)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(90.0).epsilon(1e-8));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-3));
    }

    SUBCASE("North Pole, Sea Level + 12000")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(WGS84_B + 12000.0f)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(90.0).epsilon(1e-8));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(12000.0).epsilon(1e-3));
    }

    SUBCASE("North Pole, Sea Level - 4000")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(0.0), au::make_quantity<au::MetersInEcefFrame>(WGS84_B - 4000.0f)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(90.0).epsilon(1e-8));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0).epsilon(1e-8));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(-4000.0).epsilon(1e-3));
    }

    SUBCASE("45 degrees Latitude, 30 degrees Longitude, 100km Altitude")
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(30.0f), au::make_quantity<au::MetersInGeodeticFrame>(100000.0f)};
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(45.0).epsilon(1e-4));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(30.0).epsilon(1e-4));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(100000.0).epsilon(1e-3));
    }

    SUBCASE("Negative Latitude and Longitude, High Altitude")
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(-30.0f), au::make_quantity<au::DegreesInGeodeticFrame>(-60.0f), au::make_quantity<au::MetersInGeodeticFrame>(100000.0f)};
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(-30.0).epsilon(1e-4));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(-60.0).epsilon(1e-4));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(100000.0).epsilon(1e-3));
    }
}

TEST_CASE("Geodetic to Geocentric Conversion")
{
    SUBCASE("Equator, Sea Level")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(geocentric.latitude.in(au::degreesInGeocentricFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geocentric.radius.in(au::metersInGeocentricFrame) == doctest::Approx(WGS84_A).epsilon(1e-6f));
    }

    SUBCASE("North Pole, Sea Level")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(90.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(geocentric.latitude.in(au::degreesInGeocentricFrame) == doctest::Approx(90.0f).epsilon(1e-6f));
        CHECK(geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geocentric.radius.in(au::metersInGeocentricFrame) == doctest::Approx(WGS84_B).epsilon(1e-6f));
    }

    SUBCASE("South Pole, Sea Level")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(-90.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(geocentric.latitude.in(au::degreesInGeocentricFrame) == doctest::Approx(-90.0f).epsilon(1e-6f));
        CHECK(geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geocentric.radius.in(au::metersInGeocentricFrame) == doctest::Approx(WGS84_B).epsilon(1e-6f));
    }

    SUBCASE("45 degrees Latitude, 30 degrees Longitude, 100km Altitude")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(30.0f), au::make_quantity<au::MetersInGeodeticFrame>(100000.0f)}; // 100km == 100000m
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(geocentric.radius.in(au::metersInGeocentricFrame) > WGS84_A);
        CHECK(geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(30.0f).epsilon(1e-6f));
    }

    SUBCASE("Negative Latitude and Longitude, High Altitude")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(-30.0f), au::make_quantity<au::DegreesInGeodeticFrame>(-60.0f), au::make_quantity<au::MetersInGeodeticFrame>(100000.0f)}; // 100km == 100000m
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(geocentric.radius.in(au::metersInGeocentricFrame) > WGS84_A);
        CHECK(geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(-60.0f).epsilon(1e-6f));
    }

    SUBCASE("Invalid Latitude - Returns NaN")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(100.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(std::isnan(geocentric.latitude.in(au::degreesInGeocentricFrame)));
        CHECK(std::isnan(geocentric.longitude.in(au::degreesInGeocentricFrame)));
        CHECK(std::isnan(geocentric.radius.in(au::metersInGeocentricFrame)));
    }

    SUBCASE("Large Positive Height")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(30.0f), au::make_quantity<au::MetersInGeodeticFrame>(10000000.0f)};        // 10,000 km
        Geocentric geocentric = geodeticToGeocentric(geodetic); // Should still be valid
        CHECK(std::isfinite(geocentric.latitude.in(au::degreesInGeocentricFrame)));
        CHECK(std::isfinite(geocentric.longitude.in(au::degreesInGeocentricFrame)));
        CHECK(std::isfinite(geocentric.radius.in(au::metersInGeocentricFrame)));
    }

    SUBCASE("Large Negative Height (Below Ellipsoid)")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(30.0f), au::make_quantity<au::MetersInGeodeticFrame>(-10000.0f)};          // -10km
        Geocentric geocentric = geodeticToGeocentric(geodetic); // Still valid
        CHECK(std::isfinite(geocentric.latitude.in(au::degreesInGeocentricFrame)));
        CHECK(std::isfinite(geocentric.longitude.in(au::degreesInGeocentricFrame)));
        CHECK(std::isfinite(geocentric.radius.in(au::metersInGeocentricFrame)));
    }

    SUBCASE("Longitude at 180")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(180.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(geocentric.latitude.in(au::degreesInGeocentricFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(180.0f).epsilon(1e-6f));
    }

    SUBCASE("Longitude at -180")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(-180.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        Geocentric geocentric = geodeticToGeocentric(geodetic);
        CHECK(geocentric.latitude.in(au::degreesInGeocentricFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(-180.0f).epsilon(1e-6f));
    }
}

TEST_CASE("Geocentric to Geodetic Conversion")
{
    SUBCASE("Earth Center - Returns NaN")
    {
        Geocentric geocentric = {au::make_quantity<au::DegreesInGeocentricFrame>(0.0f), au::make_quantity<au::DegreesInGeocentricFrame>(0.0f), au::make_quantity<au::MetersInGeocentricFrame>(0.0f)}; // Invalid as there is no coordinate for this location.
        Geodetic geodetic = geocentricToGeodetic(geocentric);
        CHECK(std::isnan(geodetic.latitude.in(au::degreesInGeodeticFrame)));
        CHECK(std::isnan(geodetic.longitude.in(au::degreesInGeodeticFrame)));
        CHECK(std::isnan(geodetic.height.in(au::metersInGeodeticFrame)));
    }

    SUBCASE("Equator")
    {
        Geocentric geocentric = {au::make_quantity<au::DegreesInGeocentricFrame>(0.0f), au::make_quantity<au::DegreesInGeocentricFrame>(0.0f), au::make_quantity<au::MetersInGeocentricFrame>(WGS84_A)};

        // Calculate radius
        float x = geocentric.radius.in(au::metersInGeocentricFrame)*cosf(geocentric.latitude.in(au::degreesInGeocentricFrame)) * cosf(geocentric.longitude.in(au::degreesInGeocentricFrame));
        float y = geocentric.radius.in(au::metersInGeocentricFrame)*cosf(geocentric.latitude.in(au::degreesInGeocentricFrame)) * sinf(geocentric.longitude.in(au::degreesInGeocentricFrame));
        float z = geocentric.radius.in(au::metersInGeocentricFrame)*sinf(geocentric.latitude.in(au::degreesInGeocentricFrame));
        geocentric.radius = au::make_quantity<au::MetersInGeocentricFrame>(sqrtf(x * x + y * y + z * z));

        Geodetic geodetic = geocentricToGeodetic(geocentric);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(-WGS84_A + geocentric.radius.in(au::metersInGeocentricFrame)).epsilon(1e-6f));
    }

    SUBCASE("Equator With Longitude")
    {
        Geocentric geocentric = {au::make_quantity<au::DegreesInGeocentricFrame>(0.0f), au::make_quantity<au::DegreesInGeocentricFrame>(30.0f), au::make_quantity<au::MetersInGeocentricFrame>(WGS84_A)};
        // Calculate radius
        float x = geocentric.radius.in(au::metersInGeocentricFrame)*cosf(geocentric.latitude.in(au::degreesInGeocentricFrame)) * cosf(geocentric.longitude.in(au::degreesInGeocentricFrame));
        float y = geocentric.radius.in(au::metersInGeocentricFrame)*cosf(geocentric.latitude.in(au::degreesInGeocentricFrame)) * sinf(geocentric.longitude.in(au::degreesInGeocentricFrame));
        float z = geocentric.radius.in(au::metersInGeocentricFrame)*sinf(geocentric.latitude.in(au::degreesInGeocentricFrame));
        geocentric.radius = au::make_quantity<au::MetersInGeocentricFrame>(sqrtf(x * x + y * y + z * z));

        Geodetic geodetic = geocentricToGeodetic(geocentric);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-6f));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(30.0f).epsilon(1e-6f));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(-WGS84_A + geocentric.radius.in(au::metersInGeocentricFrame)).epsilon(1e-6f));
    }

    SUBCASE("Non-Zero Geocentric Latitude")
    {
        Geocentric geocentric = {au::make_quantity<au::DegreesInGeocentricFrame>(30.0f), au::make_quantity<au::DegreesInGeocentricFrame>(45.0f), au::make_quantity<au::MetersInGeocentricFrame>(5000000.0f)}; // radius in meters
        Geodetic geodetic = geocentricToGeodetic(geocentric);
        // Further checks here would be ideal if you have reference values
        CHECK(std::isfinite(geodetic.latitude.in(au::degreesInGeodeticFrame)));
        CHECK(std::isfinite(geodetic.longitude.in(au::degreesInGeodeticFrame)));
        CHECK(std::isfinite(geodetic.height.in(au::metersInGeodeticFrame)));
    }

    SUBCASE("Geocentric Latitude near 90 - Radiius zero")
    {
        Geocentric geocentric = {au::make_quantity<au::DegreesInGeocentricFrame>(89.99999f), au::make_quantity<au::DegreesInGeocentricFrame>(45.0f), au::make_quantity<au::MetersInGeocentricFrame>(0.0000001f)}; // radius in meters

        Geodetic geodetic = geocentricToGeodetic(geocentric);
        // Should Return NAN if it cannot handle these values.
        CHECK(std::isfinite(geodetic.latitude.in(au::degreesInGeodeticFrame)));
        CHECK(std::isfinite(geodetic.longitude.in(au::degreesInGeodeticFrame)));
        CHECK(std::isfinite(geodetic.height.in(au::metersInGeodeticFrame)));
    }
}

TEST_CASE("Geodetic to ECEF Conversion")
{
    SUBCASE("Equator, Sea Level")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        ECEF ecef = geodeticToECEF(geodetic);
        CHECK(ecef.x.in(au::metersInEcefFrame) == doctest::Approx(WGS84_A).epsilon(1e-5f));
        CHECK(ecef.y.in(au::metersInEcefFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(ecef.z.in(au::metersInEcefFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
    }

    SUBCASE("North Pole, Sea Level")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(90.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        ECEF ecef = geodeticToECEF(geodetic);
        CHECK(ecef.x.in(au::metersInEcefFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(ecef.y.in(au::metersInEcefFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(ecef.z.in(au::metersInEcefFrame) == doctest::Approx(WGS84_B).epsilon(1e-5f));
    }

    SUBCASE("Invalid Latitude - Returns NaN")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(100.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        ECEF ecef = geodeticToECEF(geodetic);
        CHECK(std::isnan(ecef.x.in(au::metersInEcefFrame)));
        CHECK(std::isnan(ecef.y.in(au::metersInEcefFrame)));
        CHECK(std::isnan(ecef.z.in(au::metersInEcefFrame)));
    }

    SUBCASE("Prime Meridian, Sea Level")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        ECEF ecef = geodeticToECEF(geodetic);
        CHECK(ecef.x.in(au::metersInEcefFrame) == doctest::Approx(WGS84_A).epsilon(1e-5f));
        CHECK(ecef.y.in(au::metersInEcefFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(ecef.z.in(au::metersInEcefFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
    }

    SUBCASE("High Altitude")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::MetersInGeodeticFrame>(1000000.0f)}; // 1000km above surface
        ECEF ecef = geodeticToECEF(geodetic);
        CHECK(std::isfinite(ecef.x.in(au::metersInEcefFrame)));
        CHECK(std::isfinite(ecef.y.in(au::metersInEcefFrame)));
        CHECK(std::isfinite(ecef.z.in(au::metersInEcefFrame)));
    }

    SUBCASE("Longitude near 180")
    {
        Geodetic geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(179.9999f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        ECEF ecef = geodeticToECEF(geodetic);
        CHECK(!std::isnan(ecef.x.in(au::metersInEcefFrame)));
        CHECK(!std::isnan(ecef.y.in(au::metersInEcefFrame)));
        CHECK(!std::isnan(ecef.z.in(au::metersInEcefFrame)));
    }
}

TEST_CASE("ECEF to Geodetic Conversion")
{
    SUBCASE("Equator, Sea Level")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(WGS84_A), au::make_quantity<au::MetersInEcefFrame>(0.0f), au::make_quantity<au::MetersInEcefFrame>(0.0f)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
    }

    SUBCASE("North Pole, Sea Level")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(0.0f), au::make_quantity<au::MetersInEcefFrame>(0.0f), au::make_quantity<au::MetersInEcefFrame>(WGS84_B)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(90.0f).epsilon(1e-5f));
        CHECK(geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
        CHECK(geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
    }

    SUBCASE("45 degrees Latitude, 30 degrees Longitude, 100km Altitude")
    {
        // First, convert from geodetic to ECEF to get the expected ECEF values
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(30.0f), au::make_quantity<au::MetersInGeodeticFrame>(100000.0f)};
        ECEF ecef = geodeticToECEF(initial_geodetic);

        // Then, convert back from ECEF to geodetic and check the values
        Geodetic final_geodetic = ecefToGeodetic(ecef);

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(45.0f).epsilon(1e-4f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(30.0f).epsilon(1e-4f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(100000.0f).epsilon(1e-4f));
    }

    SUBCASE("Close to non-convergence")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(1.0), au::make_quantity<au::MetersInEcefFrame>(2.0), au::make_quantity<au::MetersInEcefFrame>(3.0)};
        Geodetic geodetic = ecefToGeodetic(ecef);

        INFO("geodetic.latitude.in(au::degreesInGeodeticFrame) ", geodetic.latitude.in(au::degreesInGeodeticFrame));
        INFO("geodetic.longitude.in(au::degreesInGeodeticFrame) ", geodetic.longitude.in(au::degreesInGeodeticFrame));
        INFO("geodetic.height.in(au::metersInGeodeticFrame) ", geodetic.height.in(au::metersInGeodeticFrame));

        CHECK_FALSE(std::isnan(geodetic.latitude.in(au::degreesInGeodeticFrame)));
        CHECK_FALSE(std::isnan(geodetic.longitude.in(au::degreesInGeodeticFrame)));
        CHECK_FALSE(std::isnan(geodetic.height.in(au::metersInGeodeticFrame)));
    }

    SUBCASE("Z axis close to zero")
    {
        ECEF ecef = {au::make_quantity<au::MetersInEcefFrame>(6378137.0f), au::make_quantity<au::MetersInEcefFrame>(0.0f), au::make_quantity<au::MetersInEcefFrame>(0.0f)};
        Geodetic geodetic = ecefToGeodetic(ecef);
        CHECK(geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(0.0f).epsilon(1e-5f));
    }
}

TEST_CASE("Round-Trip Geodetic -> ECEF -> Geodetic")
{
    SUBCASE("Positive Values")
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(-120.0f), au::make_quantity<au::MetersInGeodeticFrame>(500.0f)};
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic final_geodetic = ecefToGeodetic(ecef);
        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(1e-4f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(1e-4f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1e-4f));
    }

    SUBCASE("Negative Values and Sea Level")
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(-30.0f), au::make_quantity<au::DegreesInGeodeticFrame>(60.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic final_geodetic = ecefToGeodetic(ecef);

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(1e-4f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(1e-4f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1e-0f));
    }

    SUBCASE("Positive Values and Sea Level")
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(30.0f), au::make_quantity<au::DegreesInGeodeticFrame>(60.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic final_geodetic = ecefToGeodetic(ecef);
        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(1e-4f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(1e-4f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1e-0f));
    }

    SUBCASE("Near-Singularity Case (Equator, Height = -6300000)")
    {
        // This tests a case where the point is very close to the center of the Earth,
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(-6300000.f)};
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic final_geodetic = ecefToGeodetic(ecef);

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(1e-3f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(1e-3f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1e-3f));
    }

    SUBCASE("Near-Singularity Case (Equator, Height = -N)")
    {
        // This tests a case where the point is very close to the center of the Earth,
        // but still technically valid.
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::DegreesInGeodeticFrame>(0.0f), au::make_quantity<au::MetersInGeodeticFrame>(-6356752.314245f)}; // Height = -N, where N is the radius of curvature
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic final_geodetic = ecefToGeodetic(ecef);

        INFO("geodetic.latitude.in(au::degreesInGeodeticFrame) ", final_geodetic.latitude.in(au::degreesInGeodeticFrame));
        INFO("geodetic.longitude.in(au::degreesInGeodeticFrame) ", final_geodetic.longitude.in(au::degreesInGeodeticFrame));
        INFO("geodetic.height.in(au::metersInGeodeticFrame) ", final_geodetic.height.in(au::metersInGeodeticFrame));

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(1e-3f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(1e-3f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1e-3f));
    }
}

TEST_CASE("Round-Trip Conversion")
{
    SUBCASE("Positive Values")
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(45.0f), au::make_quantity<au::DegreesInGeodeticFrame>(-120.0f), au::make_quantity<au::MetersInGeodeticFrame>(500.0f)};
        Geocentric geocentric = geodeticToGeocentric(initial_geodetic);
        Geodetic final_geodetic = geocentricToGeodetic(geocentric);

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(1e-5f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(1e-5f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1e-5f));
    }

    SUBCASE("Negative Values and Sea Level")
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(-30.0f), au::make_quantity<au::DegreesInGeodeticFrame>(60.0f), au::make_quantity<au::MetersInGeodeticFrame>(0.0f)};
        Geocentric geocentric = geodeticToGeocentric(initial_geodetic);
        Geodetic final_geodetic = geocentricToGeodetic(geocentric);

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(1e-5f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(1e-5f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1e-5f));
    }
}

TEST_CASE("Random Geodetic to Geocentric")
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> lat_dist(-90.0f, 90.0f);
    std::uniform_real_distribution<float> lon_dist(-180.0f, 180.0f);
    std::uniform_real_distribution<float> alt_dist(0.0f, 1000.0f); // Altitude in km

    for (int i = 0; i < 50; ++i)
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(lat_dist(gen)), au::make_quantity<au::DegreesInGeodeticFrame>(lon_dist(gen)), au::make_quantity<au::MetersInGeodeticFrame>(alt_dist(gen) * 1000.0f)}; // Altitude in meters
        Geocentric geocentric = geodeticToGeocentric(initial_geodetic);

        INFO("initial_geodetic.latitude.in(au::degreesInGeodeticFrame) ", initial_geodetic.latitude.in(au::degreesInGeodeticFrame));
        INFO("initial_geodetic.longitude.in(au::degreesInGeodeticFrame) ", initial_geodetic.longitude.in(au::degreesInGeodeticFrame));
        INFO("initial_geodetic.height.in(au::metersInGeodeticFrame) ", initial_geodetic.height.in(au::metersInGeodeticFrame));

        CHECK_FALSE(doctest::IsNaN(geocentric.latitude.in(au::degreesInGeocentricFrame)));
        CHECK_FALSE(doctest::IsNaN(geocentric.longitude.in(au::degreesInGeocentricFrame)));
        CHECK_FALSE(doctest::IsNaN(geocentric.radius.in(au::metersInGeocentricFrame)));
    }
}

TEST_CASE("Random Geodetic to Geocentric and Back")
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> lat_dist(-90.0f, 90.0f);
    std::uniform_real_distribution<float> lon_dist(-180.0f, 180.0f);
    std::uniform_real_distribution<float> alt_dist(0.0f, 1000.0f); // Altitude in km

    for (int i = 0; i < 50; ++i)
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(lat_dist(gen)), au::make_quantity<au::DegreesInGeodeticFrame>(lon_dist(gen)), au::make_quantity<au::MetersInGeodeticFrame>(alt_dist(gen) * 1000.0f)}; // Altitude in meters
        Geocentric geocentric = geodeticToGeocentric(initial_geodetic);
        Geodetic final_geodetic = geocentricToGeodetic(geocentric);

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(0.1f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(0.1f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1000.0f));
    }
}

TEST_CASE("Random Geocentric to Geodetic")
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> lat_dist(-90.0f, 90.0f);
    std::uniform_real_distribution<float> lon_dist(-180.0f, 180.0f);
    std::uniform_real_distribution<float> rad_dist(6470.0f, 7470.0f); // radius in km

    for (int i = 0; i < 50; ++i)
    {
        Geocentric initial_geocentric = {au::make_quantity<au::DegreesInGeocentricFrame>(lat_dist(gen)), au::make_quantity<au::DegreesInGeocentricFrame>(lon_dist(gen)), au::make_quantity<au::MetersInGeocentricFrame>(rad_dist(gen) * 1000.0f)}; // Altitude in meters
        Geodetic geodetic = geocentricToGeodetic(initial_geocentric);

        INFO("initial_geocentric.latitude.in(au::degreesInGeocentricFrame) ", initial_geocentric.latitude.in(au::degreesInGeocentricFrame));
        INFO("initial_geocentric.longitude.in(au::degreesInGeocentricFrame) ", initial_geocentric.longitude.in(au::degreesInGeocentricFrame));
        INFO("initial_geocentric.radius.in(au::metersInGeocentricFrame) ", initial_geocentric.radius.in(au::metersInGeocentricFrame));

        CHECK_FALSE(doctest::IsNaN(geodetic.latitude.in(au::degreesInGeodeticFrame)));
        CHECK_FALSE(doctest::IsNaN(geodetic.longitude.in(au::degreesInGeodeticFrame)));
        CHECK_FALSE(doctest::IsNaN(geodetic.height.in(au::metersInGeodeticFrame)));
    }
}

TEST_CASE("Random Geocentric to Geodetic and Back")
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> lat_dist(-90.0f, 90.0f);
    std::uniform_real_distribution<float> lon_dist(-180.0f, 180.0f);
    std::uniform_real_distribution<float> rad_dist(6470.0f, 7470.0f); // radius in km

    for (int i = 0; i < 50; ++i)
    {
        Geocentric initial_geocentric = {au::make_quantity<au::DegreesInGeocentricFrame>(lat_dist(gen)), au::make_quantity<au::DegreesInGeocentricFrame>(lon_dist(gen)), au::make_quantity<au::MetersInGeocentricFrame>(rad_dist(gen) * 1000.0f)}; // Altitude in meters
        Geodetic geodetic = geocentricToGeodetic(initial_geocentric);
        Geocentric final_geocentric = geodeticToGeocentric(geodetic);

        CHECK(final_geocentric.latitude.in(au::degreesInGeocentricFrame) == doctest::Approx(initial_geocentric.latitude.in(au::degreesInGeocentricFrame)).epsilon(0.1f));
        CHECK(final_geocentric.longitude.in(au::degreesInGeocentricFrame) == doctest::Approx(initial_geocentric.longitude.in(au::degreesInGeocentricFrame)).epsilon(0.1f));
        CHECK(final_geocentric.radius.in(au::metersInGeocentricFrame) == doctest::Approx(initial_geocentric.radius.in(au::metersInGeocentricFrame)).epsilon(1000.0f));
    }
}

TEST_CASE("Random Geodetic to ECEF and Back")
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> lat_dist(-90.0f, 90.0f);
    std::uniform_real_distribution<float> lon_dist(-180.0f, 180.0f);
    std::uniform_real_distribution<float> alt_dist(0.0f, 1000.0f);

    for (int i = 0; i < 50; ++i)
    {
        Geodetic initial_geodetic = {au::make_quantity<au::DegreesInGeodeticFrame>(lat_dist(gen)), au::make_quantity<au::DegreesInGeodeticFrame>(lon_dist(gen)), au::make_quantity<au::MetersInGeodeticFrame>(alt_dist(gen) * 1000.0f)}; // Altitude in meters
        ECEF ecef = geodeticToECEF(initial_geodetic);
        Geodetic final_geodetic = ecefToGeodetic(ecef);

        INFO("initial_geodetic.latitude.in(au::degreesInGeodeticFrame) ", initial_geodetic.latitude.in(au::degreesInGeodeticFrame));
        INFO("initial_geodetic.longitude.in(au::degreesInGeodeticFrame) ", initial_geodetic.longitude.in(au::degreesInGeodeticFrame));
        INFO("initial_geodetic.height.in(au::metersInGeodeticFrame) ", initial_geodetic.height.in(au::metersInGeodeticFrame));
        INFO("ecef.x.in(au::metersInEcefFrame), ", ecef.x.in(au::metersInEcefFrame));
        INFO("ecef.y.in(au::metersInEcefFrame), ", ecef.y.in(au::metersInEcefFrame));
        INFO("ecef.z.in(au::metersInEcefFrame), ", ecef.z.in(au::metersInEcefFrame));

        CHECK(final_geodetic.latitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.latitude.in(au::degreesInGeodeticFrame)).epsilon(0.1f));
        CHECK(final_geodetic.longitude.in(au::degreesInGeodeticFrame) == doctest::Approx(initial_geodetic.longitude.in(au::degreesInGeodeticFrame)).epsilon(0.1f));
        CHECK(final_geodetic.height.in(au::metersInGeodeticFrame) == doctest::Approx(initial_geodetic.height.in(au::metersInGeodeticFrame)).epsilon(1000.0f));
    }
}

// Add new tests for temeToECEF and ecefToTEME
TEST_CASE("TEME to ECEF and ECEF to TEME Conversions")
{
    // Define a Julian Date for the conversion
    //  float jdut1 = 2458863.0f; // Example Julian Date
    constexpr float jdut1 = 9336.0f; // Example J2000 Date
    constexpr float eps = 0.25f;

    SUBCASE("Basic Conversion Test")
    {
        // Define a TEME coordinate (example)
        TEME teme_coord = {au::make_quantity<au::MetersInTemeFrame>(7000000.0f), au::make_quantity<au::MetersInTemeFrame>(0.0f), au::make_quantity<au::MetersInTemeFrame>(0.0f)}; // Example position in meters

        // Convert TEME to ECEF
        ECEF ecef_coord = temeToECEF(teme_coord, jdut1);

        // Now convert back from ECEF to TEME
        TEME recovered_teme_coord = ecefToTEME(ecef_coord, jdut1);

        // Check if the coordinates are approximately equal
        CHECK(recovered_teme_coord.x.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.x.in(au::metersInTemeFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.y.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.y.in(au::metersInTemeFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.z.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.z.in(au::metersInTemeFrame)).epsilon(eps));
    }

    SUBCASE("Another TEME Coordinate Test")
    {
        // Define a different TEME coordinate
        TEME teme_coord = {au::make_quantity<au::MetersInTemeFrame>(0.0f), au::make_quantity<au::MetersInTemeFrame>(7000000.0f), au::make_quantity<au::MetersInTemeFrame>(0.0f)};

        // Convert TEME to ECEF
        ECEF ecef_coord = temeToECEF(teme_coord, jdut1);

        // Convert back from ECEF to TEME
        TEME recovered_teme_coord = ecefToTEME(ecef_coord, jdut1);

        // Check for approximate equality
        CHECK(recovered_teme_coord.x.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.x.in(au::metersInTemeFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.y.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.y.in(au::metersInTemeFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.z.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.z.in(au::metersInTemeFrame)).epsilon(eps));
    }

    SUBCASE("TEME Coordinate with Z Component")
    {
        // Define a different TEME coordinate
        TEME teme_coord = {au::make_quantity<au::MetersInTemeFrame>(1000000.0f), au::make_quantity<au::MetersInTemeFrame>(-1000000.0f), au::make_quantity<au::MetersInTemeFrame>(5000000.0f)};

        // Convert TEME to ECEF
        ECEF ecef_coord = temeToECEF(teme_coord, jdut1);

        // Convert back from ECEF to TEME
        TEME recovered_teme_coord = ecefToTEME(ecef_coord, jdut1);

        // Check for approximate equality
        CHECK(recovered_teme_coord.x.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.x.in(au::metersInTemeFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.y.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.y.in(au::metersInTemeFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.z.in(au::metersInTemeFrame) == doctest::Approx(teme_coord.z.in(au::metersInTemeFrame)).epsilon(eps));
    }

    SUBCASE("ECEF to TEME - Zero values, to test inverse polarm")
    {
        ECEF ecef_coord = {au::make_quantity<au::MetersInEcefFrame>(0.0f), au::make_quantity<au::MetersInEcefFrame>(0.0f), au::make_quantity<au::MetersInEcefFrame>(0.0f)}; // Example position in meters
        TEME recovered_teme_coord = ecefToTEME(ecef_coord, jdut1);
        CHECK(recovered_teme_coord.x.in(au::metersInTemeFrame) == doctest::Approx(ecef_coord.x.in(au::metersInEcefFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.y.in(au::metersInTemeFrame) == doctest::Approx(ecef_coord.y.in(au::metersInEcefFrame)).epsilon(eps));
        CHECK(recovered_teme_coord.z.in(au::metersInTemeFrame) == doctest::Approx(ecef_coord.z.in(au::metersInEcefFrame)).epsilon(eps));
    }
}

TEST_CASE("TEME to ECEF")
{
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0});

    constexpr float eps = 1.0e-1f;

    SUBCASE("2025/06/25 18:00:00")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 6,
            .day = 25,
            .hour = 18,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        TEME teme_coord = {au::make_quantity<au::MetersInTemeFrame>(-3006.1573609732827f * 1000.0f), au::make_quantity<au::MetersInTemeFrame>(4331.221049310724f * 1000.0f), au::make_quantity<au::MetersInTemeFrame>(-4290.439626312989f * 1000.0f)};
        ECEF ecef_coord = temeToECEF(teme_coord, jd2000);

        CHECK(ecef_coord.x.in(au::metersInEcefFrame) == doctest::Approx(2686.63188566f * 1000.0f).epsilon(eps));
        CHECK(ecef_coord.y.in(au::metersInEcefFrame) == doctest::Approx(-4536.33846792f * 1000.0f).epsilon(eps));
        CHECK(ecef_coord.z.in(au::metersInEcefFrame) == doctest::Approx(-4290.45131185f * 1000.0f).epsilon(eps));
    }

    SUBCASE("2025/07/06 20:43:13")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 7,
            .day = 6,
            .hour = 20,
            .minute = 43,
            .second = 13,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        TEME teme_coord = {au::make_quantity<au::MetersInTemeFrame>(-4813.398435775674f * 1000.0f), au::make_quantity<au::MetersInTemeFrame>(-4416.344248277559f * 1000.0f), au::make_quantity<au::MetersInTemeFrame>(1857.5065466212982f * 1000.0f)};
        ECEF ecef_coord = temeToECEF(teme_coord, jd2000);

        CHECK(ecef_coord.x.in(au::metersInEcefFrame) == doctest::Approx(6355.96709238f * 1000.0f).epsilon(eps));
        CHECK(ecef_coord.y.in(au::metersInEcefFrame) == doctest::Approx(-1508.18261367f * 1000.0f).epsilon(eps));
        CHECK(ecef_coord.z.in(au::metersInEcefFrame) == doctest::Approx(1857.49807967f * 1000.0f).epsilon(eps));
    }
}

TEST_CASE("AU Coordinates TEME to ECEF")
{
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0});

    constexpr float eps = 1.0e-1f;

    SUBCASE("2025/06/25 18:00:00")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 6,
            .day = 25,
            .hour = 18,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-3006.1573609732827f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(4331.221049310724f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-4290.439626312989f)};
        std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);

        CHECK(ecef_coord[0].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(2686.63188566f).epsilon(eps));
        CHECK(ecef_coord[1].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(-4536.33846792f).epsilon(eps));
        CHECK(ecef_coord[2].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(-4290.45131185f).epsilon(eps));
    }

    SUBCASE("2025/07/06 20:43:13")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 7,
            .day = 6,
            .hour = 20,
            .minute = 43,
            .second = 13,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-4813.398435775674f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-4416.344248277559f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(1857.5065466212982f)};
        std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);

        CHECK(ecef_coord[0].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(6355.96709238f).epsilon(eps));
        CHECK(ecef_coord[1].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(-1508.18261367f).epsilon(eps));
        CHECK(ecef_coord[2].in(au::kilo(au::meters * au::ecefs)) == doctest::Approx(1857.49807967f).epsilon(eps));
    }
}

TEST_CASE("AU Coordinates TEME to ECEF to TEME")
{
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0});

    constexpr float eps = 1.0e-1f;

    SUBCASE("2025/06/25 18:00:00")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 6,
            .day = 25,
            .hour = 18,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-3006.1573609732827f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(4331.221049310724f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-4290.439626312989f)};
        std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);
        std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> result = ecefToteme(ecef_coord, jd2000);

        CHECK(result[0].in(au::kilo(au::meters * au::temes)) == doctest::Approx(teme_coord[0].in(au::kilo(au::meters * au::temes))).epsilon(eps));
        CHECK(result[1].in(au::kilo(au::meters * au::temes)) == doctest::Approx(teme_coord[1].in(au::kilo(au::meters * au::temes))).epsilon(eps));
        CHECK(result[2].in(au::kilo(au::meters * au::temes)) == doctest::Approx(teme_coord[2].in(au::kilo(au::meters * au::temes))).epsilon(eps));
    }

    SUBCASE("2025/07/06 20:43:13")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 7,
            .day = 6,
            .hour = 20,
            .minute = 43,
            .second = 13,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-4813.398435775674f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(-4416.344248277559f),
                au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(1857.5065466212982f)};
        std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);
        std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>, 3> result = ecefToteme(ecef_coord, jd2000);

        CHECK(result[0].in(au::kilo(au::meters * au::temes)) == doctest::Approx(teme_coord[0].in(au::kilo(au::meters * au::temes))).epsilon(eps));
        CHECK(result[1].in(au::kilo(au::meters * au::temes)) == doctest::Approx(teme_coord[1].in(au::kilo(au::meters * au::temes))).epsilon(eps));
        CHECK(result[2].in(au::kilo(au::meters * au::temes)) == doctest::Approx(teme_coord[2].in(au::kilo(au::meters * au::temes))).epsilon(eps));
    }
}

TEST_CASE("AU Velocities TEME to ECEF")
{
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0});

    constexpr float eps = 1.0e-1f;

    SUBCASE("2025/06/25 18:00:00")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 6,
            .day = 25,
            .hour = 18,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-3006.1573609732827f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(4331.221049310724f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-4290.439626312989f)};
        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);

        CHECK(ecef_coord[0].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(2686.63188566f).epsilon(eps));
        CHECK(ecef_coord[1].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(-4536.33846792f).epsilon(eps));
        CHECK(ecef_coord[2].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(-4290.45131185f).epsilon(eps));
    }

    SUBCASE("2025/07/06 20:43:13")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 7,
            .day = 6,
            .hour = 20,
            .minute = 43,
            .second = 13,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-4813.398435775674f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-4416.344248277559f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(1857.5065466212982f)};
        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);

        CHECK(ecef_coord[0].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(6355.96709238f).epsilon(eps));
        CHECK(ecef_coord[1].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(-1508.18261367f).epsilon(eps));
        CHECK(ecef_coord[2].in(au::kilo(au::meters * au::ecefs / au::seconds)) == doctest::Approx(1857.49807967f).epsilon(eps));
    }
}

TEST_CASE("AU Velocities TEME to ECEF to TEME")
{
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0});

    constexpr float eps = 1.0e-1f;

    SUBCASE("2025/06/25 18:00:00")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 6,
            .day = 25,
            .hour = 18,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-3006.1573609732827f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(4331.221049310724f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-4290.439626312989f)};
        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);
        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> result = ecefToteme(ecef_coord, jd2000);

        CHECK(result[0].in(au::kilo(au::meters * au::temes / au::seconds)) == doctest::Approx(teme_coord[0].in(au::kilo(au::meters * au::temes / au::seconds))).epsilon(eps));
        CHECK(result[1].in(au::kilo(au::meters * au::temes / au::seconds)) == doctest::Approx(teme_coord[1].in(au::kilo(au::meters * au::temes / au::seconds))).epsilon(eps));
        CHECK(result[2].in(au::kilo(au::meters * au::temes / au::seconds)) == doctest::Approx(teme_coord[2].in(au::kilo(au::meters * au::temes / au::seconds))).epsilon(eps));
    }

    SUBCASE("2025/07/06 20:43:13")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 7,
            .day = 6,
            .hour = 20,
            .minute = 43,
            .second = 13,
            .millisecond = 0});

        float jd2000 = TimeUtils::to_fractional_days(j2000, now);

        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> teme_coord =
            {
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-4813.398435775674f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(-4416.344248277559f),
                au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(1857.5065466212982f)};
        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>, 3> ecef_coord = temeToecef(teme_coord, jd2000);
        std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>, 3> result = ecefToteme(ecef_coord, jd2000);

        CHECK(result[0].in(au::kilo(au::meters * au::temes / au::seconds)) == doctest::Approx(teme_coord[0].in(au::kilo(au::meters * au::temes / au::seconds))).epsilon(eps));
        CHECK(result[1].in(au::kilo(au::meters * au::temes / au::seconds)) == doctest::Approx(teme_coord[1].in(au::kilo(au::meters * au::temes / au::seconds))).epsilon(eps));
        CHECK(result[2].in(au::kilo(au::meters * au::temes / au::seconds)) == doctest::Approx(teme_coord[2].in(au::kilo(au::meters * au::temes / au::seconds))).epsilon(eps));
    }
}

TEST_CASE("Polar Motion")
{
    auto j2000 = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
        .year = 2000,
        .month = 1,
        .day = 1,
        .hour = 12,
        .minute = 0,
        .second = 0,
        .millisecond = 0});

    SUBCASE("2025 07 24 ")
    {
        // epoch for polar file
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2025,
            .month = 7,
            .day = 24,
            .hour = 0,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        float pm[3][3];
        auto polarotion = polarmMJD2000(jdut2, pm);
        CHECK(polarotion.x == doctest::Approx(0.19715f).epsilon(1e-1f));
        CHECK(polarotion.y == doctest::Approx(0.43271f).epsilon(1e-1f));
    }

    SUBCASE("2026 01 01 ")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2026,
            .month = 1,
            .day = 1,
            .hour = 0,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        float pm[3][3];
        auto polarotion = polarmMJD2000(jdut2, pm);
        CHECK(polarotion.x == doctest::Approx(0.1091).epsilon(1e-1f));
        CHECK(polarotion.y == doctest::Approx(0.3015).epsilon(1e-1f));
    }

    SUBCASE("2026 07 24 ")
    {
        auto now = TimeUtils::to_timepoint(TimeUtils::DateTimeComponents{
            .year = 2026,
            .month = 7,
            .day = 24,
            .hour = 0,
            .minute = 0,
            .second = 0,
            .millisecond = 0});

        float jdut2 = TimeUtils::to_fractional_days(j2000, now);
        float pm[3][3];
        auto polarotion = polarmMJD2000(jdut2, pm);
        CHECK(polarotion.x == doctest::Approx(0.2032f).epsilon(1e-1f));
        CHECK(polarotion.y == doctest::Approx(0.4221f).epsilon(1e-1f));
    }
}
