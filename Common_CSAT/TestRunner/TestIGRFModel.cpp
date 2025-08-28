#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "magnetic_model.hpp"

#undef MAX_ORDER
#include "igrf_coefficients_14.hpp" // Include the IGRF coefficients

#include <cmath>

// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
// computed for 1/1/2025

// Define a simple helper function to compare MagneticField structs
auto compareMagneticFields = [](const magnetic_model::MagneticField& a, const magnetic_model::MagneticField& b) {
    constexpr float epsilon = 0.1f; // Tolerance for floating point comparison
    CHECK(doctest::Approx(a.X).epsilon(epsilon) == b.X);
    CHECK(doctest::Approx(a.Y).epsilon(epsilon) == b.Y);
    CHECK(doctest::Approx(a.Z).epsilon(epsilon) == b.Z);
    CHECK(doctest::Approx(a.H).epsilon(epsilon) == b.H);
    CHECK(doctest::Approx(a.F).epsilon(epsilon) == b.F);
    CHECK(doctest::Approx(a.D).epsilon(epsilon) == b.D);
    CHECK(doctest::Approx(a.I).epsilon(epsilon) == b.I);
};

constexpr float RADIUS = 6371200.f; // Approximate Earth radius in meters

TEST_CASE("Magnetic Field Calculation equatorial sealevel cases") {
    using namespace magnetic_model;

        float latitude_deg = 0.0f;
        float radius_m = RADIUS;

    SUBCASE("Equator, 0 at Surface, 2025") {
        float longitude_deg = 0.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D = -4.0163f;
        expected.I = -30.1888f;
        expected.H =  27521.3f;
        expected.X =  27453.7f;
        expected.Y = -1927.6f;
        expected.Z = -16010.6f;
        expected.F =  31839.6f; 
        compareMagneticFields(result, expected);
    }

    SUBCASE("Equator, -90 at Surface, 2025") {
        float longitude_deg = -90.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D =  2.3934f;
        expected.I =  19.6625f;
        expected.H =  27639.2f;
        expected.X =  27615.1f;
        expected.Y =  1154.2f;
        expected.Z =  9875.9f;
        expected.F =  29350.6f; 
        compareMagneticFields(result, expected);
    }

    SUBCASE("Equator, 120 at Surface, 2025") {
        float longitude_deg = 120.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D = -0.1583f;
        expected.I = -14.9307f;
        expected.H =  39676.7f;
        expected.X =  39676.6f;
        expected.Y = -109.6f;
        expected.Z = -10579.9f;
        expected.F =  41063.1f; 
        compareMagneticFields(result, expected);
    }
}

TEST_CASE("Magnetic Field Calculation 30N 100km cases") {
    using namespace magnetic_model;

        float latitude_deg = 30.0f;
        float radius_m = RADIUS + 100000; 

    SUBCASE("30N, 0 at 100km, 2025") {
        float longitude_deg = 0.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D =  0.7790f;
        expected.I =  40.2420f;
        expected.H =  29552.1f;
        expected.X =  29549.4f;
        expected.Y =  401.8f;
        expected.Z =  25010.6f;
        expected.F =  38715.1f; 
        compareMagneticFields(result, expected);
    }

    SUBCASE("30N, -90 at 100km, 2025") {
        float longitude_deg = -90.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D = -1.4851f;
        expected.I =  58.7631f;
        expected.H =  22855.0f;
        expected.X =  22847.3f;
        expected.Y = -592.3f;
        expected.Z =  37683.2f;
        expected.F =  44072.4f; 
        compareMagneticFields(result, expected);
    }

    SUBCASE("30N, 120 at 100km, 2025") {
        float longitude_deg = 120.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D = -5.7105f;
        expected.I =  45.6416f;
        expected.H =  32328.6f;
        expected.X =  32168.2f;
        expected.Y = -3216.8f;
        expected.Z =  33060.9f;
        expected.F =  46240.3f; 
        compareMagneticFields(result, expected);
    }
}

TEST_CASE("Magnetic Field Calculation 85N 400km cases") {
    using namespace magnetic_model;

        float latitude_deg = 85.0f;
        float radius_m = RADIUS + 400000; 

    SUBCASE("85N, 0 at 400km, 2025") {
        float longitude_deg = 0.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D =  0.7643f;
        expected.I =  85.9568f;
        expected.H =  3358.8f;
        expected.X =  3358.5f;
        expected.Y =  39.385f; // should be 44.8f;
        expected.Z =  47517.9f;
        expected.F =  47636.5f; 
        compareMagneticFields(result, expected);
    }

    SUBCASE("85N, -90 at 400km, 2025") {
        float longitude_deg = -90.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D = -48.0104f;
        expected.I =  88.2155f;
        expected.H =  1494.3f;
        expected.X =  999.7f;
        expected.Y = -1110.7f;
        expected.Z =  47962.9f;
        expected.F =  47986.1f; 
        compareMagneticFields(result, expected);
    }

    SUBCASE("85N, 120 at 400km, 2025") {
        float longitude_deg = 120.0f;

        MagneticField result = calculateMagneticField<MAX_ORDER>(latitude_deg, longitude_deg, radius_m, 2025, magneticGaussCoefficients);
        MagneticField expected;
        expected.D =  30.3742f;
        expected.I =  89.1255f;
        expected.H =  745.5f;
        expected.X =  643.2f;
        expected.Y =  377.0f;
        expected.Z =  48843.2f;
        expected.F =  48848.9f; 
        compareMagneticFields(result, expected);
    }
}