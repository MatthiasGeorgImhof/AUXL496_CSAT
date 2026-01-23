#ifndef IGRF_MODEL_HPP
#define IGRF_MODEL_HPP

#include "wmm_legendre.hpp"
#include <cmath>
#include <array>
#include <numbers>

// https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2020/WMM2020_Report.pdf

namespace magnetic_model
{

    // Structure to hold Gauss coefficients (g and h)
    struct GaussCoefficient
    {
        int n;       // Degree
        int m;       // Order
        float g;     // Gauss coefficient g_n^m
        float h;     // Gauss coefficient h_n^m
        float g_dot; // Annual rate of change of g (nT/year)
        float h_dot; // Annual rate of change of h (nT/year)
    };

    // Structure to hold Magnetic Field components
    struct MagneticField
    {
        float X; // Northward component (nT)
        float Y; // Eastward component (nT)
        float Z; // Downward component (nT)
        float F; // Total intensity (nT)
        float H; // Horizontal intensity (nT)
        float D; // Declination (degrees)
        float I; // Inclination (degrees)
    };

    // Function to calculate the magnetic field components
    template <size_t NMAX>
    MagneticField calculateMagneticField(float latitude_deg, float longitude_deg, float radius_m, int year, const std::array<GaussCoefficient, (NMAX + 1) * (NMAX + 2) / 2 - 1> &coefficients);

    constexpr float m_pif = static_cast<float>(std::numbers::pi);

    // Constants (from IGRF documentation)
    constexpr float R_EARTH = 6371200.f; // Earth's radius in m

    constexpr float DEG_TO_RAD = m_pif / 180.0f; // Conversion factor from degrees to radians
    constexpr float RAD_TO_DEG = 180.0f / m_pif; // Conversion factor from radians to degrees

    constexpr float epsilon = 1e-6f;

    // Function to calculate the magnetic field components
    template <size_t NMAX>
    MagneticField calculateMagneticField(float latitude_deg, float longitude_deg, float radius_m, int year, const std::array<GaussCoefficient, (NMAX + 1) * (NMAX + 2) / 2 - 1> &coefficients)
    {
        const int irgf_year = 2025; // Base year for coefficients

        float latitude_rad = latitude_deg * DEG_TO_RAD;
        float longitude_rad = longitude_deg * DEG_TO_RAD;

        float cos_latitude = cosf(latitude_rad);
        float sin_latitude = sinf(latitude_rad);

        constexpr size_t NTERMS = (NMAX + 1) * (NMAX + 2) / 2;

        std::array<float, NTERMS> P_vector{};
        std::array<float, NTERMS> dP_vector{};

        (void)wmm_model::MAG_PcupLow<NMAX, NTERMS>(P_vector, dP_vector, sin_latitude);
        float X = 0.0f, Y = 0.0f, Z = 0.0f;

        for (size_t i = 0; i < coefficients.size(); ++i)
        {
            const GaussCoefficient coeff = coefficients[i];
            float P = P_vector[i + 1];
            float dP = dP_vector[i + 1];

            int n = coeff.n;
            int m = coeff.m;
            float g = coeff.g;
            float h = coeff.h;

            float cos_m_longitude = cosf(static_cast<float>(m) * longitude_rad);
            float sin_m_longitude = sinf(static_cast<float>(m) * longitude_rad);

            // Time adjustment (linear interpolation)
            float time_diff = (float)(year - irgf_year); // Example: Adjust based on the file
            (void)time_diff;
            // g += coeff.g_dot * time_diff;
            // h += coeff.h_dot * time_diff;

            // Radial distance factor
            float ratio = R_EARTH / radius_m;
            float power = static_cast<float>(n + 2);
            float term = powf(ratio, power);

            // Calculate X, Y, and Z components
            if (m == 0)
            {
                X += term * g * cos_m_longitude * dP;
                Z += term * static_cast<float>(n + 1) * g * cos_m_longitude * P;
            }
            else
            {
                X += term * (g * cos_m_longitude + h * sin_m_longitude) * dP;
                Y += term * static_cast<float>(m) * (g * sin_m_longitude - h * cos_m_longitude) * P / cos_latitude;
                Z += term * static_cast<float>(n + 1) * (g * cos_m_longitude + h * sin_m_longitude) * P;
            }
        }

        MagneticField result;
        result.X = -X;
        result.Y = Y;
        result.Z = -Z;

        // Calculate other magnetic field parameters
        result.H = sqrtf(result.X * result.X + result.Y * result.Y);
        result.F = sqrtf(result.X * result.X + result.Y * result.Y + result.Z * result.Z);

        result.D = atan2f(result.Y, result.X) * RAD_TO_DEG; // Declination (East is positive)
        result.I = atan2f(result.Z, result.H) * RAD_TO_DEG; // Inclination (Down is positive)

        return result;
    }

} // namespace magnetic_model

#endif
