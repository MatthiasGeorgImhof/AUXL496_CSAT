#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include "coordinate_transformations.hpp"

#include "TimeUtils.hpp"

// Make sure to use the correct namespace
namespace coordinate_transformations
{

    constexpr float EPSILON = 1.0e-9f;
    constexpr int MAX_ITERATIONS = 2000;

    // --- Utility Function ---
    // Checks if two float values are approximately equal within a given tolerance
    bool approximatelyEqual(const float a, const float b, const float tolerance = 1e-6f)
    {
        return std::fabs(a - b) <= tolerance;
    }

    // --- Coordinate Conversion Functions (Using Structs) ---

    // Geodetic to ECEF
    ECEF geodeticToECEF(Geodetic geodetic)
    {
        ECEF ecef;

        if (geodetic.latitude.in(au::degreesInGeodeticFrame) < -90.0f || geodetic.latitude.in(au::degreesInGeodeticFrame) > 90.0f)
        {
            ecef.x = au::make_quantity<au::MetersInEcefFrame>(std::numeric_limits<float>::quiet_NaN());
            ecef.y = au::make_quantity<au::MetersInEcefFrame>(std::numeric_limits<float>::quiet_NaN());
            ecef.z = au::make_quantity<au::MetersInEcefFrame>(std::numeric_limits<float>::quiet_NaN());
            return ecef;
        }

        float lon_rad = geodetic.longitude.in(au::degreesInGeodeticFrame) * DEG_TO_RAD;
        float lat_rad = geodetic.latitude.in(au::degreesInGeodeticFrame) * DEG_TO_RAD;

        float N = WGS84_A / sqrtf(1 - WGS84_E2 * sinf(lat_rad) * sinf(lat_rad));

        // Explicitly handle North/South Pole case to avoid floating-point errors
        if (approximatelyEqual(std::fabs(geodetic.latitude.in(au::degreesInGeodeticFrame)), 90.0f))
        {
            ecef.x = au::make_quantity<au::MetersInEcefFrame>(0.0f);
            ecef.y = au::make_quantity<au::MetersInEcefFrame>(0.0f);
            ecef.z = au::make_quantity<au::MetersInEcefFrame>(((1 - WGS84_E2) * N + geodetic.height.in(au::metersInGeodeticFrame)) * sinf(lat_rad));
        }
        else
        {
            ecef.x = au::make_quantity<au::MetersInEcefFrame>((N + geodetic.height.in(au::metersInGeodeticFrame)) * cosf(lat_rad) * cosf(lon_rad));
            ecef.y = au::make_quantity<au::MetersInEcefFrame>((N + geodetic.height.in(au::metersInGeodeticFrame)) * cosf(lat_rad) * sinf(lon_rad));
            ecef.z = au::make_quantity<au::MetersInEcefFrame>(((1 - WGS84_E2) * N + geodetic.height.in(au::metersInGeodeticFrame)) * sinf(lat_rad));
        }

        return ecef;
    }

    // ECEF to Geodetic
    Geodetic ecefToGeodetic(ECEF ecef)
    {
        Geodetic geodetic;

        float p = sqrtf(ecef.x.in(au::metersInEcefFrame) * ecef.x.in(au::metersInEcefFrame) + ecef.y.in(au::metersInEcefFrame) * ecef.y.in(au::metersInEcefFrame));

        if (approximatelyEqual(p, 0.0))
        {
            // Special case: point lies on the Z-axis, poles
            geodetic.longitude = au::make_quantity<au::DegreesInGeodeticFrame>(0.0);
            geodetic.latitude = au::make_quantity<au::DegreesInGeodeticFrame>((ecef.z.in(au::metersInEcefFrame) >= 0.0) ? 90.0 : -90.0);
            geodetic.height = au::make_quantity<au::MetersInGeodeticFrame>(std::fabs(ecef.z.in(au::metersInEcefFrame)) - WGS84_A * (1.0f - WGS84_F));
            return geodetic;
        }

        float lon_rad = atan2f(ecef.y.in(au::metersInEcefFrame), ecef.x.in(au::metersInEcefFrame));
        geodetic.longitude = au::make_quantity<au::DegreesInGeodeticFrame>(lon_rad * RAD_TO_DEG);

        float lat_rad = atan2f(ecef.z.in(au::metersInEcefFrame), p * (1.0f - WGS84_E2)); // Initial approximation

        int iteration = 0;
        float lat_old;
        do
        {
            lat_old = lat_rad;
            float N = WGS84_A / sqrtf(1.0f - WGS84_E2 * sinf(lat_rad) * sinf(lat_rad));
            lat_rad = atan2f(ecef.z.in(au::metersInEcefFrame) + WGS84_E2 * N * sinf(lat_rad), p);
            geodetic.height = au::make_quantity<au::MetersInGeodeticFrame>(p / cosf(lat_rad) - N);
            iteration++;
            if (iteration > 100 * MAX_ITERATIONS)
            {
                // // // Did not converge, return NaN values
                // geodetic.latitude = std::numeric_limits<float>::quiet_NaN();
                // geodetic.longitude = std::numeric_limits<float>::quiet_NaN();
                // geodetic.height = std::numeric_limits<float>::quiet_NaN();
                // return geodetic;
                break;
            }
        } while (std::fabs(lat_rad - lat_old) > EPSILON);

        geodetic.latitude = au::make_quantity<au::DegreesInGeodeticFrame>(lat_rad * RAD_TO_DEG);
        return geodetic;
    }

    // Geodetic to Geocentric (Latitude, Longitude, Height -> Latitude, Longitude, Radius)
    Geocentric geodeticToGeocentric(Geodetic geodetic)
    {
        Geocentric geocentric;

        if (geodetic.latitude.in(au::degreesInGeodeticFrame) < -90.0f || geodetic.latitude.in(au::degreesInGeodeticFrame) > 90.0f)
        {
            geocentric.latitude = au::make_quantity<au::DegreesInGeocentricFrame>(std::numeric_limits<float>::quiet_NaN());
            geocentric.longitude = au::make_quantity<au::DegreesInGeocentricFrame>(std::numeric_limits<float>::quiet_NaN());
            geocentric.radius = au::make_quantity<au::MetersInGeocentricFrame>(std::numeric_limits<float>::quiet_NaN());
            return geocentric;
        }

        float lon_rad = geodetic.longitude.in(au::degreesInGeodeticFrame) * DEG_TO_RAD;
        float lat_rad = geodetic.latitude.in(au::degreesInGeodeticFrame) * DEG_TO_RAD;

        float N = WGS84_A / sqrtf(1 - WGS84_E2 * sinf(lat_rad) * sinf(lat_rad));

        float x = (N + geodetic.height.in(au::metersInGeodeticFrame)) * cosf(lat_rad) * cosf(lon_rad);
        float y = (N + geodetic.height.in(au::metersInGeodeticFrame)) * cosf(lat_rad) * sinf(lon_rad);
        float z = ((1 - WGS84_E2) * N + geodetic.height.in(au::metersInGeodeticFrame)) * sinf(lat_rad);

        geocentric.radius = au::make_quantity<au::MetersInGeocentricFrame>(sqrtf(x * x + y * y + z * z));
        geocentric.latitude = au::make_quantity<au::DegreesInGeocentricFrame>(asinf(z / geocentric.radius.in(au::metersInGeocentricFrame))) * RAD_TO_DEG;
        geocentric.longitude = au::make_quantity<au::DegreesInGeocentricFrame>(geodetic.longitude.in(au::degreesInGeodeticFrame));
        return geocentric;
    }

    // Geocentric to Geodetic (Requires Iterative Solution)
    // IMPORTANT:  The geocentric latitude MUST be known accurately for this to work well.
    Geodetic geocentricToGeodetic(Geocentric geocentric)
    {
        Geodetic geodetic;

        if (geocentric.radius.in(au::metersInGeocentricFrame) <= EPSILON || geocentric.latitude.in(au::degreesInGeocentricFrame) < -90.0f || geocentric.latitude.in(au::degreesInGeocentricFrame) > 90.0f)
        {
            geodetic.latitude = au::make_quantity<au::DegreesInGeodeticFrame>(std::numeric_limits<float>::quiet_NaN());
            geodetic.longitude = au::make_quantity<au::DegreesInGeodeticFrame>(std::numeric_limits<float>::quiet_NaN());
            geodetic.height = au::make_quantity<au::MetersInGeodeticFrame>(std::numeric_limits<float>::quiet_NaN());
            return geodetic;
        }

        float lon_rad = geocentric.longitude.in(au::degreesInGeocentricFrame) * DEG_TO_RAD;
        float geocentric_lat_rad = geocentric.latitude.in(au::degreesInGeocentricFrame) * DEG_TO_RAD;

        float x = geocentric.radius.in(au::metersInGeocentricFrame) * cosf(geocentric_lat_rad) * cosf(lon_rad);
        float y = geocentric.radius.in(au::metersInGeocentricFrame) * cosf(geocentric_lat_rad) * sinf(lon_rad);
        float z = geocentric.radius.in(au::metersInGeocentricFrame) * sinf(geocentric_lat_rad);

        float lat_rad = atan2f(z, sqrtf(x * x + y * y)); // Initial guess for geodetic latitude

        float lat_old;
        float N;
        int iteration = 0;

        // Iterative refinement for latitude and height
        geodetic.height = au::make_quantity<au::MetersInGeodeticFrame>(0.f); // Initial guess for height

        do
        {
            lat_old = lat_rad;
            N = WGS84_A / sqrtf(1 - WGS84_E2 * sinf(lat_rad) * sinf(lat_rad));
            geodetic.height = au::make_quantity<au::MetersInGeodeticFrame>(sqrtf(x * x + y * y) / cosf(lat_rad) - N);
            lat_rad = atan2f(z, (sqrtf(x * x + y * y) * (1 - WGS84_E2 * N / (N + geodetic.height.in(au::metersInGeodeticFrame)))));
            iteration++;

            // Check for oscillation in height and terminate early
            if (iteration > MAX_ITERATIONS)
            {
                // // Did not converge, return NaN values
                // geodetic.latitude = std::numeric_limits<float>::quiet_NaN();
                // geodetic.longitude = std::numeric_limits<float>::quiet_NaN();
                // geodetic.height = std::numeric_limits<float>::quiet_NaN();
                // return geodetic;
                break;
            }

        } while (fabsf(lat_rad - lat_old) > EPSILON);

        geodetic.latitude = au::make_quantity<au::DegreesInGeodeticFrame>(lat_rad * RAD_TO_DEG);
        geodetic.longitude = au::make_quantity<au::DegreesInGeodeticFrame>(geocentric.longitude.in(au::degreesInGeocentricFrame));
        return geodetic;
    }

    // --- TEME to ECEF Conversion ---

    inline float sgn(float x)
    {
        return (x < 0.0f) ? -1.0f : 1.0f;
    }

    inline float mag(const float x[3])
    {
        return sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    }

    inline float floatmod(float a, float b)
    {
        return (a - b * floorf(a / b));
    }

    PolarMotion polarmMJD2000(float jd2000, float pm[3][3])
    {
        constexpr float MJD2000 = 51544.5f;     // MJD for J2000 epoch
        constexpr float MJDBULLETIN = 60880.0f; // MJD for IERS Bulletin A
        constexpr float SHIFT = MJD2000 - MJDBULLETIN;
        float days = jd2000 + SHIFT; // Adjusted MJD for J2000 epoch

        constexpr float M_2PIf = 2.0f * M_PIf;
        float A = M_2PIf * days / 365.25f;
        float C = M_2PIf * days / 435.f;

        // 24 July 2025                                      Vol. XXXVIII No. 030
        float x = 0.1376f + 0.0836f * cosf(A) + 0.1286f * sinf(A) - 0.0263f * cosf(C) - 0.0762f * sinf(C);
        float y = 0.3866f + 0.1244f * cosf(A) - 0.0728f * sinf(A) - 0.0762f * cosf(C) + 0.0263f * sinf(C);

        // Polar motion coefficients in radians
        float x_ = x * 4.84813681e-6f;
        float y_ = y * 4.84813681e-6f;
        pm[0][0] = cosf(x_);
        pm[0][1] = 0.0f;
        pm[0][2] = -sinf(x_);
        pm[1][0] = sinf(x_) * sinf(y_);
        pm[1][1] = cosf(y_);
        pm[1][2] = cosf(x_) * sinf(y_);
        pm[2][0] = sinf(x_) * cosf(y_);
        pm[2][1] = -sinf(y_);
        pm[2][2] = cosf(x_) * cosf(y_);

        return {x, y};
    }

    void polarmJD(float jdut1, float pm[3][3])
    {
        // Predict polar motion coefficients using IERS Bulletin - A (Vol. XXVIII No. 030)
        float MJD = jdut1 - 2400000.5f; // Julian Date - 2,400,000.5 days
        constexpr float pi = 3.14159265358979323846f;
        float A = 2.f * pi * (MJD - 57226.f) / 365.25f;
        float C = 2.f * pi * (MJD - 57226.f) / 435.f;

        // Polar motion coefficients in radians
        float xp = (0.1033f + 0.0494f * cosf(A) + 0.0482f * sinf(A) + 0.0297f * cosf(C) + 0.0307f * sinf(C)) * 4.84813681e-6f;
        float yp = (0.3498f + 0.0441f * cosf(A) - 0.0393f * sinf(A) + 0.0307f * cosf(C) - 0.0297f * sinf(C)) * 4.84813681e-6f;

        pm[0][0] = cosf(xp);
        pm[0][1] = 0.0f;
        pm[0][2] = -sinf(xp);
        pm[1][0] = sinf(xp) * sinf(yp);
        pm[1][1] = cosf(yp);
        pm[1][2] = cosf(xp) * sinf(yp);
        pm[2][0] = sinf(xp) * cosf(yp);
        pm[2][1] = -sinf(yp);
        pm[2][2] = cosf(xp) * cosf(yp);
    }

    float gsTimeJD(float jdut1)
    {
        constexpr float pi = 3.14159265358979323846f;
        constexpr float twopi = 2.0f * pi;
        constexpr float deg2rad = pi / 180.0f;

        float tut1 = (jdut1 - 2451545.0f) / 36525.0f;
        float temp = -6.2e-6f * tut1 * tut1 * tut1 + 0.093104f * tut1 * tut1 +
                     (876600.0f * 3600.f + 8640184.812866f) * tut1 + 67310.54841f; // sec
        temp = floatmod(temp * deg2rad / 240.0f, twopi);                           // 360/86400 = 1/240, to deg, to rad

        // ------------------------ check quadrants ---------------------
        if (temp < 0.0f)
            temp += twopi;

        return temp;
    } // end gstime

    void teme2ecef(const float rteme[3], float jd2000, float recef[3])
    {
        float st[3][3];
        float rpef[3];
        float pm[3][3];

        // Get Greenwich mean sidereal time
        float gmst = TimeUtils::hoursToRadians(TimeUtils::gsTimeJ2000(jd2000));
        // float gmst = gsTimeJD(jd2000);

        // st is the pef - tod matrix
        st[0][0] = cosf(gmst);
        st[0][1] = -sinf(gmst);
        st[0][2] = 0.0f;
        st[1][0] = sinf(gmst);
        st[1][1] = cosf(gmst);
        st[1][2] = 0.0f;
        st[2][0] = 0.0f;
        st[2][1] = 0.0f;
        st[2][2] = 1.0f;

        // Get pseudo earth fixed position vector by multiplying the inverse pef-tod matrix by rteme
        rpef[0] = st[0][0] * rteme[0] + st[1][0] * rteme[1];
        rpef[1] = st[0][1] * rteme[0] + st[1][1] * rteme[1];
        rpef[2] = st[2][2] * rteme[2];

        // Get polar motion vector
        (void)polarmMJD2000(jd2000, pm);
        // (void)polarmJD(jd2000, pm);

        // ECEF postion vector is the inverse of the polar motion vector multiplied by rpef
        recef[0] = pm[0][0] * rpef[0] + pm[1][0] * rpef[1] + pm[2][0] * rpef[2];
        recef[1] = pm[0][1] * rpef[0] + pm[1][1] * rpef[1] + pm[2][1] * rpef[2];
        recef[2] = pm[0][2] * rpef[0] + pm[1][2] * rpef[1] + pm[2][2] * rpef[2];
    }

    ECEF temeToECEF(TEME teme, float jd2000)
    {
        ECEF ecef;
        float rteme[3] = {teme.x.in(au::metersInTemeFrame), teme.y.in(au::metersInTemeFrame), teme.z.in(au::metersInTemeFrame)};
        float recef[3];

        teme2ecef(rteme, jd2000, recef);

        ecef.x = au::make_quantity<au::MetersInEcefFrame>(recef[0]);
        ecef.y = au::make_quantity<au::MetersInEcefFrame>(recef[1]);
        ecef.z = au::make_quantity<au::MetersInEcefFrame>(recef[2]);

        return ecef;
    }

    std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>,3> temeToecef(std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>,3> teme, float jd2000)
    {
        float rteme[3] = {
            teme[0].in(au::kilo(au::meters * au::temes)),
            teme[1].in(au::kilo(au::meters * au::temes)),
            teme[2].in(au::kilo(au::meters * au::temes))
        };
        
        float recef[3];
        teme2ecef(rteme, jd2000, recef);

        return std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>,3> {
            au::make_quantity<au::Kilo<au::MetersInEcefFrame>>(recef[0]),
            au::make_quantity<au::Kilo<au::MetersInEcefFrame>>(recef[1]),
            au::make_quantity<au::Kilo<au::MetersInEcefFrame>>(recef[2])
        };    
    }

    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>,3> temeToecef(std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>,3> teme, float jd2000)
    {
        float rteme[3] = {
            teme[0].in(au::kilo(au::meters * au::temes / au::seconds)),
            teme[1].in(au::kilo(au::meters * au::temes / au::seconds)),
            teme[2].in(au::kilo(au::meters * au::temes / au::seconds))
        };
        
        float recef[3];
        teme2ecef(rteme, jd2000, recef);

        return std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>,3> {
            au::make_quantity<au::Kilo<au::MetersPerSecondInEcefFrame>>(recef[0]),
            au::make_quantity<au::Kilo<au::MetersPerSecondInEcefFrame>>(recef[1]),
            au::make_quantity<au::Kilo<au::MetersPerSecondInEcefFrame>>(recef[2])
        };    
    }

    void ecef2teme(const float recef[3], float jd2000, float rteme[3])
    {
        float st[3][3];
        float rpef[3];
        float pm[3][3];

        // 1. Inverse Polar Motion:
        (void)polarmMJD2000(jd2000, pm);
        // (void)polarmJD(jd2000, pm);

        // Transpose the polar motion matrix (pm) because we are inverting the transform.
        rpef[0] = pm[0][0] * recef[0] + pm[0][1] * recef[1] + pm[0][2] * recef[2];
        rpef[1] = pm[1][0] * recef[0] + pm[1][1] * recef[1] + pm[1][2] * recef[2];
        rpef[2] = pm[2][0] * recef[0] + pm[2][1] * recef[1] + pm[2][2] * recef[2];

        // 2. Inverse Greenwich Mean Sidereal Time Rotation:
        float gmst = TimeUtils::hoursToRadians(TimeUtils::gsTimeJ2000(jd2000));
        // float gmst = gsTimeJD(jd2000);

        // Need the transpose (inverse) of the st matrix.
        st[0][0] = cosf(gmst);
        st[0][1] = -sinf(gmst);
        st[0][2] = 0.0f;
        st[1][0] = sinf(gmst);
        st[1][1] = cosf(gmst);
        st[1][2] = 0.0f;
        st[2][0] = 0.0f;
        st[2][1] = 0.0f;
        st[2][2] = 1.0f;

        // Transpose st matrix
        rteme[0] = st[0][0] * rpef[0] + st[0][1] * rpef[1];
        rteme[1] = st[1][0] * rpef[0] + st[1][1] * rpef[1];
        rteme[2] = st[2][2] * rpef[2];
    }

    TEME ecefToTEME(ECEF ecef, float jd2000)
    {
        TEME teme;
        float recef[3] = {ecef.x.in(au::metersInEcefFrame), ecef.y.in(au::metersInEcefFrame), ecef.z.in(au::metersInEcefFrame)};
        float rteme[3];

        ecef2teme(recef, jd2000, rteme); // Call the inverse transformation function

        teme.x = au::make_quantity<au::MetersInTemeFrame>(rteme[0]);
        teme.y = au::make_quantity<au::MetersInTemeFrame>(rteme[1]);
        teme.z = au::make_quantity<au::MetersInTemeFrame>(rteme[2]);

        return teme;
    }

    std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>,3> ecefToteme(std::array<au::QuantityF<au::Kilo<au::MetersInEcefFrame>>,3> ecef, float jd2000)
    {
        float recef[3] = {
            ecef[0].in(au::kilo(au::meters * au::ecefs)),
            ecef[1].in(au::kilo(au::meters * au::ecefs)),
            ecef[2].in(au::kilo(au::meters * au::ecefs))
        };
        
        float rteme[3];
        ecef2teme(recef, jd2000, rteme);

        return std::array<au::QuantityF<au::Kilo<au::MetersInTemeFrame>>,3> {
            au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(rteme[0]),
            au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(rteme[1]),
            au::make_quantity<au::Kilo<au::MetersInTemeFrame>>(rteme[2])
        };    
    }

    std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>,3> ecefToteme(std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInEcefFrame>>,3> ecef, float jd2000)
    {
        float recef[3] = {
            ecef[0].in(au::kilo(au::meters * au::ecefs / au::seconds)),
            ecef[1].in(au::kilo(au::meters * au::ecefs / au::seconds)),
            ecef[2].in(au::kilo(au::meters * au::ecefs / au::seconds))
        };
        
        float rteme[3];
        ecef2teme(recef, jd2000, rteme);

        return std::array<au::QuantityF<au::Kilo<au::MetersPerSecondInTemeFrame>>,3> {
            au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(rteme[0]),
            au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(rteme[1]),
            au::make_quantity<au::Kilo<au::MetersPerSecondInTemeFrame>>(rteme[2])
        };    
    }



} // namespace coordinate_transformations
