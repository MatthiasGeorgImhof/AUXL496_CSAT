/*
sgp4coord.cpp

This file contains miscellaneous functions required for coordinate transformation.
Functions were originally written for Matlab as companion code for "Fundamentals of Astrodynamics
and Applications" by David Vallado (2007). (w) 719-573-2600, email dvallado@agi.com

Ported to C++ by Grady Hillhouse with some modifications, July 2015.
*/

// https://github.com/Hopperpop/Sgp4-Library/blob/master/src/sgp4coord.cpp


#include "sgp4coord.h"
// #include "sgp4ext.h"

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

float gstime(float jdut1)
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float twopi = 2.0f * pi;
    constexpr float deg2rad = pi / 180.0f;
    float temp, tut1;

    tut1 = (jdut1 - 2451545.0f) / 36525.0f;
    temp = -6.2e-6f * tut1 * tut1 * tut1 + 0.093104f * tut1 * tut1 +
           (876600.0f * 3600.f + 8640184.812866f) * tut1 + 67310.54841f; // sec
    temp = floatmod(temp * deg2rad / 240.0f, twopi);                     // 360/86400 = 1/240, to deg, to rad

    // ------------------------ check quadrants ---------------------
    if (temp < 0.0f)
        temp += twopi;

    return temp;
} // end gstime

/*
teme2ecef

This function transforms a vector from a true equator mean equinox (TEME)
frame to an earth-centered, earth-fixed (ECEF) frame.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
rteme           Position vector (TEME)          km
vteme           Velocity vector (TEME)          km/s
jdut1           Julian date                     days

OUTPUTS         DESCRIPTION                     RANGE/UNITS
recef           Position vector (ECEF)          km
vecef           Velocity vector (ECEF)          km/s
*/

// void teme2ecef(float rteme[3], float vteme[3], float jdut1, float recef[3], float vecef[3])
void teme2ecef(const float rteme[3], float jdut1, float recef[3])
{
    float gmst;
    float st[3][3];
    float rpef[3];
    // float vpef[3];
    float pm[3][3];
    // float omegaearth[3];

    // Get Greenwich mean sidereal time
    gmst = gstime(jdut1);

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
    rpef[0] = st[0][0] * rteme[0] + st[1][0] * rteme[1] + st[2][0] * rteme[2];
    rpef[1] = st[0][1] * rteme[0] + st[1][1] * rteme[1] + st[2][1] * rteme[2];
    rpef[2] = st[0][2] * rteme[0] + st[1][2] * rteme[1] + st[2][2] * rteme[2];

    // Get polar motion vector
    polarm(jdut1, pm);

    // ECEF postion vector is the inverse of the polar motion vector multiplied by rpef
    recef[0] = pm[0][0] * rpef[0] + pm[1][0] * rpef[1] + pm[2][0] * rpef[2];
    recef[1] = pm[0][1] * rpef[0] + pm[1][1] * rpef[1] + pm[2][1] * rpef[2];
    recef[2] = pm[0][2] * rpef[0] + pm[1][2] * rpef[1] + pm[2][2] * rpef[2];

    // Earth's angular rotation vector (omega)
    // Note: I don't have a good source for LOD. Historically it has been on the order of 2 ms so I'm just using that as a constant. The effect is very small.
    // omegaearth[0] = 0.0;
    // omegaearth[1] = 0.0;
    // omegaearth[2] = 7.29211514670698e-05 * (1.0f  - 0.0015563/86400.0);

    // Pseudo Earth Fixed velocity vector is st'*vteme - omegaearth X rpef
    // vpef[0] = st[0][0] * vteme[0] + st[1][0] * vteme[1] + st[2][0] * vteme[2] - (omegaearth[1]*rpef[2] - omegaearth[2]*rpef[1]);
    // vpef[1] = st[0][1] * vteme[0] + st[1][1] * vteme[1] + st[2][1] * vteme[2] - (omegaearth[2]*rpef[0] - omegaearth[0]*rpef[2]);
    // vpef[2] = st[0][2] * vteme[0] + st[1][2] * vteme[1] + st[2][2] * vteme[2] - (omegaearth[0]*rpef[1] - omegaearth[1]*rpef[0]);

    // ECEF velocty vector is the inverse of the polar motion vector multiplied by vpef
    // vecef[0] = pm[0][0] * vpef[0] + pm[1][0] * vpef[1] + pm[2][0] * vpef[2];
    // vecef[1] = pm[0][1] * vpef[0] + pm[1][1] * vpef[1] + pm[2][1] * vpef[2];
    // vecef[2] = pm[0][2] * vpef[0] + pm[1][2] * vpef[1] + pm[2][2] * vpef[2];
}

/*
polarm

This function calulates the transformation matrix that accounts for polar
motion. Polar motion coordinates are estimated using IERS Bulletin
rather than directly input for simplicity.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
jdut1           Julian date                     days

OUTPUTS         DESCRIPTION
pm              Transformation matrix for ECEF - PEF
*/

void polarm(float jdut1, float pm[3][3])
{
    float MJD; // Julian Date - 2,400,000.5 days
    float A;
    float C;
    float xp; // Polar motion coefficient in radians
    float yp; // Polar motion coefficient in radians

    // Predict polar motion coefficients using IERS Bulletin - A (Vol. XXVIII No. 030)
    MJD = jdut1 - 2400000.5f;
    constexpr float pi = 3.14159265358979323846f;
    A = 2.f * pi * (MJD - 57226.f) / 365.25f;
    C = 2.f * pi * (MJD - 57226.f) / 435.f;

    xp = (0.1033f + 0.0494f * cosf(A) + 0.0482f * sinf(A) + 0.0297f * cosf(C) + 0.0307f * sinf(C)) * 4.84813681e-6f;
    yp = (0.3498f + 0.0441f * cosf(A) - 0.0393f * sinf(A) + 0.0307f * cosf(C) - 0.0297f * sinf(C)) * 4.84813681e-6f;

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

/*
ijk2ll

This function calulates the latitude, longitude and altitude
given the ECEF position matrix.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
r               Position matrix (ECEF)          km

OUTPUTS         DESCRIPTION
latlongh        Latitude, longitude, and altitude (rad, rad, and km)
*/

void ijk2ll(const float r[3], float latlongh[3])
{
    constexpr float pi = 3.14159265358979323846f;
    constexpr float twopi = 2.0f * pi;
    constexpr float small = 0.00000001f;      // small value for tolerances
    constexpr float re = 6378.137f;           // radius of earth in km
    constexpr float eesqrd = 0.006694385000f; // eccentricity of earth sqrd
    float magr, temp, rtasc;

    magr = mag(r);
    temp = sqrtf(r[0] * r[0] + r[1] * r[1]);

    if (fabs(temp) < small)
    {
        rtasc = sgn(r[2]) * pi * 0.5f;
    }
    else
    {
        rtasc = atan2f(r[1], r[0]);
    }

    latlongh[1] = rtasc;

    if (fabs(latlongh[1]) >= pi)
    {
        if (latlongh[1] < 0.0f)
        {
            latlongh[1] += twopi;
        }
        else
        {
            latlongh[1] -= twopi;
        }
    }

    latlongh[0] = asinf(r[2] / magr);

    // Iterate to find geodetic latitude
    int i = 1;
    float olddelta = latlongh[0] + 10.0f;
    float sintemp, c = 0;

    while ((fabs(olddelta - latlongh[0]) >= small) && (i < 10))
    {
        olddelta = latlongh[0];
        sintemp = sinf(latlongh[0]);
        c = re / sqrtf(1.0f - eesqrd * sintemp * sintemp);
        latlongh[0] = atanf((r[2] + c * eesqrd * sintemp) / temp);
        i++;
    }

    if (0.5f * pi - fabs(latlongh[0]) > pi / 180.0f)
    {
        latlongh[2] = (temp / cosf(latlongh[0])) - c;
    }
    else
    {
        latlongh[2] = r[2] / sinf(latlongh[0]) - c * (1.0f - eesqrd);
    }
}

/*
site

This function finds the position and velocity vectors for a site. The
outputs are in the ECEF coordinate system. Note that the velocity vector
is zero because the coordinate system rotates with the earth.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
latgd           Site geodetic latitude          -pi/2 to pi/2 in radians
lon             Longitude                       -2pi to 2pi in radians
alt             Site altitude                   km

OUTPUTS         DESCRIPTION
rs              Site position vector            km
vs              Site velocity vector            km/s
*/

// void site(float latgd, float lon, float alt, float rs[3], float vs[3])
void site(float latgd, float lon, float alt, float rs[3])
{
    constexpr float re = 6378.137f;           // radius of earth in km
    constexpr float eesqrd = 0.006694385000f; // eccentricity of earth sqrd

    // Find rdel and rk components of site vector
    float sinlat = sinf(latgd);
    float cearth = re / sqrtf(1.0f - (eesqrd * sinlat * sinlat));
    float rdel = (cearth + alt) * cosf(latgd);
    float rk = ((1.0f - eesqrd) * cearth + alt) * sinlat;

    // Find site position vector (ECEF)
    rs[0] = rdel * cosf(lon);
    rs[1] = rdel * sinf(lon);
    rs[2] = rk;

    // Velocity of site is zero because the coordinate system is rotating with the earth
    // vs[0] = 0.0;
    // vs[1] = 0.0;
    // vs[2] = 0.0;
}

/*
rv2azel

This function calculates the range, elevation, and azimuth (and their rates)
from the TEME vectors output by the SGP4 function.

Author: David Vallado, 2007
Ported to C++ by Grady Hillhouse with some modifications, July 2015.

INPUTS          DESCRIPTION                     RANGE/UNITS
ro              Sat. position vector (TEME)     km
vo              Sat. velocity vector (TEME)     km/s
latgd           Site geodetic latitude          -pi/2 to pi/2 in radians
lon             Site longitude                  -2pi to 2pi in radians
alt             Site altitude                   km
jdut1           Julian date                     days

OUTPUTS         DESCRIPTION
razel           Range, azimuth, and elevation matrix
razelrates      Range rate, azimuth rate, and elevation rate matrix
*/

// void rv2azel(float ro[3], float vo[3], float latgd, float lon, float alt, float jdut1, float razel[3], float razelrates[3])
void rv2azel(const float ro[3], float latgd, float lon, float alt, float jdut1, float razel[3])
{
    // Locals
    constexpr float pi = 3.14159265358979323846f;
    constexpr float halfpi = pi * 0.5f;
    float small = 0.00000001f;
    float temp;
    float rs[3];
    // float vs[3];
    float recef[3];
    // float vecef[3];
    float rhoecef[3];
    // float drhoecef[3];
    float tempvec[3];
    float rhosez[3];
    // float drhosez[3];
    float magrhosez;
    float rho, az, el;
    // float drho, daz, del;

    // Get site vector in ECEF coordinate system
    // site(latgd, lon, alt, rs, vs);
    site(latgd, lon, alt, rs);

    // Convert TEME vectors to ECEF coordinate system
    // teme2ecef(ro, vo, jdut1, recef, vecef);
    teme2ecef(ro, jdut1, recef);

    // Find ECEF range vectors
    for (int i = 0; i < 3; i++)
    {
        rhoecef[i] = recef[i] - rs[i];
        // drhoecef[i] = vecef[i];
    }
    rho = mag(rhoecef); // Range in km

    // Convert to SEZ (topocentric horizon coordinate system)
    rot3(rhoecef, lon, tempvec);
    rot2(tempvec, (halfpi - latgd), rhosez);

    // rot3(drhoecef, lon, tempvec);
    // rot2(tempvec, (halfpi-latgd), drhosez);

    // Calculate azimuth, and elevation
    temp = sqrtf(rhosez[0] * rhosez[0] + rhosez[1] * rhosez[1]);
    if (temp < small)
    {
        el = sgn(rhosez[2]) * halfpi;
        // az = atan2f(drhosez[1], -drhosez[0]);
        az = NAN;
    }
    else
    {
        magrhosez = mag(rhosez);
        el = asinf(rhosez[2] / magrhosez);
        az = atan2f(rhosez[1], -rhosez[0]);
    }

    // Calculate rates for range, azimuth, and elevation
    /**
    drho = dot(rhosez,drhosez) / rho;

    if(fabs(temp*temp) > small)
    {
        daz = (drhosez[0]*rhosez[1] - drhosez[1]*rhosez[0]) / (temp * temp);
    }
    else
    {
        daz = 0.0;
    }

    if(fabs(temp) > small)
    {
        del = (drhosez[2] - drho*sinf(el)) / temp;
    }
    else
    {
        del = 0.0;
    }
    ***/
    // Move values to output vectors
    razel[0] = rho; // Range (km)
    razel[1] = az;  // Azimuth (radians)
    razel[2] = el;  // Elevation (radians)

    // razelrates[0] = drho;       //Range rate (km/s)
    // razelrates[1] = daz;        //Azimuth rate (rad/s)
    // razelrates[2] = del;        //Elevation rate (rad/s)
}

void rot3(const float invec[3], float xval, float outvec[3])
{
    float temp = invec[1];
    float c = cosf(xval);
    float s = sinf(xval);

    outvec[1] = c * invec[1] - s * invec[0];
    outvec[0] = c * invec[0] + s * temp;
    outvec[2] = invec[2];
}

void rot2(const float invec[3], float xval, float outvec[3])
{
    float temp = invec[2];
    float c = cosf(xval);
    float s = sinf(xval);

    outvec[2] = c * invec[2] + s * invec[0];
    outvec[0] = c * invec[0] - s * temp;
    outvec[1] = invec[1];
}

/*
getJulianFromUnix

returns the Julian Date from Unix Time
*/

float getJulianFromUnix(float unixSecs)
{
    return (unixSecs / 86400.0f) + 2440587.5f;
}

unsigned long getUnixFromJulian(float julian)
{
    return (unsigned long)((julian - 2440587.5f) * 86400.0f + 0.5f);
}
