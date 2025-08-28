#ifndef _sgp4coord_
#define _sgp4coord_

/*     
sgp4coord.h

This file contains miscellaneous functions required for coordinate transformation.
Functions were originally written for Matlab as companion code for "Fundamentals of Astrodynamics 
and Applications" by David Vallado (2007). (w) 719-573-2600, email dvallado@agi.com

Ported to C++ by Grady Hillhouse with some modifications, July 2015.
*/

// https://github.com/Hopperpop/Sgp4-Library/blob/master/src/sgp4coord.h

#include <math.h>
#include <string.h>

//void teme2ecef(float rteme[3], float vteme[3], float jdut1, float recef[3], float vecef[3]);
void teme2ecef(const float rteme[3], float jdut1, float recef[3]);

void polarm(float jdut1, float pm[3][3]);

void ijk2ll(const float r[3], float latlongh[3]);

//void site(float latgd, float lon, float alt, float rs[3], float vs[3]);
void site(float latgd, float lon, float alt, float rs[3]);

//void rv2azel(float ro[3], float vo[3], float latgd, float lon, float alt, float jdut1, float razel[3], float razelrates[3]);
void rv2azel(const float ro[3], float latgd, float lon, float alt, float jdut1, float razel[3]);

void rot3(const float invec[3], float xval, float outvec[3]);

void rot2(const float invec[3], float xval, float outvec[3]);

float getJulianFromUnix(float unixSecs);

unsigned long getUnixFromJulian(float julian);

#endif
