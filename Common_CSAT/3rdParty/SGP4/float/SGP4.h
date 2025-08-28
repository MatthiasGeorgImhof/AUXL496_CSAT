#ifndef _SGP4h_
#define _SGP4h_
/*     ----------------------------------------------------------------
*
*                                 SGP4.h
*
*    this file contains the sgp4 procedures for analytical propagation
*    of a satellite. the code was originally released in the 1980 and 1986
*    spacetrack papers. a detailed discussion of the theory and history
*    may be found in the 2006 aiaa paper by vallado, crawford, hujsak,
*    and kelso.
*
*    current :
*              12 mar 20  david vallado
*                           chg satnum to string for alpha 5 or 9-digit
*    changes :
*               7 dec 15  david vallado
*                           fix jd, jdfrac
*               3 nov 14  david vallado
*                           update to msvs2013 c++
*              30 Dec 11  david vallado
*                           consolidate updated code version
*              30 Aug 10  david vallado
*                           delete unused variables in initl
*                           replace pow inetger 2, 3 with multiplies for speed
*               3 Nov 08  david vallado
*                           put returns in for error codes
*              29 sep 08  david vallado
*                           fix atime for faster operation in dspace
*                           add operationmode for afspc (a) or improved (i)
*                           performance mode
*              20 apr 07  david vallado
*                           misc fixes for constants
*              11 aug 06  david vallado
*                           chg lyddane choice back to strn3, constants, misc doc
*              15 dec 05  david vallado
*                           misc fixes
*              26 jul 05  david vallado
*                           fixes for paper
*                           note that each fix is preceded by a
*                           comment with "sgp4fix" and an explanation of
*                           what was changed
*              10 aug 04  david vallado
*                           2nd printing baseline working
*              14 may 01  david vallado
*                           2nd edition baseline
*                     80  norad
*                           original baseline
*       ----------------------------------------------------------------      */

// https://github.com/CelesTrak/fundamentals-of-astrodynamics/blob/main/software/cpp/SGP4/SGP4/SGP4.h

#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

#define SGP4Version  "SGP4 Version 2020-07-13"

// -------------------------- structure declarations ----------------------------
typedef enum
{
  wgs72old,
  wgs72,
  wgs84
} gravconsttype;

typedef struct elsetrec
{
  char      satnum[6];
  int       epochyr, epochtynumrev;
  int       error;
  char      operationmode;
  char      init, method;

  /* Near Earth */
  int    isimp;
  float aycof  , con41  , cc1    , cc4      , cc5    , d2      , d3   , d4    ,
         delmo  , eta    , argpdot, omgcof   , sinmao , t       , t2cof, t3cof ,
         t4cof  , t5cof  , x1mth2 , x7thm1   , mdot   , nodedot, xlcof , xmcof ,
         nodecf;

  /* Deep Space */
  int    irez;
  float d2201  , d2211  , d3210  , d3222    , d4410  , d4422   , d5220 , d5232 ,
         d5421  , d5433  , dedt   , del1     , del2   , del3    , didt  , dmdt  ,
         dnodt  , domdt  , e3     , ee2      , peo    , pgho    , pho   , pinco ,
         plo    , se2    , se3    , sgh2     , sgh3   , sgh4    , sh2   , sh3   ,
         si2    , si3    , sl2    , sl3      , sl4    , gsto    , xfact , xgh2  ,
         xgh3   , xgh4   , xh2    , xh3      , xi2    , xi3     , xl2   , xl3   ,
         xl4    , xlamo  , zmol   , zmos     , atime  , xli     , xni;

  float a, altp, alta, epochdays, jdsatepoch, jdsatepochF, nddot, ndot,
	     bstar, rcse, inclo, nodeo, ecco, argpo, mo, no_kozai;
  // sgp4fix add new variables from tle
  char  classification, intldesg[11];
  int   ephtype;
  long  elnum    , revnum;
  // sgp4fix add unkozai'd variable
  float no_unkozai;
  // sgp4fix add singly averaged variables
  float am     , em     , im     , Om       , om     , mm      , nm;
  // sgp4fix add constant parameters to eliminate mutliple calls during execution
  float tumin, mus, radiusearthkm, xke, j2, j3, j4, j3oj2;

  //       Additional elements to capture relevant TLE and object information:       
  long dia_mm; // RSO dia in mm
  float period_sec; // Period in seconds
  unsigned char active; // "Active S/C" flag (0=n, 1=y) 
  unsigned char not_orbital; // "Orbiting S/C" flag (0=n, 1=y)  
  float rcs_m2; // "RCS (m^2)" storage  

} elsetrec;


namespace SGP4Funcs 
{

	//	public class SGP4Class
	//	{

	bool sgp4init
		(
		gravconsttype whichconst, char opsmode, const char satn[9], const float epoch,
		const float xbstar, const float xndot, const float xnddot, const float xecco, const float xargpo,
		const float xinclo, const float xmo, const float xno,
		const float xnodeo, elsetrec& satrec
		);

	bool sgp4
		(
		// no longer need gravconsttype whichconst, all data contained in satrec
		elsetrec& satrec, float tsince,
		float r[3], float v[3]
		);

	void getgravconst
		(
		gravconsttype whichconst,
		float& tumin,
		float& mus,
		float& radiusearthkm,
		float& xke,
		float& j2,
		float& j3,
		float& j4,
		float& j3oj2
		);

	// older sgp4io methods
	void twoline2rv
		(
		char      longstr1[130], char longstr2[130],
		char      typerun, char typeinput, char opsmode,
		gravconsttype       whichconst,
		float& startmfe, float& stopmfe, float& deltamin,
		elsetrec& satrec
		);

	// older sgp4ext methods
	float  gstime_SGP4
		(
		float jdut1
		);

	float  sgn_SGP4
		(
		float x
		);

	float  mag_SGP4
		(
		float x[3]
		);

	void    cross_SGP4
		(
		float vec1[3], float vec2[3], float outvec[3]
		);

	float  dot_SGP4
		(
		float x[3], float y[3]
		);

	float  angle_SGP4
		(
		float vec1[3],
		float vec2[3]
		);

	void    newtonnu_SGP4
		(
		float ecc, float nu,
		float& e0, float& m
		);

	float  asinh_SGP4
		(
		float xval
		);

	void    rv2coe_SGP4
		(
		float r[3], float v[3], float mus,
		float& p, float& a, float& ecc, float& incl, float& omega, float& argp,
		float& nu, float& m, float& arglat, float& truelon, float& lonper
		);

	void    jday_SGP4
		(
		int year, int mon, int day, int hr, int minute, float sec,
		float& jd, float& jdFrac
		);

	void    days2mdhms_SGP4
		(
		int year, float days,
		int& mon, int& day, int& hr, int& minute, float& sec
		);

	void    invjday_SGP4
		(
		float jd, float jdFrac,
		int& year, int& mon, int& day,
		int& hr, int& minute, float& sec
		);


}  // namespace

#endif
