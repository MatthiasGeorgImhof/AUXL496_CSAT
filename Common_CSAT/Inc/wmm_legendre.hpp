#ifndef WMM_LEGENDRE_HPP
#define WMM_LEGENDRE_HPP

#include <array>
#include <string>
#include <cmath>
#include <cstdint> // Include for uint16_t

// https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2020/WMM2020_Report.pdf

namespace wmm_model
{

    template <uint16_t nMax, size_t N>
    bool MAG_PcupLow(std::array<float, N> &Pcup, std::array<float, N> &dPcup, float x);

    template <uint16_t nMax, size_t N>
    bool MAG_PcupHigh(std::array<float, N> &Pcup, std::array<float, N> &dPcup, float x);

    template <uint16_t nMax, size_t N>
    bool MAG_PcupLow(std::array<float, N> &Pcup, std::array<float, N> &dPcup, float x)

    /*   This function evaluates all of the Schmidt-semi normalized associated Legendre
            functions up to degree nMax.

            Calling Parameters:
                    INPUT
                            nMax:	 Maximum spherical harmonic degree to compute.
                            x:		cos(colatitude) or sin(latitude).

                    OUTPUT
                            Pcup:	A vector of all associated Legendgre polynomials evaluated at
                                            x up to nMax.
                       dPcup: Derivative of Pcup(x) with respect to latitude

            Notes: Overflow may occur if nMax > 20 , especially for high-latitudes.
            Use MAG_PcupHigh for large nMax.

       Written by Manoj Nair, June, 2009 . Manoj.C.Nair@Noaa.Gov.

      Note: In geomagnetism, the derivatives of ALF are usually found with
      respect to the colatitudes. Here the derivatives are found with respect
      to the latitude. The difference is a sign reversal for the derivative of
      the Associated Legendre Functions.
     */
    {
        static_assert((nMax + 1) * (nMax + 2) / 2 == N);
        uint16_t n, m, index, index1, index2;
        float k, z;
        Pcup[0] = 1.0f;
        dPcup[0] = 0.0f;
        z = sqrtf((1.0f - x) * (1.0f + x));

        std::array<float, N + 1> schmidtQuasiNorm;

        /*	 First,	Compute the Gauss-normalized associated Legendre  functions*/

        for (n = 1; n <= nMax; n++)
        {
            for (m = 0; m <= n; m++)
            {
                index = static_cast<uint16_t>((n * (n + 1) / 2 + m));

                if (n == m)
                {
                    index1 = static_cast<uint16_t>((n - 1) * n / 2 + m - 1);
                    Pcup[index] = z * Pcup[index1];
                    dPcup[index] = z * dPcup[index1] + x * Pcup[index1];
                }
                else if (n == 1 && m == 0)
                {
                    index1 = static_cast<uint16_t>((n - 1) * n / 2 + m);
                    Pcup[index] = x * Pcup[index1];
                    dPcup[index] = x * dPcup[index1] - z * Pcup[index1];
                }
                else if (n > 1 && n != m)
                {
                    index1 = static_cast<uint16_t>((n - 2) * (n - 1) / 2 + m);
                    index2 = static_cast<uint16_t>((n - 1) * n / 2 + m);
                    if (m > n - 2)
                    {
                        Pcup[index] = x * Pcup[index2];
                        dPcup[index] = x * dPcup[index2] - z * Pcup[index2];
                    }
                    else
                    {
                        k = (float)(((n - 1) * (n - 1)) - (m * m)) / (float)((2 * n - 1) * (2 * n - 3));
                        Pcup[index] = x * Pcup[index2] - k * Pcup[index1];
                        dPcup[index] = x * dPcup[index2] - z * Pcup[index2] - k * dPcup[index1];
                    }
                }
            }
        }
        /* Compute the ration between the the Schmidt quasi-normalized associated Legendre
         * functions and the Gauss-normalized version. */

        schmidtQuasiNorm[0] = 1.0;
        for (n = 1; n <= nMax; n++)
        {
            index = static_cast<uint16_t>((n * (n + 1) / 2));
            index1 = static_cast<uint16_t>((n - 1) * n / 2);
            /* for m = 0 */
            schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * (float)(2 * n - 1) / (float)n;

            for (m = 1; m <= n; m++)
            {
                index = static_cast<uint16_t>((n * (n + 1) / 2 + m));
                index1 = static_cast<uint16_t>((n * (n + 1) / 2 + m - 1));
                schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * sqrtf((float)((n - m + 1) * (m == 1 ? 2 : 1)) / (float)(n + m));
            }
        }

        /* Converts the  Gauss-normalized associated Legendre
                  functions to the Schmidt quasi-normalized version using pre-computed
                  relation stored in the variable schmidtQuasiNorm */

        for (n = 1; n <= nMax; n++)
        {
            for (m = 0; m <= n; m++)
            {
                index = static_cast<uint16_t>((n * (n + 1) / 2 + m));
                Pcup[index] = Pcup[index] * schmidtQuasiNorm[index];
                dPcup[index] = -dPcup[index] * schmidtQuasiNorm[index];
                /* The sign is changed since the new WMM routines use derivative with respect to latitude
                insted of co-latitude */
            }
        }

        return true;
    } /*MAG_PcupLow */

    template <uint16_t nMax, size_t N>
    bool MAG_PcupHigh(std::array<float, N> &Pcup, std::array<float, N> &dPcup, float x)

    /*	This function evaluates all of the Schmidt-semi normalized associated Legendre
            functions up to degree nMax. The functions are initially scaled by
            10^280 sin^m in order to minimize the effects of underflow at large m
            near the poles (see Holmes and Featherstone 2002, J. Geodesy, 76, 279-299).
            Note that this function performs the same operation as MAG_PcupLow.
            However this function also can be used for high degree (large nMax) models.
IGRF_MODEL_HPP
            Calling Parameters:
                    INPUT
                            nMax:	 Maximum spherical harmonic degree to compute.
                            x:		cos(colatitude) or sin(latitude).

                    OUTPUT
                            Pcup:	A vector of all associated Legendgre polynomials evaluated at
                                            x up to nMax. The lenght must by greater or equal to (nMax+1)*(nMax+2)/2.
                      dPcup:   Derivative of Pcup(x) with respect to latitude

                    CALLS : none
            Notes:



      Adopted from the FORTRAN code written by Mark Wieczorek September 25, 2005.

      Manoj Nair, Nov, 2009 Manoj.C.Nair@Noaa.Gov

      Change from the previous version
      The prevous version computes the derivatives as
      dP(n,m)(x)/dx, where x = sin(latitude) (or cos(colatitude) ).
      However, the WMM Geomagnetic routines requires dP(n,m)(x)/dlatitude.
      Hence the derivatives are multiplied by sin(latitude).
      Removed the options for CS phase and normalizations.

      Note: In geomagnetism, the derivatives of ALF are usually found with
      respect to the colatitudes. Here the derivatives are found with respect
      to the latitude. The difference is a sign reversal for the derivative of
      the Associated Legendre Functions.

      The derivatives can't be computed for latitude = |90| degrees.
     */
    {
        static_assert((nMax + 1) * (nMax + 2) / 2 == N);

        float pm2, pm1, pmm, plm, rescalem, z;
        uint16_t k, kstart, m, n;

        z = sqrtf((1.0f - x) * (1.0f + x));

        if (z == 0)
            return false;
        if (fabs(x) == 1.0)
            return false;

        std::array<float, N + 1> f1;
        std::array<float, N + 1> f2;
        std::array<float, N + 1> PreSqr;

        constexpr float scalef = 1.0e-32f;

        for (n = 0; n <= 2 * nMax + 1; ++n)
        {
            PreSqr[n] = sqrtf((float)(n));
        }

        k = 2;

        for (n = 2; n <= nMax; n++)
        {
            k = k + 1;
            f1[k] = (float)(2 * n - 1) / (float)(n);
            f2[k] = (float)(n - 1) / (float)(n);
            for (m = 1; m <= n - 2; m++)
            {
                k = k + 1;
                f1[k] = (float)(2 * n - 1) / PreSqr[n + m] / PreSqr[n - m];
                f2[k] = PreSqr[n - m - 1U] * PreSqr[n + m - 1U] / PreSqr[n + m] / PreSqr[n - m];
            }
            k = k + 2;
        }

        /*z = sin (geocentric latitude) */

        pm2 = 1.0;
        Pcup[0] = 1.0;
        dPcup[0] = 0.0;
        if (nMax == 0)
            return true;

        pm1 = x;
        Pcup[1] = pm1;
        dPcup[1] = z;
        k = 1;

        for (n = 2; n <= nMax; n++)
        {
            k = k + n;
            plm = f1[k] * x * pm1 - f2[k] * pm2;
            Pcup[k] = plm;
            dPcup[k] = (float)(n) * (pm1 - x * plm) / z;
            pm2 = pm1;
            pm1 = plm;
        }

        pmm = PreSqr[2] * scalef;
        rescalem = 1.0f / scalef;
        kstart = 0;

        for (m = 1; m <= nMax - 1; ++m)
        {
            rescalem = rescalem * z;

            /* Calculate Pcup(m,m)*/
            kstart = static_cast<uint16_t>(kstart + m + 1U);
            pmm = pmm * PreSqr[2U * m + 1U] / PreSqr[2U * m];
            Pcup[kstart] = pmm * rescalem / PreSqr[2U * m + 1U];
            dPcup[kstart] = -((float)(m)*x * Pcup[kstart] / z);
            pm2 = pmm / PreSqr[2U * m + 1U];
            /* Calculate Pcup(m+1,m)*/
            k = static_cast<uint16_t>(kstart + m + 1U);
            pm1 = x * PreSqr[2U * m + 1U] * pm2;
            Pcup[k] = pm1 * rescalem;
            dPcup[k] = ((pm2 * rescalem) * PreSqr[2U * m + 1U] - x * (float)(m + 1) * Pcup[k]) / z;
            /* Calculate Pcup(n,m)*/
            for (n = m + 2U; n <= nMax; ++n)
            {
                k = k + n;
                plm = x * f1[k] * pm1 - f2[k] * pm2;
                Pcup[k] = plm * rescalem;
                dPcup[k] = (PreSqr[n + m] * PreSqr[n - m] * (pm1 * rescalem) - (float)(n)*x * Pcup[k]) / z;
                pm2 = pm1;
                pm1 = plm;
            }
        }

        /* Calculate Pcup(nMax,nMax)*/
        rescalem = rescalem * z;
        kstart = static_cast<uint16_t>(kstart + m + 1);
        pmm = pmm / PreSqr[2 * nMax];
        Pcup[kstart] = pmm * rescalem;
        dPcup[kstart] = -(float)(nMax)*x * Pcup[kstart] / z;

        return true;
    } /* MAG_PcupHigh */

} // namespace wmm_model

#endif // WMM_LEGENDRE_HPP