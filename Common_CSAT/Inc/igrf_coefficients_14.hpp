#ifndef IGRF_COEFFICIENTS_14_H
#define IGRF_COEFFICIENTS_14_H

#include <array>
#include <algorithm>
#include <ranges>
#include <limits>
#include "magnetic_model.hpp"

namespace magnetic_model {

#ifndef MAX_ORDER
#define MAX_ORDER 13 // Maximum order for IGRF model
#endif

static constexpr std::array magneticGaussCoefficients = {
    GaussCoefficient {1, 0, -29350.0f, 0.0f, 12.6f, 0.0f},
    GaussCoefficient {1, 1, -1410.3f, 4545.5f, 10.0f, -21.5f},
#if MAX_ORDER >= 2
    GaussCoefficient {2, 0, -2556.2f, 0.0f, -11.2f, 0.0f},
    GaussCoefficient {2, 1, 2950.9f, -3133.6f, -5.3f, -27.3f},
    GaussCoefficient {2, 2, 1648.7f, -814.2f, -8.3f, -11.1f},
#if MAX_ORDER >= 3
    GaussCoefficient {3, 0, 1360.9f, 0.0f, -1.5f, 0.0f},
    GaussCoefficient {3, 1, -2404.2f, -56.9f, -4.4f, 3.8f},
    GaussCoefficient {3, 2, 1243.8f, 237.6f, 0.4f, -0.2f},
    GaussCoefficient {3, 3, 453.4f, -549.6f, -15.6f, -3.9f},
#if MAX_ORDER >= 4
    GaussCoefficient {4, 0, 894.7f, 0.0f, -1.7f, 0.0f},
    GaussCoefficient {4, 1, 799.6f, 278.6f, -2.3f, -1.3f},
    GaussCoefficient {4, 2, 55.8f, -134.0f, -5.8f, 4.1f},
    GaussCoefficient {4, 3, -281.1f, 212.0f, 5.4f, 1.6f},
    GaussCoefficient {4, 4, 12.0f, -375.4f, -6.8f, -4.1f},
#if MAX_ORDER >= 5
    GaussCoefficient {5, 0, -232.9f, 0.0f, 0.6f, 0.0f},
    GaussCoefficient {5, 1, 369.0f, 45.3f, 1.3f, -0.5f},
    GaussCoefficient {5, 2, 187.2f, 220.0f, 0.0f, 2.1f},
    GaussCoefficient {5, 3, -138.7f, -122.9f, 0.7f, 0.5f},
    GaussCoefficient {5, 4, -141.9f, 42.9f, 2.3f, 1.7f},
    GaussCoefficient {5, 5, 20.9f, 106.2f, 1.0f, 1.9f},
#if MAX_ORDER >= 6
    GaussCoefficient {6, 0, 64.3f, 0.0f, -0.2f, 0.0f},
    GaussCoefficient {6, 1, 63.8f, -18.4f, -0.3f, 0.3f},
    GaussCoefficient {6, 2, 76.7f, 16.8f, 0.8f, -1.6f},
    GaussCoefficient {6, 3, -115.7f, 48.9f, 1.2f, -0.4f},
    GaussCoefficient {6, 4, -40.9f, -59.8f, -0.8f, 0.8f},
    GaussCoefficient {6, 5, 14.9f, 10.9f, 0.4f, 0.7f},
    GaussCoefficient {6, 6, -60.8f, 72.8f, 0.9f, 0.9f},
#if MAX_ORDER >= 7
    GaussCoefficient {7, 0, 79.6f, 0.0f, -0.1f, 0.0f},
    GaussCoefficient {7, 1, -76.9f, -48.9f, -0.1f, 0.6f},
    GaussCoefficient {7, 2, -8.8f, -14.4f, -0.1f, 0.5f},
    GaussCoefficient {7, 3, 59.3f, -1.0f, 0.5f, -0.7f},
    GaussCoefficient {7, 4, 15.8f, 23.5f, -0.1f, 0.0f},
    GaussCoefficient {7, 5, 2.5f, -7.4f, -0.8f, -0.9f},
    GaussCoefficient {7, 6, -11.2f, -25.1f, -0.8f, 0.5f},
    GaussCoefficient {7, 7, 14.3f, -2.2f, 0.9f, -0.3f},
#if MAX_ORDER >= 8
    GaussCoefficient {8, 0, 23.1f, 0.0f, -0.1f, 0.0f},
    GaussCoefficient {8, 1, 10.9f, 7.2f, 0.2f, -0.3f},
    GaussCoefficient {8, 2, -17.5f, -12.6f, 0.0f, 0.4f},
    GaussCoefficient {8, 3, 2.0f, 11.5f, 0.4f, -0.3f},
    GaussCoefficient {8, 4, -21.8f, -9.7f, -0.1f, 0.4f},
    GaussCoefficient {8, 5, 16.9f, 12.7f, 0.3f, -0.5f},
    GaussCoefficient {8, 6, 14.9f, 0.7f, 0.1f, -0.6f},
    GaussCoefficient {8, 7, -16.8f, -5.2f, 0.0f, 0.3f},
    GaussCoefficient {8, 8, 1.0f, 3.9f, 0.3f, 0.2f},
#if MAX_ORDER >= 9
    GaussCoefficient {9, 0, 4.7f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {9, 1, 8.0f, -24.8f, 0.0f, 0.0f},
    GaussCoefficient {9, 2, 3.0f, 12.1f, 0.0f, 0.0f},
    GaussCoefficient {9, 3, -0.2f, 8.3f, 0.0f, 0.0f},
    GaussCoefficient {9, 4, -2.5f, -3.4f, 0.0f, 0.0f},
    GaussCoefficient {9, 5, -13.1f, -5.3f, 0.0f, 0.0f},
    GaussCoefficient {9, 6, 2.4f, 7.2f, 0.0f, 0.0f},
    GaussCoefficient {9, 7, 8.6f, -0.6f, 0.0f, 0.0f},
    GaussCoefficient {9, 8, -8.7f, 0.8f, 0.0f, 0.0f},
    GaussCoefficient {9, 9, -12.8f, 9.8f, 0.0f, 0.0f},
#if MAX_ORDER >= 10
    GaussCoefficient {10, 0, -1.3f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {10, 1, -6.4f, 3.3f, 0.0f, 0.0f},
    GaussCoefficient {10, 2, 0.2f, 0.1f, 0.0f, 0.0f},
    GaussCoefficient {10, 3, 2.0f, 2.5f, 0.0f, 0.0f},
    GaussCoefficient {10, 4, -1.0f, 5.4f, 0.0f, 0.0f},
    GaussCoefficient {10, 5, -0.5f, -9.0f, 0.0f, 0.0f},
    GaussCoefficient {10, 6, -0.9f, 0.4f, 0.0f, 0.0f},
    GaussCoefficient {10, 7, 1.5f, -4.2f, 0.0f, 0.0f},
    GaussCoefficient {10, 8, 0.9f, -3.8f, 0.0f, 0.0f},
    GaussCoefficient {10, 9, -2.6f, 0.9f, 0.0f, 0.0f},
    GaussCoefficient {10, 10, -3.9f, -9.0f, 0.0f, 0.0f},
#if MAX_ORDER >= 11
    GaussCoefficient {11, 0, 3.0f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {11, 1, -1.4f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {11, 2, -2.5f, 2.8f, 0.0f, 0.0f},
    GaussCoefficient {11, 3, 2.4f, -0.6f, 0.0f, 0.0f},
    GaussCoefficient {11, 4, -0.6f, 0.1f, 0.0f, 0.0f},
    GaussCoefficient {11, 5, 0.0f, 0.5f, 0.0f, 0.0f},
    GaussCoefficient {11, 6, -0.6f, -0.3f, 0.0f, 0.0f},
    GaussCoefficient {11, 7, -0.1f, -1.2f, 0.0f, 0.0f},
    GaussCoefficient {11, 8, 1.1f, -1.7f, 0.0f, 0.0f},
    GaussCoefficient {11, 9, -1.0f, -2.9f, 0.0f, 0.0f},
    GaussCoefficient {11, 10, -0.1f, -1.8f, 0.0f, 0.0f},
    GaussCoefficient {11, 11, 2.6f, -2.3f, 0.0f, 0.0f},
#if MAX_ORDER >= 12
    GaussCoefficient {12, 0, -2.0f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {12, 1, -0.1f, -1.2f, 0.0f, 0.0f},
    GaussCoefficient {12, 2, 0.4f, 0.6f, 0.0f, 0.0f},
    GaussCoefficient {12, 3, 1.2f, 1.0f, 0.0f, 0.0f},
    GaussCoefficient {12, 4, -1.2f, -1.5f, 0.0f, 0.0f},
    GaussCoefficient {12, 5, 0.6f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {12, 6, 0.5f, 0.6f, 0.0f, 0.0f},
    GaussCoefficient {12, 7, 0.5f, -0.2f, 0.0f, 0.0f},
    GaussCoefficient {12, 8, -0.1f, 0.8f, 0.0f, 0.0f},
    GaussCoefficient {12, 9, -0.5f, 0.1f, 0.0f, 0.0f},
    GaussCoefficient {12, 10, -0.2f, -0.9f, 0.0f, 0.0f},
    GaussCoefficient {12, 11, -1.2f, 0.1f, 0.0f, 0.0f},
    GaussCoefficient {12, 12, -0.7f, 0.2f, 0.0f, 0.0f},
#if MAX_ORDER >= 13
    GaussCoefficient {13, 0, 0.2f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {13, 1, -0.9f, -0.9f, 0.0f, 0.0f},
    GaussCoefficient {13, 2, 0.6f, 0.7f, 0.0f, 0.0f},
    GaussCoefficient {13, 3, 0.7f, 1.2f, 0.0f, 0.0f},
    GaussCoefficient {13, 4, -0.2f, -0.3f, 0.0f, 0.0f},
    GaussCoefficient {13, 5, 0.5f, -1.3f, 0.0f, 0.0f},
    GaussCoefficient {13, 6, 0.1f, -0.1f, 0.0f, 0.0f},
    GaussCoefficient {13, 7, 0.7f, 0.2f, 0.0f, 0.0f},
    GaussCoefficient {13, 8, 0.0f, -0.2f, 0.0f, 0.0f},
    GaussCoefficient {13, 9, 0.3f, 0.5f, 0.0f, 0.0f},
    GaussCoefficient {13, 10, 0.2f, 0.6f, 0.0f, 0.0f},
    GaussCoefficient {13, 11, -0.4f, -0.6f, 0.0f, 0.0f},
    GaussCoefficient {13, 12, -0.5f, -0.3f, 0.0f, 0.0f},
    GaussCoefficient {13, 13, -0.4f, -0.5f, 0.0f, 0.0f}
#endif // MAX_ORDER >= 13
#endif // MAX_ORDER >= 12
#endif // MAX_ORDER >= 11
#endif // MAX_ORDER >= 10
#endif // MAX_ORDER >= 9
#endif // MAX_ORDER >= 8
#endif // MAX_ORDER >= 7
#endif // MAX_ORDER >= 6
#endif // MAX_ORDER >= 5
#endif // MAX_ORDER >= 4
#endif // MAX_ORDER >= 3
#endif // MAX_ORDER >= 2    
};

constexpr size_t N_COFS = magneticGaussCoefficients.size();
constexpr int getN(const GaussCoefficient& gc) { return gc.n; }
static constexpr int NMAX = std::ranges::max(magneticGaussCoefficients | std::views::transform(getN));
static_assert((NMAX+1) * (NMAX+2) / 2 - 1 == N_COFS, "NMAX does not align with number of coefficients");

} // namespace magnetic_model

#endif // IGRF_COEFFICIENTS_14_H
