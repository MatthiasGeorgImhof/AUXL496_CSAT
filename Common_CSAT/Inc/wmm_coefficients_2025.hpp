#ifndef WMM_COEFFICIENTS_2025_H
#define WMM_COEFFICIENTS_2025_H

#include <array>
#include <algorithm>
#include <ranges>
#include <limits>
#include "magnetic_model.hpp"

namespace magnetic_model {

#ifndef MAX_ORDER
#define MAX_ORDER 12 // Maximum order for WMM model
#endif

static constexpr std::array magneticGaussCoefficients = {
    GaussCoefficient {1, 0, -29351.8f, 0.0f, 12.0f, 0.0f},
    GaussCoefficient {1, 1, -1410.8f, 4545.4f, 9.7f, -21.5f},
#if MAX_ORDER >= 2
    GaussCoefficient {2, 0, -2556.6f, 0.0f, -11.6f, 0.0f},
    GaussCoefficient {2, 1, 2951.1f, -3133.6f, -5.2f, -27.7f},
    GaussCoefficient {2, 2, 1649.3f, -815.1f, -8.0f, -12.1f},
#if MAX_ORDER >= 3
    GaussCoefficient {3, 0, 1361.0f, 0.0f, -1.3f, 0.0f},
    GaussCoefficient {3, 1, -2404.1f, -56.6f, -4.2f, 4.0f},
    GaussCoefficient {3, 2, 1243.8f, 237.5f, 0.4f, -0.3f},
    GaussCoefficient {3, 3, 453.6f, -549.5f, -15.6f, -4.1f},
#if MAX_ORDER >= 4
    GaussCoefficient {4, 0, 895.0f, 0.0f, -1.6f, 0.0f},
    GaussCoefficient {4, 1, 799.5f, 278.6f, -2.4f, -1.1f},
    GaussCoefficient {4, 2, 55.7f, -133.9f, -6.0f, 4.1f},
    GaussCoefficient {4, 3, -281.1f, 212.0f, 5.6f, 1.6f},
    GaussCoefficient {4, 4, 12.1f, -375.6f, -7.0f, -4.4f},
#if MAX_ORDER >= 5
    GaussCoefficient {5, 0, -233.2f, 0.0f, 0.6f, 0.0f},
    GaussCoefficient {5, 1, 368.9f, 45.4f, 1.4f, -0.5f},
    GaussCoefficient {5, 2, 187.2f, 220.2f, 0.0f, 2.2f},
    GaussCoefficient {5, 3, -138.7f, -122.9f, 0.6f, 0.4f},
    GaussCoefficient {5, 4, -142.0f, 43.0f, 2.2f, 1.7f},
    GaussCoefficient {5, 5, 20.9f, 106.1f, 0.9f, 1.9f},
#if MAX_ORDER >= 6
    GaussCoefficient {6, 0, 64.4f, 0.0f, -0.2f, 0.0f},
    GaussCoefficient {6, 1, 63.8f, -18.4f, -0.4f, 0.3f},
    GaussCoefficient {6, 2, 76.9f, 16.8f, 0.9f, -1.6f},
    GaussCoefficient {6, 3, -115.7f, 48.8f, 1.2f, -0.4f},
    GaussCoefficient {6, 4, -40.9f, -59.8f, -0.9f, 0.9f},
    GaussCoefficient {6, 5, 14.9f, 10.9f, 0.3f, 0.7f},
    GaussCoefficient {6, 6, -60.7f, 72.7f, 0.9f, 0.9f},
#if MAX_ORDER >= 7
    GaussCoefficient {7, 0, 79.5f, 0.0f, -0.0f, 0.0f},
    GaussCoefficient {7, 1, -77.0f, -48.9f, -0.1f, 0.6f},
    GaussCoefficient {7, 2, -8.8f, -14.4f, -0.1f, 0.5f},
    GaussCoefficient {7, 3, 59.3f, -1.0f, 0.5f, -0.8f},
    GaussCoefficient {7, 4, 15.8f, 23.4f, -0.1f, 0.0f},
    GaussCoefficient {7, 5, 2.5f, -7.4f, -0.8f, -1.0f},
    GaussCoefficient {7, 6, -11.1f, -25.1f, -0.8f, 0.6f},
    GaussCoefficient {7, 7, 14.2f, -2.3f, 0.8f, -0.2f},
#if MAX_ORDER >= 8
    GaussCoefficient {8, 0, 23.2f, 0.0f, -0.1f, 0.0f},
    GaussCoefficient {8, 1, 10.8f, 7.1f, 0.2f, -0.2f},
    GaussCoefficient {8, 2, -17.5f, -12.6f, 0.0f, 0.5f},
    GaussCoefficient {8, 3, 2.0f, 11.4f, 0.5f, -0.4f},
    GaussCoefficient {8, 4, -21.7f, -9.7f, -0.1f, 0.4f},
    GaussCoefficient {8, 5, 16.9f, 12.7f, 0.3f, -0.5f},
    GaussCoefficient {8, 6, 15.0f, 0.7f, 0.2f, -0.6f},
    GaussCoefficient {8, 7, -16.8f, -5.2f, -0.0f, 0.3f},
    GaussCoefficient {8, 8, 0.9f, 3.9f, 0.2f, 0.2f},
#if MAX_ORDER >= 9
    GaussCoefficient {9, 0, 4.6f, 0.0f, -0.0f, 0.0f},
    GaussCoefficient {9, 1, 7.8f, -24.8f, -0.1f, -0.3f},
    GaussCoefficient {9, 2, 3.0f, 12.2f, 0.1f, 0.3f},
    GaussCoefficient {9, 3, -0.2f, 8.3f, 0.3f, -0.3f},
    GaussCoefficient {9, 4, -2.5f, -3.3f, -0.3f, 0.3f},
    GaussCoefficient {9, 5, -13.1f, -5.2f, 0.0f, 0.2f},
    GaussCoefficient {9, 6, 2.4f, 7.2f, 0.3f, -0.1f},
    GaussCoefficient {9, 7, 8.6f, -0.6f, -0.1f, -0.2f},
    GaussCoefficient {9, 8, -8.7f, 0.8f, 0.1f, 0.4f},
    GaussCoefficient {9, 9, -12.9f, 10.0f, -0.1f, 0.1f},
#if MAX_ORDER >= 10
    GaussCoefficient {10, 0, -1.3f, 0.0f, 0.1f, 0.0f},
    GaussCoefficient {10, 1, -6.4f, 3.3f, 0.0f, 0.0f},
    GaussCoefficient {10, 2, 0.2f, 0.0f, 0.1f, -0.0f},
    GaussCoefficient {10, 3, 2.0f, 2.4f, 0.1f, -0.2f},
    GaussCoefficient {10, 4, -1.0f, 5.3f, -0.0f, 0.1f},
    GaussCoefficient {10, 5, -0.6f, -9.1f, -0.3f, -0.1f},
    GaussCoefficient {10, 6, -0.9f, 0.4f, 0.0f, 0.1f},
    GaussCoefficient {10, 7, 1.5f, -4.2f, -0.1f, 0.0f},
    GaussCoefficient {10, 8, 0.9f, -3.8f, -0.1f, -0.1f},
    GaussCoefficient {10, 9, -2.7f, 0.9f, -0.0f, 0.2f},
    GaussCoefficient {10, 10, -3.9f, -9.1f, -0.0f, -0.0f},
#if MAX_ORDER >= 11
    GaussCoefficient {11, 0, 2.9f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {11, 1, -1.5f, 0.0f, -0.0f, -0.0f},
    GaussCoefficient {11, 2, -2.5f, 2.9f, 0.0f, 0.1f},
    GaussCoefficient {11, 3, 2.4f, -0.6f, 0.0f, -0.0f},
    GaussCoefficient {11, 4, -0.6f, 0.2f, 0.0f, 0.1f},
    GaussCoefficient {11, 5, -0.1f, 0.5f, -0.1f, -0.0f},
    GaussCoefficient {11, 6, -0.6f, -0.3f, 0.0f, -0.0f},
    GaussCoefficient {11, 7, -0.1f, -1.2f, -0.0f, 0.1f},
    GaussCoefficient {11, 8, 1.1f, -1.7f, -0.1f, -0.0f},
    GaussCoefficient {11, 9, -1.0f, -2.9f, -0.1f, 0.0f},
    GaussCoefficient {11, 10, -0.2f, -1.8f, -0.1f, 0.0f},
    GaussCoefficient {11, 11, 2.6f, -2.3f, -0.1f, 0.0f},
#if MAX_ORDER >= 12
    GaussCoefficient {12, 0, -2.0f, 0.0f, 0.0f, 0.0f},
    GaussCoefficient {12, 1, -0.2f, -1.3f, 0.0f, -0.0f},
    GaussCoefficient {12, 2, 0.3f, 0.7f, -0.0f, 0.0f},
    GaussCoefficient {12, 3, 1.2f, 1.0f, -0.0f, -0.1f},
    GaussCoefficient {12, 4, -1.3f, -1.4f, -0.0f, 0.1f},
    GaussCoefficient {12, 5, 0.6f, -0.0f, -0.0f, -0.0f},
    GaussCoefficient {12, 6, 0.6f, 0.6f, 0.1f, -0.0f},
    GaussCoefficient {12, 7, 0.5f, -0.1f, -0.0f, -0.0f},
    GaussCoefficient {12, 8, -0.1f, 0.8f, 0.0f, 0.0f},
    GaussCoefficient {12, 9, -0.4f, 0.1f, 0.0f, -0.0f},
    GaussCoefficient {12, 10, -0.2f, -1.0f, -0.1f, -0.0f},
    GaussCoefficient {12, 11, -1.3f, 0.1f, -0.0f, 0.0f},
    GaussCoefficient {12, 12, -0.7f, 0.2f, -0.1f, -0.1f}
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

} // namespace magnetic_coefficients

#endif // WMM_COEFFICIENTS_2025_H