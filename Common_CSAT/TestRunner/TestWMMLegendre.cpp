#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "wmm_legendre.hpp"

#include <cmath>

constexpr float DEG_TO_RAD = M_PIf / 180.0f; // Conversion factor from degrees to radians
constexpr float RAD_TO_DEG = 180.0f / M_PIf; // Conversion factor from radians to degrees

TEST_CASE("Associated Legendre Polynomial Tests: 0.5")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;
    constexpr std::array<float, N + 1> expected = {1.f, 0.5000f, 0.8660f, -0.1250f, 0.7500f, 0.6495f, -0.4375f, 0.1326f,
                                                   0.7262f, 0.5135f, -0.2891f, -0.4279f, 0.3144f, 0.6793f, 0.4160f};

    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    bool result = MAG_PcupLow<nMax, N>(Pcup, dPcup, 0.5f);
    (void)Pcup;
    (void)dPcup;
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK(expected[k] == doctest::Approx(Pcup[k]).epsilon(1e-4f));
            }
            ++k;
        }
    }
}

TEST_CASE("Associated Legendre Polynomial Tests MAG_PcupHigh: 0.5")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;
    constexpr std::array<float, N + 1> expected = {1.0000f, 0.5000f, 0.8660f, -0.1250f, 0.7500f, 0.6495f, -0.4375f, 0.1326f,
                                                   0.7262f, 0.5135f, -0.2891f, -0.4279f, 0.3144f, 0.6793f, 0.4160f};

    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    bool result = MAG_PcupHigh<nMax, N>(Pcup, dPcup, 0.5f);
    (void)Pcup;
    (void)dPcup;
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK(expected[k] == doctest::Approx(Pcup[k]).epsilon(1e-4f));
            }
            ++k;
        }
    }
}

TEST_CASE("Associated Legendre Polynomial Tests: 0.1")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;
    constexpr std::array<float, N + 1> expected = {1.0000f, 0.1000f, 0.9950f, -0.4850f, 0.1723f, 0.8574f, -0.1475f,
                                                   -0.5788f, 0.1917f, 0.7787f, 0.3379f, -0.2305f, -0.5147f, 0.2060f, 0.7248f};

    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    bool result = MAG_PcupLow<nMax, N>(Pcup, dPcup, 0.1f);
    (void)Pcup;
    (void)dPcup;
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK(expected[k] == doctest::Approx(Pcup[k]).epsilon(1e-4f));
            }
            ++k;
        }
    }
}

TEST_CASE("Associated Legendre Polynomial Tests: 0.9")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;
    constexpr std::array<float, N + 1> expected = {1.000000f, 0.900000f, 0.435890f, 0.715000f, 0.679485f, 0.164545f,
                                                   0.472500f, 0.814127f, 0.331140f, 0.065474f, 0.207938f, 0.828077f,
                                                   0.496016f, 0.155906f, 0.026696f};

    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    bool result = MAG_PcupLow<nMax, N>(Pcup, dPcup, 0.9f);
    (void)Pcup;
    (void)dPcup;
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK(expected[k] == doctest::Approx(Pcup[k]).epsilon(1e-4f));
            }
            ++k;
        }
    }
}

TEST_CASE("Associated Legendre Polynomial Tests: -0.9")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;
    constexpr std::array<float, N + 1> expected = {1.000000f, -0.900000f, 0.435890f, 0.715000f, -0.679485f, 0.164545f,
                                                   -0.472500f, 0.814127f, -0.331140f, 0.065474f, 0.207938f, -0.828077f,
                                                   0.496016f, -0.155906f, 0.026696f};

    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    bool result = MAG_PcupLow<nMax, N>(Pcup, dPcup, -0.9f);
    (void)Pcup;
    (void)dPcup;
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK(expected[k] == doctest::Approx(Pcup[k]).epsilon(1e-4f));
            }
            ++k;
        }
    }
}

TEST_CASE("Associated Legendre Polynomial Tests MAG_PcupHigh: -0.9")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;
    constexpr std::array<float, N + 1> expected = {1.000000f, -0.900000f, 0.435890f, 0.715000f, -0.679485f, 0.164545f,
                                                   -0.472500f, 0.814127f, -0.331140f, 0.065474f, 0.207938f, -0.828077f,
                                                   0.496016f, -0.155906f, 0.026696f};

    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    bool result = MAG_PcupHigh<nMax, N>(Pcup, dPcup, -0.9f);
    (void)Pcup;
    (void)dPcup;
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK(expected[k] == doctest::Approx(Pcup[k]).epsilon(1e-4f));
            }
            ++k;
        }
    }
}

TEST_CASE("Derivative Associated Legendre Polynomial Tests: 0.5")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;

    std::array<float, N> Pcuph{};
    std::array<float, N> Pcupl{};
    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    const float angle = 1.0472f;
    const float delta = 1e-3f;
    bool result = false;
    result = MAG_PcupLow<nMax, N>(Pcuph, dPcup, sinf(angle + 0.5f * delta));
    result = MAG_PcupLow<nMax, N>(Pcupl, dPcup, sinf(angle - 0.5f * delta));
    result = MAG_PcupLow<nMax, N>(Pcup, dPcup, sinf(angle));
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK((Pcuph[k]-Pcupl[k])/delta == doctest::Approx(dPcup[k]).epsilon(1e-3f)); // -expected to convert to latitude derivative
            }
            ++k;
        }
    }
}

TEST_CASE("Derivative Associated Legendre Polynomial Tests: -0.5")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;

    std::array<float, N> Pcuph{};
    std::array<float, N> Pcupl{};
    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    const float angle = -1.0472f;
    const float delta = 1e-3f;
    bool result = false;
    result = MAG_PcupLow<nMax, N>(Pcuph, dPcup, sinf(angle + 0.5f * delta));
    result = MAG_PcupLow<nMax, N>(Pcupl, dPcup, sinf(angle - 0.5f * delta));
    result = MAG_PcupLow<nMax, N>(Pcup, dPcup, sinf(angle));
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK((Pcuph[k]-Pcupl[k])/delta == doctest::Approx(dPcup[k]).epsilon(1e-3f)); // -expected to convert to latitude derivative
            }
            ++k;
        }
    }
}

TEST_CASE("Derivative Associated Legendre Polynomial Tests MAG_PcupHigh: 0.5")
{
    using namespace wmm_model;

    constexpr uint16_t nMax = 4;
    constexpr size_t N = (nMax + 1) * (nMax + 2) / 2;

    std::array<float, N> Pcuph{};
    std::array<float, N> Pcupl{};
    std::array<float, N> Pcup{};
    std::array<float, N> dPcup{};

    const float angle = 1.0472f;
    const float delta = 1e-3f;
    bool result = false;
    result = MAG_PcupHigh<nMax, N>(Pcuph, dPcup, sinf(angle + 0.5f * delta));
    result = MAG_PcupHigh<nMax, N>(Pcupl, dPcup, sinf(angle - 0.5f * delta));
    result = MAG_PcupHigh<nMax, N>(Pcup, dPcup, sinf(angle));
    CHECK(result == true);

    uint16_t k = 0;
    for (uint16_t i = 0; i <= nMax; ++i)
    {
        for (uint16_t j = 0; j <= i; ++j)
        {
            char name[128];
            sprintf(name, "n=%d, m=%d", i, j);
            SUBCASE(name)
            {
                CHECK((Pcuph[k]-Pcupl[k])/delta == doctest::Approx(dPcup[k]).epsilon(1e-3f)); // -expected to convert to latitude derivative
            }
            ++k;
        }
    }
}

