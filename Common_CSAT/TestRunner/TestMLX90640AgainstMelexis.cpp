#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "MLX90640Calibration.hpp"   // your constexpr parser
#include "MLX90640EEPROM.h"
#include "3rdParty/MLX90640_API.h"            // Melexis reference

TEST_CASE("MLX90640: our calibration matches Melexis ExtractParameters")
{
    // 1) Prepare a mutable EEPROM buffer for the Melexis API
    uint16_t ee[832];
    for (int i = 0; i < 832; ++i)
    {
        ee[i] = MLX90640_EEPROM[i];
    }

    // 2) Run Melexis reference extraction
    paramsMLX90640 ref{};
    int err = MLX90640_ExtractParameters(ee, &ref);
    CHECK(err == MLX90640_NO_ERROR);

    // 3) Our constexpr calibration
    const MLX90640_Calibration& ours = MLX90640_CAL;

    // ---- Globals ----
    CHECK(ours.kVdd  == ref.kVdd);
    CHECK(ours.vdd25 == ref.vdd25);

    CHECK(ours.KvPTAT == doctest::Approx(ref.KvPTAT));
    CHECK(ours.KtPTAT == doctest::Approx(ref.KtPTAT));
    CHECK(ours.vPTAT25 == ref.vPTAT25);
    CHECK(ours.alphaPTAT == doctest::Approx(ref.alphaPTAT));

    CHECK(ours.gainEE == ref.gainEE);
    CHECK(ours.tgc == doctest::Approx(ref.tgc));
    CHECK(ours.cpKv == doctest::Approx(ref.cpKv));
    CHECK(ours.cpKta == doctest::Approx(ref.cpKta));

    CHECK(ours.resolutionEE == ref.resolutionEE);
    CHECK(ours.calibrationModeEE == ref.calibrationModeEE);

    CHECK(ours.KsTa == doctest::Approx(ref.KsTa));
    for (int i = 0; i < 5; ++i)
    {
        CHECK(ours.ksTo[i] == doctest::Approx(ref.ksTo[i]));
        CHECK(ours.ct[i] == ref.ct[i]);
    }

    // ---- CP parameters ----
    for (int i = 0; i < 2; ++i)
    {
        CHECK(ours.cpAlpha[i] == doctest::Approx(ref.cpAlpha[i]));
        CHECK(ours.cpOffset[i] == ref.cpOffset[i]);
    }

    // ---- Chess / IL compensation ----
    for (int i = 0; i < 3; ++i)
    {
        CHECK(ours.ilChessC[i] == doctest::Approx(ref.ilChessC[i]));
    }

    // ---- Scales ----
    CHECK(ours.alphaScale == ref.alphaScale);
    CHECK(ours.ktaScale == ref.ktaScale);
    CHECK(ours.kvScale == ref.kvScale);

    // ---- Pixel arrays (spot check or full check) ----
    // You can start with a few indices, then expand to full loops.

    int indices[] = {0, 1, 123, 511, 767};
    for (int idx : indices)
    {
        CAPTURE(idx);
        CHECK(ours.offset[idx] == ref.offset[idx]);
        CHECK(ours.alpha[idx]  == ref.alpha[idx]);
        CHECK(ours.kta[idx]    == ref.kta[idx]);
        CHECK(ours.kv[idx]     == ref.kv[idx]);
    }

    // Once this passes, you can uncomment the full loop:

    /*
    for (int i = 0; i < 768; ++i)
    {
        CHECK(ours.offset[i] == ref.offset[i]);
        CHECK(ours.alpha[i]  == ref.alpha[i]);
        CHECK(ours.kta[i]    == ref.kta[i]);
        CHECK(ours.kv[i]     == ref.kv[i]);
    }
    */

    // ---- Deviating pixels ----
    for (int i = 0; i < 5; ++i)
    {
        CHECK(ours.brokenPixels[i]  == ref.brokenPixels[i]);
        CHECK(ours.outlierPixels[i] == ref.outlierPixels[i]);
    }
}
