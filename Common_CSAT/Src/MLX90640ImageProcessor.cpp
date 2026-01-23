#include "MLX90640ImageProcessor.hpp"
#include "MLX90640Calibration.hpp"
#include <cmath>

bool MLX90640ImageProcessor::demultiplexFrame(const uint16_t* frame,
                                              RawImage& outRaw) const
{
    const uint16_t* sub0 = frame;
    const uint16_t* sub1 = frame + 834;

    for (std::size_t row = 0; row < HEIGHT; ++row)
    {
        for (std::size_t col = 0; col < WIDTH; ++col)
        {
            std::size_t idx = row * WIDTH + col;

            // Checkerboard pattern:
            // subpage 0: (row + col) even
            // subpage 1: (row + col) odd
            bool useSub1 = ((row + col) & 1u) != 0;

            outRaw[idx] = useSub1
                ? int16_t(sub1[idx])
                : int16_t(sub0[idx]);
        }
    }

    return true;
}

bool MLX90640ImageProcessor::computeTemperatures(const RawImage& raw,
                                                 TempImage& outTemps,
                                                 float Ta) const
{
    const auto& P = MLX90640_CAL;

    // Compute Vdd exactly like Melexis
    float Vdd = (Ta - 25.0f) * P.KvPTAT + P.vdd25;  // simplified model

    for (size_t i = 0; i < PIXELS; ++i)
    {
        float rawPix = float(raw[i]);

        // Offset compensation
        float offsetComp = rawPix - float(P.offset[i]);

        // Gain compensation
        float gainComp = offsetComp / float(P.gainEE);

        // Kta/Kv compensation
        float ktaComp = gainComp / (1.0f + float(P.kta[i]) * (Ta - 25.0f));
        float kvComp  = ktaComp / (1.0f + float(P.kv[i])  * (Vdd - 3.3f));

        // Alpha compensation
        float alphaComp = float(P.alpha[i]) * (1.0f + P.KsTa * (Ta - 25.0f));

        // Sx term (simplified Melexis model)
        float Sx = P.ksTo[1] * std::sqrt(std::sqrt(alphaComp * kvComp));

        // Temperature calculation (simplified)
        float To = std::sqrt(std::sqrt(
                     kvComp / (alphaComp * (1.0f - P.tgc) + Sx)
                 ));

        outTemps[i] = To;
    }

    return true;
}
