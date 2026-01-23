// MLX90640ImageProcessor.hpp

#pragma once
#include <cstdint>
#include <array>

class MLX90640ImageProcessor
{
public:
    static constexpr int WIDTH  = 32;
    static constexpr int HEIGHT = 24;
    static constexpr int PIXELS = WIDTH * HEIGHT;

    using RawImage  = std::array<int16_t, PIXELS>;
    using TempImage = std::array<float,   PIXELS>;

    MLX90640ImageProcessor() = default;

    bool demultiplexFrame(const uint16_t* frame, RawImage& outRaw) const;

    bool computeTemperatures(const RawImage& raw,
                             TempImage& outTemps,
                             float Ta = 25.0f) const;
};
