#ifndef __CAMERA_DRIVER_HPP_
#define __CAMERA_DRIVER_HPP_

#include <cstdint>
#include <concepts>

enum class PixelFormat : uint8_t {
    YUV422,
    RGB565,
    JPEG
};

template <typename T>
concept CameraDriver =
    requires(T cam,
             uint16_t w, uint16_t h,
             PixelFormat fmt,
             uint32_t exposure,
             float gain,
             bool enable)
{
    { cam.init() }                -> std::same_as<bool>;
    { cam.setResolution(w, h) }   -> std::same_as<bool>;
    { cam.setFormat(fmt) }        -> std::same_as<bool>;
    { cam.setExposure(exposure) } -> std::same_as<bool>;
    { cam.setGain(gain) }         -> std::same_as<bool>;
    { cam.enableTestPattern(enable) } -> std::same_as<bool>;
};

#endif // __CAMERA_DRIVER_HPP_
