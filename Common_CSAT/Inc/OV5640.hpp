#ifndef _OV5640_HPP_
#define _OV5640_HPP_

#include "Transport.hpp"
#include "OV5640_Registers.hpp"
#include "OV5640_Initialization.hpp"
#include "CameraDriver.hpp"
#include <array>
#include <cstdint>
#include <concepts>

template <typename Transport>
    requires RegisterModeTransport<Transport>
class OV5640
{
public:
    explicit OV5640(Transport &transport)
        : transport_(transport)
    {
    }

    //
    // ────────────────────────────────────────────────────────────────
    //  High‑level CameraDriver API
    // ────────────────────────────────────────────────────────────────
    //

    bool init()
    {
    	static constexpr auto& Initialization = cfg_init_;
    	for (auto& w : Initialization) {
    	    if (!writeRegister(w.addr, w.data)) return false;
    	}

        return true;
    }

    bool setResolution(uint16_t width, uint16_t height)
    {
        // TIMING_DVPHO (0x3808, 0x3809)
        if (!writeRegister(static_cast<uint16_t>(OV5640_Register::TIMING_DVPHO), static_cast<uint8_t>(width >> 8)))
            return false;
        if (!writeRegister(static_cast<uint16_t>(OV5640_Register::TIMING_DVPHO) + 1, static_cast<uint8_t>(width & 0xFF)))
            return false;

        // TIMING_DVPVO (0x380A, 0x380B)
        if (!writeRegister(static_cast<uint16_t>(OV5640_Register::TIMING_DVPVO), static_cast<uint8_t>(height >> 8)))
            return false;
        if (!writeRegister(static_cast<uint16_t>(OV5640_Register::TIMING_DVPVO) + 1, static_cast<uint8_t>(height & 0xFF)))
            return false;

        return true;
    }

    bool setFormat(PixelFormat fmt)
    {
        switch (fmt)
        {
        case PixelFormat::YUV422:
            return writeRegister(OV5640_Register::FORMAT_CONTROL00, 0x30);
        case PixelFormat::RGB565:
            return writeRegister(OV5640_Register::FORMAT_CONTROL00, 0x61);
        case PixelFormat::JPEG:
            return writeRegister(OV5640_Register::JPG_MODE_SELECT, 0x03);
        }
        return false;
    }

    bool setExposure(uint32_t exposure_us)
    {
        uint32_t exp = exposure_us;

        uint8_t hi = (exp >> 12) & 0x0F;
        uint8_t med = (exp >> 4) & 0xFF;
        uint8_t lo = (exp << 4) & 0xF0;

        return writeRegister(OV5640_Register::AEC_PK_EXPOSURE_HI, hi) &&
               writeRegister(OV5640_Register::AEC_PK_EXPOSURE_MED, med) &&
               writeRegister(OV5640_Register::AEC_PK_EXPOSURE_LO, lo);
    }

    bool setGain(float gain)
    {
        uint16_t g = static_cast<uint16_t>(gain * 16.0f);

        return writeRegister(static_cast<uint16_t>(OV5640_Register::AEC_PK_REAL_GAIN), g >> 8) &&
               writeRegister(static_cast<uint16_t>(OV5640_Register::AEC_PK_REAL_GAIN) + 1, g & 0xFF);
    }

    bool enableTestPattern(bool enable)
    {
        return writeRegister(OV5640_Register::PRE_ISP_TEST_SET1,
                             enable ? 0x80 : 0x00);
    }

    bool enableDvpMode()
    {
        // OV5640_EnableDVPMode equivalent
        // MIPI_CONTROL00 (0x4800) = 0x58 in ST driver
        return writeRegister(0x4800, 0x58);
    }

    bool setPolaritiesPclkHighHrefHighVsyncHigh()
    {
        // OV5640_SetPolarities(..., HIGH, HIGH, HIGH)
        // POLARITY_CTRL (0x4740) = 0x22 in ST driver
        return writeRegister(0x4740, 0x22);
    }

    //
    // ────────────────────────────────────────────────────────────────
    //  Enum overloads → uint16_t overloads
    // ────────────────────────────────────────────────────────────────
    //

    bool writeRegister(OV5640_Register reg, uint8_t value)
    {
        return writeRegister(static_cast<uint16_t>(reg), value);
    }

    bool writeRegister(OV5640_Register reg, const uint8_t *data, size_t size)
    {
        return writeRegister(static_cast<uint16_t>(reg), data, size);
    }

    uint8_t readRegister(OV5640_Register reg)
    {
        return readRegister(static_cast<uint16_t>(reg));
    }

    bool readRegister(OV5640_Register reg, uint8_t *buffer, size_t size)
    {
        return readRegister(static_cast<uint16_t>(reg), buffer, size);
    }

    //
    // ────────────────────────────────────────────────────────────────
    //  uint16_t overloads (real implementation)
    // ────────────────────────────────────────────────────────────────
    //

    bool writeRegister(uint16_t reg, uint8_t value)
    {
        return transport_.write_reg(reg, &value, 1);
    }

    bool writeRegister(uint16_t reg, const uint8_t *data, size_t size)
    {
        constexpr size_t MaxSize = 32;
        if (size > MaxSize || size % 2 != 0)
            return false;

        std::array<uint8_t, MaxSize> tx{};

        for (size_t i = 0; i < size; i += 2)
        {
            tx[i] = data[i + 1];
            tx[i + 1] = data[i];
        }

        return transport_.write_reg(reg, tx.data(), size);
    }

    uint8_t readRegister(uint16_t reg)
    {
        uint8_t rx{};
        transport_.read_reg(reg, &rx, 1);
        return rx;
    }

    bool readRegister(uint16_t reg, uint8_t *buffer, size_t size)
    {
        if (size % 2 != 0)
            return false;

        if (!transport_.read_reg(reg, buffer, size))
            return false;

        for (size_t i = 0; i < size; i += 2)
            std::swap(buffer[i], buffer[i + 1]);

        return true;
    }

private:
    Transport &transport_;
};

#endif // _OV5640_HPP_
