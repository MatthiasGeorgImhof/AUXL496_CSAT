
#ifndef _MMC5983_H_
#define _MMC5983_H_

#include <cstdint>
#include <optional>
#include "au.hpp"
#include "IMU.hpp"
#include "Transport.hpp"
#include "Logger.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

enum class MMC5983_REGISTERS : uint8_t
{
    MMC5983_XOUT0 = 0x00,
    MMC5983_XOUT1 = 0x01,
    MMC5983_YOUT0 = 0x02,
    MMC5983_YOUT1 = 0x03,
    MMC5983_ZOUT0 = 0x04,
    MMC5983_ZOUT1 = 0x05,
    MMC5983_XYZOUT2 = 0x06,
    MMC5983_TOUT = 0x07,
    MMC5983_STATUS = 0x08,
    MMC5983_CONTROL0 = 0x09,
    MMC5983_CONTROL1 = 0x0a,
    MMC5983_CONTROL2 = 0x0b,
    MMC5983_CONTROL3 = 0x0c,
    MMC5983_PRODUCTID = 0x2F,
};

#include <array>

struct MagnetometerCalibration
{
    std::array<float, 3> bias;
    std::array<std::array<float, 3>, 3> scale;
};

constexpr MagnetometerCalibration DefaultMMC5983Calibration = {
    {0.0f, 0.0f, 0.0f},
    {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}}};

class MMC5983Core
{
public:
    static constexpr int32_t NULL_VALUE = 131072;

    static int32_t toInt32(uint8_t lsb, uint8_t isb, uint8_t msb)
    {
        return ((msb << 10) | (isb << 2) | lsb) - NULL_VALUE;
    }

    static MagneticFieldInBodyFrame convertMag(const std::array<float,3> mag)
    {
        constexpr float COUNT_PER_GAUSS = 16384.f;
        constexpr float GAUSS_PER_TESLA = 10000.f;
        constexpr float TESLA_PER_COUNT = 1.f / (COUNT_PER_GAUSS * GAUSS_PER_TESLA);

        // MMC5983 sensor frame is left handed: front x, left y, z down
        // want: Magnetometer NED (front, right, down)
        // Katy TX
        // Device Heading	Sensor X (forward)	Sensor Y (left)	Sensor Z (down)
        // North			+22.9 µT		–2.2 µT				+41 µT
        // East				-2.2 µT			-22.9 µT			+41 µT
        // South			–22.9 µT		+2.2 µT				+41 µT
        // West				+2.2 µT			+22.9 µT			+41 µT

        return { au::make_quantity<au::TeslaInBodyFrame>(mag[0] * TESLA_PER_COUNT),
		-au::make_quantity<au::TeslaInBodyFrame>(mag[1] * TESLA_PER_COUNT),
		au::make_quantity<au::TeslaInBodyFrame>(mag[2] * TESLA_PER_COUNT)
        };
    }

    static au::QuantityF<au::Celsius> convertTmp(const uint8_t value)
    {
        constexpr float LSB_PER_TMP = 0.8f;
        constexpr float TMP_SHIFT = -75.0f;
        return au::make_quantity<au::Celsius>(TMP_SHIFT + value * LSB_PER_TMP);
    }

    static std::array<int32_t, 3> parseMagnetometerData(const uint8_t *buf)
    {
        return {
            toInt32((buf[6] >> 6) & 0b11, buf[1], buf[0]),
            toInt32((buf[6] >> 4) & 0b11, buf[3], buf[2]),
            toInt32((buf[6] >> 2) & 0b11, buf[5], buf[4])};
    }

    static std::array<float, 3> calibrateMagnetometer(const uint8_t *rx_buf, const MagnetometerCalibration &calibration)
    {
        auto parsed = MMC5983Core::parseMagnetometerData(rx_buf);
        const std::array<float, 3> uncalibrated {
        		static_cast<float>(parsed[0]) - calibration.bias[0],
        		static_cast<float>(parsed[1]) - calibration.bias[1],
        		static_cast<float>(parsed[2]) - calibration.bias[2]
        };

        return {
            uncalibrated[0] * calibration.scale[0][0] + uncalibrated[1] * calibration.scale[0][1] + uncalibrated[2] * calibration.scale[0][2],
            uncalibrated[0] * calibration.scale[1][0] + uncalibrated[1] * calibration.scale[1][1] + uncalibrated[2] * calibration.scale[1][2],
            uncalibrated[0] * calibration.scale[2][0] + uncalibrated[1] * calibration.scale[2][1] + uncalibrated[2] * calibration.scale[2][2]};
    }
};

template <typename Transport>
    requires RegisterModeTransport<Transport>
class MMC5983
{
public:
    MMC5983() = delete;
    explicit MMC5983(const Transport &transport, const MagnetometerCalibration &calibration = DefaultMMC5983Calibration) : transport_(transport), calibration_(calibration) {}
    bool initialize() const;
    bool configureContinuousMode(uint8_t freq_code, uint8_t set_interval_code, bool auto_set) const;

    uint8_t readStatus() const;
    std::optional<ChipID> readChipID() const;

    std::optional<MagneticFieldInBodyFrame> readMagnetometer() const;
    std::optional<Temperature> readThermometer() const;

    std::array<int32_t, 3> readRawMagnetometer() const;
    uint8_t readRawThermometer() const;

    bool performSet() const { return writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL0, 0x08); }
    const MagnetometerCalibration &calibration() const { return calibration_; }

public:
    bool writeRegister(MMC5983_REGISTERS reg, uint8_t value) const;
    bool readRegister(MMC5983_REGISTERS reg, uint8_t &value) const;
    bool readRegisters(MMC5983_REGISTERS reg, uint8_t *rx_buf, uint16_t rx_len) const;

private:
    const Transport &transport_;
    const MagnetometerCalibration &calibration_;
    static constexpr uint8_t MMC5983_READ_BIT = 0x80;
};

template <typename Transport>
    requires RegisterModeTransport<Transport>
bool MMC5983<Transport>::writeRegister(MMC5983_REGISTERS reg, uint8_t value) const
{
    return transport_.write_reg(static_cast<uint8_t>(reg), &value, 1);
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
bool MMC5983<Transport>::readRegisters(MMC5983_REGISTERS reg, uint8_t *rx_buf, uint16_t rx_len) const
{
    return transport_.read_reg(static_cast<uint8_t>(reg), rx_buf, rx_len);
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
bool MMC5983<Transport>::readRegister(MMC5983_REGISTERS reg, uint8_t &value) const
{
    return transport_.read_reg(static_cast<uint8_t>(reg), &value, 1);
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
bool MMC5983<Transport>::configureContinuousMode(uint8_t freq_code, uint8_t set_interval_code, bool auto_set) const
{
    uint8_t ctrl1 = auto_set ? 0x80 : 0x00;
    uint8_t ctrl2 = (auto_set ? 0x80 : 0x00) | (set_interval_code << 4) | (1 << 3) | freq_code;
    return writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL1, ctrl1) &&
           writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL2, ctrl2);
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
bool MMC5983<Transport>::initialize() const
{
    return configureContinuousMode(/*freq_code=*/0b101, /*set_interval_code=*/0b011, /*auto_set=*/true);
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
std::optional<ChipID> MMC5983<Transport>::readChipID() const
{
    uint8_t value{};
    if (!readRegisters(MMC5983_REGISTERS::MMC5983_PRODUCTID, &value, 1))
    {
        return std::nullopt;
    }
    return value;
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
uint8_t MMC5983<Transport>::readStatus() const
{
    uint8_t value = 0;
    readRegister(MMC5983_REGISTERS::MMC5983_STATUS, value);
    return value;
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
std::optional<MagneticFieldInBodyFrame> MMC5983<Transport>::readMagnetometer() const
{
    uint8_t rx_buf[8]{};

    if (!readRegisters(MMC5983_REGISTERS::MMC5983_XOUT0, rx_buf, sizeof(rx_buf)))
    {
        return std::nullopt;
    }

    return MMC5983Core::convertMag(MMC5983Core::calibrateMagnetometer(rx_buf, calibration_));
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
std::optional<Temperature> MMC5983<Transport>::readThermometer() const
{
    uint8_t rx_buf[1]{};

    if (!readRegisters(MMC5983_REGISTERS::MMC5983_TOUT, rx_buf, sizeof(rx_buf)))
    {
        return std::nullopt;
    }

    return Temperature{MMC5983Core::convertTmp(rx_buf[0])};
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
std::array<int32_t, 3> MMC5983<Transport>::readRawMagnetometer() const
{
    uint8_t rx_buf[8]{};

    if (!readRegisters(MMC5983_REGISTERS::MMC5983_XOUT0, rx_buf, sizeof(rx_buf)))
    {
        memset(rx_buf, 0, sizeof(rx_buf));
    }

    return MMC5983Core::parseMagnetometerData(rx_buf);
}

template <typename Transport>
    requires RegisterModeTransport<Transport>
uint8_t MMC5983<Transport>::readRawThermometer() const
{
    writeRegister(MMC5983_REGISTERS::MMC5983_CONTROL0, 0b10);
    HAL_Delay(5);

    uint8_t rx_buf[1]{};

    if (!readRegisters(MMC5983_REGISTERS::MMC5983_TOUT, rx_buf, sizeof(rx_buf)))
    {
        rx_buf[0] = 255;
    }

    return rx_buf[0];
}

#endif /* _MMC5983_H_ */
