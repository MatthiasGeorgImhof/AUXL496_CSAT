
#ifndef INC_BMI270_H_
#define INC_BMI270_H_

#include <cstdint>
#include <optional>
#include "au.hpp"
#include "IMU.hpp"
#include "Transport.hpp"
#include "Logger.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

enum class BMI270_REGISTERS : uint8_t
{
	CHIP_ID = 0x00,
	ERR_REG = 0x02,
	STATUS = 0x03,

	AUX_DATA_X_LSB = 0x04,
	AUX_DATA_X_MSB = 0x05,
	AUX_DATA_Y_LSB = 0x06,
	AUX_DATA_Y_MSB = 0x07,
	AUX_DATA_Z_LSB = 0x08,
	AUX_DATA_Z_MSB = 0x09,
	AUX_DATA_R_LSB = 0x0A,
	AUX_DATA_R_MSB = 0x0B,

	ACC_DATA_X_LSB = 0x0C,
	ACC_DATA_X_MSB = 0x0D,
	ACC_DATA_Y_LSB = 0x0E,
	ACC_DATA_Y_MSB = 0x0F,
	ACC_DATA_Z_LSB = 0x10,
	ACC_DATA_Z_MSB = 0x11,

	GYR_DATA_X_LSB = 0x12,
	GYR_DATA_X_MSB = 0x13,
	GYR_DATA_Y_LSB = 0x14,
	GYR_DATA_Y_MSB = 0x15,
	GYR_DATA_Z_LSB = 0x16,
	GYR_DATA_Z_MSB = 0x17,

	SENSOR_TIME_0 = 0x18,
	SENSOR_TIME_1 = 0x19,
	SENSOR_TIME_2 = 0x1A,

	INTERNAL_STATUS = 0x21,

	TMP_DATA_LSB = 0x22,
	TMP_DATA_MSB = 0x23,

	FEAT_PAGE = 0x2F,
	FEATURES_START = 0x30,
	FEATURES_0 = 0x30,
	FEATURES_1 = 0x31,
	FEATURES_2 = 0x32,
	FEATURES_3 = 0x33,
	FEATURES_4 = 0x34,
	FEATURES_5 = 0x35,
	FEATURES_6 = 0x36,
	FEATURES_7 = 0x37,
	FEATURES_8 = 0x38,
	FEATURES_9 = 0x39,
	FEATURES_10 = 0x3A,
	FEATURES_11 = 0x3B,
	FEATURES_12 = 0x3C,
	FEATURES_13 = 0x3D,
	FEATURES_14 = 0x3E,
	FEATURES_15 = 0x3F,
	FEATURES_END = 0x3F,

	ACC_CONF = 0x40,
	ACC_RANGE = 0x41,
	GYR_CONF = 0x42,
	GYR_RANGE = 0x43,

	AUX_CONF = 0x44,
	SATURATION = 0x4A,
	AUX_DEV_ID = 0x4B,
	AUX_IF_CONF = 0x4C,
	AUX_RD_ADDR = 0x4D,
	AUX_WR_ADDR = 0x4E,
	AUX_WR_DATA = 0x4F,

	ERR_REG_MASK = 0x52,
	INIT_CTRL = 0x59,
	INIT_ADDR_0 = 0x5B,
	INIT_ADDR_1 = 0x5C,
	INIT_DATA = 0x5E,
	INTERNAL_ERROR = 0x5F,

	IF_CONF = 0x6B,
	PWR_CONF = 0x7C,
	PWR_CTRL = 0x7D,
	CMD = 0x7E,
};

static constexpr uint8_t bmi270_maximum_fifo_config_file[] = {
	0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
	0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
	0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
	0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
	0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
	0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
	0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
	0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
	0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
	0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
	0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
	0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
	0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
	0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
	0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
	0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
	0xf5, 0xeb, 0x2c, 0xe1, 0x6f};
static_assert(sizeof(bmi270_maximum_fifo_config_file) == 328);

struct BMI270_STATUS
{
	uint8_t status;
	uint8_t error;
	uint8_t internal_status;
};

template <typename Transport>
	requires RegisterModeTransport<Transport>
class BMI270
{
public:
	BMI270() = delete;
	explicit BMI270(const Transport &transport) : transport_(transport) {}
	bool initialize() const;
	bool configure() const;

	BMI270_STATUS readStatus() const;
	std::optional<ChipID> readChipID() const;

	std::optional<AccelerationInBodyFrame> readAccelerometer() const;
	std::optional<AngularVelocityInBodyFrame> readGyroscope() const;
	std::optional<Temperature> readThermometer() const;

	std::array<int16_t, 3> readRawAccelerometer() const;
	std::array<int16_t, 3> readRawGyroscope() const;
	uint16_t readRawThermometer() const;

public:
	const Transport &getTransport() const { return transport_; }
	bool writeRegister(BMI270_REGISTERS reg, uint8_t value) const;
	bool writeRegisters(BMI270_REGISTERS reg, const uint8_t *tx_buf, uint16_t tx_len) const;
	bool readRegister(BMI270_REGISTERS reg, uint8_t &value) const;
	bool readRegisters(BMI270_REGISTERS reg, uint8_t *rx_buf, uint16_t rx_len) const;
	bool writeRegisterWithCheck(BMI270_REGISTERS reg, uint8_t value) const;
	bool writeRegisterWithRepeat(BMI270_REGISTERS reg, uint8_t value) const;

	int16_t toInt16(const uint8_t lsb, const uint8_t msb) const
	{
		return (msb << 8) | lsb;
	}

	uint16_t toUInt16(const uint8_t lsb, const uint8_t msb) const
	{
		return (msb << 8) | lsb;
	}

	au::QuantityF<au::MetersPerSecondSquaredInBodyFrame> convertAcc(const uint8_t lsb, const uint8_t msb) const
	{
		constexpr float LSB_PER_G = 16384.0f;
		constexpr float G = 9.80665f;
		return au::make_quantity<au::MetersPerSecondSquaredInBodyFrame>(toInt16(lsb, msb) * G / LSB_PER_G);
	}

	au::QuantityF<au::DegreesPerSecondInBodyFrame> convertGyr(const uint8_t lsb, const uint8_t msb) const
	{
		constexpr float LSB_PER_DPS = 16.4f; // ±2000°/s range
		return au::make_quantity<au::DegreesPerSecondInBodyFrame>(toInt16(lsb, msb) / LSB_PER_DPS);
	}

	au::QuantityF<au::Celsius> convertTmp(const uint8_t lsb, const uint8_t msb) const
	{
		constexpr float LSB_PER_TMP = 1.0 / 512.0f; // 0.001953125 °C/LSB
		constexpr float TMP_SHIFT = 23.0f;
		return au::make_quantity<au::Celsius>(TMP_SHIFT + toInt16(lsb, msb) * LSB_PER_TMP);
	}

protected:
	const Transport &transport_;

	static constexpr uint8_t BMI270_READ_BIT = 0x80;

	static constexpr uint8_t BMI270_CMD_SOFT_RESET = 0xB6;
	static constexpr uint8_t BMI270_CMD_FIFO_FLUSH = 0xB0;
	static constexpr uint8_t BMI270_CMD_ACCEL_ENABLE = 0x04;
	static constexpr uint8_t BMI270_CMD_GYRO_ENABLE = 0x08;
	static constexpr uint8_t BMI270_CMD_AUX_ENABLE = 0x02;

	static constexpr uint8_t BMI270_AUX_EN = 0x01;
	static constexpr uint8_t BMI270_GYR_EN = 0x02;
	static constexpr uint8_t BMI270_ACC_EN = 0x04;
	static constexpr uint8_t BMI270_TMP_EN = 0x08;

	static constexpr uint8_t BMI270_CHIP_ID = 0x24;
	static constexpr uint8_t BMI270_FIFO_ACC_EN = 0x40;
	static constexpr uint8_t BMI270_FIFO_GYR_EN = 0x80;
};

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::writeRegister(BMI270_REGISTERS reg, uint8_t value) const
{
	return transport_.write_reg(static_cast<uint16_t>(reg), &value, 1);
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::writeRegisters(BMI270_REGISTERS reg, const uint8_t *tx_buf, uint16_t tx_len) const
{
	return transport_.write_reg(static_cast<uint16_t>(reg), tx_buf, tx_len);
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::readRegisters(BMI270_REGISTERS reg, uint8_t *rx_buf, uint16_t rx_len) const
{
	return transport_.read_reg(static_cast<uint16_t>(reg) | BMI270_READ_BIT, rx_buf, rx_len);
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::readRegister(BMI270_REGISTERS reg, uint8_t &value) const
{

	uint8_t rx_buf[2]{};
	bool ok = readRegisters(reg, rx_buf, sizeof(rx_buf));
	value = rx_buf[1];
	return ok;
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::writeRegisterWithCheck(BMI270_REGISTERS reg, uint8_t value) const
{
	if (!writeRegister(reg, value))
	{
		return false;
	}
	uint8_t data;
	if (!readRegister(reg, data))
	{
		return false;
	}
	return (data == value);
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::writeRegisterWithRepeat(BMI270_REGISTERS reg, uint8_t value) const
{
	constexpr uint8_t N_REPEAT = 8;
	for (uint8_t i = 0; i < N_REPEAT; ++i)
	{
		if (writeRegisterWithCheck(reg, value))
		{
			return true;
		}
	}
	return false;
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::initialize() const
{
	bool init = false;
	bool read_chip_id = false;

	constexpr uint8_t BMI270_HARDWARE_INIT_MAX_TRIES = 16;
	for (uint8_t i = 0; i < BMI270_HARDWARE_INIT_MAX_TRIES; ++i)
	{
		uint8_t chip_id = 0;
		/* If CSB sees a rising edge after power-up, the device interface switches to SPI
		 * after 200 μs until a reset or the next power-up occurs therefore it is recommended
		 * to perform a SPI single read of register CHIP_ID (the obtained value will be invalid)
		 * before the actual communication start, in order to use the SPI interface */
		readRegister(BMI270_REGISTERS::CHIP_ID, chip_id);
		HAL_Delay(1);

		writeRegisterWithCheck(BMI270_REGISTERS::CMD, BMI270_CMD_SOFT_RESET);
		HAL_Delay(2); // power on and soft reset time is 2ms

		// switch to SPI mode again
		readRegister(BMI270_REGISTERS::CHIP_ID, chip_id);
		HAL_Delay(1);

		readRegister(BMI270_REGISTERS::CHIP_ID, chip_id);
		if (chip_id != BMI270_CHIP_ID)
		{
			continue;
		}

		// successfully identified the chip, proceed with initialisation
		read_chip_id = true;

		// disable power save
		writeRegisterWithCheck(BMI270_REGISTERS::PWR_CONF, 0x00);
		HAL_Delay(1);

		// upload config
		writeRegisterWithCheck(BMI270_REGISTERS::INIT_CTRL, 0x00);

		// Transfer the config file, data packet includes INIT_DATA
		// transport_.write(bmi270_maximum_fifo_config_file, sizeof(bmi270_maximum_fifo_config_file));
		writeRegisters(BMI270_REGISTERS::INIT_DATA, bmi270_maximum_fifo_config_file, sizeof(bmi270_maximum_fifo_config_file));

		// config is done
		writeRegisterWithCheck(BMI270_REGISTERS::INIT_CTRL, 1);

		// check the results
		HAL_Delay(20);

		uint8_t status = 0;
		readRegister(BMI270_REGISTERS::INTERNAL_STATUS, status);

		if ((status & 1) == 1)
		{
			init = true;
			log(LOG_LEVEL_DEBUG, "BMI270 initialized after %d retries\r\n", i + 1);
			break;
		}
	}

	if (read_chip_id && !init)
	{
		log(LOG_LEVEL_ERROR, "BMI270: failed to init\n");
	}

	return init;
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
bool BMI270<Transport>::configure() const
{
	if (!writeRegisterWithCheck(BMI270_REGISTERS::ACC_CONF, 0x08))
	{
		return false;
	}
	if (!writeRegisterWithCheck(BMI270_REGISTERS::ACC_RANGE, 0x00))
	{
		return false;
	}

	if (!writeRegisterWithCheck(BMI270_REGISTERS::GYR_CONF, 0x08))
	{
		return false;
	}
	if (!writeRegisterWithCheck(BMI270_REGISTERS::GYR_RANGE, 0x00))
	{
		return false;
	}

	if (!writeRegisterWithCheck(BMI270_REGISTERS::PWR_CTRL, BMI270_GYR_EN | BMI270_ACC_EN | BMI270_TMP_EN))
	{
		return false;
	}
	log(LOG_LEVEL_DEBUG, "BMI270 configured for ACC, GYR, TMP\r\n");

	return true;
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
BMI270_STATUS BMI270<Transport>::readStatus() const
{
	BMI270_STATUS s{};
	readRegister(BMI270_REGISTERS::STATUS, s.status);
	readRegister(BMI270_REGISTERS::ERR_REG, s.error);
	readRegister(BMI270_REGISTERS::INTERNAL_STATUS, s.internal_status);

	return s;
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<ChipID> BMI270<Transport>::readChipID() const
{
	uint8_t value{};
	if (!readRegister(BMI270_REGISTERS::CHIP_ID, value))
	{
		return std::nullopt;
	}
	return value;
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<AccelerationInBodyFrame> BMI270<Transport>::readAccelerometer() const
{
	uint8_t rx[7]{}; // rx[0] is a dummy byte to give the BMI time to respond

	if (!readRegisters(BMI270_REGISTERS::ACC_DATA_X_LSB, rx, sizeof(rx)))
	{
		return std::nullopt;
	}

	// Accelerometer is front (+0.91), left/west (+9.81). up (+9.81)
	// wanted NED (front east down) to have positive +9.81 when oriented
	// Orientation	Axis Rotation Positive Direction
	// front		X	-9.81 when x back points
	// right/east	Y	+9.81 when y points up
	// down			Z	+9.81

	return AccelerationInBodyFrame{
		-convertAcc(rx[1], rx[2]),
		convertAcc(rx[3], rx[4]),
		convertAcc(rx[5], rx[6])};
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<AngularVelocityInBodyFrame> BMI270<Transport>::readGyroscope() const
{
	uint8_t rx[7]{}; // rx[0] is a dummy byte to give the BMI time to respond

	if (!readRegisters(BMI270_REGISTERS::GYR_DATA_X_LSB, rx, sizeof(rx)))
	{
		return std::nullopt;
	}

	// Gyroscope is front, left/west. up
	// wanted
	// Orientation	Axis Rotation Positive Direction
	// front		X	Roll	Right wing down
	// right/east	Y	Pitch	Nose up
	// down		Z	Yaw		Nose right

	return AngularVelocityInBodyFrame{
		convertGyr(rx[1], rx[2]),
		-convertGyr(rx[3], rx[4]),
		-convertGyr(rx[5], rx[6])};
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::optional<Temperature> BMI270<Transport>::readThermometer() const
{
	uint8_t rx[3]{}; // rx[0] is a dummy byte to give the BMI time to respond

	if (!readRegisters(BMI270_REGISTERS::TMP_DATA_LSB, rx, sizeof(rx)))
	{
		return std::nullopt;
	}

	return Temperature{convertTmp(rx[1], rx[2])};
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::array<int16_t, 3> BMI270<Transport>::readRawAccelerometer() const
{
	uint8_t rx[7]{}; // rx[0] is a dummy byte to give the BMI time to respond

	if (!readRegisters(BMI270_REGISTERS::ACC_DATA_X_LSB, rx, sizeof(rx)))
	{
		memset(rx, 0xFF, sizeof(rx));
	}

	return {toInt16(rx[1], rx[2]),
			toInt16(rx[3], rx[4]),
			toInt16(rx[5], rx[6])};
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
std::array<int16_t, 3> BMI270<Transport>::readRawGyroscope() const
{
	uint8_t rx[7]{}; // rx[0] is a dummy byte to give the BMI time to respond

	if (!readRegisters(BMI270_REGISTERS::GYR_DATA_X_LSB, rx, sizeof(rx)))
	{
		memset(rx, 0, sizeof(rx));
	}

	return {toInt16(rx[1], rx[2]),
			toInt16(rx[3], rx[4]),
			toInt16(rx[5], rx[6])};
}

template <typename Transport>
	requires RegisterModeTransport<Transport>
uint16_t BMI270<Transport>::readRawThermometer() const
{
	uint8_t rx[3]{}; // rx[0] is a dummy byte to give the BMI time to respond

	if (!readRegisters(BMI270_REGISTERS::TMP_DATA_LSB, rx, sizeof(rx)))
	{
		memset(rx, 0, sizeof(rx));
	}

	return toUInt16(rx[1], rx[2]);
}

#endif /* INC_BMI270_H_ */
