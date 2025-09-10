
#ifndef INC_BMI270_H_
#define INC_BMI270_H_

#include <cstdint>
#include <optional>
#include "au.hpp"
#include "IMU.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

struct SPI_PortDef
{
	UART_HandleTypeDef *hspi;
	GPIO_TypeDef* GPIO;
	uint16_t pin;

	void select() const
	{
		HAL_GPIO_WritePin(GPIO, pin, GPIO_PIN_RESET);
	}

	void deselect() const
	{
		HAL_GPIO_WritePin(GPIO, pin, GPIO_PIN_SET);
	}
};

class BMI270
{
public:
	BMI270() = delete;
	BMI270(SPI_PortDef spi);

	std::optional<ChipID> readChipID() const;
	std::optional<AccelerationInBodyFrame> readAccelerometer() const;
	std::optional<AngularVelocityInBodyFrame> readGyroscope() const;
	std::optional<MagneticFieldInBodyFrame> readMagnetometer() const;
	std::optional<Temperature> readThermometer() const;
private:
	SPI_PortDef spi_;
};

static_assert(ProvidesChipID<BMI270>);
static_assert(HasBodyAccelerometer<BMI270>);
static_assert(HasBodyGyroscope<BMI270>);
static_assert(HasBodyMagnetometer<BMI270>);
static_assert(HasThermometer<BMI270>);

#endif /* INC_BMI270_H_ */
