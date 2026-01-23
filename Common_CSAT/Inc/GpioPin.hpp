// GPIOPin.hpp

#pragma once

#ifdef __linux__
#include "mock_hal.h"
#endif
#ifdef __arm__
#include "stm32l4xx_hal.h"
#endif

enum class PinState : uint8_t {
    LOW = 0,
    HIGH = 1
};

template<uint32_t PortAddr, uint16_t Pin>
struct GpioPin {
//	 static constexpr GPIO_TypeDef* port = Port;
    static GPIO_TypeDef* port() { return reinterpret_cast<GPIO_TypeDef*>(PortAddr); }
	static constexpr uint16_t pin = Pin;

	void set(PinState state) const {
        HAL_GPIO_WritePin(port(), Pin, state == PinState::HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void high() const {
        HAL_GPIO_WritePin(port(), Pin, GPIO_PIN_SET);
    }

    void low() const {
        HAL_GPIO_WritePin(port(), Pin, GPIO_PIN_RESET);
    }

    bool read() const {
        return HAL_GPIO_ReadPin(port(), Pin) == GPIO_PIN_SET;
    }
};

// GPIOPin.hpp
