#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "GpioPin.hpp"
#include "mock_hal.h"


// Mock GPIO port
static constexpr uint32_t MOCK_PORT_BASE = static_cast<uint32_t>(0xDEADBEEF); // value irrelevant for mock 
GPIO_TypeDef gpio_typedef;
GPIO_TypeDef* MOCK_PORT = &gpio_typedef;

TEST_CASE("GpioPin set HIGH and LOW updates pin state") {
    GpioPin<MOCK_PORT_BASE, GPIO_PIN_1> pin;

    pin.set(PinState::HIGH);
    CHECK(get_gpio_pin_state(MOCK_PORT, GPIO_PIN_1) == GPIO_PIN_SET);

    pin.set(PinState::LOW);
    CHECK(get_gpio_pin_state(MOCK_PORT, GPIO_PIN_1) == GPIO_PIN_RESET);
}

TEST_CASE("GpioPin high() and low() update pin state") {
    GpioPin<MOCK_PORT_BASE, GPIO_PIN_2> pin;

    pin.high();
    CHECK(get_gpio_pin_state(MOCK_PORT, GPIO_PIN_2) == GPIO_PIN_SET);

    pin.low();
    CHECK(get_gpio_pin_state(MOCK_PORT, GPIO_PIN_2) == GPIO_PIN_RESET);
}

TEST_CASE("GpioPin read() reflects mock HAL state") {
    set_gpio_pin_state(MOCK_PORT, GPIO_PIN_3, GPIO_PIN_SET);
    GpioPin<MOCK_PORT_BASE, GPIO_PIN_3> pin;
    CHECK(pin.read() == true);

    set_gpio_pin_state(MOCK_PORT, GPIO_PIN_3, GPIO_PIN_RESET);
    CHECK(pin.read() == false);
}
