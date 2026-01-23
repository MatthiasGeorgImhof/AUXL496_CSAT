#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "CameraPowerConverters.hpp"
#include "GpioPin.hpp"

#include "mock_hal.h"
#include <cstdint>

TEST_CASE("CameraPowerConverters enable/disable")
{
    constexpr uint32_t MOCK_PORT_A = 0x1243;
    constexpr uint32_t MOCK_PORT_B = 0x1244;

    GPIO_TypeDef mock_portA = {};
    GPIO_TypeDef mock_portB = {};

    reset_gpio_port_state(&mock_portA);
    reset_gpio_port_state(&mock_portB);

    using Rail1V8 = GpioPin<MOCK_PORT_A, GPIO_PIN_0>;
    using Rail2V8 = GpioPin<MOCK_PORT_B, GPIO_PIN_1>;

    CameraPowerConverters<Rail1V8, Rail2V8> converters;

    SUBCASE("Enable sets both rails high")
    {
        converters.enable();
        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_SET);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_SET);
    }

    SUBCASE("Disable sets both rails low")
    {
        converters.enable();
        converters.disable();
        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_RESET);
    }
}
