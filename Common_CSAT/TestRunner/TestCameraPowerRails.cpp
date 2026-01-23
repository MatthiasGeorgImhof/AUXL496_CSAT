#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "CameraPowerRails.hpp"
#include "mock_hal.h"
#include "GpioPin.hpp"

TEST_CASE("CameraPowerRails basic enable/disable behavior")
{
    // Mock GPIO ports
    constexpr uint32_t MOCK_PORT_A = 0x1243;
    constexpr uint32_t MOCK_PORT_B = 0x1244;
    constexpr uint32_t MOCK_PORT_C = 0x1245;

    GPIO_TypeDef mock_portA = {};
    GPIO_TypeDef mock_portB = {};
    GPIO_TypeDef mock_portC = {};

    reset_gpio_port_state(&mock_portA);
    reset_gpio_port_state(&mock_portB);
    reset_gpio_port_state(&mock_portC);

    // Define mock pins using GpioPin template
    using RailA = GpioPin<(MOCK_PORT_A), GPIO_PIN_0>;
    using RailB = GpioPin<(MOCK_PORT_B), GPIO_PIN_1>;
    using RailC = GpioPin<(MOCK_PORT_C), GPIO_PIN_2>;

    CameraPowerRails<RailA, RailB, RailC> rails;

    SUBCASE("All rails start low")
    {
        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portC, GPIO_PIN_2) == GPIO_PIN_RESET);
    }

    SUBCASE("Enable Rail A")
    {
        rails.enable(Rail::A);
        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_SET);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portC, GPIO_PIN_2) == GPIO_PIN_RESET);
    }

    SUBCASE("Enable Rail B")
    {
        rails.enable(Rail::B);
        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_SET);
        CHECK(get_gpio_pin_state(&mock_portC, GPIO_PIN_2) == GPIO_PIN_RESET);
    }

    SUBCASE("Enable Rail C")
    {
        rails.enable(Rail::C);
        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portC, GPIO_PIN_2) == GPIO_PIN_SET);
    }

    SUBCASE("Disable Rail A")
    {
        rails.enable(Rail::A);
        rails.disable(Rail::A);
        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_RESET);
    }

    SUBCASE("Disable Rail B")
    {
        rails.enable(Rail::B);
        rails.disable(Rail::B);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_RESET);
    }

    SUBCASE("Disable Rail C")
    {
        rails.enable(Rail::C);
        rails.disable(Rail::C);
        CHECK(get_gpio_pin_state(&mock_portC, GPIO_PIN_2) == GPIO_PIN_RESET);
    }

    SUBCASE("disableAll resets all rails")
    {
        rails.enable(Rail::A);
        rails.enable(Rail::B);
        rails.enable(Rail::C);

        rails.disableAll();

        CHECK(get_gpio_pin_state(&mock_portA, GPIO_PIN_0) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portB, GPIO_PIN_1) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_portC, GPIO_PIN_2) == GPIO_PIN_RESET);
    }
}
