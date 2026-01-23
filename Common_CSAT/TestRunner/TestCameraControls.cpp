

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "CameraControls.hpp"
#include "mock_hal.h"
#include "GpioPin.hpp"

TEST_CASE("CameraControls basic GPIO behavior")
{
    // Mock GPIO ports
    constexpr uint32_t MOCK_PORT_CLK = 0x1243;
    constexpr uint32_t MOCK_PORT_RST = 0x1244;
    constexpr uint32_t MOCK_PORT_PWDN = 0x1245;

    GPIO_TypeDef mock_port_clk = {};
    GPIO_TypeDef mock_port_rst = {};
    GPIO_TypeDef mock_port_pwdn = {};

    reset_gpio_port_state(&mock_port_clk);
    reset_gpio_port_state(&mock_port_rst);
    reset_gpio_port_state(&mock_port_pwdn);

    // Define mock pins using GpioPin template
    using ClkPin = GpioPin<MOCK_PORT_CLK, GPIO_PIN_0>;
    using ResetPin = GpioPin<MOCK_PORT_RST, GPIO_PIN_1>;
    using PwdnPin = GpioPin<MOCK_PORT_PWDN, GPIO_PIN_2>;

    CameraControls<ClkPin, ResetPin, PwdnPin> ctrl;

    SUBCASE("Clock control")
    {
        ctrl.clock_on();
        CHECK(get_gpio_pin_state(&mock_port_clk, GPIO_PIN_0) == GPIO_PIN_SET);

        ctrl.clock_off();
        CHECK(get_gpio_pin_state(&mock_port_clk, GPIO_PIN_0) == GPIO_PIN_RESET);
    }

    SUBCASE("Reset control")
    {
        ctrl.reset_assert();
        CHECK(get_gpio_pin_state(&mock_port_rst, GPIO_PIN_1) == GPIO_PIN_RESET);

        ctrl.reset_release();
        CHECK(get_gpio_pin_state(&mock_port_rst, GPIO_PIN_1) == GPIO_PIN_SET);
    }

    SUBCASE("Power-down control")
    {
        ctrl.powerdown_on();
        CHECK(get_gpio_pin_state(&mock_port_pwdn, GPIO_PIN_2) == GPIO_PIN_SET);

        ctrl.powerdown_off();
        CHECK(get_gpio_pin_state(&mock_port_pwdn, GPIO_PIN_2) == GPIO_PIN_RESET);
    }

    SUBCASE("bringup() sequence")
    {
        ctrl.bringup();

        // bringup() does:
        //   clock_on()
        //   powerdown_off()
        //   reset_release()

        CHECK(get_gpio_pin_state(&mock_port_clk, GPIO_PIN_0) == GPIO_PIN_SET);
        CHECK(get_gpio_pin_state(&mock_port_pwdn, GPIO_PIN_2) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_port_rst, GPIO_PIN_1) == GPIO_PIN_SET);
    }

    SUBCASE("Signals do not interfere with each other")
    {
        ctrl.clock_on();
        CHECK(get_gpio_pin_state(&mock_port_rst, GPIO_PIN_1) == GPIO_PIN_RESET);
        CHECK(get_gpio_pin_state(&mock_port_pwdn, GPIO_PIN_2) == GPIO_PIN_RESET);

        ctrl.reset_release();
        CHECK(get_gpio_pin_state(&mock_port_clk, GPIO_PIN_0) == GPIO_PIN_SET);
        CHECK(get_gpio_pin_state(&mock_port_pwdn, GPIO_PIN_2) == GPIO_PIN_RESET);

        ctrl.powerdown_on();
        CHECK(get_gpio_pin_state(&mock_port_rst, GPIO_PIN_1) == GPIO_PIN_SET);
    }
}