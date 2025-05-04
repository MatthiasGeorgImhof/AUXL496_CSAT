#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "CameraSwitch.hpp"
#include "mock_hal.h"

TEST_CASE("CameraSwitch setState Test with Mock HAL") {
    GPIO_TypeDef *gpio_port_a = nullptr;
    GPIO_TypeDef *gpio_port_b = nullptr;

    CameraSwitch cameraSwitch(gpio_port_a, GPIO_PIN_5, gpio_port_b, GPIO_PIN_10);

    // Reset GPIO Pin States before each test
    set_gpio_pin_state(gpio_port_a, GPIO_PIN_5, GPIO_PIN_RESET);
    set_gpio_pin_state(gpio_port_b, GPIO_PIN_10, GPIO_PIN_RESET);

    SUBCASE("Set to OFF") {
        cameraSwitch.setState(CameraSwitchState::OFF);
        REQUIRE(get_gpio_pin_state(gpio_port_a, GPIO_PIN_5) == GPIO_PIN_RESET);
        REQUIRE(get_gpio_pin_state(gpio_port_b, GPIO_PIN_10) == GPIO_PIN_RESET);
    }

    SUBCASE("Set to A") {
        cameraSwitch.setState(CameraSwitchState::A);
        REQUIRE(get_gpio_pin_state(gpio_port_a, GPIO_PIN_5) == GPIO_PIN_RESET);
        REQUIRE(get_gpio_pin_state(gpio_port_b, GPIO_PIN_10) == GPIO_PIN_SET);
    }

    SUBCASE("Set to B") {
        cameraSwitch.setState(CameraSwitchState::B);
        REQUIRE(get_gpio_pin_state(gpio_port_a, GPIO_PIN_5) == GPIO_PIN_SET);
        REQUIRE(get_gpio_pin_state(gpio_port_b, GPIO_PIN_10) == GPIO_PIN_SET);
    }

    SUBCASE("Set to C") {
        cameraSwitch.setState(CameraSwitchState::C);
        REQUIRE(get_gpio_pin_state(gpio_port_a, GPIO_PIN_5) == GPIO_PIN_SET);
        REQUIRE(get_gpio_pin_state(gpio_port_b, GPIO_PIN_10) == GPIO_PIN_RESET);
    }
}