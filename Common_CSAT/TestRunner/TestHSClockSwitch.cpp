#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "HSClockSwitch.hpp"
#include "mock_hal/mock_hal_clock.h"
#include "mock_hal/mock_hal_time.h"
#include "mock_hal/mock_hal_gpio.h"

TEST_CASE("HSClockSwitch Tests") {
    SUBCASE("Switch to HSE Test") {
        HSClockSwitch Switch;

        // Reset the mock RCC state and time before the test
        reset_rcc();
        HAL_SetTick(0);

        // Initially, neither HSE nor HSI are ready
        set_hse_ready(false);
        set_hsi_ready(false);

        // Simulate HSE becoming ready *after* configuration
        set_hse_ready(true);

        HAL_StatusTypeDef status = Switch.switchToHSE();

        REQUIRE(status == HAL_OK);
        REQUIRE((RCC->CR & RCC_CR_HSEON) != 0);       // HSE should be enabled
        REQUIRE((RCC->CR & RCC_CR_HSERDY) != 0);      // HSE should be ready
        REQUIRE(HAL_RCC_GetSysClockSource() == RCC_SYSCLKSOURCE_HSE); // System clock should be HSE
    }

    SUBCASE("Switch to HSI Test") {
        HSClockSwitch Switch;

        // Reset the mock RCC state and time before the test
        reset_rcc();
        HAL_SetTick(0);

        // Initially, neither HSE nor HSI are ready
        set_hse_ready(false);
        set_hsi_ready(false);

        // Simulate HSI becoming ready *after* configuration
        set_hsi_ready(true);

        HAL_StatusTypeDef status = Switch.switchToHSI();

        REQUIRE(status == HAL_OK);
        REQUIRE((RCC->CR & RCC_CR_HSION) != 0);       // HSI should be enabled
        REQUIRE((RCC->CR & RCC_CR_HSIRDY) != 0);      // HSI should be ready
        REQUIRE(HAL_RCC_GetSysClockSource() == RCC_SYSCLKSOURCE_HSI); // System clock should be HSI
    }

    SUBCASE("HSE Timeout Test") {
        HSClockSwitch Switch;

        // Reset the mock RCC state and time before the test
        reset_rcc();
        HAL_SetTick(0);

        // HSE will *never* become ready
        set_hse_ready(false);

        HAL_StatusTypeDef status = Switch.switchToHSE();

        REQUIRE(status != HAL_OK); // Should time out waiting for HSE
        REQUIRE((RCC->CR & RCC_CR_HSEON) != 0); // HSE should be enabled (attempted)
        REQUIRE((RCC->CR & RCC_CR_HSERDY) == 0); // HSE should NOT be ready
    }

    SUBCASE("HSI Timeout Test") {
        HSClockSwitch Switch;

        // Reset the mock RCC state and time before the test
        reset_rcc();
        HAL_SetTick(0);

        // HSI will *never* become ready
        set_hsi_ready(false);

        HAL_StatusTypeDef status = Switch.switchToHSI();

        REQUIRE(status != HAL_OK); // Should time out waiting for HSI
        REQUIRE((RCC->CR & RCC_CR_HSION) != 0); // HSI should be enabled (attempted)
        REQUIRE((RCC->CR & RCC_CR_HSIRDY) == 0); // HSI should NOT be ready
    }

    SUBCASE("HSE Already Enabled") {
        HSClockSwitch Switch;

        // Reset the mock RCC state and time before the test
        reset_rcc();
        HAL_SetTick(0);

        // Simulate that HSE is already enabled.
        RCC->CR |= RCC_CR_HSEON;
        set_hse_ready(true);

        HAL_StatusTypeDef status = Switch.switchToHSE();

        REQUIRE(status == HAL_OK);
        REQUIRE((RCC->CR & RCC_CR_HSEON) != 0); // HSE should be enabled (still)
        REQUIRE((RCC->CR & RCC_CR_HSERDY) != 0); // HSE should be ready
        REQUIRE(HAL_RCC_GetSysClockSource() == RCC_SYSCLKSOURCE_HSE); // System clock should be HSE
    }

    SUBCASE("HAL_Delay is called during clock switch")
    {
        HSClockSwitch Switch;

        // Reset the mock RCC state and time before the test
        reset_rcc();
        HAL_SetTick(0);

        // Initially, neither HSE nor HSI are ready
        set_hse_ready(false);
        set_hsi_ready(true);

        //Switch to HSI
        HAL_StatusTypeDef status = Switch.switchToHSI();

        REQUIRE(status == HAL_OK);

        //Check we waited
        REQUIRE(HAL_GetTick() > 0);
    }
}

TEST_CASE("HSClockSwitchWithEnable Tests") {
    GPIO_TypeDef GPIOx;
    uint16_t pin = GPIO_PIN_1;
    HSClockSwitchWithEnable Switch(&GPIOx, pin);

    SUBCASE("Switch to HSE Test") {
        // Reset the mock RCC state, time and GPIO before the test
        reset_rcc();
        HAL_SetTick(0);
        reset_gpio_port_state(&GPIOx);

        // Initially, neither HSE nor HSI are ready
        set_hse_ready(false);
        set_hsi_ready(false);

        // Simulate HSE becoming ready *after* configuration
        set_hse_ready(true);

        HAL_StatusTypeDef status = Switch.switchToHSE();

        REQUIRE(status == HAL_OK);
        REQUIRE((RCC->CR & RCC_CR_HSEON) != 0);       // HSE should be enabled
        REQUIRE((RCC->CR & RCC_CR_HSERDY) != 0);      // HSE should be ready
        REQUIRE(HAL_RCC_GetSysClockSource() == RCC_SYSCLKSOURCE_HSE); // System clock should be HSE
        REQUIRE(get_gpio_pin_state(&GPIOx, pin) == GPIO_PIN_SET); // Pin should be set high
    }

    SUBCASE("Switch to HSI Test") {
        // Reset the mock RCC state, time and GPIO before the test
        reset_rcc();
        HAL_SetTick(0);
        reset_gpio_port_state(&GPIOx);
        set_gpio_pin_state(&GPIOx, pin, GPIO_PIN_SET);

        // Initially, neither HSE nor HSI are ready
        set_hse_ready(false);
        set_hsi_ready(false);

        // Simulate HSI becoming ready *after* configuration
        set_hsi_ready(true);

        HAL_StatusTypeDef status = Switch.switchToHSI();

        REQUIRE(status == HAL_OK);
        REQUIRE((RCC->CR & RCC_CR_HSION) != 0);       // HSI should be enabled
        REQUIRE((RCC->CR & RCC_CR_HSIRDY) != 0);      // HSI should be ready
        REQUIRE(HAL_RCC_GetSysClockSource() == RCC_SYSCLKSOURCE_HSI); // System clock should be HSI
        REQUIRE(get_gpio_pin_state(&GPIOx, pin) == GPIO_PIN_RESET); // Pin should be set low
    }
}