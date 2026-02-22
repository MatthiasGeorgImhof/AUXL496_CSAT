#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal/mock_hal_clock.h"
#include "mock_hal/mock_hal_time.h" // For HAL_Delay (if needed)

#include <iostream> //For printing values to the console (debugging)

//Test that the test harness is working.
TEST_CASE("Testing doctest setup") {
    CHECK(1 == 1);
}

TEST_CASE("HAL_RCC_OscConfig - HSE Enable Success") {
    // Arrange: Initialize the mock environment
    set_hse_ready(true);
    RCC_OscInitTypeDef osc_init {};
        osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        osc_init.HSEState = RCC_HSE_ON;
        osc_init.HSIState = RCC_HSI_OFF;
        osc_init.HSICalibrationValue = 0;
        osc_init.PLL.PLLState = RCC_PLL_NONE;

    // Act: Call the function under test
    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&osc_init);

    // Assert: Verify the results
    CHECK(status == HAL_OK);
    CHECK((RCC->CR & RCC_CR_HSEON) == RCC_CR_HSEON); // Check that HSEON bit is set
    CHECK((RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY); // Check that HSERDY bit is set
}

TEST_CASE("HAL_RCC_OscConfig - HSE Enable Failure") {
    // Arrange: Initialize the mock environment to simulate HSE failure
    set_hse_ready(false);
    RCC_OscInitTypeDef osc_init {};
        osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        osc_init.HSEState = RCC_HSE_ON;
        osc_init.HSIState = RCC_HSI_OFF;
        osc_init.HSICalibrationValue = 0;
        osc_init.PLL.PLLState = RCC_PLL_NONE;

    // Act: Call the function under test
    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&osc_init);

    // Assert: Verify the results
    CHECK(status == HAL_ERROR);
    CHECK((RCC->CR & RCC_CR_HSEON) == RCC_CR_HSEON); // Check that HSEON bit is set
    CHECK((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY); // Check that HSERDY bit is NOT set
}

TEST_CASE("HAL_RCC_OscConfig - HSE Disable") {
    // Arrange: Set HSE as ready, then disable it
    set_hse_ready(true);
    RCC->CR |= RCC_CR_HSEON;
    RCC->CR |= RCC_CR_HSERDY;

    RCC_OscInitTypeDef osc_init {};
      osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
      osc_init.HSEState = RCC_HSE_OFF;
      osc_init.HSIState = RCC_HSI_OFF;
      osc_init.HSICalibrationValue = 0;
      osc_init.PLL.PLLState = RCC_PLL_NONE;

    // Act
    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&osc_init);

    // Assert
    CHECK(status == HAL_OK);
    CHECK((RCC->CR & RCC_CR_HSEON) == 0); // Check that HSEON bit is cleared
    CHECK((RCC->CR & RCC_CR_HSERDY) == 0); // Check that HSERDY bit is cleared
}

TEST_CASE("HAL_RCC_OscConfig - HSI Enable Success") {
    // Arrange:
    set_hsi_ready(true);
    RCC_OscInitTypeDef osc_init {};
      osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
      osc_init.HSEState = RCC_HSE_OFF;
      osc_init.HSIState = RCC_HSI_ON;
      osc_init.HSICalibrationValue = 0;
      osc_init.PLL.PLLState = RCC_PLL_NONE;

    // Act:
    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&osc_init);

    // Assert:
    CHECK(status == HAL_OK);
    CHECK((RCC->CR & RCC_CR_HSION) == RCC_CR_HSION);
    CHECK((RCC->CR & RCC_CR_HSIRDY) == RCC_CR_HSIRDY);
}

TEST_CASE("HAL_RCC_OscConfig - HSI Enable Failure") {
    // Arrange:
    set_hsi_ready(false);
    RCC_OscInitTypeDef osc_init {};
      osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
      osc_init.HSEState = RCC_HSE_OFF;
      osc_init.HSIState = RCC_HSI_ON;
      osc_init.HSICalibrationValue = 0;
      osc_init.PLL.PLLState = RCC_PLL_NONE;

    // Act:
    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&osc_init);

    // Assert:
    CHECK(status == HAL_ERROR);
    CHECK((RCC->CR & RCC_CR_HSION) == RCC_CR_HSION);
    CHECK((RCC->CR & RCC_CR_HSIRDY) != RCC_CR_HSIRDY);
}

TEST_CASE("HAL_RCC_OscConfig - HSI Disable") {
    // Arrange: Set HSI as ready, then disable it
    set_hsi_ready(true);
    RCC->CR |= RCC_CR_HSION;
    RCC->CR |= RCC_CR_HSIRDY;

    RCC_OscInitTypeDef osc_init {};
        osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc_init.HSEState = RCC_HSE_OFF;
        osc_init.HSIState = RCC_HSI_OFF;
        osc_init.HSICalibrationValue = 0;
        osc_init.PLL.PLLState = RCC_PLL_NONE;

    // Act
    HAL_StatusTypeDef status = HAL_RCC_OscConfig(&osc_init);

    // Assert
    CHECK(status == HAL_OK);
    CHECK((RCC->CR & RCC_CR_HSION) == 0); // Check that HSION bit is cleared
    CHECK((RCC->CR & RCC_CR_HSIRDY) == 0); // Check that HSIRDY bit is cleared
}

TEST_CASE("HAL_RCC_ClockConfig - Set System Clock Source") {
    // Arrange:
    RCC_ClkInitTypeDef clk_init = {
        .ClockType = RCC_CLOCKTYPE_SYSCLK,
        .SYSCLKSource = RCC_SYSCLKSOURCE_HSE, // Example source
        .AHBCLKDivider = RCC_SYSCLK_DIV1,
        .APB1CLKDivider = RCC_HCLK_DIV1,
        .APB2CLKDivider = RCC_HCLK_DIV1
    };
    uint32_t flash_latency = 2;

    // Act:
    HAL_StatusTypeDef status = HAL_RCC_ClockConfig(&clk_init, flash_latency);

    // Assert:
    CHECK(status == HAL_OK);
    CHECK(HAL_RCC_GetSysClockSource() == RCC_SYSCLKSOURCE_HSE);
    CHECK(HAL_RCC_GetFlashLatency() == flash_latency);
    CHECK(get_rcc_clk_init_struct().SYSCLKSource == RCC_SYSCLKSOURCE_HSE);
    CHECK(get_flash_latency() == flash_latency);
}

TEST_CASE("set_hse_ready sets RCC->CR bits correctly") {
    //Arrange
    set_hse_ready(true);

    //Act and Assert
    CHECK((RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY);

    //Arrange
    set_hse_ready(false);

    //Act and Assert
    CHECK((RCC->CR & RCC_CR_HSERDY) == 0);
}

TEST_CASE("set_hsi_ready sets RCC->CR bits correctly") {
    //Arrange
    set_hsi_ready(true);

    //Act and Assert
    CHECK((RCC->CR & RCC_CR_HSIRDY) == RCC_CR_HSIRDY);

    //Arrange
    set_hsi_ready(false);

    //Act and Assert
    CHECK((RCC->CR & RCC_CR_HSIRDY) == 0);
}