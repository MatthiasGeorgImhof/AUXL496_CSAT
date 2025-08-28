#include "HSClockSwitch.hpp"

constexpr uint32_t TIMEOUT = 10;

HSClockSwitch::HSClockSwitch() {}

HAL_StatusTypeDef HSClockSwitch::switchToHSE()
{
    // 1. Configure the HSE clock
    HAL_StatusTypeDef status = configureHSE();
    if (status != HAL_OK)
    {
        return status;
    }

    // 2. Select HSE as the system clock source
    status = selectClockSource(RCC_SYSCLKSOURCE_HSE);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HSClockSwitch::switchToHSI()
{
    // 1. Configure the HSI clock
    HAL_StatusTypeDef status = configureHSI();
    if (status != HAL_OK)
    {
        return status;
    }

    // 2. Select HSI as the system clock source
    status = selectClockSource(RCC_SYSCLKSOURCE_HSI);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HSClockSwitch::configureHSE()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSE,
        .HSEState = RCC_HSE_ON,
        .HSIState = RCC_HSI_OFF,
        .HSICalibrationValue = 0, // Or the default value
        .PLL = {.PLLState = RCC_PLL_NONE} // Initialize PLL struct
    };

    // Check if HSE is already enabled. If so, no need to configure.
    if ((RCC->CR & RCC_CR_HSEON) == RCC_CR_HSEON)
    {
        return HAL_OK; // HSE is already running
    }

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Optional: Wait for HSE to be ready.  Important for reliable operation.
    uint32_t timeout = TIMEOUT;
    while (!(RCC->CR & RCC_CR_HSERDY))
    {
        if (timeout == 0)
        {
            return HAL_TIMEOUT; // HSE failed to start
        }
        HAL_Delay(1); // Small delay
        timeout--;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HSClockSwitch::configureHSI()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSI,
        .HSEState = RCC_HSE_OFF, // Ensure HSE is off when configuring HSI
        .HSIState = RCC_HSI_ON,
        .HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT,
        .PLL = {.PLLState = RCC_PLL_NONE} // Initialize PLL struct
    };

    // Check if HSI is already enabled. If so, no need to configure.
    if ((RCC->CR & RCC_CR_HSION) == RCC_CR_HSION)
    {
        return HAL_OK; // HSI is already running
    }

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Optional: Wait for HSI to be ready.  Important for reliable operation.
    uint32_t timeout = TIMEOUT;
    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
        if (timeout == 0)
        {
            return HAL_TIMEOUT; // HSI failed to start
        }
        HAL_Delay(1); // Small delay
        timeout--;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HSClockSwitch::selectClockSource(uint32_t clockSource)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {
        .ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
        .SYSCLKSource = clockSource,
        .AHBCLKDivider = RCC_SYSCLK_DIV1,
        .APB1CLKDivider = RCC_HCLK_DIV1,
        .APB2CLKDivider = RCC_HCLK_DIV1
    };

    // Get the current Flash latency
    uint32_t flashLatency = HAL_RCC_GetFlashLatency();

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flashLatency) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // Ensure the new clock source is actually selected
    while (HAL_RCC_GetSysClockSource() != clockSource)
    {
        // Wait for the clock switch to complete. Add a timeout if needed.
    }

    return HAL_OK;
}

HSClockSwitchWithEnable::HSClockSwitchWithEnable(GPIO_TypeDef* GPIO, uint16_t pins)
    : HSClockSwitch(), GPIO_(GPIO), pins_(pins) {}

HAL_StatusTypeDef HSClockSwitchWithEnable::switchToHSE()
{
    
    HAL_GPIO_WritePin(GPIO_, pins_, GPIO_PIN_SET);
    HAL_StatusTypeDef result = HSClockSwitch::switchToHSE();
    return result;
}

HAL_StatusTypeDef HSClockSwitchWithEnable::switchToHSI()
{
    HAL_StatusTypeDef result = HSClockSwitch::switchToHSI();
    HAL_GPIO_WritePin(GPIO_, pins_, GPIO_PIN_RESET);
    return result;
}
