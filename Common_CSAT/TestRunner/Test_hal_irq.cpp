#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

TEST_CASE("Test HAL IRQ Enable")
{
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

TEST_CASE("Test HAL IRQ Disable")
{
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
}
