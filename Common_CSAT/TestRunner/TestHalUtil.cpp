#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "hal_util.h"
#include "mock_hal.h"
#include <stdio.h>

TEST_CASE("DELAY_init and DELAY_microseconds test") {
    // Setup: Configure SysTick to a known frequency
    uint32_t testTicks = 2000; // e.g., 16 MHz clock, meaning SysTick happens every 1ms (1000us)
    HAL_SYSTICK_Config(testTicks); // Configure SysTick
    SysTick->VAL = 999; //Start the timer count at zero

    // Initialize DELAY_init to prep for delay micro
    DELAY_init();

    // Tests
    CHECK(gSysTickLoad == testTicks); //Check gSysTickLoad

    //Test that gTicksPerMicrosecondFloor is correct
    CHECK(gTicksPerMicrosecondFloor == (testTicks+1) / 1000);

    //Test that gTicksPerMicrosecondMod1000 is correct
    CHECK(gTicksPerMicrosecondMod1000 == (testTicks+1) % 1000);

    //Test a few delay values.
    uint32_t initialTick = HAL_GetTick();
    DELAY_microseconds(100); //Delay 10 us
    uint32_t finalTick = HAL_GetTick();

    //We need an approximate value to check this is true, given that ticks are only at a millisecond, and not microseconds
    CHECK((finalTick-initialTick) == 0); //Because 100us is so small, it should have been rounded off.
    return;
    initialTick = HAL_GetTick();
    SysTick->VAL = testTicks;
    DELAY_microseconds(500); //Delay 500us
    finalTick = HAL_GetTick();

    CHECK((finalTick-initialTick) == 0); //Because 500us is so small, it should have been rounded off.

    initialTick = HAL_GetTick();
    SysTick->VAL = testTicks;
    DELAY_microseconds(1000); //Delay 1000us
    finalTick = HAL_GetTick();
    CHECK((finalTick-initialTick) == 1); //1000 us should mean one tick.
}