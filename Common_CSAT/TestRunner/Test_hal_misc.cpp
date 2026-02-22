#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

TEST_CASE("HAL_Delay")
{
    set_current_tick(0);
    uint32_t testTicks = 10;
    HAL_SYSTICK_Config(testTicks);  // Load 10 ticks
    HAL_Delay(100);
    CHECK(HAL_GetTick() == 100);
    HAL_Delay(200);
    CHECK(HAL_GetTick() == 300);

    SUBCASE("HAL_Delay(0)") {
        uint32_t initialTick = HAL_GetTick();
        HAL_Delay(0);
        CHECK(HAL_GetTick() == initialTick);
    }

    SUBCASE("HAL_Delay(10000)") {  
        uint32_t initialTick = HAL_GetTick();
        HAL_Delay(10000);
        CHECK(HAL_GetTick() > initialTick); 
    }

    SUBCASE("HAL_Delay(UINT32_MAX)") {  
        uint32_t initialTick = HAL_GetTick();
        HAL_Delay(UINT32_MAX);
        CHECK(HAL_GetTick() == initialTick);
    }
}

TEST_CASE("HAL_GetTick")
{
    set_current_tick(10);
    CHECK(HAL_GetTick() == 10);
    set_current_tick(20);
    CHECK(HAL_GetTick() == 20);
}

TEST_CASE("HAL_SYSTICK_Config")
{
    // Setup: Reset SysTick load value before each test
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    uint32_t testTicks = 16000000; // Example value (16 MHz clock)
    HAL_StatusTypeDef result = HAL_SYSTICK_Config(testTicks);

    CHECK(result == HAL_OK);
    CHECK(SysTick->LOAD == testTicks); // Check the actual register through our helper
    //CHECK(SysTick->VAL == 0); //VAL also tested
    SUBCASE("HAL_SYSTICK_Config(0)") {
        HAL_StatusTypeDef result = HAL_SYSTICK_Config(0);
        CHECK(result == HAL_OK);
        CHECK(SysTick->LOAD == 0);
        CHECK(SysTick->VAL == 0);
    }
}

TEST_CASE("HAL_InitTick")
{
    // Since HAL_InitTick doesn't *really* do anything in the mock,
    // we can only check that it returns HAL_OK without crashing.

    HAL_StatusTypeDef result = HAL_InitTick(10); // Example priority value
    CHECK(result == HAL_OK);
}

TEST_CASE("HAL Utilities and GSYSTICKLOAD values")
{
    // Test after HAL_Init and DELAY_init for general values, can add more tests later
    uint32_t testTicks = 48000;
    HAL_SYSTICK_Config(testTicks);
    CHECK(SysTick->LOAD == testTicks);
}

TEST_CASE("HAL_IncTick") {
    uint32_t testTicks = 10;
    HAL_SYSTICK_Config(testTicks);  // Load 10 ticks

    SUBCASE("Decrementing SysTick->VAL") {
        set_current_tick(10); //set global tick
        SysTick->VAL = 1;
        HAL_IncTick();
        CHECK(SysTick->VAL == 0);
        CHECK(HAL_GetTick() == 10); //Tick not incremented yet
    }

    SUBCASE("SysTick Rollover") {
        set_current_tick(10); //set global tick
        SysTick->VAL = 0; // Set to zero to trigger rollover
        HAL_IncTick();
        CHECK(SysTick->VAL == testTicks);   //Resets to LOAD (10) after rollover
        CHECK(HAL_GetTick() == 11); // Tick incremented after rollover

    }

     SUBCASE("Tick Increment after Rollover")
     {
         set_current_tick(5); // Set HAL tick to 5.
         SysTick->VAL = 0;    // Set current VAL to 0 to trigger Rollover

         HAL_IncTick(); // Decrements VAL to 0, resets to LOAD, and increments HAL_GetTick.

         CHECK(SysTick->VAL == testTicks);         // VAL reset to initial configured value (testTicks = 10)
         CHECK(HAL_GetTick() == 6); // HAL tick incremented to 6
     }
}