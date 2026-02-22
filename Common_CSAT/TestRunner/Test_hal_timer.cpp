#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <stdio.h>

//--- Timer Tests ---

TEST_CASE("HAL_TIM_PWM_Start and Stop")
{
    TIM_HandleTypeDef htim{};
    reset_timer_state(&htim);

    SUBCASE("Start PWM on channel 1")
    {
        HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
        CHECK(is_pwm_started(&htim, TIM_CHANNEL_1) == true);
    }

    SUBCASE("Stop PWM on channel 1")
    {
        HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim, TIM_CHANNEL_1);
        CHECK(is_pwm_started(&htim, TIM_CHANNEL_1) == false);
    }

    SUBCASE("Start PWM on multiple channels")
    {
        HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim, TIM_CHANNEL_2);
        CHECK(is_pwm_started(&htim, TIM_CHANNEL_1));
        CHECK(is_pwm_started(&htim, TIM_CHANNEL_2));
    }
}

TEST_CASE("__HAL_TIM_SET_COMPARE behavior")
{
    TIM_HandleTypeDef htim{};
    reset_timer_state(&htim);

    SUBCASE("Set compare value on channel 1")
    {
        __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, 123);
        CHECK(get_compare_value(&htim, TIM_CHANNEL_1) == 123);
    }

    SUBCASE("Set compare values independently")
    {
        __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, 100);
        __HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_2, 200);
        CHECK(get_compare_value(&htim, TIM_CHANNEL_1) == 100);
        CHECK(get_compare_value(&htim, TIM_CHANNEL_2) == 200);
    }
}

TEST_CASE("Invalid channel handling")
{
    TIM_HandleTypeDef htim{};
    reset_timer_state(&htim);

    SUBCASE("Start on invalid channel")
    {
        HAL_TIM_PWM_Start(&htim, 0xFFFF); // Should not crash
        CHECK(is_pwm_started(&htim, 0xFFFF) == false);
    }

    SUBCASE("Set compare on invalid channel")
    {
        __HAL_TIM_SET_COMPARE(&htim, 0xFFFF, 999);
        CHECK(get_compare_value(&htim, 0xFFFF) == 0);
    }
}
