// mock_hal_timer.c

#include "mock_hal/mock_hal_timer.h"
#include <stdio.h>

static size_t channel_index(uint32_t channel) {
    switch (channel) {
        case TIM_CHANNEL_1: return 0;
        case TIM_CHANNEL_2: return 1;
        case TIM_CHANNEL_3: return 2;
        case TIM_CHANNEL_4: return 3;
        case TIM_CHANNEL_5: return 4;
        case TIM_CHANNEL_6: return 5;
        case TIM_CHANNEL_7: return 6;
        case TIM_CHANNEL_8: return 7;
        case TIM_CHANNEL_9: return 8;
        case TIM_CHANNEL_10: return 9;
        case TIM_CHANNEL_11: return 10;
        case TIM_CHANNEL_12: return 11;
        case TIM_CHANNEL_13: return 12;
        case TIM_CHANNEL_14: return 13;
        case TIM_CHANNEL_15: return 14;
        case TIM_CHANNEL_16: return 15;
        case TIM_CHANNEL_17: return 16;
        case TIM_CHANNEL_18: return 17;
        case TIM_CHANNEL_19: return 18;
        case TIM_CHANNEL_20: return 10;
       default: return MAX_TIMER_CHANNELS; // Invalid
    }
}

void HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t channel) {
    size_t idx = channel_index(channel);
    if (idx < MAX_TIMER_CHANNELS) {
        htim->State.pwm_started[idx] = true;
    } else {
        printf("ERROR: Invalid channel in HAL_TIM_PWM_Start\n");
    }
}

void HAL_TIM_PWM_Stop(TIM_HandleTypeDef* htim, uint32_t channel) {
    size_t idx = channel_index(channel);
    if (idx < MAX_TIMER_CHANNELS) {
        htim->State.pwm_started[idx] = false;
    } else {
        printf("ERROR: Invalid channel in HAL_TIM_PWM_Stop\n");
    }
}

void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t value) {
    size_t idx = channel_index(channel);
    if (idx < MAX_TIMER_CHANNELS) {
        htim->State.compare_value[idx] = value;
    } else {
        printf("ERROR: Invalid channel in __HAL_TIM_SET_COMPARE\n");
    }
}

bool is_pwm_started(TIM_HandleTypeDef* htim, uint32_t channel) {
    size_t idx = channel_index(channel);
    return (idx < MAX_TIMER_CHANNELS) ? htim->State.pwm_started[idx] : false;
}

uint32_t get_compare_value(TIM_HandleTypeDef* htim, uint32_t channel) {
    size_t idx = channel_index(channel);
    return (idx < MAX_TIMER_CHANNELS) ? htim->State.compare_value[idx] : 0;
}

void reset_timer_state(TIM_HandleTypeDef* htim) {
    for (size_t i = 0; i < MAX_TIMER_CHANNELS; ++i) {
        htim->State.pwm_started[i] = false;
        htim->State.compare_value[i] = 0;
    }
    htim->State.arr_value = 0;
}
