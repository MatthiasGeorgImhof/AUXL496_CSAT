#ifndef MOCK_HAL_TIMER_H
#define MOCK_HAL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//--- Timer Channel Defines ---
#define TIM_CHANNEL_1 ((uint32_t)0x000001)
#define TIM_CHANNEL_2 ((uint32_t)0x000002)
#define TIM_CHANNEL_3 ((uint32_t)0x000004)
#define TIM_CHANNEL_4 ((uint32_t)0x000008)
#define TIM_CHANNEL_5 ((uint32_t)0x000010)
#define TIM_CHANNEL_6 ((uint32_t)0x000020)
#define TIM_CHANNEL_7 ((uint32_t)0x000040)
#define TIM_CHANNEL_8 ((uint32_t)0x000080)
#define TIM_CHANNEL_9 ((uint32_t)0x000100)
#define TIM_CHANNEL_10 ((uint32_t)0x000200)
#define TIM_CHANNEL_11 ((uint32_t)0x000400)
#define TIM_CHANNEL_12 ((uint32_t)0x000800)
#define TIM_CHANNEL_13 ((uint32_t)0x001000)
#define TIM_CHANNEL_14 ((uint32_t)0x002000)
#define TIM_CHANNEL_15 ((uint32_t)0x004000)
#define TIM_CHANNEL_16 ((uint32_t)0x008000)
#define TIM_CHANNEL_17 ((uint32_t)0x010000)
#define TIM_CHANNEL_18 ((uint32_t)0x020000)
#define TIM_CHANNEL_19 ((uint32_t)0x040000)
#define TIM_CHANNEL_20 ((uint32_t)0x080000)
#define MAX_TIMER_CHANNELS 20

//--- Timer Mock Structures ---
typedef struct {
    bool pwm_started[MAX_TIMER_CHANNELS];
    uint32_t compare_value[MAX_TIMER_CHANNELS];
    uint32_t arr_value; // Auto-reload value
} MockTimerState;

typedef struct {
    void* Instance; // Timer instance (TIM15, TIM16, etc.)
    MockTimerState State;
} TIM_HandleTypeDef;

//--- Timer Mock Function Prototypes ---
void HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t channel);
void HAL_TIM_PWM_Stop(TIM_HandleTypeDef* htim, uint32_t channel);
void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t value);

//--- Accessors for Testing ---
bool is_pwm_started(TIM_HandleTypeDef* htim, uint32_t channel);
uint32_t get_compare_value(TIM_HandleTypeDef* htim, uint32_t channel);
void reset_timer_state(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif

#endif // MOCK_HAL_TIMER_H
