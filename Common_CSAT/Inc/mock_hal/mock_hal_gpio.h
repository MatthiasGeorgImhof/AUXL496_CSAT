#ifndef MOCK_HAL_GPIO_H
#define MOCK_HAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Include core definitions
#include "mock_hal/mock_hal_core.h"

//--- GPIO Defines ---
#define GPIO_PIN_0  ((uint16_t)0x0001)  // Pin 0 selected
#define GPIO_PIN_1  ((uint16_t)0x0002)  // Pin 1 selected
#define GPIO_PIN_2  ((uint16_t)0x0004)  // Pin 2 selected
#define GPIO_PIN_3  ((uint16_t)0x0008)  // Pin 3 selected
#define GPIO_PIN_4  ((uint16_t)0x0010)  // Pin 4 selected
#define GPIO_PIN_5  ((uint16_t)0x0020)  // Pin 5 selected
#define GPIO_PIN_6  ((uint16_t)0x0040)  // Pin 6 selected
#define GPIO_PIN_7  ((uint16_t)0x0080)  // Pin 7 selected
#define GPIO_PIN_8  ((uint16_t)0x0100)  // Pin 8 selected
#define GPIO_PIN_9  ((uint16_t)0x0200)  // Pin 9 selected
#define GPIO_PIN_10 ((uint16_t)0x0400) // Pin 10 selected
#define GPIO_PIN_11 ((uint16_t)0x0800) // Pin 11 selected
#define GPIO_PIN_12 ((uint16_t)0x1000) // Pin 12 selected
#define GPIO_PIN_13 ((uint16_t)0x2000) // Pin 13 selected
#define GPIO_PIN_14 ((uint16_t)0x4000) // Pin 14 selected
#define GPIO_PIN_15 ((uint16_t)0x8000) // Pin 15 selected
#define GPIO_PIN_All ((uint16_t)0xFFFF)  // All pins selected
#define GPIO_PIN_MASK ((uint32_t)0x0000FFFF) // PIN mask for assert test

//--- GPIO Structures ---
typedef struct {
    uint32_t Pin;         // Specifies the GPIO pins to be configured
    uint32_t Mode;        // Specifies the operating mode for the selected pins
    uint32_t Pull;        // Specifies the Pull-up or Pull-down resistance for the selected pins
    uint32_t Speed;       // Specifies the speed for the selected pins
    uint32_t Alternate;   // Specifies the alternate function to be used for the selected pins
} GPIO_InitTypeDef;

typedef struct {
    void *Instance;      // GPIO port instance (GPIOA, GPIOB, etc.) - void* for mocking
    GPIO_InitTypeDef Init; // GPIO initialization structure
} GPIO_TypeDef;

//--- GPIO Mock Function Prototypes ---
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

//--- GPIO Access Function Prototypes ---
GPIO_PinState get_gpio_pin_state(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void set_gpio_pin_state(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_GPIO_H */