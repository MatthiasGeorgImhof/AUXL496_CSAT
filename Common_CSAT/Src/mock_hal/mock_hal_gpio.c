#ifdef __x86_64__

#include "mock_hal/mock_hal_gpio.h"
#include <stdio.h>  // For printf (error handling)

//--- GPIO Port State ---
typedef struct {
    GPIO_PinState pin_state[32]; // Assuming a maximum of 32 pins per port.  Adjust if needed.
} MockGPIO_PortState;

MockGPIO_PortState gpio_port_state;  //  Only one port for mocking

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init) {
  // In a simple mock, we don't *really* initialize anything.
  // But you *could* store the GPIO_Init values if you wanted to make assertions about them later.
  //For now, just save init structure
   GPIOx->Init = *GPIO_Init;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef */*GPIOx*/, uint16_t GPIO_Pin) {
    uint32_t pin_number = 0;
    while(GPIO_Pin != (1 << pin_number) && pin_number < 32) {
        pin_number++;
    }

    if (pin_number < 32) {
        return gpio_port_state.pin_state[pin_number];
    } else {
        // Handle invalid pin number (e.g., return GPIO_PIN_RESET or an error code).
        return GPIO_PIN_RESET; // Or potentially an error value
    }
}

void HAL_GPIO_WritePin(GPIO_TypeDef */*GPIOx*/, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    uint32_t pin_number = 0;
    while(GPIO_Pin != (1 << pin_number) && pin_number < 32) {
        pin_number++;
    }

    if (pin_number < 32) {
        gpio_port_state.pin_state[pin_number] = PinState;
    } else {
        // Handle invalid pin number (e.g., print an error message).
        printf("ERROR: Invalid GPIO_Pin in HAL_GPIO_WritePin\n");
    }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef */*GPIOx*/, uint16_t GPIO_Pin) {
    uint32_t pin_number = 0;
    while(GPIO_Pin != (1 << pin_number) && pin_number < 32) {
        pin_number++;
    }

    if (pin_number < 32) {
       gpio_port_state.pin_state[pin_number] = (gpio_port_state.pin_state[pin_number] == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    } else {
        // Handle invalid pin number
        printf("ERROR: Invalid GPIO_Pin in HAL_GPIO_TogglePin\n");
    }
}

GPIO_PinState get_gpio_pin_state(GPIO_TypeDef */*GPIOx*/, uint16_t GPIO_Pin)
{
    uint32_t pin_number = 0;
    while(GPIO_Pin != (1 << pin_number) && pin_number < 32) {
        pin_number++;
    }

    if (pin_number < 32) {
        return gpio_port_state.pin_state[pin_number];
    } else {
        // Handle invalid pin number (e.g., return GPIO_PIN_RESET or an error code).
        return GPIO_PIN_RESET; // Or potentially an error value
    }
}

void set_gpio_pin_state(GPIO_TypeDef */*GPIOx*/, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    uint32_t pin_number = 0;
    while(GPIO_Pin != (1 << pin_number) && pin_number < 32) {
        pin_number++;
    }

    if (pin_number < 32) {
        gpio_port_state.pin_state[pin_number] = PinState;
    } else {
        // Handle invalid pin number (e.g., print an error message).
        printf("ERROR: Invalid GPIO_Pin in HAL_GPIO_WritePin\n");
    }
}
#endif