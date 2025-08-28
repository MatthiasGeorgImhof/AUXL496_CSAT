#include "mock_hal/mock_hal_clock.h"
#include "mock_hal/mock_hal_time.h" // For HAL_Delay

#include <cstring> // for memset

// Static variables to track clock state
static bool hse_is_ready = false;
static bool hsi_is_ready = false;
static uint32_t current_sys_clock_source = RCC_SYSCLKSOURCE_HSI;
static uint32_t simulated_flash_latency = 0;
static RCC_OscInitTypeDef last_rcc_osc_init_struct;
static RCC_ClkInitTypeDef last_rcc_clk_init_struct;

RCC_TypeDef mock_RCC = {0};
RCC_TypeDef *RCC = &mock_RCC;

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct) {
    // Mock implementation: Store the configuration and simulate success/failure.

    //Save the configuration:
    last_rcc_osc_init_struct = *RCC_OscInitStruct;

    if (RCC_OscInitStruct->OscillatorType == RCC_OSCILLATORTYPE_HSE) {
        if (RCC_OscInitStruct->HSEState == RCC_HSE_ON) {
            // Simulate enabling HSE.  For real hardware, this takes time.
            // Here, we can simply set the HSE ready flag after a delay.
            // Simulate the RCC->CR register (important for HSClockSwitcher)
            RCC->CR |= RCC_CR_HSEON;
            HAL_Delay(1); // Simulate startup delay. You could make this configurable.
            if (hse_is_ready) {
                RCC->CR |= RCC_CR_HSERDY;
                return HAL_OK;
            } else {
                return HAL_ERROR; // Simulate HSE failing to start
            }
        } else if (RCC_OscInitStruct->HSEState == RCC_HSE_OFF) {
            RCC->CR &= ~RCC_CR_HSEON;
            RCC->CR &= ~RCC_CR_HSERDY;
            return HAL_OK;
        }
    } else if (RCC_OscInitStruct->OscillatorType == RCC_OSCILLATORTYPE_HSI) {
        if (RCC_OscInitStruct->HSIState == RCC_HSI_ON) {
            RCC->CR |= RCC_CR_HSION;
            HAL_Delay(1); // Simulate startup delay
            if (hsi_is_ready) {
                RCC->CR |= RCC_CR_HSIRDY;
                return HAL_OK;
            } else {
                return HAL_ERROR; // Simulate HSI failing to start
            }
        } else if (RCC_OscInitStruct->HSIState == RCC_HSI_OFF) {
            RCC->CR &= ~RCC_CR_HSION;
            RCC->CR &= ~RCC_CR_HSIRDY;
            return HAL_OK;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FlashLatency) {
    // Mock implementation: Store the clock configuration and simulate switching.
    last_rcc_clk_init_struct = *RCC_ClkInitStruct;
    simulated_flash_latency = FlashLatency;
    current_sys_clock_source = RCC_ClkInitStruct->SYSCLKSource; // Update the system clock source

    return HAL_OK;
}

uint32_t HAL_RCC_GetSysClockSource(void) {
    // Mock implementation: Return the currently selected system clock source.
    return current_sys_clock_source;
}

uint32_t HAL_RCC_GetFlashLatency(void) {
    // Mock implementation: Return the simulated flash latency
    return simulated_flash_latency;
}

void set_hse_ready(bool ready) {
    hse_is_ready = ready;
    if (ready) {
        RCC->CR |= RCC_CR_HSERDY;
    } else {
        RCC->CR &= ~RCC_CR_HSERDY;
    }
}

void set_hsi_ready(bool ready) {
    hsi_is_ready = ready;
    if (ready) {
        RCC->CR |= RCC_CR_HSIRDY;
    } else {
        RCC->CR &= ~RCC_CR_HSIRDY;
    }
}

void set_sys_clock_source(uint32_t source) {
    current_sys_clock_source = source;
}

void set_flash_latency(uint32_t latency) {
    simulated_flash_latency = latency;
}

RCC_OscInitTypeDef get_rcc_osc_init_struct(){
  return last_rcc_osc_init_struct;
}

RCC_ClkInitTypeDef get_rcc_clk_init_struct(){
  return last_rcc_clk_init_struct;
}

uint32_t get_flash_latency(){
  return simulated_flash_latency;
}

void reset_rcc(void) {
    memset(&mock_RCC, 0, sizeof(mock_RCC)); // Reset RCC registers
    hse_is_ready = false;
    hsi_is_ready = false;
    current_sys_clock_source = RCC_SYSCLKSOURCE_HSI;
    simulated_flash_latency = 0;
    memset(&last_rcc_osc_init_struct, 0, sizeof(last_rcc_osc_init_struct));
    memset(&last_rcc_clk_init_struct, 0, sizeof(last_rcc_clk_init_struct));
}