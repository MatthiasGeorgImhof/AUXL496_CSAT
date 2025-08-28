#ifndef MOCK_HAL_CLOCK_H
#define MOCK_HAL_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "mock_hal/mock_hal_core.h" // For HAL_StatusTypeDef, etc.

//--- Clock Defines ---

// RCC Oscillator Types
#define RCC_OSCILLATORTYPE_HSI   ((uint32_t)0x00000001U)
#define RCC_OSCILLATORTYPE_HSE   ((uint32_t)0x00000002U)

// RCC HSE State
#define RCC_HSE_OFF              ((uint32_t)0x00000000U)
#define RCC_HSE_ON               ((uint32_t)0x00000001U)
#define RCC_HSE_BYPASS           ((uint32_t)0x00000003U)  // Optional if you need to test bypass

// RCC HSI State
#define RCC_HSI_OFF             ((uint32_t)0x00000000U)
#define RCC_HSI_ON              ((uint32_t)0x00000001U)

#define RCC_HSICALIBRATION_DEFAULT  ((uint8_t)0x10U) // Default HSI calibration value

// RCC PLL State
#define RCC_PLL_NONE             ((uint32_t)0x00000000U)
#define RCC_PLL_ON               ((uint32_t)0x00000001U)

// Clock Types
#define RCC_CLOCKTYPE_SYSCLK     ((uint32_t)0x00000001U)
#define RCC_CLOCKTYPE_HCLK       ((uint32_t)0x00000002U)
#define RCC_CLOCKTYPE_PCLK1      ((uint32_t)0x00000004U)
#define RCC_CLOCKTYPE_PCLK2      ((uint32_t)0x00000008U)

// System Clock Source
#define RCC_SYSCLKSOURCE_HSI     ((uint32_t)0x00000000U)
#define RCC_SYSCLKSOURCE_HSE     ((uint32_t)0x00000004U)
#define RCC_SYSCLKSOURCE_PLLCLK  ((uint32_t)0x00000008U) // Not used in the provided class

// AHB Clock Divider
#define RCC_SYSCLK_DIV1          ((uint32_t)0x00000000U)

// APB Clock Divider
#define RCC_HCLK_DIV1            ((uint32_t)0x00000000U)

// RCC Register Bit Definitions (for checking status)
#define RCC_CR_HSEON             ((uint32_t)0x00010000U)  // HSE clock enable bit
#define RCC_CR_HSERDY            ((uint32_t)0x00020000U)  // HSE clock ready flag
#define RCC_CR_HSION             ((uint32_t)0x00000001U)  // HSI clock enable bit
#define RCC_CR_HSIRDY            ((uint32_t)0x00000002U)  // HSI clock ready flag

//--- Clock Structures ---

typedef struct {
    uint32_t OscillatorType;  // Specifies the oscillator type to configure
    uint32_t HSEState;        // Specifies the new state of the HSE
    uint32_t HSIState;        // Specifies the new state of the HSI
    uint8_t HSICalibrationValue; // Specifies the HSI calibration trimming value.
    struct {
        uint32_t PLLState;    // Specifies the new state of the PLL.
    } PLL;
} RCC_OscInitTypeDef;

typedef struct {
    uint32_t ClockType;       // Specifies the clock to be configured
    uint32_t SYSCLKSource;    // Specifies the clock source for SYSCLK
    uint32_t AHBCLKDivider;   // Specifies the AHB clock divider
    uint32_t APB1CLKDivider;  // Specifies the APB1 clock divider
    uint32_t APB2CLKDivider;  // Specifies the APB2 clock divider
} RCC_ClkInitTypeDef;

//Mock RCC Structure - important to track the values set.
typedef struct {
  uint32_t CR;  // Clock control register
  // add more registers as needed for mocking
} RCC_TypeDef;

extern RCC_TypeDef *RCC;

//--- Clock Function Prototypes ---
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FlashLatency);
uint32_t HAL_RCC_GetSysClockSource(void);
uint32_t HAL_RCC_GetFlashLatency(void);

//--- Clock Helper Function Prototypes ---

// Set desired state of HSE ready flag
void set_hse_ready(bool ready);
// Set desired state of HSI ready flag
void set_hsi_ready(bool ready);

// Allows setting the system clock source for tests
void set_sys_clock_source(uint32_t source);

// Set the simulated flash latency (used by HAL_RCC_ClockConfig)
void set_flash_latency(uint32_t latency);

//Getter functions to see what was set
RCC_OscInitTypeDef get_rcc_osc_init_struct();
RCC_ClkInitTypeDef get_rcc_clk_init_struct();
uint32_t get_flash_latency();

// Reset the RCC structure to a known state
void reset_rcc(void);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_CLOCK_H */