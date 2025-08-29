#ifndef MOCK_HAL_CLOCK_H
#define MOCK_HAL_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "mock_hal/mock_hal_core.h" // For HAL_StatusTypeDef, etc.
#define RCC_PLLP_SUPPORT

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

typedef struct
{
  uint32_t PLLState;   /*!< The new state of the PLL.
                            This parameter can be a value of @ref RCC_PLL_Config                      */
  uint32_t PLLSource;  /*!< RCC_PLLSource: PLL entry clock source.
                            This parameter must be a value of @ref RCC_PLL_Clock_Source               */
  uint32_t PLLM;       /*!< PLLM: Division factor for PLL VCO input clock.
                            This parameter must be a number between Min_Data = 1 and Max_Data = 16 on STM32L4Rx/STM32L4Sx devices.
                            This parameter must be a number between Min_Data = 1 and Max_Data = 8 on the other devices */
  uint32_t PLLN;       /*!< PLLN: Multiplication factor for PLL VCO output clock.
                            This parameter must be a number between Min_Data = 8 and Max_Data = 86    */
#if defined(RCC_PLLP_SUPPORT)
  uint32_t PLLP;       /*!< PLLP: Division factor for SAI clock.
                            This parameter must be a value of @ref RCC_PLLP_Clock_Divider             */
#endif /* RCC_PLLP_SUPPORT */
  uint32_t PLLQ;       /*!< PLLQ: Division factor for SDMMC1, RNG and USB clocks.
                            This parameter must be a value of @ref RCC_PLLQ_Clock_Divider             */
  uint32_t PLLR;       /*!< PLLR: Division for the main system clock.
                            User have to set the PLLR parameter correctly to not exceed max frequency 120MHZ
                            on STM32L4Rx/STM32L4Sx devices else 80MHz on the other devices.
                            This parameter must be a value of @ref RCC_PLLR_Clock_Divider             */
} RCC_PLLInitTypeDef;


typedef struct
{
  uint32_t OscillatorType;       /*!< The oscillators to be configured.
                                      This parameter can be a value of @ref RCC_Oscillator_Type                   */
  uint32_t HSEState;             /*!< The new state of the HSE.
                                      This parameter can be a value of @ref RCC_HSE_Config                        */
  uint32_t LSEState;             /*!< The new state of the LSE.
                                      This parameter can be a value of @ref RCC_LSE_Config                        */
  uint32_t HSIState;             /*!< The new state of the HSI.
                                      This parameter can be a value of @ref RCC_HSI_Config                        */
  uint32_t HSICalibrationValue;  /*!< The calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                      This parameter must be a number between Min_Data = 0 and Max_Data = 31 on
                                      STM32L43x/STM32L44x/STM32L47x/STM32L48x devices.
                                      This parameter must be a number between Min_Data = 0 and Max_Data = 127 on
                                      the other devices */
  uint32_t LSIState;             /*!< The new state of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Config                        */
#if defined(RCC_CSR_LSIPREDIV)
  uint32_t LSIDiv;               /*!< The division factor of the LSI.
                                      This parameter can be a value of @ref RCC_LSI_Div                           */
#endif /* RCC_CSR_LSIPREDIV */
  uint32_t MSIState;             /*!< The new state of the MSI.
                                      This parameter can be a value of @ref RCC_MSI_Config */
  uint32_t MSICalibrationValue;  /*!< The calibration trimming value (default is RCC_MSICALIBRATION_DEFAULT).
                                      This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF */
  uint32_t MSIClockRange;        /*!< The MSI frequency range.
                                      This parameter can be a value of @ref RCC_MSI_Clock_Range  */
  uint32_t HSI48State;             /*!< The new state of the HSI48 (only applicable to STM32L43x/STM32L44x/STM32L49x/STM32L4Ax devices).
                                        This parameter can be a value of @ref RCC_HSI48_Config */
  RCC_PLLInitTypeDef PLL;        /*!< Main PLL structure parameters                                               */

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