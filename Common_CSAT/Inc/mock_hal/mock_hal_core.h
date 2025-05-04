#ifndef MOCK_HAL_CORE_H
#define MOCK_HAL_CORE_H

#include <stdint.h>
#include <stdbool.h>

//--- Buffer Size Defines ---
#define CAN_TX_BUFFER_SIZE    50   // Size of the CAN transmit buffer
#define CAN_RX_BUFFER_SIZE    50   // Size of the CAN receive buffer
#define UART_TX_BUFFER_SIZE   256  // Size of the UART transmit buffer
#define UART_RX_BUFFER_SIZE   256  // Size of the UART receive buffer
#define I2C_MEM_BUFFER_SIZE   256  // Size of the I2C memory buffer
#define USB_TX_BUFFER_SIZE    256  // Size of the USB transmit buffer
#define SPI_TX_BUFFER_SIZE    256  // Size of the SPI transmit buffer
#define SPI_RX_BUFFER_SIZE    256  // Size of the SPI receive buffer
#define DCMI_IMAGE_BUFFER_SIZE  (640*480)  //Example VGA resolution. Adjust for your use case

//------------------------------------------------------------------------------
//  CONDITIONAL DEFINITION OF __IO
//------------------------------------------------------------------------------

#ifndef __IO
  #ifdef MOCK_HAL_VOLATILE
    #define __IO volatile
  #else
    #define __IO
  #endif
#endif

//------------------------------------------------------------------------------
//  DEFINES - Configuration and Constants
//------------------------------------------------------------------------------

//--- General HAL Defines ---
#define HAL_MAX_DELAY           0xFFFFFFFFU  // Maximum delay value for HAL_Delay
#define HAL_OK                  0x00         // HAL Status: Operation successful
#define HAL_ERROR               0x01         // HAL Status: Operation failed
#define HAL_BUSY                0x02         // HAL Status: Resource busy
#define HAL_TIMEOUT             0x03         // HAL Status: Operation timed out

//------------------------------------------------------------------------------
//  ENUMS - Enumerated Types
//------------------------------------------------------------------------------

//--- HAL Status ---
typedef uint32_t HAL_StatusTypeDef;

//--- GPIO Pin State ---
typedef enum {
    GPIO_PIN_RESET = 0,  // GPIO Pin is reset (low)
    GPIO_PIN_SET = 1      // GPIO Pin is set (high)
} GPIO_PinState;

//---HAL Lock Type Definitions ---
typedef enum
{
  HAL_UNLOCKED = 0x00U,                       /*!< Object is not locked                                      */
  HAL_LOCKED   = 0x01U                        /*!< Object is locked                                          */
} HAL_LockTypeDef;


#endif /* MOCK_HAL_CORE_H */