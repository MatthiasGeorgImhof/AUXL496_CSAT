#ifndef MOCK_HAL_DCMI_H
#define MOCK_HAL_DCMI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Include core definitions
#include "mock_hal/mock_hal_core.h"

//--- DCMI Defines ---
#define DCMI_MODE_CONTINUOUS      0x00000000U  // Example: Continuous capture mode
#define DCMI_SYNCHRO_HARDWARE     0x00000001U  // Example: Hardware synchronization

//--- DCMI Structures ---
typedef enum
{
  HAL_DCMI_STATE_RESET             = 0x00U,  /*!< DCMI not yet initialized or disabled                     */
  HAL_DCMI_STATE_READY             = 0x01U,  /*!< DCMI initialized and ready for use                        */
  HAL_DCMI_STATE_BUSY              = 0x02U,  /*!< DCMI transfer is ongoing                                  */
  HAL_DCMI_STATE_TIMEOUT           = 0x03U,  /*!< DCMI timeout state                                        */
  HAL_DCMI_STATE_ERROR             = 0x04U,  /*!< DCMI error state                                          */
} HAL_DCMI_StateTypeDef;

typedef struct {
    uint32_t SynchroMode;        // Example: Synchronization mode
    uint32_t VSyncPolarity;      // Example: VSync polarity
    uint32_t HSyncPolarity;      // Example: HSync polarity
    uint32_t DataEnablePolarity; // Example: Data Enable Polarity
    uint32_t PCKPolarity;         // Example: Pixel Clock Polarity
    uint32_t CaptureRate;         // Example: Capture Rate
    uint32_t ExtendedDataMode;    // Example: Extended Data Mode
} DCMI_InitTypeDef;

typedef struct {
    void *Instance;               // DCMI peripheral instance (e.g., DCMI)
    DCMI_InitTypeDef Init;          // DCMI initialization structure
    HAL_LockTypeDef Lock;           // If you need to mock locking
    __IO HAL_DCMI_StateTypeDef State; // HAL state of peripheral
    uint32_t ErrorCode;             // Error code
    uint8_t *pFrameBuffer;          // Pointer to frame buffer (added for testing)
    uint32_t FrameWidth;            // Frame width (added for testing)
    uint32_t FrameHeight;           // Frame height (added for testing)
} DCMI_HandleTypeDef;

//--- DCMI Mock Function Prototypes ---
HAL_StatusTypeDef HAL_DCMI_Init(DCMI_HandleTypeDef *hdcmi);
HAL_StatusTypeDef HAL_DCMI_DeInit(DCMI_HandleTypeDef *hdcmi);
HAL_StatusTypeDef HAL_DCMI_Start(DCMI_HandleTypeDef *hdcmi, uint32_t Mode, uint32_t DMA_InitStruct);
HAL_StatusTypeDef HAL_DCMI_Stop(DCMI_HandleTypeDef *hdcmi);
HAL_DCMI_StateTypeDef HAL_DCMI_GetState(DCMI_HandleTypeDef *hdcmi);
uint32_t HAL_DCMI_GetError(DCMI_HandleTypeDef *hdcmi); //return ErrorCode

//--- DCMI Access/Helper Function Prototypes ---
void set_dcmi_frame_buffer(DCMI_HandleTypeDef *hdcmi, uint8_t *buffer, uint32_t width, uint32_t height);
uint8_t* get_dcmi_frame_buffer(DCMI_HandleTypeDef *hdcmi); // Added getter

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_DCMI_H */