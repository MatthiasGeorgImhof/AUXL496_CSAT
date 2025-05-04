#ifdef __x86_64__

#include "mock_hal/mock_hal_dcmi.h"
#include <string.h>

//--- DCMI Buffers ---
uint8_t dcmi_image_buffer[DCMI_IMAGE_BUFFER_SIZE]; // DCMI Image buffer

HAL_StatusTypeDef HAL_DCMI_Init(DCMI_HandleTypeDef *hdcmi) {
    // Mock the initialization process.
    if (hdcmi == NULL) return HAL_ERROR;

    // You might want to store the hdcmi->Init values for later assertions.
    hdcmi->State = HAL_DCMI_STATE_READY;
    hdcmi->ErrorCode = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DCMI_DeInit(DCMI_HandleTypeDef *hdcmi) {
    if (hdcmi == NULL) return HAL_ERROR;

    hdcmi->State = HAL_DCMI_STATE_RESET;
    hdcmi->ErrorCode = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DCMI_Start(DCMI_HandleTypeDef *hdcmi, uint32_t /*Mode*/, uint32_t /*DMA_InitStruct*/) {
    if (hdcmi == NULL || hdcmi->pFrameBuffer == NULL) return HAL_ERROR;

    hdcmi->State = HAL_DCMI_STATE_BUSY;

    // Simulate data capture (basic example)
    for (uint32_t i = 0; i < (hdcmi->FrameWidth*hdcmi->FrameHeight); i++) {
        hdcmi->pFrameBuffer[i] = (uint8_t)i;  // Simple repeating pattern
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_DCMI_Stop(DCMI_HandleTypeDef *hdcmi) {
    if (hdcmi == NULL) return HAL_ERROR;

    hdcmi->State = HAL_DCMI_STATE_READY;
    return HAL_OK;
}

HAL_DCMI_StateTypeDef HAL_DCMI_GetState(DCMI_HandleTypeDef *hdcmi){
    if(hdcmi == NULL) return HAL_DCMI_STATE_ERROR;

    return hdcmi->State;
}

uint32_t HAL_DCMI_GetError(DCMI_HandleTypeDef *hdcmi){
    if(hdcmi == NULL) return 1; //HAL_ERROR

    return hdcmi->ErrorCode;
}

// Set the DCMI frame buffer and dimensions for testing
void set_dcmi_frame_buffer(DCMI_HandleTypeDef *hdcmi, uint8_t *buffer, uint32_t width, uint32_t height) {
    if (hdcmi != NULL) {
        hdcmi->pFrameBuffer = buffer;
        hdcmi->FrameWidth = width;
        hdcmi->FrameHeight = height;
    }
}

// Get the DCMI frame buffer for testing
uint8_t* get_dcmi_frame_buffer(DCMI_HandleTypeDef *hdcmi) {
    if (hdcmi != NULL) {
        return hdcmi->pFrameBuffer;
    }
    return NULL;
}

#endif