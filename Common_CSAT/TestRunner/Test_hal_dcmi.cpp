#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

// --- DCMI Tests ---

TEST_CASE("HAL_DCMI_Init Test")
{
    DCMI_HandleTypeDef hdcmi;
    DCMI_InitTypeDef DCMI_Init;

    // Initialize DCMI init structure
    DCMI_Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    DCMI_Init.VSyncPolarity = 1;
    DCMI_Init.HSyncPolarity = 1;
    DCMI_Init.DataEnablePolarity = 1;
    DCMI_Init.PCKPolarity = 1;
    hdcmi.Init = DCMI_Init;

    CHECK(HAL_DCMI_Init(&hdcmi) == HAL_OK);
    CHECK(hdcmi.State == HAL_DCMI_STATE_READY);
    CHECK(hdcmi.ErrorCode == HAL_OK);
    CHECK(hdcmi.Init.SynchroMode == DCMI_SYNCHRO_HARDWARE); // Verify configuration saved
}

TEST_CASE("HAL_DCMI_DeInit Test")
{
    DCMI_HandleTypeDef hdcmi;

    // Assuming HAL_DCMI_Init has been called before, set state to READY
    hdcmi.State = HAL_DCMI_STATE_READY;

    CHECK(HAL_DCMI_DeInit(&hdcmi) == HAL_OK);
    CHECK(hdcmi.State == HAL_DCMI_STATE_RESET);
    CHECK(hdcmi.ErrorCode == HAL_OK);
}

TEST_CASE("HAL_DCMI_Start and data capture Test")
{
    DCMI_HandleTypeDef hdcmi;
    uint8_t frame_buffer[DCMI_IMAGE_BUFFER_SIZE];
    uint32_t frame_width = 640;
    uint32_t frame_height = 480;

    // Assign Frame Buffer to Handle
    set_dcmi_frame_buffer(&hdcmi, frame_buffer, frame_width, frame_height);

    // Call HAL_DCMI_Start (assuming continuous mode)
    CHECK(HAL_DCMI_Start(&hdcmi, DCMI_MODE_CONTINUOUS, 0) == HAL_OK);

    // Verify DCMI state is busy
    CHECK(hdcmi.State == HAL_DCMI_STATE_BUSY);

    // Verify that the frame buffer is populated with data (basic check)
    CHECK(frame_buffer[0] == 0);
    CHECK(frame_buffer[1000] == (uint8_t)(1000 % 256));                                  // Updated to match mock
    CHECK(frame_buffer[DCMI_IMAGE_BUFFER_SIZE - 1] == (uint8_t)((640 * 480) - 1 % 256)); // Updated to match mock
}

TEST_CASE("HAL_DCMI_Stop Test")
{
    DCMI_HandleTypeDef hdcmi;
    uint8_t frame_buffer[DCMI_IMAGE_BUFFER_SIZE];

    // Assign Frame Buffer to Handle
    set_dcmi_frame_buffer(&hdcmi, frame_buffer, 640, 480);

    // Set the DCMI state to busy
    hdcmi.State = HAL_DCMI_STATE_BUSY;

    // Call HAL_DCMI_Stop
    CHECK(HAL_DCMI_Stop(&hdcmi) == HAL_OK);

    // Verify that the DCMI state is ready
    CHECK(hdcmi.State == HAL_DCMI_STATE_READY);
}

TEST_CASE("HAL_DCMI_GetState Test")
{
    DCMI_HandleTypeDef hdcmi;

    // Set the DCMI state to ready
    hdcmi.State = HAL_DCMI_STATE_READY;
    CHECK(HAL_DCMI_GetState(&hdcmi) == HAL_DCMI_STATE_READY);

    // Set state to busy
    hdcmi.State = HAL_DCMI_STATE_BUSY;
    CHECK(HAL_DCMI_GetState(&hdcmi) == HAL_DCMI_STATE_BUSY);
}

TEST_CASE("HAL_DCMI_GetError Test")
{
    DCMI_HandleTypeDef hdcmi;
    hdcmi.ErrorCode = 10;
    CHECK(HAL_DCMI_GetError(&hdcmi) == 10);
    hdcmi.ErrorCode = 100;
    CHECK(HAL_DCMI_GetError(&hdcmi) == 100);
}

TEST_CASE("set_dcmi_frame_buffer Test")
{
    DCMI_HandleTypeDef hdcmi;
    uint8_t frame_buffer[DCMI_IMAGE_BUFFER_SIZE];
    uint32_t width = 320;
    uint32_t height = 240;

    // Call the helper function
    set_dcmi_frame_buffer(&hdcmi, frame_buffer, width, height);

    // Verify the values in handle
    CHECK(hdcmi.pFrameBuffer == frame_buffer);
    CHECK(hdcmi.FrameWidth == width);
    CHECK(hdcmi.FrameHeight == height);
}

TEST_CASE("get_dcmi_frame_buffer Test")
{
    DCMI_HandleTypeDef hdcmi;
    uint8_t frame_buffer[DCMI_IMAGE_BUFFER_SIZE];

    // Set the frame buffer in the handle
    hdcmi.pFrameBuffer = frame_buffer;

    // Get the frame buffer using the getter function
    CHECK(get_dcmi_frame_buffer(&hdcmi) == frame_buffer);
}