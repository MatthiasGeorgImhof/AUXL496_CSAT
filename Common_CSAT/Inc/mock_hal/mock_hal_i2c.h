#ifndef MOCK_HAL_I2C_H
#define MOCK_HAL_I2C_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

// Include core definitions
#include "mock_hal/mock_hal_core.h"

#define MOCK_HAL_I2C_ENABLED

//--- I2C Defines ---
#define I2C_MEMADD_SIZE_8BIT (0x00000001U)  // I2C Memory address size: 8-bit
#define I2C_MEMADD_SIZE_16BIT (0x00000002U) // I2C Memory address size: 16-bit

    //--- I2C Structures ---
    typedef struct
    {
        uint32_t ClockSpeed;      // Specifies the clock speed
        uint32_t DutyCycle;       // Specifies the duty cycle
        uint32_t OwnAddress1;     // Specifies the own address 1
        uint32_t AddressingMode;  // Specifies the addressing mode
        uint32_t DualAddressMode; // Specifies the dual addressing mode
        uint32_t OwnAddress2;     // Specifies the own address 2
        uint32_t GeneralCallMode; // Specifies the general call mode
        uint32_t NoStretchMode;   // Specifies the no stretch mode
        uint32_t Master;          // Master flag
        uint32_t Init;            // Init flag
    } I2C_InitTypeDef;

    typedef struct
    {
        I2C_InitTypeDef Instance; // I2C initialization structure
    } I2C_HandleTypeDef;

    //--- I2C Mock Function Prototypes ---
    uint32_t HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);

    //--- I2C Helper Function Prototypes ---
    void inject_i2c_rx_data(uint16_t DevAddress, const uint8_t *data, uint16_t size);
    void inject_i2c_tx_data(uint16_t DevAddress, const uint8_t *data, uint16_t size);

    void clear_i2c_rx_data();
    void clear_i2c_tx_data();
    void clear_i2c_addresses();

    // I2C Getters
    uint16_t get_i2c_dev_address();
    uint16_t get_i2c_mem_address();
    int get_i2c_rx_buffer_count();
    uint8_t *get_i2c_rx_buffer();
    int get_i2c_tx_buffer_count();
    uint8_t *get_i2c_tx_buffer();

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_I2C_H */