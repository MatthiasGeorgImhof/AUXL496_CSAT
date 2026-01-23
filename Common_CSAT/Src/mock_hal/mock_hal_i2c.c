#ifdef __x86_64__

#include "mock_hal/mock_hal_i2c.h"
#include <string.h>

//--- I2C Buffers ---
uint8_t i2c_rx_buffer[I2C_MEM_BUFFER_SIZE]; // I2C RX buffer
uint16_t i2c_rx_buffer_count = 0;           // Number of bytes in I2C RX buffer

uint8_t i2c_tx_buffer[I2C_MEM_BUFFER_SIZE]; // I2C TX buffer
uint16_t i2c_tx_buffer_count = 0;           // Number of bytes in I2C TX buffer

uint16_t i2c_mem_buffer_dev_address; // I2C device address
uint16_t i2c_mem_buffer_mem_address; // I2C memory address

uint32_t HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t /*Timeout*/)
{
    if (hi2c == NULL || pData == NULL)
    {
        return 1; // HAL_ERROR; // Check for NULL pointers
    }
    if (Size > I2C_MEM_BUFFER_SIZE)
    {
        return 1; // HAL_ERROR; // Check for size limits
    }

    // Store the parameters for later verification
    i2c_mem_buffer_dev_address = DevAddress;
    i2c_mem_buffer_mem_address = 0; // Not a memory transfer

    i2c_tx_buffer_count = Size;
    memcpy(i2c_tx_buffer, pData, Size);
    return 0;
}

uint32_t HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t /*Timeout*/)
{
    if (hi2c == NULL || pData == NULL)
        return 1;
    if (i2c_rx_buffer_count < Size)
        return 1;
    if (i2c_mem_buffer_dev_address != DevAddress)
        return 1;

    memcpy(pData, i2c_rx_buffer, Size);
    return 0;
}

uint32_t HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t /*MemAddSize*/, uint8_t *pData, uint16_t Size, uint32_t /*Timeout*/)
{
    if (hi2c == NULL || pData == NULL)
    {
        return 1; // HAL_ERROR;
    }

    // if (i2c_mem_buffer_dev_address != DevAddress)
    // {
    //     return 1; // HAL_ERROR, invalid address
    // }

    if (i2c_rx_buffer_count < Size)
    {
        return 1; // HAL_ERROR, invalid address
    }

    // Store the parameters for later verification
    i2c_mem_buffer_dev_address = DevAddress;
    i2c_mem_buffer_mem_address = MemAddress;

    memcpy(pData, i2c_rx_buffer, Size);
    return 0;
}

uint32_t HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t /*MemAddSize*/, uint8_t *pData, uint16_t Size, uint32_t /*Timeout*/)
{
    if (hi2c == NULL || pData == NULL)
    {
        return 1; // HAL_ERROR;
    }

    // Store the parameters for later verification
    i2c_mem_buffer_dev_address = DevAddress;
    i2c_mem_buffer_mem_address = MemAddress;

    i2c_tx_buffer_count = Size;
    memcpy(i2c_tx_buffer, pData, Size);
    return 0;
}

// I2C Injectors
void inject_i2c_tx_data(uint16_t DevAddress, const uint8_t *data, uint16_t size)
{
    i2c_mem_buffer_dev_address = DevAddress;
    memcpy(i2c_tx_buffer, data, size);
    i2c_tx_buffer_count = size;
}

void inject_i2c_rx_data(uint16_t DevAddress, const uint8_t *data, uint16_t size)
{
    i2c_mem_buffer_dev_address = DevAddress;
    memcpy(i2c_rx_buffer, data, size);
    i2c_rx_buffer_count = size;
}

// I2C Deleters
void clear_i2c_tx_data()
{
    memset(i2c_tx_buffer, 0, sizeof(i2c_tx_buffer)); // Set all elements to 0
    i2c_tx_buffer_count = 0;
}

void clear_i2c_rx_data()
{
    memset(i2c_rx_buffer, 0, sizeof(i2c_rx_buffer)); // Set all elements to 0
    i2c_rx_buffer_count = 0;
}

void clear_i2c_addresses()
{
    i2c_mem_buffer_dev_address = 0;
    i2c_mem_buffer_mem_address = 0;
}

int get_i2c_rx_buffer_count()
{
    return i2c_rx_buffer_count;
}

uint8_t *get_i2c_tx_buffer()
{
    return i2c_tx_buffer;
}

int get_i2c_tx_buffer_count()
{
    return i2c_tx_buffer_count;
}

uint8_t *get_i2c_rx_buffer()
{
    return i2c_rx_buffer;
}

uint16_t get_i2c_dev_address()
{
    return i2c_mem_buffer_dev_address;
}

uint16_t get_i2c_mem_address()
{
    return i2c_mem_buffer_mem_address;
}

#endif