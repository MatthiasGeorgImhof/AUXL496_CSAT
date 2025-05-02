#ifndef STM32_I2C_FLASH_ACCESSOR_H
#define STM32_I2C_FLASH_ACCESSOR_H

#include "stm32l4xx_hal.h"
#include <array>
#include <iostream>
#include "imagebuffer/accessor.hpp"

class STM32I2CFlashAccessor
{
public:
  STM32I2CFlashAccessor(I2C_HandleTypeDef *hi2c, size_t flash_start, size_t total_size) : hi2c_(hi2c), FLASH_START_ADDRESS(flash_start), TOTAL_BUFFER_SIZE(total_size) {} // Constructor takes I2C handle

  // Override the base class methods to use your STM32 HAL and I2C
  AccessorError write(uint32_t address, const uint8_t *data, size_t size);
  AccessorError read(uint32_t address, uint8_t *data, size_t size);
  AccessorError erase(uint32_t address);

  size_t getAlignment() const { return 1; };
  size_t getFlashMemorySize() const { return TOTAL_BUFFER_SIZE; };
  size_t getFlashStartAddress() const { return FLASH_START_ADDRESS; };

private:
  I2C_HandleTypeDef *hi2c_; // I2C handle
  const size_t FLASH_START_ADDRESS;
  const size_t TOTAL_BUFFER_SIZE;

  // Helper Functions (Implement these using your I2C flash chip's datasheet and commands)
  int32_t flash_sendCommand(uint8_t command);
  int32_t flash_sendAddress(uint32_t address);
  int32_t flash_writeEnable();
  int32_t flash_pageWrite(uint32_t address, const uint8_t *data, size_t size);
  int32_t flash_readData(uint32_t address, uint8_t *data, size_t size);
  int32_t flash_sectorErase(uint32_t address);
};

// Configuration (Replace with your values)
const uint8_t FLASH_I2C_ADDRESS = 0x50 << 1; // Example: 7-bit address 0x50, shifted left for write
const uint8_t WRITE_ENABLE_COMMAND = 0x06;
const uint8_t PAGE_PROGRAM_COMMAND = 0x02;
const uint8_t READ_DATA_COMMAND = 0x03;
const uint8_t SECTOR_ERASE_COMMAND = 0xD8; // Example
const uint32_t PAGE_SIZE = 256;            // Example Page Size

// Function to send a command to the flash chip
int32_t STM32I2CFlashAccessor::flash_sendCommand(uint8_t command)
{
  return HAL_I2C_Master_Transmit(hi2c_, FLASH_I2C_ADDRESS, &command, 1, HAL_MAX_DELAY);
}

// Function to send an address to the flash chip (24-bit address)
AccessorError STM32I2CFlashAccessor::flash_sendAddress(uint32_t address)
{
  std::array<uint8_t, 3> addressBytes;      // Array to hold three address bytes
  addressBytes[0] = (address >> 16) & 0xFF; // Most significant byte
  addressBytes[1] = (address >> 8) & 0xFF;
  addressBytes[2] = address & 0xFF; // Least significant byte
  return HAL_I2C_Master_Transmit(hi2c_, FLASH_I2C_ADDRESS, addressBytes.data(), 3, HAL_MAX_DELAY);
}

// Function to write enable
AccessorError STM32I2CFlashAccessor::flash_writeEnable()
{
  return flash_sendCommand(WRITE_ENABLE_COMMAND);
}

// Function to write a page to the flash memory
AccessorError STM32I2CFlashAccessor::flash_pageWrite(uint32_t address, const uint8_t *data, size_t size)
{

  if (size > PAGE_SIZE)
  {
    std::cerr << "Error: Write size exceeds page size." << std::endl;
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status;

  // Write enable
  status = flash_writeEnable();
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_writeEnable failed" << std::endl;
    return status;
  }

  // Buffer to store the full message (command byte, address bytes and data)
  std::vector<uint8_t> fullMessage;
  fullMessage.push_back(PAGE_PROGRAM_COMMAND);   // Add page program command
  fullMessage.push_back((address >> 16) & 0xFF); // Address MSB
  fullMessage.push_back((address >> 8) & 0xFF);
  fullMessage.push_back(address & 0xFF);                    // Address LSB
  fullMessage.insert(fullMessage.end(), data, data + size); // Add the image data

  // Transmit to Flash
  status = HAL_I2C_Master_Transmit(hi2c_, FLASH_I2C_ADDRESS, fullMessage.data(), fullMessage.size(), HAL_MAX_DELAY);

  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_pageWrite failed" << std::endl;
  }
  // Wait for write to complete
  HAL_Delay(10); // Add a delay for the write operation to complete (check datasheet for timing) - this is a blocking delay, you might want to make this non-blocking.

  return status;
}

// Function to read data from the flash memory
int32_t STM32I2CFlashAccessor::flash_readData(uint32_t address, uint8_t *data, size_t size)
{
  HAL_StatusTypeDef status;

  // Send Read Data command
  status = flash_sendCommand(READ_DATA_COMMAND);
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_sendCommand failed" << std::endl;
    return status;
  }

  // Send the address
  status = flash_sendAddress(address);
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_sendAddress failed" << std::endl;
    return status;
  }
  // Receive the data
  status = HAL_I2C_Master_Receive(hi2c_, FLASH_I2C_ADDRESS, data, size, HAL_MAX_DELAY);
  if (status != HAL_OK)
  {
    std::cerr << "Error: HAL_I2C_Master_Receive failed" << std::endl;
    return status;
  }
  return status;
}

// Function to erase a sector
int32_t STM32I2CFlashAccessor::flash_sectorErase(uint32_t address)
{
  HAL_StatusTypeDef status;

  // Write enable
  status = flash_writeEnable();
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_writeEnable failed" << std::endl;
    return status;
  }
  // Send sector Erase command
  status = flash_sendCommand(SECTOR_ERASE_COMMAND);
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_sendCommand failed" << std::endl;
    return status;
  }

  // Send the address
  status = flash_sendAddress(address);
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_sendAddress failed" << std::endl;
    return status;
  }
  HAL_Delay(10); // Add a delay for the erase operation to complete
  return status;
}

int32_t STM32I2CFlashAccessor::write(uint32_t address, const uint8_t *data, size_t size)
{
  // This function need to handle the page boundaries for the Flash Chip!
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t currentAddress = address;
  const uint8_t *currentData = data;
  size_t bytesRemaining = size;

  while (bytesRemaining > 0)
  {
    // Calculate the number of bytes that can be written on the current page.
    uint32_t offset = currentAddress % PAGE_SIZE;
    size_t bytesToWrite = std::min((size_t)(PAGE_SIZE - offset), bytesRemaining);

    // Write the page
    status = flash_pageWrite(currentAddress, currentData, bytesToWrite);
    if (status != HAL_OK)
    {
      std::cerr << "Error: i2c_flash_write - flash_pageWrite failed" << std::endl;
      return status; // Or handle the error as needed
    }

    // Update counters
    currentAddress += bytesToWrite;
    currentData += bytesToWrite;
    bytesRemaining -= bytesToWrite;
  }

  return status;
}

// flash_read implementation
int32_t STM32I2CFlashAccessor::read(uint32_t address, uint8_t *data, size_t size)
{
  HAL_StatusTypeDef status = flash_readData(address, data, size);
  return status;
}

int32_t STM32I2CFlashAccessor::erase(uint32_t address)
{
  HAL_StatusTypeDef status = flash_sectorErase(address);
  return status;
}

static_assert(Accessor<STM32I2CFlashAccessor>, "STM32I2CFlashAccessor does not satisfy the Accessor concept!");

#endif /* STM32_I2C_FLASH_ACCESSOR_H */