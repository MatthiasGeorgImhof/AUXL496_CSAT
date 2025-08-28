#ifndef STM32_SPI_FLASH_ACCESSOR_H
#define STM32_SPI_FLASH_ACCESSOR_H

#include "stm32l4xx_hal.h"
#include <array>
#include <iostream>
#include <vector>
#include <algorithm>
#include "imagebuffer/accessor.hpp"

class STM32SPIFlashAccessor
{
public:
  STM32SPIFlashAccessor(SPI_HandleTypeDef *hspi, GPIO_TypeDef *nss_port, uint16_t nss_pin, size_t flash_start, size_t total_size)
      : hspi_(hspi), nss_port_(nss_port), nss_pin_(nss_pin), FLASH_START_ADDRESS(flash_start), TOTAL_BUFFER_SIZE(total_size) {} // Constructor takes SPI handle

  // Override the base class methods to use your STM32 HAL and SPI
  HAL_StatusTypeDef write(size_t address, const uint8_t *data, size_t size);
  HAL_StatusTypeDef read(size_t address, uint8_t *data, size_t size);
  HAL_StatusTypeDef erase(size_t address);

  size_t getAlignment() const { return 1; };
  size_t getFlashMemorySize() const { return TOTAL_BUFFER_SIZE; };
  size_t getFlashStartAddress() const { return FLASH_START_ADDRESS; };

private:
  SPI_HandleTypeDef *hspi_; // SPI handle
  GPIO_TypeDef *nss_port_;  // NSS GPIO Port
  uint16_t nss_pin_;        // NSS GPIO Pin
  const size_t FLASH_START_ADDRESS;
  const size_t TOTAL_BUFFER_SIZE;

  // Helper Functions (Implement these using your SPI flash chip's datasheet and commands)
  HAL_StatusTypeDef flash_sendCommand(uint8_t command);
  HAL_StatusTypeDef flash_sendAddress(size_t address);
  HAL_StatusTypeDef flash_writeEnable();
  HAL_StatusTypeDef flash_pageWrite(size_t address, const uint8_t *data, size_t size);
  HAL_StatusTypeDef flash_readData(size_t address, uint8_t *data, size_t size);
  HAL_StatusTypeDef flash_sectorErase(size_t address);

  void select();
  void deselect();
};

// Configuration (Replace with your values)
// SPI Flash Commands (Example - adjust to your specific chip)
const uint8_t WRITE_ENABLE_COMMAND = 0x06;
const uint8_t PAGE_PROGRAM_COMMAND = 0x02;
const uint8_t READ_DATA_COMMAND = 0x03;
const uint8_t SECTOR_ERASE_COMMAND = 0xD8; // Example
const uint32_t PAGE_SIZE = 256;            // Example Page Size

// Function to select the flash chip
inline void STM32SPIFlashAccessor::select()
{
  HAL_GPIO_WritePin(nss_port_, nss_pin_, GPIO_PIN_RESET); // Assert NSS
}

// Function to deselect the flash chip
inline void STM32SPIFlashAccessor::deselect()
{
  HAL_GPIO_WritePin(nss_port_, nss_pin_, GPIO_PIN_SET); // Deassert NSS
}

// Function to send a command to the flash chip
HAL_StatusTypeDef STM32SPIFlashAccessor::flash_sendCommand(uint8_t command)
{
  HAL_StatusTypeDef status;
  select();
  status = HAL_SPI_Transmit(hspi_, &command, 1, HAL_MAX_DELAY);
  deselect();
  return status;
}

// Function to send an address to the flash chip (24-bit address)
HAL_StatusTypeDef STM32SPIFlashAccessor::flash_sendAddress(size_t address)
{
  std::array<uint8_t, 3> addressBytes;      // Array to hold three address bytes
  addressBytes[0] = (address >> 16) & 0xFF; // Most significant byte
  addressBytes[1] = (address >> 8) & 0xFF;
  addressBytes[2] = address & 0xFF; // Least significant byte

  select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_, addressBytes.data(), 3, HAL_MAX_DELAY);
  deselect();

  return status;
}

// Function to write enable
HAL_StatusTypeDef STM32SPIFlashAccessor::flash_writeEnable()
{
  return flash_sendCommand(WRITE_ENABLE_COMMAND);
}

// Function to write a page to the flash memory
HAL_StatusTypeDef STM32SPIFlashAccessor::flash_pageWrite(size_t address, const uint8_t *data, size_t size)
{
  if (size > PAGE_SIZE)
  {
    std::cerr << "Error: Write size exceeds page size." << std::endl;
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status;
  std::vector<uint8_t> tx_buffer;
  // Write enable
  status = flash_writeEnable();
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_writeEnable failed" << std::endl;
    return status;
  }

  // Prepare the transmit buffer with command, address, and data
  tx_buffer.push_back(PAGE_PROGRAM_COMMAND);
  tx_buffer.push_back((address >> 16) & 0xFF);
  tx_buffer.push_back((address >> 8) & 0xFF);
  tx_buffer.push_back(address & 0xFF);

  // Add the data to the TX buffer
  tx_buffer.insert(tx_buffer.end(), data, data + size);

  // Select the chip and transmit
  select();

  status = HAL_SPI_Transmit(hspi_, tx_buffer.data(), tx_buffer.size(), HAL_MAX_DELAY);

  deselect();

  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_pageWrite failed" << std::endl;
  }

  // Wait for write to complete - implement some way to check status, polling the BUSY/RDY bit
  HAL_Delay(10); // Add a delay for the write operation to complete (check datasheet for timing) - this is a blocking delay, you might want to make this non-blocking.

  return status;
}

// Function to read data from the flash memory
HAL_StatusTypeDef STM32SPIFlashAccessor::flash_readData(size_t address, uint8_t *data, size_t size)
{
  HAL_StatusTypeDef status;
  std::vector<uint8_t> tx_buffer;
  tx_buffer.push_back(READ_DATA_COMMAND);
  tx_buffer.push_back((address >> 16) & 0xFF);
  tx_buffer.push_back((address >> 8) & 0xFF);
  tx_buffer.push_back(address & 0xFF);

  select();
  status = HAL_SPI_Transmit(hspi_, tx_buffer.data(), tx_buffer.size(), HAL_MAX_DELAY);
  if (status != HAL_OK)
  {
    deselect();
    std::cerr << "Error: flash_sendCommand failed" << std::endl;
    return status;
  }

  status = HAL_SPI_Receive(hspi_, data, size, HAL_MAX_DELAY);
  if (status != HAL_OK)
  {
    deselect();
    std::cerr << "Error: HAL_SPI_Receive failed" << std::endl;
    return status;
  }
  deselect();

  return status;
}

// Function to erase a sector
HAL_StatusTypeDef STM32SPIFlashAccessor::flash_sectorErase(size_t address)
{
  HAL_StatusTypeDef status;
  std::vector<uint8_t> tx_buffer;

  // Write enable
  status = flash_writeEnable();
  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_writeEnable failed" << std::endl;
    return status;
  }

  // Send sector Erase command
  tx_buffer.push_back(SECTOR_ERASE_COMMAND);
  tx_buffer.push_back((address >> 16) & 0xFF);
  tx_buffer.push_back((address >> 8) & 0xFF);
  tx_buffer.push_back(address & 0xFF);

  select();
  status = HAL_SPI_Transmit(hspi_, tx_buffer.data(), tx_buffer.size(), HAL_MAX_DELAY);

  deselect();

  if (status != HAL_OK)
  {
    std::cerr << "Error: flash_sendCommand failed" << std::endl;
    return status;
  }

  HAL_Delay(10); // Add a delay for the erase operation to complete
  return status;
}

HAL_StatusTypeDef STM32SPIFlashAccessor::write(size_t address, const uint8_t *data, size_t size)
{
  // This function needs to handle the page boundaries for the Flash Chip!
  HAL_StatusTypeDef status = HAL_OK;
  size_t currentAddress = address;
  const uint8_t *currentData = data;
  size_t bytesRemaining = size;

  while (bytesRemaining > 0)
  {
    // Calculate the number of bytes that can be written on the current page.
    size_t offset = currentAddress % PAGE_SIZE;
    size_t bytesToWrite = std::min((size_t)(PAGE_SIZE - offset), bytesRemaining);

    // Write the page
    status = flash_pageWrite(currentAddress, currentData, bytesToWrite);
    if (status != HAL_OK)
    {
      std::cerr << "Error: spi_flash_write - flash_pageWrite failed" << std::endl;
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
HAL_StatusTypeDef STM32SPIFlashAccessor::read(size_t address, uint8_t *data, size_t size)
{
  HAL_StatusTypeDef status = flash_readData(address, data, size);
  return status;
}

HAL_StatusTypeDef STM32SPIFlashAccessor::erase(size_t address)
{
  HAL_StatusTypeDef status = flash_sectorErase(address);
  return status;
}

static_assert(Accessor<BufferedAccessor>, "BufferedAccessor does not satisfy the Accessor concept!");

#endif /* STM32_SPI_FLASH_ACCESSOR_H */