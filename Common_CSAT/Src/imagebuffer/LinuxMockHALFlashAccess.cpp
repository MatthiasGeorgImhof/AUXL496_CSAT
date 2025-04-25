#include "imagebuffer/LinuxMockHALFlashAccess.hpp"
#include <iostream>
#include <cstring>
#include "mock_hal.h"

LinuxMockHALFlashAccess::LinuxMockHALFlashAccess() {}

int32_t LinuxMockHALFlashAccess::hal_i2c_master_transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    // Call the mocked HAL function (remember the extern "C" in mock_hal.h)
    return HAL_I2C_Master_Transmit(nullptr, DevAddress, pData, Size, Timeout);
}

int32_t LinuxMockHALFlashAccess::hal_i2c_mem_read(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    // Call the mocked HAL function (remember the extern "C" in mock_hal.h)
    return HAL_I2C_Mem_Read(nullptr, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}

int32_t LinuxMockHALFlashAccess::hal_i2c_mem_write(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    // Call the mocked HAL function (remember the extern "C" in mock_hal.h)
    return HAL_I2C_Mem_Write(nullptr, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}

int32_t LinuxMockHALFlashAccess::write(uint32_t /*address*/, const uint8_t* data, size_t size) {
    // Now, implement the I2C write sequence using the HAL mocks. This needs to match
    // what your STM32 code does. For example:
   

    if (size > 256) {
        std::cerr << "Size exceeded" << std::endl;
        return -1;
    }
          HAL_StatusTypeDef status = (HAL_StatusTypeDef)hal_i2c_master_transmit(0xA0, (uint8_t*)data, size, 100); //Dummy Params
    if(status == HAL_OK)
      return 0;
    else
      return 1;
}

int32_t LinuxMockHALFlashAccess::read(uint32_t address, uint8_t* data, size_t size) {
    uint16_t mockDevAddress = 0xA0;
    uint16_t mockMemAddress = static_cast<uint16_t>(address);
    uint16_t mockMemAddSize = 0;
    uint32_t mockTimeout = 100;

  //Implement a memory region here
    if(address > 0x100) {
      return 1;
    }
    // Implement the flash read sequence using the HAL mocks
    
        if (size > 256) {
            std::cerr << "Size exceeded" << std::endl;
            return -1;
        }
          HAL_StatusTypeDef status = (HAL_StatusTypeDef)hal_i2c_mem_read(mockDevAddress, mockMemAddress, mockMemAddSize, (uint8_t*)data, static_cast<uint16_t>(size), mockTimeout); //Dummy Params
        if(status == HAL_OK)
          return 0;
        else
          return 1;
}

int32_t LinuxMockHALFlashAccess::erase(uint32_t /*address*/) {
     // We will shut up a clang flag
    
    // Implement the flash erase sequence using the HAL mocks
    return 0; // Success
}