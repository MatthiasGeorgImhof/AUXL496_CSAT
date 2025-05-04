//#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

TEST_CASE("HAL_I2C_Master_Transmit")
{
    I2C_HandleTypeDef hi2c;
    uint8_t data[] = {0x12, 0x34, 0x56};
    CHECK(HAL_I2C_Master_Transmit(&hi2c, 0x50, data, 3, 100) == HAL_OK);
}

TEST_CASE("HAL_I2C_Mem_Read success")
{
    I2C_HandleTypeDef hi2c;
    uint8_t expected_data[] = {0xAA, 0xBB, 0xCC};
    uint8_t read_data[3];
    inject_i2c_mem_data(0x50, 0x10, expected_data, 3);

    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x50, 0x10, 1, read_data, 3, 100) == HAL_OK);
    CHECK(memcmp(read_data, expected_data, 3) == 0);
    clear_i2c_mem_data();
}

TEST_CASE("HAL_I2C_Mem_Read Fail invalid address")
{
    I2C_HandleTypeDef hi2c;
    uint8_t expected_data[] = {0xAA, 0xBB, 0xCC};
    uint8_t read_data[3];
    inject_i2c_mem_data(0x50, 0x10, expected_data, 3);

    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x51, 0x10, 1, read_data, 3, 100) == HAL_ERROR);
    clear_i2c_mem_data();
}

TEST_CASE("HAL_I2C_Mem_Read Different size")
{
    I2C_HandleTypeDef hi2c;
    uint8_t expected_data[] = {0xAA, 0xBB, 0xCC};
    uint8_t read_data[4];
    inject_i2c_mem_data(0x50, 0x10, expected_data, 3);

    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x50, 0x10, 1, read_data, 4, 100) == HAL_OK);
    clear_i2c_mem_data();
}

TEST_CASE("HAL_I2C_Mem_Write")
{
    I2C_HandleTypeDef hi2c;
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    CHECK(HAL_I2C_Mem_Write(&hi2c, 0x50, 0x20, 1, data, 4, 100) == HAL_OK);
}
