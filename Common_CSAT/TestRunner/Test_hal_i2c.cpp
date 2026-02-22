#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>

// ------------------------------------------------------------
// Master Transmit
// ------------------------------------------------------------
TEST_CASE("HAL_I2C_Master_Transmit writes into TX buffer")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t data[] = {0x12, 0x34, 0x56};

    CHECK(HAL_I2C_Master_Transmit(&hi2c, 0x50, data, 3, 100) == HAL_OK);

    CHECK(get_i2c_dev_address() == 0x50);
    CHECK(get_i2c_tx_buffer_count() == 3);
    CHECK(memcmp(get_i2c_tx_buffer(), data, 3) == 0);
}

TEST_CASE("HAL_I2C_Master_Transmit fails on null pointers")
{
    I2C_HandleTypeDef hi2c;
    uint8_t data[1] = {0x00};

    CHECK(HAL_I2C_Master_Transmit(nullptr, 0x50, data, 1, 100) == HAL_ERROR);
    CHECK(HAL_I2C_Master_Transmit(&hi2c, 0x50, nullptr, 1, 100) == HAL_ERROR);
}

TEST_CASE("HAL_I2C_Master_Transmit fails when Size exceeds buffer")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t big[I2C_MEM_BUFFER_SIZE + 1] = {};

    CHECK(HAL_I2C_Master_Transmit(&hi2c, 0x50, big,
                                  I2C_MEM_BUFFER_SIZE + 1, 100) == HAL_ERROR);
}

// ------------------------------------------------------------
// Mem_Read
// ------------------------------------------------------------
TEST_CASE("HAL_I2C_Mem_Read success")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t expected[] = {0xAA, 0xBB, 0xCC};
    uint8_t out[3] = {};

    inject_i2c_rx_data(0x50, expected, 3);

    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x50, 0x1234, 2, out, 3, 100) == HAL_OK);
    CHECK(memcmp(out, expected, 3) == 0);

    CHECK(get_i2c_dev_address() == 0x50);
    CHECK(get_i2c_mem_address() == 0x1234);
}

// they do not fail on wrong DevAddress anymore due to removed check
// TEST_CASE("HAL_I2C_Mem_Read fails with wrong DevAddress")
// {
//     clear_i2c_rx_data();
//     clear_i2c_addresses();

//     I2C_HandleTypeDef hi2c;
//     uint8_t expected[] = {0xAA, 0xBB};
//     uint8_t out[2];

//     inject_i2c_rx_data(0x50, expected, 2);

//     CHECK(HAL_I2C_Mem_Read(&hi2c, 0x51, 0x10, 1, out, 2, 100) == HAL_ERROR);
// }

TEST_CASE("HAL_I2C_Mem_Read fails when Size > injected RX size")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t expected[] = {0xAA};
    uint8_t out[2];

    inject_i2c_rx_data(0x50, expected, 1);

    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x50, 0x10, 1, out, 2, 100) == HAL_ERROR);
}

TEST_CASE("HAL_I2C_Mem_Read fails on null pointers")
{
    I2C_HandleTypeDef hi2c;
    uint8_t buf[1];

    CHECK(HAL_I2C_Mem_Read(nullptr, 0x50, 0x10, 1, buf, 1, 100) == HAL_ERROR);
    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x50, 0x10, 1, nullptr, 1, 100) == HAL_ERROR);
}

// ------------------------------------------------------------
// Mem_Write
// ------------------------------------------------------------
TEST_CASE("HAL_I2C_Mem_Write writes into TX buffer and tracks addresses")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};

    CHECK(HAL_I2C_Mem_Write(&hi2c, 0x50, 0x2222, 2, data, 4, 100) == HAL_OK);

    CHECK(get_i2c_dev_address() == 0x50);
    CHECK(get_i2c_mem_address() == 0x2222);
    CHECK(get_i2c_tx_buffer_count() == 4);
    CHECK(memcmp(get_i2c_tx_buffer(), data, 4) == 0);
}

TEST_CASE("HAL_I2C_Mem_Write fails on null pointers")
{
    I2C_HandleTypeDef hi2c;
    uint8_t data[1] = {0x00};

    CHECK(HAL_I2C_Mem_Write(nullptr, 0x50, 0x10, 1, data, 1, 100) == HAL_ERROR);
    CHECK(HAL_I2C_Mem_Write(&hi2c, 0x50, 0x10, 1, nullptr, 1, 100) == HAL_ERROR);
}

// ------------------------------------------------------------
// Master Receive
// ------------------------------------------------------------
TEST_CASE("HAL_I2C_Master_Receive success")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t expected[] = {0xAA, 0xBB};
    uint8_t out[2];

    inject_i2c_rx_data(0x50, expected, 2);

    CHECK(HAL_I2C_Master_Receive(&hi2c, 0x50, out, 2, 100) == HAL_OK);
    CHECK(memcmp(out, expected, 2) == 0);
}

TEST_CASE("HAL_I2C_Master_Receive fails with wrong DevAddress")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t expected[] = {0xAA, 0xBB};
    uint8_t out[2];

    inject_i2c_rx_data(0x50, expected, 2);

    CHECK(HAL_I2C_Master_Receive(&hi2c, 0x51, out, 2, 100) == HAL_ERROR);
}

TEST_CASE("HAL_I2C_Master_Receive fails when Size > injected RX size")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    I2C_HandleTypeDef hi2c;
    uint8_t expected[] = {0xAA};
    uint8_t out[2];

    inject_i2c_rx_data(0x50, expected, 1);

    CHECK(HAL_I2C_Master_Receive(&hi2c, 0x50, out, 2, 100) == HAL_ERROR);
}

TEST_CASE("HAL_I2C_Master_Receive fails on null pointers")
{
    I2C_HandleTypeDef hi2c;
    uint8_t buf[1];

    CHECK(HAL_I2C_Master_Receive(nullptr, 0x50, buf, 1, 100) == HAL_ERROR);
    CHECK(HAL_I2C_Master_Receive(&hi2c, 0x50, nullptr, 1, 100) == HAL_ERROR);
}

// ------------------------------------------------------------
// Helper Function Tests
// ------------------------------------------------------------
TEST_CASE("inject_i2c_tx_data and clear helpers behave as expected")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    uint8_t data[] = {0x11, 0x22};
    inject_i2c_tx_data(0x60, data, 2);

    CHECK(get_i2c_dev_address() == 0x60);
    CHECK(get_i2c_tx_buffer_count() == 2);
    CHECK(memcmp(get_i2c_tx_buffer(), data, 2) == 0);

    clear_i2c_tx_data();
    CHECK(get_i2c_tx_buffer_count() == 0);

    clear_i2c_addresses();
    CHECK(get_i2c_dev_address() == 0);
    CHECK(get_i2c_mem_address() == 0);
}

TEST_CASE("inject_i2c_rx_data and clear helpers behave as expected")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    uint8_t data[] = {0x33, 0x44, 0x55};
    inject_i2c_rx_data(0x70, data, 3);

    CHECK(get_i2c_dev_address() == 0x70);
    CHECK(get_i2c_rx_buffer_count() == 3);
    CHECK(memcmp(get_i2c_rx_buffer(), data, 3) == 0);

    clear_i2c_rx_data();
    CHECK(get_i2c_rx_buffer_count() == 0);
}
