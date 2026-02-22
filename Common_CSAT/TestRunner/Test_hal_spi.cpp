#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

//--- SPI Tests ---

TEST_CASE("HAL_SPI_Init Test")
{
    SPI_HandleTypeDef hspi;
    init_spi_handle(&hspi); // Initialize SPI handle

    CHECK(HAL_SPI_Init(&hspi) == HAL_OK);
    // Add more checks here if you want to verify values within the SPI_InitTypeDef
}

TEST_CASE("HAL_SPI_Transmit Test")
{
    SPI_HandleTypeDef hspi;
    uint8_t tx_data[] = "SPI test";
    init_spi_handle(&hspi);

    CHECK(HAL_SPI_Transmit(&hspi, tx_data, sizeof(tx_data) - 1, 100) == HAL_OK);
    CHECK(get_spi_tx_buffer_count() == sizeof(tx_data) - 1);
    CHECK(memcmp(get_spi_tx_buffer(), tx_data, sizeof(tx_data) - 1) == 0);

    clear_spi_tx_buffer();
    CHECK(get_spi_tx_buffer_count() == 0);
}

TEST_CASE("HAL_SPI_Receive Test")
{
    SPI_HandleTypeDef hspi;
    init_spi_handle(&hspi);
    uint8_t expected_rx_data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rx_data[4];

    inject_spi_rx_data(expected_rx_data, sizeof(expected_rx_data));

    CHECK(HAL_SPI_Receive(&hspi, rx_data, sizeof(rx_data), 100) == HAL_OK);
    CHECK(memcmp(rx_data, expected_rx_data, sizeof(rx_data)) == 0);

    clear_spi_rx_buffer();
    CHECK(get_spi_rx_buffer_count() == 0);
}

TEST_CASE("HAL_SPI_TransmitReceive Test")
{
    SPI_HandleTypeDef hspi;
    init_spi_handle(&hspi);
    uint8_t tx_data[] = "TxData";
    uint8_t expected_rx_data[] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
    uint8_t rx_data[6];

    inject_spi_rx_data(expected_rx_data, sizeof(expected_rx_data));

    CHECK(HAL_SPI_TransmitReceive(&hspi, tx_data, rx_data, sizeof(tx_data) - 1, 100) == HAL_OK);

    CHECK(get_spi_tx_buffer_count() == sizeof(tx_data) - 1);
    CHECK(memcmp(get_spi_tx_buffer(), tx_data, sizeof(tx_data) - 1) == 0);
    CHECK(memcmp(rx_data, expected_rx_data, sizeof(tx_data) - 1) == 0);

    clear_spi_tx_buffer();
    clear_spi_rx_buffer();
    CHECK(get_spi_tx_buffer_count() == 0);
    CHECK(get_spi_rx_buffer_count() == 0);
}

TEST_CASE("HAL_SPI_TransmitReceive - Size greater than RX")
{
    SPI_HandleTypeDef hspi;
    init_spi_handle(&hspi);
    uint8_t tx_data[] = "TxData";
    uint8_t expected_rx_data[] = {0x10, 0x20, 0x30, 0x40};
    uint8_t rx_data[6];

    inject_spi_rx_data(expected_rx_data, sizeof(expected_rx_data));

    CHECK(HAL_SPI_TransmitReceive(&hspi, tx_data, rx_data, 6, 100) == HAL_ERROR);

    clear_spi_tx_buffer();
    clear_spi_rx_buffer();
    CHECK(get_spi_tx_buffer_count() == 0);
    CHECK(get_spi_rx_buffer_count() == 0);
}
