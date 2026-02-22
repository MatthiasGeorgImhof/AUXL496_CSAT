#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

TEST_CASE("HAL_UART_Transmit")
{
    UART_HandleTypeDef huart;
    uint8_t data[] = "test data";
    init_uart_handle(&huart);
    CHECK(HAL_UART_Transmit(&huart, data, sizeof(data) - 1, 1000) == HAL_OK);
    CHECK(get_uart_tx_buffer_count() == 9);
    CHECK(memcmp(get_uart_tx_buffer(), data, sizeof(data) - 1) == 0);
    clear_uart_tx_buffer();
    CHECK(get_uart_tx_buffer_count() == 0);
}

TEST_CASE("HAL_UART_Transmit_DMA")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t data[] = "test data";
    CHECK(HAL_UART_Transmit_DMA(&huart, data, sizeof(data) - 1) == HAL_OK);
    CHECK(get_uart_tx_buffer_count() == 9);
    CHECK(memcmp(get_uart_tx_buffer(), data, sizeof(data) - 1) == 0);
    clear_uart_tx_buffer();
    CHECK(get_uart_tx_buffer_count() == 0);
}

TEST_CASE("HAL_UART_Receive timeout")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where only some bytes are received before timeout
    inject_uart_rx_data(expected_data, 2); // Only Injecting 2 bytes
    set_current_tick(0);                   // set mock time to 0

    CHECK(HAL_UART_Receive(&huart, recv_buffer, 5, 100) == HAL_ERROR); // we should get timeout
    CHECK(memcmp(recv_buffer, expected_data, 2) == 0);                 // Check the 2 bytes we received
    clear_uart_rx_buffer();
}

TEST_CASE("HAL_UART_Receive no timeout")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where all bytes are received
    inject_uart_rx_data(expected_data, 5); // Inject all bytes
    set_current_tick(0);                   // set mock time to 0

    CHECK(HAL_UART_Receive(&huart, recv_buffer, 5, 100) == HAL_OK); // we should not get timeout
    CHECK(memcmp(recv_buffer, expected_data, 5) == 0);              // Check the 5 bytes we received
    clear_uart_rx_buffer();
}

TEST_CASE("HAL_UART_Receive_DMA no timeout")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where all bytes are received
    inject_uart_rx_data(expected_data, 5); // Inject all bytes
    set_current_tick(0);
    CHECK(HAL_UART_Receive_DMA(&huart, recv_buffer, 5) == HAL_OK); // we should not get timeout
    CHECK(memcmp(recv_buffer, expected_data, 5) == 0);             // Check the 5 bytes we received
    clear_uart_rx_buffer();
}

TEST_CASE("HAL_UART_Receive_DMA partial")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where all bytes are received
    inject_uart_rx_data(expected_data, 2); // Inject 2 bytes
    set_current_tick(0);
    CHECK(HAL_UART_Receive_DMA(&huart, recv_buffer, 5) == HAL_ERROR); // we should get an error because not all bytes are available
    CHECK(memcmp(recv_buffer, expected_data, 2) == 0);                // Check the 2 bytes we received
    clear_uart_rx_buffer();
}

TEST_CASE("HAL_UART_Transmit Buffer Overflow")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t data[UART_TX_BUFFER_SIZE + 1]; // Create data larger than the buffer
    memset(data, 'A', sizeof(data));

    // Fill the buffer partially
    uint8_t partial_data[10] = {0};
    HAL_UART_Transmit(&huart, partial_data, sizeof(partial_data), 1000);

    CHECK(HAL_UART_Transmit(&huart, data, sizeof(data) - 1, 1000) == HAL_ERROR); // Expect HAL_ERROR
}

TEST_CASE("HAL_UARTEx_GetRxEventType Test - Half Transfer")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    set_mocked_uart_rx_event_type(HAL_UART_RXEVENT_HT);
    CHECK(HAL_UARTEx_GetRxEventType(&huart) == HAL_UART_RXEVENT_HT);
}

TEST_CASE("HAL_UARTEx_GetRxEventType Test - Idle Line")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    set_mocked_uart_rx_event_type(HAL_UART_RXEVENT_IDLE);
    CHECK(HAL_UARTEx_GetRxEventType(&huart) == HAL_UART_RXEVENT_IDLE);
}

TEST_CASE("HAL_UARTEx_ReceiveToIdle_DMA Test")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t expected_data[] = "idle test";
    uint8_t recv_buffer[20];

    // Inject data into the RX buffer
    inject_uart_rx_data(expected_data, sizeof(expected_data) - 1);

    CHECK(HAL_UARTEx_ReceiveToIdle_DMA(&huart, recv_buffer, sizeof(expected_data) - 1) == HAL_OK);
    CHECK(memcmp(recv_buffer, expected_data, sizeof(expected_data) - 1) == 0);
    clear_uart_rx_buffer();
}

TEST_CASE("HAL_UARTEx_ReceiveToIdle_DMA partial")
{
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    uint8_t expected_data[] = "idle test";
    uint8_t recv_buffer[20];

    // Inject partial data into the RX buffer
    inject_uart_rx_data(expected_data, 5);

    CHECK(HAL_UARTEx_ReceiveToIdle_DMA(&huart, recv_buffer, sizeof(expected_data) - 1) == HAL_ERROR);
    CHECK(memcmp(recv_buffer, expected_data, 5) == 0);
    clear_uart_rx_buffer();
}
