#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>

TEST_CASE("HAL_CAN_AddTxMessage Standard ID") {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    uint32_t mailbox;

    header.StdId = 0x123;
    header.IDE = 0; // Standard ID
    header.DLC = 8;

    CHECK(HAL_CAN_AddTxMessage(NULL, &header, data, &mailbox) == HAL_OK);
    CHECK(get_can_tx_buffer_count() == 1);
    CAN_TxMessage_t msg = get_can_tx_message(0);
    CHECK(msg.TxHeader.StdId == 0x123);
    CHECK(msg.TxHeader.IDE == 0);
    CHECK(msg.TxHeader.DLC == 8);
    CHECK(memcmp(msg.pData, data, 8) == 0);
    clear_can_tx_buffer();
    CHECK(get_can_tx_buffer_count() == 0);
}

TEST_CASE("HAL_CAN_AddTxMessage Extended ID") {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint32_t mailbox;

    header.ExtId = 0x1234567;
    header.IDE = 1; // Extended ID
    header.DLC = 8;

    CHECK(HAL_CAN_AddTxMessage(NULL, &header, data, &mailbox) == HAL_OK);
    CHECK(get_can_tx_buffer_count() == 1);
    CAN_TxMessage_t msg = get_can_tx_message(0);
    CHECK(msg.TxHeader.ExtId == 0x1234567);
    CHECK(msg.TxHeader.IDE == 1);
    CHECK(msg.TxHeader.DLC == 8);
    CHECK(memcmp(msg.pData, data, 8) == 0);
    clear_can_tx_buffer();
    CHECK(get_can_tx_buffer_count() == 0);
}

TEST_CASE("HAL_CAN_GetRxMessage Standard ID") {
     CAN_RxHeaderTypeDef header;
     CAN_HandleTypeDef hcan;
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t rx_data[8];

    header.StdId = 0x123;
    header.IDE = 0;
    header.DLC = 8;

    inject_can_rx_message(header, data);

    CHECK(HAL_CAN_GetRxMessage(&hcan, 0, &header, rx_data) == HAL_OK);
    CHECK(header.StdId == 0x123);
    CHECK(header.IDE == 0);
    CHECK(header.DLC == 8);
    CHECK(memcmp(rx_data, data, 8) == 0);
}


TEST_CASE("HAL_CAN_GetRxMessage Extended ID") {
    CAN_RxHeaderTypeDef header;
    CAN_HandleTypeDef hcan;
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t rx_data[8];

    header.ExtId = 0x1234567;
    header.IDE = 1;
    header.DLC = 8;

    inject_can_rx_message(header, data);

    CHECK(HAL_CAN_GetRxMessage(&hcan, 0, &header, rx_data) == HAL_OK);
    CHECK(header.ExtId == 0x1234567);
    CHECK(header.IDE == 1);
    CHECK(header.DLC == 8);
    CHECK(memcmp(rx_data, data, 8) == 0);
}

TEST_CASE("HAL_CAN_GetTxMailboxesFreeLevel") {
    set_current_free_mailboxes(1);
    CHECK(HAL_CAN_GetTxMailboxesFreeLevel(NULL) == 1);
    set_current_free_mailboxes(3);
    CHECK(HAL_CAN_GetTxMailboxesFreeLevel(NULL) == 3);
}


TEST_CASE("HAL_CAN_ConfigFilter"){
    CAN_FilterTypeDef filter;
    CHECK(HAL_CAN_ConfigFilter(NULL, &filter) == HAL_OK);
}

TEST_CASE("HAL_CAN_GetRxFifoFillLevel"){
    set_current_rx_fifo_fill_level(1);
    CHECK(HAL_CAN_GetRxFifoFillLevel(NULL,0) == 1);
    set_current_rx_fifo_fill_level(0);
    CHECK(HAL_CAN_GetRxFifoFillLevel(NULL,0) == 0);
}


TEST_CASE("HAL_UART_Transmit") {
    UART_HandleTypeDef huart;
    uint8_t data[] = "test data";
    init_uart_handle(&huart);
    CHECK(HAL_UART_Transmit(&huart, data, sizeof(data)-1, 1000) == HAL_OK);
    CHECK(get_uart_tx_buffer_count() == 9);
    CHECK(memcmp(get_uart_tx_buffer(), data, sizeof(data)-1) == 0);
    clear_uart_tx_buffer();
    CHECK(get_uart_tx_buffer_count() == 0);
}

TEST_CASE("HAL_UART_Transmit_DMA") {
    UART_HandleTypeDef huart;
     init_uart_handle(&huart);
    uint8_t data[] = "test data";
    CHECK(HAL_UART_Transmit_DMA(&huart, data, sizeof(data)-1) == HAL_OK);
    CHECK(get_uart_tx_buffer_count() == 9);
    CHECK(memcmp(get_uart_tx_buffer(), data, sizeof(data)-1) == 0);
    clear_uart_tx_buffer();
    CHECK(get_uart_tx_buffer_count() == 0);
}


TEST_CASE("HAL_UART_Receive timeout") {
    UART_HandleTypeDef huart;
     init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where only some bytes are received before timeout
    inject_uart_rx_data(expected_data, 2); // Only Injecting 2 bytes
    set_current_tick(0); // set mock time to 0

    CHECK(HAL_UART_Receive(&huart, recv_buffer, 5, 100) == HAL_ERROR); // we should get timeout
    CHECK(memcmp(recv_buffer, expected_data, 2) == 0); //Check the 2 bytes we received
    clear_uart_rx_buffer();
}

TEST_CASE("HAL_UART_Receive no timeout") {
     UART_HandleTypeDef huart;
      init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where all bytes are received
    inject_uart_rx_data(expected_data, 5); // Inject all bytes
    set_current_tick(0); // set mock time to 0

    CHECK(HAL_UART_Receive(&huart, recv_buffer, 5, 100) == HAL_OK); // we should not get timeout
    CHECK(memcmp(recv_buffer, expected_data, 5) == 0); //Check the 5 bytes we received
    clear_uart_rx_buffer();
}


TEST_CASE("HAL_UART_Receive_DMA no timeout") {
    UART_HandleTypeDef huart;
     init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where all bytes are received
    inject_uart_rx_data(expected_data, 5); // Inject all bytes
    set_current_tick(0);
    CHECK(HAL_UART_Receive_DMA(&huart, recv_buffer, 5) == HAL_OK); // we should not get timeout
    CHECK(memcmp(recv_buffer, expected_data, 5) == 0); //Check the 5 bytes we received
    clear_uart_rx_buffer();
}


TEST_CASE("HAL_UART_Receive_DMA partial") {
    UART_HandleTypeDef huart;
     init_uart_handle(&huart);
    uint8_t expected_data[] = "hello";
    uint8_t recv_buffer[10];

    // Simulate a situation where all bytes are received
    inject_uart_rx_data(expected_data, 2); // Inject 2 bytes
    set_current_tick(0);
    CHECK(HAL_UART_Receive_DMA(&huart, recv_buffer, 5) == HAL_ERROR); // we should get an error because not all bytes are available
    CHECK(memcmp(recv_buffer, expected_data, 2) == 0); //Check the 2 bytes we received
    clear_uart_rx_buffer();
}

TEST_CASE("HAL_Delay"){
    set_current_tick(0);
    HAL_Delay(100);
    CHECK(HAL_GetTick() == 100);
    HAL_Delay(200);
    CHECK(HAL_GetTick() == 300);
}


TEST_CASE("HAL_GetTick"){
    set_current_tick(10);
    CHECK(HAL_GetTick() == 10);
    set_current_tick(20);
    CHECK(HAL_GetTick() == 20);
}

TEST_CASE("HAL_I2C_Master_Transmit") {
    I2C_HandleTypeDef hi2c;
    uint8_t data[] = {0x12, 0x34, 0x56};
    CHECK(HAL_I2C_Master_Transmit(&hi2c, 0x50, data, 3, 100) == HAL_OK);
}

TEST_CASE("HAL_I2C_Mem_Read success") {
    I2C_HandleTypeDef hi2c;
    uint8_t expected_data[] = {0xAA, 0xBB, 0xCC};
    uint8_t read_data[3];
    inject_i2c_mem_data(0x50, 0x10, expected_data, 3);

    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x50, 0x10, 1, read_data, 3, 100) == HAL_OK);
    CHECK(memcmp(read_data, expected_data, 3) == 0);
    clear_i2c_mem_data();
}


TEST_CASE("HAL_I2C_Mem_Read Fail invalid address") {
    I2C_HandleTypeDef hi2c;
    uint8_t expected_data[] = {0xAA, 0xBB, 0xCC};
    uint8_t read_data[3];
    inject_i2c_mem_data(0x50, 0x10, expected_data, 3);
    
    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x51, 0x10, 1, read_data, 3, 100) == HAL_ERROR);
    clear_i2c_mem_data();
}

TEST_CASE("HAL_I2C_Mem_Read Different size") {
    I2C_HandleTypeDef hi2c;
     uint8_t expected_data[] = {0xAA, 0xBB, 0xCC};
    uint8_t read_data[4];
    inject_i2c_mem_data(0x50, 0x10, expected_data, 3);
    
    CHECK(HAL_I2C_Mem_Read(&hi2c, 0x50, 0x10, 1, read_data, 4, 100) == HAL_OK);
     clear_i2c_mem_data();
}



TEST_CASE("HAL_I2C_Mem_Write") {
    I2C_HandleTypeDef hi2c;
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    CHECK(HAL_I2C_Mem_Write(&hi2c, 0x50, 0x20, 1, data, 4, 100) == HAL_OK);
}

TEST_CASE("HAL_UART_Transmit Buffer Overflow") {
    UART_HandleTypeDef huart;
     init_uart_handle(&huart);
    uint8_t data[UART_TX_BUFFER_SIZE + 1]; // Create data larger than the buffer
    memset(data, 'A', sizeof(data));

    //Fill the buffer partially
    uint8_t partial_data[10] = {0};
    HAL_UART_Transmit(&huart, partial_data, sizeof(partial_data), 1000);

    CHECK(HAL_UART_Transmit(&huart, data, sizeof(data) - 1, 1000) == HAL_ERROR); // Expect HAL_ERROR
}

//--- GPIO Tests ---
TEST_CASE("HAL_GPIO_Init Test") {
    GPIO_TypeDef GPIOx;
    GPIO_InitTypeDef GPIO_Init;
    GPIO_Init.Pin = 1;
    GPIO_Init.Mode = 1;
    GPIO_Init.Pull = 1;
    GPIO_Init.Speed = 1;
    GPIO_Init.Alternate = 1;

    HAL_GPIO_Init(&GPIOx, &GPIO_Init);

    CHECK(GPIOx.Init.Pin == 1);
    CHECK(GPIOx.Init.Mode == 1);
    CHECK(GPIOx.Init.Pull == 1);
    CHECK(GPIOx.Init.Speed == 1);
    CHECK(GPIOx.Init.Alternate == 1);
}

TEST_CASE("HAL_GPIO_WritePin and HAL_GPIO_ReadPin Test") {
    GPIO_TypeDef GPIOx;
    uint16_t GPIO_Pin = 1;

    // Write a pin high, and check if we can read it back as high.
    HAL_GPIO_WritePin(&GPIOx, GPIO_Pin, GPIO_PIN_SET);
    CHECK(HAL_GPIO_ReadPin(&GPIOx, GPIO_Pin) == GPIO_PIN_SET);

    // Write a pin low, and check if we can read it back as low.
    HAL_GPIO_WritePin(&GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    CHECK(HAL_GPIO_ReadPin(&GPIOx, GPIO_Pin) == GPIO_PIN_RESET);

    GPIO_Pin = 1 << 5;
    HAL_GPIO_WritePin(&GPIOx, GPIO_Pin, GPIO_PIN_SET);
    CHECK(HAL_GPIO_ReadPin(&GPIOx, GPIO_Pin) == GPIO_PIN_SET);

     GPIO_Pin = 1 << 5;
    HAL_GPIO_WritePin(&GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    CHECK(HAL_GPIO_ReadPin(&GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
}

TEST_CASE("HAL_GPIO_TogglePin Test") {
    GPIO_TypeDef GPIOx;
    uint16_t GPIO_Pin = 1;

    // Start with the pin low.
    HAL_GPIO_WritePin(&GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    CHECK(HAL_GPIO_ReadPin(&GPIOx, GPIO_Pin) == GPIO_PIN_RESET);

    // Toggle it high.
    HAL_GPIO_TogglePin(&GPIOx, GPIO_Pin);
    CHECK(HAL_GPIO_ReadPin(&GPIOx, GPIO_Pin) == GPIO_PIN_SET);

    // Toggle it back low.
    HAL_GPIO_TogglePin(&GPIOx, GPIO_Pin);
    CHECK(HAL_GPIO_ReadPin(&GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
}

TEST_CASE("get_gpio_pin_state and set_gpio_pin_state") {
    GPIO_TypeDef GPIOx;
    uint16_t GPIO_Pin = 1 << 2; // Pin 2

    //Initially Reset
    CHECK(get_gpio_pin_state(&GPIOx, GPIO_Pin) == GPIO_PIN_RESET);

    // Set it to high using set_gpio_pin_state
    set_gpio_pin_state(&GPIOx, GPIO_Pin, GPIO_PIN_SET);
    CHECK(get_gpio_pin_state(&GPIOx, GPIO_Pin) == GPIO_PIN_SET);

    //Set it to low using set_gpio_pin_state
    set_gpio_pin_state(&GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    CHECK(get_gpio_pin_state(&GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
}

TEST_CASE("CDC_Transmit_FS Test") {
    uint8_t data[] = "USB test data";
    uint16_t len = sizeof(data) - 1;

    CHECK(CDC_Transmit_FS(data, len) == 0);
    CHECK(get_usb_tx_buffer_count() == len);
    CHECK(memcmp(get_usb_tx_buffer(), data, len) == 0);

    clear_usb_tx_buffer();
    CHECK(get_usb_tx_buffer_count() == 0);
}

TEST_CASE("HAL_UARTEx_GetRxEventType Test - Half Transfer") {
    UART_HandleTypeDef huart;
    init_uart_handle(&huart);
    set_mocked_uart_rx_event_type(HAL_UART_RXEVENT_HT);
    CHECK(HAL_UARTEx_GetRxEventType(&huart) == HAL_UART_RXEVENT_HT);
}

TEST_CASE("HAL_UARTEx_GetRxEventType Test - Idle Line") {
    UART_HandleTypeDef huart;
     init_uart_handle(&huart);
    set_mocked_uart_rx_event_type(HAL_UART_RXEVENT_IDLE);
    CHECK(HAL_UARTEx_GetRxEventType(&huart) == HAL_UART_RXEVENT_IDLE);
}

TEST_CASE("HAL_UARTEx_ReceiveToIdle_DMA Test") {
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

TEST_CASE("HAL_UARTEx_ReceiveToIdle_DMA partial") {
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

//--- SPI Tests ---

TEST_CASE("HAL_SPI_Init Test") {
    SPI_HandleTypeDef hspi;
    init_spi_handle(&hspi); // Initialize SPI handle

    CHECK(HAL_SPI_Init(&hspi) == HAL_OK);
    // Add more checks here if you want to verify values within the SPI_InitTypeDef
}

TEST_CASE("HAL_SPI_Transmit Test") {
    SPI_HandleTypeDef hspi;
    uint8_t tx_data[] = "SPI test";
    init_spi_handle(&hspi);

    CHECK(HAL_SPI_Transmit(&hspi, tx_data, sizeof(tx_data) - 1, 100) == HAL_OK);
    CHECK(get_spi_tx_buffer_count() == sizeof(tx_data) - 1);
    CHECK(memcmp(get_spi_tx_buffer(), tx_data, sizeof(tx_data) - 1) == 0);

    clear_spi_tx_buffer();
    CHECK(get_spi_tx_buffer_count() == 0);
}

TEST_CASE("HAL_SPI_Receive Test") {
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

TEST_CASE("HAL_SPI_TransmitReceive Test") {
    SPI_HandleTypeDef hspi;
     init_spi_handle(&hspi);
    uint8_t tx_data[] = "TxData";
    uint8_t expected_rx_data[] = {0x10, 0x20, 0x30, 0x40,0x50,0x60};
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

TEST_CASE("HAL_SPI_TransmitReceive - Size greater than RX") {
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