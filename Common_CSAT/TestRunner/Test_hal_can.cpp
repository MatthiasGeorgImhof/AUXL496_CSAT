#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

TEST_CASE("HAL_CAN_AddTxMessage Standard ID")
{
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

TEST_CASE("HAL_CAN_AddTxMessage Extended ID")
{
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

TEST_CASE("HAL_CAN_GetRxMessage Standard ID")
{
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

TEST_CASE("HAL_CAN_GetRxMessage Extended ID")
{
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

TEST_CASE("HAL_CAN_GetTxMailboxesFreeLevel")
{
    set_current_free_mailboxes(1U);
    CHECK(HAL_CAN_GetTxMailboxesFreeLevel(NULL) == 1);
    set_current_free_mailboxes(3U);
    CHECK(HAL_CAN_GetTxMailboxesFreeLevel(NULL) == 3);
}

TEST_CASE("HAL_CAN_ConfigFilter")
{
    CAN_FilterTypeDef filter;
    CHECK(HAL_CAN_ConfigFilter(NULL, &filter) == HAL_OK);
}

TEST_CASE("HAL_CAN_GetRxFifoFillLevel")
{
    set_current_rx_fifo_fill_level(1);
    CHECK(HAL_CAN_GetRxFifoFillLevel(NULL, 0) == 1);
    set_current_rx_fifo_fill_level(0);
    CHECK(HAL_CAN_GetRxFifoFillLevel(NULL, 0) == 0);
}
