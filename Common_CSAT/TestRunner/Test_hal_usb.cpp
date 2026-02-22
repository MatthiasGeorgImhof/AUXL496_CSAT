#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "mock_hal.h"
#include <string.h>
#include <stdio.h>

TEST_CASE("CDC_Transmit_FS Test")
{
    uint8_t data[] = "USB test data";
    uint16_t len = sizeof(data) - 1;

    CHECK(CDC_Transmit_FS(data, len) == 0);
    CHECK(get_usb_tx_buffer_count() == len);
    CHECK(memcmp(get_usb_tx_buffer(), data, len) == 0);

    clear_usb_tx_buffer();
    CHECK(get_usb_tx_buffer_count() == 0);
}
