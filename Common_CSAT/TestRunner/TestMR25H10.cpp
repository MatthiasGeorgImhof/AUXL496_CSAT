#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "MR25H10.hpp"
#include "mock_hal.h"
#include "Transport.hpp"

SPI_HandleTypeDef mock_spi;
GPIO_TypeDef mock_gpio;

using Config = SPI_Stream_Config<mock_spi, GPIO_PIN_5, 128>;
using Transport = SPIStreamTransport<Config>;
using SRAM = MR25H10<Transport>;

TEST_CASE("MR25H10 readStatus returns correct value") {
    clear_spi_rx_buffer();

    uint8_t status = 0xAC;
    inject_spi_rx_data(&status, 1);  // Inject status byte

    Config config(&mock_gpio);
    Transport transport(config);
    SRAM sram(transport);

    auto result = sram.readStatus();
    REQUIRE(result.has_value());
    CHECK(result.value() == status);
}

TEST_CASE("MR25H10 writeStatus sends correct command and data") {
    clear_spi_tx_buffer();

    Config config(&mock_gpio);
    Transport transport(config);
    SRAM sram(transport);

    uint8_t status = 0x5A;
    CHECK(sram.writeStatus(status) == true);

    auto tx = get_spi_tx_buffer();
    CHECK(get_spi_tx_buffer_count() == 2);
    CHECK(tx[0] == static_cast<uint8_t>(MR25H10_COMMANDS::MR25H10_WRSR));
    CHECK(tx[1] == status);
}
