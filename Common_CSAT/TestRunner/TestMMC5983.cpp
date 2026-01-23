#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "MMC5983.hpp"

#include "mock_hal.h"
#include "Transport.hpp"

TEST_CASE("MMC5983Core toInt32 decodes signed 24-bit value")
{
    CHECK(MMC5983Core::toInt32(0x00, 0x00, 0x00) == -131072); // NULL_VALUE
    CHECK(MMC5983Core::toInt32(0x00, 0x00, 0x01) == -130048);
    CHECK(MMC5983Core::toInt32(0xFF, 0xFF, 0xFF) == 131071); // max value
}

TEST_CASE("MMC5983Core convertMag returns Tesla quantity")
{
    auto mag = MMC5983Core::convertMag({0.f, 0.f, 1.f}); // small positive
    CHECK(mag[0].in(au::teslaInBodyFrame) == doctest::Approx(0.0f));
    CHECK(mag[1].in(au::teslaInBodyFrame) == doctest::Approx(0.0f));
    CHECK(mag[2].in(au::teslaInBodyFrame) > 0.0f);
}

TEST_CASE("MMC5983Core convertTmp converts raw temperature")
{
    CHECK(MMC5983Core::convertTmp(0).in(au::celsius_qty) == doctest::Approx(-75.0f));
    CHECK(MMC5983Core::convertTmp(100).in(au::celsius_qty) == doctest::Approx(5.0f));
}

TEST_CASE("MMC5983Core parseRawMagnetometerData decodes packed buffer")
{
    uint8_t buf[] = {
        0x02, 0x01, // X MSB, ISB
        0x05, 0x04, // Y MSB, ISB
        0x08, 0x07, // Z MSB, ISB
        0xE4        // packed LSBs: X=3, Y=2, Z=1
    };

    auto result = MMC5983Core::parseMagnetometerData(buf);
    CHECK(result.at(0) == MMC5983Core::toInt32(3, 0x01, 0x02));
    CHECK(result.at(1) == MMC5983Core::toInt32(2, 0x04, 0x05));
    CHECK(result.at(2) == MMC5983Core::toInt32(1, 0x07, 0x08));
}

#
#
#

SPI_HandleTypeDef mock_spi;
GPIO_TypeDef mock_gpio;

using Config = SPI_Register_Config<mock_spi, GPIO_PIN_5, 128>;
using Transport = SPIRegisterTransport<Config>;
using Magnetometer = MMC5983<Transport>;

TEST_CASE("MMC5983 readChipID returns correct ID")
{
    clear_spi_rx_buffer();
    uint8_t id[] = {0x30}; // ID
    inject_spi_rx_data(id, sizeof(id));

    Config config(&mock_gpio);
    Transport transport(config);
    Magnetometer mag(transport);

    auto result = mag.readChipID();
    REQUIRE(result.has_value());
    CHECK(result.value() == 0x30);
}

TEST_CASE("MMC5983 readRawMagnetometer returns decoded values")
{
    clear_spi_rx_buffer();
    uint8_t raw[] = {
        0x02, 0x01, // X MSB, ISB
        0x05, 0x04, // Y MSB, ISB
        0x08, 0x07, // Z MSB, ISB
        0xE4,       // packed LSBs
        0x00, 0x00  // padding
    };
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    Magnetometer mag(transport);

    auto result = mag.readRawMagnetometer();
    CHECK(result.at(0) == MMC5983Core::toInt32(3, 0x01, 0x02));
    CHECK(result.at(1) == MMC5983Core::toInt32(2, 0x04, 0x05));
    CHECK(result.at(2) == MMC5983Core::toInt32(1, 0x07, 0x08));
}

TEST_CASE("MMC5983 readThermometer returns calibrated temperature")
{
    clear_spi_rx_buffer();
    uint8_t temp[] = {0x4B}; // raw = 75 → -75 + 75*0.8 = -15.0 °C
    inject_spi_rx_data(temp, sizeof(temp));

    Config config(&mock_gpio);
    Transport transport(config);
    Magnetometer mag(transport);

    auto result = mag.readThermometer();
    REQUIRE(result.has_value());
    CHECK(result->in(au::celsius_qty) == doctest::Approx(-15.0f));
}

TEST_CASE("MMC5983 configureContinuousMode writes correct registers")
{
    clear_spi_tx_buffer();

    Config config(&mock_gpio);
    Transport transport(config);
    Magnetometer mag(transport);

    CHECK(mag.configureContinuousMode(0b101, 0b011, true) == true);

    auto tx = get_spi_tx_buffer();
    CHECK(tx[0] == static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_CONTROL1));
    CHECK(tx[1] == 0x80); // ctrl1
    CHECK(tx[2] == static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_CONTROL2));
    CHECK(tx[3] == 0xBD); // ctrl2 = 0x80 | (0b011 << 4) | (1 << 3) | 0b101
}
