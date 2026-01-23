#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "BMI270_MMC5983.hpp"
#include "mock_hal.h"
#include "Transport.hpp"

SPI_HandleTypeDef mock_spi;
GPIO_TypeDef mock_gpio;

using Config = SPI_Register_Config<mock_spi, GPIO_PIN_5, 128>;
using Transport = SPIRegisterTransport<Config>;
using IMUCombo = BMI270_MMC5983<Transport>;

static_assert(HasBodyGyroscope<IMUCombo>, "IMUType does not satisfy HasBodyGyroscope");
static_assert(HasBodyAccelerometer<IMUCombo>, "IMUType does not satisfy HasBodyAccelerometer");
static_assert(HasBodyMagnetometer<IMUCombo>, "IMUType does not satisfy HasBodyMagnetometer");

TEST_CASE("BMI270_MMC5983 configure succeeds with correct MMC5983 ID")
{
    clear_spi_rx_buffer();

    // Inject MMC5983 ID response at AUX_DATA_X_LSB
    uint8_t id_response[] = {0x00, 0x30}; // dummy + MMC5983_ID
    inject_spi_rx_data(id_response, sizeof(id_response));

    Config config(&mock_gpio);
    Transport transport(config);
    IMUCombo imu(transport);

    CHECK(imu.configure() == true);
}

TEST_CASE("BMI270_MMC5983 configure fails with incorrect ID")
{
    clear_spi_rx_buffer();

    // Inject wrong ID
    uint8_t bad_id[] = {0x42};
    inject_spi_rx_data(bad_id, sizeof(bad_id));

    Config config(&mock_gpio);
    Transport transport(config);
    IMUCombo imu(transport);

    CHECK(imu.configure() == false);
}

TEST_CASE("BMI270AuxTransport write_then_read shifts dummy byte")
{
    clear_spi_rx_buffer();

    // Inject dummy + 3 magnetometer bytes
    uint8_t raw[] = {0xff, 0x11, 0x22, 0x33}; // dummy + X_LSB, X_MSB, Y_LSB
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> bmi(transport);
    BMI270AuxTransport<BMI270<Transport>> aux(bmi);

    uint8_t rx[3] = {};
    auto ok = aux.read_reg(static_cast<uint8_t>(MMC5983_REGISTERS::MMC5983_XOUT0), rx, sizeof(rx));
    CHECK(ok == true);

    // Check shifted result
    CHECK(rx[0] == 0x11);
    CHECK(rx[1] == 0x22);
    CHECK(rx[2] == 0x33);
}

TEST_CASE("BMI270_MMC5983 readRawMagnetometer returns expected values")
{
    clear_spi_rx_buffer();

uint8_t raw[] = {
    0xff,       // dummy    
    0x02, 0x01, // X MSB, ISB
    0x05, 0x04, // Y MSB, ISB
    0x08, 0x07, // Z MSB, ISB
    0xe5,       // packed LSBs: X=3, Y=2, Z=1
    0x00, 0x00
};

    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    IMUCombo imu(transport);

    auto result = imu.readRawMagnetometer();
    CHECK(result.at(0) == MMC5983Core::toInt32((raw[7] >> 6) & 0b11, raw[2], raw[1]));
    CHECK(result.at(1) == MMC5983Core::toInt32((raw[7] >> 4) & 0b11, raw[4], raw[3]));
    CHECK(result.at(2) == MMC5983Core::toInt32((raw[7] >> 2) & 0b11, raw[6], raw[5]));
}
