#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "BMI270.hpp"
#include "mock_hal.h"
#include "Transport.hpp"

SPI_HandleTypeDef mock_spi;
GPIO_TypeDef mock_gpio;

using Config = SPI_Register_Config<mock_spi, GPIO_PIN_5, 128>;
using Transport = SPIRegisterTransport<Config>;
using IMUType = BMI270<Transport>;

static_assert(ProvidesChipID<IMUType>);
static_assert(HasBodyAccelerometer<IMUType>);
static_assert(HasBodyGyroscope<IMUType>);
static_assert(HasThermometer<IMUType>);

TEST_CASE("BMI270 readChipID returns correct value")
{
    clear_spi_rx_buffer();

    // Inject mock chip ID response
    uint8_t chip_id[] {0xff, 0x24};
    inject_spi_rx_data(chip_id, sizeof(chip_id));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);
    auto result = imu.readChipID();

    REQUIRE(result.has_value());
    CHECK(result.value() == chip_id[1]);
}

TEST_CASE("BMI270 readAccelerometer returns scaled values")
{
    clear_spi_rx_buffer();

    // Inject raw accelerometer data: X=16384, Y=8192, Z=-8192
    uint8_t acc_raw[] = {
        0xff,        // dummy
        0x00, 0x40, // X = 0x4000 = 16384
        0x00, 0x20, // Y = 0x2000 = 8192
        0x00, 0xE0  // Z = 0xE000 = -8192 (two's complement)
    };
    inject_spi_rx_data(acc_raw, sizeof(acc_raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);
    auto result = imu.readAccelerometer();

    REQUIRE(result.has_value());
    float x = result->at(0).in(au::metersPerSecondSquaredInBodyFrame);
    float y = result->at(1).in(au::metersPerSecondSquaredInBodyFrame);
    float z = result->at(2).in(au::metersPerSecondSquaredInBodyFrame);

    CHECK(x == doctest::Approx(-9.80665f)); // X
    CHECK(y == doctest::Approx(4.90333f));  // Y
    CHECK(z == doctest::Approx(-4.90333f)); // Z
}

TEST_CASE("BMI270 readGyroscope returns scaled values")
{
    clear_spi_rx_buffer();

    // Inject raw gyro data: X=164, Y=-164, Z=0
    uint8_t gyro_raw[] {
        0xff,       // dummy   
        0xA4, 0x00, // X = 0x00A4 = 164
        0x5C, 0xFF, // Y = 0xFF5C = -164
        0x00, 0x00  // Z = 0
    };
    inject_spi_rx_data(gyro_raw, sizeof(gyro_raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);
    auto result = imu.readGyroscope();

    REQUIRE(result.has_value());

    float x = result->at(0).in(au::degreesPerSecondInBodyFrame);
    float y = result->at(1).in(au::degreesPerSecondInBodyFrame);
    float z = result->at(2).in(au::degreesPerSecondInBodyFrame);

    CHECK(x == doctest::Approx(10.0f));  // 164 / 16.4
    CHECK(y == doctest::Approx(10.0f)); //  -164 / 16.4
    CHECK(z == doctest::Approx(0.0f));   // 0
}

TEST_CASE("BMI270 readThermometer returns scaled temperature")
{
    clear_spi_rx_buffer();

    // Raw value = 0x0200 → 512 → 23 + 512 / 512 = 24.0 °C
    uint8_t temp_raw[] {0xff, 0x00, 0x02}; // dummy, LSB = 0x00, MSB = 0x02
    inject_spi_rx_data(temp_raw, sizeof(temp_raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);
    auto result = imu.readThermometer();

    REQUIRE(result.has_value());

    float temp = result->in(au::celsius_qty);
    CHECK(temp == doctest::Approx(24.0f));
}

TEST_CASE("BMI270 readThermometer handles edge cases correctly")
{
    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    SUBCASE("Raw value 0x0000 → 23.0 °C")
    {
        clear_spi_rx_buffer();
        uint8_t temp_raw[] {0xff, 0x00, 0x00}; // dummy, raw = 0
        inject_spi_rx_data(temp_raw, sizeof(temp_raw));

        auto result = imu.readThermometer();
        REQUIRE(result.has_value());

        float temp = result->in(au::celsius_qty);
        CHECK(temp == doctest::Approx(23.0f));
    }

    SUBCASE("Raw value 0x0200 → 24.0 °C")
    {
        clear_spi_rx_buffer();
        uint8_t temp_raw[] {0xff, 0x00, 0x02}; // dummy, raw = 512
        inject_spi_rx_data(temp_raw, sizeof(temp_raw));

        auto result = imu.readThermometer();
        REQUIRE(result.has_value());

        float temp = result->in(au::celsius_qty);
        CHECK(temp == doctest::Approx(24.0f));
    }

    SUBCASE("Raw value 0x7FFF → ~86.998 °C")
    {
        clear_spi_rx_buffer();
        uint8_t temp_raw[] {0xff, 0xFF, 0x7F}; // dummy, raw = 32767
        inject_spi_rx_data(temp_raw, sizeof(temp_raw));

        auto result = imu.readThermometer();
        REQUIRE(result.has_value());

        float temp = result->in(au::celsius_qty);
        CHECK(temp == doctest::Approx(86.998f));
    }

    SUBCASE("Raw value 0x8001 → ~-40.998 °C")
    {
        clear_spi_rx_buffer();
        uint8_t temp_raw[] {0xff, 0x01, 0x80}; // dummy, raw = -32767
        inject_spi_rx_data(temp_raw, sizeof(temp_raw));

        auto result = imu.readThermometer();
        REQUIRE(result.has_value());

        float temp = result->in(au::celsius_qty);
        CHECK(temp == doctest::Approx(-40.998f));
    }
}


TEST_CASE("BMI270 readStatus() returns correct status values") {
    clear_spi_rx_buffer();

    // Inject mock status, error, and internal status register values
    uint8_t status_raw[] = {
        0xff, 0x1A, // STATUS
        0xff, 0x2B, // ERR_REG
        0xff, 0x3C  // INTERNAL_STATUS
    };
    inject_spi_rx_data(status_raw, sizeof(status_raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    BMI270_STATUS status = imu.readStatus();

    CHECK(status.status == 0x1A);
    CHECK(status.error == 0x2B);
    CHECK(status.internal_status == 0x3C);
}


TEST_CASE("BMI270 readRawAccelerometer() returns correct raw accelerometer values") {
    clear_spi_rx_buffer();

    // Inject raw accelerometer data: X=16384, Y=8192, Z=-8192
    uint8_t acc_raw[] = {
        0xff,        // dummy
        0x00, 0x40, // X = 0x4000 = 16384
        0x00, 0x20, // Y = 0x2000 = 8192
        0x00, 0xE0  // Z = 0xE000 = -8192 (two's complement)
    };
    inject_spi_rx_data(acc_raw, sizeof(acc_raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);
    auto result = imu.readRawAccelerometer();

    CHECK(result[0] == 16384);
    CHECK(result[1] == 8192);
    CHECK(result[2] == -8192);
}

TEST_CASE("BMI270 readRawGyroscope() returns correct raw gyroscope values") {
    clear_spi_rx_buffer();

    // Inject raw gyro data: X=164, Y=-164, Z=0
    uint8_t gyro_raw[] = {
        0xff,       // dummy
        0xA4, 0x00, // X = 0x00A4 = 164
        0x5C, 0xFF, // Y = 0xFF5C = -164
        0x00, 0x00  // Z = 0
    };
    inject_spi_rx_data(gyro_raw, sizeof(gyro_raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);
    auto result = imu.readRawGyroscope();

    CHECK(result[0] == 164);
    CHECK(result[1] == -164);
    CHECK(result[2] == 0);
}

TEST_CASE("BMI270 readRawThermometer() returns correct raw thermometer value") {
    clear_spi_rx_buffer();

    // Raw value = 0x0200 → 512
    uint8_t temp_raw[] = {0xff, 0x00, 0x02}; // dummy, LSB = 0x00, MSB = 0x02
    inject_spi_rx_data(temp_raw, sizeof(temp_raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);
    uint16_t result = imu.readRawThermometer();

    CHECK(result == 512);
}

TEST_CASE("BMI270 writeRegister writes correct register and value")
{
    clear_spi_tx_buffer();

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    imu.writeRegister(BMI270_REGISTERS::ACC_CONF, 0x55);

    REQUIRE(get_spi_tx_buffer_count() == 2);
    CHECK(get_spi_tx_buffer()[0] == static_cast<uint8_t>(BMI270_REGISTERS::ACC_CONF));
    CHECK(get_spi_tx_buffer()[1] == 0x55);
}

TEST_CASE("BMI270 readRegister reads correct value after dummy byte")
{
    clear_spi_rx_buffer();

    uint8_t raw[] = {0xFF, 0xAB};   // dummy + value
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    uint8_t v = 0;
    CHECK(imu.readRegister(BMI270_REGISTERS::ACC_RANGE, v) == true);
    CHECK(v == 0xAB);
}

TEST_CASE("BMI270 readRegisters reads dummy + payload correctly")
{
    clear_spi_rx_buffer();

    uint8_t raw[] = {0xFF, 0x11, 0x22, 0x33};
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    uint8_t rx[4] = {};
    CHECK(imu.readRegisters(BMI270_REGISTERS::ACC_DATA_X_LSB, rx, sizeof(rx)) == true);

    CHECK(rx[0] == 0xFF);
    CHECK(rx[1] == 0x11);
    CHECK(rx[2] == 0x22);
    CHECK(rx[3] == 0x33);
}

TEST_CASE("BMI270 writeRegisterWithCheck succeeds when readback matches")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    uint8_t raw[] = {0xFF, 0x77};   // dummy + readback
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.writeRegisterWithCheck(BMI270_REGISTERS::GYR_CONF, 0x77) == true);
}

TEST_CASE("BMI270 writeRegisterWithCheck fails when readback mismatches")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    uint8_t raw[] = {0xFF, 0x00};   // dummy + wrong value
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.writeRegisterWithCheck(BMI270_REGISTERS::GYR_CONF, 0x77) == false);
}

TEST_CASE("BMI270 writeRegisterWithRepeat retries until success")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    // First 3 attempts fail, 4th succeeds
    uint8_t raw[] = {
        0xFF, 0x00,   // fail
        0xFF, 0x00,   // fail
        0xFF, 0x00,   // fail
        0xFF, 0x55    // success
    };
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.writeRegisterWithRepeat(BMI270_REGISTERS::ACC_RANGE, 0x55) == true);
}

TEST_CASE("BMI270 writeRegisterWithRepeat fails after all retries")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    uint8_t raw[16];
    for (int i = 0; i < 16; i += 2)
    {
        raw[i] = 0xFF;     // dummy
        raw[i+1] = 0x00;   // always wrong
    }
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.writeRegisterWithRepeat(BMI270_REGISTERS::ACC_RANGE, 0x55) == false);
}

TEST_CASE("BMI270 configure succeeds when all register writes succeed")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    // Every readback matches
    uint8_t raw[] = {
        0xFF, 0x08,   // ACC_CONF
        0xFF, 0x00,   // ACC_RANGE
        0xFF, 0x08,   // GYR_CONF
        0xFF, 0x00,   // GYR_RANGE
        0xFF, 0x0E    // PWR_CTRL
    };
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.configure() == true);
}

TEST_CASE("BMI270 configure fails when a register write fails")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    // First readback OK, second wrong
    uint8_t raw[] = {
        0xFF, 0x08,   // ACC_CONF OK
        0xFF, 0xFF    // ACC_RANGE wrong
    };
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.configure() == false);
}

TEST_CASE("BMI270 initialize succeeds when chip ID matches and status OK")
{
    clear_spi_rx_buffer();

    uint8_t raw[] = {
        0xFF, 0x00,   // CHIP_ID dummy read
        0xFF, 0xB6,   // readback for CMD writeRegisterWithCheck
        0xFF, 0x24,   // CHIP_ID after reset
        0xFF, 0x24,   // CHIP_ID again
        0xFF, 0x00,   // readback for PWR_CONF
        0xFF, 0x00,   // readback for INIT_CTRL=0
        0xFF, 0x01,   // readback for INIT_CTRL=1
        0xFF, 0x01    // INTERNAL_STATUS OK
    };
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.initialize() == true);
}

TEST_CASE("BMI270 initialize fails when chip ID never matches")
{
    clear_spi_rx_buffer();

    uint8_t raw[64];
    for (int i = 0; i < 64; i += 2)
    {
        raw[i] = 0xFF;
        raw[i+1] = 0x00;   // wrong chip ID
    }
    inject_spi_rx_data(raw, sizeof(raw));

    Config config(&mock_gpio);
    Transport transport(config);
    BMI270<Transport> imu(transport);

    CHECK(imu.initialize() == false);
}
