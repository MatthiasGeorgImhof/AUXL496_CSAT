#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "PowerMonitor.hpp"
#include "mock_hal.h"

// ─────────────────────────────────────────────
// Test Fixture
// ─────────────────────────────────────────────
struct PowerMonitorFixture
{
    static inline I2C_HandleTypeDef hi2c{};
    static constexpr uint8_t address = 0x40;

    using Config    = I2C_Register_Config<hi2c, address>;
    using Transport = I2CRegisterTransport<Config>;

    Transport transport;

    // NOTE: monitor is NOT constructed here — tests construct it explicitly
    PowerMonitorFixture()
    {
        clear_i2c_addresses();
        clear_i2c_rx_data();
    }

    void inject16(uint16_t value)
    {
        uint8_t data[2] = {
            static_cast<uint8_t>((value >> 8U) & 0xFFU),
            static_cast<uint8_t>( value        & 0xFFU)
        };
        inject_i2c_rx_data(address << 1, data, 2);
    }
};

// ─────────────────────────────────────────────
// Constructor Behavior
// ─────────────────────────────────────────────
TEST_CASE_FIXTURE(PowerMonitorFixture,
                  "Constructor writes calibration register")
{
    clear_i2c_addresses();
    clear_i2c_rx_data();

    PowerMonitor<Transport> monitor(transport);

    uint16_t expected_cal = 5120000U / (10U * 25U);
    uint8_t msb = static_cast<uint8_t>((expected_cal >> 8U) & 0xFFU);
    uint8_t lsb = static_cast<uint8_t>( expected_cal        & 0xFFU);

    CHECK_EQ(get_i2c_tx_buffer_count(), 2);
    CHECK_EQ(get_i2c_mem_address(),
             static_cast<uint8_t>(INA226_REGISTERS::INA226_CALIBRATION));
    CHECK_EQ(get_i2c_tx_buffer()[0], msb);
    CHECK_EQ(get_i2c_tx_buffer()[1], lsb);
}

// ─────────────────────────────────────────────
// reset() Behavior
// ─────────────────────────────────────────────
TEST_CASE_FIXTURE(PowerMonitorFixture,
                  "reset() writes configuration and calibration registers")
{
    PowerMonitor<Transport> monitor(transport);

    clear_i2c_addresses();
    clear_i2c_tx_data();

    CHECK(monitor.reset());

    // Last write should be CALIBRATION
    CHECK_EQ(get_i2c_mem_address(),
             static_cast<uint8_t>(INA226_REGISTERS::INA226_CALIBRATION));
    CHECK_EQ(get_i2c_tx_buffer_count(), 2);
}

// ─────────────────────────────────────────────
// Getter Scaling Tests
// ─────────────────────────────────────────────
TEST_CASE_FIXTURE(PowerMonitorFixture,
                  "Getter functions return scaled values")
{
    PowerMonitor<Transport> monitor(transport);

    SUBCASE("Shunt Voltage")
    {
        uint16_t raw = 100;
        inject16(raw);

        uint16_t out{};
        CHECK(monitor.getShuntVoltage(out));
        CHECK_EQ(out, 5U * raw / 2U);

        CHECK_EQ(get_i2c_mem_address(),
                 static_cast<uint8_t>(INA226_REGISTERS::INA226_SHUNT_VOLTAGE));
    }

    SUBCASE("Bus Voltage")
    {
        uint16_t raw = 200;
        inject16(raw);

        uint16_t out{};
        CHECK(monitor.getBusVoltage(out));
        CHECK_EQ(out, 5U * raw / 4U);

        CHECK_EQ(get_i2c_mem_address(),
                 static_cast<uint8_t>(INA226_REGISTERS::INA226_BUS_VOLTAGE));
    }

    SUBCASE("Power")
    {
        uint16_t raw = 75;
        inject16(raw);

        uint16_t out{};
        CHECK(monitor.getPower(out));
        CHECK_EQ(out, raw * 25U * 25U);

        CHECK_EQ(get_i2c_mem_address(),
                 static_cast<uint8_t>(INA226_REGISTERS::INA226_POWER));
    }

    SUBCASE("Current")
    {
        uint16_t raw = 400;
        inject16(raw);

        uint16_t out{};
        CHECK(monitor.getCurrent(out));
        CHECK_EQ(out, raw * 25U);

        CHECK_EQ(get_i2c_mem_address(),
                 static_cast<uint8_t>(INA226_REGISTERS::INA226_CURRENT));
    }

    SUBCASE("Manufacturer ID")
    {
        uint16_t raw = 0x1234;
        inject16(raw);

        uint16_t out{};
        CHECK(monitor.getManufacturerId(out));
        CHECK_EQ(out, raw);

        CHECK_EQ(get_i2c_mem_address(),
                 static_cast<uint8_t>(INA226_REGISTERS::INA226_MANUFACTURER));
    }

    SUBCASE("Die ID")
    {
        uint16_t raw = 0x5678;
        inject16(raw);

        uint16_t out{};
        CHECK(monitor.getDieId(out));
        CHECK_EQ(out, raw);

        CHECK_EQ(get_i2c_mem_address(),
                 static_cast<uint8_t>(INA226_REGISTERS::INA226_DIE_ID));
    }
}

// ─────────────────────────────────────────────
// setConfig() Behavior
// ─────────────────────────────────────────────
TEST_CASE_FIXTURE(PowerMonitorFixture,
                  "setConfig writes correct register and payload")
{
    PowerMonitor<Transport> monitor(transport);

    uint16_t cfg = 0x1234U;
    uint8_t msb = static_cast<uint8_t>((cfg >> 8U) & 0xFFU);
    uint8_t lsb = static_cast<uint8_t>( cfg        & 0xFFU);

    monitor.setConfig(cfg);

    CHECK_EQ(get_i2c_dev_address(), address << 1);
    CHECK_EQ(get_i2c_mem_address(),
             static_cast<uint8_t>(INA226_REGISTERS::INA226_CONFIGURATION));
    CHECK_EQ(get_i2c_tx_buffer_count(), 2);
    CHECK_EQ(get_i2c_tx_buffer()[0], msb);
    CHECK_EQ(get_i2c_tx_buffer()[1], lsb);
}

// ─────────────────────────────────────────────
// Overflow Handling
// ─────────────────────────────────────────────
TEST_CASE_FIXTURE(PowerMonitorFixture,
                  "checkAndCast clamps values to uint16_t max")
{
    PowerMonitor<Transport> monitor(transport);

    SUBCASE("Shunt Voltage")
    {
        uint8_t data[2] = {0x7F, 0xFF};
        inject_i2c_rx_data(address << 1, data, 2);

        uint16_t out{};
        CHECK(monitor.getShuntVoltage(out));
        CHECK_EQ(out, std::numeric_limits<uint16_t>::max());
    }

    SUBCASE("Bus Voltage")
    {
        uint8_t data[2] = {0xFF, 0xFF};
        inject_i2c_rx_data(address << 1, data, 2);

        uint16_t out{};
        CHECK(monitor.getBusVoltage(out));
        CHECK_EQ(out, std::numeric_limits<uint16_t>::max());
    }

    SUBCASE("Power")
    {
        uint8_t data[2] = {0xFF, 0xFF};
        inject_i2c_rx_data(address << 1, data, 2);

        uint16_t out{};
        CHECK(monitor.getPower(out));
        CHECK_EQ(out, std::numeric_limits<uint16_t>::max());
    }

    SUBCASE("Current")
    {
        uint8_t data[2] = {0xFF, 0xFF};
        inject_i2c_rx_data(address << 1, data, 2);

        uint16_t out{};
        CHECK(monitor.getCurrent(out));
        CHECK_EQ(out, std::numeric_limits<uint16_t>::max());
    }
}

// ─────────────────────────────────────────────
// I2C Write Failure Placeholder
// ─────────────────────────────────────────────
TEST_CASE_FIXTURE(PowerMonitorFixture,
                  "I2C Write Failure (placeholder)")
{
    PowerMonitor<Transport> monitor(transport);

    uint16_t cfg = 0x1234U;
    monitor.setConfig(cfg);

    CHECK(true); // mock HAL cannot simulate write failure yet
}
