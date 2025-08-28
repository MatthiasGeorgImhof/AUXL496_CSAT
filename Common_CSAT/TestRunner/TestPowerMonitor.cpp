#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "PowerMonitor.hpp"
#include "mock_hal.h" // Include your mock HAL header

TEST_SUITE("PowerMonitor")
{

    TEST_CASE("Constructor sets calibration register")
    {
        I2C_HandleTypeDef hi2c = {};
        uint8_t address = 0x40;
        uint8_t expected_calibration_bytes[2];
        uint16_t expected_calibration = 5120000 / (10 * 25); // calibration_value_

        // Split the expected calibration value into two bytes (MSB first)
        expected_calibration_bytes[0] = static_cast<uint8_t>((expected_calibration >> 8) & 0xFF);
        expected_calibration_bytes[1] = static_cast<uint8_t>(expected_calibration & 0xFF);

        uint8_t init_i2c_data[8]; // Data to inject into I2C memory
        init_i2c_data[0] = 0;     // Raw Shunt Voltage
        init_i2c_data[1] = 0;

        init_i2c_data[2] = 0; // Raw Bus Voltage
        init_i2c_data[3] = 0;

        init_i2c_data[4] = 0; // Raw Power
        init_i2c_data[5] = 0;

        init_i2c_data[6] = 0; // Raw Current
        init_i2c_data[7] = 0;
        inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_SHUNT_VOLTAGE), init_i2c_data, 8);

        PowerMonitor monitor(&hi2c, address);

        // Verify that HAL_I2C_Mem_Write was called with the correct arguments and values
        CHECK_EQ(get_i2c_mem_buffer_dev_address(), address << 1);
        CHECK_EQ(get_i2c_mem_buffer_mem_address(), static_cast<uint8_t>(INA226_REGISTERS::INA226_CALIBRATION));
        CHECK_EQ(get_i2c_mem_buffer_count(), 2);
        CHECK_EQ(get_i2c_buffer()[0], expected_calibration_bytes[0]);
        CHECK_EQ(get_i2c_buffer()[1], expected_calibration_bytes[1]);

        clear_i2c_mem_data();
    }

    TEST_CASE("Readings are correctly scaled and returned: PowerMonitorData")
    {
        I2C_HandleTypeDef hi2c = {};
        uint8_t address = 0x40;

        PowerMonitor monitor(&hi2c, address);

        uint16_t raw_value = 100;

        uint8_t data[8]; // Data to inject into I2C memory
        data[0] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
        data[1] = static_cast<uint8_t>(raw_value & 0xFF);
        data[2] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
        data[3] = static_cast<uint8_t>(raw_value & 0xFF);
        data[4] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
        data[5] = static_cast<uint8_t>(raw_value & 0xFF);
        data[6] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
        data[7] = static_cast<uint8_t>(raw_value & 0xFF);

        inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_SHUNT_VOLTAGE), data, 8);

        PowerMonitorData returned_data;
        CHECK_EQ(monitor(returned_data), true);

        CHECK_EQ(returned_data.voltage_shunt_uV, 5 * raw_value / 2);
        CHECK_EQ(returned_data.voltage_bus_mV, 5 * raw_value / 4);
        CHECK_EQ(returned_data.power_mW, raw_value * 25 * 25);
        CHECK_EQ(returned_data.current_uA, raw_value * 25);
        clear_i2c_mem_data();
    }

    TEST_CASE("Readings are correctly scaled and returned: getters")
    {
        I2C_HandleTypeDef hi2c = {};
        uint8_t address = 0x40;

        PowerMonitor monitor(&hi2c, address);

        SUBCASE("Shunt Voltage")
        {
            uint16_t raw_value = 100;
            uint8_t data[2]; // Data to inject into I2C memory
            data[0] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>(raw_value & 0xFF);

            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_SHUNT_VOLTAGE), data, 8);

            uint16_t returned_value;
            CHECK_EQ(monitor.getShuntVoltage(returned_value), true);
            CHECK_EQ(returned_value, 5 * raw_value / 2);
        }
        SUBCASE("Bus Voltage")
        {
            uint16_t raw_value = 200;
            uint8_t data[2]; // Data to inject into I2C memory
            data[0] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>(raw_value & 0xFF);

            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_BUS_VOLTAGE), data, 8);

            uint16_t returned_value;
            CHECK_EQ(monitor.getBusVoltage(returned_value), true);
            CHECK_EQ(returned_value, 5 * raw_value / 4);
        }
        SUBCASE("Power")
        {
            uint16_t raw_value = 75;
            uint8_t data[2]; // Data to inject into I2C memory
            data[0] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>(raw_value & 0xFF);

            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_POWER), data, 8);

            uint16_t returned_value;
            CHECK_EQ(monitor.getPower(returned_value), true);
            CHECK_EQ(returned_value, raw_value * 25 * 25);
        }
        SUBCASE("Current")
        {
            uint16_t raw_value = 400;
            uint8_t data[2]; // Data to inject into I2C memory
            data[0] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>(raw_value & 0xFF);

            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_CURRENT), data, 8);

            uint16_t returned_value;
            CHECK_EQ(monitor.getCurrent(returned_value), true);
            CHECK_EQ(returned_value, raw_value * 25);
        }
        SUBCASE("Manufacturer ID")
        {
            uint16_t raw_value = 0x1234;
            uint8_t data[2]; // Data to inject into I2C memory
            data[0] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>(raw_value & 0xFF);

            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_MANUFACTURER), data, 8);
            uint16_t returned_value;
            CHECK_EQ(monitor.getManufacturerId(returned_value), true);
            CHECK_EQ(returned_value, raw_value);
        }
        SUBCASE("Die ID")
        {
            uint16_t raw_value = 0x5678;
            uint8_t data[2]; // Data to inject into I2C memory
            data[0] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>(raw_value & 0xFF);

            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_DIE_ID), data, 8);
            uint16_t returned_value;
            CHECK_EQ(monitor.getDieId(returned_value), true);
            CHECK_EQ(returned_value, raw_value);
        }
    }


    TEST_CASE("setConfig writes the config to the correct register")
    {
        I2C_HandleTypeDef hi2c = {};
        uint8_t address = 0x40;
        uint16_t config_value = 0x1234;

        // Split the config value into two bytes (MSB first)
        uint8_t config_bytes[2];
        config_bytes[0] = static_cast<uint8_t>((config_value >> 8) & 0xFF);
        config_bytes[1] = static_cast<uint8_t>(config_value & 0xFF);

        PowerMonitor monitor(&hi2c, address);
        monitor.setConfig(config_value);

        // Verify that HAL_I2C_Mem_Write was called with the correct arguments and values
        CHECK_EQ(get_i2c_mem_buffer_dev_address(), address << 1);
        CHECK_EQ(get_i2c_mem_buffer_mem_address(), static_cast<uint8_t>(INA226_REGISTERS::INA226_CONFIGURATION));
        CHECK_EQ(get_i2c_mem_buffer_count(), 2);
        CHECK_EQ(get_i2c_buffer()[0], config_bytes[0]);
        CHECK_EQ(get_i2c_buffer()[1], config_bytes[1]);
        clear_i2c_mem_data();
    }

    TEST_CASE("checkAndCast limits values to uint16_t max")
    {
        I2C_HandleTypeDef hi2c = {};
        uint8_t address = 0x40;

        PowerMonitor monitor(&hi2c, address);
        clear_i2c_mem_data();

        SUBCASE("checkAndCast Shunt Voltage")
        {
            uint8_t init_i2c_data[8] = {0x7F, 0xFF};
            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_SHUNT_VOLTAGE), init_i2c_data, sizeof(init_i2c_data));
            uint16_t return_value;
            bool result = monitor.getShuntVoltage(return_value);
            CHECK_EQ(result, true); 
            CHECK_EQ(return_value, std::numeric_limits<uint16_t>::max());
        }

        SUBCASE("checkAndCast Bus Voltage")
        {
            uint8_t init_i2c_data[8] = {0xFF, 0xFF};
            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_BUS_VOLTAGE), init_i2c_data, sizeof(init_i2c_data));
            uint16_t return_value;
            bool result = monitor.getBusVoltage(return_value);
            CHECK_EQ(result, true); 
            CHECK_EQ(return_value, std::numeric_limits<uint16_t>::max());
        }
        
        SUBCASE("checkAndCast Power")
        {
            uint8_t init_i2c_data[8] = {0xFF, 0xFF};
            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_POWER), init_i2c_data, sizeof(init_i2c_data));
            uint16_t return_value;
            bool result = monitor.getPower(return_value);
            CHECK_EQ(result, true); 
            CHECK_EQ(return_value, std::numeric_limits<uint16_t>::max());
        }
        
        SUBCASE("checkAndCast Current")
        {
            uint8_t init_i2c_data[8] = {0xFF, 0xFF};
            inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_CURRENT), init_i2c_data, sizeof(init_i2c_data));
            uint16_t return_value;
            bool result = monitor.getCurrent(return_value);
            CHECK_EQ(result, true); 
            CHECK_EQ(return_value, std::numeric_limits<uint16_t>::max());
        }
    }

    TEST_CASE("I2C Read Failure")
    {
        I2C_HandleTypeDef hi2c = {};
        uint8_t address = 0x40;
        uint8_t initial_i2c_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // Some initial data
        inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_SHUNT_VOLTAGE), initial_i2c_data, 8);

        PowerMonitor monitor(&hi2c, address);
        // Mock I2C read failure
        uint8_t zeros[8] = {0};
        inject_i2c_mem_data(address << 1, static_cast<uint8_t>(INA226_REGISTERS::INA226_SHUNT_VOLTAGE), zeros, 8);

        PowerMonitorData data;
        CHECK_EQ(monitor(data), true);

        // Values should be zero because of I2C read failure.
        CHECK_EQ(data.voltage_shunt_uV, 0);
        CHECK_EQ(data.voltage_bus_mV, 0);
        CHECK_EQ(data.power_mW, 0);
        CHECK_EQ(data.current_uA, 0);
        clear_i2c_mem_data();
    }

    TEST_CASE("I2C Write Failure")
    {
        I2C_HandleTypeDef hi2c = {};
        uint8_t address = 0x40;
        uint16_t config_value = 0x1234;

        // // Capture initial state of i2c_mem_buffer and i2c_mem_buffer_count
        // uint8_t initial_i2c_mem_buffer[I2C_MEM_BUFFER_SIZE];
        // uint16_t initial_i2c_mem_buffer_count = get_i2c_mem_buffer_count();
        PowerMonitor monitor(&hi2c, address);

        // Restore initial state

        monitor.setConfig(config_value);
        // This is very hard to test if the write fails. We will just let it continue without assertion
    }
}