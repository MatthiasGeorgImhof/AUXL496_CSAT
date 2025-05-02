#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "PowerSwitch.hpp"
#include "mock_hal.h"

#include <iostream>  // For debugging output (remove in production tests)
#include <bitset>    // For visualizing bit patterns

TEST_CASE("PowerSwitch On/Off/Status") {
    I2C_HandleTypeDef hi2c;
    uint8_t address = 0x40;

    PowerSwitch pm(&hi2c, address);

    // Reset the I2C buffer before each test case
    clear_i2c_mem_data();

    SUBCASE("Turn on slot 0") {
        pm.on(0);

        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b00000001); // Bit 0 should be set
        CHECK(pm.status(0) == true);
    }

    SUBCASE("Turn off slot 0") {
        // First, turn it on
        pm.on(0);
        pm.off(0);

        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b00000000); // Bit 0 should be cleared
        CHECK(pm.status(0) == false);
    }

    SUBCASE("Turn on slot 1") {
        pm.on(1);

        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b00000100); // Bit 2 should be set
        CHECK(pm.status(1) == true);
    }

    SUBCASE("Turn on slot 2") {
        pm.on(2);
        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b00010000); // Bit 4 should be set
        CHECK(pm.status(2) == true);
    }

    SUBCASE("Turn on slot 3") {
        pm.on(3);
        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b01000000); // Bit 6 should be set
        CHECK(pm.status(3) == true);
    }

    SUBCASE("Turn on and off multiple slots") {
        pm.on(0);
        pm.on(2);
        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b00010001);
        CHECK(pm.status(0) == true);
        CHECK(pm.status(2) == true);

        pm.off(0);
        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b00010000);
        CHECK(pm.status(0) == false);
        CHECK(pm.status(2) == true);

        pm.off(2);
        CHECK(get_i2c_buffer_count() == 1);
        CHECK(get_i2c_buffer()[0] == 0b00000000);
        CHECK(pm.status(0) == false);
        CHECK(pm.status(2) == false);
    }

    SUBCASE("Invalid slot check") {
        CHECK(get_i2c_buffer_count() == 0); // I2C transactions cancelled
        pm.on(4);  // Invalid slot
        CHECK(get_i2c_buffer()[0] == 0b00000000); //register_value_ should still be 0
        CHECK(pm.status(4) == false);
    }

    SUBCASE("Initial status is off") {
        CHECK(pm.status(0) == false);
        CHECK(pm.status(1) == false);
        CHECK(pm.status(2) == false);
        CHECK(pm.status(3) == false);
    }

}