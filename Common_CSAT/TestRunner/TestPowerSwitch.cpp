#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "PowerSwitch.hpp"
#include "mock_hal.h"
#include "Transport.hpp"

GPIO_TypeDef mock_gpio_port{};
constexpr uint16_t mock_gpio_pin = GPIO_PIN_0;

TEST_CASE("PowerSwitch On/Off/Status")
{
    static I2C_HandleTypeDef hi2c;
    constexpr uint8_t address = 0x40;

    using SwitchConfig =
        I2C_Register_Config<hi2c, address, I2CAddressWidth::Bits8>;

    using SwitchTransport =
        I2CRegisterTransport<SwitchConfig>;

    SwitchTransport transport;

    clear_i2c_addresses();
    clear_i2c_tx_data();
    clear_i2c_rx_data();

    PowerSwitch<SwitchTransport> pm(transport, &mock_gpio_port, mock_gpio_pin);

    auto last_payload = [&]() -> const uint8_t* {
        return get_i2c_tx_buffer();
    };

    auto payload_count = [&]() -> int {
        return get_i2c_tx_buffer_count();
    };

    auto last_reg = [&]() -> uint8_t {
        return static_cast<uint8_t>(get_i2c_mem_address());
    };

    SUBCASE("Turn on slot 0")
    {
        pm.on(CIRCUITS::CIRCUIT_0);

        CHECK(last_reg() == static_cast<uint8_t>(MCP23008_REGISTERS::MCP23008_OLAT));
        CHECK(payload_count() == 1);
        CHECK(last_payload()[0] == 0b00000001);
        CHECK(pm.status(CIRCUITS::CIRCUIT_0));
    }

    SUBCASE("Turn off slot 0")
    {
        pm.on(CIRCUITS::CIRCUIT_0);
        pm.off(CIRCUITS::CIRCUIT_0);

        CHECK(last_reg() == static_cast<uint8_t>(MCP23008_REGISTERS::MCP23008_OLAT));
        CHECK(payload_count() == 1);
        CHECK(last_payload()[0] == 0b00000000);
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_0));
    }

    SUBCASE("Turn on slot 1")
    {
        pm.on(CIRCUITS::CIRCUIT_1);

        CHECK(last_reg() == static_cast<uint8_t>(MCP23008_REGISTERS::MCP23008_OLAT));
        CHECK(payload_count() == 1);
        CHECK(last_payload()[0] == 0b00000010);
        CHECK(pm.status(CIRCUITS::CIRCUIT_1));
    }

    SUBCASE("Turn on slot 2")
    {
        pm.on(CIRCUITS::CIRCUIT_2);

        CHECK(last_reg() == static_cast<uint8_t>(MCP23008_REGISTERS::MCP23008_OLAT));
        CHECK(payload_count() == 1);
        CHECK(last_payload()[0] == 0b00000100);
        CHECK(pm.status(CIRCUITS::CIRCUIT_2));
    }

    SUBCASE("Turn on and off multiple slots")
    {
        pm.on(CIRCUITS::CIRCUIT_0);
        pm.on(CIRCUITS::CIRCUIT_2);

        CHECK(last_payload()[0] == 0b00000101);

        pm.off(CIRCUITS::CIRCUIT_0);
        CHECK(last_payload()[0] == 0b00000100);

        pm.off(CIRCUITS::CIRCUIT_2);
        CHECK(last_payload()[0] == 0b00000000);
    }

    SUBCASE("Initial status is off")
    {
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_0));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_1));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_2));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_3));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_4));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_5));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_6));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_7));
    }

    SUBCASE("Set state with bitmask")
    {
        pm.setState(0b10101010);

        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_0));
        CHECK(pm.status(CIRCUITS::CIRCUIT_1));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_2));
        CHECK(pm.status(CIRCUITS::CIRCUIT_3));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_4));
        CHECK(pm.status(CIRCUITS::CIRCUIT_5));
        CHECK_FALSE(pm.status(CIRCUITS::CIRCUIT_6));
        CHECK(pm.status(CIRCUITS::CIRCUIT_7));
    }

    SUBCASE("Get state reads from OLAT")
    {
        uint8_t response = 0b11001100;
        inject_i2c_rx_data(address << 1, &response, 1);

        auto state = pm.getState();
        CHECK(state == 0b11001100);
    }

    SUBCASE("Reset pin is set high on releaseReset")
    {
        set_gpio_pin_state(&mock_gpio_port, mock_gpio_pin, GPIO_PIN_RESET);
        pm.releaseReset();
        CHECK(get_gpio_pin_state(&mock_gpio_port, mock_gpio_pin) == GPIO_PIN_SET);
    }

    SUBCASE("Reset pin is set low on holdReset")
    {
        set_gpio_pin_state(&mock_gpio_port, mock_gpio_pin, GPIO_PIN_SET);
        pm.holdReset();
        CHECK(get_gpio_pin_state(&mock_gpio_port, mock_gpio_pin) == GPIO_PIN_RESET);
    }
}
