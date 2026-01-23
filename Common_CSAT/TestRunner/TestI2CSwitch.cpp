#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "I2CSwitch.hpp"
#include "mock_hal.h"
#include "Transport.hpp"

// A mock reset pin using the GpioPin template
constexpr uint32_t MOCK_RESET_PIN_PORT = 0x1243; // Port ignored by mock HAL
using MockResetPin = GpioPin<MOCK_RESET_PIN_PORT, GPIO_PIN_0>;   // Port ignored by mock HAL

TEST_CASE("I2CSwitch channel selection and disable")
{
    static I2C_HandleTypeDef hi2c;
    constexpr uint8_t address = 0x70; // TCA9546A default address

    using SwitchConfig    = I2C_Stream_Config<hi2c, address>;
    using SwitchTransport = I2CStreamTransport<SwitchConfig>;
    SwitchTransport transport;

    // Instantiate switch with mock reset pin
    I2CSwitch<SwitchTransport, MockResetPin> switcher(transport);

    clear_i2c_rx_data();
    clear_i2c_tx_data();
    clear_i2c_addresses();
    reset_gpio_port_state(nullptr);

    SUBCASE("Reset pin defaults high after construction")
    {
        CHECK(get_gpio_pin_state(nullptr, GPIO_PIN_0) == GPIO_PIN_RESET);
    }

    SUBCASE("Status readback returns last read value")
    {
        clear_i2c_rx_data();

        uint8_t expected = 0xAB;
        inject_i2c_rx_data(address << 1, &expected, 1);

        uint8_t result = 0;
        CHECK(switcher.status(result) == true);
        CHECK(result == expected);
    }

    SUBCASE("Select Channel 0")
    {
        CHECK(switcher.select(I2CSwitchChannel::Channel0) == true);
        CHECK(get_i2c_tx_buffer_count() == 1);
        CHECK(get_i2c_tx_buffer()[0] == 0x01);
    }

    SUBCASE("Select Channel 1")
    {
        CHECK(switcher.select(I2CSwitchChannel::Channel1) == true);
        CHECK(get_i2c_tx_buffer_count() == 1);
        CHECK(get_i2c_tx_buffer()[0] == 0x02);
    }

    SUBCASE("Select Channel 2")
    {
        CHECK(switcher.select(I2CSwitchChannel::Channel2) == true);
        CHECK(get_i2c_tx_buffer_count() == 1);
        CHECK(get_i2c_tx_buffer()[0] == 0x04);
    }

    SUBCASE("Select Channel 3")
    {
        CHECK(switcher.select(I2CSwitchChannel::Channel3) == true);
        CHECK(get_i2c_tx_buffer_count() == 1);
        CHECK(get_i2c_tx_buffer()[0] == 0x08);
    }

    SUBCASE("Disable all channels")
    {
        CHECK(switcher.disableAll() == true);
        CHECK(get_i2c_tx_buffer_count() == 1);
        CHECK(get_i2c_tx_buffer()[0] == 0x00);
    }

    SUBCASE("Select None explicitly")
    {
        CHECK(switcher.select(I2CSwitchChannel::None) == true);
        CHECK(get_i2c_tx_buffer_count() == 1);
        CHECK(get_i2c_tx_buffer()[0] == 0x00);
    }

    SUBCASE("Reset pin goes high on releaseReset")
    {
        switcher.holdReset();
        switcher.releaseReset();
        CHECK(get_gpio_pin_state(nullptr, GPIO_PIN_0) == GPIO_PIN_SET);
    }

    SUBCASE("Reset pin goes low on holdReset")
    {
        switcher.releaseReset();
        switcher.holdReset();
        CHECK(get_gpio_pin_state(nullptr, GPIO_PIN_0) == GPIO_PIN_RESET);
    }
}
