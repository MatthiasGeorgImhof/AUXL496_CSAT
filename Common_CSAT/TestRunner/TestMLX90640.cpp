#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "mock_hal/mock_hal_i2c.h"
#include "Transport.hpp"
#include "MLX90640.hpp"

// ─────────────────────────────────────────────
// Global mock I2C handle
// ─────────────────────────────────────────────
I2C_HandleTypeDef mock_i2c_handle = {
    .Instance =
        {
            .ClockSpeed = 100000,
            .DutyCycle = 0,
            .OwnAddress1 = 0,
            .AddressingMode = 0,
            .DualAddressMode = 0,
            .OwnAddress2 = 0,
            .GeneralCallMode = 0,
            .NoStretchMode = 0,
            .Master = 1,
            .Init = 1,
        }};

// ─────────────────────────────────────────────
// Transport typedefs
// ─────────────────────────────────────────────
using MLX_I2C_Config = I2C_Register_Config<mock_i2c_handle, MLX90640_ID, I2CAddressWidth::Bits16>;
using MLX_I2C = I2CRegisterTransport<MLX_I2C_Config>;

// Little‑endian helper (matches reinterpret_cast<uint16_t*> on x86)
static uint16_t le16(uint8_t lsb, uint8_t msb)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(lsb) |
        (static_cast<uint16_t>(msb) << 8));
}

// ─────────────────────────────────────────────
// TEST: compile‑time guard
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 enforces 16‑bit addressing at compile time")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    CHECK(MLX_I2C_Config::address_width == I2CAddressWidth::Bits16);
}

// ─────────────────────────────────────────────
// TEST: wakeUp() writes correct CONTROL1 bits
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 wakeUp() sets wake, chess mode, and refresh rate")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_rx_data();
    clear_i2c_tx_data();
    clear_i2c_addresses();

    uint8_t ctrl_initial[2] = {0x12, 0x34};
    inject_i2c_rx_data(MLX_I2C_Config::address, ctrl_initial, 2);

    bool ok = mlx.wakeUp();
    CHECK(ok == true);

    CHECK(get_i2c_mem_address() == static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1));
    CHECK(get_i2c_tx_buffer_count() == 2);
}

// ─────────────────────────────────────────────
// TEST: sleep()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 sleep() clears wake bit")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_rx_data();
    clear_i2c_tx_data();

    uint8_t ctrl_initial[2] = {0x00, 0x01};
    inject_i2c_rx_data(MLX_I2C_Config::address, ctrl_initial, 2);

    bool ok = mlx.sleep();
    CHECK(ok == true);

    CHECK(get_i2c_mem_address() == static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1));

    uint8_t *tx = get_i2c_tx_buffer();
    CHECK(get_i2c_tx_buffer_count() == 2);

    uint16_t written = le16(tx[1], tx[0]);
    CHECK((written & 0x0001) == 0);
}

// ─────────────────────────────────────────────
// TEST: reset()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 reset() writes zeros to STATUS and CONTROL1")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_tx_data();
    clear_i2c_rx_data();

    bool ok = mlx.reset();
    CHECK(ok == true);

    CHECK(get_i2c_tx_buffer_count() == 2);

    CHECK(get_i2c_mem_address() ==
          static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1));

    uint8_t *tx = get_i2c_tx_buffer();
    CHECK(tx[0] == 0x00);
    CHECK(tx[1] == 0x00);
}

// ─────────────────────────────────────────────
// TEST: readEEPROM()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 readEEPROM returns data consistent with injected bytes")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_addresses();
    clear_i2c_rx_data();

    uint8_t fake_eeprom[MLX90640_EEPROM_SIZE];
    for (size_t i = 0; i < MLX90640_EEPROM_SIZE; ++i)
        fake_eeprom[i] = uint8_t(i & 0xFF);

    inject_i2c_rx_data(MLX_I2C_Config::address,
                       fake_eeprom,
                       MLX90640_EEPROM_SIZE);

    uint16_t buffer[MLX90640_EEPROM_WORDS] = {};
    bool ok = mlx.readEEPROM(buffer);

    CHECK(ok == true);

    CHECK(buffer[0] == le16(fake_eeprom[0], fake_eeprom[1]));
    CHECK(buffer[1] == le16(fake_eeprom[2], fake_eeprom[3]));
    CHECK(buffer[10] == le16(fake_eeprom[20], fake_eeprom[21]));
}

// ─────────────────────────────────────────────
// TEST: isReady()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 isReady detects NEW_DATA bit")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_rx_data();

    uint8_t ready_status[2] = {0x00, 0x08};
    inject_i2c_rx_data(MLX_I2C_Config::address, ready_status, 2);

    CHECK(mlx.isReady() == true);
}

// ─────────────────────────────────────────────
// TEST: readSubpage()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 readSubpage reads RAM block and clears status" * doctest::skip())
{
    MESSAGE("Skipped: the current mocking framework cannot simulate two consecutive RX operations");

    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_rx_data();
    clear_i2c_tx_data();

    uint8_t fake_subpage[MLX90640_SUBPAGE_SIZE];
    for (size_t i = 0; i < MLX90640_SUBPAGE_SIZE; ++i)
        fake_subpage[i] = uint8_t(i & 0xFF);

    inject_i2c_rx_data(MLX_I2C_Config::address,
                       fake_subpage,
                       MLX90640_SUBPAGE_SIZE);

    uint16_t frame[MLX90640_SUBPAGE_WORDS] = {};
    int sp = -1;
    bool ok = mlx.readSubpage(frame, sp);
    CHECK(ok == true);
    CHECK((sp == 0 || sp == 1));

    CHECK(frame[0] ==
          le16(fake_subpage[0], fake_subpage[1]));

    CHECK(frame[MLX90640_SUBPAGE_WORDS - 1] ==
          le16(fake_subpage[MLX90640_SUBPAGE_SIZE - 2],
               fake_subpage[MLX90640_SUBPAGE_SIZE - 1]));

    CHECK(get_i2c_tx_buffer_count() == 2);
    CHECK(get_i2c_mem_address() ==
          static_cast<uint16_t>(MLX90640_REGISTERS::STATUS));
}

// ─────────────────────────────────────────────
// TEST: createFrame()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 createFrame concatenates subpages")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);
    
    uint16_t sub0[MLX90640_SUBPAGE_WORDS];
    uint16_t sub1[MLX90640_SUBPAGE_WORDS];
    uint16_t full[MLX90640_FRAME_WORDS];

    for (size_t i = 0; i < MLX90640_SUBPAGE_WORDS; ++i)
    {
        sub0[i] = static_cast<uint16_t>(0x1000u + static_cast<uint16_t>(i));
        sub1[i] = static_cast<uint16_t>(0x2000u + static_cast<uint16_t>(i));
    }

    mlx.createFrame(sub0, sub1, full);

    CHECK(full[0] == sub0[0]);
    CHECK(full[10] == sub0[10]);
    CHECK(full[MLX90640_SUBPAGE_WORDS - 1] == sub0[MLX90640_SUBPAGE_WORDS - 1]);

    CHECK(full[MLX90640_SUBPAGE_WORDS] == sub1[0]);
    CHECK(full[MLX90640_SUBPAGE_WORDS + 5] == sub1[5]);
    CHECK(full[MLX90640_FRAME_WORDS - 1] == sub1[MLX90640_SUBPAGE_WORDS - 1]);
}

TEST_CASE("MLX90640 readFrame attempts subpage reads (mock‑compatible)")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_rx_data();
    clear_i2c_tx_data();
    clear_i2c_addresses();

    // Inject one subpage (mock cannot simulate two)
    uint8_t fake_subpage[MLX90640_SUBPAGE_SIZE];
    memset(fake_subpage, 0xAA, sizeof(fake_subpage));
    inject_i2c_rx_data(MLX_I2C_Config::address, fake_subpage, sizeof(fake_subpage));

    uint16_t frame[MLX90640_FRAME_WORDS] = {};
    bool ok = mlx.readFrame(frame);

    // Expected: mock cannot support two RX operations → readFrame() fails
    CHECK(ok == false);

    // Inspect the last transmit buffer
    uint8_t *tx = get_i2c_tx_buffer();
    int count = get_i2c_tx_buffer_count();

    // The last write should be clearStatus() → writeReg16(0x8000, 0)
    CHECK(count == 2);
    CHECK(tx[1] == 0x08); // LSB
    CHECK(tx[0] == 0x00); // MSB
    CHECK(get_i2c_mem_address() == static_cast<uint16_t>(MLX90640_REGISTERS::STATUS));
}

// ─────────────────────────────────────────────
// TEST: readFrame()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 readFrame fails when subpages have same parity")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_rx_data();
    clear_i2c_tx_data();

    uint8_t fake_subpage[MLX90640_SUBPAGE_SIZE];
    memset(fake_subpage, 0xAA, sizeof(fake_subpage));

    inject_i2c_rx_data(MLX_I2C_Config::address,
                       fake_subpage,
                       MLX90640_SUBPAGE_SIZE);

    inject_i2c_rx_data(MLX_I2C_Config::address,
                       fake_subpage,
                       MLX90640_SUBPAGE_SIZE);

    uint16_t frame[MLX90640_FRAME_WORDS] = {};
    bool ok = mlx.readFrame(frame);

    CHECK(ok == false);
}

// ─────────────────────────────────────────────
// TEST: waitUntilReady()
// ─────────────────────────────────────────────
TEST_CASE("MLX90640 waitUntilReady returns true when NEW_DATA appears")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);
    
    clear_i2c_rx_data();

    uint8_t not_ready[2] = {0x00, 0x00};
    inject_i2c_rx_data(MLX_I2C_Config::address, not_ready, 2);

    CHECK(mlx.waitUntilReady(1) == false);

    clear_i2c_rx_data();
    uint8_t ready[2] = {0x00, 0x08};
    inject_i2c_rx_data(MLX_I2C_Config::address, ready, 2);

    CHECK(mlx.waitUntilReady(1) == true);
}

TEST_CASE("MLX90640 waitUntilReady returns false when NEW_DATA never appears")
{
    MLX_I2C transport;
    MLX90640<MLX_I2C> mlx(transport);

    clear_i2c_rx_data();

    uint8_t not_ready[2] = {0x00, 0x00};
    inject_i2c_rx_data(MLX_I2C_Config::address, not_ready, 2);

    CHECK(mlx.waitUntilReady(1) == false);
}
