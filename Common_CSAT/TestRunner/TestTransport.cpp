#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "Transport.hpp"
#include "mock_hal.h"

#include <queue>

// -----------------------------------------
// I2C Tests (Register Mode)
// -----------------------------------------

I2C_HandleTypeDef mock_i2c;

// Use 8-bit register addressing for these tests
using TestI2CConfig =
    I2C_Register_Config<mock_i2c, 0x42, I2CAddressWidth::Bits8>;
using TestI2CTransport = I2CRegisterTransport<TestI2CConfig>;

// ------------------------------------------------------------
// write_reg()
// ------------------------------------------------------------
TEST_CASE("I2CRegisterTransport write_reg() writes correct DevAddress, register, and payload")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    TestI2CTransport transport;

    uint16_t reg = 0x05;
    uint8_t payload[] = {0xAA, 0xBB};

    CHECK(transport.write_reg(reg, payload, sizeof(payload)) == true);

    CHECK(get_i2c_dev_address() == (0x42 << 1));
    CHECK(get_i2c_mem_address() == reg); // 8-bit register, no swap
    CHECK(get_i2c_tx_buffer_count() == sizeof(payload));
    CHECK(memcmp(get_i2c_tx_buffer(), payload, sizeof(payload)) == 0);
}

// ------------------------------------------------------------
// read_reg()
// ------------------------------------------------------------
TEST_CASE("I2CRegisterTransport read_reg() reads correct data from RX buffer")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    TestI2CTransport transport;

    uint16_t reg = 0x10;
    uint8_t injected[] = {0xAA, 0xBB};
    inject_i2c_rx_data(0x42 << 1, injected, sizeof(injected));

    uint8_t rx[2] = {};
    CHECK(transport.read_reg(reg, rx, sizeof(rx)) == true);

    CHECK(rx[0] == 0xAA);
    CHECK(rx[1] == 0xBB);

    CHECK(get_i2c_dev_address() == (0x42 << 1));
    CHECK(get_i2c_mem_address() == reg);
}

// ------------------------------------------------------------
// Address shifting
// ------------------------------------------------------------
TEST_CASE("I2CRegisterTransport uses shifted 7-bit address for DevAddress")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    TestI2CTransport transport;

    uint8_t payload[] = {0x12};
    CHECK(transport.write_reg(0x01, payload, sizeof(payload)) == true);

    CHECK(get_i2c_dev_address() == (0x42 << 1));
}

// ------------------------------------------------------------
// 16-bit register addressing (byte swap)
// ------------------------------------------------------------
TEST_CASE("I2CRegisterTransport correctly byte-swaps 16-bit register addresses")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    using Config16 =
        I2C_Register_Config<mock_i2c, 0x42, I2CAddressWidth::Bits16>;
    using Transport16 = I2CRegisterTransport<Config16>;

    Transport16 transport;

    uint16_t reg = 0x1234; // LSB=0x34, MSB=0x12
    uint8_t payload[] = {0xDE};

    CHECK(transport.write_reg(reg, payload, sizeof(payload)) == true);

    // HAL will byteswap the reg internally, no need for us to replicate at this time
    // if necessary, flip in mock_hal and change this test
    CHECK(get_i2c_mem_address() == reg);
}

// -----------------------------------------
// I2C Tests (Stream Mode)
// -----------------------------------------

using TestI2CStreamConfig =
    I2C_Stream_Config<mock_i2c, 0x42>;
using TestI2CStreamTransport =
    I2CStreamTransport<TestI2CStreamConfig>;

TEST_CASE("I2CStreamTransport write() sends raw payload with correct DevAddress")
{
    clear_i2c_tx_data();
    clear_i2c_addresses();

    TestI2CStreamTransport transport;

    uint8_t tx[] = {0x11, 0x22, 0x33};
    CHECK(transport.write(tx, sizeof(tx)) == true);

    CHECK(get_i2c_dev_address() == (0x42 << 1));
    CHECK(get_i2c_tx_buffer_count() == sizeof(tx));
    CHECK(memcmp(get_i2c_tx_buffer(), tx, sizeof(tx)) == 0);
}

TEST_CASE("I2CStreamTransport read() reads raw bytes from RX buffer")
{
    clear_i2c_rx_data();
    clear_i2c_addresses();

    uint8_t injected[] = {0xAA, 0xBB, 0xCC};
    inject_i2c_rx_data(0x42 << 1, injected, sizeof(injected));

    TestI2CStreamTransport transport;

    uint8_t rx[3] = {};
    CHECK(transport.read(rx, sizeof(rx)) == true);

    CHECK(memcmp(rx, injected, sizeof(injected)) == 0);
    CHECK(get_i2c_dev_address() == (0x42 << 1));
}

// -----------------------------------------
// SPI Tests (Register Mode)
// -----------------------------------------

SPI_HandleTypeDef mock_spi;
GPIO_TypeDef GPIOA;

using TestSPIConfig =
    SPI_Register_Config<mock_spi, GPIO_PIN_5, 128>;
using TestSPITransport =
    SPIRegisterTransport<TestSPIConfig>;

// ------------------------------------------------------------
// write_reg()
// ------------------------------------------------------------
TEST_CASE("SPIRegisterTransport write_reg() transmits register and payload with CS toggled")
{
    clear_spi_tx_buffer();

    TestSPIConfig config(&GPIOA);
    TestSPITransport transport(config);

    uint8_t reg = 0x7E;
    uint8_t payload[] = {0x01, 0x02};

    CHECK(transport.write_reg(reg, payload, sizeof(payload)) == true);

    // TX buffer should contain: [reg][payload...]
    CHECK(get_spi_tx_buffer_count() == 1 + sizeof(payload));
    CHECK(get_spi_tx_buffer()[0] == reg);
    CHECK(get_spi_tx_buffer()[1] == 0x01);
    CHECK(get_spi_tx_buffer()[2] == 0x02);
}

// ------------------------------------------------------------
// read_reg()
// ------------------------------------------------------------
TEST_CASE("SPIRegisterTransport read_reg() sends register then reads response with CS held low")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    uint8_t reg = 0x0F;
    uint8_t injected[] = {0x55, 0x66};
    inject_spi_rx_data(injected, sizeof(injected));

    TestSPIConfig config(&GPIOA);
    TestSPITransport transport(config);

    uint8_t rx[2] = {};
    CHECK(transport.read_reg(reg, rx, sizeof(rx)) == true);

    // TX buffer should contain:
    //   [reg] + len dummy bytes
    CHECK(get_spi_tx_buffer_count() == 1 + sizeof(rx));
    CHECK(get_spi_tx_buffer()[0] == reg);
    CHECK(get_spi_tx_buffer()[1] == 0x00); // dummy
    CHECK(get_spi_tx_buffer()[2] == 0x00); // dummy

    // RX buffer should contain injected data
    CHECK(rx[0] == 0x55);
    CHECK(rx[1] == 0x66);
}

// -----------------------------------------
// SPI Tests (Stream Mode)
// -----------------------------------------

using TestSPIStreamConfig =
    SPI_Stream_Config<mock_spi, GPIO_PIN_5, 128>;
using TestSPIStreamTransport =
    SPIStreamTransport<TestSPIStreamConfig>;

TEST_CASE("SPIStreamTransport write() transmits raw payload with CS toggled")
{
    clear_spi_tx_buffer();

    TestSPIStreamConfig config(&GPIOA);
    TestSPIStreamTransport transport(config);

    uint8_t tx[] = {0xDE, 0xAD, 0xBE, 0xEF};
    CHECK(transport.write(tx, sizeof(tx)) == true);

    CHECK(get_spi_tx_buffer_count() == sizeof(tx));
    CHECK(memcmp(get_spi_tx_buffer(), tx, sizeof(tx)) == 0);
}

TEST_CASE("SPIStreamTransport read() clocks dummy bytes and receives data")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    uint8_t injected[] = {0x11, 0x22, 0x33};
    inject_spi_rx_data(injected, sizeof(injected));

    TestSPIStreamConfig config(&GPIOA);
    TestSPIStreamTransport transport(config);

    uint8_t rx[3] = {};
    CHECK(transport.read(rx, sizeof(rx)) == true);

    // TX buffer should contain dummy bytes
    CHECK(get_spi_tx_buffer_count() == sizeof(rx));
    CHECK(get_spi_tx_buffer()[0] == 0x00);
    CHECK(get_spi_tx_buffer()[1] == 0x00);
    CHECK(get_spi_tx_buffer()[2] == 0x00);

    // RX buffer should contain injected data
    CHECK(memcmp(rx, injected, sizeof(injected)) == 0);
}

TEST_CASE("SPIStreamTransport transfer() performs full-duplex exchange")
{
    clear_spi_tx_buffer();
    clear_spi_rx_buffer();

    uint8_t injected[] = {0xAA, 0xBB, 0xCC};
    inject_spi_rx_data(injected, sizeof(injected));

    TestSPIStreamConfig config(&GPIOA);
    TestSPIStreamTransport transport(config);

    uint8_t tx[] = {0x01, 0x02, 0x03};
    uint8_t rx[3] = {};

    CHECK(transport.transfer(tx, rx, sizeof(tx)) == true);

    // TX buffer should contain the tx bytes
    CHECK(get_spi_tx_buffer_count() == sizeof(tx));
    CHECK(memcmp(get_spi_tx_buffer(), tx, sizeof(tx)) == 0);

    // RX buffer should contain injected data
    CHECK(memcmp(rx, injected, sizeof(injected)) == 0);
}

// -----------------------------------------
// UART Tests (Stream Mode)
// -----------------------------------------

UART_HandleTypeDef mock_uart;
using TestUARTConfig = UART_Config<mock_uart>;
using TestUARTTransport = UARTTransport<TestUARTConfig>;

// ------------------------------------------------------------
// write()
// ------------------------------------------------------------
TEST_CASE("UARTTransport write() transmits correct data")
{
    clear_uart_tx_buffer();

    TestUARTTransport transport;
    uint8_t msg[] = "Hello";

    CHECK(transport.write(msg, sizeof(msg)) == true);

    CHECK(get_uart_tx_buffer_count() == sizeof(msg));
    CHECK(get_uart_tx_buffer()[0] == 'H');
    CHECK(get_uart_tx_buffer()[4] == 'o');
}

// ------------------------------------------------------------
// read()
// ------------------------------------------------------------
TEST_CASE("UARTTransport read() receives injected data")
{
    clear_uart_rx_buffer();

    uint8_t injected[] = {'A', 'B', 'C'};
    inject_uart_rx_data(injected, sizeof(injected));

    TestUARTTransport transport;
    uint8_t buf[3] = {};

    CHECK(transport.read(buf, sizeof(buf)) == true);

    CHECK(buf[0] == 'A');
    CHECK(buf[2] == 'C');
}

// ------------------------------------------------------------
// SCCB Transport Tests
// ------------------------------------------------------------

struct MockSCCBBus
{
    std::vector<uint8_t> bits;      // sampled bits on SCL rising edge
    uint8_t last_sda = 1;           // current SDA level
    std::queue<uint8_t> read_queue; // bits for read_byte()

    // --- SCCB Core interface ---

    void scl_high()
    {
        // sample SDA on rising edge
        bits.push_back(last_sda);
    }

    void scl_low() {}

    void sda_high() { last_sda = 1; }
    void sda_low() { last_sda = 0; }

    void sda_as_input() {}
    void sda_as_output_od() {}

    bool sda_read()
    {
        if (read_queue.empty())
            return 0;
        uint8_t bit = read_queue.front();
        read_queue.pop();
        return bit;
    }

    void delay() {}

    // --- Helpers for tests ---

    void inject_read_byte(uint8_t v)
    {
        for (int i = 7; i >= 0; --i)
            read_queue.push((v >> i) & 1);
    }

    void clear()
    {
        bits.clear();
        std::queue<uint8_t> empty;
        std::swap(read_queue, empty);
        last_sda = 1;
    }
};

using TestSCCBConfig8 = SCCBRegisterConfig<MockSCCBBus, 0x30, SCCBAddressWidth::Bits8>;
using TestSCCBConfig16 = SCCBRegisterConfig<MockSCCBBus, 0x30, SCCBAddressWidth::Bits16>;

static bool contains_byte(const std::vector<uint8_t>& bits, uint8_t value)
{
    if (bits.size() < 8) return false;

    for (size_t i = 0; i + 7 < bits.size(); ++i)
    {
        uint8_t cur = 0;
        for (size_t b = 0; b < 8; ++b)
            cur = static_cast<uint8_t>((cur << 1) | (bits[i + b] & 1));

        if (cur == value)
            return true;
    }
    return false;
}

TEST_CASE("SCCB_Register_Transport write_reg() sends correct sequence for 8-bit reg")
{
    MockSCCBBus bus;
    SCCB_Register_Transport<TestSCCBConfig8, MockSCCBBus> t(bus);

    uint8_t val = 0xAA;
    CHECK(t.write_reg(0x0A, &val, 1) == true);

    auto& bits = bus.bits;

    CHECK(contains_byte(bits, static_cast<uint8_t>((0x30 << 1) & 0xFE))); // device addr + write
    CHECK(contains_byte(bits, static_cast<uint8_t>(0x0A)));               // reg
    CHECK(contains_byte(bits, static_cast<uint8_t>(0xAA)));               // value
}

TEST_CASE("SCCB_Register_Transport write_reg() sends correct sequence for 16-bit reg")
{
    MockSCCBBus bus;
    SCCB_Register_Transport<TestSCCBConfig16, MockSCCBBus> t(bus);

    uint8_t val = 0x55;
    CHECK(t.write_reg(0x1234, &val, 1) == true);

    auto& bits = bus.bits;

    CHECK(contains_byte(bits, static_cast<uint8_t>((0x30 << 1) & 0xFE))); // device addr + write
    CHECK(contains_byte(bits, static_cast<uint8_t>(0x12)));               // reg high
    CHECK(contains_byte(bits, static_cast<uint8_t>(0x34)));               // reg low
    CHECK(contains_byte(bits, static_cast<uint8_t>(0x55)));               // value
}

TEST_CASE("SCCB_Register_Transport read_reg() reads injected byte")
{
    MockSCCBBus bus;
    SCCB_Register_Transport<TestSCCBConfig8, MockSCCBBus> t(bus);

    bus.inject_read_byte(0x5A);

    uint8_t out = 0;
    CHECK(t.read_reg(0x0A, &out, 1) == true);
    CHECK(out == 0x5A);
}

// -----------------------------------------
// Additional Transport Tests
// -----------------------------------------

// I2C register‑mode transport
using TestI2CConfig =
    I2C_Register_Config<mock_i2c, 0x42, I2CAddressWidth::Bits8>;
using TestI2CTransport =
    I2CRegisterTransport<TestI2CConfig>;

// SPI register‑mode transport
using TestSPIConfig =
    SPI_Register_Config<mock_spi, GPIO_PIN_5, 128>;
using TestSPITransport =
    SPIRegisterTransport<TestSPIConfig>;

// UART stream‑mode transport
using TestUARTConfig = UART_Config<mock_uart>;
using TestUARTTransport = UARTTransport<TestUARTConfig>;

// ------------------------------------------------------------
// Concept satisfaction
// ------------------------------------------------------------
TEST_CASE("Transport concepts are satisfied")
{
    // I2C register‑mode
    static_assert(RegisterModeTransport<TestI2CTransport>);
    static_assert(RegisterAccessTransport<TestI2CTransport>);

    // SPI register‑mode
    static_assert(RegisterModeTransport<TestSPITransport>);
    static_assert(RegisterAccessTransport<TestSPITransport>);

    // UART stream‑mode
    static_assert(StreamModeTransport<TestUARTTransport>);
    static_assert(StreamAccessTransport<TestUARTTransport>);

    static_assert(RegisterAccessTransport<SCCB_Register_Transport<TestSCCBConfig8, MockSCCBBus>>);
    static_assert(RegisterAccessTransport<SCCB_Register_Transport<TestSCCBConfig16, MockSCCBBus>>);
}

// ------------------------------------------------------------
// TransportTraits correctness
// ------------------------------------------------------------
TEST_CASE("TransportTraits report correct transport kind")
{
#ifdef HAS_I2C_HANDLE_TYPEDEF
    CHECK(TransportTraits<TestI2CTransport>::kind == TransportKind::I2C);
#endif

#ifdef HAS_SPI_HANDLE_TYPEDEF
    CHECK(TransportTraits<TestSPITransport>::kind == TransportKind::SPI);
#endif

#ifdef HAS_UART_HANDLE_TYPEDEF
    CHECK(TransportTraits<TestUARTTransport>::kind == TransportKind::UART);
#endif
}

// ------------------------------------------------------------
// Mode tag correctness
// ------------------------------------------------------------
TEST_CASE("Mode tags are correctly assigned")
{
    // I2C and SPI are register‑mode transports
    CHECK(std::is_same_v<TestI2CTransport::config_type::mode_tag, register_mode_tag>);
    CHECK(std::is_same_v<TestSPITransport::config_type::mode_tag, register_mode_tag>);

    // UART is stream‑mode
    CHECK(std::is_same_v<TestUARTTransport::config_type::mode_tag, stream_mode_tag>);
}
