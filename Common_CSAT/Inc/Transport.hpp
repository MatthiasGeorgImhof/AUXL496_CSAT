#ifndef __TRANSPORT_HPP__
#define __TRANSPORT_HPP__

#ifdef __arm__
#include "stm32l4xx_hal.h"
#include "usb_device.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

#include "transport_config.h"

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <concepts>
#include <type_traits>

#include "SCCB.hpp"

// Mode tags
struct register_mode_tag
{
};
struct stream_mode_tag
{
};

// Transport tags
struct i2c_tag
{
};
struct sccb_tag
{
};
struct spi_tag
{
};
struct uart_tag
{
};

// -----------------------------------------
// I2C Transport (Register Mode)
// -----------------------------------------

#ifdef HAS_I2C_HANDLE_TYPEDEF

enum class I2CAddressWidth : uint16_t
{
    Bits8 = I2C_MEMADD_SIZE_8BIT,
    Bits16 = I2C_MEMADD_SIZE_16BIT
};

template <
    I2C_HandleTypeDef &HandleRef,
    uint16_t Address,
    I2CAddressWidth AddrWidth = I2CAddressWidth::Bits8,
    uint32_t Timeout = 100>
struct I2C_Register_Config
{
    using transport_tag = i2c_tag;
    using mode_tag = register_mode_tag;

    static I2C_HandleTypeDef &handle() { return HandleRef; }
    static constexpr uint16_t address = Address << 1;
    static constexpr I2CAddressWidth address_width = AddrWidth;
    static constexpr uint32_t timeout = Timeout;

    static_assert(std::is_same_v<decltype(HandleRef), I2C_HandleTypeDef &>, "Handle must be I2C_HandleTypeDef&");
    static_assert(Address <= 0x7F, "I2C address must be 7-bit");
    static_assert(Timeout > 0 && Timeout < 10000, "Timeout must be a reasonable value");
};

template <typename Config>
    requires std::is_same_v<typename Config::transport_tag, i2c_tag> &&
             std::is_same_v<typename Config::mode_tag, register_mode_tag>
class I2CRegisterTransport
{
public:
    using config_type = Config;

private:
    static constexpr uint16_t encode_reg(uint16_t reg)
    {
        // HAL will flip reg when necessary
        return reg;
    }

public:
    bool write_reg(uint16_t reg, const uint8_t *data, uint16_t len) const
    {
        return HAL_I2C_Mem_Write(&Config::handle(), Config::address, encode_reg(reg), static_cast<uint16_t>(Config::address_width), const_cast<uint8_t *>(data), len, Config::timeout) == HAL_OK;
    }

    bool read_reg(uint16_t reg, uint8_t *data, uint16_t len) const
    {
        return HAL_I2C_Mem_Read(&Config::handle(), Config::address, encode_reg(reg), static_cast<uint16_t>(Config::address_width), data, len, Config::timeout) == HAL_OK;
    }

    static_assert(Config::address_width == I2CAddressWidth::Bits8 || Config::address_width == I2CAddressWidth::Bits16);
};

template <I2C_HandleTypeDef &HandleRef, uint16_t Address, uint32_t Timeout = 100>
struct I2C_Stream_Config
{
    using transport_tag = i2c_tag;
    using mode_tag = stream_mode_tag;

    static I2C_HandleTypeDef &handle() { return HandleRef; }
    static constexpr uint16_t address = Address << 1;
    static constexpr uint32_t timeout = Timeout;

    static_assert(std::is_same_v<decltype(HandleRef), I2C_HandleTypeDef &>, "Handle must be I2C_HandleTypeDef&");
    static_assert(Address <= 0x7F, "I2C address must be 7-bit");
    static_assert(Timeout > 0 && Timeout < 10000, "Timeout must be a reasonable value");
};

template <typename Config>
    requires std::is_same_v<typename Config::transport_tag, i2c_tag> &&
             std::is_same_v<typename Config::mode_tag, stream_mode_tag>
class I2CStreamTransport
{
public:
    using config_type = Config;

    bool write(const uint8_t *data, uint16_t len) const
    {
        return HAL_I2C_Master_Transmit(&Config::handle(), Config::address, const_cast<uint8_t *>(data), len, Config::timeout) == HAL_OK;
    }

    bool read(uint8_t *data, uint16_t len) const
    {
        return HAL_I2C_Master_Receive(&Config::handle(), Config::address, data, len, Config::timeout) == HAL_OK;
    }
};

#endif // HAS_I2C_HANDLE_TYPEDEF

// -----------------------------------------
// SCCB Transport (Register Mode)
// -----------------------------------------

#ifdef HAS_SCCB

enum class SCCBAddressWidth : uint8_t {
    Bits8,
    Bits16
};

template <
    typename Bus,
    uint8_t Address,
    SCCBAddressWidth AddrWidth = SCCBAddressWidth::Bits8>
struct SCCBRegisterConfig
{
    using transport_tag = sccb_tag;
    using mode_tag      = register_mode_tag;
    using bus_type      = Bus;

    static constexpr uint8_t address = Address;
    static constexpr SCCBAddressWidth address_width = AddrWidth;
};

template <typename Config, typename Bus>
class SCCB_Register_Transport
{
public:
    using config_type = Config;

    explicit SCCB_Register_Transport(Bus& bus)
        : bus_(bus) {}

    bool write_reg(uint16_t reg, const uint8_t* data, uint16_t len) const
    {
        if (len != 1) return false;

        SCCB_Core::start(bus_);
        SCCB_Core::write_byte(bus_, Config::address << 1); // write
        write_reg_addr(reg);
        SCCB_Core::write_byte(bus_, *data);
        SCCB_Core::stop(bus_);
        return true;
    }

    bool read_reg(uint16_t reg, uint8_t* data, uint16_t len) const
    {
        if (len != 1) return false;

        // write register address
        SCCB_Core::start(bus_);
        SCCB_Core::write_byte(bus_, Config::address << 1);
        write_reg_addr(reg);
        SCCB_Core::stop(bus_);

        // read data
        SCCB_Core::start(bus_);
        SCCB_Core::write_byte(bus_, (Config::address << 1) | 1);
        *data = SCCB_Core::read_byte(bus_);
        SCCB_Core::stop(bus_);

        return true;
    }

private:
    Bus& bus_;

    void write_reg_addr(uint16_t reg) const
    {
        if constexpr (Config::address_width == SCCBAddressWidth::Bits8)
        {
            SCCB_Core::write_byte(bus_, static_cast<uint8_t>(reg));
        }
        else
        {
            SCCB_Core::write_byte(bus_, static_cast<uint8_t>(reg >> 8));
            SCCB_Core::write_byte(bus_, static_cast<uint8_t>(reg & 0xFF));
        }
    }
};

#endif // HAS_SCCB

// -----------------------------------------
// SPI Transport (Register Mode)
// -----------------------------------------

#ifdef HAS_SPI_HANDLE_TYPEDEF

template <SPI_HandleTypeDef &HandleRef, uint16_t Pin,
          std::size_t MaxTransferSize, uint32_t Timeout = 100>
struct SPI_Register_Config
{
    SPI_Register_Config() = delete;
    SPI_Register_Config(GPIO_TypeDef *csport) : csPort(csport) {}
    using transport_tag = spi_tag;
    using mode_tag = register_mode_tag;

    static SPI_HandleTypeDef &handle() { return HandleRef; }
    static constexpr uint16_t csPin = Pin;
    static constexpr uint32_t timeout = Timeout;
    static constexpr std::size_t max_transfer_size = MaxTransferSize;

public:
    GPIO_TypeDef *csPort;

    static_assert(std::is_same_v<decltype(HandleRef), SPI_HandleTypeDef &>, "Handle must be SPI_HandleTypeDef&");
    static_assert(Timeout > 0 && Timeout < 10000, "Timeout must be a reasonable value");
    static_assert(MaxTransferSize > 0 && MaxTransferSize <= 1024, "MaxTransferSize must be reasonable");
};

template <typename Config>
    requires std::is_same_v<typename Config::transport_tag, spi_tag> &&
             std::is_same_v<typename Config::mode_tag, register_mode_tag>
class SPIRegisterTransport
{
public:
    using config_type = Config;

    explicit SPIRegisterTransport(const Config &cfg) : config(cfg) { deselect(); }

    bool write_reg(uint8_t reg, const uint8_t *data, uint16_t len) const
    {
        if (static_cast<std::size_t>(len) + 1U > Config::max_transfer_size)
        {
            return false;
        }

        uint8_t buffer[Config::max_transfer_size];
        buffer[0] = reg;
        if (len > 0)
        {
            std::memcpy(&buffer[1], data, len);
        }

        select();
        bool ok = HAL_SPI_Transmit(&Config::handle(), buffer, len + 1, Config::timeout) == HAL_OK;
        deselect();
        return ok;
    }

    bool read_reg(uint8_t reg, uint8_t *data, uint16_t len) const
    {
        if (static_cast<std::size_t>(len) > Config::max_transfer_size)
        {
            return false;
        }
        
        uint8_t dummy_tx[Config::max_transfer_size] = {};
        uint8_t rx[Config::max_transfer_size] = {};

        select();
        // Send register address
        bool ok = HAL_SPI_Transmit(&Config::handle(), &reg, 1, Config::timeout) == HAL_OK;
        if (ok && len > 0)
        {
            ok = HAL_SPI_TransmitReceive(&Config::handle(), dummy_tx, rx, len, Config::timeout) == HAL_OK;
            if (ok)
            {
                std::memcpy(data, rx, len);
            }
        }
        deselect();
        return ok;
    }

private:
    Config config;

    void select() const { HAL_GPIO_WritePin(config.csPort, Config::csPin, GPIO_PIN_RESET); }
    void deselect() const { HAL_GPIO_WritePin(config.csPort, Config::csPin, GPIO_PIN_SET); }
};

template <SPI_HandleTypeDef &HandleRef,
          uint16_t Pin,
          std::size_t MaxTransferSize,
          uint32_t Timeout = 100>
struct SPI_Stream_Config
{
    SPI_Stream_Config() = delete;
    SPI_Stream_Config(GPIO_TypeDef *csport) : csPort(csport) {}

    using transport_tag = spi_tag;
    using mode_tag = stream_mode_tag;

    static SPI_HandleTypeDef &handle() { return HandleRef; }
    static constexpr uint16_t csPin = Pin;
    static constexpr uint32_t timeout = Timeout;
    static constexpr std::size_t max_transfer_size = MaxTransferSize;

public:
    GPIO_TypeDef *csPort;
};

template <typename Config>
    requires std::is_same_v<typename Config::transport_tag, spi_tag> &&
             std::is_same_v<typename Config::mode_tag, stream_mode_tag>
class SPIStreamTransport
{
public:
    using config_type = Config;

    explicit SPIStreamTransport(const Config &cfg) : config(cfg) { deselect(); }

    bool write(const uint8_t *tx_buf, uint16_t tx_len) const
    {
        if (tx_len > Config::max_transfer_size)
            return false;

        select();
        bool ok = HAL_SPI_Transmit(&Config::handle(), const_cast<uint8_t *>(tx_buf), tx_len, Config::timeout) == HAL_OK;
        deselect();
        return ok;
    }

    bool read(uint8_t *rx_buf, uint16_t len) const
    {
        if (len > Config::max_transfer_size)
            return false;

        select();
        bool ok = HAL_SPI_TransmitReceive(&Config::handle(), rx_buf, rx_buf, len, Config::timeout) == HAL_OK;
        deselect();
        return ok;
    }

    bool transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) const
    {
        if (len > Config::max_transfer_size)
            return false;
        select();
        bool ok = HAL_SPI_TransmitReceive(&Config::handle(), tx_buf, rx_buf, len, Config::timeout) == HAL_OK;
        deselect();
        return ok;
    }

private:
    Config config;

    void select() const { HAL_GPIO_WritePin(config.csPort, Config::csPin, GPIO_PIN_RESET); }
    void deselect() const { HAL_GPIO_WritePin(config.csPort, Config::csPin, GPIO_PIN_SET); }
};

#endif // HAS_SPI_HANDLE_TYPEDEF
// -----------------------------------------
// UART Transport (Stream Mode)
// -----------------------------------------

#ifdef HAS_UART_HANDLE_TYPEDEF

template <UART_HandleTypeDef &HandleRef, uint32_t Timeout = 100>
struct UART_Config
{
    using transport_tag = uart_tag;
    using mode_tag = stream_mode_tag;

    static UART_HandleTypeDef &handle() { return HandleRef; }
    static constexpr uint32_t timeout = Timeout;

    static_assert(std::is_same_v<decltype(HandleRef), UART_HandleTypeDef &>, "Handle must be UART_HandleTypeDef&");
    static_assert(Timeout > 0 && Timeout < 10000, "Timeout must be a reasonable value");
};

template <typename Config>
    requires std::is_same_v<typename Config::transport_tag, uart_tag>
class UARTTransport
{
public:
    using config_type = Config;

    bool write(const uint8_t *buf, uint16_t len) const
    {
        return HAL_UART_Transmit(&Config::handle(), const_cast<uint8_t *>(buf), len, Config::timeout) == HAL_OK;
    }

    bool read(uint8_t *buf, uint16_t len) const
    {
        return HAL_UART_Receive(&Config::handle(), buf, len, Config::timeout) == HAL_OK;
    }
};

#endif // HAS_UART_HANDLE_TYPEDEF

// -----------------------------------------
// Transport Concepts
// -----------------------------------------

// Mode tags (simple classification)
template <typename T>
concept RegisterModeTransport =
    std::is_same_v<typename T::config_type::mode_tag, register_mode_tag>;

template <typename T>
concept StreamModeTransport =
    std::is_same_v<typename T::config_type::mode_tag, stream_mode_tag>;


// -----------------------------------------
// Register‑mode transport requirements
// -----------------------------------------
//
// A register‑mode transport must provide:
//   bool write_reg(uint16_t reg, const uint8_t* data, uint16_t len);
//   bool read_reg(uint16_t reg, uint8_t* data, uint16_t len);
//
template <typename T>
concept RegisterAccessTransport =
    RegisterModeTransport<T> &&
    requires(T t, uint16_t reg, uint8_t* buf, uint16_t len)
    {
        { t.write_reg(reg, buf, len) } -> std::same_as<bool>;
        { t.read_reg(reg, buf, len) }  -> std::same_as<bool>;
    };


// -----------------------------------------
// Stream‑mode transport requirements
// -----------------------------------------
//
// A stream‑mode transport must provide:
//   bool write(const uint8_t* data, uint16_t len);
//   bool read(uint8_t* data, uint16_t len);
//
// SPI stream‑mode may also provide:
//   bool transfer(uint8_t* tx, uint8_t* rx, uint16_t len);
// but this is optional.
//
template <typename T>
concept StreamAccessTransport =
    StreamModeTransport<T> &&
    requires(T t, uint8_t* buf, uint16_t len)
    {
        { t.write(buf, len) } -> std::same_as<bool>;
        { t.read(buf, len) }  -> std::same_as<bool>;
    };


// Optional full‑duplex SPI concept
template <typename T>
concept FullDuplexStreamTransport =
    StreamModeTransport<T> &&
    requires(T t, uint8_t* tx, uint8_t* rx, uint16_t len)
    {
        { t.transfer(tx, rx, len) } -> std::same_as<bool>;
    };


// -----------------------------------------
// Unified Transport Protocol Concept
// -----------------------------------------
//
// A valid transport is either:
//   - a register‑mode transport (I2C/SPI register mode)
//   - a stream‑mode transport (I2C/SPI/UART stream mode)
//
template <typename T>
concept TransportProtocol =
    RegisterAccessTransport<T> || StreamAccessTransport<T>;


// -----------------------------------------
// Transport Kind Traits
// -----------------------------------------

enum class TransportKind
{
    I2C,
	SCCB,
    SPI,
    UART
};

template <typename T>
struct TransportTraits;

#ifdef HAS_I2C_HANDLE_TYPEDEF
template <typename Config>
struct TransportTraits<I2CRegisterTransport<Config>>
{
    static constexpr TransportKind kind = TransportKind::I2C;
};
template <typename Config>
struct TransportTraits<I2CStreamTransport<Config>>
{
    static constexpr TransportKind kind = TransportKind::I2C;
};
#endif

#ifdef HAS_SCCB_HANDLE_TYPEDEF
template <typename Config>
struct TransportTraits<SCCB_Register_Transport<Config>>
{
    static constexpr TransportKind kind = TransportKind::SCCB;
};
#endif

#ifdef HAS_SPI_HANDLE_TYPEDEF
template <typename Config>
struct TransportTraits<SPIRegisterTransport<Config>>
{
    static constexpr TransportKind kind = TransportKind::SPI;
};
template <typename Config>
struct TransportTraits<SPIStreamTransport<Config>>
{
    static constexpr TransportKind kind = TransportKind::SPI;
};
#endif

#ifdef HAS_UART_HANDLE_TYPEDEF
template <typename Config>
struct TransportTraits<UARTTransport<Config>>
{
    static constexpr TransportKind kind = TransportKind::UART;
};
#endif

#endif // __TRANSPORT_HPP__
