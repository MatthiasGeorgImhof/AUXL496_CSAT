#pragma once
#include <cstdint>
#include <cstring>
#include "Transport.hpp"

#include "Logger.hpp"

// ─────────────────────────────────────────────
// MLX90640 Constants
// ─────────────────────────────────────────────
constexpr static uint8_t MLX90640_ID = 0x33;
constexpr static std::size_t MLX90640_EEPROM_WORDS = 832;
constexpr static std::size_t MLX90640_EEPROM_SIZE = MLX90640_EEPROM_WORDS * sizeof(uint16_t);
constexpr static std::size_t MLX90640_SUBPAGE_WORDS = 834;
constexpr static std::size_t MLX90640_SUBPAGE_SIZE = MLX90640_SUBPAGE_WORDS * sizeof(uint16_t);
constexpr static std::size_t MLX90640_FRAME_WORDS = 2 * MLX90640_SUBPAGE_WORDS;
constexpr static std::size_t MLX90640_FRAME_SIZE = MLX90640_FRAME_WORDS * sizeof(uint16_t);

// ─────────────────────────────────────────────
// MLX90640 Refresh Rates
// ─────────────────────────────────────────────

enum class MLX90640_RefreshRate : uint16_t
{
    Hz0_5 = 0b000 << 5, // 0.5 Hz
    Hz1 = 0b001 << 5,   // 1 Hz
    Hz2 = 0b010 << 5,   // 2 Hz
    Hz4 = 0b011 << 5,   // 4 Hz
    Hz8 = 0b100 << 5,   // 8 Hz
    Hz16 = 0b101 << 5,  // 16 Hz
    Hz32 = 0b110 << 5,  // 32 Hz
    Hz64 = 0b111 << 5   // 64 Hz
};

constexpr static uint32_t MLX90640_BOOT_TIME_MS = 80;
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz0_5 = 4000; // ms
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz1 = 2000;   // ms
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz2 = 1000;   // ms
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz4 = 500;    // ms
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz8 = 250;    // ms
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz16 = 125;   // ms
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz32 = 63;    // ms
constexpr static uint32_t MLX90640_REFRESH_INTERVAL_Hz64 = 32;    // ms

constexpr uint32_t getRefreshIntervalMs(MLX90640_RefreshRate rate)
{
    switch (rate)
    {
    case MLX90640_RefreshRate::Hz0_5: return MLX90640_REFRESH_INTERVAL_Hz0_5;
    case MLX90640_RefreshRate::Hz1:   return MLX90640_REFRESH_INTERVAL_Hz1;
    case MLX90640_RefreshRate::Hz2:   return MLX90640_REFRESH_INTERVAL_Hz2;
    case MLX90640_RefreshRate::Hz4:   return MLX90640_REFRESH_INTERVAL_Hz4;
    case MLX90640_RefreshRate::Hz8:   return MLX90640_REFRESH_INTERVAL_Hz8;
    case MLX90640_RefreshRate::Hz16:  return MLX90640_REFRESH_INTERVAL_Hz16;
    case MLX90640_RefreshRate::Hz32:  return MLX90640_REFRESH_INTERVAL_Hz32;
    case MLX90640_RefreshRate::Hz64:  return MLX90640_REFRESH_INTERVAL_Hz64;
    }

    // Defensive fallback for invalid values
    return MLX90640_REFRESH_INTERVAL_Hz4; // sensible default
}

// ─────────────────────────────────────────────
// MLX90640 Register Map
// ─────────────────────────────────────────────
enum class MLX90640_REGISTERS : uint16_t
{
    STATUS = 0x8000,      // Status register (NEW_DATA, flags)
    CONTROL1 = 0x800D,    // Control register 1 (mode, refresh rate, power)
    RAM_START = 0x0400,   // Start of RAM subpage data
    EEPROM_START = 0x2400 // Start of EEPROM
};

// ─────────────────────────────────────────────
// MLX90640 Driver
// ─────────────────────────────────────────────
template <RegisterModeTransport Transport>
class MLX90640
{
    // Enforce 16‑bit register addressing for MLX90640
    static_assert(
        Transport::config_type::address_width == I2CAddressWidth::Bits16,
        "MLX90640 requires I2CAddressWidth::Bits16 (16‑bit register addressing).");

public:
    explicit MLX90640(const Transport &t) : transport(t) {}

    // ─────────────────────────────────────────────
    // Initialization: wake device, chess mode, refresh rate
    // ─────────────────────────────────────────────
    bool wakeUp(MLX90640_RefreshRate rate = MLX90640_RefreshRate::Hz4)
    {
        uint16_t ctrl;

        if (!readReg16(static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1), ctrl))
            return false;

        constexpr uint16_t CLEAR_MASK =
            (1u << 0) | // sleep/wake
            (1u << 3) | // subpage mode
            (1u << 12); // chess/TV mode

        ctrl &= static_cast<uint16_t>(~CLEAR_MASK);

        uint16_t newCtrl = ctrl;
        newCtrl |= uint16_t(1u << 0);                         // wake
        newCtrl |= uint16_t(1u << 12);                        // chess mode
        newCtrl &= static_cast<uint16_t>(~uint16_t(1u << 3)); // auto subpage

        if (!writeReg16(static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1), newCtrl))
            return false;

        return setRefreshRate(rate);
    }

    bool setRefreshRate(MLX90640_RefreshRate rate)
    {
        uint16_t ctrlReg;

        if (!readReg16(static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1), ctrlReg))
            return false;

        ctrlReg &= static_cast<uint16_t>(~uint16_t(0b111u << 5));
        ctrlReg |= static_cast<uint16_t>(rate);

        return writeReg16(static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1), ctrlReg);
    }

    // ─────────────────────────────────────────────
    // Put device into sleep mode
    // ─────────────────────────────────────────────
    bool sleep()
    {
        uint16_t ctrl;

        if (!readReg16(static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1), ctrl))
            return false;

        // Bit 0 = 0 → sleep mode
        uint16_t newCtrl = static_cast<uint16_t>(ctrl & static_cast<uint16_t>(~uint16_t(0x0001)));

        return writeReg16(static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1), newCtrl);
    }

    // ─────────────────────────────────────────────
    // Optional soft reset
    // ─────────────────────────────────────────────
    bool reset()
    {
        // Clear NEW_DATA and other status bits
        if (!clearStatus())
            return false;

        // Put CONTROL1 into default (sleep) state
        if (!writeReg16(static_cast<uint16_t>(MLX90640_REGISTERS::CONTROL1), 0x0000))
            return false;

        return true;
    }

    // ─────────────────────────────────────────────
    // Read EEPROM (832 words)
    // ─────────────────────────────────────────────
    bool readEEPROM(uint16_t *eeprom)
    {
        return readBlock(
            static_cast<uint16_t>(MLX90640_REGISTERS::EEPROM_START),
            reinterpret_cast<uint8_t *>(eeprom),
            MLX90640_EEPROM_SIZE);
    }

    // ─────────────────────────────────────────────
    // Non-blocking readiness check (NEW_DATA bit)
    // ─────────────────────────────────────────────
    bool isReady()
    {
        uint16_t status;
        if (!readStatus(status))
            return false;

        bool ready = (status & 0x0008u) != 0u; // NEW_DATA bit
        log(LOG_LEVEL_DEBUG, "MLX90640::isReady: STATUS=0x%04X, NEW_DATA=%u\r\n", status, ready ? 1u : 0u);
        return ready;
    }

    bool waitUntilReady(unsigned maxAttempts = 512)
    {
        for (unsigned i = 0; i < maxAttempts; ++i)
        {
            if (isReady())
                return true;
            HAL_Delay(1);
        }
        log(LOG_LEVEL_WARNING, "MLX90640::waitUntilReady: timed out\r\n");
        return false;
    }

    // ─────────────────────────────────────────────
    // Read a single MLX90640_SUBPAGE (834 words)
    // ─────────────────────────────────────────────
    //
    // Correct sequence:
    //  1. Assume caller has waited for NEW_DATA.
    //  2. Read STATUS → get subpage ID and confirm NEW_DATA.
    //  3. Read RAM snapshot.
    //  4. Clear NEW_DATA.
    //
    bool readSubpage(uint16_t *buf, int &subpage)
    {
        if (!buf)
            return false;

        // 1. Read STATUS first
        uint16_t status = 0;
        if (!readStatus(status))
        {
            log(LOG_LEVEL_ERROR, "MLX90640::readSubpage: read STATUS failed\r\n");
            return false;
        }

        // if ((status & 0x0008u) == 0u)
        // {
        //     // NEW_DATA not set → caller should have waited, but be defensive
        //     log(LOG_LEVEL_WARNING, "MLX90640::readSubpage: NEW_DATA not set (STATUS=0x%04X)\r\n", status);
        //     return false;
        // }

        subpage = static_cast<int>(status & 0x0001u); // bit 0 = subpage ID
        log(LOG_LEVEL_DEBUG, "MLX90640::readSubpage: STATUS=0x%04X, subpage=%d\r\n", status, subpage);

        // 2. Read RAM snapshot
        if (!readBlock(
                static_cast<uint16_t>(MLX90640_REGISTERS::RAM_START),
                reinterpret_cast<uint8_t *>(buf),
                MLX90640_SUBPAGE_SIZE))
        {
            log(LOG_LEVEL_ERROR, "MLX90640::readSubpage: read RAM failed\r\n");
            return false;
        }

        // 3. Clear NEW_DATA (write‑1‑to‑clear)
        if (!clearStatus())
        {
            log(LOG_LEVEL_ERROR, "MLX90640::readSubpage: clearStatus failed\r\n");
            return false;
        }

        return true;
    }

    // ─────────────────────────────────────────────
    // Merge two subpages into a full frame
    // ─────────────────────────────────────────────
    void createFrame(const uint16_t *sub0,
                     const uint16_t *sub1,
                     uint16_t *fullFrame)
    {
        if (!sub0 || !sub1 || !fullFrame)
            return;

        std::memcpy(fullFrame, sub0, MLX90640_SUBPAGE_SIZE);
        std::memcpy(fullFrame + MLX90640_SUBPAGE_WORDS, sub1, MLX90640_SUBPAGE_SIZE);
    }

    // ─────────────────────────────────────────────
    // Read a full frame (two subpages)
    // ─────────────────────────────────────────────
    bool readFrame(uint16_t *frame)
    {
        if (!frame)
            return false;

        uint16_t sub0[MLX90640_SUBPAGE_WORDS];
        uint16_t sub1[MLX90640_SUBPAGE_WORDS];
        int spA = -1;
        int spB = -1;

        // First subpage
        if (!waitUntilReady())
        {
            log(LOG_LEVEL_ERROR, "MLX90640::readFrame: waitUntilReady A failed\r\n");
            return false;
        }
        if (!readSubpage(sub0, spA))
        {
            log(LOG_LEVEL_ERROR, "MLX90640::readFrame: readSubpage A failed\r\n");
            return false;
        }

        // Second subpage
        if (!waitUntilReady())
        {
            log(LOG_LEVEL_ERROR, "MLX90640::readFrame: waitUntilReady B failed\r\n");
            return false;
        }
        if (!readSubpage(sub1, spB))
        {
            log(LOG_LEVEL_ERROR, "MLX90640::readFrame: readSubpage B failed\r\n");
            return false;
        }

        log(LOG_LEVEL_DEBUG, "MLX90640::readFrame: spA=%d, spB=%d\r\n", spA, spB);

        if (spA == spB)
        {
            log(LOG_LEVEL_WARNING, "MLX90640::readFrame: spA == spB (%d) → frame rejected\r\n", spA);
            return false;
        }

        if (spA == 0)
            createFrame(sub0, sub1, frame);
        else
            createFrame(sub1, sub0, frame);

        return true;
    }

    // ─────────────────────────────────────────────
    // Status helpers
    // ─────────────────────────────────────────────
    bool readStatus(uint16_t &status)
    {
        return readReg16(static_cast<uint16_t>(MLX90640_REGISTERS::STATUS), status);
    }

    // MLX90640 STATUS is write‑1‑to‑clear.
    // NEW_DATA is bit 3 → we write 0x0008 to clear it.
    bool clearStatus()
    {
        constexpr uint16_t CLEAR_NEW_DATA = 0x0008;
        bool ok = writeReg16(static_cast<uint16_t>(MLX90640_REGISTERS::STATUS),
                             CLEAR_NEW_DATA);
        if (!ok)
        {
            log(LOG_LEVEL_ERROR, "MLX90640::clearStatus: write STATUS failed\r\n");
        }
        return ok;
    }

    bool readStatusBlock(uint16_t out[16])
    {
        if (!out)
            return false;

        // 0x8000 through 0x800F → 16 consecutive 16‑bit registers
        constexpr uint16_t START = static_cast<uint16_t>(MLX90640_REGISTERS::STATUS);
        constexpr size_t COUNT = 16;

        for (size_t i = 0; i < COUNT; ++i)
        {
            uint16_t value;
            if (!readReg16(static_cast<uint16_t>(START + i), value))
                return false;

            out[i] = value;
        }

        return true;
    }

private:
    const Transport &transport;

    // ─────────────────────────────────────────────
    // Low-level register access
    // ─────────────────────────────────────────────
    bool readReg16(uint16_t reg, uint16_t &out)
    {
        uint8_t buf[2] = {0, 0};
        if (!transport.read_reg(reg, buf, sizeof(buf)))
            return false;

        out = (uint16_t(buf[0]) << 8) | buf[1];
        return true;
    }

    bool writeReg16(uint16_t reg, uint16_t value)
    {
        uint8_t buf[2] = {
            uint8_t(value >> 8),
            uint8_t(value & 0xFF)};
        return transport.write_reg(reg, buf, sizeof(buf));
    }

    bool readBlock(uint16_t startReg, uint8_t *dest, std::size_t bytes)
    {
        if (!dest || bytes == 0)
            return false;

        // Many I2C stacks (or devices) struggle with very large transfers.
        // Use a conservative chunk size; 64 bytes is usually safe.
        constexpr std::size_t MAX_CHUNK = 256;

        uint16_t reg = startReg;
        std::size_t remaining = bytes;
        std::size_t offset = 0;

        while (remaining > 0)
        {
            std::size_t chunk = (remaining > MAX_CHUNK) ? MAX_CHUNK : remaining;

            bool ok = transport.read_reg(reg,
                                         dest + offset,
                                         static_cast<uint16_t>(chunk));
            if (!ok)
            {
                log(LOG_LEVEL_ERROR,
                    "MLX90640::readBlock: read_reg(0x%04X, len=%u) FAILED\r\n",
                    reg,
                    static_cast<unsigned>(chunk));
                return false;
            }

            // MLX90640 uses 16‑bit word addressing for RAM/EEPROM.
            // Each register address advances by 1 word (2 bytes).
            reg = static_cast<uint16_t>(reg + (chunk / 2));
            offset += chunk;
            remaining -= chunk;
        }

        return true;
    }
};
