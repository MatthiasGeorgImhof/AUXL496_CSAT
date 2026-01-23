#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "OV2640.hpp"
#include <array>
#include <vector>
#include <cstdint>
#include <cstring>

// Dummy config for RegisterModeTransport concept
struct DummyConfig
{
    using mode_tag = register_mode_tag;
};

// Mock register‑mode transport
class MockTransport
{
public:
    using config_type = DummyConfig;

    bool write_reg(uint16_t reg, const uint8_t* data, size_t size)
    {
        last_reg  = reg;
        last_write.assign(data, data + size);
        return write_ok;
    }

    bool read_reg(uint16_t reg, uint8_t* data, size_t size)
    {
        last_reg = reg;
        last_read.resize(size);

        for (size_t i = 0; i < size; ++i)
            last_read[i] = (i < mock_response.size()) ? mock_response[i] : uint8_t{0};

        std::memcpy(data, last_read.data(), size);
        return read_ok;
    }

    void setMockResponse(std::initializer_list<uint8_t> bytes)
    {
        mock_response.assign(bytes.begin(), bytes.end());
    }

    uint16_t              last_reg{0};
    std::vector<uint8_t>  last_write;
    std::vector<uint8_t>  last_read;
    std::vector<uint8_t>  mock_response;
    bool                  write_ok{true};
    bool                  read_ok{true};
};

//
// ─────────────────────────────────────────────────────────────
//  Basic register access tests
// ─────────────────────────────────────────────────────────────
//

TEST_CASE("writeRegister(enum, uint8_t) forwards to uint8_t register address")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    cam.writeRegister(OV2640_Register::REG_COM7, 0x80);

    CHECK(tx.last_reg == static_cast<uint8_t>(OV2640_Register::REG_COM7));
    CHECK(tx.last_write == std::vector<uint8_t>({0x80}));
}

TEST_CASE("writeRegister(enum, multi-byte) writes raw bytes (no endian swap)")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    uint8_t payload[4] = {0x11, 0x22, 0x33, 0x44};
    cam.writeRegister(OV2640_Register::DSP_CTRL0, payload, 4);

    CHECK(tx.last_reg == static_cast<uint8_t>(OV2640_Register::DSP_CTRL0));
    CHECK(tx.last_write == std::vector<uint8_t>({0x11, 0x22, 0x33, 0x44}));
}

TEST_CASE("readRegister(enum) forwards to uint8_t register address")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    tx.setMockResponse({0xAB});
    uint8_t result = cam.readRegister(OV2640_Register::REG_PID);

    CHECK(result == 0xAB);
    CHECK(tx.last_reg == static_cast<uint8_t>(OV2640_Register::REG_PID));
}

TEST_CASE("readRegister(enum, multi-byte) copies raw bytes (no endian swap)")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    tx.setMockResponse({0x12, 0x34, 0x56, 0x78});

    uint8_t buffer[4] = {};
    bool ok = cam.readRegister(OV2640_Register::DSP_CTRL1, buffer, 4);

    CHECK(ok);
    CHECK(buffer[0] == 0x12);
    CHECK(buffer[1] == 0x34);
    CHECK(buffer[2] == 0x56);
    CHECK(buffer[3] == 0x78);
    CHECK(tx.last_reg == static_cast<uint8_t>(OV2640_Register::DSP_CTRL1));
}

TEST_CASE("writeRegister rejects odd-sized payloads")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    uint8_t data[3] = {0x01, 0x02, 0x03};
    bool ok = cam.writeRegister(OV2640_Register::DSP_CTRL0, data, 3);
    CHECK_FALSE(ok);
}

TEST_CASE("readRegister rejects odd-sized buffers")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    tx.setMockResponse({0x12, 0x34});
    uint8_t buffer[3];
    bool ok = cam.readRegister(OV2640_Register::DSP_CTRL0, buffer, 3);
    CHECK_FALSE(ok);
}

//
// ─────────────────────────────────────────────────────────────
//  High-level API tests
// ─────────────────────────────────────────────────────────────
//

TEST_CASE("init() performs bank switch + soft reset and loads table")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    cam.init();

    CHECK(tx.last_reg == 0x00);
    CHECK(tx.last_write[0] == 0x00);
}

TEST_CASE("setFormat writes correct DSP_FORMAT_CTRL values")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    cam.setFormat(PixelFormat::YUV422);
    CHECK(tx.last_reg == static_cast<uint8_t>(OV2640_Register::DSP_FORMAT_CTRL));
    CHECK(tx.last_write[0] == 0x30);

    cam.setFormat(PixelFormat::RGB565);
    CHECK(tx.last_write[0] == 0x61);

    cam.setFormat(PixelFormat::JPEG);
    CHECK(tx.last_write[0] == 0x10);
}

TEST_CASE("setResolution writes ZMOW/ZMOH/ZMHH correctly")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    cam.setResolution(320, 240);

    // Last write is ZMHH
    CHECK(tx.last_reg == static_cast<uint8_t>(OV2640_Register::DSP_ZMHH));

    uint8_t expected_high =
        ((320 & 0x03) << 2) |
        ((240 & 0x03));

    CHECK(tx.last_write[0] == expected_high);
}

TEST_CASE("enableTestPattern writes correct DSP_IMAGE_MODE value")
{
    MockTransport tx;
    OV2640<MockTransport> cam(tx);

    cam.enableTestPattern(true);
    CHECK(tx.last_reg == static_cast<uint8_t>(OV2640_Register::DSP_IMAGE_MODE));
    CHECK(tx.last_write[0] == 0x02);

    cam.enableTestPattern(false);
    CHECK(tx.last_write[0] == 0x00);
}

//
// ─────────────────────────────────────────────────────────────
//  Concept tests
// ─────────────────────────────────────────────────────────────
//

TEST_CASE("MockTransport satisfies RegisterModeTransport")
{
    static_assert(RegisterModeTransport<MockTransport>);
    CHECK(true);
}
