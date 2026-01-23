#ifndef _OV2640_REGISTERS_HPP_
#define _OV2640_REGISTERS_HPP_

#include <cstdint>

// OV2640 default SCCB address (7-bit)
constexpr uint8_t OV2640_ADDRESS = 0x30;   // 0x60 write, 0x61 read

// OV2640 uses 8-bit register addresses
enum class OV2640_Register : uint8_t
{
    // Bank select
    BANK_SEL            = 0xFF,
    BANK_SENSOR         = 0x01,
    BANK_DSP            = 0x00,

    // Sensor Bank (0x01)
    REG_PID             = 0x0A,   // Product ID high
    REG_VER             = 0x0B,   // Product ID low

    REG_COM1            = 0x03,
    REG_COM2            = 0x04,
    REG_COM3            = 0x0C,
    REG_COM4            = 0x0D,
    REG_COM5            = 0x0E,
    REG_COM6            = 0x0F,

    REG_AEC             = 0x10,
    REG_CLKRC           = 0x11,
    REG_COM7            = 0x12,   // Reset / format control
    REG_COM8            = 0x13,
    REG_COM9            = 0x14,
    REG_COM10           = 0x15,

    REG_HSTART          = 0x17,
    REG_HSTOP           = 0x18,
    REG_VSTART          = 0x19,
    REG_VSTOP           = 0x1A,
    REG_MIDH            = 0x1C,
    REG_MIDL            = 0x1D,

    REG_AEW             = 0x24,
    REG_AEB             = 0x25,
    REG_VPT             = 0x26,

    REG_HREF            = 0x32,
    REG_TSLB            = 0x3A,
    REG_COM11           = 0x3B,
    REG_COM12           = 0x3C,
    REG_COM13           = 0x3D,
    REG_COM14           = 0x3E,

    REG_EDGE            = 0x3F,
    REG_COM15           = 0x40,
    REG_COM16           = 0x41,
    REG_COM17           = 0x42,

    // DSP Bank (0x00)
    DSP_R_BYPASS        = 0x05,
    DSP_QS              = 0x44,
    DSP_CTRL0           = 0xC2,
    DSP_CTRL1           = 0xC3,
    DSP_CTRL2           = 0xC4,
    DSP_CTRL3           = 0xC5,
    DSP_CTRL4           = 0xC6,

    DSP_HSIZE1          = 0xC0,
    DSP_HSIZE2          = 0xC1,
    DSP_VSIZE1          = 0xC2,
    DSP_VSIZE2          = 0xC3,

    DSP_OUT_CTRL        = 0xD7,
    DSP_OUT_CTRL2       = 0xD9,

    DSP_FORMAT_CTRL     = 0xDA,   // YUV/RGB/JPEG
    DSP_FORMAT_MUX      = 0xDA,

    DSP_RESET           = 0xE0,
    DSP_IMAGE_MODE      = 0xDA,
    DSP_IMAGE_MODE2     = 0xE1,

    DSP_ZMOW            = 0x5A,
    DSP_ZMOH            = 0x5B,
    DSP_ZMHH            = 0x5C,

    DSP_RESET2          = 0xE0,
};

// OV2640 supported resolutions
enum class OV2640_ModeID : uint8_t {
    QQVGA_160x120   = 0,
    QVGA_320x240    = 1,
    VGA_640x480     = 2,
    SVGA_800x600    = 3,
    XGA_1024x768    = 4,
    SXGA_1280x1024  = 5,
    UXGA_1600x1200  = 6
};

// OV2640 output formats
enum class OV2640_FormatValue : uint8_t {
    YUV422 = 0x30,
    RGB565 = 0x61,
    JPEG   = 0x10
};

// OV2640 color effects
enum class OV2640_ColorEffectID : uint8_t {
    None      = 0x00,
    Negative  = 0x40,
    Grayscale = 0x20,
    RedTint   = 0x60,
    GreenTint = 0x80,
    BlueTint  = 0xA0,
    Sepia     = 0xC0
};

enum class OV2640_TestPattern : uint8_t {
    Disabled = 0x00,
    ColorBar = 0x02
};

constexpr uint8_t OV2640_COM7_SRST = 0x80;

#endif // _OV2640_REGISTERS_HPP_
