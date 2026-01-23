#ifndef _OV5640_REGISTERS_HPP_
#define _OV5640_REGISTERS_HPP_

#include <cstdint>

constexpr uint8_t OV5640_ADDRESS = 0x3c;

enum class OV5640_Register : uint16_t
{
    // System and ID
    SYS_RESET02              = 0x3002,
    SYS_CLOCK_ENABLE02       = 0x3006,
    SYS_CTRL0                = 0x3008,
    CHIP_ID                  = 0x300A,
    CHIP_ID_H                = 0x300A,
    CHIP_ID_L                = 0x300B,
    IO_MIPI_CTRL00           = 0x300E,
    PAD_OUTPUT_ENABLE01      = 0x3017,
    PAD_OUTPUT_ENABLE02      = 0x3018,
    PAD_OUTPUT00             = 0x3019,
    SYSTEM_CONTROL1          = 0x302E,

    // PLL and Clock
    SC_PLL_CTRL0             = 0x3034,
    SC_PLL_CTRL1             = 0x3035,
    SC_PLL_CTRL2             = 0x3036,
    SC_PLL_CTRL3             = 0x3037,

    // SCCB and Slave
    SLAVE_ID                 = 0x3100,
    SCCB_SYS_CTRL1           = 0x3103,
    SYS_ROOT_DIVIDER         = 0x3108,

    // AWB
    AWB_R_GAIN               = 0x3400,
    AWB_G_GAIN               = 0x3402,
    AWB_B_GAIN               = 0x3404,
    AWB_MANUAL_CTRL          = 0x3406,

    // Exposure and Gain
    AEC_PK_EXPOSURE_HI       = 0x3500,
    AEC_PK_EXPOSURE_MED      = 0x3501,
    AEC_PK_EXPOSURE_LO       = 0x3502,
    AEC_PK_MANUAL            = 0x3503,
    AEC_PK_REAL_GAIN         = 0x350A,
    AEC_PK_VTS               = 0x350C,

    // Timing
    TIMING_DVPHO             = 0x3808,
    TIMING_DVPVO             = 0x380A,
    TIMING_HTS               = 0x380C,
    TIMING_VTS               = 0x380E,
    TIMING_TC_REG20          = 0x3820,
    TIMING_TC_REG21          = 0x3821,

    // AEC Control
    AEC_CTRL00               = 0x3A00,
    AEC_B50_STEP             = 0x3A08,
    AEC_B60_STEP             = 0x3A0A,
    AEC_CTRL0D               = 0x3A0D,
    AEC_CTRL0E               = 0x3A0E,
    AEC_CTRL0F               = 0x3A0F,
    AEC_CTRL10               = 0x3A10,
    AEC_CTRL11               = 0x3A11,
    AEC_CTRL1B               = 0x3A1B,
    AEC_CTRL1E               = 0x3A1E,
    AEC_CTRL1F               = 0x3A1F,

    // Flicker and Noise
    HZ5060_CTRL00            = 0x3C00,
    HZ5060_CTRL01            = 0x3C01,
    SIGMADELTA_CTRL0C        = 0x3C0C,

    // Frame and Format
    FRAME_CTRL01             = 0x4202,
    FORMAT_CONTROL00         = 0x4300,

    // VFIFO and JPEG
    VFIFO_HSIZE              = 0x4602,
    VFIFO_VSIZE              = 0x4604,
    JPG_MODE_SELECT          = 0x4713,
    POLARITY_CTRL00          = 0x4740,

    // MIPI and Debug
    MIPI_CTRL00              = 0x4800,
    DEBUG_MODE               = 0x4814,

    // ISP and SDE
    ISP_FORMAT_MUX_CTRL      = 0x501F,
    PRE_ISP_TEST_SET1        = 0x503D,
    SDE_CTRL0                = 0x5580,
    SDE_CTRL1                = 0x5581,
    SDE_CTRL3                = 0x5583,
    SDE_CTRL4                = 0x5584,
    SDE_CTRL5                = 0x5585,

    // Misc
    AVG_READOUT              = 0x56A1
};

enum class OV5640_ModeID : uint8_t {
    QCIF_176x144   = 0,
    QVGA_320x240   = 1,
    VGA_640x480    = 2,
    NTSC_720x480   = 3,
    PAL_720x576    = 4,
    XGA_1024x768   = 5,
    HD_1280x720    = 6,
    FHD_1920x1080  = 7,
    QSXGA_2592x1944 = 8
};

enum class OV5640_FormatID : uint8_t {
    YUV422 = 0,
    RGB565 = 1,
    JPEG   = 2
};

enum class OV5640_ColorEffectID : uint8_t {
    None      = 0x00,
    Negative  = 0x40,
    Grayscale = 0x20,
    RedTint   = 0x60,
    GreenTint = 0x80,
    BlueTint  = 0xA0,
    Sepia     = 0xC0
};

#endif // _OV5640_REGISTERS_HPP_


