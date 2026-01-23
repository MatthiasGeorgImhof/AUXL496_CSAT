/*
 * OV5650_Initialiation.hpp
 *
 *  Created on: Jan 17, 2026
 *      Author: mgi
 */

#ifndef OV5650_INITIALIATION_HPP_
#define OV5650_INITIALIATION_HPP_

#include <cstdint>
#include "OVXXXX_Common.hpp"

namespace OV5640_REGISTERS_STM
{
    constexpr uint16_t SYSREM_RESET00 = 0x3000U;
    constexpr uint16_t SYSREM_RESET01 = 0x3001U;
    constexpr uint16_t SYSREM_RESET02 = 0x3002U;
    constexpr uint16_t SYSREM_RESET03 = 0x3003U;
    constexpr uint16_t CLOCK_ENABLE00 = 0x3004U;
    constexpr uint16_t CLOCK_ENABLE01 = 0x3005U;
    constexpr uint16_t CLOCK_ENABLE02 = 0x3006U;
    constexpr uint16_t CLOCK_ENABLE03 = 0x3007U;
    constexpr uint16_t SYSTEM_CTROL0 = 0x3008U;
    constexpr uint16_t CHIP_ID_HIGH_BYTE = 0x300AU;
    constexpr uint16_t CHIP_ID_LOW_BYTE = 0x300BU;
    constexpr uint16_t MIPI_CONTROL00 = 0x300EU;
    constexpr uint16_t PAD_OUTPUT_ENABLE00 = 0x3016U;
    constexpr uint16_t PAD_OUTPUT_ENABLE01 = 0x3017U;
    constexpr uint16_t PAD_OUTPUT_ENABLE02 = 0x3018U;
    constexpr uint16_t PAD_OUTPUT_VALUE00 = 0x3019U;
    constexpr uint16_t PAD_OUTPUT_VALUE01 = 0x301AU;
    constexpr uint16_t PAD_OUTPUT_VALUE02 = 0x301BU;
    constexpr uint16_t PAD_SELECT00 = 0x301CU;
    constexpr uint16_t PAD_SELECT01 = 0x301DU;
    constexpr uint16_t PAD_SELECT02 = 0x301EU;
    constexpr uint16_t CHIP_REVISION = 0x302AU;
    constexpr uint16_t PAD_CONTROL00 = 0x301CU;
    constexpr uint16_t SC_PWC = 0x3031U;
    constexpr uint16_t SC_PLL_CONTRL0 = 0x3034U;
    constexpr uint16_t SC_PLL_CONTRL1 = 0x3035U;
    constexpr uint16_t SC_PLL_CONTRL2 = 0x3036U;
    constexpr uint16_t SC_PLL_CONTRL3 = 0x3037U;
    constexpr uint16_t SC_PLL_CONTRL4 = 0x3038U;
    constexpr uint16_t SC_PLL_CONTRL5 = 0x3039U;
    constexpr uint16_t SC_PLLS_CTRL0 = 0x303AU;
    constexpr uint16_t SC_PLLS_CTRL1 = 0x303BU;
    constexpr uint16_t SC_PLLS_CTRL2 = 0x303CU;
    constexpr uint16_t SC_PLLS_CTRL3 = 0x303DU;
    constexpr uint16_t IO_PAD_VALUE00 = 0x3050U;
    constexpr uint16_t IO_PAD_VALUE01 = 0x3051U;
    constexpr uint16_t IO_PAD_VALUE02 = 0x3052U;

    /* SCCB control [0x3100 ~ 0x3108]                       */
    constexpr uint16_t SCCB_ID = 0x3100U;
    constexpr uint16_t SCCB_SYSTEM_CTRL0 = 0x3102U;
    constexpr uint16_t SCCB_SYSTEM_CTRL1 = 0x3103U;
    constexpr uint16_t SYSTEM_ROOT_DIVIDER = 0x3108U;

    /* SRB control [= 0x3200 ~ 0x3213]                        */
    constexpr uint16_t GROUP_ADDR0 = 0x3200U;
    constexpr uint16_t GROUP_ADDR1 = 0x3201U;
    constexpr uint16_t GROUP_ADDR2 = 0x3202U;
    constexpr uint16_t GROUP_ADDR3 = 0x3203U;
    constexpr uint16_t SRM_GROUP_ACCESS = 0x3212U;
    constexpr uint16_t SRM_GROUP_STATUS = 0x3213U;

    /* AWB gain control [= 0x3400 ~ 0x3406]                   */
    constexpr uint16_t AWB_R_GAIN_MSB = 0x3400U;
    constexpr uint16_t AWB_R_GAIN_LSB = 0x3401U;
    constexpr uint16_t AWB_G_GAIN_MSB = 0x3402U;
    constexpr uint16_t AWB_G_GAIN_LSB = 0x3403U;
    constexpr uint16_t AWB_B_GAIN_MSB = 0x3404U;
    constexpr uint16_t AWB_B_GAIN_LSB = 0x3405U;
    constexpr uint16_t AWB_MANUAL_CONTROL = 0x3406U;

    /* AEC/AGC control [= 0x3500 ~ 0x350D]                    */
    constexpr uint16_t AEC_PK_EXPOSURE_19_16 = 0x3500U;
    constexpr uint16_t AEC_PK_EXPOSURE_HIGH = 0x3501U;
    constexpr uint16_t AEC_PK_EXPOSURE_LOW = 0x3502U;
    constexpr uint16_t AEC_PK_MANUAL = 0x3503U;
    constexpr uint16_t AEC_PK_REAL_GAIN_9_8 = 0x350AU;
    constexpr uint16_t AEC_PK_REAL_GAIN_LOW = 0x350BU;
    constexpr uint16_t AEC_PK_VTS_HIGH = 0x350CU;
    constexpr uint16_t AEC_PK_VTS_LOW = 0x350DU;

    /* VCM control [= 0x3600 ~ 0x3606]                        */
    constexpr uint16_t VCM_CONTROL_0 = 0x3602U;
    constexpr uint16_t VCM_CONTROL_1 = 0x3603U;
    constexpr uint16_t VCM_CONTROL_2 = 0x3604U;
    constexpr uint16_t VCM_CONTROL_3 = 0x3605U;
    constexpr uint16_t VCM_CONTROL_4 = 0x3606U;

    /* timing control [= 0x3800 ~ 0x3821]                    */
    constexpr uint16_t TIMING_HS_HIGH = 0x3800U;
    constexpr uint16_t TIMING_HS_LOW = 0x3801U;
    constexpr uint16_t TIMING_VS_HIGH = 0x3802U;
    constexpr uint16_t TIMING_VS_LOW = 0x3803U;
    constexpr uint16_t TIMING_HW_HIGH = 0x3804U;
    constexpr uint16_t TIMING_HW_LOW = 0x3805U;
    constexpr uint16_t TIMING_VH_HIGH = 0x3806U;
    constexpr uint16_t TIMING_VH_LOW = 0x3807U;
    constexpr uint16_t TIMING_DVPHO_HIGH = 0x3808U;
    constexpr uint16_t TIMING_DVPHO_LOW = 0x3809U;
    constexpr uint16_t TIMING_DVPVO_HIGH = 0x380AU;
    constexpr uint16_t TIMING_DVPVO_LOW = 0x380BU;
    constexpr uint16_t TIMING_HTS_HIGH = 0x380CU;
    constexpr uint16_t TIMING_HTS_LOW = 0x380DU;
    constexpr uint16_t TIMING_VTS_HIGH = 0x380EU;
    constexpr uint16_t TIMING_VTS_LOW = 0x380FU;
    constexpr uint16_t TIMING_HOFFSET_HIGH = 0x3810U;
    constexpr uint16_t TIMING_HOFFSET_LOW = 0x3811U;
    constexpr uint16_t TIMING_VOFFSET_HIGH = 0x3812U;
    constexpr uint16_t TIMING_VOFFSET_LOW = 0x3813U;
    constexpr uint16_t TIMING_X_INC = 0x3814U;
    constexpr uint16_t TIMING_Y_INC = 0x3815U;
    constexpr uint16_t HSYNC_START_HIGH = 0x3816U;
    constexpr uint16_t HSYNC_START_LOW = 0x3817U;
    constexpr uint16_t HSYNC_WIDTH_HIGH = 0x3818U;
    constexpr uint16_t HSYNC_WIDTH_LOW = 0x3819U;
    constexpr uint16_t TIMING_TC_REG20 = 0x3820U;
    constexpr uint16_t TIMING_TC_REG21 = 0x3821U;

    /* AEC/AGC power down domain control [= 0x3A00 ~ 0x3A25] */
    constexpr uint16_t AEC_CTRL00 = 0x3A00U;
    constexpr uint16_t AEC_CTRL01 = 0x3A01U;
    constexpr uint16_t AEC_CTRL02 = 0x3A02U;
    constexpr uint16_t AEC_CTRL03 = 0x3A03U;
    constexpr uint16_t AEC_CTRL04 = 0x3A04U;
    constexpr uint16_t AEC_CTRL05 = 0x3A05U;
    constexpr uint16_t AEC_CTRL06 = 0x3A06U;
    constexpr uint16_t AEC_CTRL07 = 0x3A07U;
    constexpr uint16_t AEC_B50_STEP_HIGH = 0x3A08U;
    constexpr uint16_t AEC_B50_STEP_LOW = 0x3A09U;
    constexpr uint16_t AEC_B60_STEP_HIGH = 0x3A0AU;
    constexpr uint16_t AEC_B60_STEP_LOW = 0x3A0BU;
    constexpr uint16_t AEC_AEC_CTRL0C = 0x3A0CU;
    constexpr uint16_t AEC_CTRL0D = 0x3A0DU;
    constexpr uint16_t AEC_CTRL0E = 0x3A0EU;
    constexpr uint16_t AEC_CTRL0F = 0x3A0FU;
    constexpr uint16_t AEC_CTRL10 = 0x3A10U;
    constexpr uint16_t AEC_CTRL11 = 0x3A11U;
    constexpr uint16_t AEC_CTRL13 = 0x3A13U;
    constexpr uint16_t AEC_MAX_EXPO_HIGH = 0x3A14U;
    constexpr uint16_t AEC_MAX_EXPO_LOW = 0x3A15U;
    constexpr uint16_t AEC_CTRL17 = 0x3A17U;
    constexpr uint16_t AEC_GAIN_CEILING_HIGH = 0x3A18U;
    constexpr uint16_t AEC_GAIN_CEILING_LOW = 0x3A19U;
    constexpr uint16_t AEC_DIFF_MIN = 0x3A1AU;
    constexpr uint16_t AEC_CTRL1B = 0x3A1BU;
    constexpr uint16_t LED_ADD_ROW_HIGH = 0x3A1CU;
    constexpr uint16_t LED_ADD_ROW_LOW = 0x3A1DU;
    constexpr uint16_t AEC_CTRL1E = 0x3A1EU;
    constexpr uint16_t AEC_CTRL1F = 0x3A1FU;
    constexpr uint16_t AEC_CTRL20 = 0x3A20U;
    constexpr uint16_t AEC_CTRL21 = 0x3A21U;
    constexpr uint16_t AEC_CTRL25 = 0x3A25U;

    /* strobe control [= 0x3B00 ~ 0x3B0C]                      */
    constexpr uint16_t STROBE_CTRL = 0x3B00U;
    constexpr uint16_t FREX_EXPOSURE02 = 0x3B01U;
    constexpr uint16_t FREX_SHUTTER_DLY01 = 0x3B02U;
    constexpr uint16_t FREX_SHUTTER_DLY00 = 0x3B03U;
    constexpr uint16_t FREX_EXPOSURE01 = 0x3B04U;
    constexpr uint16_t FREX_EXPOSURE00 = 0x3B05U;
    constexpr uint16_t FREX_CTRL07 = 0x3B06U;
    constexpr uint16_t FREX_MODE = 0x3B07U;
    constexpr uint16_t FREX_RQST = 0x3B08U;
    constexpr uint16_t FREX_HREF_DLY = 0x3B09U;
    constexpr uint16_t FREX_RST_LENGTH = 0x3B0AU;
    constexpr uint16_t STROBE_WIDTH_HIGH = 0x3B0BU;
    constexpr uint16_t STROBE_WIDTH_LOW = 0x3B0CU;

    /* 50/60Hz detector control [= 0x3C00 ~ 0x3C1E]            */
    constexpr uint16_t _5060HZ_CTRL00 = 0x3C00U;
    constexpr uint16_t _5060HZ_CTRL01 = 0x3C01U;
    constexpr uint16_t _5060HZ_CTRL02 = 0x3C02U;
    constexpr uint16_t _5060HZ_CTRL03 = 0x3C03U;
    constexpr uint16_t _5060HZ_CTRL04 = 0x3C04U;
    constexpr uint16_t _5060HZ_CTRL05 = 0x3C05U;
    constexpr uint16_t LIGHTMETER1_TH_HIGH = 0x3C06U;
    constexpr uint16_t LIGHTMETER1_TH_LOW = 0x3C07U;
    constexpr uint16_t LIGHTMETER2_TH_HIGH = 0x3C08U;
    constexpr uint16_t LIGHTMETER2_TH_LOW = 0x3C09U;
    constexpr uint16_t SAMPLE_NUMBER_HIGH = 0x3C0AU;
    constexpr uint16_t SAMPLE_NUMBER_LOW = 0x3C0BU;
    constexpr uint16_t SIGMA_DELTA_CTRL0C = 0x3C0CU;
    constexpr uint16_t SUM50_BYTE4 = 0x3C0DU;
    constexpr uint16_t SUM50_BYTE3 = 0x3C0EU;
    constexpr uint16_t SUM50_BYTE2 = 0x3C0FU;
    constexpr uint16_t SUM50_BYTE1 = 0x3C10U;
    constexpr uint16_t SUM60_BYTE4 = 0x3C11U;
    constexpr uint16_t SUM60_BYTE3 = 0x3C12U;
    constexpr uint16_t SUM60_BYTE2 = 0x3C13U;
    constexpr uint16_t SUM60_BYTE1 = 0x3C14U;
    constexpr uint16_t SUM5060_HIGH = 0x3C15U;
    constexpr uint16_t SUM5060_LOW = 0x3C16U;
    constexpr uint16_t BLOCK_CNTR_HIGH = 0x3C17U;
    constexpr uint16_t BLOCK_CNTR_LOW = 0x3C18U;
    constexpr uint16_t B6_HIGH = 0x3C19U;
    constexpr uint16_t B6_LOW = 0x3C1AU;
    constexpr uint16_t LIGHTMETER_OUTPUT_BYTE3 = 0x3C1BU;
    constexpr uint16_t LIGHTMETER_OUTPUT_BYTE2 = 0x3C1CU;
    constexpr uint16_t LIGHTMETER_OUTPUT_BYTE1 = 0x3C1DU;
    constexpr uint16_t SUM_THRESHOLD = 0x3C1EU;

    /* OTP control [= 0x3D00 ~ 0x3D21]                         */
    /* MC control [= 0x3F00 ~ 0x3F0D]                          */
    /* BLC control [= 0x4000 ~ 0x4033]                         */
    constexpr uint16_t BLC_CTRL00 = 0x4000U;
    constexpr uint16_t BLC_CTRL01 = 0x4001U;
    constexpr uint16_t BLC_CTRL02 = 0x4002U;
    constexpr uint16_t BLC_CTRL03 = 0x4003U;
    constexpr uint16_t BLC_CTRL04 = 0x4004U;
    constexpr uint16_t BLC_CTRL05 = 0x4005U;

    /* frame control [= 0x4201 ~ 0x4202]                       */
    constexpr uint16_t FRAME_CTRL01 = 0x4201U;
    constexpr uint16_t FRAME_CTRL02 = 0x4202U;

    /* format control [= 0x4300 ~ 0x430D]                      */
    constexpr uint16_t FORMAT_CTRL00 = 0x4300U;
    constexpr uint16_t FORMAT_CTRL01 = 0x4301U;
    constexpr uint16_t YMAX_VAL_HIGH = 0x4302U;
    constexpr uint16_t YMAX_VAL_LOW = 0x4303U;
    constexpr uint16_t YMIN_VAL_HIGH = 0x4304U;
    constexpr uint16_t YMIN_VAL_LOW = 0x4305U;
    constexpr uint16_t UMAX_VAL_HIGH = 0x4306U;
    constexpr uint16_t UMAX_VAL_LOW = 0x4307U;
    constexpr uint16_t UMIN_VAL_HIGH = 0x4308U;
    constexpr uint16_t UMIN_VAL_LOW = 0x4309U;
    constexpr uint16_t VMAX_VAL_HIGH = 0x430AU;
    constexpr uint16_t VMAX_VAL_LOW = 0x430BU;
    constexpr uint16_t VMIN_VAL_HIGH = 0x430CU;
    constexpr uint16_t VMIN_VAL_LOW = 0x430DU;

    /* JPEG control [= 0x4400 ~ 0x4431]                        */
    constexpr uint16_t JPEG_CTRL00 = 0x4400U;
    constexpr uint16_t JPEG_CTRL01 = 0x4401U;
    constexpr uint16_t JPEG_CTRL02 = 0x4402U;
    constexpr uint16_t JPEG_CTRL03 = 0x4403U;
    constexpr uint16_t JPEG_CTRL04 = 0x4404U;
    constexpr uint16_t JPEG_CTRL05 = 0x4405U;
    constexpr uint16_t JPEG_CTRL06 = 0x4406U;
    constexpr uint16_t JPEG_CTRL07 = 0x4407U;
    constexpr uint16_t JPEG_ISI_CTRL1 = 0x4408U;
    constexpr uint16_t JPEG_CTRL09 = 0x4409U;
    constexpr uint16_t JPEG_CTRL0A = 0x440AU;
    constexpr uint16_t JPEG_CTRL0B = 0x440BU;
    constexpr uint16_t JPEG_CTRL0C = 0x440CU;
    constexpr uint16_t JPEG_QT_DATA = 0x4410U;
    constexpr uint16_t JPEG_QT_ADDR = 0x4411U;
    constexpr uint16_t JPEG_ISI_DATA = 0x4412U;
    constexpr uint16_t JPEG_ISI_CTRL2 = 0x4413U;
    constexpr uint16_t JPEG_LENGTH_BYTE3 = 0x4414U;
    constexpr uint16_t JPEG_LENGTH_BYTE2 = 0x4415U;
    constexpr uint16_t JPEG_LENGTH_BYTE1 = 0x4416U;
    constexpr uint16_t JFIFO_OVERFLOW = 0x4417U;

    /* VFIFO control [= 0x4600 ~ 0x460D]                       */
    constexpr uint16_t VFIFO_CTRL00 = 0x4600U;
    constexpr uint16_t VFIFO_HSIZE_HIGH = 0x4602U;
    constexpr uint16_t VFIFO_HSIZE_LOW = 0x4603U;
    constexpr uint16_t VFIFO_VSIZE_HIGH = 0x4604U;
    constexpr uint16_t VFIFO_VSIZE_LOW = 0x4605U;
    constexpr uint16_t VFIFO_CTRL0C = 0x460CU;
    constexpr uint16_t VFIFO_CTRL0D = 0x460DU;

    /* DVP control [= 0x4709 ~ 0x4745]                         */
    constexpr uint16_t DVP_VSYNC_WIDTH0 = 0x4709U;
    constexpr uint16_t DVP_VSYNC_WIDTH1 = 0x470AU;
    constexpr uint16_t DVP_VSYNC_WIDTH2 = 0x470BU;
    constexpr uint16_t PAD_LEFT_CTRL = 0x4711U;
    constexpr uint16_t PAD_RIGHT_CTRL = 0x4712U;
    constexpr uint16_t JPG_MODE_SELECT = 0x4713U;
    constexpr uint16_t _656_DUMMY_LINE = 0x4715U;
    constexpr uint16_t CCIR656_CTRL = 0x4719U;
    constexpr uint16_t HSYNC_CTRL00 = 0x471BU;
    constexpr uint16_t DVP_VSYN_CTRL = 0x471DU;
    constexpr uint16_t DVP_HREF_CTRL = 0x471FU;
    constexpr uint16_t VSTART_OFFSET = 0x4721U;
    constexpr uint16_t VEND_OFFSET = 0x4722U;
    constexpr uint16_t DVP_CTRL23 = 0x4723U;
    constexpr uint16_t CCIR656_CTRL00 = 0x4730U;
    constexpr uint16_t CCIR656_CTRL01 = 0x4731U;
    constexpr uint16_t CCIR656_FS = 0x4732U;
    constexpr uint16_t CCIR656_FE = 0x4733U;
    constexpr uint16_t CCIR656_LS = 0x4734U;
    constexpr uint16_t CCIR656_LE = 0x4735U;
    constexpr uint16_t CCIR656_CTRL06 = 0x4736U;
    constexpr uint16_t CCIR656_CTRL07 = 0x4737U;
    constexpr uint16_t CCIR656_CTRL08 = 0x4738U;
    constexpr uint16_t POLARITY_CTRL = 0x4740U;
    constexpr uint16_t TEST_PATTERN = 0x4741U;
    constexpr uint16_t DATA_ORDER = 0x4745U;

    /* MIPI control [= 0x4800 ~ 0x4837]                        */
    constexpr uint16_t MIPI_CTRL00 = 0x4800U;
    constexpr uint16_t MIPI_CTRL01 = 0x4801U;
    constexpr uint16_t MIPI_CTRL05 = 0x4805U;
    constexpr uint16_t MIPI_DATA_ORDER = 0x480AU;
    constexpr uint16_t MIN_HS_ZERO_HIGH = 0x4818U;
    constexpr uint16_t MIN_HS_ZERO_LOW = 0x4819U;
    constexpr uint16_t MIN_MIPI_HS_TRAIL_HIGH = 0x481AU;
    constexpr uint16_t MIN_MIPI_HS_TRAIL_LOW = 0x481BU;
    constexpr uint16_t MIN_MIPI_CLK_ZERO_HIGH = 0x481CU;
    constexpr uint16_t MIN_MIPI_CLK_ZERO_LOW = 0x481DU;
    constexpr uint16_t MIN_MIPI_CLK_PREPARE_HIGH = 0x481EU;
    constexpr uint16_t MIN_MIPI_CLK_PREPARE_LOW = 0x481FU;
    constexpr uint16_t MIN_CLK_POST_HIGH = 0x4820U;
    constexpr uint16_t MIN_CLK_POST_LOW = 0x4821U;
    constexpr uint16_t MIN_CLK_TRAIL_HIGH = 0x4822U;
    constexpr uint16_t MIN_CLK_TRAIL_LOW = 0x4823U;
    constexpr uint16_t MIN_LPX_PCLK_HIGH = 0x4824U;
    constexpr uint16_t MIN_LPX_PCLK_LOW = 0x4825U;
    constexpr uint16_t MIN_HS_PREPARE_HIGH = 0x4826U;
    constexpr uint16_t MIN_HS_PREPARE_LOW = 0x4827U;
    constexpr uint16_t MIN_HS_EXIT_HIGH = 0x4828U;
    constexpr uint16_t MIN_HS_EXIT_LOW = 0x4829U;
    constexpr uint16_t MIN_HS_ZERO_UI = 0x482AU;
    constexpr uint16_t MIN_HS_TRAIL_UI = 0x482BU;
    constexpr uint16_t MIN_CLK_ZERO_UI = 0x482CU;
    constexpr uint16_t MIN_CLK_PREPARE_UI = 0x482DU;
    constexpr uint16_t MIN_CLK_POST_UI = 0x482EU;
    constexpr uint16_t MIN_CLK_TRAIL_UI = 0x482FU;
    constexpr uint16_t MIN_LPX_PCLK_UI = 0x4830U;
    constexpr uint16_t MIN_HS_PREPARE_UI = 0x4831U;
    constexpr uint16_t MIN_HS_EXIT_UI = 0x4832U;
    constexpr uint16_t PCLK_PERIOD = 0x4837U;

    /* ISP frame control [= 0x4901 ~ 0x4902]                   */
    constexpr uint16_t ISP_FRAME_CTRL01 = 0x4901U;
    constexpr uint16_t ISP_FRAME_CTRL02 = 0x4902U;

    /* ISP top control [= 0x5000 ~ 0x5063]                     */
    constexpr uint16_t ISP_CONTROL00 = 0x5000U;
    constexpr uint16_t ISP_CONTROL01 = 0x5001U;
    constexpr uint16_t ISP_CONTROL03 = 0x5003U;
    constexpr uint16_t ISP_CONTROL05 = 0x5005U;
    constexpr uint16_t ISP_MISC0 = 0x501DU;
    constexpr uint16_t ISP_MISC1 = 0x501EU;
    constexpr uint16_t FORMAT_MUX_CTRL = 0x501FU;
    constexpr uint16_t DITHER_CTRL0 = 0x5020U;
    constexpr uint16_t DRAW_WINDOW_CTRL00 = 0x5027U;
    constexpr uint16_t DRAW_WINDOW_LEFT_CTRL_HIGH = 0x5028U;
    constexpr uint16_t DRAW_WINDOW_LEFT_CTRL_LOW = 0x5029U;
    constexpr uint16_t DRAW_WINDOW_RIGHT_CTRL_HIGH = 0x502AU;
    constexpr uint16_t DRAW_WINDOW_RIGHT_CTRL_LOW = 0x502BU;
    constexpr uint16_t DRAW_WINDOW_TOP_CTRL_HIGH = 0x502CU;
    constexpr uint16_t DRAW_WINDOW_TOP_CTRL_LOW = 0x502DU;
    constexpr uint16_t DRAW_WINDOW_BOTTOM_CTRL_HIGH = 0x502EU;
    constexpr uint16_t DRAW_WINDOW_BOTTOM_CTRL_LOW = 0x502FU;
    constexpr uint16_t DRAW_WINDOW_HBW_CTRL_HIGH = 0x5030U; /* HBW: Horizontal Boundary Width */
    constexpr uint16_t DRAW_WINDOW_HBW_CTRL_LOW = 0x5031U;
    constexpr uint16_t DRAW_WINDOW_VBW_CTRL_HIGH = 0x5032U; /* VBW: Vertical Boundary Width */
    constexpr uint16_t DRAW_WINDOW_VBW_CTRL_LOW = 0x5033U;
    constexpr uint16_t DRAW_WINDOW_Y_CTRL = 0x5034U;
    constexpr uint16_t DRAW_WINDOW_U_CTRL = 0x5035U;
    constexpr uint16_t DRAW_WINDOW_V_CTRL = 0x5036U;
    constexpr uint16_t PRE_ISP_TEST_SETTING1 = 0x503DU;
    constexpr uint16_t ISP_SENSOR_BIAS_I = 0x5061U;
    constexpr uint16_t ISP_SENSOR_GAIN1_I = 0x5062U;
    constexpr uint16_t ISP_SENSOR_GAIN2_I = 0x5063U;

    /* AWB control [= 0x5180 ~ 0x51D0]                         */
    constexpr uint16_t AWB_CTRL00 = 0x5180U;
    constexpr uint16_t AWB_CTRL01 = 0x5181U;
    constexpr uint16_t AWB_CTRL02 = 0x5182U;
    constexpr uint16_t AWB_CTRL03 = 0x5183U;
    constexpr uint16_t AWB_CTRL04 = 0x5184U;
    constexpr uint16_t AWB_CTRL05 = 0x5185U;
    constexpr uint16_t AWB_CTRL06 = 0x5186U; /* Advanced AWB control registers: = 0x5186 ~ 0x5190 */
    constexpr uint16_t AWB_CTRL07 = 0x5187U;
    constexpr uint16_t AWB_CTRL08 = 0x5188U;
    constexpr uint16_t AWB_CTRL09 = 0x5189U;
    constexpr uint16_t AWB_CTRL10 = 0x518AU;
    constexpr uint16_t AWB_CTRL11 = 0x518BU;
    constexpr uint16_t AWB_CTRL12 = 0x518CU;
    constexpr uint16_t AWB_CTRL13 = 0x518DU;
    constexpr uint16_t AWB_CTRL14 = 0x518EU;
    constexpr uint16_t AWB_CTRL15 = 0x518FU;
    constexpr uint16_t AWB_CTRL16 = 0x5190U;
    constexpr uint16_t AWB_CTRL17 = 0x5191U;
    constexpr uint16_t AWB_CTRL18 = 0x5192U;
    constexpr uint16_t AWB_CTRL19 = 0x5193U;
    constexpr uint16_t AWB_CTRL20 = 0x5194U;
    constexpr uint16_t AWB_CTRL21 = 0x5195U;
    constexpr uint16_t AWB_CTRL22 = 0x5196U;
    constexpr uint16_t AWB_CTRL23 = 0x5197U;
    constexpr uint16_t AWB_CTRL24 = 0x5198U;
    constexpr uint16_t AWB_CTRL25 = 0x5199U;
    constexpr uint16_t AWB_CTRL26 = 0x519AU;
    constexpr uint16_t AWB_CTRL27 = 0x519BU;
    constexpr uint16_t AWB_CTRL28 = 0x519CU;
    constexpr uint16_t AWB_CTRL29 = 0x519DU;
    constexpr uint16_t AWB_CTRL30 = 0x519EU;
    constexpr uint16_t AWB_CURRENT_R_GAIN_HIGH = 0x519FU;
    constexpr uint16_t AWB_CURRENT_R_GAIN_LOW = 0x51A0U;
    constexpr uint16_t AWB_CURRENT_G_GAIN_HIGH = 0x51A1U;
    constexpr uint16_t AWB_CURRENT_G_GAIN_LOW = 0x51A2U;
    constexpr uint16_t AWB_CURRENT_B_GAIN_HIGH = 0x51A3U;
    constexpr uint16_t AWB_CURRENT_B_GAIN_LOW = 0x51A4U;
    constexpr uint16_t AWB_AVERAGE_R = 0x51A5U;
    constexpr uint16_t AWB_AVERAGE_G = 0x51A6U;
    constexpr uint16_t AWB_AVERAGE_B = 0x51A7U;
    constexpr uint16_t AWB_CTRL74 = 0x5180U;

    /* CIP control [= 0x5300 ~ 0x530F]                         */
    constexpr uint16_t CIP_SHARPENMT_TH1 = 0x5300U;
    constexpr uint16_t CIP_SHARPENMT_TH2 = 0x5301U;
    constexpr uint16_t CIP_SHARPENMT_OFFSET1 = 0x5302U;
    constexpr uint16_t CIP_SHARPENMT_OFFSET2 = 0x5303U;
    constexpr uint16_t CIP_DNS_TH1 = 0x5304U;
    constexpr uint16_t CIP_DNS_TH2 = 0x5305U;
    constexpr uint16_t CIP_DNS_OFFSET1 = 0x5306U;
    constexpr uint16_t CIP_DNS_OFFSET2 = 0x5307U;
    constexpr uint16_t CIP_CTRL = 0x5308U;
    constexpr uint16_t CIP_SHARPENTH_TH1 = 0x5309U;
    constexpr uint16_t CIP_SHARPENTH_TH2 = 0x530AU;
    constexpr uint16_t CIP_SHARPENTH_OFFSET1 = 0x530BU;
    constexpr uint16_t CIP_SHARPENTH_OFFSET2 = 0x530CU;
    constexpr uint16_t CIP_EDGE_MT_AUTO = 0x530DU;
    constexpr uint16_t CIP_DNS_TH_AUTO = 0x530EU;
    constexpr uint16_t CIP_SHARPEN_TH_AUTO = 0x530FU;

    /* CMX control [= 0x5380 ~ 0x538B]                         */
    constexpr uint16_t CMX_CTRL = 0x5380U;
    constexpr uint16_t CMX1 = 0x5381U;
    constexpr uint16_t CMX2 = 0x5382U;
    constexpr uint16_t CMX3 = 0x5383U;
    constexpr uint16_t CMX4 = 0x5384U;
    constexpr uint16_t CMX5 = 0x5385U;
    constexpr uint16_t CMX6 = 0x5386U;
    constexpr uint16_t CMX7 = 0x5387U;
    constexpr uint16_t CMX8 = 0x5388U;
    constexpr uint16_t CMX9 = 0x5389U;
    constexpr uint16_t CMXSIGN_HIGH = 0x538AU;
    constexpr uint16_t CMXSIGN_LOW = 0x538BU;

    /* gamma control [= 0x5480 ~ 0x5490]                       */
    constexpr uint16_t GAMMA_CTRL00 = 0x5480U;
    constexpr uint16_t GAMMA_YST00 = 0x5481U;
    constexpr uint16_t GAMMA_YST01 = 0x5482U;
    constexpr uint16_t GAMMA_YST02 = 0x5483U;
    constexpr uint16_t GAMMA_YST03 = 0x5484U;
    constexpr uint16_t GAMMA_YST04 = 0x5485U;
    constexpr uint16_t GAMMA_YST05 = 0x5486U;
    constexpr uint16_t GAMMA_YST06 = 0x5487U;
    constexpr uint16_t GAMMA_YST07 = 0x5488U;
    constexpr uint16_t GAMMA_YST08 = 0x5489U;
    constexpr uint16_t GAMMA_YST09 = 0x548AU;
    constexpr uint16_t GAMMA_YST0A = 0x548BU;
    constexpr uint16_t GAMMA_YST0B = 0x548CU;
    constexpr uint16_t GAMMA_YST0C = 0x548DU;
    constexpr uint16_t GAMMA_YST0D = 0x548EU;
    constexpr uint16_t GAMMA_YST0E = 0x548FU;
    constexpr uint16_t GAMMA_YST0F = 0x5490U;

    /* SDE control [= 0x5580 ~ 0x558C]                         */
    constexpr uint16_t SDE_CTRL0 = 0x5580U;
    constexpr uint16_t SDE_CTRL1 = 0x5581U;
    constexpr uint16_t SDE_CTRL2 = 0x5582U;
    constexpr uint16_t SDE_CTRL3 = 0x5583U;
    constexpr uint16_t SDE_CTRL4 = 0x5584U;
    constexpr uint16_t SDE_CTRL5 = 0x5585U;
    constexpr uint16_t SDE_CTRL6 = 0x5586U;
    constexpr uint16_t SDE_CTRL7 = 0x5587U;
    constexpr uint16_t SDE_CTRL8 = 0x5588U;
    constexpr uint16_t SDE_CTRL9 = 0x5589U;
    constexpr uint16_t SDE_CTRL10 = 0x558AU;
    constexpr uint16_t SDE_CTRL11 = 0x558BU;
    constexpr uint16_t SDE_CTRL12 = 0x558CU;

    /* scale control [= 0x5600 ~ 0x5606]                       */
    constexpr uint16_t SCALE_CTRL0 = 0x5600U;
    constexpr uint16_t SCALE_CTRL1 = 0x5601U;
    constexpr uint16_t SCALE_CTRL2 = 0x5602U;
    constexpr uint16_t SCALE_CTRL3 = 0x5603U;
    constexpr uint16_t SCALE_CTRL4 = 0x5604U;
    constexpr uint16_t SCALE_CTRL5 = 0x5605U;
    constexpr uint16_t SCALE_CTRL6 = 0x5606U;

    /* AVG control [= 0x5680 ~ 0x56A2]                         */
    constexpr uint16_t X_START_HIGH = 0x5680U;
    constexpr uint16_t X_START_LOW = 0x5681U;
    constexpr uint16_t Y_START_HIGH = 0x5682U;
    constexpr uint16_t Y_START_LOW = 0x5683U;
    constexpr uint16_t X_WINDOW_HIGH = 0x5684U;
    constexpr uint16_t X_WINDOW_LOW = 0x5685U;
    constexpr uint16_t Y_WINDOW_HIGH = 0x5686U;
    constexpr uint16_t Y_WINDOW_LOW = 0x5687U;
    constexpr uint16_t WEIGHT00 = 0x5688U;
    constexpr uint16_t WEIGHT01 = 0x5689U;
    constexpr uint16_t WEIGHT02 = 0x568AU;
    constexpr uint16_t WEIGHT03 = 0x568BU;
    constexpr uint16_t WEIGHT04 = 0x568CU;
    constexpr uint16_t WEIGHT05 = 0x568DU;
    constexpr uint16_t WEIGHT06 = 0x568EU;
    constexpr uint16_t WEIGHT07 = 0x568FU;
    constexpr uint16_t AVG_CTRL10 = 0x5690U;
    constexpr uint16_t AVG_WIN_00 = 0x5691U;
    constexpr uint16_t AVG_WIN_01 = 0x5692U;
    constexpr uint16_t AVG_WIN_02 = 0x5693U;
    constexpr uint16_t AVG_WIN_03 = 0x5694U;
    constexpr uint16_t AVG_WIN_10 = 0x5695U;
    constexpr uint16_t AVG_WIN_11 = 0x5696U;
    constexpr uint16_t AVG_WIN_12 = 0x5697U;
    constexpr uint16_t AVG_WIN_13 = 0x5698U;
    constexpr uint16_t AVG_WIN_20 = 0x5699U;
    constexpr uint16_t AVG_WIN_21 = 0x569AU;
    constexpr uint16_t AVG_WIN_22 = 0x569BU;
    constexpr uint16_t AVG_WIN_23 = 0x569CU;
    constexpr uint16_t AVG_WIN_30 = 0x569DU;
    constexpr uint16_t AVG_WIN_31 = 0x569EU;
    constexpr uint16_t AVG_WIN_32 = 0x569FU;
    constexpr uint16_t AVG_WIN_33 = 0x56A0U;
    constexpr uint16_t AVG_READOUT = 0x56A1U;
    constexpr uint16_t AVG_WEIGHT_SUM = 0x56A2U;

    /* LENC control [= 0x5800 ~ 0x5849]                        */
    constexpr uint16_t GMTRX00 = 0x5800U;
    constexpr uint16_t GMTRX01 = 0x5801U;
    constexpr uint16_t GMTRX02 = 0x5802U;
    constexpr uint16_t GMTRX03 = 0x5803U;
    constexpr uint16_t GMTRX04 = 0x5804U;
    constexpr uint16_t GMTRX05 = 0x5805U;
    constexpr uint16_t GMTRX10 = 0x5806U;
    constexpr uint16_t GMTRX11 = 0x5807U;
    constexpr uint16_t GMTRX12 = 0x5808U;
    constexpr uint16_t GMTRX13 = 0x5809U;
    constexpr uint16_t GMTRX14 = 0x580AU;
    constexpr uint16_t GMTRX15 = 0x580BU;
    constexpr uint16_t GMTRX20 = 0x580CU;
    constexpr uint16_t GMTRX21 = 0x580DU;
    constexpr uint16_t GMTRX22 = 0x580EU;
    constexpr uint16_t GMTRX23 = 0x580FU;
    constexpr uint16_t GMTRX24 = 0x5810U;
    constexpr uint16_t GMTRX25 = 0x5811U;
    constexpr uint16_t GMTRX30 = 0x5812U;
    constexpr uint16_t GMTRX31 = 0x5813U;
    constexpr uint16_t GMTRX32 = 0x5814U;
    constexpr uint16_t GMTRX33 = 0x5815U;
    constexpr uint16_t GMTRX34 = 0x5816U;
    constexpr uint16_t GMTRX35 = 0x5817U;
    constexpr uint16_t GMTRX40 = 0x5818U;
    constexpr uint16_t GMTRX41 = 0x5819U;
    constexpr uint16_t GMTRX42 = 0x581AU;
    constexpr uint16_t GMTRX43 = 0x581BU;
    constexpr uint16_t GMTRX44 = 0x581CU;
    constexpr uint16_t GMTRX45 = 0x581DU;
    constexpr uint16_t GMTRX50 = 0x581EU;
    constexpr uint16_t GMTRX51 = 0x581FU;
    constexpr uint16_t GMTRX52 = 0x5820U;
    constexpr uint16_t GMTRX53 = 0x5821U;
    constexpr uint16_t GMTRX54 = 0x5822U;
    constexpr uint16_t GMTRX55 = 0x5823U;
    constexpr uint16_t BRMATRX00 = 0x5824U;
    constexpr uint16_t BRMATRX01 = 0x5825U;
    constexpr uint16_t BRMATRX02 = 0x5826U;
    constexpr uint16_t BRMATRX03 = 0x5827U;
    constexpr uint16_t BRMATRX04 = 0x5828U;
    constexpr uint16_t BRMATRX05 = 0x5829U;
    constexpr uint16_t BRMATRX06 = 0x582AU;
    constexpr uint16_t BRMATRX07 = 0x582BU;
    constexpr uint16_t BRMATRX08 = 0x582CU;
    constexpr uint16_t BRMATRX09 = 0x582DU;
    constexpr uint16_t BRMATRX20 = 0x582EU;
    constexpr uint16_t BRMATRX21 = 0x582FU;
    constexpr uint16_t BRMATRX22 = 0x5830U;
    constexpr uint16_t BRMATRX23 = 0x5831U;
    constexpr uint16_t BRMATRX24 = 0x5832U;
    constexpr uint16_t BRMATRX30 = 0x5833U;
    constexpr uint16_t BRMATRX31 = 0x5834U;
    constexpr uint16_t BRMATRX32 = 0x5835U;
    constexpr uint16_t BRMATRX33 = 0x5836U;
    constexpr uint16_t BRMATRX34 = 0x5837U;
    constexpr uint16_t BRMATRX40 = 0x5838U;
    constexpr uint16_t BRMATRX41 = 0x5839U;
    constexpr uint16_t BRMATRX42 = 0x583AU;
    constexpr uint16_t BRMATRX43 = 0x583BU;
    constexpr uint16_t BRMATRX44 = 0x583CU;
    constexpr uint16_t LENC_BR_OFFSET = 0x583DU;
    constexpr uint16_t MAX_GAIN = 0x583EU;
    constexpr uint16_t MIN_GAIN = 0x583FU;
    constexpr uint16_t MIN_Q = 0x5840U;
    constexpr uint16_t LENC_CTRL59 = 0x5841U;
    constexpr uint16_t BR_HSCALE_HIGH = 0x5842U;
    constexpr uint16_t BR_HSCALE_LOW = 0x5843U;
    constexpr uint16_t BR_VSCALE_HIGH = 0x5844U;
    constexpr uint16_t BR_VSCALE_LOW = 0x5845U;
    constexpr uint16_t G_HSCALE_HIGH = 0x5846U;
    constexpr uint16_t G_HSCALE_LOW = 0x5847U;
    constexpr uint16_t G_VSCALE_HIGH = 0x5848U;
    constexpr uint16_t G_VSCALE_LOW = 0x5849U;

    /* AFC control [= 0x6000 ~ 0x603F]                         */
    constexpr uint16_t AFC_CTRL00 = 0x6000U;
    constexpr uint16_t AFC_CTRL01 = 0x6001U;
    constexpr uint16_t AFC_CTRL02 = 0x6002U;
    constexpr uint16_t AFC_CTRL03 = 0x6003U;
    constexpr uint16_t AFC_CTRL04 = 0x6004U;
    constexpr uint16_t AFC_CTRL05 = 0x6005U;
    constexpr uint16_t AFC_CTRL06 = 0x6006U;
    constexpr uint16_t AFC_CTRL07 = 0x6007U;
    constexpr uint16_t AFC_CTRL08 = 0x6008U;
    constexpr uint16_t AFC_CTRL09 = 0x6009U;
    constexpr uint16_t AFC_CTRL10 = 0x600AU;
    constexpr uint16_t AFC_CTRL11 = 0x600BU;
    constexpr uint16_t AFC_CTRL12 = 0x600CU;
    constexpr uint16_t AFC_CTRL13 = 0x600DU;
    constexpr uint16_t AFC_CTRL14 = 0x600EU;
    constexpr uint16_t AFC_CTRL15 = 0x600FU;
    constexpr uint16_t AFC_CTRL16 = 0x6010U;
    constexpr uint16_t AFC_CTRL17 = 0x6011U;
    constexpr uint16_t AFC_CTRL18 = 0x6012U;
    constexpr uint16_t AFC_CTRL19 = 0x6013U;
    constexpr uint16_t AFC_CTRL20 = 0x6014U;
    constexpr uint16_t AFC_CTRL21 = 0x6015U;
    constexpr uint16_t AFC_CTRL22 = 0x6016U;
    constexpr uint16_t AFC_CTRL23 = 0x6017U;
    constexpr uint16_t AFC_CTRL24 = 0x6018U;
    constexpr uint16_t AFC_CTRL25 = 0x6019U;
    constexpr uint16_t AFC_CTRL26 = 0x601AU;
    constexpr uint16_t AFC_CTRL27 = 0x601BU;
    constexpr uint16_t AFC_CTRL28 = 0x601CU;
    constexpr uint16_t AFC_CTRL29 = 0x601DU;
    constexpr uint16_t AFC_CTRL30 = 0x601EU;
    constexpr uint16_t AFC_CTRL31 = 0x601FU;
    constexpr uint16_t AFC_CTRL32 = 0x6020U;
    constexpr uint16_t AFC_CTRL33 = 0x6021U;
    constexpr uint16_t AFC_CTRL34 = 0x6022U;
    constexpr uint16_t AFC_CTRL35 = 0x6023U;
    constexpr uint16_t AFC_CTRL36 = 0x6024U;
    constexpr uint16_t AFC_CTRL37 = 0x6025U;
    constexpr uint16_t AFC_CTRL38 = 0x6026U;
    constexpr uint16_t AFC_CTRL39 = 0x6027U;
    constexpr uint16_t AFC_CTRL40 = 0x6028U;
    constexpr uint16_t AFC_CTRL41 = 0x6029U;
    constexpr uint16_t AFC_CTRL42 = 0x602AU;
    constexpr uint16_t AFC_CTRL43 = 0x602BU;
    constexpr uint16_t AFC_CTRL44 = 0x602CU;
    constexpr uint16_t AFC_CTRL45 = 0x602DU;
    constexpr uint16_t AFC_CTRL46 = 0x602EU;
    constexpr uint16_t AFC_CTRL47 = 0x602FU;
    constexpr uint16_t AFC_CTRL48 = 0x6030U;
    constexpr uint16_t AFC_CTRL49 = 0x6031U;
    constexpr uint16_t AFC_CTRL50 = 0x6032U;
    constexpr uint16_t AFC_CTRL51 = 0x6033U;
    constexpr uint16_t AFC_CTRL52 = 0x6034U;
    constexpr uint16_t AFC_CTRL53 = 0x6035U;
    constexpr uint16_t AFC_CTRL54 = 0x6036U;
    constexpr uint16_t AFC_CTRL55 = 0x6037U;
    constexpr uint16_t AFC_CTRL56 = 0x6038U;
    constexpr uint16_t AFC_CTRL57 = 0x6039U;
    constexpr uint16_t AFC_CTRL58 = 0x603AU;
    constexpr uint16_t AFC_CTRL59 = 0x603BU;
    constexpr uint16_t AFC_CTRL60 = 0x603CU;
    constexpr uint16_t AFC_READ58 = 0x603DU;
    constexpr uint16_t AFC_READ59 = 0x603EU;
    constexpr uint16_t AFC_READ60 = 0x603FU;
}

/* Initialization sequence for OV5640 */
static constexpr Word_Byte_t OV5640_Common_STM[] =
    {
        {OV5640_REGISTERS_STM::SCCB_SYSTEM_CTRL1, 0x11},
        {OV5640_REGISTERS_STM::SYSTEM_CTROL0, 0x82},
        {OV5640_REGISTERS_STM::SCCB_SYSTEM_CTRL1, 0x03},
        {0x3630, 0x36},
        {0x3631, 0x0e},
        {0x3632, 0xe2},
        {0x3633, 0x12},
        {0x3621, 0xe0},
        {0x3704, 0xa0},
        {0x3703, 0x5a},
        {0x3715, 0x78},
        {0x3717, 0x01},
        {0x370b, 0x60},
        {0x3705, 0x1a},
        {0x3905, 0x02},
        {0x3906, 0x10},
        {0x3901, 0x0a},
        {0x3731, 0x12},
        {0x3600, 0x08},
        {0x3601, 0x33},
        {0x302d, 0x60},
        {0x3620, 0x52},
        {0x371b, 0x20},
        {0x471c, 0x50},
        {OV5640_REGISTERS_STM::AEC_CTRL13, 0x43},
        {OV5640_REGISTERS_STM::AEC_GAIN_CEILING_HIGH, 0x00},
        {OV5640_REGISTERS_STM::AEC_GAIN_CEILING_LOW, 0xf8},
        {0x3635, 0x13},
        {0x3636, 0x03},
        {0x3634, 0x40},
        {0x3622, 0x01},
        {OV5640_REGISTERS_STM::_5060HZ_CTRL01, 0x34},
        {OV5640_REGISTERS_STM::_5060HZ_CTRL04, 0x28},
        {OV5640_REGISTERS_STM::_5060HZ_CTRL05, 0x98},
        {OV5640_REGISTERS_STM::LIGHTMETER1_TH_HIGH, 0x00},
        {OV5640_REGISTERS_STM::LIGHTMETER1_TH_LOW, 0x00},
        {OV5640_REGISTERS_STM::LIGHTMETER2_TH_HIGH, 0x01},
        {OV5640_REGISTERS_STM::LIGHTMETER2_TH_LOW, 0x2c},
        {OV5640_REGISTERS_STM::SAMPLE_NUMBER_HIGH, 0x9c},
        {OV5640_REGISTERS_STM::SAMPLE_NUMBER_LOW, 0x40},
        {OV5640_REGISTERS_STM::TIMING_TC_REG20, 0x06},
        {OV5640_REGISTERS_STM::TIMING_TC_REG21, 0x00},
        {OV5640_REGISTERS_STM::TIMING_X_INC, 0x31},
        {OV5640_REGISTERS_STM::TIMING_Y_INC, 0x31},
        {OV5640_REGISTERS_STM::TIMING_HS_HIGH, 0x00},
        {OV5640_REGISTERS_STM::TIMING_HS_LOW, 0x00},
        {OV5640_REGISTERS_STM::TIMING_VS_HIGH, 0x00},
        {OV5640_REGISTERS_STM::TIMING_VS_LOW, 0x04},
        {OV5640_REGISTERS_STM::TIMING_HW_HIGH, 0x0a},
        {OV5640_REGISTERS_STM::TIMING_HW_LOW, 0x3f},
        {OV5640_REGISTERS_STM::TIMING_VH_HIGH, 0x07},
        {OV5640_REGISTERS_STM::TIMING_VH_LOW, 0x9b},
        {OV5640_REGISTERS_STM::TIMING_DVPHO_HIGH, 0x03},
        {OV5640_REGISTERS_STM::TIMING_DVPHO_LOW, 0x20},
        {OV5640_REGISTERS_STM::TIMING_DVPVO_HIGH, 0x02},
        {OV5640_REGISTERS_STM::TIMING_DVPVO_LOW, 0x58},
        /* For 800x480 resolution: OV5640_REGISTERS_STM::TIMING_HTS=0x790, OV5640_REGISTERS_STM::TIMING_VTS=0x440 */
        {OV5640_REGISTERS_STM::TIMING_HTS_HIGH, 0x07},
        {OV5640_REGISTERS_STM::TIMING_HTS_LOW, 0x90},
        {OV5640_REGISTERS_STM::TIMING_VTS_HIGH, 0x04},
        {OV5640_REGISTERS_STM::TIMING_VTS_LOW, 0x40},
        {OV5640_REGISTERS_STM::TIMING_HOFFSET_HIGH, 0x00},
        {OV5640_REGISTERS_STM::TIMING_HOFFSET_LOW, 0x10},
        {OV5640_REGISTERS_STM::TIMING_VOFFSET_HIGH, 0x00},
        {OV5640_REGISTERS_STM::TIMING_VOFFSET_LOW, 0x06},
        {0x3618, 0x00},
        {0x3612, 0x29},
        {0x3708, 0x64},
        {0x3709, 0x52},
        {0x370c, 0x03},
        {OV5640_REGISTERS_STM::AEC_CTRL02, 0x03},
        {OV5640_REGISTERS_STM::AEC_CTRL03, 0xd8},
        {OV5640_REGISTERS_STM::AEC_B50_STEP_HIGH, 0x01},
        {OV5640_REGISTERS_STM::AEC_B50_STEP_LOW, 0x27},
        {OV5640_REGISTERS_STM::AEC_B60_STEP_HIGH, 0x00},
        {OV5640_REGISTERS_STM::AEC_B60_STEP_LOW, 0xf6},
        {OV5640_REGISTERS_STM::AEC_CTRL0E, 0x03},
        {OV5640_REGISTERS_STM::AEC_CTRL0D, 0x04},
        {OV5640_REGISTERS_STM::AEC_MAX_EXPO_HIGH, 0x03},
        {OV5640_REGISTERS_STM::AEC_MAX_EXPO_LOW, 0xd8},
        {OV5640_REGISTERS_STM::BLC_CTRL01, 0x02},
        {OV5640_REGISTERS_STM::BLC_CTRL04, 0x02},
        {OV5640_REGISTERS_STM::SYSREM_RESET00, 0x00},
        {OV5640_REGISTERS_STM::SYSREM_RESET02, 0x1c},
        {OV5640_REGISTERS_STM::CLOCK_ENABLE00, 0xff},
        {OV5640_REGISTERS_STM::CLOCK_ENABLE02, 0xc3},
        {OV5640_REGISTERS_STM::MIPI_CONTROL00, 0x58},
        {0x302e, 0x00},
        {OV5640_REGISTERS_STM::POLARITY_CTRL, 0x22},
        {OV5640_REGISTERS_STM::FORMAT_CTRL00, 0x6F},
        {OV5640_REGISTERS_STM::FORMAT_MUX_CTRL, 0x01},
        {OV5640_REGISTERS_STM::JPG_MODE_SELECT, 0x03},
        {OV5640_REGISTERS_STM::JPEG_CTRL07, 0x04},
        {0x440e, 0x00},
        {0x460b, 0x35},
        {0x460c, 0x23},
        {OV5640_REGISTERS_STM::PCLK_PERIOD, 0x22},
        {0x3824, 0x02},
        {OV5640_REGISTERS_STM::ISP_CONTROL00, 0xa7},
        {OV5640_REGISTERS_STM::ISP_CONTROL01, 0xa3},
        {OV5640_REGISTERS_STM::AWB_CTRL00, 0xff},
        {OV5640_REGISTERS_STM::AWB_CTRL01, 0xf2},
        {OV5640_REGISTERS_STM::AWB_CTRL02, 0x00},
        {OV5640_REGISTERS_STM::AWB_CTRL03, 0x14},
        {OV5640_REGISTERS_STM::AWB_CTRL04, 0x25},
        {OV5640_REGISTERS_STM::AWB_CTRL05, 0x24},
        {OV5640_REGISTERS_STM::AWB_CTRL06, 0x09},
        {OV5640_REGISTERS_STM::AWB_CTRL07, 0x09},
        {OV5640_REGISTERS_STM::AWB_CTRL08, 0x09},
        {OV5640_REGISTERS_STM::AWB_CTRL09, 0x75},
        {OV5640_REGISTERS_STM::AWB_CTRL10, 0x54},
        {OV5640_REGISTERS_STM::AWB_CTRL11, 0xe0},
        {OV5640_REGISTERS_STM::AWB_CTRL12, 0xb2},
        {OV5640_REGISTERS_STM::AWB_CTRL13, 0x42},
        {OV5640_REGISTERS_STM::AWB_CTRL14, 0x3d},
        {OV5640_REGISTERS_STM::AWB_CTRL15, 0x56},
        {OV5640_REGISTERS_STM::AWB_CTRL16, 0x46},
        {OV5640_REGISTERS_STM::AWB_CTRL17, 0xf8},
        {OV5640_REGISTERS_STM::AWB_CTRL18, 0x04},
        {OV5640_REGISTERS_STM::AWB_CTRL19, 0x70},
        {OV5640_REGISTERS_STM::AWB_CTRL20, 0xf0},
        {OV5640_REGISTERS_STM::AWB_CTRL21, 0xf0},
        {OV5640_REGISTERS_STM::AWB_CTRL22, 0x03},
        {OV5640_REGISTERS_STM::AWB_CTRL23, 0x01},
        {OV5640_REGISTERS_STM::AWB_CTRL24, 0x04},
        {OV5640_REGISTERS_STM::AWB_CTRL25, 0x12},
        {OV5640_REGISTERS_STM::AWB_CTRL26, 0x04},
        {OV5640_REGISTERS_STM::AWB_CTRL27, 0x00},
        {OV5640_REGISTERS_STM::AWB_CTRL28, 0x06},
        {OV5640_REGISTERS_STM::AWB_CTRL29, 0x82},
        {OV5640_REGISTERS_STM::AWB_CTRL30, 0x38},
        {OV5640_REGISTERS_STM::CMX1, 0x1e},
        {OV5640_REGISTERS_STM::CMX2, 0x5b},
        {OV5640_REGISTERS_STM::CMX3, 0x08},
        {OV5640_REGISTERS_STM::CMX4, 0x0a},
        {OV5640_REGISTERS_STM::CMX5, 0x7e},
        {OV5640_REGISTERS_STM::CMX6, 0x88},
        {OV5640_REGISTERS_STM::CMX7, 0x7c},
        {OV5640_REGISTERS_STM::CMX8, 0x6c},
        {OV5640_REGISTERS_STM::CMX9, 0x10},
        {OV5640_REGISTERS_STM::CMXSIGN_HIGH, 0x01},
        {OV5640_REGISTERS_STM::CMXSIGN_LOW, 0x98},
        {OV5640_REGISTERS_STM::CIP_SHARPENMT_TH1, 0x08},
        {OV5640_REGISTERS_STM::CIP_SHARPENMT_TH2, 0x30},
        {OV5640_REGISTERS_STM::CIP_SHARPENMT_OFFSET1, 0x10},
        {OV5640_REGISTERS_STM::CIP_SHARPENMT_OFFSET2, 0x00},
        {OV5640_REGISTERS_STM::CIP_DNS_TH1, 0x08},
        {OV5640_REGISTERS_STM::CIP_DNS_TH2, 0x30},
        {OV5640_REGISTERS_STM::CIP_DNS_OFFSET1, 0x08},
        {OV5640_REGISTERS_STM::CIP_DNS_OFFSET2, 0x16},
        {OV5640_REGISTERS_STM::CIP_CTRL, 0x08},
        {OV5640_REGISTERS_STM::CIP_SHARPENTH_TH1, 0x30},
        {OV5640_REGISTERS_STM::CIP_SHARPENTH_TH2, 0x04},
        {OV5640_REGISTERS_STM::CIP_SHARPENTH_OFFSET1, 0x06},
        {OV5640_REGISTERS_STM::GAMMA_CTRL00, 0x01},
        {OV5640_REGISTERS_STM::GAMMA_YST00, 0x08},
        {OV5640_REGISTERS_STM::GAMMA_YST01, 0x14},
        {OV5640_REGISTERS_STM::GAMMA_YST02, 0x28},
        {OV5640_REGISTERS_STM::GAMMA_YST03, 0x51},
        {OV5640_REGISTERS_STM::GAMMA_YST04, 0x65},
        {OV5640_REGISTERS_STM::GAMMA_YST05, 0x71},
        {OV5640_REGISTERS_STM::GAMMA_YST06, 0x7d},
        {OV5640_REGISTERS_STM::GAMMA_YST07, 0x87},
        {OV5640_REGISTERS_STM::GAMMA_YST08, 0x91},
        {OV5640_REGISTERS_STM::GAMMA_YST09, 0x9a},
        {OV5640_REGISTERS_STM::GAMMA_YST0A, 0xaa},
        {OV5640_REGISTERS_STM::GAMMA_YST0B, 0xb8},
        {OV5640_REGISTERS_STM::GAMMA_YST0C, 0xcd},
        {OV5640_REGISTERS_STM::GAMMA_YST0D, 0xdd},
        {OV5640_REGISTERS_STM::GAMMA_YST0E, 0xea},
        {OV5640_REGISTERS_STM::GAMMA_YST0F, 0x1d},
        {OV5640_REGISTERS_STM::SDE_CTRL0, 0x02},
        {OV5640_REGISTERS_STM::SDE_CTRL3, 0x40},
        {OV5640_REGISTERS_STM::SDE_CTRL4, 0x10},
        {OV5640_REGISTERS_STM::SDE_CTRL9, 0x10},
        {OV5640_REGISTERS_STM::SDE_CTRL10, 0x00},
        {OV5640_REGISTERS_STM::SDE_CTRL11, 0xf8},
        {OV5640_REGISTERS_STM::GMTRX00, 0x23},
        {OV5640_REGISTERS_STM::GMTRX01, 0x14},
        {OV5640_REGISTERS_STM::GMTRX02, 0x0f},
        {OV5640_REGISTERS_STM::GMTRX03, 0x0f},
        {OV5640_REGISTERS_STM::GMTRX04, 0x12},
        {OV5640_REGISTERS_STM::GMTRX05, 0x26},
        {OV5640_REGISTERS_STM::GMTRX10, 0x0c},
        {OV5640_REGISTERS_STM::GMTRX11, 0x08},
        {OV5640_REGISTERS_STM::GMTRX12, 0x05},
        {OV5640_REGISTERS_STM::GMTRX13, 0x05},
        {OV5640_REGISTERS_STM::GMTRX14, 0x08},
        {OV5640_REGISTERS_STM::GMTRX15, 0x0d},
        {OV5640_REGISTERS_STM::GMTRX20, 0x08},
        {OV5640_REGISTERS_STM::GMTRX21, 0x03},
        {OV5640_REGISTERS_STM::GMTRX22, 0x00},
        {OV5640_REGISTERS_STM::GMTRX23, 0x00},
        {OV5640_REGISTERS_STM::GMTRX24, 0x03},
        {OV5640_REGISTERS_STM::GMTRX25, 0x09},
        {OV5640_REGISTERS_STM::GMTRX30, 0x07},
        {OV5640_REGISTERS_STM::GMTRX31, 0x03},
        {OV5640_REGISTERS_STM::GMTRX32, 0x00},
        {OV5640_REGISTERS_STM::GMTRX33, 0x01},
        {OV5640_REGISTERS_STM::GMTRX34, 0x03},
        {OV5640_REGISTERS_STM::GMTRX35, 0x08},
        {OV5640_REGISTERS_STM::GMTRX40, 0x0d},
        {OV5640_REGISTERS_STM::GMTRX41, 0x08},
        {OV5640_REGISTERS_STM::GMTRX42, 0x05},
        {OV5640_REGISTERS_STM::GMTRX43, 0x06},
        {OV5640_REGISTERS_STM::GMTRX44, 0x08},
        {OV5640_REGISTERS_STM::GMTRX45, 0x0e},
        {OV5640_REGISTERS_STM::GMTRX50, 0x29},
        {OV5640_REGISTERS_STM::GMTRX51, 0x17},
        {OV5640_REGISTERS_STM::GMTRX52, 0x11},
        {OV5640_REGISTERS_STM::GMTRX53, 0x11},
        {OV5640_REGISTERS_STM::GMTRX54, 0x15},
        {OV5640_REGISTERS_STM::GMTRX55, 0x28},
        {OV5640_REGISTERS_STM::BRMATRX00, 0x46},
        {OV5640_REGISTERS_STM::BRMATRX01, 0x26},
        {OV5640_REGISTERS_STM::BRMATRX02, 0x08},
        {OV5640_REGISTERS_STM::BRMATRX03, 0x26},
        {OV5640_REGISTERS_STM::BRMATRX04, 0x64},
        {OV5640_REGISTERS_STM::BRMATRX05, 0x26},
        {OV5640_REGISTERS_STM::BRMATRX06, 0x24},
        {OV5640_REGISTERS_STM::BRMATRX07, 0x22},
        {OV5640_REGISTERS_STM::BRMATRX08, 0x24},
        {OV5640_REGISTERS_STM::BRMATRX09, 0x24},
        {OV5640_REGISTERS_STM::BRMATRX20, 0x06},
        {OV5640_REGISTERS_STM::BRMATRX21, 0x22},
        {OV5640_REGISTERS_STM::BRMATRX22, 0x40},
        {OV5640_REGISTERS_STM::BRMATRX23, 0x42},
        {OV5640_REGISTERS_STM::BRMATRX24, 0x24},
        {OV5640_REGISTERS_STM::BRMATRX30, 0x26},
        {OV5640_REGISTERS_STM::BRMATRX31, 0x24},
        {OV5640_REGISTERS_STM::BRMATRX32, 0x22},
        {OV5640_REGISTERS_STM::BRMATRX33, 0x22},
        {OV5640_REGISTERS_STM::BRMATRX34, 0x26},
        {OV5640_REGISTERS_STM::BRMATRX40, 0x44},
        {OV5640_REGISTERS_STM::BRMATRX41, 0x24},
        {OV5640_REGISTERS_STM::BRMATRX42, 0x26},
        {OV5640_REGISTERS_STM::BRMATRX43, 0x28},
        {OV5640_REGISTERS_STM::BRMATRX44, 0x42},
        {OV5640_REGISTERS_STM::LENC_BR_OFFSET, 0xce},
        {0x5025, 0x00},
        {OV5640_REGISTERS_STM::AEC_CTRL0F, 0x30},
        {OV5640_REGISTERS_STM::AEC_CTRL10, 0x28},
        {OV5640_REGISTERS_STM::AEC_CTRL1B, 0x30},
        {OV5640_REGISTERS_STM::AEC_CTRL1E, 0x26},
        {OV5640_REGISTERS_STM::AEC_CTRL11, 0x60},
        {OV5640_REGISTERS_STM::AEC_CTRL1F, 0x14},
        {OV5640_REGISTERS_STM::SYSTEM_CTROL0, 0x02},
};

//https://github.com/Digilent/Zybo-Z7-SW/blob/5d358aa3469869382f9fb515b65af3469775e3c8/src/Zybo-Z7-20-pcam-5c/src/ov5640/OV5640.h
static constexpr Word_Byte_t cfg_init_[] =
	{
		//[7]=0 Software reset; [6]=1 Software power down; Default=0x02
		{0x3008, 0x42},
		//[1]=1 System input clock from PLL; Default read = 0x11
		{0x3103, 0x03},
		//[3:0]=0000 MD2P,MD2N,MCP,MCN input; Default=0x00
		{0x3017, 0x00},
		//[7:2]=000000 MD1P,MD1N, D3:0 input; Default=0x00
		{0x3018, 0x00},
		//[6:4]=001 PLL charge pump, [3:0]=1000 MIPI 8-bit mode
		{0x3034, 0x18},

		//              +----------------+        +------------------+         +---------------------+        +---------------------+
		//XVCLK         | PRE_DIV0       |        | Mult (4+252)     |         | Sys divider (0=16)  |        | MIPI divider (0=16) |
		//+-------+-----> 3037[3:0]=0001 +--------> 3036[7:0]=0x38   +---------> 3035[7:4]=0001      +--------> 3035[3:0]=0001      |
		//12MHz   |     | / 1            | 12MHz  | * 56             | 672MHz  | / 1                 | 672MHz | / 1                 |
		//        |     +----------------+        +------------------+         +----------+----------+        +----------+----------+
		//        |                                                                       |                              |
		//        |                                                                       |                      MIPISCLK|672MHz
		//        |                                                                       |                              |
		//        |     +----------------+        +------------------+         +----------v----------+        +----------v----------+
		//        |     | PRE_DIVSP      |        | R_DIV_SP         |         | PLL R divider       |        | MIPI PHY            | MIPI_CLK
		//        +-----> 303d[5:4]=01   +--------> 303d[2]=0 (+1)   |         | 3037[4]=1 (+1)      |        |                     +------->
		//              | / 1.5          |  8MHz  | / 1              |         | / 2                 |        | / 2                 | 336MHz
		//              +----------------+        +---------+--------+         +----------+----------+        +---------------------+
		//                                                  |                             |
		//                                                  |                             |
		//                                                  |                             |
		//              +----------------+        +---------v--------+         +----------v----------+        +---------------------+
		//              | SP divider     |        | Mult             |         | BIT div (MIPI 8/10) |        | SCLK divider        | SCLK
		//              | 303c[3:0]=0x1  +<-------+ 303b[4:0]=0x19   |         | 3034[3:0]=0x8)      +----+---> 3108[1:0]=01 (2^)   +------->
		//              | / 1            | 200MHz | * 25             |         | / 2                 |    |   | / 2                 | 84MHz
		//              +--------+-------+        +------------------+         +----------+----------+    |   +---------------------+
		//                       |                                                        |               |
		//                       |                                                        |               |
		//                       |                                                        |               |
		//              +--------v-------+                                     +----------v----------+    |   +---------------------+
		//              | R_SELD5 div    | ADCCLK                              | PCLK div            |    |   | SCLK2x divider      |
		//              | 303d[1:0]=001  +------->                             | 3108[5:4]=00 (2^)   |    +---> 3108[3:2]=00 (2^)   +------->
		//              | / 1            | 200MHz                              | / 1                 |        | / 1                 | 168MHz
		//              +----------------+                                     +----------+----------+        +---------------------+
		//                                                                                |
		//                                                                                |
		//                                                                                |
		//                                                                     +----------v----------+        +---------------------+
		//                                                                     | P divider (* #lanes)| PCLK   | Scale divider       |
		//                                                                     | 3035[3:0]=0001      +--------> 3824[4:0]           |
		//                                                                     | / 1                 | 168MHz | / 2                 |
		//                                                                     +---------------------+        +---------------------+

		//PLL1 configuration
		//[7:4]=0001 System clock divider /1, [3:0]=0001 Scale divider for MIPI /1
		{0x3035, 0x11},
		//[7:0]=56 PLL multiplier
		{0x3036, 0x38},
		//[4]=1 PLL root divider /2, [3:0]=1 PLL pre-divider /1
		{0x3037, 0x11},
		//[5:4]=00 PCLK root divider /1, [3:2]=00 SCLK2x root divider /1, [1:0]=01 SCLK root divider /2
		{0x3108, 0x01},
		//PLL2 configuration
		//[5:4]=01 PRE_DIV_SP /1.5, [2]=1 R_DIV_SP /1, [1:0]=00 DIV12_SP /1
		{0x303D, 0x10},
		//[4:0]=11001 PLL2 multiplier DIV_CNT5B = 25
		{0x303B, 0x19},

		{0x3630, 0x2e},
		{0x3631, 0x0e},
		{0x3632, 0xe2},
		{0x3633, 0x23},
		{0x3621, 0xe0},
		{0x3704, 0xa0},
		{0x3703, 0x5a},
		{0x3715, 0x78},
		{0x3717, 0x01},
		{0x370b, 0x60},
		{0x3705, 0x1a},
		{0x3905, 0x02},
		{0x3906, 0x10},
		{0x3901, 0x0a},
		{0x3731, 0x02},
		//VCM debug mode
		{0x3600, 0x37},
		{0x3601, 0x33},
		//System control register changing not recommended
		{0x302d, 0x60},
		//??
		{0x3620, 0x52},
		{0x371b, 0x20},
		//?? DVP
		{0x471c, 0x50},

		{0x3a13, 0x43},
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
		{0x3635, 0x13},
		{0x3636, 0x06},
		{0x3634, 0x44},
		{0x3622, 0x01},
		{0x3c01, 0x34},
		{0x3c04, 0x28},
		{0x3c05, 0x98},
		{0x3c06, 0x00},
		{0x3c07, 0x08},
		{0x3c08, 0x00},
		{0x3c09, 0x1c},
		{0x3c0a, 0x9c},
		{0x3c0b, 0x40},

		//[7]=1 color bar enable, [3:2]=00 eight color bar
		{0x503d, 0x00},
		//[2]=1 ISP vflip, [1]=1 sensor vflip
		{0x3820, 0x46},

		//[7:5]=010 Two lane mode, [4]=0 MIPI HS TX no power down, [3]=0 MIPI LP RX no power down, [2]=1 MIPI enable, [1:0]=10 Debug mode; Default=0x58
		{0x300e, 0x45},
		//[5]=0 Clock free running, [4]=1 Send line short packet, [3]=0 Use lane1 as default, [2]=1 MIPI bus LP11 when no packet; Default=0x04
		{0x4800, 0x14},
		{0x302e, 0x08},
		//[7:4]=0x3 YUV422, [3:0]=0x0 YUYV
		//{0x4300, 0x30},
		//[7:4]=0x6 RGB565, [3:0]=0x0 {b[4:0],g[5:3],g[2:0],r[4:0]}
		{0x4300, 0x6f},
		{0x501f, 0x01},

		{0x4713, 0x03},
		{0x4407, 0x04},
		{0x440e, 0x00},
		{0x460b, 0x35},
		//[1]=0 DVP PCLK divider manual control by 0x3824[4:0]
		{0x460c, 0x20},
		//[4:0]=1 SCALE_DIV=INT(3824[4:0]/2)
		{0x3824, 0x01},

		//MIPI timing
		//		{0x4805, 0x10}, //LPX global timing select=auto
		//		{0x4818, 0x00}, //hs_prepare + hs_zero_min ns
		//		{0x4819, 0x96},
		//		{0x482A, 0x00}, //hs_prepare + hs_zero_min UI
		//
		//		{0x4824, 0x00}, //lpx_p_min ns
		//		{0x4825, 0x32},
		//		{0x4830, 0x00}, //lpx_p_min UI
		//
		//		{0x4826, 0x00}, //hs_prepare_min ns
		//		{0x4827, 0x32},
		//		{0x4831, 0x00}, //hs_prepare_min UI

		//[7]=1 LENC correction enabled, [5]=1 RAW gamma enabled, [2]=1 Black pixel cancellation enabled, [1]=1 White pixel cancellation enabled, [0]=1 Color interpolation enabled
		{0x5000, 0x07},
		//[7]=0 Special digital effects, [5]=0 scaling, [2]=0 UV average disabled, [1]=1 Color matrix enabled, [0]=1 Auto white balance enabled
		{0x5001, 0x03}
	};

#endif /* OV5650_INITIALIATION_HPP_ */
