// MLX90640Calibration.h

#pragma once

#include <cstdint>
#include <array>
#include "MLX90640EEPROM.h"

struct MLX90640_Calibration
{
    int16_t kVdd;
    int16_t vdd25;

    float KvPTAT;
    float KtPTAT;
    uint16_t vPTAT25;
    float alphaPTAT;

    int16_t gainEE;

    float tgc;
    float cpKv;
    float cpKta;

    uint8_t resolutionEE;
    uint8_t calibrationModeEE;

    float KsTa;
    float ksTo[5];
    int16_t ct[5];

    uint16_t alpha[768];
    uint8_t alphaScale;

    int16_t offset[768];

    int8_t kta[768];
    uint8_t ktaScale;

    int8_t kv[768];
    uint8_t kvScale;

    float cpAlpha[2];
    int16_t cpOffset[2];

    float ilChessC[3];

    uint16_t brokenPixels[5];
    uint16_t outlierPixels[5];
};

constexpr uint8_t msb(uint16_t w) noexcept { return uint8_t(w >> 8); }
constexpr uint8_t lsb(uint16_t w) noexcept { return uint8_t(w & 0xFF); }

constexpr int16_t s16(uint16_t w) noexcept { return static_cast<int16_t>(w); }

constexpr uint32_t pow2u(int n) noexcept
{
    return (n >= 0 && n < 32) ? (1u << n) : 0u;
}

constexpr int32_t pow2i(int n) noexcept
{
    int32_t r = 1;
    if (n > 0)
    {
        for (int i = 0; i < n; ++i)
        {
            r *= 2;
        }
    }
    return r;
}

constexpr float pow2f(int n) noexcept
{
    float r = 1.0f;

    if (n > 0)
    {
        for (int i = 0; i < n; ++i)
        {
            r *= 2.0f;
        }
    }
    else if (n < 0)
    {
        for (int i = 0; i > n; --i)
        {
            r *= 0.5f;
        }
    }

    return r;
}

template <const uint16_t *EE>
constexpr MLX90640_Calibration parse_eeprom_phase1()
{
    MLX90640_Calibration c{};

    // -----------------------------
    // VDD parameters
    // -----------------------------
    {
        const int8_t kVdd_raw = int8_t(msb(EE[51]));
        int16_t vdd25_raw = int16_t(lsb(EE[51]));

        vdd25_raw = int16_t(((vdd25_raw - 256) << 5) - 8192);

        c.kVdd = int16_t(32 * kVdd_raw);
        c.vdd25 = vdd25_raw;
    }

    // -----------------------------
    // PTAT parameters
    // -----------------------------
    {
        // KvPTAT: bits 15..10 (6 bits signed)
        int KvPTAT = int((EE[50] >> 10) & 0x3F);
        if (KvPTAT > 31)
            KvPTAT -= 64;
        c.KvPTAT = float(KvPTAT) / 4096.0f;

        // KtPTAT: bits 9..0 (10 bits signed)
        int KtPTAT = int(EE[50] & 0x03FF);
        if (KtPTAT > 511)
            KtPTAT -= 1024;
        c.KtPTAT = float(KtPTAT) / 8.0f;

        c.vPTAT25 = EE[49];

        // alphaPTAT: nibble 3 of EE[16], scaled
        const uint8_t nibble = uint8_t((EE[16] >> 12) & 0xF);
        c.alphaPTAT = float(nibble) / pow2f(14) + 8.0f;
    }

    // -----------------------------
    // Gain
    // -----------------------------
    c.gainEE = s16(EE[48]);

    // -----------------------------
    // TGC
    // -----------------------------
    c.tgc = float(int8_t(lsb(EE[60]))) / 32.0f;

    // -----------------------------
    // Resolution
    // -----------------------------
    c.resolutionEE = uint8_t((EE[56] >> 12) & 0x3);

    // -----------------------------
    // KsTa
    // -----------------------------
    c.KsTa = float(int8_t(msb(EE[60]))) / 8192.0f;

    // -----------------------------
    // KsTo parameters
    // -----------------------------
    {
        const int step = int((EE[63] >> 12) & 0x3) * 10;

        c.ct[0] = -40;
        c.ct[1] = 0;
        c.ct[2] = int16_t((EE[63] >> 4) & 0xFF) * step;
        c.ct[3] = int16_t(c.ct[2] + (EE[63] & 0xF) * step);
        c.ct[4] = 400;

        const int KsToScale = int((EE[63] >> 8) & 0xF) + 8;
        const float scale = float(1u << KsToScale);

        c.ksTo[0] = float(int8_t(lsb(EE[61]))) / scale;
        c.ksTo[1] = float(int8_t(msb(EE[61]))) / scale;
        c.ksTo[2] = float(int8_t(lsb(EE[62]))) / scale;
        c.ksTo[3] = float(int8_t(msb(EE[62]))) / scale;
        c.ksTo[4] = -0.0002f;
    }

    // -----------------------------
    // CP parameters
    // -----------------------------
    {
        // cpAlpha[0], cpAlpha[1]
        c.cpAlpha[0] = float((EE[52] & 0x03F0) >> 4) / pow2f(EE[32] >> 12);
        c.cpAlpha[1] = float((EE[53] & 0x03F0) >> 4) / pow2f(EE[32] >> 12);

        // cpOffset[0], cpOffset[1]
        c.cpOffset[0] = s16(EE[58]);
        c.cpOffset[1] = s16(EE[59]);

        // cpKta, cpKv
        c.cpKta = float(int8_t(lsb(EE[54]))) / pow2f((EE[32] >> 4) & 0xF);
        c.cpKv = float(int8_t(msb(EE[54]))) / pow2f((EE[32] >> 8) & 0xF);
    }

    // -----------------------------
    // Calibration mode EE
    // -----------------------------
    c.calibrationModeEE = uint8_t((EE[56] >> 8) & 0x1);

    return c;
}

constexpr uint8_t nibble1(uint16_t w) noexcept { return uint8_t((w >> 0) & 0xF); }
constexpr uint8_t nibble2(uint16_t w) noexcept { return uint8_t((w >> 4) & 0xF); }
constexpr uint8_t nibble3(uint16_t w) noexcept { return uint8_t((w >> 8) & 0xF); }
constexpr uint8_t nibble4(uint16_t w) noexcept { return uint8_t((w >> 12) & 0xF); }

constexpr float absf(float x) noexcept
{
    return x < 0.0f ? -x : x;
}

constexpr float SCALEALPHA = 0.000001f;

// ---- keep your struct MLX90640_Calibration as-is ----
// ---- keep msb/lsb/s16/pow2f/nibble1..4/absf/SCALEALPHA as you have them ----

// ------------------------------------------------------------
// VDD parameters (ExtractVDDParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_vdd(MLX90640_Calibration& c)
{
    int8_t kVdd   = int8_t(msb(EE[51]));
    int16_t vdd25 = int16_t(lsb(EE[51]));
    vdd25 = int16_t(((vdd25 - 256) << 5) - 8192);

    c.kVdd  = int16_t(32 * kVdd);
    c.vdd25 = vdd25;
}

// ------------------------------------------------------------
// PTAT parameters (ExtractPTATParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_ptat(MLX90640_Calibration& c)
{
    float KvPTAT = float((EE[50] & 0xFC00u) >> 10);   // MSBITS_6
    if (KvPTAT > 31.0f) KvPTAT -= 64.0f;
    KvPTAT /= 4096.0f;

    float KtPTAT = float(EE[50] & 0x03FFu);          // LSBITS_10
    if (KtPTAT > 511.0f) KtPTAT -= 1024.0f;
    KtPTAT /= 8.0f;

    int16_t vPTAT25 = int16_t(EE[49]);

    // NOTE: Melexis uses full nibble field (bits 15..12) shifted and divided by POW2(14)
    // (eeData[16] & NIBBLE4_MASK) / POW2(14) == (nibble4 << 12) / 2^14 = nibble4 / 4
    float alphaPTAT = float(EE[16] & 0xF000u) / pow2f(14) + 8.0f;

    c.KvPTAT   = KvPTAT;
    c.KtPTAT   = KtPTAT;
    c.vPTAT25  = uint16_t(vPTAT25);
    c.alphaPTAT = alphaPTAT;
}

// ------------------------------------------------------------
// Gain parameters (ExtractGainParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_gain(MLX90640_Calibration& c)
{
    c.gainEE = s16(EE[48]);
}

// ------------------------------------------------------------
// TGC (ExtractTgcParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_tgc(MLX90640_Calibration& c)
{
    c.tgc = float(int8_t(lsb(EE[60]))) / 32.0f;
}

// ------------------------------------------------------------
// Resolution (ExtractResolutionParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_resolution(MLX90640_Calibration& c)
{
    uint8_t resolutionEE = uint8_t((EE[56] & 0x3000u) >> 12);
    c.resolutionEE = resolutionEE;
}

// ------------------------------------------------------------
// KsTa (ExtractKsTaParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_ksta(MLX90640_Calibration& c)
{
    c.KsTa = float(int8_t(msb(EE[60]))) / 8192.0f;
}

// ------------------------------------------------------------
// KsTo + CT (ExtractKsToParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_ksto(MLX90640_Calibration& c)
{
    int8_t step = int8_t(((EE[63] & 0x3000u) >> 12) * 10);

    c.ct[0] = -40;
    c.ct[1] = 0;
    c.ct[2] = int16_t(nibble2(EE[63]));
    c.ct[3] = int16_t(nibble3(EE[63]));

    c.ct[2] = int16_t(c.ct[2] * step);
    c.ct[3] = int16_t(c.ct[2] + c.ct[3] * step);
    c.ct[4] = 400;

    int32_t KsToScale = int32_t(nibble1(EE[63])) + 8;
    const float scale = float(1u << KsToScale);

    c.ksTo[0] = float(int8_t(lsb(EE[61]))) / scale;
    c.ksTo[1] = float(int8_t(msb(EE[61]))) / scale;
    c.ksTo[2] = float(int8_t(lsb(EE[62]))) / scale;
    c.ksTo[3] = float(int8_t(msb(EE[62]))) / scale;
    c.ksTo[4] = -0.0002f;
}

// ------------------------------------------------------------
// CP parameters (ExtractCPParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_cp(MLX90640_Calibration& c)
{
    float alphaSP[2];
    int16_t offsetSP[2];

    uint8_t alphaScale = uint8_t(nibble4(EE[32]) + 27);

    offsetSP[0] = int16_t(EE[58] & 0x03FFu);
    if (offsetSP[0] > 511) offsetSP[0] = int16_t(offsetSP[0] - 1024);

    offsetSP[1] = int16_t((EE[58] & 0xFC00u) >> 10);
    if (offsetSP[1] > 31) offsetSP[1] = int16_t(offsetSP[1] - 64);
    offsetSP[1] = int16_t(offsetSP[1] + offsetSP[0]);

    alphaSP[0] = float(EE[57] & 0x03FFu);
    if (alphaSP[0] > 511.0f) alphaSP[0] -= 1024.0f;
    alphaSP[0] /= pow2f(alphaScale);

    alphaSP[1] = float((EE[57] & 0xFC00u) >> 10);
    if (alphaSP[1] > 31.0f) alphaSP[1] -= 64.0f;
    alphaSP[1] = (1.0f + alphaSP[1] / 128.0f) * alphaSP[0];

    float cpKta = float(int8_t(lsb(EE[59])));
    uint8_t ktaScale1 = uint8_t(nibble2(EE[56]) + 8);
    c.cpKta = cpKta / pow2f(ktaScale1);

    float cpKv = float(int8_t(msb(EE[59])));
    uint8_t kvScale = uint8_t(nibble3(EE[56]));
    c.cpKv = cpKv / pow2f(kvScale);

    c.cpAlpha[0] = alphaSP[0];
    c.cpAlpha[1] = alphaSP[1];
    c.cpOffset[0] = offsetSP[0];
    c.cpOffset[1] = offsetSP[1];
}

// ------------------------------------------------------------
// Alpha parameters (ExtractAlphaParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_alpha(MLX90640_Calibration& c)
{
    int accRow[24];
    int accColumn[32];
    float alphaTemp[768];

    uint8_t accRemScale    = nibble1(EE[32]);
    uint8_t accColumnScale = nibble2(EE[32]);
    uint8_t accRowScale    = nibble3(EE[32]);
    uint8_t baseAlphaScale = uint8_t(nibble4(EE[32]) + 30);
    int alphaRef           = int(EE[33]);

    // Row accumulators
    for (int i = 0; i < 6; ++i)
    {
        const uint16_t w = EE[34 + i];
        const int p = i * 4;
        accRow[p + 0] = int(nibble1(w));
        accRow[p + 1] = int(nibble2(w));
        accRow[p + 2] = int(nibble3(w));
        accRow[p + 3] = int(nibble4(w));
    }
    for (int i = 0; i < 24; ++i)
    {
        if (accRow[i] > 7) accRow[i] -= 16;
    }

    // Column accumulators
    for (int i = 0; i < 8; ++i)
    {
        const uint16_t w = EE[40 + i];
        const int p = i * 4;
        accColumn[p + 0] = int(nibble1(w));
        accColumn[p + 1] = int(nibble2(w));
        accColumn[p + 2] = int(nibble3(w));
        accColumn[p + 3] = int(nibble4(w));
    }
    for (int i = 0; i < 32; ++i)
    {
        if (accColumn[i] > 7) accColumn[i] -= 16;
    }

    // Per-pixel alphaTemp
    for (int i = 0; i < 24; ++i)
    {
        for (int j = 0; j < 32; ++j)
        {
            const int p = 32 * i + j;
            const uint16_t ee = EE[64 + p];

            float at = float((ee & 0x03F0u) >> 4);
            if (at > 31.0f) at -= 64.0f;
            at *= float(1 << accRemScale);

            at = float(alphaRef)
                + float(accRow[i]   << accRowScale)
                + float(accColumn[j] << accColumnScale)
                + at;

            at /= pow2f(baseAlphaScale);
            at -= c.tgc * (c.cpAlpha[0] + c.cpAlpha[1]) / 2.0f;
            at = SCALEALPHA / at;

            alphaTemp[p] = at;
        }
    }

    // Find max
    float temp = alphaTemp[0];
    for (int i = 1; i < 768; ++i)
    {
        if (alphaTemp[i] > temp) temp = alphaTemp[i];
    }

    uint8_t alphaScale = 0;
    while (temp < 32767.4f)
    {
        temp *= 2.0f;
        alphaScale++;
    }

    // Quantize
    for (int i = 0; i < 768; ++i)
    {
        float v = alphaTemp[i] * pow2f(alphaScale);
        c.alpha[i] = uint16_t(v + 0.5f);
    }

    c.alphaScale = alphaScale;
}

// ------------------------------------------------------------
// Offset parameters (ExtractOffsetParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_offset(MLX90640_Calibration& c)
{
    int occRow[24];
    int occColumn[32];

    uint8_t occRemScale    = nibble1(EE[16]);
    uint8_t occColumnScale = nibble2(EE[16]);
    uint8_t occRowScale    = nibble3(EE[16]);
    int16_t offsetRef      = s16(EE[17]);

    for (int i = 0; i < 6; ++i)
    {
        const uint16_t w = EE[18 + i];
        const int p = i * 4;
        occRow[p + 0] = int(nibble1(w));
        occRow[p + 1] = int(nibble2(w));
        occRow[p + 2] = int(nibble3(w));
        occRow[p + 3] = int(nibble4(w));
    }
    for (int i = 0; i < 24; ++i)
    {
        if (occRow[i] > 7) occRow[i] -= 16;
    }

    for (int i = 0; i < 8; ++i)
    {
        const uint16_t w = EE[24 + i];
        const int p = i * 4;
        occColumn[p + 0] = int(nibble1(w));
        occColumn[p + 1] = int(nibble2(w));
        occColumn[p + 2] = int(nibble3(w));
        occColumn[p + 3] = int(nibble4(w));
    }
    for (int i = 0; i < 32; ++i)
    {
        if (occColumn[i] > 7) occColumn[i] -= 16;
    }

    for (int i = 0; i < 24; ++i)
    {
        for (int j = 0; j < 32; ++j)
        {
            const int p = 32 * i + j;
            const uint16_t ee = EE[64 + p];

            int16_t offset = int16_t((ee & 0xFC00u) >> 10);
            if (offset > 31) offset = int16_t(offset - 64);

            offset = int16_t(offset * (1 << occRemScale));
            offset = int16_t(offsetRef
                             + (occRow[i]    << occRowScale)
                             + (occColumn[j] << occColumnScale)
                             + offset);
            c.offset[p] = offset;
        }
    }
}

// ------------------------------------------------------------
// Kta pixel parameters (ExtractKtaPixelParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_kta(MLX90640_Calibration& c)
{
    float ktaTemp[768];
    int8_t KtaRC[4];

    KtaRC[0] = int8_t(msb(EE[54]));
    KtaRC[2] = int8_t(lsb(EE[54]));
    KtaRC[1] = int8_t(msb(EE[55]));
    KtaRC[3] = int8_t(lsb(EE[55]));

    uint8_t ktaScale1 = uint8_t(nibble2(EE[56]) + 8);
    uint8_t ktaScale2 = nibble1(EE[56]);

    for (int i = 0; i < 24; ++i)
    {
        for (int j = 0; j < 32; ++j)
        {
            const int p = 32 * i + j;
            const uint16_t ee = EE[64 + p];

            uint8_t split = uint8_t(2 * (p / 32 - (p / 64) * 2) + (p & 1));

            float kt = float((ee & 0x000Eu) >> 1);
            if (kt > 3.0f) kt -= 8.0f;
            kt *= float(1 << ktaScale2);
            kt += float(KtaRC[split]);
            kt /= pow2f(ktaScale1);

            ktaTemp[p] = kt;
        }
    }

    float temp = absf(ktaTemp[0]);
    for (int p = 1; p < 768; ++p)
    {
        float v = absf(ktaTemp[p]);
        if (v > temp) temp = v;
    }

    ktaScale1 = 0;
    while (temp < 63.4f)
    {
        temp *= 2.0f;
        ktaScale1++;
    }

    for (int p = 0; p < 768; ++p)
    {
        float v = ktaTemp[p] * pow2f(ktaScale1);
        if (v < 0.0f) v -= 0.5f;
        else          v += 0.5f;
        c.kta[p] = int8_t(v);
    }

    c.ktaScale = ktaScale1;
}

// ------------------------------------------------------------
// Kv pixel parameters (ExtractKvPixelParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_kv(MLX90640_Calibration& c)
{
    float kvTemp[768];
    int8_t KvT[4];

    int8_t KvRoCo = int8_t(nibble4(EE[52]));
    if (KvRoCo > 7) KvRoCo = int8_t(KvRoCo - 16);
    KvT[0] = KvRoCo;

    int8_t KvReCo = int8_t(nibble3(EE[52]));
    if (KvReCo > 7) KvReCo = int8_t(KvReCo - 16);
    KvT[2] = KvReCo;

    int8_t KvRoCe = int8_t(nibble2(EE[52]));
    if (KvRoCe > 7) KvRoCe = int8_t(KvRoCe - 16);
    KvT[1] = KvRoCe;

    int8_t KvReCe = int8_t(nibble1(EE[52]));
    if (KvReCe > 7) KvReCe = int8_t(KvReCe - 16);
    KvT[3] = KvReCe;

    uint8_t kvScale = nibble3(EE[56]);

    for (int i = 0; i < 24; ++i)
    {
        for (int j = 0; j < 32; ++j)
        {
            const int p = 32 * i + j;
            uint8_t split = uint8_t(2 * (p / 32 - (p / 64) * 2) + (p & 1));
            float kv = float(KvT[split]) / pow2f(kvScale);
            kvTemp[p] = kv;
        }
    }

    float temp = absf(kvTemp[0]);
    for (int p = 1; p < 768; ++p)
    {
        float v = absf(kvTemp[p]);
        if (v > temp) temp = v;
    }

    kvScale = 0;
    while (temp < 63.4f)
    {
        temp *= 2.0f;
        kvScale++;
    }

    for (int p = 0; p < 768; ++p)
    {
        float v = kvTemp[p] * pow2f(kvScale);
        if (v < 0.0f) v -= 0.5f;
        else          v += 0.5f;
        c.kv[p] = int8_t(v);
    }

    c.kvScale = kvScale;
}

// ------------------------------------------------------------
// CILC parameters (ExtractCILCParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_cilc(MLX90640_Calibration& c)
{
    uint8_t calibrationModeEE = uint8_t((EE[10] & 0x0800u) >> 4);
    calibrationModeEE ^= 0x80u;

    float ilChessC0 = float(EE[53] & 0x003Fu);
    if (ilChessC0 > 31.0f) ilChessC0 -= 64.0f;
    ilChessC0 /= 16.0f;

    float ilChessC1 = float((EE[53] & 0x07C0u) >> 6);
    if (ilChessC1 > 15.0f) ilChessC1 -= 32.0f;
    ilChessC1 /= 2.0f;

    float ilChessC2 = float((EE[53] & 0xF800u) >> 11);
    if (ilChessC2 > 15.0f) ilChessC2 -= 32.0f;
    ilChessC2 /= 8.0f;

    c.calibrationModeEE = calibrationModeEE;
    c.ilChessC[0] = ilChessC0;
    c.ilChessC[1] = ilChessC1;
    c.ilChessC[2] = ilChessC2;
}

// ------------------------------------------------------------
// Deviating pixels (ExtractDeviatingPixels) – adjacency checks omitted
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr void extract_deviating_pixels(MLX90640_Calibration& c)
{
    for (int i = 0; i < 5; ++i)
    {
        c.brokenPixels[i]  = 0xFFFFu;
        c.outlierPixels[i] = 0xFFFFu;
    }

    uint16_t brokenPixCnt  = 0;
    uint16_t outlierPixCnt = 0;

    for (uint16_t pix = 0;
         pix < 768 && brokenPixCnt < 5 && outlierPixCnt < 5;
         ++pix)
    {
        const uint16_t word = EE[64 + pix];
        if (word == 0)
        {
            c.brokenPixels[brokenPixCnt++] = pix;
        }
        else if ((word & 0x0001u) != 0u)
        {
            c.outlierPixels[outlierPixCnt++] = pix;
        }
    }

    // NOTE: adjacency error codes are intentionally omitted (flight code doesn’t need them);
    // your ground-side tooling can run the full Melexis ExtractDeviatingPixels if desired.
}

// ------------------------------------------------------------
// Orchestrator (equivalent to MLX90640_ExtractParameters)
// ------------------------------------------------------------
template<const uint16_t* EE>
constexpr MLX90640_Calibration parse_eeprom()
{
    MLX90640_Calibration c{};

    extract_vdd<EE>(c);
    extract_ptat<EE>(c);
    extract_gain<EE>(c);
    extract_tgc<EE>(c);
    extract_resolution<EE>(c);
    extract_ksta<EE>(c);
    extract_ksto<EE>(c);
    extract_cp<EE>(c);
    extract_alpha<EE>(c);
    extract_offset<EE>(c);
    extract_kta<EE>(c);
    extract_kv<EE>(c);
    extract_cilc<EE>(c);
    extract_deviating_pixels<EE>(c);

    return c;
}

constexpr MLX90640_Calibration MLX90640_CAL = parse_eeprom<MLX90640_EEPROM>();
