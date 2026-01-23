#ifndef __DCMICapture_HPP__
#define __DCMICapture_HPP__

#include <cstdint>

#ifdef __arm__
#include "stm32l4xx_hal.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

#include "CameraDriver.hpp"

class DcmiCapture
{
public:
    DcmiCapture() = default;

    [[maybe_unused]]
    bool configure(PixelFormat fmt, uint16_t /*width*/, uint16_t /*height*/)
    {
        // Enable clocks
        __HAL_RCC_DCMI_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        // Reset DCMI
        DCMI->CR = 0;

        // Configure polarity (sensor dependent)
        DCMI->CR =
            DCMI_CR_CM |     // continuous mode
            DCMI_CR_PCKPOL | // pixel clock polarity
            DCMI_CR_HSPOL |  // HSYNC polarity
            DCMI_CR_VSPOL;   // VSYNC polarity

        // JPEG mode if needed
        if (fmt == PixelFormat::JPEG)
            DCMI->CR |= DCMI_CR_JPEG;

        // Enable DCMI
        DCMI->CR |= DCMI_CR_ENABLE;

        return true;
    }

    bool start(uint32_t *buffer, size_t words)
    {
        // Configure DMA2 Channel 6
        DMA2_Channel6->CCR = 0;
        DMA2_Channel6->CPAR = (uint32_t)&DCMI->DR;
        DMA2_Channel6->CMAR = (uint32_t)buffer;
        DMA2_Channel6->CNDTR = words;

        DMA2_Channel6->CCR =
            DMA_CCR_MINC |
            DMA_CCR_PSIZE_1 | // 32-bit
            DMA_CCR_MSIZE_1 | // 32-bit
            DMA_CCR_TCIE;

        DMA2_Channel6->CCR |= DMA_CCR_EN;

        // Start capture
        DCMI->CR |= DCMI_CR_CAPTURE;

        return true;
    }

    bool stop()
    {
        DCMI->CR &= ~DCMI_CR_CAPTURE;
        DMA2_Channel6->CCR &= ~DMA_CCR_EN;
        return true;
    }

    bool frameReady() const
    {
        return (DCMI->MISR & DCMI_MIS_FRAME_MIS) != 0;
    }

    bool testPolarity(PixelFormat fmt,
                      uint16_t width,
                      uint16_t height,
                      uint32_t* buffer,
                      size_t words)
    {
        // 1. Configure DCMI with given polarity
        configure(fmt, width, height);

        // 2. Start DMA + DCMI
        start(buffer, words);

        // 3. Wait a short time
        for (uint32_t i = 0; i < 1000000; ++i) { __asm__ volatile("nop"); }

        // 4. Stop capture
        stop();

        // 5. Check if DMA wrote anything
        for (size_t i = 0; i < words; i++)
            if (buffer[i] != 0)
                return true;

        return false;
    }

    void setPolarity(bool hsyncActiveHigh, bool vsyncActiveHigh)
    {
        uint32_t cr = DCMI->CR;

        if (hsyncActiveHigh)
            cr |= DCMI_CR_HSPOL;
        else
            cr &= ~DCMI_CR_HSPOL;

        if (vsyncActiveHigh)
            cr |= DCMI_CR_VSPOL;
        else
            cr &= ~DCMI_CR_VSPOL;

        DCMI->CR = cr;
    }

    bool findWorkingPolarity(PixelFormat fmt,
                             uint16_t width,
                             uint16_t height,
                             uint32_t* buffer,
                             size_t words)
    {
        const bool polarityOptions[2] = {false, true};

        for (bool h : polarityOptions)
        for (bool v : polarityOptions)
        {
            setPolarity(h, v);

            if (testPolarity(fmt, width, height, buffer, words))
            {
                // store working polarity
                workingHsync = h;
                workingVsync = v;
                return true;
            }
        }

        return false;
    }

public:
    bool workingHsync, workingVsync;

};

#endif // __DCMICapture_HPP__
