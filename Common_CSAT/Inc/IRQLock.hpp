#ifndef IRQ_LOCK_H
#define IRQ_LOCK_H

#ifdef __arm____
#include "stm32l4xx_hal.h"
#else
#include "mock_hal.h"
#endif

template<int IRQn>
class IrqLock
{
public:
    static void lock()
    {
        if (counter_++ == 0)
        {
            HAL_NVIC_DisableIRQ(static_cast<IRQn_Type>(IRQn));
        }
    }

    static void unlock()
    {
        if (counter_ == 0)
            return;

        if (--counter_ == 0)
        {
            HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>(IRQn));
        }
    }

private:
    static inline uint32_t counter_ = 0;
};

using CanTxIrqLock = IrqLock<CAN1_TX_IRQn>;
using CanRx0IrqLock = IrqLock<CAN1_RX0_IRQn>;
using CanRx1IrqLock = IrqLock<CAN1_RX1_IRQn>;

#endif // IRQ_LOCK_H