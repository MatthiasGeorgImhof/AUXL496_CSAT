#pragma once

#ifdef __arm____
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal.h"
#else
#include "mock_hal.h"
#endif


// Forward declaration only
struct CanardAdapter;

class CanTxQueueDrainer
{
public:
    CanTxQueueDrainer(CanardAdapter* adapter, CAN_HandleTypeDef* hcan);

    void drain();
    void irq_safe_drain();

private:
    CanardAdapter* adapter_;
    CAN_HandleTypeDef* hcan_;
};

