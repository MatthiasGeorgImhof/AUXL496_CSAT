#ifdef __x86_64__

#include "mock_hal/mock_hal_irq.h"

void HAL_NVIC_EnableIRQ(IRQn_Type /*IRQn*/) {}
void HAL_NVIC_DisableIRQ(IRQn_Type /*IRQn*/) {}

#endif