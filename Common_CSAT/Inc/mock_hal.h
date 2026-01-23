#ifdef __x86_64__

#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mock_hal/mock_hal_core.h"
#include "mock_hal/mock_hal_can.h"
#include "mock_hal/mock_hal_clock.h"
#include "mock_hal/mock_hal_dcmi.h"
#include "mock_hal/mock_hal_gpio.h"
#include "mock_hal/mock_hal_i2c.h"
#include "mock_hal/mock_hal_irq.h"
#include "mock_hal/mock_hal_mem.h"
#include "mock_hal/mock_hal_rtc.h"
#include "mock_hal/mock_hal_spi.h"
#include "mock_hal/mock_hal_time.h"
#include "mock_hal/mock_hal_timer.h"
#include "mock_hal/mock_hal_uart.h"
#include "mock_hal/mock_hal_usb.h"

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_H */

#endif /* __x86_64__ */
