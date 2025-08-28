#ifndef MOCK_HAL_USB_H
#define MOCK_HAL_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cstddef>
#include <stdint.h>
#include <stdbool.h>

// Include core definitions
#include "mock_hal/mock_hal_core.h"

//--- USB CDC Mock Function Prototypes ---
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//--- USB Helper Function Prototypes ---
void clear_usb_tx_buffer();
size_t get_usb_tx_buffer_count();
uint8_t* get_usb_tx_buffer();

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_USB_H */