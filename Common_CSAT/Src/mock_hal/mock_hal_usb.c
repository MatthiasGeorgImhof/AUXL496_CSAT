#ifdef __x86_64__

#include "mock_hal/mock_hal_usb.h"
#include <string.h>

//--- USB Buffers ---
uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];   // USB transmit buffer
size_t usb_tx_buffer_count = 0;                  // Number of bytes in USB TX buffer

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
  if (usb_tx_buffer_count + Len <= USB_TX_BUFFER_SIZE) {
        memcpy(usb_tx_buffer + usb_tx_buffer_count, Buf, Len);
        usb_tx_buffer_count += Len;
        return 0; // Success
    }
    return 1;  // Error (Buffer overflow or something similar)
}

// USB Deleters
void clear_usb_tx_buffer(){
    memset(usb_tx_buffer, 0, sizeof(usb_tx_buffer)); //Set all elements to 0
    usb_tx_buffer_count = 0;
}

// USB Getters
size_t get_usb_tx_buffer_count() {
    return usb_tx_buffer_count;
}

uint8_t* get_usb_tx_buffer() {
    return usb_tx_buffer;
}

#endif