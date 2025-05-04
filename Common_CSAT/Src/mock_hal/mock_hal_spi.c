#ifdef __x86_64__

#include "mock_hal/mock_hal_spi.h"
#include <string.h>
#include <stdio.h> //For printf

//--- SPI Buffers ---
uint8_t spi_tx_buffer[SPI_TX_BUFFER_SIZE];   // SPI transmit buffer
int spi_tx_buffer_count = 0;                  // Number of bytes in SPI TX buffer

uint8_t spi_rx_buffer[SPI_RX_BUFFER_SIZE];   // SPI receive buffer
int spi_rx_buffer_count = 0;                  // Number of bytes in SPI RX buffer
int spi_rx_buffer_read_pos = 0;               // Read position in SPI RX buffer

uint32_t HAL_SPI_Transmit(SPI_HandleTypeDef */*hspi*/, uint8_t *pData, uint16_t Size, uint32_t /*Timeout*/) {
    if (!pData) return 1; // HAL_ERROR
    if (spi_tx_buffer_count + Size <= SPI_TX_BUFFER_SIZE) {
        memcpy(spi_tx_buffer + spi_tx_buffer_count, pData, Size);
        spi_tx_buffer_count += Size;
        return 0; // HAL_OK
    }
    return 1; // HAL_ERROR, buffer overflow
}

uint32_t HAL_SPI_Receive(SPI_HandleTypeDef */*hspi*/, uint8_t *pData, uint16_t Size, uint32_t /*Timeout*/) {
   if (!pData) return 1; // HAL_ERROR

    //Copy injected rx data into provided buffer
    if (spi_rx_buffer_count > 0) {
        if (Size > spi_rx_buffer_count) {
            return 1; //HAL_ERROR. Trying to receive more than available
        }
        memcpy(pData, spi_rx_buffer + spi_rx_buffer_read_pos, Size);
        spi_rx_buffer_read_pos += Size;

        if (spi_rx_buffer_read_pos >= spi_rx_buffer_count) {
            spi_rx_buffer_read_pos = 0; // reset to beginning
        }
         return 0; // HAL_OK
    }
    return 1; // HAL_ERROR. no data available
}


uint32_t HAL_SPI_TransmitReceive(SPI_HandleTypeDef */*hspi*/, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t /*Timeout*/) {
   if (!pTxData || !pRxData) return 1; // HAL_ERROR

    //Transmit part
    if (spi_tx_buffer_count + Size <= SPI_TX_BUFFER_SIZE) {
        memcpy(spi_tx_buffer + spi_tx_buffer_count, pTxData, Size);
        spi_tx_buffer_count += Size;
    } else {
       return 1; // HAL_ERROR, buffer overflow
    }

     //Receive part
    if (spi_rx_buffer_count > 0) {
        if (Size > spi_rx_buffer_count) {
             return 1; //HAL_ERROR. Trying to receive more than available
        }

        memcpy(pRxData, spi_rx_buffer + spi_rx_buffer_read_pos, Size);
        spi_rx_buffer_read_pos += Size;

        if (spi_rx_buffer_read_pos >= spi_rx_buffer_count) {
            spi_rx_buffer_read_pos = 0; // reset to beginning
        }
    } else {
        return 1; // HAL_ERROR, no data available
    }

    return 0; // HAL_OK
}

// Add a mock implementation for SPI initialization
uint32_t HAL_SPI_Init(SPI_HandleTypeDef *hspi) {
    if (!hspi) return 1; // HAL_ERROR

    // In a simple mock, we don't *really* initialize anything.
    // But you *could* store the SPI_InitTypeDef values if you wanted to make assertions about them later.
    return 0; // HAL_OK
}

// SPI Injectors
void inject_spi_rx_data(uint8_t *data, int size) {
    if (spi_rx_buffer_count + size <= SPI_RX_BUFFER_SIZE) {
        memcpy(spi_rx_buffer + spi_rx_buffer_count, data, size);
        spi_rx_buffer_count += size;
    }
}

// SPI Deleters
void clear_spi_tx_buffer() {
    memset(spi_tx_buffer, 0, sizeof(spi_tx_buffer));  // Set all elements to 0
    spi_tx_buffer_count = 0;
}

void clear_spi_rx_buffer() {
    memset(spi_rx_buffer, 0, sizeof(spi_rx_buffer));
    spi_rx_buffer_count = 0;
    spi_rx_buffer_read_pos = 0;
}

// ----- Getter Functions -----

int get_spi_tx_buffer_count() {
    return spi_tx_buffer_count;
}

uint8_t* get_spi_tx_buffer() {
    return spi_tx_buffer;
}

int get_spi_rx_buffer_count() {
    return spi_rx_buffer_count;
}

uint8_t* get_spi_rx_buffer() {
    return spi_rx_buffer;
}

void init_spi_handle(SPI_HandleTypeDef *hspi) {
   // Set some default values for the SPI handle
    hspi->Init.Mode = 0;
    hspi->Init.Direction = 0;
    hspi->Init.DataSize = 0;
    hspi->Init.ClockPolarity = 0;
    hspi->Init.ClockPhase = 0;
    hspi->Init.NSS = 0;
    hspi->Init.BaudRatePrescaler = 0;
    hspi->Init.FirstBit = 0;
    hspi->Init.TIMode = 0;
    hspi->Init.CRCCalculation = 0;
    hspi->Init.CRCPolynomial = 0;
    hspi->Init.DataAlign = 0;
    hspi->Init.FifoThreshold = 0;
    hspi->Init.TxCRCInitializationPattern = 0;
    hspi->Init.RxCRCInitializationPattern = 0;
    hspi->Init.MasterSSIdleness = 0;
    hspi->Init.MasterKeepIOState = 0;
    hspi->Init.SuspendState = 0;
}

// ----- SPI Helper Functions -----

void copy_spi_tx_to_rx() {
    if (spi_tx_buffer_count > 0) {
        if (spi_rx_buffer_count + spi_tx_buffer_count <= SPI_RX_BUFFER_SIZE) {
            memcpy(spi_rx_buffer + spi_rx_buffer_count, spi_tx_buffer, spi_tx_buffer_count);
            spi_rx_buffer_count += spi_tx_buffer_count;
            clear_spi_tx_buffer();  // Optionally clear the TX buffer after copying
        } else {
            // Handle the case where the RX buffer is not large enough
            printf("Warning: SPI RX buffer overflow in copy_spi_tx_to_rx!\n");
            // You might choose to copy only what fits, or return an error, etc.
        }
    }
}

#endif