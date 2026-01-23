#ifndef MOCK_HAL_SPI_H
#define MOCK_HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cstddef>
#include <stdint.h>
#include <stdbool.h>

// Include core definitions
#include "mock_hal/mock_hal_core.h"

#define MOCK_HAL_SPI_ENABLED

//--- SPI Structures ---
typedef struct {
  uint32_t Mode;                      // Specifies the operation mode for the SPI.
  uint32_t Direction;                 // Specifies the data transfer direction for the SPI.
  uint32_t DataSize;                  // Specifies the data size for the SPI.
  uint32_t ClockPolarity;             // Specifies the serial clock steady state.
  uint32_t ClockPhase;                // Specifies the clock active edge for the SPI serial clock.
  uint32_t NSS;                       // Specifies the NSS signal management mode.
  uint32_t BaudRatePrescaler;         // Specifies the Baud Rate prescaler value
  uint32_t FirstBit;                  // Specifies whether data transfers start from MSB or LSB bit.
  uint32_t TIMode;                    // Specifies if the TI mode is enabled or disabled.
  uint32_t CRCCalculation;            // Specifies if the CRC calculation is enabled or disabled.
  uint32_t CRCPolynomial;            // Specifies the polynomial for the CRC calculation
  uint32_t DataAlign;                 // Specifies the data alignment MSB/LSB.
  uint32_t FifoThreshold;             // Specifies the FIFO threshold.
  uint32_t TxCRCInitializationPattern;// SPI transmit CRC initial value
  uint32_t RxCRCInitializationPattern;// SPI receive CRC initial value
  uint32_t MasterSSIdleness;          // Specifies the Idle Time between two data frame transmission using the NSS management by software.
  uint32_t MasterKeepIOState;        // Specifies if the SPI Master Keep IO State is enabled or disabled.
  uint32_t SuspendState;              // Specifies the SPI suspend mode.
} SPI_InitTypeDef;

typedef struct
{
  void *Instance;  // SPI registers base address.
  SPI_InitTypeDef Init;        // SPI communication parameters.
} SPI_HandleTypeDef;

//--- SPI Mock Function Prototypes ---
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi); // Add SPI init function

//--- SPI Helper Function Prototypes ---
void clear_spi_tx_buffer();
void clear_spi_rx_buffer();
void inject_spi_rx_data(uint8_t *data, size_t size);
void copy_spi_tx_to_rx();

//--- Getter Function Prototypes ---
size_t get_spi_tx_buffer_count();
uint8_t* get_spi_tx_buffer();
size_t get_spi_rx_buffer_count();
uint8_t* get_spi_rx_buffer();
void init_spi_handle(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_SPI_H */