#ifdef __x86_64__

#include "mock_hal/mock_hal_uart.h"
#include "mock_hal/mock_hal_time.h"
#include <string.h>

//--- UART Buffers ---
uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];   // UART transmit buffer
int uart_tx_buffer_count = 0;                  // Number of bytes in UART TX buffer

uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];   // UART receive buffer
int uart_rx_buffer_count = 0;                  // Number of bytes in UART RX buffer
int uart_rx_buffer_read_pos = 0;               // Read position in UART RX buffer

//------------------------------------------------------------------------------
//  GLOBAL MOCKED VARIABLES - State
//------------------------------------------------------------------------------

//--- General Mock Variables ---
extern uint32_t current_tick;

//--- UARTEx Mock Variables ---
HAL_UART_RxEventTypeTypeDef mocked_uart_rx_event_type = 0; // Mocked UART Rx event type

uint32_t HAL_UART_Transmit(UART_HandleTypeDef */*huart*/, uint8_t *pData, uint16_t Size, uint32_t /*Timeout*/)
{
    if (uart_tx_buffer_count + Size <= UART_TX_BUFFER_SIZE) {
        memcpy(uart_tx_buffer + uart_tx_buffer_count, pData, Size);
        uart_tx_buffer_count += Size;
        return 0; // HAL_OK
    }
    return 1; // HAL_ERROR, buffer overflow
}


uint32_t HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
     if (huart == NULL || pData == NULL) return HAL_ERROR;

    if (uart_tx_buffer_count + Size <= UART_TX_BUFFER_SIZE) {
        memcpy(uart_tx_buffer + uart_tx_buffer_count, pData, Size);
        uart_tx_buffer_count += Size;
        return 0; // HAL_OK
    }
    return 1; // HAL_ERROR, buffer overflow
}

uint32_t HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
  if (huart == NULL || pData == NULL) return HAL_ERROR;
  uint32_t start_tick = current_tick;
  uint16_t bytes_received = 0;

  while(bytes_received < Size) {
     if(uart_rx_buffer_read_pos >= uart_rx_buffer_count) {
       if( (current_tick-start_tick) >= Timeout) {
            break; //Timeout
        } else {
            //Call delay for context switch or tick update
            HAL_Delay(1);
            continue;
        }
     }
     pData[bytes_received] = uart_rx_buffer[uart_rx_buffer_read_pos];
     uart_rx_buffer_read_pos++;
     bytes_received++;
    }

  return bytes_received == Size ? 0 : 1; //HAL_OK if all bytes are received
}

uint32_t HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    if (huart == NULL || pData == NULL) return HAL_ERROR;
    uint16_t bytes_received = 0;

  while(bytes_received < Size) {
     if(uart_rx_buffer_read_pos >= uart_rx_buffer_count) {
          break; // No Timeout in DMA version, just return the available data
     }
      pData[bytes_received] = uart_rx_buffer[uart_rx_buffer_read_pos];
      uart_rx_buffer_read_pos++;
      bytes_received++;
  }


  return bytes_received == Size ? 0 : 1; //HAL_OK if all bytes are received
}

// UART Injectors
void inject_uart_rx_data(uint8_t *data, int size) {
  if (uart_rx_buffer_count + size <= UART_RX_BUFFER_SIZE) {
        memcpy(uart_rx_buffer + uart_rx_buffer_count, data, size);
        uart_rx_buffer_count += size;
    }
}

// UART Deleters
void clear_uart_rx_buffer(){
  memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer)); //Set all elements to 0
  uart_rx_buffer_count = 0;
  uart_rx_buffer_read_pos = 0;
}

void clear_uart_tx_buffer() {
    memset(uart_tx_buffer, 0, sizeof(uart_tx_buffer)); //Set all elements to 0
    uart_tx_buffer_count = 0;
}

// UART Getters
int get_uart_tx_buffer_count(){
  return uart_tx_buffer_count;
}

uint8_t* get_uart_tx_buffer() {
  return uart_tx_buffer;
}

void init_uart_handle(UART_HandleTypeDef *huart)
{
    huart->Init.BaudRate = 115200;
    huart->Init.WordLength = 8;
    huart->Init.StopBits = 1;
    huart->Init.Parity = 0;
    huart->Init.Mode = 3;
    huart->Init.HwFlowCtl = 0;
    huart->Init.OverSampling = 0;
    huart->Init.OneBitSampling = 0;
    huart->Init.ADVFEATURE = 0;
}

// ----- Mock UARTEx Functions -----

void set_mocked_uart_rx_event_type(HAL_UART_RxEventTypeTypeDef event_type) {
    mocked_uart_rx_event_type = event_type;
}

uint32_t HAL_UARTEx_GetRxEventType(UART_HandleTypeDef */*huart*/) {
    return mocked_uart_rx_event_type;
}

uint32_t HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
   if (huart == NULL || pData == NULL) return HAL_ERROR;
    uint16_t bytes_received = 0;

    while(bytes_received < Size) {
        if(uart_rx_buffer_read_pos >= uart_rx_buffer_count) {
            break; // No Timeout in DMA version, just return the available data
        }
        pData[bytes_received] = uart_rx_buffer[uart_rx_buffer_read_pos];
        uart_rx_buffer_read_pos++;
        bytes_received++;
    }

    return bytes_received == Size ? 0 : 1; //HAL_OK if all bytes are received
}

#endif