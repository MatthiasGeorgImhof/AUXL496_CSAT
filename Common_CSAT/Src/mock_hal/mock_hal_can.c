#ifdef __x86_64__

#include "mock_hal/mock_hal_can.h"
#include <string.h>
#include <stdio.h> // For logging (optional)

//--- CAN Buffers ---
CAN_TxMessage_t can_tx_buffer[CAN_TX_BUFFER_SIZE]; // CAN transmit buffer
int can_tx_buffer_count = 0;                       // Number of messages in CAN TX buffer

CAN_RxMessage_t can_rx_buffer[CAN_RX_BUFFER_SIZE]; // CAN receive buffer
int can_rx_buffer_count = 0;                       // Number of messages in CAN RX buffer

//------------------------------------------------------------------------------
//  GLOBAL MOCKED VARIABLES - State
//------------------------------------------------------------------------------

//--- General Mock Variables ---
extern uint32_t current_tick;
uint32_t current_free_mailboxes = 3;        // Number of free CAN mailboxes
uint32_t current_rx_fifo_fill_level = 0;   // Fill level of CAN RX FIFO


uint32_t HAL_CAN_AddTxMessage(void */*hcan*/, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
{
    if (pHeader == NULL) return 1; //HAL_ERROR
    if (can_tx_buffer_count < CAN_TX_BUFFER_SIZE) {
        can_tx_buffer[can_tx_buffer_count].TxHeader = *pHeader;

        // Copy data if the buffer pointer is not null
        if (aData != NULL) {
             memcpy(&can_tx_buffer[can_tx_buffer_count].pData[0], aData, pHeader->DLC);
        } else {
             memset(&can_tx_buffer[can_tx_buffer_count].pData[0], 0, pHeader->DLC);
        }

        can_tx_buffer[can_tx_buffer_count].Mailbox = 0; // Mock mailbox
        *pTxMailbox = 0;
        can_tx_buffer_count++;
        return 0; // HAL_OK
    }
    return 1; // HAL_ERROR, buffer full
}


uint32_t HAL_CAN_GetRxMessage(void *hcan, uint32_t /*Fifo*/, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]) {
    if(hcan == NULL || pHeader == NULL || aData == NULL) {
        return 1; //HAL_ERROR
    }
    if(current_rx_fifo_fill_level == 0) {
        return 1; // HAL_ERROR
    }
    if (can_rx_buffer_count > 0) {
       *pHeader = can_rx_buffer[0].RxHeader;
       // Copy data to given location if aData pointer is not null
       if (aData != NULL) {
           memcpy(aData, &can_rx_buffer[0].pData[0], pHeader->DLC);
       }
        // Shift buffer
        for(int i=0; i<can_rx_buffer_count-1; i++) {
          can_rx_buffer[i] = can_rx_buffer[i+1];
        }
        can_rx_buffer_count--;
        current_rx_fifo_fill_level--;
        return 0; // HAL_OK
    }

    return 1; // HAL_ERROR, No data available
}


uint32_t HAL_CAN_GetTxMailboxesFreeLevel(void */*hcan*/) {
    return current_free_mailboxes;
}

uint32_t HAL_CAN_ConfigFilter(void */*hcan*/, CAN_FilterTypeDef */*sFilterConfig*/) {
    // Mock implementation - for now, just return HAL_OK
    return 0; // HAL_OK
}

uint32_t HAL_CAN_GetRxFifoFillLevel(void */*hcan*/, uint32_t /*Fifo*/) {
    return current_rx_fifo_fill_level;
}

// ----- Injector and Deleter Functions ------

// CAN Injectors
void inject_can_rx_message(CAN_RxHeaderTypeDef header, uint8_t data[]) {
    if (can_rx_buffer_count < CAN_RX_BUFFER_SIZE) {
        can_rx_buffer[can_rx_buffer_count].RxHeader = header;
        memcpy(&can_rx_buffer[can_rx_buffer_count].pData[0], data, header.DLC);
         can_rx_buffer_count++;
         current_rx_fifo_fill_level++;
    }
}

// CAN Deleters
void clear_can_tx_buffer() {
    memset(can_tx_buffer, 0, sizeof(can_tx_buffer));  // Set all elements to 0
    can_tx_buffer_count = 0;
}

void clear_can_rx_buffer() {
    memset(can_rx_buffer, 0, sizeof(can_rx_buffer));
    can_rx_buffer_count = 0;
    current_rx_fifo_fill_level = 0;  // IMPORTANT: Reset the fill level!
}

// CAN Mover
void TxHeaderToRxHeader(const CAN_TxHeaderTypeDef *txHeader, CAN_RxHeaderTypeDef *rxHeader, uint8_t fifoNumber) {
    if (!txHeader || !rxHeader) {
        // Handle null pointer error (e.g., return, log an error, etc.)
        return; // Or potentially set some error flag
    }

    rxHeader->StdId = txHeader->StdId;
    rxHeader->ExtId = txHeader->ExtId;
    rxHeader->IDE = txHeader->IDE;
    rxHeader->RTR = txHeader->RTR;
    rxHeader->DLC = txHeader->DLC;
    rxHeader->FIFONumber = fifoNumber;

    memcpy(rxHeader->Data, txHeader->Data, txHeader->DLC);
}

void move_can_tx_to_rx() {
    for (int i = 0; i < can_tx_buffer_count; ++i) {
        if (can_rx_buffer_count < CAN_RX_BUFFER_SIZE) {
            TxHeaderToRxHeader(&can_tx_buffer[i].TxHeader, &can_rx_buffer[i].RxHeader, 0);
            memcpy(can_rx_buffer[i].pData, can_tx_buffer[i].pData, sizeof(uint32_t)*2);

            can_rx_buffer_count++;
            current_rx_fifo_fill_level++;
        } else {
            // Handle RX buffer overflow (optional)
            printf("Warning: CAN RX buffer overflow!\n");
            break; // Stop moving messages
        }
    }

    // Clear the TX buffer
    clear_can_tx_buffer();
}

// ----- Getter Functions -----

// CAN Getters
int get_can_tx_buffer_count(){
  return can_tx_buffer_count;
}

CAN_TxMessage_t get_can_tx_message(int pos){
  if(pos < can_tx_buffer_count){
    return can_tx_buffer[pos];
  }
  CAN_TxMessage_t empty;
  memset(&empty,0, sizeof(empty));
  return empty;
}

// Status Setters
void set_current_free_mailboxes(uint32_t free_mailboxes) {
  current_free_mailboxes = free_mailboxes;
}

void set_current_rx_fifo_fill_level(uint32_t rx_fifo_level){
    current_rx_fifo_fill_level = rx_fifo_level;
}

// track which interrupts are currently enabled
static uint32_t mock_can_enabled_interrupts = 0;

HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
    (void)hcan;
    mock_can_enabled_interrupts |= ActiveITs;
    return 0; // HAL_OK
}

HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs)
{
    (void)hcan;
    mock_can_enabled_interrupts &= ~ActiveITs;
    return 0; // HAL_OK
}


#endif