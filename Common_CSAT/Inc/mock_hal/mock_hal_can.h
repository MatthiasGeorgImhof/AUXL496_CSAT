#ifndef MOCK_HAL_CAN_H
#define MOCK_HAL_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Include core definitions
#include "mock_hal/mock_hal_core.h"

//--- CAN Defines ---
#define CAN_FILTERMODE_IDMASK       (0x00000000U)  // Identifier mask mode
#define CAN_FILTERMODE_IDLIST       (0x00000001U)  // Identifier list mode
#define CAN_FILTERSCALE_16BIT       (0x00000000U)  // Two 16-bit filters
#define CAN_FILTERSCALE_32BIT       (0x00000001U)  // One 32-bit filter
#define CAN_FILTER_DISABLE          (0x00000000U)  // Disable filter
#define CAN_FILTER_ENABLE           (0x00000001U)  // Enable filter
#define CAN_FILTER_FIFO0            (0x00000000U)  // Filter FIFO 0 assignment
#define CAN_FILTER_FIFO1            (0x00000001U)  // Filter FIFO 1 assignment
#define CAN_ID_STD                  (0x00000000U)  // Standard Id
#define CAN_ID_EXT                  (0x00000004U)  // Extended Id
#define CAN_RTR_DATA                (0x00000000U)  // Data frame
#define CAN_RTR_REMOTE              (0x00000002U)  // Remote frame
#define CAN_RX_FIFO0                (0x00000000U)  // CAN receive FIFO 0
#define CAN_RX_FIFO1                (0x00000001U)  // CAN receive FIFO 1
#define CAN_TX_MAILBOX0             (0x00000001U)  // Tx Mailbox 0
#define CAN_TX_MAILBOX1             (0x00000002U)  // Tx Mailbox 1
#define CAN_TX_MAILBOX2             (0x00000004U)  // Tx Mailbox 2

//--- CAN Structures ---
typedef struct {
    uint32_t StdId;      // Standard Identifier
    uint32_t ExtId;      // Extended Identifier
    uint8_t IDE;         // Identifier Extension
    uint8_t RTR;         // Remote Transmission Request
    uint8_t DLC;         // Data Length Code
    uint8_t Data[8];     // Data bytes (up to 8 bytes)
} CAN_TxHeaderTypeDef;

typedef struct {
    uint32_t StdId;      // Standard Identifier
    uint32_t ExtId;      // Extended Identifier
    uint8_t IDE;         // Identifier Extension
    uint8_t RTR;         // Remote Transmission Request
    uint8_t DLC;         // Data Length Code
    uint8_t Data[8];     // Data bytes (up to 8 bytes)
    uint8_t FIFONumber;  // FIFO Number (0 or 1)
} CAN_RxHeaderTypeDef;

typedef struct {
  uint8_t  FilterBank;            // Filter bank number
  uint32_t FilterMode;            // Filter mode (Id Mask or Id List)
  uint32_t FilterScale;           // Filter scale (16-bit or 32-bit)
  uint32_t FilterIdHigh;          // Filter ID (upper 16 bits)
  uint32_t FilterIdLow;           // Filter ID (lower 16 bits)
  uint32_t FilterMaskIdHigh;      // Filter Mask ID (upper 16 bits)
  uint32_t FilterMaskIdLow;       // Filter Mask ID (lower 16 bits)
  uint32_t FilterFIFOAssignment;   // FIFO Assignment (0 or 1)
  uint32_t FilterActivation;       // Filter Activation (Enable or Disable)
  uint32_t SlaveStartFilterBank; //Can be used to determine start filter bank on slave
} CAN_FilterTypeDef;

typedef struct {
    CAN_TxHeaderTypeDef TxHeader; // CAN Tx Header
    uint32_t  pData[2];           // Pointer to data (assuming 64-bit architecture)
    uint32_t  Mailbox;           // Mailbox number
} CAN_TxMessage_t;

typedef struct {
    CAN_RxHeaderTypeDef RxHeader; // CAN Rx Header
    uint32_t  pData[2];           // Pointer to data (assuming 64-bit architecture)
} CAN_RxMessage_t;

typedef struct {
    int dummy;                   // Minimal definition for mocking
} CAN_HandleTypeDef;

//--- CAN Mock Function Prototypes ---
HAL_StatusTypeDef HAL_CAN_AddTxMessage(void *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(void *hcan, uint32_t Fifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
HAL_StatusTypeDef HAL_CAN_GetTxMailboxesFreeLevel(void *hcan);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(void *hcan, CAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_CAN_GetRxFifoFillLevel(void *hcan, uint32_t Fifo);

//--- CAN Helper Function Prototypes ---
void inject_can_rx_message(CAN_RxHeaderTypeDef header, uint8_t data[]);
void clear_can_tx_buffer();
void clear_can_rx_buffer();
void move_can_tx_to_rx();

//--- Getter Function Prototypes ---
int get_can_tx_buffer_count();
CAN_TxMessage_t get_can_tx_message(int pos);
void set_current_free_mailboxes(uint32_t free_mailboxes);
void set_current_rx_fifo_fill_level(uint32_t rx_fifo_level);

HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);

#define __HAL_CAN_ENABLE_IT(__HANDLE__, __INTERRUPT__)   (void)0
#define __HAL_CAN_DISABLE_IT(__HANDLE__, __INTERRUPT__)  (void)0

#ifdef __cplusplus
}
#endif

#endif /* MOCK_HAL_CAN_H */