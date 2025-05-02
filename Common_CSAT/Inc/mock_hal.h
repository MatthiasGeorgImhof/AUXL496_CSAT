#ifdef __x86_64__
#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

//------------------------------------------------------------------------------
//  DEFINES
//------------------------------------------------------------------------------

#define HAL_MAX_DELAY           0xFFFFFFFFU
#define I2C_MEMADD_SIZE_8BIT    (0x00000001U)
#define I2C_MEMADD_SIZE_16BIT   (0x00000002U)

//--- HAL Status ---
#define  HAL_OK       0x00
#define  HAL_ERROR    0x01
#define  HAL_BUSY     0x02
#define  HAL_TIMEOUT  0x03

//--- Buffer Sizes ---
#define CAN_TX_BUFFER_SIZE  50
#define CAN_RX_BUFFER_SIZE  50
#define UART_TX_BUFFER_SIZE 256
#define UART_RX_BUFFER_SIZE 256
#define I2C_MEM_BUFFER_SIZE 256
#define USB_TX_BUFFER_SIZE  256
#define SPI_TX_BUFFER_SIZE 256
#define SPI_RX_BUFFER_SIZE 256

//--- GPIO Defines ---
#define GPIO_PIN_0  ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1  ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2  ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3  ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4  ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5  ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6  ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7  ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8  ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9  ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10 ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11 ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12 ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13 ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14 ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15 ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All ((uint16_t)0xFFFF)  /* All pins selected */
#define GPIO_PIN_MASK ((uint32_t)0x0000FFFF) /* PIN mask for assert test */

//--- CAN Defines ---
#define CAN_FILTERMODE_IDMASK       (0x00000000U)  /*!< Identifier mask mode */
#define CAN_FILTERMODE_IDLIST       (0x00000001U)  /*!< Identifier list mode */
#define CAN_FILTERSCALE_16BIT       (0x00000000U)  /*!< Two 16-bit filters */
#define CAN_FILTERSCALE_32BIT       (0x00000001U)  /*!< One 32-bit filter  */
#define CAN_FILTER_DISABLE          (0x00000000U)  /*!< Disable filter */
#define CAN_FILTER_ENABLE           (0x00000001U)  /*!< Enable filter  */
#define CAN_FILTER_FIFO0            (0x00000000U)  /*!< Filter FIFO 0 assignment for filter x */
#define CAN_FILTER_FIFO1            (0x00000001U)  /*!< Filter FIFO 1 assignment for filter x */
#define CAN_ID_STD                  (0x00000000U)  /*!< Standard Id */
#define CAN_ID_EXT                  (0x00000004U)  /*!< Extended Id */
#define CAN_RTR_DATA                (0x00000000U)  /*!< Data frame   */
#define CAN_RTR_REMOTE              (0x00000002U)  /*!< Remote frame */
#define CAN_RX_FIFO0                (0x00000000U)  /*!< CAN receive FIFO 0 */
#define CAN_RX_FIFO1                (0x00000001U)  /*!< CAN receive FIFO 1 */
#define CAN_TX_MAILBOX0             (0x00000001U)  /*!< Tx Mailbox 0  */
#define CAN_TX_MAILBOX1             (0x00000002U)  /*!< Tx Mailbox 1  */
#define CAN_TX_MAILBOX2             (0x00000004U)  /*!< Tx Mailbox 2  */

//--- UARTEx Defines ---
#define HAL_UART_RXEVENT_HT   0  // Choose a representative value.  Could be an enum.
#define HAL_UART_RXEVENT_IDLE 1

//------------------------------------------------------------------------------
//  ENUMS
//------------------------------------------------------------------------------

typedef uint32_t HAL_StatusTypeDef;

// GPIO Pin State Definition
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

//------------------------------------------------------------------------------
//  STRUCTURE DEFINITIONS
//------------------------------------------------------------------------------

//--- GPIO Structures ---
typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

typedef struct {
    void *Instance; // e.g., GPIOA, GPIOB, etc.  Use void* for mocking
    GPIO_InitTypeDef Init;
} GPIO_TypeDef;

//--- CAN Structures ---
typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint8_t IDE;
    uint8_t RTR;
    uint8_t DLC;
    uint8_t Data[8];
} CAN_TxHeaderTypeDef;

typedef struct {
    uint32_t StdId;
    uint32_t ExtId;
    uint8_t IDE;
    uint8_t RTR;
    uint8_t DLC;
    uint8_t Data[8];
    uint8_t FIFONumber;
} CAN_RxHeaderTypeDef;

typedef struct {
  uint8_t  FilterBank;
  uint32_t FilterMode;
  uint32_t FilterScale;
  uint32_t FilterIdHigh;
  uint32_t FilterIdLow;
  uint32_t FilterMaskIdHigh;
  uint32_t FilterMaskIdLow;
  uint32_t FilterFIFOAssignment;
  uint32_t FilterActivation;
  uint32_t SlaveStartFilterBank;
} CAN_FilterTypeDef;

typedef struct {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t  pData[2];
    uint32_t  Mailbox;
} CAN_TxMessage_t;

typedef struct {
    CAN_RxHeaderTypeDef RxHeader;
    uint32_t  pData[2];
} CAN_RxMessage_t;

typedef struct {
    int dummy; // Minimal definition
} CAN_HandleTypeDef;

//--- UART Structures ---
typedef struct {
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
    uint32_t HwFlowCtl;
    uint32_t OverSampling;
    uint32_t OneBitSampling;
    uint32_t ADVFEATURE;
} UART_InitTypeDef;

typedef struct {
   UART_InitTypeDef Init;
} UART_HandleTypeDef;

typedef uint32_t HAL_UART_RxEventTypeTypeDef; // Or whatever type is appropriate

//--- I2C Structures ---
typedef struct {
    uint32_t ClockSpeed;
    uint32_t DutyCycle;
    uint32_t OwnAddress1;
    uint32_t AddressingMode;
    uint32_t DualAddressMode;
    uint32_t OwnAddress2;
    uint32_t GeneralCallMode;
    uint32_t NoStretchMode;
    uint32_t Master;
    uint32_t Init;
} I2C_InitTypeDef;

typedef struct {
   I2C_InitTypeDef Instance;
} I2C_HandleTypeDef;

//--- SPI Structures ---

typedef struct {
  uint32_t Mode;         /*!< Specifies the operation mode for the SPI.
                                  This parameter can be a value of @ref SPI_Mode */

  uint32_t Direction;    /*!< Specifies the data transfer direction for the SPI.
                                  This parameter can be a value of @ref SPI_Direction_Mode */

  uint32_t DataSize;     /*!< Specifies the data size for the SPI.
                                  This parameter can be a value of @ref SPI_Data_Size */

  uint32_t ClockPolarity; /*!< Specifies the serial clock steady state.
                                  This parameter can be a value of @ref SPI_Clock_Polarity */

  uint32_t ClockPhase;    /*!< Specifies the clock active edge for the SPI serial clock.
                                  This parameter can be a value of @ref SPI_Clock_Phase */

  uint32_t NSS;           /*!< Specifies the NSS signal management mode.
                                  This parameter can be a value of @ref SPI_NSS_management */

  uint32_t BaudRatePrescaler; /*!< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock speed.
                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler */

  uint32_t FirstBit;      /*!< Specifies whether data transfers start from MSB or LSB bit.
                                  This parameter can be a value of @ref SPI_MSB_LSB_transmission */

  uint32_t TIMode;        /*!< Specifies if the TI mode is enabled or disabled.
                                  This parameter can be a value of @ref SPI_TI_Mode */

  uint32_t CRCCalculation; /*!< Specifies if the CRC calculation is enabled or disabled.
                                  This parameter can be a value of @ref SPI_CRC_calculation */

  uint32_t CRCPolynomial; /*!< Specifies the polynomial for the CRC calculation
                                  @note The CRC polynomial must be in the range [0x7, 0x409F] */

  uint32_t DataAlign;     /*!< Specifies the data alignment MSB/LSB.
                                  This parameter can be a value of @ref SPI_Data_Align */

  uint32_t FifoThreshold; /*!< Specifies the FIFO threshold.
                                  This parameter can be a value of @ref SPI_FIFO_Threshold */

  uint32_t TxCRCInitializationPattern; /*!< SPI transmit CRC initial value */

  uint32_t RxCRCInitializationPattern; /*!< SPI receive CRC initial value */

  uint32_t MasterSSIdleness;   /*!< Specifies the Idle Time between two data frame transmission using the NSS management by software.
                                  This parameter can be a value of @ref SPI_Master_SS_Idleness */

  uint32_t MasterKeepIOState; /*!< Specifies if the SPI Master Keep IO State is enabled or disabled.
                                  This parameter can be a value of @ref SPI_Master_Keep_IO_State */

  uint32_t SuspendState;       /*!< Specifies the SPI suspend mode.
                                  This parameter can be a value of @ref SPI_Suspend_State */
} SPI_InitTypeDef;

typedef struct
{
  void *Instance;  /*!< SPI registers base address. */

  SPI_InitTypeDef Init;        /*!< SPI communication parameters. */
} SPI_HandleTypeDef;


//------------------------------------------------------------------------------
//  MOCK FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//--- CAN Mock Function Prototypes ---
HAL_StatusTypeDef HAL_CAN_AddTxMessage(void *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(void *hcan, uint32_t Fifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
HAL_StatusTypeDef HAL_CAN_GetTxMailboxesFreeLevel(void *hcan);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(void *hcan, CAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_CAN_GetRxFifoFillLevel(void *hcan, uint32_t Fifo);

//--- UART Mock Function Prototypes ---
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

//--- Time Mock Function Prototypes ---
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
void     HAL_SetTick(uint32_t tick);

//--- I2C Mock Function Prototypes ---
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);

//--- USB CDC Mock Function Prototypes ---
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void clear_usb_tx_buffer();
int get_usb_tx_buffer_count();
uint8_t* get_usb_tx_buffer();

//--- GPIO Mock Function Prototypes ---
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

//--- UARTEx Mock Function Prototypes ---
HAL_StatusTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

//--- SPI Mock Function Prototypes ---
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi); // Add SPI init function

//------------------------------------------------------------------------------
//  EXTERNAL VARIABLES
//------------------------------------------------------------------------------

//--- UARTEx External Variables ---
extern uint32_t mocked_uart_rx_event_type;
void set_mocked_uart_rx_event_type(uint32_t event_type);


//------------------------------------------------------------------------------
//  HELPER FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

//--- Injector and Deleter Function Prototypes ---
void inject_can_rx_message(CAN_RxHeaderTypeDef header, uint8_t data[]);
void clear_can_tx_buffer();
void clear_can_rx_buffer();
void move_can_tx_to_rx();
void inject_uart_rx_data(uint8_t *data, int size);
void clear_uart_rx_buffer();
void clear_uart_tx_buffer();
void inject_i2c_mem_data(uint16_t DevAddress, uint16_t MemAddress,uint8_t *data, uint16_t size);
void clear_i2c_mem_data();
void clear_spi_tx_buffer();
void clear_spi_rx_buffer();
void inject_spi_rx_data(uint8_t *data, int size);

//--- Getter Function Prototypes ---
int get_can_tx_buffer_count();
CAN_TxMessage_t get_can_tx_message(int pos);
int get_uart_tx_buffer_count();
uint8_t* get_uart_tx_buffer();
void set_current_free_mailboxes(int free_mailboxes);
void set_current_rx_fifo_fill_level(int rx_fifo_level);
void set_current_tick(uint32_t tick);
void init_uart_handle(UART_HandleTypeDef *huart);
int get_spi_tx_buffer_count();
uint8_t* get_spi_tx_buffer();
int get_spi_rx_buffer_count();
uint8_t* get_spi_rx_buffer();
void init_spi_handle(SPI_HandleTypeDef *hspi);
void copy_spi_tx_to_rx();
int get_i2c_buffer_count();
uint8_t* get_i2c_buffer();
// I2C Getters
uint16_t get_i2c_mem_buffer_dev_address();
uint16_t get_i2c_mem_buffer_mem_address();
uint16_t get_i2c_mem_buffer_count();

//--- GPIO Access Function Prototypes ---
GPIO_PinState get_gpio_pin_state(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void set_gpio_pin_state(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* MOCK_HAL_H */
#endif /* __x86_64__ */
