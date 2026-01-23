/*
 * cpphal.h
 *
 *  Created on: Sep 14, 2025
 *      Author: mgi
 */

#ifndef INC_CPPHAL_H_
#define INC_CPPHAL_H_

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern RTC_HandleTypeDef hrtc;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#endif /* INC_CPPHAL_H_ */
