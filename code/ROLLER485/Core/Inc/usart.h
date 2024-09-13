/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define UART1_TX_BUF_SIZE	1*50
#define UART1_RX_BUF_SIZE	1*512

typedef struct
{
    uint8_t  *pTxBuf; 
    uint8_t  *pRxBuf; 
    uint16_t LEN; 
    uint8_t  FLG; 
}UART_DAT;

extern UART_DAT dat_Uart1;
extern uint8_t usart_tx_flag;
extern uint8_t usart_fault_flag;
/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void user_usart3_uart_init(void);
void hard_uart_begin(void);
void UART_DMA_Send(USART_TypeDef *USARTx,uint8_t *str,uint16_t size);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

