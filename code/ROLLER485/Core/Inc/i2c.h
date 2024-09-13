/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define BLDC_ADDRESS                         (0x64)    // 1000000 (A0+A1=GND)
#define BLDC_WRITE_ADDRESS                   (BLDC_ADDRESS << 1)    // 1000000 (A0+A1=GND)
#define BLDC_READ_ADDRESS                    ((BLDC_ADDRESS << 1) | 1)    // 1000000 (A0+A1=GND)
#define I2C_TIMOUT_MS     10
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
void I2C1_Start(void);
uint8_t I2C_Write_Bytes(uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *pData, uint8_t TxLen,uint16_t Timeout);
uint8_t I2C_Read_Bytes(uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *pData, uint8_t RxLen,uint16_t Timeout);
uint8_t I2C_Read_16bits_reg_Bytes(uint8_t DeviceAddress, uint16_t MemAddress, uint8_t *pData, uint8_t RxLen,uint16_t Timeout);
uint8_t I2C_Write_16bits_reg_Bytes(uint8_t DeviceAddress, uint16_t MemAddress, uint8_t *pData, uint8_t TxLen,uint16_t Timeout);
uint8_t I2C1_ReceiveData(uint8_t device_id , uint8_t *pdata, uint8_t size, uint8_t timeout);
uint8_t I2C1_TransmitData(uint8_t device_id ,uint8_t *pdata, uint8_t size, uint16_t timeout);
uint8_t I2C1_TransmitData_RepeatedStart(uint8_t device_id ,uint8_t *pdata, uint8_t size, uint16_t timeout);
void user_i2c_init(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

