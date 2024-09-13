/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#include "i2c_ex.h"
/* USER CODE END 0 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PA15   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_SYSCFG_EnableFastModePlus(LL_SYSCFG_I2C_FASTMODEPLUS_PB7);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x60505F8C;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0x64<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */
  set_i2c_slave_address(0x64);
  /* USER CODE END I2C1_Init 2 */

}

/* USER CODE BEGIN 1 */
void user_i2c_init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PA15   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_SYSCFG_EnableFastModePlus(LL_SYSCFG_I2C_FASTMODEPLUS_PB7);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x60505F8C;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = i2c_address[0]<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */
  set_i2c_slave_address(i2c_address[0]);
  /* USER CODE END I2C1_Init 2 */

}

void I2C1_Start(void)
{
  LL_I2C_Enable(I2C1);
}

void I2C1_ClearCR(void)
{
  (I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));
}

uint8_t I2C1_TransmitData(uint8_t device_id ,uint8_t *pdata, uint8_t size, uint16_t timeout)
{
  // busyフラグをチェック
  while(LL_I2C_IsActiveFlag_BUSY(I2C1) == SET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }       
  }

  LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  for(uint8_t i = 0; i < size; i++){
    while(LL_I2C_IsActiveFlag_TXE(I2C1)== RESET) {
        if(LL_SYSTICK_IsActiveCounterFlag())
        {
            if (timeout-- == 0)
            {
                I2C1_ClearCR();
                return 1;
            }
        }         
    }
    LL_I2C_TransmitData8(I2C1, *pdata++);
  }

  while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }       
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  
  // 設定をリセットする
  I2C1_ClearCR();

  return 0;
}

uint8_t I2C1_TransmitData_RepeatedStart(uint8_t device_id ,uint8_t *pdata, uint8_t size, uint16_t timeout)
{
  // busyフラグをチェック
  while(LL_I2C_IsActiveFlag_BUSY(I2C1) == SET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }
  }

  LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

  for(uint8_t i = 0; i < size; i++){
    while(LL_I2C_IsActiveFlag_TXE(I2C1)== RESET) {
        if(LL_SYSTICK_IsActiveCounterFlag())
        {
            if (timeout-- == 0)
            {
                I2C1_ClearCR();
                return 1;
            }
        }        
    }
    LL_I2C_TransmitData8(I2C1, *pdata++);
  }
  while(LL_I2C_IsActiveFlag_TC(I2C1)==RESET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }
  }
  
  return 0;
}

uint8_t I2C1_ReceiveData(uint8_t device_id , uint8_t *pdata, uint8_t size, uint8_t timeout)
{
  // 初期設定 
  LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

  for(uint8_t i = 0; i < size; i++){
    while(LL_I2C_IsActiveFlag_RXNE(I2C1) == RESET) {
        if(LL_SYSTICK_IsActiveCounterFlag())
        {
            if (timeout-- == 0)
            {
                I2C1_ClearCR();
                return 1;
            }
        }        
    }
    *pdata++ = LL_I2C_ReceiveData8(I2C1);
  }
  
  while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }      
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  
  // 設定をリセットする
  I2C1_ClearCR();

  return 0;
}

uint8_t I2C_Write_Bytes(uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *pData, uint8_t TxLen,uint16_t Timeout)
{
    uint8_t tx_buffer[TxLen+1];

    tx_buffer[0] = MemAddress;
    memcpy(&tx_buffer[1], pData, TxLen);

    return I2C1_TransmitData(DeviceAddress, tx_buffer, TxLen+1, Timeout);	
}

uint8_t I2C_Write_16bits_reg_Bytes(uint8_t DeviceAddress, uint16_t MemAddress, uint8_t *pData, uint8_t TxLen,uint16_t Timeout)
{
    uint8_t tx_buffer[TxLen+2];

    tx_buffer[0] = MemAddress;
    tx_buffer[1] = ((MemAddress >> 8) & 0xff);
    memcpy(&tx_buffer[2], pData, TxLen);

    return I2C1_TransmitData(DeviceAddress, tx_buffer, TxLen+1, Timeout);	
}

uint8_t I2C_Read_Bytes(uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *pData, uint8_t RxLen,uint16_t Timeout)
{
    if (!I2C1_TransmitData_RepeatedStart(DeviceAddress, &MemAddress, 1, Timeout)) {
        return I2C1_ReceiveData(DeviceAddress ,pData, RxLen, Timeout);

    }
    else {
        return 1;
    }

}
uint8_t I2C_Read_16bits_reg_Bytes(uint8_t DeviceAddress, uint16_t MemAddress, uint8_t *pData, uint8_t RxLen,uint16_t Timeout)
{
    if (!I2C1_TransmitData_RepeatedStart(DeviceAddress, &MemAddress, 2, Timeout)) {
        return I2C1_ReceiveData(DeviceAddress ,pData, RxLen, Timeout);

    }
    else {
        return 1;
    }
}
/* USER CODE END 1 */
