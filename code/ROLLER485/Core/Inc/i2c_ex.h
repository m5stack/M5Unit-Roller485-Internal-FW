/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_ex_H
#define __i2c_ex_H
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern volatile uint32_t i2c_timeout_counter;
extern volatile uint32_t i2c_stop_timeout_flag;
extern volatile uint32_t i2c_stop_timeout_counter;

extern void i2c1_it_enable(void);
extern void i2c1_it_disable(void);
extern void i2c1_set_send_data(uint8_t *tx_ptr, uint16_t len);
extern void set_i2c_slave_address(uint8_t addr);
#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
