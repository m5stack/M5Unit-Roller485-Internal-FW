/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mysys.h"
#include "ws2812.h"
#include "flash.h"
#include "i2c_ex.h"
#include "motordriver.h"
#include "encoder.h"
#include "u8g2_disp_fun.h"
#include "myadc.h"
#include "smart_knob.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS 0x64
#define FIRMWARE_VERSION 1
#define APPLICATION_ADDRESS     ((uint32_t)0x08002000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buffer[100] = {0};
uint8_t act_flag = 0;
uint8_t last_act_flag = 0;
uint8_t flash_data[FLASH_DATA_SIZE] = {0};
uint8_t i2c_address[1] = {I2C_ADDRESS};
volatile uint8_t fm_version = FIRMWARE_VERSION;
uint16_t pos_readback;
uint8_t motor_disable_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);

	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

static uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  GXT_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (GXT_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}

//获取系统时间，单位us
uint32_t micros(void)
{
  return getCurrentMicros();
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;
    flash_data[0] = i2c_address[0];
    flash_data[1] = motor_mode;
    flash_data[2] = angle_cal_offset;
    flash_data[3] = (angle_cal_offset >> 8);
    flash_data[4] = motor_id;
    //speed p
    flash_data[5] = speed_pid_int[0];
    flash_data[6] = speed_pid_int[0] >> 8;
    flash_data[7] = speed_pid_int[0] >> 16;
    flash_data[8] = speed_pid_int[0] >> 24;
    //speed i
    flash_data[9] = speed_pid_int[1];
    flash_data[10] = speed_pid_int[1] >> 8;
    flash_data[11] = speed_pid_int[1] >> 16;
    flash_data[12] = speed_pid_int[1] >> 24;
    //speed d
    flash_data[13] = speed_pid_int[2];
    flash_data[14] = speed_pid_int[2] >> 8;
    flash_data[15] = speed_pid_int[2] >> 16;
    flash_data[16] = speed_pid_int[2] >> 24;
    //pos p
    flash_data[17] = pos_pid_int[0];
    flash_data[18] = pos_pid_int[0] >> 8;
    flash_data[19] = pos_pid_int[0] >> 16;
    flash_data[20] = pos_pid_int[0] >> 24;
    //pos i
    flash_data[21] = pos_pid_int[1];
    flash_data[22] = pos_pid_int[1] >> 8;
    flash_data[23] = pos_pid_int[1] >> 16;
    flash_data[24] = pos_pid_int[1] >> 24;
    //pos d
    flash_data[25] = pos_pid_int[2];
    flash_data[26] = pos_pid_int[2] >> 8;
    flash_data[27] = pos_pid_int[2] >> 16;
    flash_data[28] = pos_pid_int[2] >> 24;
    flash_data[29] = comm_type;
    flash_data[30] = speed_pid_index;
    flash_data[31] = pos_pid_index;
    flash_data[32] = bps_index;
    flash_data[33] = brightness_index;
    flash_data[34] = rgb_show_mode;
    flash_data[35] = motor_stall_protection_flag;
    flash_data[36] = motor_overvalue_protection_flag;
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    motor_mode = flash_data[1];
    angle_cal_offset = flash_data[2] | (flash_data[3] << 8);
    motor_id = flash_data[4];
    //speed p
    memcpy((uint8_t *)&speed_pid_int[0], (uint8_t *)&flash_data[5], 4);
    //speed i
    memcpy((uint8_t *)&speed_pid_int[1], (uint8_t *)&flash_data[9], 4);
    //speed d
    memcpy((uint8_t *)&speed_pid_int[2], (uint8_t *)&flash_data[13], 4);
    //pos p
    memcpy((uint8_t *)&pos_pid_int[0], (uint8_t *)&flash_data[17], 4);
    //pos i
    memcpy((uint8_t *)&pos_pid_int[1], (uint8_t *)&flash_data[21], 4);
    //pos d
    memcpy((uint8_t *)&pos_pid_int[2], (uint8_t *)&flash_data[25], 4);
    comm_type = flash_data[29];
    speed_pid_index = flash_data[30];
    pos_pid_index = flash_data[31];
    bps_index = flash_data[32];
    brightness_index = flash_data[33];
    rgb_show_mode = flash_data[34];
    motor_stall_protection_flag = flash_data[35];
    motor_overvalue_protection_flag = flash_data[36];
    MotorDriverSetAngleOffset(angle_cal_offset);
    for (int i = 0; i < 3; i+=2) {
      pos_pid_float[i] = (float)pos_pid_int[i] / 100000;
    }  
    pos_pid_float[1] = (float)pos_pid_int[1] / 10000000;
    for (int i = 0; i < 3; i+=2) {
      speed_pid_float[i] = (float)speed_pid_int[i] / 100000;
    }      
    speed_pid_float[1] = (float)speed_pid_int[1] / 10000000;
  }

  if (motor_mode == MODE_SPEED_ERR_PROTECT) {
    motor_mode = MODE_SPEED;
  }
  else if (motor_mode == MODE_POS_ERR_PROTECT) {
    motor_mode = MODE_POS;
  }    
}

void flash_data_write_back(void)
{
  if (motor_mode == MODE_SPEED_ERR_PROTECT) {
    motor_mode = MODE_SPEED;
  }
  else if (motor_mode == MODE_POS_ERR_PROTECT) {
    motor_mode = MODE_POS;
  }
    
  flash_data[0] = i2c_address[0];
  flash_data[1] = motor_mode;
  flash_data[2] = angle_cal_offset;
  flash_data[3] = (angle_cal_offset >> 8);
  flash_data[4] = motor_id;
  //speed p
  flash_data[5] = speed_pid_int[0];
  flash_data[6] = speed_pid_int[0] >> 8;
  flash_data[7] = speed_pid_int[0] >> 16;
  flash_data[8] = speed_pid_int[0] >> 24;
  //speed i
  flash_data[9] = speed_pid_int[1];
  flash_data[10] = speed_pid_int[1] >> 8;
  flash_data[11] = speed_pid_int[1] >> 16;
  flash_data[12] = speed_pid_int[1] >> 24;
  //speed d
  flash_data[13] = speed_pid_int[2];
  flash_data[14] = speed_pid_int[2] >> 8;
  flash_data[15] = speed_pid_int[2] >> 16;
  flash_data[16] = speed_pid_int[2] >> 24;
  //pos p
  flash_data[17] = pos_pid_int[0];
  flash_data[18] = pos_pid_int[0] >> 8;
  flash_data[19] = pos_pid_int[0] >> 16;
  flash_data[20] = pos_pid_int[0] >> 24;
  //pos i
  flash_data[21] = pos_pid_int[1];
  flash_data[22] = pos_pid_int[1] >> 8;
  flash_data[23] = pos_pid_int[1] >> 16;
  flash_data[24] = pos_pid_int[1] >> 24;
  //pos d
  flash_data[25] = pos_pid_int[2];
  flash_data[26] = pos_pid_int[2] >> 8;
  flash_data[27] = pos_pid_int[2] >> 16;
  flash_data[28] = pos_pid_int[2] >> 24;
  flash_data[29] = comm_type;
  flash_data[30] = speed_pid_index;
  flash_data[31] = pos_pid_index;
  flash_data[32] = bps_index;
  flash_data[33] = brightness_index;
  flash_data[34] = rgb_show_mode;
  flash_data[35] = motor_stall_protection_flag;
  flash_data[36] = motor_overvalue_protection_flag;
  writeMessageToFlash(flash_data , FLASH_DATA_SIZE);  
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len)
{
  uint8_t buf[4];
  uint8_t rx_buf[16];
  uint8_t tx_buf[16];
  uint8_t rx_mark[16] = {0};  

  if (len > 1) {
    if (rx_data[0] <= 0x0F) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x00+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x00+i] = 1;     
      } 

      if (rx_mark[0]) {
        motor_output = rx_data[1];
        if (motor_output) {
          if (!over_vol_flag && !err_stalled_flag && motor_mode < MODE_MAX) {
            if (motor_mode == MODE_DIAL && motor_disable_flag) {
              init_smart_knob();
            }          
            motor_disable_flag = 0;
            MotorDriverSetMode(MDRV_MODE_RUN);
          }
        }
        else {
          motor_disable_flag = 1;
          MotorDriverSetMode(MDRV_MODE_OFF);
        }        
      }

      if (rx_mark[1]) {
        if (rx_buf[1] && rx_buf[1] < MODE_MAX && !err_stalled_flag) {
          motor_mode = rx_buf[1];
          if (last_motor_mode != motor_mode) {
            if (motor_mode < MODE_DIAL) {
              MotorDriverSetCurrentReal(0);
              init_pid();
              pid_ctrl_speed_t.iTerm = 0;
              pid_ctrl_pos_t.iTerm = 0;
            }
            else if (motor_mode == MODE_DIAL) {
              init_smart_knob();
            }
            else if (motor_mode == MODE_POS_SPEED) {
              MotorDriverSetCurrentReal(0);
              PIDTuningsSet(&pid_ctrl_speed_t, speed_pid_plus_float[0], speed_pid_plus_float[1], speed_pid_plus_float[2]);
              PIDTuningsSet(&pid_ctrl_pos_t, pos_pid_plus_float[0], pos_pid_plus_float[1], pos_pid_plus_float[2]);
              pid_ctrl_speed_t.iTerm = 0;
              pid_ctrl_pos_t.iTerm = 0; 
            }
            last_motor_mode = motor_mode;
          }
        }
      }

      if (rx_mark[14]) {
        if (rx_buf[14]) {
          mode_switch_flag = 1;
        }
        else {
          mode_switch_flag = 0;
        }
      }

      if (rx_mark[10]) {
        if (rx_buf[10]) {
          motor_overvalue_protection_flag = 1;
        }
        else {
          motor_overvalue_protection_flag = 0;
        }
      }

      if (rx_mark[15]) {
        if (rx_buf[15]) {
          motor_stall_protection_flag = 1;
        }
        else {
          motor_stall_protection_flag = 0;
        }
      }
    }    
    else if (rx_data[0] == 0xF1) {
      if (rx_data[1]) {
        if (!over_vol_flag)
          MotorDriverSetMode(MDRV_MODE_ENC_CAL);
      }
    }
    else if (rx_data[0] == 0xF2) {
      if (rx_data[1]) {
        //set encoder offset 配置编码器偏移，此处实际使用需要校准之后开机均从flash读取
        angle_cal_offset = GetMotorDriverEncCalOffset();
        MotorDriverSetAngleOffset(angle_cal_offset);
        flash_data_write_back();                
      }
    }
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x40+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x40+i] = 1;     
      } 

      if (rx_mark[0]) {
        speed_point &= ~0x000000ff;
        speed_point |= rx_buf[0];
      }
      if (rx_mark[1]) {
        speed_point &= ~0x0000ff00;
        speed_point |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        speed_point &= ~0x00ff0000;
        speed_point |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        speed_point &= ~0xff000000;
        speed_point |= (rx_buf[3] << 24);
      }

      if (speed_point > MY_INT32_MAX)
        speed_point = MY_INT32_MAX;
      else if (speed_point < MY_INT32_MIN)
        speed_point = MY_INT32_MIN;
      
      pid_ctrl_speed_t.setpoint = (float)speed_point / 100.0f;
    }
    else if (rx_data[0] >= 0x80 && rx_data[0] <= 0x83) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x80+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x80+i] = 1;     
      } 

      if (rx_mark[0]) {
        pos_point &= ~0x000000ff;
        pos_point |= rx_buf[0];
      }
      if (rx_mark[1]) {
        pos_point &= ~0x0000ff00;
        pos_point |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        pos_point &= ~0x00ff0000;
        pos_point |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        pos_point &= ~0xff000000;
        pos_point |= (rx_buf[3] << 24);
      }

      if (pos_point > MY_INT32_MAX)
        pos_point = MY_INT32_MAX;
      else if (pos_point < MY_INT32_MIN)
        pos_point = MY_INT32_MIN;
      
      pid_ctrl_pos_t.setpoint = (float)pos_point / 100.0f;
    }
    else if (rx_data[0] >= 0xB0 && rx_data[0] <= 0xB3) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0xB0+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0xB0+i] = 1;     
      } 

      if (rx_mark[0]) {
        current_point &= ~0x000000ff;
        current_point |= rx_buf[0];
      }
      if (rx_mark[1]) {
        current_point &= ~0x0000ff00;
        current_point |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        current_point &= ~0x00ff0000;
        current_point |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        current_point &= ~0xff000000;
        current_point |= (rx_buf[3] << 24);
      }

      if (current_point > 120000)
        current_point = 120000;
      else if (current_point < -120000)
        current_point = -120000;

      float current_set = (float)current_point / 100.0f;

      MotorDriverSetCurrentReal(current_set);
    }
    else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x12) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x10+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x10+i] = 1;     
      } 
      if (rx_mark[0]) {
        motor_id = rx_buf[0];
      }           
      if (rx_mark[1]) {
       if (rx_buf[1] <= 2) {
         bps_index = rx_buf[1];
       }
      }           
      if (rx_mark[2]) {
       if (rx_buf[2] <= 100) {
        brightness_index = rx_buf[2];
        ws2812_show();
       }
      }           
    }
    else if (rx_data[0] >= 0x50 && rx_data[0] <= 0x53) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x50+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x50+i] = 1;     
      }

      if (rx_mark[0]) {
        max_speed_current &= ~0x000000ff;
        max_speed_current |= rx_buf[0];
      }
      if (rx_mark[1]) {
        max_speed_current &= ~0x0000ff00;
        max_speed_current |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        max_speed_current &= ~0x00ff0000;
        max_speed_current |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        max_speed_current &= ~0xff000000;
        max_speed_current |= (rx_buf[3] << 24);
      }

      if (max_speed_current < 0)
        max_speed_current = -max_speed_current;
      if (max_speed_current > 120000)
        max_speed_current = 120000;

      pid_ctrl_speed_t.outMin = -((float)max_speed_current / 100);
      pid_ctrl_speed_t.outMax = (float)max_speed_current / 100;
    }
    else if (rx_data[0] >= 0x20 && rx_data[0] <= 0x23) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x20+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x20+i] = 1;     
      }

      if (rx_mark[0]) {
        max_pos_current &= ~0x000000ff;
        max_pos_current |= rx_buf[0];
      }
      if (rx_mark[1]) {
        max_pos_current &= ~0x0000ff00;
        max_pos_current |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        max_pos_current &= ~0x00ff0000;
        max_pos_current |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        max_pos_current &= ~0xff000000;
        max_pos_current |= (rx_buf[3] << 24);
      }

      if (max_pos_current < 0)
        max_pos_current = -max_pos_current;
      if (max_pos_current > 120000)
        max_pos_current = 120000;

      pid_ctrl_pos_t.outMin = -((float)max_pos_current / 100);
      pid_ctrl_pos_t.outMax = (float)max_pos_current / 100;
    }
    else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x7B) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x70+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x70+i] = 1;     
      }

      if (rx_mark[0]) {
        speed_pid_int[0] &= ~0x000000ff;
        speed_pid_int[0] |= rx_buf[0];
      }
      if (rx_mark[1]) {
        speed_pid_int[0] &= ~0x0000ff00;
        speed_pid_int[0] |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        speed_pid_int[0] &= ~0x00ff0000;
        speed_pid_int[0] |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        speed_pid_int[0] &= ~0xff000000;
        speed_pid_int[0] |= (rx_buf[3] << 24);
      }

      if (rx_mark[4]) {
        speed_pid_int[1] &= ~0x000000ff;
        speed_pid_int[1] |= rx_buf[4];
      }
      if (rx_mark[5]) {
        speed_pid_int[1] &= ~0x0000ff00;
        speed_pid_int[1] |= (rx_buf[5] << 8);
      }
      if (rx_mark[6]) {
        speed_pid_int[1] &= ~0x00ff0000;
        speed_pid_int[1] |= (rx_buf[6] << 16);
      }
      if (rx_mark[7]) {
        speed_pid_int[1] &= ~0xff000000;
        speed_pid_int[1] |= (rx_buf[7] << 24);
      }

      if (rx_mark[8]) {
        speed_pid_int[2] &= ~0x000000ff;
        speed_pid_int[2] |= rx_buf[8];
      }
      if (rx_mark[9]) {
        speed_pid_int[2] &= ~0x0000ff00;
        speed_pid_int[2] |= (rx_buf[9] << 8);
      }
      if (rx_mark[10]) {
        speed_pid_int[2] &= ~0x00ff0000;
        speed_pid_int[2] |= (rx_buf[10] << 16);
      }
      if (rx_mark[11]) {
        speed_pid_int[2] &= ~0xff000000;
        speed_pid_int[2] |= (rx_buf[11] << 24);
      }

      for (int i = 0; i < 3; i+=2) {
        speed_pid_float[i] = (float)speed_pid_int[i] / 100000;
      }
      speed_pid_float[1] = (float)speed_pid_int[1] / 10000000;
      if (speed_pid_index == 0) {
        PIDTuningsSet(&pid_ctrl_speed_t, speed_pid_float[0], speed_pid_float[1], speed_pid_float[2]);
        pid_ctrl_speed_t.iTerm = 0;
      }
    }
    else if (rx_data[0] >= 0xA0 && rx_data[0] <= 0xAB) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0xA0+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0xA0+i] = 1;     
      }

      if (rx_mark[0]) {
        pos_pid_int[0] &= ~0x000000ff;
        pos_pid_int[0] |= rx_buf[0];
      }
      if (rx_mark[1]) {
        pos_pid_int[0] &= ~0x0000ff00;
        pos_pid_int[0] |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        pos_pid_int[0] &= ~0x00ff0000;
        pos_pid_int[0] |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        pos_pid_int[0] &= ~0xff000000;
        pos_pid_int[0] |= (rx_buf[3] << 24);
      }

      if (rx_mark[4]) {
        pos_pid_int[1] &= ~0x000000ff;
        pos_pid_int[1] |= rx_buf[4];
      }
      if (rx_mark[5]) {
        pos_pid_int[1] &= ~0x0000ff00;
        pos_pid_int[1] |= (rx_buf[5] << 8);
      }
      if (rx_mark[6]) {
        pos_pid_int[1] &= ~0x00ff0000;
        pos_pid_int[1] |= (rx_buf[6] << 16);
      }
      if (rx_mark[7]) {
        pos_pid_int[1] &= ~0xff000000;
        pos_pid_int[1] |= (rx_buf[7] << 24);
      }

      if (rx_mark[8]) {
        pos_pid_int[2] &= ~0x000000ff;
        pos_pid_int[2] |= rx_buf[8];
      }
      if (rx_mark[9]) {
        pos_pid_int[2] &= ~0x0000ff00;
        pos_pid_int[2] |= (rx_buf[9] << 8);
      }
      if (rx_mark[10]) {
        pos_pid_int[2] &= ~0x00ff0000;
        pos_pid_int[2] |= (rx_buf[10] << 16);
      }
      if (rx_mark[11]) {
        pos_pid_int[2] &= ~0xff000000;
        pos_pid_int[2] |= (rx_buf[11] << 24);
      }

      for (int i = 0; i < 3; i+=2) {
        pos_pid_float[i] = (float)pos_pid_int[i] / 100000;
      }
      pos_pid_float[1] = (float)pos_pid_int[1] / 10000000;
      if (pos_pid_index == 0) {
        PIDTuningsSet(&pid_ctrl_pos_t, pos_pid_float[0], pos_pid_float[1], pos_pid_float[2]);
        pid_ctrl_pos_t.iTerm = 0; 
      }      
    }
    else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x33))
    {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x30+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x30+i] = 1;     
      }
      if (rx_mark[0]) {
        rgb_color_buffer[rgb_color_buffer_index] &= ~0x000000ff;
        rgb_color_buffer[rgb_color_buffer_index] |= rx_buf[0];
      }
      if (rx_mark[1]) {
        rgb_color_buffer[rgb_color_buffer_index] &= ~0x0000ff00;
        rgb_color_buffer[rgb_color_buffer_index] |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        rgb_color_buffer[rgb_color_buffer_index] &= ~0x00ff0000;
        rgb_color_buffer[rgb_color_buffer_index] |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        if (rx_buf[3])
          rgb_show_mode = 1;
        else
          rgb_show_mode = 0;
      }   
      if (rx_mark[0] || rx_mark[1] || rx_mark[2]) {
        lastest_rgb_color = rgb_color_buffer[rgb_color_buffer_index];
        if (rgb_color_buffer_index < (RGB_BUFFER_SIZE-1))   
          ++rgb_color_buffer_index;
      }
    }
    else if (rx_data[0] >= 0x3C && rx_data[0] <= 0x3F) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x3C+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x3C+i] = 1;     
      }

      if (rx_mark[0]) {
        current_position &= ~0x000000ff;
        current_position |= rx_buf[0];
      }
      if (rx_mark[1]) {
        current_position &= ~0x0000ff00;
        current_position |= (rx_buf[1] << 8);
      }
      if (rx_mark[2]) {
        current_position &= ~0x00ff0000;
        current_position |= (rx_buf[2] << 16);
      }
      if (rx_mark[3]) {
        current_position &= ~0xff000000;
        current_position |= (rx_buf[3] << 24);
      }
    }
    else if (rx_data[0] == 0xF0) {
      if (rx_data[1] == 1) {
        flash_data_write_back();
      }
    }
    else if (rx_data[0] == 0xFF) {
      if (rx_data[1] < 128) {
        i2c_address[0] = rx_data[1];
        flash_data_write_back();
        user_i2c_init();
      }
    }
    else if (rx_data[0] == 0xFD)
    {
      if (rx_data[1] == 1) {
        LL_I2C_DeInit(I2C1);
        LL_I2C_DisableAutoEndMode(I2C1);
        LL_I2C_Disable(I2C1);
        LL_I2C_DisableIT_ADDR(I2C1);
        HAL_TIM_Base_DeInit(&htim1);
        HAL_TIM_Base_DeInit(&htim3);
        HAL_TIM_Base_MspDeInit(&htim1);
        HAL_TIM_PWM_MspDeInit(&htim3);
        HAL_SPI_DeInit(&hspi1);
        HAL_SPI_MspDeInit(&hspi1);
        HAL_ADC_DeInit(&hadc1);  
        HAL_ADC_MspDeInit(&hadc1);
        LL_USART_DeInit(USART3);
        LL_USART_DisableIT_IDLE(USART3);
          
        LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
        LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_2);
          
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        LL_USART_DisableDMAReq_RX(USART3);     
        HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);  
        HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);  
        HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);  
        HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);  
        HAL_ADC_Stop_DMA(&hadc1); 
        NVIC_SystemReset();
      }        
    }
  }
  else if (len == 1) {
    if (rx_data[0] == 0xF3) {
      uint8_t cal_status = IsMotorDriverEncCalBusy();
      i2c1_set_send_data(&cal_status, 1);
    }    
    else if (rx_data[0] <= 0x0F) {
      motor_output ? (motor_output = 1) : (motor_output = 0);
      uint8_t motor_mode_temp = motor_mode;
      if (motor_mode == MODE_SPEED_ERR_PROTECT) {
        motor_mode_temp = MODE_SPEED;
      }
      else if (motor_mode == MODE_POS_ERR_PROTECT) {
        motor_mode_temp = MODE_POS;
      }
      tx_buf[0] = motor_output;
      tx_buf[1] = motor_mode_temp;
      tx_buf[0x0A] = motor_overvalue_protection_flag;
      tx_buf[0x0C] = sys_status;
      tx_buf[0x0D] = error_code;
      tx_buf[0x0E] = mode_switch_flag;
      tx_buf[0x0F] = motor_stall_protection_flag;
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]], 0x0F-rx_data[0]+1);
    }
    else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x12) {
      tx_buf[0] = motor_id;
      tx_buf[1] = bps_index;
      tx_buf[2] = brightness_index;
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x10], 0x12-rx_data[0]+1);      
    }
    else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x3F))
    {
      int32_t vol_int32 = (int32_t)vol_lpf;
      memcpy(&tx_buf[0], (uint8_t *)&lastest_rgb_color, 3);
      tx_buf[3] = rgb_show_mode;
      memcpy(&tx_buf[4], (uint8_t *)&vol_int32, 4);
      memcpy(&tx_buf[8], (uint8_t *)&internal_temp, 4);
      memcpy(&tx_buf[12], (uint8_t *)&current_position, 4);
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x30], 0x3F-rx_data[0]+1);
    } 
    else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
      i2c1_set_send_data((uint8_t *)&speed_point, 4);
    } 
    else if (rx_data[0] >= 0x50 && rx_data[0] <= 0x53) {
      i2c1_set_send_data((uint8_t *)&max_speed_current, 4);
    }       
    else if (rx_data[0] >= 0x20 && rx_data[0] <= 0x23) {
      i2c1_set_send_data((uint8_t *)&max_pos_current, 4);
    }       
    else if (rx_data[0] >= 0x60 && rx_data[0] <= 0x63) {
      int32_t motor_rpm_int = 0;
      motor_rpm_int = motor_rpm * 100;
      i2c1_set_send_data((uint8_t *)&motor_rpm_int, 4);
    }
    else if (rx_data[0] >= 0x80 && rx_data[0] <= 0x83) {
      i2c1_set_send_data((uint8_t *)&pos_point, 4);
    }
    else if (rx_data[0] >= 0x90 && rx_data[0] <= 0x93) {
      int32_t mechanical_angle_int = mechanical_angle * 100;
      i2c1_set_send_data((uint8_t *)&mechanical_angle_int, 4);
    }
    else if (rx_data[0] >= 0xB0 && rx_data[0] <= 0xB3) {
      i2c1_set_send_data((uint8_t *)&current_point, 4);
    }    
    else if (rx_data[0] >= 0xC0 && rx_data[0] <= 0xC3) {
      int32_t ph_current_int = ph_crrent_lpf * 100;
      i2c1_set_send_data((uint8_t *)&ph_current_int, 4);
    } 
    else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x7B) {
      switch (speed_pid_index)
      {
      case 0:
        memcpy(tx_buf, (uint8_t *)&speed_pid_int[0], 12);
        break;
      case 1:
        memcpy(tx_buf, (uint8_t *)&speed_pid_low_int[0], 12);
        break;
      case 2:
        memcpy(tx_buf, (uint8_t *)&speed_pid_mid_int[0], 12);
        break;
      case 3:
        memcpy(tx_buf, (uint8_t *)&speed_pid_high_int[0], 12);
        break;
      
      default:
        break;
      }

      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0x70], 0x7B-rx_data[0]+1);  
    }       
    else if (rx_data[0] >= 0xA0 && rx_data[0] <= 0xAB) {
      switch (pos_pid_index) {
      case 0:
        memcpy(tx_buf, (uint8_t *)&pos_pid_int[0], 12);
        break;        
      case 1:
        memcpy(tx_buf, (uint8_t *)&pos_pid_low_int[0], 12);
        break;        
      case 2:
        memcpy(tx_buf, (uint8_t *)&pos_pid_mid_int[0], 12);
        break;        
      case 3:
        memcpy(tx_buf, (uint8_t *)&pos_pid_high_int[0], 12);
        break;        
      }
      
      i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]-0xA0], 0xAB-rx_data[0]+1);  
    }       
    else if (rx_data[0] == 0xFE) {
      i2c1_set_send_data((uint8_t *)&fm_version, 1);  
    }       
    else if (rx_data[0] == 0xFF) {
      i2c1_set_send_data((uint8_t *)&i2c_address[0], 1);  
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  // MX_USART3_UART_Init();
  MX_TIM3_Init();
  // MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	InitMysys();
  sk6812_init(PIXEL_MAX); 	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		LoopMysys();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
