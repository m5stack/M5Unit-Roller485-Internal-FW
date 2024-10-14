/**
  ******************************************************************************
  * File Name          : ws2812.c
  * Description        : This file provides code for the configuration
  *                      of ws2812.
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

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include <stdlib.h>

/* USER CODE BEGIN 0 */
#include "tim.h"
#include "mysys.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
uint32_t *color_buf = NULL;
uint8_t rled[PIXEL_MAX] = {0};
uint8_t gled[PIXEL_MAX] = {0};
uint8_t bled[PIXEL_MAX] = {0};

frame_buf frame = { 
	.head[0] = 0,
	.head[1] = 0,
	.head[2] = 0,
	.tail = 0,
};

void sk6812_init(uint8_t num) {
  color_buf = (uint32_t *)calloc(num, sizeof(uint32_t));
}

void neopixel_set_color(uint8_t num, uint32_t color) {
	rled[num] = ((color >> 16) & 0xff) * ((float)brightness_index / 100.0f);
	gled[num] = ((color >> 8) & 0xff) * ((float)brightness_index / 100.0f);
	bled[num] = (color & 0xff) * ((float)brightness_index / 100.0f);
	color_buf[num] = color;
}

void ws2812_show(void)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
	uint8_t i, j;

	for(i = 0; i < PIXEL_MAX; i++) {
		for(j = 0; j < 8; j++) {														// G->R->B
			 frame.data[24 * i + j] = (gled[i] & (0x80 >> j)) ? BIT_1 : BIT_0; 			// 将高低位扩展到16bit
			 frame.data[24 * i + j + 8]   = (rled[i] & (0x80 >> j)) ? BIT_1 : BIT_0;	
			 frame.data[24 * i + j + 16]  = (bled[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
		}
	}

	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_2, (uint32_t *)&frame, 3 + 24 * PIXEL_MAX + 1);
}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
