/*

  oled_u8g2.c

  Monochrome minimal user interface: Glue code between mui and u8g2.

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2021, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/
#include "gpio.h"
#include "oled_u8g2.h"


#define OLED_RST_Clr() GPIOB->BRR=GPIO_PIN_11
#define OLED_RST_Set() GPIOB->BSRR=GPIO_PIN_11

#define OLED_DC_Clr() GPIOB->BRR=GPIO_PIN_14
#define OLED_DC_Set() GPIOB->BSRR=GPIO_PIN_14

#define OLED_SCLK_Clr() GPIOB->BRR=GPIO_PIN_13
#define OLED_SCLK_Set() GPIOB->BSRR=GPIO_PIN_13

#define OLED_SDIN_Clr() GPIOB->BRR=GPIO_PIN_15
#define OLED_SDIN_Set() GPIOB->BSRR=GPIO_PIN_15

u8g2_t u8g2;

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            //GPIO已经初始化
        break;
        case U8X8_MSG_GPIO_SPI_DATA:
            if(arg_int)OLED_SDIN_Set();
            else OLED_SDIN_Clr();
        break;
        case U8X8_MSG_GPIO_SPI_CLOCK:
            if(arg_int)OLED_SCLK_Set();
            else OLED_SCLK_Clr();
        break;        
        case U8X8_MSG_GPIO_CS:
            //CS默认接地
        case U8X8_MSG_GPIO_DC:
            if(arg_int)OLED_DC_Set();
            else OLED_DC_Clr();
        break;
        case U8X8_MSG_GPIO_RESET:
            if(arg_int)OLED_RST_Set();
            else OLED_RST_Clr();
        break;
        //Function which delays 100ns  
        case U8X8_MSG_DELAY_100NANO:  
            __NOP();  
        break;  
        case U8X8_MSG_DELAY_MILLI:
            HAL_Delay(arg_int);
        break;
        default:
            return 0;//A message was received which is not implemented, return 0 to indicate an error
  }
  return 1;
}

void u8g2Init(u8g2_t *u8g2)
{
/********************************************     
U8G2_R0     //不旋转，不镜像     
U8G2_R1     //旋转90度
U8G2_R2     //旋转180度   
U8G2_R3     //旋转270度
U8G2_MIRROR   //没有旋转，横向显示左右镜像
U8G2_MIRROR_VERTICAL    //没有旋转，竖向显示镜像
********************************************/
//    u8g2_Setup_sh1106_128x64_noname_2(u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);  // 初始化1.3寸OLED u8g2 结构体
	u8g2_Setup_ssd1306_64x48_er_f(u8g2, U8G2_R2, u8x8_byte_4wire_sw_spi, u8x8_stm32_gpio_and_delay);  // 初始化0.96寸OLED u8g2 结构体
	u8g2_InitDisplay(u8g2);     //初始化显示
	u8g2_SetPowerSave(u8g2, 0); //开启显示
}