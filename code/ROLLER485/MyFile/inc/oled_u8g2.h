#ifndef __OLED_U8G2_H_
#define __OLED_U8G2_H_

#include "u8g2.h"
#include "u8x8.h"

extern u8g2_t u8g2;

void OLED_U8G2_Init(void);
void u8g2Init(u8g2_t *u8g2);
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
    U8X8_UNUSED void *arg_ptr);

#endif