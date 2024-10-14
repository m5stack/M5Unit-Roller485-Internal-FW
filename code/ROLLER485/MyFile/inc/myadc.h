/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __MYADC_H__
#define __MYADC_H__
#include "stm32g4xx.h"
#include "arm_const_structs.h"

extern int32_t internal_temp;

void MyADCInit(void);
void MyAdcProcess(void);
void MyADCZeroCal(void);

uint16_t    MyAdcGetVal(uint8_t adc_hw,uint8_t channel);
int32_t     MyAdcGetCurrent(uint8_t channel);

#endif
