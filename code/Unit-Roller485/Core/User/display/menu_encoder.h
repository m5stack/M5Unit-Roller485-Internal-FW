/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __MENU_ENCODER_H__
#define __MENU_ENCODER_H__

#include "stm32g4xx.h"

typedef struct {
    int32_t encoder_value;
    int32_t last_encoder_value;
    int32_t encoder_up_down_value;
    int32_t last_encoder_up_down_value;
    uint8_t encoder_down;
    uint8_t encoder_up;
    uint16_t last_feedback_position;
} menu_encoder_value_typedef;

extern menu_encoder_value_typedef encoder_value_t;

void menu_encoder_update(void);

#endif
