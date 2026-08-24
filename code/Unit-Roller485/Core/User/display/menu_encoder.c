/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "menu_encoder.h"

#include "motordriver.h"

static uint8_t menu_encoder_enable_flag = 1;
static uint8_t menu_encoder_init_flag   = 0;

menu_encoder_value_typedef encoder_value_t;

void menu_encoder_update(void)
{
    if (menu_encoder_enable_flag) {
        if (angle_corrected < encoder_value_t.last_feedback_position &&
            encoder_value_t.last_feedback_position > 12384 && angle_corrected < 4000) {
            encoder_value_t.encoder_value += (16383 - encoder_value_t.last_feedback_position);
            encoder_value_t.encoder_value += angle_corrected;
            encoder_value_t.last_feedback_position = angle_corrected;
        } else if (angle_corrected > encoder_value_t.last_feedback_position &&
                   encoder_value_t.last_feedback_position < 4000 && angle_corrected > 12384) {
            encoder_value_t.encoder_value -= (16383 - angle_corrected);
            encoder_value_t.encoder_value -= encoder_value_t.last_feedback_position;
            encoder_value_t.last_feedback_position = angle_corrected;
        } else if (angle_corrected != encoder_value_t.last_feedback_position) {
            encoder_value_t.encoder_value += (angle_corrected - encoder_value_t.last_feedback_position);
            encoder_value_t.last_feedback_position = angle_corrected;
        }

        encoder_value_t.encoder_up_down_value = encoder_value_t.encoder_value / 200;
        if (encoder_value_t.last_encoder_up_down_value != encoder_value_t.encoder_up_down_value) {
            if (encoder_value_t.encoder_up_down_value - encoder_value_t.last_encoder_up_down_value > 6) {
                encoder_value_t.encoder_down               = 1;
                encoder_value_t.last_encoder_up_down_value = encoder_value_t.encoder_up_down_value;
            } else if (encoder_value_t.encoder_up_down_value - encoder_value_t.last_encoder_up_down_value < -6) {
                encoder_value_t.encoder_up                 = 1;
                encoder_value_t.last_encoder_up_down_value = encoder_value_t.encoder_up_down_value;
            }
        }

        if (!menu_encoder_init_flag) {
            encoder_value_t.last_feedback_position = angle_corrected;
            encoder_value_t.encoder_value          = 0;
            encoder_value_t.encoder_down           = 0;
            encoder_value_t.encoder_up             = 0;
            menu_encoder_init_flag                 = 1;
        }
    }
}
