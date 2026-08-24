/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "encoder.h"
#include "motordriver.h"

uint8_t speed_encoder_init_flag = 0;

encoder_value_typedef speed_encoder_value_t;

void speed_encoder_update(void)
{
    if (!speed_encoder_init_flag) {
        speed_encoder_value_t.last_feedback_position = angle_corrected;
        speed_encoder_value_t.encoder_value          = 0;
        speed_encoder_init_flag                      = 1;
    }

    if (angle_corrected < speed_encoder_value_t.last_feedback_position &&
        speed_encoder_value_t.last_feedback_position > 12384 && angle_corrected < 4000) {
        speed_encoder_value_t.encoder_value += (16383 - speed_encoder_value_t.last_feedback_position);
        speed_encoder_value_t.encoder_value += angle_corrected;
        speed_encoder_value_t.last_feedback_position = angle_corrected;
    } else if (angle_corrected > speed_encoder_value_t.last_feedback_position &&
               speed_encoder_value_t.last_feedback_position < 4000 && angle_corrected > 12384) {
        speed_encoder_value_t.encoder_value -= (16383 - angle_corrected);
        speed_encoder_value_t.encoder_value -= speed_encoder_value_t.last_feedback_position;
        speed_encoder_value_t.last_feedback_position = angle_corrected;
    } else if (angle_corrected != speed_encoder_value_t.last_feedback_position) {
        speed_encoder_value_t.encoder_value += (angle_corrected - speed_encoder_value_t.last_feedback_position);
        speed_encoder_value_t.last_feedback_position = angle_corrected;
    }
}
