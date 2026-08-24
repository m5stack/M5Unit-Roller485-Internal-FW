/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __DISPLAY_APP_H__
#define __DISPLAY_APP_H__

#include <stdint.h>

enum { COMM_TYPE_NONE = 0, COMM_TYPE_I2C, COMM_TYPE_485, COMM_TYPE_485_I2C, COMM_TYPE_MAX };
enum { DIS_INFO = 0, DIS_GRAPHY, DIS_CHAR, DIS_PID, DIS_MAX, DIS_OVP, DIS_STALL, DIS_OVER_VALUE };

extern uint8_t comm_type;
extern uint8_t dis_show_flag;
extern uint8_t last_dis_show_flag;
extern uint32_t rgb_flash_slow;

void display_app_init(void);
void display_app_show_startup(void);
void display_app_update_input(void);
void display_app_update_page(void);
void display_app_render(void);

#endif
