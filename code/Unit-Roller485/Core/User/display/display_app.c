/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "display_app.h"

#include "button.h"
#include "main.h"
#include "mysys.h"
#include "oled_u8g2.h"
#include "smart_knob.h"
#include "u8g2_disp_fun.h"

uint8_t dis_show_flag      = DIS_INFO;
uint8_t last_dis_show_flag = DIS_INFO;

static uint16_t disp_ph_current = 0;
static uint32_t act_delay       = 0;
static uint32_t status_flag     = 0;

void display_app_init(void)
{
    u8g2Init(&u8g2);
    if (!HAL_GPIO_ReadPin(SYS_SW_GPIO_Port, SYS_SW_Pin)) {
        u8g2_disp_menu_init();
        u8g2_disp_menu_update();
    }
}

void display_app_show_startup(void)
{
    u8g2_disp_init();
}

void display_app_update_input(void)
{
    button_update();
    u8g2_disp_update_mode();
    // u8g2_disp_update_status();
    u8g2_disp_update_page();
    u8g2_disp_update_comm();
}

void display_app_update_page(void)
{
    if (my_button.was_click) {
        dis_show_flag++;
        if (dis_show_flag >= DIS_MAX) {
            dis_show_flag = DIS_INFO;
        }
        last_dis_show_flag  = dis_show_flag;
        my_button.was_click = 0;
    }

    if (my_button.is_longlongpressed) {
        if (mode_switch_flag) {
            motor_mode++;
            if (motor_mode >= MODE_POS_SPEED) {
                motor_mode = MODE_SPEED;
            }
            if (motor_mode == MODE_DIAL) {
                init_smart_knob();
            }
            my_button.is_longlongpressed = 0;
        }
    }

    if (ph_crrent_lpf < 0) {
        disp_ph_current = (uint16_t)(-ph_crrent_lpf);
    } else {
        disp_ph_current = (uint16_t)ph_crrent_lpf;
    }

    if (over_vol_flag) {
        dis_show_flag = DIS_OVP;
    } else if (err_stalled_flag) {
        dis_show_flag = DIS_STALL;
    } else if (over_value_flag) {
        dis_show_flag = DIS_OVER_VALUE;
    } else {
        dis_show_flag = last_dis_show_flag;
    }
}

void display_app_render(void)
{
    switch (dis_show_flag) {
        case DIS_CHAR:
            u8g2_disp_char();
            break;
        case DIS_GRAPHY:
            u8g2_disp_all();
            break;
        case DIS_INFO:
            u8g2_disp_info();
            break;
        case DIS_PID:
            u8g2_disp_pid();
            break;
        case DIS_OVP:
            u8g2_disp_ovp();
            break;
        case DIS_STALL:
            u8g2_disp_stall();
            break;
        case DIS_OVER_VALUE:
            u8g2_disp_over_value();
            break;
        default:
            break;
    }

    ws2812_flash();

    if (act_delay < HAL_GetTick()) {
        running_index++;
        if (running_index > 1) {
            running_index = 0;
        }
        if (status_flag) {
            status_flag = 0;
            // OLED_ShowString(0,0," M5 BLDC  ",8,1);
        } else {
            status_flag = 1;
            // OLED_ShowString(0,0," M5 BLDC *",8,1);
        }
        act_delay = HAL_GetTick() + 1000;
    }
}
