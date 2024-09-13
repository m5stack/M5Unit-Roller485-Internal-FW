#ifndef __U8G2_DISP_FUN_H_
#define __U8G2_DISP_FUN_H_

#include "oled_u8g2.h"
#include "mysys.h"
#include <stdio.h>
#include "encoder.h"
#include "button.h"

enum {COMM_TYPE_NONE = 0, COMM_TYPE_I2C, COMM_TYPE_485, COMM_TYPE_485_I2C, COMM_TYPE_MAX};

extern uint8_t is_menu_flag;
extern uint8_t comm_flash_count;
extern uint8_t comm_type;
extern uint32_t rgb_color;
extern uint32_t rgb_flash_slow;
extern uint32_t rgb_flash_flag;

void u8g2_disp_init(void);
void u8g2_disp_update_status(void);
void u8g2_disp_update_vin(float vin);
void u8g2_disp_menu_init(void);
void u8g2_disp_menu_update(void);
void u8g2_disp_menu_0_1(void);
void u8g2_disp_menu_0_2(void);
void u8g2_disp_menu_0_3(void);
void u8g2_disp_menu_1_1(void);
void u8g2_disp_menu_0_4(void);
void u8g2_disp_menu_2_1(void);
void u8g2_disp_menu_3_1(void);
void u8g2_disp_menu_0_5(void);
void Drawgauge(int x, int y, int r, int p, int v, int minVal, int maxVal);
void DrawPos(int x, int y, int r, int p, int v, int minVal, int maxVal);
void DrawCurrent(int x, int y, int r, int p, int v, int minVal, int maxVal);
void u8g2_disp_pos(void);
void u8g2_disp_speed(void);
void u8g2_disp_current(void);
void u8g2_disp_all(void);
void u8g2_disp_update_comm(void);
void ws2812_flash(void);
void u8g2_disp_update_page(void);
void u8g2_disp_update_mode(void);
void u8g2_disp_char(void);
void u8g2_disp_info(void);
void u8g2_disp_pid(void);
void u8g2_disp_ovp(void);
void u8g2_disp_stall(void);
void u8g2_disp_over_value(void);
void u8g2_disp_menu_4_1(void);
void u8g2_disp_menu_0_6(void);
void u8g2_disp_menu_0_7(void);
void u8g2_disp_menu_0_8(void);
void u8g2_disp_menu_5_1(void);
void u8g2_disp_menu_6_1(void);
void u8g2_disp_menu_7_1(void);
void u8g2_disp_menu_8_1(void);
void u8g2_disp_menu_0_9(void);
void u8g2_disp_menu_0_10(void);
void u8g2_disp_menu_0_11(void);
void u8g2_disp_menu_9_1(void);
void u8g2_disp_menu_10_1(void);

#endif