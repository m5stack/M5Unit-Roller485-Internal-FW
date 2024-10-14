/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __MYSYS_H__
#define __MYSYS_H__
#include "stm32g4xx.h"
#include "pid_controller.h"

#define RGB_BUFFER_SIZE 10

enum {SYS_STANDBY = 0, SYS_RUNNING, SYS_ERROR};
enum {DIS_INFO = 0, DIS_GRAPHY, DIS_CHAR, DIS_PID, DIS_MAX, DIS_OVP, DIS_STALL, DIS_OVER_VALUE};
enum {MODE_SPEED = 1, MODE_POS, MODE_CURRENT, MODE_DIAL, MODE_POS_SPEED, MODE_MAX, MODE_SPEED_ERR_PROTECT, MODE_POS_ERR_PROTECT};
enum {ERR_NONE = 0,
      ERR_OVER_VOLTAGE = 1 << 0,
      ERR_STALLED = 1 << 1,
      ERR_OVER_VALUE = 1 << 2};
extern uint8_t sys_status;
extern uint8_t running_index;
extern uint8_t dis_show_flag;
extern uint8_t last_dis_show_flag;
extern uint8_t motor_mode;
extern uint8_t last_motor_mode;
extern uint8_t motor_id;
extern uint16_t angle_cal_offset;
extern uint8_t motor_output;
extern int32_t speed_point;
extern int32_t pos_point;
extern int32_t current_point;
extern volatile uint32_t usart_tx_delay;
extern float ph_current_rt,ph_crrent_lpf;
extern float motor_rpm;
extern float motor_rps;
extern float speed_pid_float[3];
extern float pos_pid_float[3];
extern uint32_t speed_pid_low_int[3];
extern uint32_t speed_pid_mid_int[3];
extern uint32_t speed_pid_high_int[3];
extern float speed_pid_low_float[3];
extern float speed_pid_mid_float[3];
extern float speed_pid_high_float[3];
extern uint32_t pos_pid_low_int[3];
extern uint32_t pos_pid_mid_int[3];
extern uint32_t pos_pid_high_int[3];
extern float pos_pid_low_float[3];
extern float pos_pid_mid_float[3];
extern float pos_pid_high_float[3];
extern uint32_t speed_pid_plus_int[3];
extern float speed_pid_plus_float[3];
extern uint32_t pos_pid_plus_int[3];
extern float pos_pid_plus_float[3];
extern uint8_t speed_pid_index;
extern uint8_t pos_pid_index;
extern PIDControl pid_ctrl_speed_t;
extern PIDControl pid_ctrl_pos_t;
extern int32_t max_speed_current;
extern int32_t max_pos_current;
extern float mechanical_angle,mechanical_turns,last_mechanical_turns;
extern float mechanical_rad;
extern uint32_t speed_pid_int[3];
extern uint32_t pos_pid_int[3];
extern float vol_input,vol_lpf;
extern uint8_t error_code;
extern uint8_t over_vol_protect_mode;
extern uint8_t over_vol_flag;
extern uint8_t over_value_flag;
extern uint8_t over_vol_protect_auto_flag;
extern uint32_t over_vol_protect_auto_counter;
extern uint16_t speed_err_value;
extern uint8_t speed_err_timeout;
extern uint8_t speed_err_count_flag;
extern uint8_t speed_err_auto_flag;
extern uint32_t speed_err_recover_try_counter;
extern uint8_t err_recover_try_max;
extern uint8_t err_stalled_flag;
extern uint16_t pos_err_value;
extern uint8_t pos_err_timeout;
extern uint8_t pos_err_count_flag;
extern uint8_t pos_err_auto_flag;
extern uint32_t pos_err_counter;
extern uint32_t pos_err_auto_counter;
extern uint32_t pos_err_recover_counter;
extern uint32_t pos_err_recover_try_counter;
extern uint8_t mode_switch_flag;
extern uint32_t bps_list[3];
extern uint8_t bps_index;
extern float brightness_list[4];
extern uint8_t brightness_index;
extern uint8_t rgb_show_mode;
extern uint32_t rgb_color_buffer[RGB_BUFFER_SIZE];
extern uint32_t rgb_color_buffer_index;
extern uint32_t lastest_rgb_color;
extern uint8_t motor_stall_protection_flag;
extern uint8_t motor_overvalue_protection_flag;

void InitMysys(void);
void LoopMysys(void);
uint8_t crc8_MAXIM(uint8_t *data, uint8_t len);
void init_pid(void);

#endif
