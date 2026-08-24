
/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "mysys.h"
#include "motordriver.h"
#include "myadc.h"
#include "tim.h"
#include "tle5012b.h"

#include "arm_const_structs.h"

#include "display_app.h"
#include "encoder.h"
#include "rgb.h"
#include "i2c.h"
#include "i2c_ex.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pid_controller.h"
#include "smart_knob.h"
#include "arm_math.h"
#include "i2c.h"

#define MAX_STALLED_CURRENT (500)

uint16_t counter_loop_foc, counter_loop_control;

uint16_t pid_compute_counter = 0;

float32_t vol_input, vol_lpf;

float32_t ph_current_rt, ph_crrent_lpf;

float32_t encoder_absolute_angle_new, encoder_absolute_angle_old;
float mechanical_angle           = 0.0f;
int32_t mechanical_turns         = 0;
float mechanical_rad             = 0.0f;
float mechanical_delta_rad       = 0.0f;
float mechanical_delta_rad_accum = 0.0f;
int32_t pos_point_turns          = 0;
float pos_point_angle            = 0.0f;
int32_t diff_encoder_value       = 0;
float32_t diff_encoder_value_lpf = 0.0f;
float32_t pos_output_lpf         = 0.0f;
float32_t motor_rpm;
float32_t motor_rps;
float32_t angle_target, angle_error;

float32_t angle_kp;
float32_t uq_limit;

uint8_t speed_pid_index   = 0;
uint32_t speed_pid_int[3] = {1500000, 1000, 40000000};
float speed_pid_float[3]  = {15.0f, 0.0001f, 400.0f};

uint32_t speed_pid_low_int[3]  = {1500000, 1000, 40000000};
float speed_pid_low_float[3]   = {15.0f, 0.0001f, 400.0f};
uint32_t speed_pid_mid_int[3]  = {2500000, 30, 20000000};
float speed_pid_mid_float[3]   = {25.0f, 0.000003f, 200.0f};
uint32_t speed_pid_high_int[3] = {2500000, 30, 40000000};
float speed_pid_high_float[3]  = {25.0f, 0.000003f, 400.0f};

uint8_t pos_pid_index   = 0;
uint32_t pos_pid_int[3] = {1500000, 30, 40000000};
float pos_pid_float[3]  = {15.0f, 0.000003f, 400.0f};

uint32_t pos_pid_low_int[3]  = {1500000, 30, 40000000};
float pos_pid_low_float[3]   = {15.0f, 0.000003f, 400.0f};
uint32_t pos_pid_mid_int[3]  = {1500000, 1, 400000000};
float pos_pid_mid_float[3]   = {15.0f, 0.0000001f, 4000.0f};
uint32_t pos_pid_high_int[3] = {1500000, 1, 800000000};
float pos_pid_high_float[3]  = {15.0f, 0.0000001f, 8000.0f};

float speed_pid_plus_float[3] = {15.0f, 0.0001f, 400.0f};
float pos_pid_plus_float[3]   = {12.0f, 0.01f, 2000.0f};

PIDControl pid_ctrl_speed_t;
PIDControl pid_ctrl_pos_t;
int32_t speed_point       = 0;
int32_t max_speed_current = 100000;
int32_t max_pos_current   = 100000;
int32_t pos_point         = 0;
int32_t current_point     = 0;
float current_point_float = 0.0f;

uint8_t error_code                     = ERR_NONE;
uint8_t over_vol_protect_mode          = 0;
uint8_t over_vol_protect_auto_flag     = 0;
uint32_t over_vol_protect_auto_counter = 0;
uint8_t over_vol_flag                  = 0;

uint8_t over_value_flag = 0;

uint8_t err_recover_try_max            = 5;
uint8_t err_stalled_flag               = 0;
uint16_t speed_err_value               = 0;
uint8_t speed_err_timeout              = 0;
uint8_t speed_err_count_flag           = 0;
uint8_t speed_err_auto_flag            = 0;
uint32_t speed_err_counter             = 0;
uint32_t speed_err_auto_counter        = 0;
uint32_t speed_err_recover_counter     = 0;
uint32_t speed_err_recover_try_counter = 0;

float speed_err_rate = 0.8f;
float pos_err_rate   = 0.4f;

uint16_t pos_err_value               = 0;
uint8_t pos_err_timeout              = 0;
uint8_t pos_err_count_flag           = 0;
uint8_t pos_err_auto_flag            = 0;
uint32_t pos_err_counter             = 0;
uint32_t pos_err_auto_counter        = 0;
uint32_t pos_err_recover_counter     = 0;
uint32_t pos_err_recover_try_counter = 0;

uint8_t sys_status    = SYS_STANDBY;
uint8_t running_index = 0;

volatile uint32_t usart_tx_delay = 0;
uint8_t usart_tx_flag            = 0;

uint8_t motor_mode      = MODE_SPEED;
uint8_t last_motor_mode = MODE_SPEED;

uint8_t motor_id = 0;

uint16_t angle_cal_offset = 0;

uint8_t motor_output = 0;

uint8_t mode_switch_flag                = 0;
uint8_t motor_stall_protection_flag     = 1;
uint8_t motor_overvalue_protection_flag = 0;

float rpm_rps_count_temp = 0;

uint32_t bps_list[3]     = {115200, 19200, 9600};
uint8_t bps_index        = 0;
uint8_t brightness_index = 100;

uint8_t rgb_show_mode                      = 0;
uint32_t rgb_color_buffer[RGB_BUFFER_SIZE] = {0};
uint32_t rgb_color_buffer_index            = 0;
uint32_t lastest_rgb_color                 = 0;

void Rpm_Count_100us(void);

void _sys_exit(int x)
{
    x = x;
}

static uint16_t position_angle_to_cdeg(float angle_deg)
{
    long angle_cdeg = lroundf(angle_deg * 100.0f);

    if (angle_cdeg < 0) {
        return 0;
    }
    if (angle_cdeg > 35999) {
        return 35999;
    }
    return (uint16_t)angle_cdeg;
}

static int64_t position_to_cdeg(int32_t turns, float angle_deg)
{
    return (int64_t)turns * 36000 + position_angle_to_cdeg(angle_deg);
}

static int32_t legacy_position_from_parts(int32_t turns, float angle_deg)
{
    int64_t position_cdeg = position_to_cdeg(turns, angle_deg);

    if (position_cdeg > MY_INT32_MAX) {
        return MY_INT32_MAX;
    }
    if (position_cdeg < MY_INT32_MIN) {
        return MY_INT32_MIN;
    }
    return (int32_t)position_cdeg;
}

static float get_pos_error_deg(void)
{
    int32_t target_turns;
    int32_t actual_turns;
    uint16_t target_angle_cdeg;
    uint16_t actual_angle_cdeg;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    target_turns      = pos_point_turns;
    target_angle_cdeg = position_angle_to_cdeg(pos_point_angle);
    actual_turns      = mechanical_turns;
    actual_angle_cdeg = position_angle_to_cdeg(mechanical_angle);
    if (!primask) {
        __enable_irq();
    }

    int64_t turn_error = (int64_t)target_turns - actual_turns;
    return (float)turn_error * 360.0f + (float)((int32_t)target_angle_cdeg - actual_angle_cdeg) / 100.0f;
}

void set_pos_point_turns_angle(int32_t turns, float angle_deg)
{
    if (!isfinite(angle_deg)) {
        return;
    }

    while (angle_deg >= 360.0f && turns < INT32_MAX) {
        angle_deg -= 360.0f;
        turns++;
    }
    while (angle_deg < 0.0f && turns > INT32_MIN) {
        angle_deg += 360.0f;
        turns--;
    }

    if (angle_deg >= 360.0f) {
        angle_deg = 359.99f;
    } else if (angle_deg < 0.0f) {
        angle_deg = 0.0f;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pos_point_turns = turns;
    pos_point_angle = angle_deg;
    pos_point       = legacy_position_from_parts(turns, angle_deg);
    if (!primask) {
        __enable_irq();
    }
}

void set_pos_point_legacy_cdeg(int32_t position_cdeg)
{
    int32_t turns   = position_cdeg / 36000;
    float angle_deg = (float)(position_cdeg % 36000) / 100.0f;

    set_pos_point_turns_angle(turns, angle_deg);
}

void get_target_position_snapshot(int32_t *turns, uint16_t *angle_cdeg)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *turns      = pos_point_turns;
    *angle_cdeg = position_angle_to_cdeg(pos_point_angle);
    if (!primask) {
        __enable_irq();
    }
}

void get_mechanical_position_snapshot(int32_t *turns, uint16_t *angle_cdeg)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *turns      = mechanical_turns;
    *angle_cdeg = position_angle_to_cdeg(mechanical_angle);
    if (!primask) {
        __enable_irq();
    }
}

float consume_mechanical_delta_rad(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float delta_rad            = mechanical_delta_rad_accum;
    mechanical_delta_rad_accum = 0.0f;
    if (!primask) {
        __enable_irq();
    }
    return delta_rad;
}

uint8_t is_legacy_position_out_of_range(void)
{
    int32_t turns;
    uint16_t angle_cdeg;

    get_mechanical_position_snapshot(&turns, &angle_cdeg);
    int64_t position_cdeg = (int64_t)turns * 36000 + angle_cdeg;
    return position_cdeg > MY_INT32_MAX || position_cdeg < MY_INT32_MIN;
}

int32_t get_legacy_position_cdeg(void)
{
    int32_t turns;
    uint16_t angle_cdeg;

    get_mechanical_position_snapshot(&turns, &angle_cdeg);
    return legacy_position_from_parts(turns, (float)angle_cdeg / 100.0f);
}

int32_t normalize_current_limit(int32_t current_limit)
{
    int64_t magnitude = current_limit;

    if (magnitude < 0) {
        magnitude = -magnitude;
    }
    if (magnitude > 120000) {
        magnitude = 120000;
    }

    return (int32_t)magnitude;
}

int32_t apply_pid_current_limit(PIDControl *pid, int32_t current_limit)
{
    current_limit = normalize_current_limit(current_limit);
    float limit   = (float)current_limit / 100.0f;

    if (limit > 0.0f) {
        PIDOutputLimitsSet(pid, -limit, limit);
    } else {
        pid->outMin      = 0.0f;
        pid->outMax      = 0.0f;
        pid->output      = 0.0f;
        pid->output_prev = 0.0f;
        pid->iTerm       = 0.0f;
    }

    return current_limit;
}

void init_pid(void)
{
    switch (speed_pid_index) {
        case 0:
            PIDInit(&pid_ctrl_speed_t, speed_pid_float[0], speed_pid_float[1], speed_pid_float[2], 1.0f,
                    -(float)max_speed_current / 100, (float)max_speed_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_speed_t.setpoint = 0;
            break;
        case 1:
            PIDInit(&pid_ctrl_speed_t, speed_pid_low_float[0], speed_pid_low_float[1], speed_pid_low_float[2], 1.0f,
                    -(float)max_speed_current / 100, (float)max_speed_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_speed_t.setpoint = 0;
            break;
        case 2:
            PIDInit(&pid_ctrl_speed_t, speed_pid_mid_float[0], speed_pid_mid_float[1], speed_pid_mid_float[2], 1.0f,
                    -(float)max_speed_current / 100, (float)max_speed_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_speed_t.setpoint = 0;
            break;
        case 3:
            PIDInit(&pid_ctrl_speed_t, speed_pid_high_float[0], speed_pid_high_float[1], speed_pid_high_float[2], 1.0f,
                    -(float)max_speed_current / 100, (float)max_speed_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_speed_t.setpoint = 0;
            break;

        default:
            break;
    }

    switch (pos_pid_index) {
        case 0:
            PIDInit(&pid_ctrl_pos_t, pos_pid_float[0], pos_pid_float[1], pos_pid_float[2], 1.0f,
                    -(float)max_pos_current / 100, (float)max_pos_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_pos_t.setpoint = 0;
            break;
        case 1:
            PIDInit(&pid_ctrl_pos_t, pos_pid_low_float[0], pos_pid_low_float[1], pos_pid_low_float[2], 1.0f,
                    -(float)max_pos_current / 100, (float)max_pos_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_pos_t.setpoint = 0;
            break;
        case 2:
            PIDInit(&pid_ctrl_pos_t, pos_pid_mid_float[0], pos_pid_mid_float[1], pos_pid_mid_float[2], 1.0f,
                    -(float)max_pos_current / 100, (float)max_pos_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_pos_t.setpoint = 0;
            break;
        case 3:
            PIDInit(&pid_ctrl_pos_t, pos_pid_high_float[0], pos_pid_high_float[1], pos_pid_high_float[2], 1.0f,
                    -(float)max_pos_current / 100, (float)max_pos_current / 100, AUTOMATIC, DIRECT);
            pid_ctrl_pos_t.setpoint = 0;
            break;

        default:
            break;
    }
}

void speed_pid(void)
{
    pid_ctrl_speed_t.input = motor_rpm;
    if (sys_status == SYS_RUNNING) {
        if ((fabsf(pid_ctrl_speed_t.setpoint) > 1e-6f) &&
            (fabsf(pid_ctrl_speed_t.point_error / pid_ctrl_speed_t.setpoint) > 0.5f)) {
            rgb_flash_slow = 0;
        } else {
            rgb_flash_slow = 1;
        }
    }
    if (0) {
        pid_ctrl_speed_t.iTerm = 0;
        MotorDriverSetCurrentReal(0.0f);
    } else {
        PIDCompute(&pid_ctrl_speed_t);
        if (speed_err_value) {
            // 第一次堵转判断
            if (!speed_err_auto_flag) {
                if (fabsf(pid_ctrl_speed_t.point_error) >= speed_err_value &&
                    fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
                    if (!speed_err_count_flag) {
                        speed_err_counter    = HAL_GetTick();
                        speed_err_count_flag = 1;
                    }
                } else {
                    if (speed_err_count_flag == 1) speed_err_count_flag = 0;
                }

                // 跳转到堵转保护模式，2S后自动恢复
                if (speed_err_count_flag == 1 && HAL_GetTick() - speed_err_counter > speed_err_timeout * 1000) {
                    motor_mode = MODE_SPEED_ERR_PROTECT;
                    error_code |= ERR_STALLED;
                    MotorDriverSetMode(MDRV_MODE_OFF);
                    pid_ctrl_speed_t.iTerm = 0;
                    speed_err_auto_counter = HAL_GetTick();
                    speed_err_auto_flag    = 1;
                    return;
                }
            }
            // 自动恢复后判断是否有堵转
            else {
                if (speed_err_count_flag == 2) {
                    speed_err_recover_counter = HAL_GetTick();
                    speed_err_count_flag      = 1;
                }
                if (fabsf(pid_ctrl_speed_t.point_error) >= speed_err_value &&
                    fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
                    if (HAL_GetTick() - speed_err_recover_counter > 500) {
                        motor_mode = MODE_SPEED_ERR_PROTECT;
                        error_code |= ERR_STALLED;
                        MotorDriverSetMode(MDRV_MODE_OFF);
                        pid_ctrl_speed_t.iTerm = 0;
                        speed_err_auto_counter = HAL_GetTick();
                    }
                } else {
                    if (HAL_GetTick() - speed_err_recover_counter > 500) {
                        speed_err_recover_try_counter = 0;
                        error_code &= ~ERR_STALLED;
                        speed_err_count_flag = 0;
                        speed_err_auto_flag  = 0;
                        err_stalled_flag     = 0;
                    }
                }
                if (motor_disable_flag) {
                    if (motor_mode == MODE_SPEED_ERR_PROTECT) {
                        MotorDriverSetMode(MDRV_MODE_OFF);
                        err_stalled_flag = 1;
                    } else {
                        error_code &= ~ERR_STALLED;
                        MotorDriverSetMode(MDRV_MODE_OFF);
                    }
                    motor_disable_flag = 0;
                }
            }

            MotorDriverSetCurrentReal(pid_ctrl_speed_t.output);
        } else {
            if (!err_stalled_flag) {
                speed_err_count_flag = 0;
                error_code &= ~ERR_STALLED;
                speed_err_auto_flag = 0;
            } else {
                pid_ctrl_speed_t.iTerm = 0;
            }
            MotorDriverSetCurrentReal(pid_ctrl_speed_t.output);
        }
    }
}

void pos_pid(void)
{
    static uint8_t i_overflow_count_flag = 0;
    static uint32_t i_overflow_counter   = 0;
    float position_error                 = get_pos_error_deg();

    pid_ctrl_pos_t.setpoint      = 0.0f;
    pid_ctrl_pos_t.input         = -position_error;
    uint8_t i_overflow_condition = (fabsf(position_error) <= 360.0f) && (fabsf(pid_ctrl_pos_t.iTerm) > 30.0f) &&
                                   (fabsf(pid_ctrl_pos_t.output) < 30.0f);

    if (i_overflow_condition) {
        if (!i_overflow_count_flag) {
            i_overflow_count_flag = 1;
            i_overflow_counter    = HAL_GetTick();
        } else if ((uint32_t)(HAL_GetTick() - i_overflow_counter) >= 100U) {
            pid_ctrl_pos_t.iTerm  = 0.0f;
            i_overflow_count_flag = 0;
        }
    } else {
        i_overflow_count_flag = 0;
    }
    if (sys_status == SYS_RUNNING) {
        if (fabsf(pid_ctrl_pos_t.point_error) > 10.0f) {
            rgb_flash_slow = 0;
        } else {
            rgb_flash_slow = 1;
        }
    }
    PIDCompute(&pid_ctrl_pos_t);
    if (pos_err_value) {
        // 第一次堵转判断
        if (!pos_err_auto_flag) {
            if (fabsf(pid_ctrl_pos_t.point_error) >= pos_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
                if (!pos_err_count_flag) {
                    pos_err_counter    = HAL_GetTick();
                    pos_err_count_flag = 1;
                }
            } else {
                if (pos_err_count_flag == 1) pos_err_count_flag = 0;
            }

            // 跳转到堵转保护模式，2S后自动恢复
            if (pos_err_count_flag == 1 && HAL_GetTick() - pos_err_counter > pos_err_timeout * 1000) {
                motor_mode = MODE_POS_ERR_PROTECT;
                error_code |= ERR_STALLED;
                MotorDriverSetMode(MDRV_MODE_OFF);
                pid_ctrl_pos_t.iTerm = 0;
                pos_err_auto_counter = HAL_GetTick();
                pos_err_auto_flag    = 1;
                return;
            }
        }
        // 自动恢复后判断是否有堵转
        else {
            if (pos_err_count_flag == 2) {
                pos_err_recover_counter = HAL_GetTick();
                pos_err_count_flag      = 1;
            }
            if (fabsf(pid_ctrl_pos_t.point_error) >= pos_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
                if (HAL_GetTick() - pos_err_recover_counter > 500) {
                    motor_mode = MODE_POS_ERR_PROTECT;
                    error_code |= ERR_STALLED;
                    MotorDriverSetMode(MDRV_MODE_OFF);
                    pid_ctrl_pos_t.iTerm = 0;
                    pos_err_auto_counter = HAL_GetTick();
                }
            } else {
                if (HAL_GetTick() - pos_err_recover_counter > 500) {
                    pos_err_recover_try_counter = 0;
                    error_code &= ~ERR_STALLED;
                    pos_err_count_flag = 0;
                    pos_err_auto_flag  = 0;
                    err_stalled_flag   = 0;
                }
            }
            if (motor_disable_flag) {
                if (motor_mode == MODE_POS_ERR_PROTECT) {
                    MotorDriverSetMode(MDRV_MODE_OFF);
                    err_stalled_flag = 1;
                } else {
                    error_code &= ~ERR_STALLED;
                    MotorDriverSetMode(MDRV_MODE_OFF);
                }
                motor_disable_flag = 0;
            }
        }

        MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
    } else {
        if (!err_stalled_flag) {
            pos_err_count_flag = 0;
            error_code &= ~ERR_STALLED;
            pos_err_auto_flag = 0;
        } else {
            pid_ctrl_pos_t.iTerm = 0;
        }
        MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
    }
}

void pos_speed_pid(void)
{
    float position_error = get_pos_error_deg();

    pid_ctrl_pos_t.setpoint = 0.0f;
    pid_ctrl_pos_t.input    = -position_error;
    PIDCompute(&pid_ctrl_pos_t);
    pos_output_lpf +=
        (1.0f / (1.0f + 1.0f / (2.0f * 3.14f * 0.00019642857f * 2.0f))) * (pid_ctrl_pos_t.output - pos_output_lpf);
    pid_ctrl_speed_t.setpoint = pos_output_lpf;
}

uint8_t crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while (len--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8c;
            } else
                crc >>= 1;
        }
    }
    return crc;
}

void InitMysys(void)
{
    encoder_absolute_angle_new = 0;
    encoder_absolute_angle_old = 0;
    mechanical_angle           = 0;
    mechanical_turns           = 0;
    mechanical_rad             = 0;
    mechanical_delta_rad       = 0;
    mechanical_delta_rad_accum = 0;
    pos_point_turns            = 0;
    pos_point_angle            = 0;
    pos_point                  = 0;
    angle_error                = 0;

    angle_target = 240.0f;
    angle_kp     = 0.5f;

    uq_limit = 500.0f;

    counter_loop_foc     = 0;
    counter_loop_control = 0;

    GPIOB->BSRR = 1 << 1;  // enable DRV8311 to enable intrlnal current sensor

    MyADCInit();
    TIM1->CCR4 = 995;  // Enable TIM1 CH4 for ADC trigger

    // Enable TIM1 channels for PWM generate
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_Delay(20);
    // TIM1 update interrupt for FOC and outside control loop
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    MotorDriverInit();

    MyADCZeroCal();

    EncoderInit();
    HAL_Delay(300);
    init_flash_data();
    display_app_init();
    init_pid();
    if (comm_type == COMM_TYPE_I2C) {
        user_i2c_init();
        i2c1_it_enable();
    } else if (comm_type == COMM_TYPE_485) {
        user_usart3_uart_init();
        hard_uart_begin();
    } else if (comm_type == COMM_TYPE_485_I2C) {
        user_usart3_uart_init();
        user_i2c_init();
        I2C1_Start();
        hard_uart_begin();
    }

    display_app_show_startup();
}

void LoopMysys(void)
{
    while (1) {
        i2c_timeout_counter      = 0;
        uint32_t i2c_timeout_now = HAL_GetTick();
        if (i2c_stop_timeout_flag) {
            if ((uint32_t)(i2c_timeout_now - i2c_stop_timeout_delay) >= 10U) {
                i2c_stop_timeout_counter++;
                i2c_stop_timeout_delay = i2c_timeout_now;
            }
        }
        if (i2c_stop_timeout_flag && i2c_stop_timeout_counter > 50U) {
            LL_I2C_DeInit(I2C1);
            LL_I2C_DisableAutoEndMode(I2C1);
            LL_I2C_Disable(I2C1);
            LL_I2C_DisableIT_ADDR(I2C1);
            i2c1_reset_receive_state();
            i2c1_reset_timeout_state();
            user_i2c_init();
            i2c1_it_enable();
        }
        if (usart_fault_flag) {
            MX_USART3_UART_Init();
            hard_uart_begin();
            usart_fault_flag = 0;
        }
        display_app_update_input();

        if (LL_USART_IsActiveFlag_ORE(USART3)) {
            LL_USART_ClearFlag_ORE(USART3);
            MX_USART3_UART_Init();
        }
        if (usart_tx_flag && HAL_GetTick() - usart_tx_delay >= 50) {
            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_RESET);
            usart_tx_flag = 0;
        }
        display_app_update_page();
        if (rgb_color_buffer_index && rgb_show_mode) {
            uint32_t rgb_show_index = rgb_color_buffer_index;
            for (uint32_t i = 0; i < rgb_show_index; i++) {
                neopixel_set_color(0, rgb_color_buffer[i]);
                neopixel_set_color(1, rgb_color_buffer[i]);
                ws2812_show();
            }
            rgb_color_buffer_index = 0;
        }
        display_app_render();
    }
}

void Loop_FOC(void)
{
    GPIOB->BSRR = GPIO_PIN_9;
    MotorDriverProcess();
    MyAdcProcess();
    GPIOB->BRR = GPIO_PIN_9;
}

void Loop_Control(void)
{
    encoder_absolute_angle_old = encoder_absolute_angle_new;
    encoder_absolute_angle_new = MotorDriverGetMechanicalAngle() / 10.0f;

    float angle_diff   = encoder_absolute_angle_new - encoder_absolute_angle_old;
    int32_t next_turns = mechanical_turns;

    if (angle_diff > 180.0f) {
        if (next_turns > INT32_MIN) {
            next_turns--;
        }
        angle_diff -= 360.0f;
    } else if (angle_diff < -180.0f) {
        if (next_turns < INT32_MAX) {
            next_turns++;
        }
        angle_diff += 360.0f;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mechanical_turns     = next_turns;
    mechanical_angle     = encoder_absolute_angle_new;
    mechanical_rad       = mechanical_angle * PI / 180.0f;
    mechanical_delta_rad = angle_diff * PI / 180.0f;
    mechanical_delta_rad_accum += mechanical_delta_rad;
    if (!primask) {
        __enable_irq();
    }

    // lpfdata += (1.0 / (1.0 + 1.0/(2.0f * 3.14f *T*fc)))*(rawdata - lpfdata );
    // lpfdata ： 滤波后的数据。
    // rawdata ： 滤波前的原始数据。
    // T： 数据的采样频率的倒数，即采样周期，单位是秒。
    // fc : 截止频率。截止频率就是超过该频率的数据（噪声）都被过滤掉，只保留低于该截止频率的数据。

    ph_current_rt = MotorDriverGetPhaseCurrentReal();
    ph_crrent_lpf += (1.0f / (1.0f + 1.0f / (2.0f * 3.14f * 0.0002f * 2.0f))) * (ph_current_rt - ph_crrent_lpf);

    speed_encoder_update();

    diff_encoder_value                  = speed_encoder_value_t.encoder_value;
    speed_encoder_value_t.encoder_value = 0;
    diff_encoder_value_lpf += (1.0f / (1.0f + 1.0f / (2.0f * 3.14f * 0.00017857142857f * 2.0f))) *
                              (diff_encoder_value - diff_encoder_value_lpf);

    rpm_rps_count_temp = diff_encoder_value_lpf / 16383.0f;
    motor_rpm          = rpm_rps_count_temp * 336000;
    motor_rps          = rpm_rps_count_temp * 2016000 * PI / 180.0f;

    // get input voltage
    vol_input = (MyAdcGetVal(1, 4) * 330 * 6.4545454545f) / 4095;  // adc1_in4, e.g. 1036 = 10.36v
    vol_lpf += (1.0f / (1.0f + 1.0f / (2.0f * 3.14f * 0.0002f * 2.0f))) * (vol_input - vol_lpf);

    if (!over_vol_flag) {
        if (vol_lpf > 1800) {
            over_vol_flag = 1;
            error_code |= ERR_OVER_VOLTAGE;
            MotorDriverSetMode(MDRV_MODE_OFF);
            if (!over_vol_protect_auto_flag && over_vol_protect_mode) {
                over_vol_protect_auto_counter = HAL_GetTick();
                over_vol_protect_auto_flag    = 1;
            }
        }
    } else {
        if (vol_lpf <= 1750) {
            over_vol_flag = 0;
            error_code &= ~ERR_OVER_VOLTAGE;
            sys_status = SYS_STANDBY;
            if (!over_vol_protect_auto_flag && over_vol_protect_mode) {
                over_vol_protect_auto_counter = HAL_GetTick();
                over_vol_protect_auto_flag    = 1;
            }
        } else {
            over_vol_flag = 1;
            error_code |= ERR_OVER_VOLTAGE;
            MotorDriverSetMode(MDRV_MODE_OFF);
        }
    }
    if (motor_overvalue_protection_flag) {
        if (is_legacy_position_out_of_range()) {
            over_value_flag = 1;
            error_code |= ERR_OVER_VALUE;
            MotorDriverSetMode(MDRV_MODE_OFF);
        } else {
            if (over_value_flag) {
                over_value_flag = 0;
                error_code &= ~ERR_OVER_VALUE;
                sys_status = SYS_STANDBY;
            }
        }
    } else {
        if (over_value_flag) {
            over_value_flag = 0;
            error_code &= ~ERR_OVER_VALUE;
            sys_status = SYS_STANDBY;
        }
    }
}

float avg_filter(float *data, int len)
{
    float sum = 0;
    float min = data[0];
    float max = data[0];
    for (int i = 0; i < len; i++) {
        if (data[i] < min) {
            min = data[i];
        }
        if (data[i] > max) {
            max = data[i];
        }
        sum += data[i];
    }

    sum -= min;
    sum -= max;

    return sum / (len - 2);
}

void Rpm_Count_100us(void)
{
    speed_encoder_update();

    diff_encoder_value                  = speed_encoder_value_t.encoder_value;
    speed_encoder_value_t.encoder_value = 0;
    diff_encoder_value_lpf += (1.0f / (1.0f + 1.0f / (2.0f * 3.14f * 0.00017857142857f * 2.0f))) *
                              (diff_encoder_value - diff_encoder_value_lpf);
    motor_rpm = diff_encoder_value_lpf / 16383.0f * 336000;
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    // HAL_TIM_IRQHandler(&htim1);

    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    // MotorDriverProcess();
    if (counter_loop_foc < 2) {
        counter_loop_foc += 1;
    } else {
        counter_loop_foc = 0;
        Loop_FOC();
    }

    if (counter_loop_control < 9) {
        counter_loop_control += 1;
    } else {
        counter_loop_control = 0;
        Loop_Control();
    }

    if (pid_compute_counter < 10) {
        pid_compute_counter += 1;
    } else {
        pid_compute_counter = 0;
        switch (motor_mode) {
            case MODE_SPEED:
                if (sys_status == SYS_RUNNING) {
                    if (motor_stall_protection_flag) {
                        speed_err_value   = abs((int32_t)(speed_err_rate * pid_ctrl_speed_t.setpoint));
                        speed_err_timeout = 3;
                    } else {
                        speed_err_value   = 0;
                        speed_err_timeout = 0;
                    }
                    speed_pid();
                }
                break;
            case MODE_POS_SPEED:
                if (sys_status == SYS_RUNNING) {
                    pos_speed_pid();
                    speed_pid();
                }
                break;
            case MODE_POS:
                if (sys_status == SYS_RUNNING) {
                    if (motor_stall_protection_flag) {
                        float position_error = fabsf(get_pos_error_deg());
                        if (position_error > 10.0f)
                            pos_err_value = 10;
                        else
                            pos_err_value = (uint16_t)fabsf(pos_err_rate * position_error);
                        pos_err_timeout = 3;
                    } else {
                        pos_err_value   = 0;
                        pos_err_timeout = 0;
                    }
                    pos_pid();
                }
                break;
            case MODE_SPEED_ERR_PROTECT:
                if (HAL_GetTick() - speed_err_auto_counter > 2000 && speed_err_count_flag) {
                    if (!err_stalled_flag && err_recover_try_max &&
                        speed_err_recover_try_counter <= err_recover_try_max - 1) {
                        speed_err_recover_try_counter++;
                        MotorDriverSetMode(MDRV_MODE_RUN);
                        motor_mode           = MODE_SPEED;
                        speed_err_count_flag = 2;
                    } else {
                        err_stalled_flag = 1;
                    }
                }
                break;
            case MODE_POS_ERR_PROTECT:
                if (HAL_GetTick() - pos_err_auto_counter > 2000 && pos_err_count_flag) {
                    if (!err_stalled_flag && err_recover_try_max &&
                        pos_err_recover_try_counter <= err_recover_try_max - 1) {
                        pos_err_recover_try_counter++;
                        MotorDriverSetMode(MDRV_MODE_RUN);
                        motor_mode         = MODE_POS;
                        pos_err_count_flag = 2;
                    } else {
                        err_stalled_flag = 1;
                    }
                }
                break;
            case MODE_CURRENT:
                if (sys_status == SYS_RUNNING) {
                    current_point_float = (float)current_point / 100.0f;
                    if (current_point_float != 0.0f && fabsf(ph_crrent_lpf) < fabsf(current_point_float) * 0.90f) {
                        rgb_flash_slow = 1;
                    } else {
                        rgb_flash_slow = 0;
                    }
                }
                break;
            case MODE_DIAL:
                if (sys_status == SYS_RUNNING || sys_status == SYS_STANDBY) handle_smart_knob();
                break;

            default:
                break;
        }
    }
}
