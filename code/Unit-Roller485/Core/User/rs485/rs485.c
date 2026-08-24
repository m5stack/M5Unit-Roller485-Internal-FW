/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "rs485.h"

#include <string.h>

#include "i2c.h"
#include "i2c_ex.h"
#include "motordriver.h"
#include "myadc.h"
#include "mysys.h"
#include "rgb.h"
#include "smart_knob.h"
#include "u8g2_disp_fun.h"
#include "usart.h"

uint8_t change_baudrate_flag = 0;

static void send_standard_response(uint8_t command, uint8_t value)
{
    HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
    usart_tx_delay = HAL_GetTick();
    usart_tx_flag  = 1;
    for (int i = 0; i < 17; i++) {
        dat_Uart1.pTxBuf[i] = 0;
    }
    dat_Uart1.pTxBuf[0]  = 0xAA;
    dat_Uart1.pTxBuf[1]  = 0x55;
    dat_Uart1.pTxBuf[2]  = command + 0x10;
    dat_Uart1.pTxBuf[3]  = motor_id;
    dat_Uart1.pTxBuf[4]  = value;
    dat_Uart1.pTxBuf[16] = crc8_MAXIM(&dat_Uart1.pTxBuf[2], 14);
    UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
}

static void send_readback_response(uint8_t command, const uint8_t *payload, uint8_t payload_length)
{
    if (payload == NULL || payload_length > 15U) {
        return;
    }

    HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
    usart_tx_delay = HAL_GetTick();
    usart_tx_flag  = 1;
    memset(dat_Uart1.pTxBuf, 0, 20);
    dat_Uart1.pTxBuf[0] = 0xAA;
    dat_Uart1.pTxBuf[1] = 0x55;
    dat_Uart1.pTxBuf[2] = command + 0x10;
    dat_Uart1.pTxBuf[3] = motor_id;
    memcpy(&dat_Uart1.pTxBuf[4], payload, payload_length);
    dat_Uart1.pTxBuf[19] = crc8_MAXIM(&dat_Uart1.pTxBuf[2], 17);
    UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 20);
}

void Usart_Receive_Data(USART_TypeDef *USARTx)
{
    if (LL_USART_IsActiveFlag_IDLE(USARTx) && LL_USART_IsEnabledIT_IDLE(USARTx)) {
        LL_USART_ClearFlag_IDLE(USARTx);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        dat_Uart1.LEN = UART1_RX_BUF_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);

        dat_Uart1.FLG                   = 1;
        dat_Uart1.pRxBuf[dat_Uart1.LEN] = 0;

        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, UART1_RX_BUF_SIZE);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
        if (dat_Uart1.LEN) {
            comm_flash_count = 4;
            if (dat_Uart1.LEN == 15) {
                if (crc8_MAXIM(dat_Uart1.pRxBuf, 14) == dat_Uart1.pRxBuf[14] && dat_Uart1.pRxBuf[1] == motor_id) {
                    switch (dat_Uart1.pRxBuf[0]) {
                        case 1:
                            if (dat_Uart1.pRxBuf[2] >= 1 && dat_Uart1.pRxBuf[2] <= 4 && !err_stalled_flag) {
                                motor_mode = dat_Uart1.pRxBuf[2];
                                if (last_motor_mode != motor_mode) {
                                    if (motor_mode < MODE_DIAL) {
                                        MotorDriverSetCurrentReal(0);
                                        init_pid();
                                        pid_ctrl_speed_t.iTerm = 0;
                                        pid_ctrl_pos_t.iTerm   = 0;
                                    } else if (motor_mode == MODE_DIAL) {
                                        init_smart_knob();
                                    } else if (motor_mode == MODE_POS_SPEED) {
                                        MotorDriverSetCurrentReal(0);
                                        PIDTuningsSet(&pid_ctrl_speed_t, speed_pid_plus_float[0],
                                                      speed_pid_plus_float[1], speed_pid_plus_float[2]);
                                        PIDTuningsSet(&pid_ctrl_pos_t, pos_pid_plus_float[0], pos_pid_plus_float[1],
                                                      pos_pid_plus_float[2]);
                                        pid_ctrl_speed_t.iTerm = 0;
                                        pid_ctrl_pos_t.iTerm   = 0;
                                    }
                                    last_motor_mode = motor_mode;
                                }

                                HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                                usart_tx_delay = HAL_GetTick();
                                usart_tx_flag  = 1;
                                for (int i = 0; i < 17; i++) {
                                    dat_Uart1.pTxBuf[i] = 0;
                                }
                                dat_Uart1.pTxBuf[0]      = 0xAA;
                                dat_Uart1.pTxBuf[1]      = 0x55;
                                dat_Uart1.pTxBuf[2]      = dat_Uart1.pRxBuf[0] + 0x10;
                                dat_Uart1.pTxBuf[3]      = motor_id;
                                dat_Uart1.pTxBuf[4]      = motor_mode;
                                dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                                UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            }
                            break;

                        case 0:
                            motor_output = dat_Uart1.pRxBuf[2];
                            if (motor_output) {
                                if (!over_vol_flag && !err_stalled_flag && motor_mode < MODE_MAX) {
                                    if (motor_mode == MODE_DIAL && motor_disable_flag) {
                                        init_smart_knob();
                                    }
                                    motor_disable_flag = 0;
                                    MotorDriverSetMode(MDRV_MODE_RUN);
                                }
                            } else {
                                motor_disable_flag = 1;
                                MotorDriverSetMode(MDRV_MODE_OFF);
                            }
                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]      = 0xAA;
                            dat_Uart1.pTxBuf[1]      = 0x55;
                            dat_Uart1.pTxBuf[2]      = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[3]      = motor_id;
                            dat_Uart1.pTxBuf[4]      = motor_output;
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 2:
                            if (dat_Uart1.pRxBuf[2])
                                over_vol_protect_mode = 1;
                            else
                                over_vol_protect_mode = 0;

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]      = 0xAA;
                            dat_Uart1.pTxBuf[1]      = 0x55;
                            dat_Uart1.pTxBuf[2]      = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[3]      = motor_id;
                            dat_Uart1.pTxBuf[4]      = over_vol_protect_mode;
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 3:
                            err_recover_try_max = dat_Uart1.pRxBuf[2];

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]      = 0xAA;
                            dat_Uart1.pTxBuf[1]      = 0x55;
                            dat_Uart1.pTxBuf[2]      = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[3]      = motor_id;
                            dat_Uart1.pTxBuf[4]      = err_recover_try_max;
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 4:
                            memcpy((uint8_t *)&speed_err_value, &dat_Uart1.pRxBuf[2], 2);
                            speed_err_timeout = dat_Uart1.pRxBuf[6];

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &speed_err_value, 2);
                            dat_Uart1.pTxBuf[6 + 2]  = speed_err_timeout;
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 5:
                            memcpy((uint8_t *)&pos_err_value, &dat_Uart1.pRxBuf[2], 2);
                            pos_err_timeout = dat_Uart1.pRxBuf[6];

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &pos_err_value, 2);
                            dat_Uart1.pTxBuf[6 + 2]  = pos_err_timeout;
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 6:
                            if (dat_Uart1.pRxBuf[2]) {
                                if (over_vol_flag) sys_status = SYS_STANDBY;
                                over_vol_flag = 0;
                                error_code &= ~ERR_OVER_VOLTAGE;
                            }
                            if (dat_Uart1.pRxBuf[6]) {
                                speed_err_recover_try_counter = 0;
                                pos_err_recover_try_counter   = 0;
                                if (motor_mode == MODE_SPEED_ERR_PROTECT) {
                                    sys_status = SYS_STANDBY;
                                    motor_mode = MODE_SPEED;
                                } else if (motor_mode == MODE_POS_ERR_PROTECT) {
                                    sys_status = SYS_STANDBY;
                                    motor_mode = MODE_POS;
                                }
                                error_code &= ~ERR_STALLED;
                                speed_err_count_flag = 0;
                                speed_err_auto_flag  = 0;
                                pos_err_count_flag   = 0;
                                pos_err_auto_flag    = 0;
                                err_stalled_flag     = 0;
                            }
                            send_standard_response(dat_Uart1.pRxBuf[0], dat_Uart1.pRxBuf[2]);
                            break;
                        case 7:
                            if (dat_Uart1.pRxBuf[2]) {
                                flash_data_write_back();
                            }
                            send_standard_response(dat_Uart1.pRxBuf[0], dat_Uart1.pRxBuf[2]);
                            break;
                        case 8:
                            memcpy((uint8_t *)&current_position, &dat_Uart1.pRxBuf[2], 4);

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &current_position, 4);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;
                        case 9:
                            if (dat_Uart1.pRxBuf[2]) {
                                mode_switch_flag = 1;
                            } else {
                                mode_switch_flag = 0;
                            }

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]      = 0xAA;
                            dat_Uart1.pTxBuf[1]      = 0x55;
                            dat_Uart1.pTxBuf[0 + 2]  = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2]  = motor_id;
                            dat_Uart1.pTxBuf[2 + 2]  = dat_Uart1.pRxBuf[2];
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;
                        case 0x0A:
                            memcpy(&rgb_color_buffer[rgb_color_buffer_index], &dat_Uart1.pRxBuf[2], 3);
                            if (dat_Uart1.pRxBuf[5])
                                rgb_show_mode = 1;
                            else
                                rgb_show_mode = 0;
                            if (dat_Uart1.pRxBuf[6] <= 100) {
                                brightness_index = dat_Uart1.pRxBuf[6];
                                ws2812_show();
                            }

                            if (rgb_color_buffer[rgb_color_buffer_index]) {
                                lastest_rgb_color = rgb_color_buffer[rgb_color_buffer_index];
                                if (rgb_color_buffer_index < (RGB_BUFFER_SIZE - 1)) ++rgb_color_buffer_index;
                            }

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy(&dat_Uart1.pTxBuf[2 + 2], &dat_Uart1.pRxBuf[2], 5);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;
                        case 0x0B:
                            if (dat_Uart1.pRxBuf[2] <= 2) {
                                bps_index            = dat_Uart1.pRxBuf[2];
                                change_baudrate_flag = 1;
                            }

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy(&dat_Uart1.pTxBuf[2 + 2], &dat_Uart1.pRxBuf[2], 1);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;
                        case 0x0C:
                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy(&dat_Uart1.pTxBuf[2 + 2], &dat_Uart1.pRxBuf[2], 1);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            motor_id = dat_Uart1.pRxBuf[2];
                            break;
                        case 0x0D:
                            if (dat_Uart1.pRxBuf[2] <= 1) {
                                motor_stall_protection_flag = dat_Uart1.pRxBuf[2];
                            }

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy(&dat_Uart1.pTxBuf[2 + 2], &dat_Uart1.pRxBuf[2], 1);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;
                        case 0x0E:
                            if (dat_Uart1.pRxBuf[2] <= 1) {
                                motor_overvalue_protection_flag = dat_Uart1.pRxBuf[2];
                            }

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy(&dat_Uart1.pTxBuf[2 + 2], &dat_Uart1.pRxBuf[2], 1);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 0x20:
                            memcpy((uint8_t *)&speed_point, &dat_Uart1.pRxBuf[2], 4);
                            memcpy((uint8_t *)&max_speed_current, &dat_Uart1.pRxBuf[6], 4);
                            if (speed_point > MY_INT32_MAX)
                                speed_point = MY_INT32_MAX;
                            else if (speed_point < MY_INT32_MIN)
                                speed_point = MY_INT32_MIN;
                            // pid_ctrl_speed_t.iTerm = 0;
                            pid_ctrl_speed_t.setpoint = (float)speed_point / 100.0f;
                            max_speed_current         = apply_pid_current_limit(&pid_ctrl_speed_t, max_speed_current);

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = 0x30;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &speed_point, 4);
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &max_speed_current, 4);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 0x21:
                            memcpy((uint8_t *)&speed_pid_int[0], &dat_Uart1.pRxBuf[2], 4);
                            memcpy((uint8_t *)&speed_pid_int[1], &dat_Uart1.pRxBuf[6], 4);
                            memcpy((uint8_t *)&speed_pid_int[2], &dat_Uart1.pRxBuf[0x0A], 4);
                            for (int i = 0; i < 3; i += 2) {
                                speed_pid_float[i] = (float)speed_pid_int[i] / 100000;
                            }
                            speed_pid_float[1] = (float)speed_pid_int[1] / 10000000;
                            if (speed_pid_index == 0) {
                                PIDTuningsSet(&pid_ctrl_speed_t, speed_pid_float[0], speed_pid_float[1],
                                              speed_pid_float[2]);
                                pid_ctrl_speed_t.iTerm = 0;
                            }

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = 0x31;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            switch (speed_pid_index) {
                                case 0:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &speed_pid_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &speed_pid_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &speed_pid_int[2], 4);
                                    break;
                                case 1:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &speed_pid_low_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &speed_pid_low_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &speed_pid_low_int[2], 4);
                                    break;
                                case 2:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &speed_pid_mid_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &speed_pid_mid_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &speed_pid_mid_int[2], 4);
                                    break;
                                case 3:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &speed_pid_high_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &speed_pid_high_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &speed_pid_high_int[2], 4);
                                    break;

                                default:
                                    break;
                            }
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 0x22:
                            memcpy((uint8_t *)&pos_point, &dat_Uart1.pRxBuf[2], 4);
                            memcpy((uint8_t *)&max_pos_current, &dat_Uart1.pRxBuf[6], 4);
                            if (pos_point > MY_INT32_MAX)
                                pos_point = MY_INT32_MAX;
                            else if (pos_point < MY_INT32_MIN)
                                pos_point = MY_INT32_MIN;
                            set_pos_point_legacy_cdeg(pos_point);
                            max_pos_current = apply_pid_current_limit(&pid_ctrl_pos_t, max_pos_current);

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = 0x32;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &pos_point, 4);
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &max_pos_current, 4);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 0x23:
                            memcpy((uint8_t *)&pos_pid_int[0], &dat_Uart1.pRxBuf[2], 4);
                            memcpy((uint8_t *)&pos_pid_int[1], &dat_Uart1.pRxBuf[6], 4);
                            memcpy((uint8_t *)&pos_pid_int[2], &dat_Uart1.pRxBuf[0x0A], 4);
                            for (int i = 0; i < 3; i += 2) {
                                pos_pid_float[i] = (float)pos_pid_int[i] / 100000;
                            }
                            pos_pid_float[1] = (float)pos_pid_int[1] / 10000000;
                            if (pos_pid_index == 0) {
                                PIDTuningsSet(&pid_ctrl_pos_t, pos_pid_float[0], pos_pid_float[1], pos_pid_float[2]);
                                pid_ctrl_pos_t.iTerm = 0;
                            }

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = 0x33;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            switch (pos_pid_index) {
                                case 0:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &pos_pid_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &pos_pid_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &pos_pid_int[2], 4);
                                    break;
                                case 1:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &pos_pid_low_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &pos_pid_low_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &pos_pid_low_int[2], 4);
                                    break;
                                case 2:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &pos_pid_mid_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &pos_pid_mid_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &pos_pid_mid_int[2], 4);
                                    break;
                                case 3:
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &pos_pid_high_int[0], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &pos_pid_high_int[1], 4);
                                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[0x0A + 2], &pos_pid_high_int[2], 4);
                                    break;
                                default:
                                    break;
                            }

                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 0x24:
                            memcpy((uint8_t *)&current_point, &dat_Uart1.pRxBuf[2], 4);
                            if (current_point > 120000)
                                current_point = 120000;
                            else if (current_point < -120000)
                                current_point = -120000;

                            float current_set = (float)current_point / 100.0f;

                            MotorDriverSetCurrentReal(current_set);

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0]     = 0xAA;
                            dat_Uart1.pTxBuf[1]     = 0x55;
                            dat_Uart1.pTxBuf[0 + 2] = 0x34;
                            dat_Uart1.pTxBuf[1 + 2] = motor_id;
                            memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &current_point, 4);
                            dat_Uart1.pTxBuf[14 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;

                        case 0x25: {
                            int32_t target_turns;
                            uint16_t target_angle_cdeg;
                            int32_t requested_current_limit;

                            memcpy(&target_turns, &dat_Uart1.pRxBuf[2], sizeof(target_turns));
                            memcpy(&target_angle_cdeg, &dat_Uart1.pRxBuf[6], sizeof(target_angle_cdeg));
                            memcpy(&requested_current_limit, &dat_Uart1.pRxBuf[10], sizeof(requested_current_limit));

                            if (target_angle_cdeg <= 35999U) {
                                uint32_t primask = __get_PRIMASK();
                                __disable_irq();
                                max_pos_current = apply_pid_current_limit(&pid_ctrl_pos_t, requested_current_limit);
                                set_pos_point_turns_angle(target_turns, (float)target_angle_cdeg / 100.0f);
                                if (!primask) {
                                    __enable_irq();
                                }
                            }

                            get_target_position_snapshot(&target_turns, &target_angle_cdeg);

                            HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                            usart_tx_delay = HAL_GetTick();
                            usart_tx_flag  = 1;
                            for (int i = 0; i < 17; i++) {
                                dat_Uart1.pTxBuf[i] = 0;
                            }
                            dat_Uart1.pTxBuf[0] = 0xAA;
                            dat_Uart1.pTxBuf[1] = 0x55;
                            dat_Uart1.pTxBuf[2] = 0x35;
                            dat_Uart1.pTxBuf[3] = motor_id;
                            memcpy(&dat_Uart1.pTxBuf[4], &target_turns, sizeof(target_turns));
                            memcpy(&dat_Uart1.pTxBuf[8], &target_angle_cdeg, sizeof(target_angle_cdeg));
                            memcpy(&dat_Uart1.pTxBuf[12], &max_pos_current, sizeof(max_pos_current));
                            dat_Uart1.pTxBuf[16] = crc8_MAXIM(&dat_Uart1.pTxBuf[2], 14);
                            UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 17);
                            break;
                        }

                        default:
                            break;
                    }
                }
            } else if (dat_Uart1.LEN == 4) {
                if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                    dat_Uart1.pRxBuf[0] == 0x40) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                        usart_tx_delay = HAL_GetTick();
                        usart_tx_flag  = 1;
                        for (int i = 0; i < 20; i++) {
                            dat_Uart1.pTxBuf[i] = 0;
                        }
                        dat_Uart1.pTxBuf[0]          = 0xAA;
                        dat_Uart1.pTxBuf[1]          = 0x55;
                        dat_Uart1.pTxBuf[0 + 2]      = 0x50;
                        dat_Uart1.pTxBuf[1 + 2]      = motor_id;
                        int32_t motor_rpm_int        = motor_rpm * 100;
                        int32_t mechanical_angle_int = get_legacy_position_cdeg();
                        int32_t ph_current_int       = ph_crrent_lpf * 100;
                        memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &motor_rpm_int, 4);
                        memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &mechanical_angle_int, 4);
                        memcpy((uint8_t *)&dat_Uart1.pTxBuf[10 + 2], &ph_current_int, 4);
                        dat_Uart1.pTxBuf[14 + 2] = motor_mode;
                        dat_Uart1.pTxBuf[15 + 2] = sys_status;
                        dat_Uart1.pTxBuf[16 + 2] = error_code;
                        dat_Uart1.pTxBuf[17 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 17);
                        UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 20);
                    }
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                           dat_Uart1.pRxBuf[0] == 0x44) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        int32_t actual_turns;
                        uint16_t actual_angle_cdeg;
                        int32_t target_turns;
                        uint16_t target_angle_cdeg;

                        uint32_t primask = __get_PRIMASK();
                        __disable_irq();
                        get_mechanical_position_snapshot(&actual_turns, &actual_angle_cdeg);
                        get_target_position_snapshot(&target_turns, &target_angle_cdeg);
                        if (!primask) {
                            __enable_irq();
                        }

                        HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                        usart_tx_delay = HAL_GetTick();
                        usart_tx_flag  = 1;
                        for (int i = 0; i < 20; i++) {
                            dat_Uart1.pTxBuf[i] = 0;
                        }
                        dat_Uart1.pTxBuf[0] = 0xAA;
                        dat_Uart1.pTxBuf[1] = 0x55;
                        dat_Uart1.pTxBuf[2] = 0x54;
                        dat_Uart1.pTxBuf[3] = motor_id;
                        memcpy(&dat_Uart1.pTxBuf[4], &actual_turns, sizeof(actual_turns));
                        memcpy(&dat_Uart1.pTxBuf[8], &actual_angle_cdeg, sizeof(actual_angle_cdeg));
                        memcpy(&dat_Uart1.pTxBuf[12], &target_turns, sizeof(target_turns));
                        memcpy(&dat_Uart1.pTxBuf[16], &target_angle_cdeg, sizeof(target_angle_cdeg));
                        dat_Uart1.pTxBuf[19] = crc8_MAXIM(&dat_Uart1.pTxBuf[2], 17);
                        UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 20);
                    }
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                           dat_Uart1.pRxBuf[0] == 0x41) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                        usart_tx_delay = HAL_GetTick();
                        usart_tx_flag  = 1;
                        for (int i = 0; i < 20; i++) {
                            dat_Uart1.pTxBuf[i] = 0;
                        }
                        dat_Uart1.pTxBuf[0]     = 0xAA;
                        dat_Uart1.pTxBuf[1]     = 0x55;
                        dat_Uart1.pTxBuf[0 + 2] = 0x51;
                        dat_Uart1.pTxBuf[1 + 2] = motor_id;
                        int32_t vol_int32       = (int32_t)vol_lpf;
                        memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &vol_int32, 4);
                        memcpy((uint8_t *)&dat_Uart1.pTxBuf[6 + 2], &internal_temp, 4);
                        memcpy((uint8_t *)&dat_Uart1.pTxBuf[10 + 2], &current_position, 4);
                        dat_Uart1.pTxBuf[14 + 2] = rgb_show_mode;
                        dat_Uart1.pTxBuf[15 + 2] = brightness_index;
                        dat_Uart1.pTxBuf[17 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 17);
                        UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 20);
                    }
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                           dat_Uart1.pRxBuf[0] == 0x42) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                        usart_tx_delay = HAL_GetTick();
                        usart_tx_flag  = 1;
                        for (int i = 0; i < 20; i++) {
                            dat_Uart1.pTxBuf[i] = 0;
                        }
                        dat_Uart1.pTxBuf[0]     = 0xAA;
                        dat_Uart1.pTxBuf[1]     = 0x55;
                        dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                        dat_Uart1.pTxBuf[1 + 2] = motor_id;
                        switch (speed_pid_index) {
                            case 0:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&speed_pid_int[0], 12);
                                break;
                            case 1:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&speed_pid_low_int[0], 12);
                                break;
                            case 2:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&speed_pid_mid_int[0], 12);
                                break;
                            case 3:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&speed_pid_high_int[0], 12);
                                break;

                            default:
                                break;
                        }
                        memcpy((uint8_t *)&dat_Uart1.pTxBuf[14 + 2], (uint8_t *)&lastest_rgb_color, 3);
                        dat_Uart1.pTxBuf[17 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 17);
                        UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 20);
                    }
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                           dat_Uart1.pRxBuf[0] == 0x43) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                        usart_tx_delay = HAL_GetTick();
                        usart_tx_flag  = 1;
                        for (int i = 0; i < 20; i++) {
                            dat_Uart1.pTxBuf[i] = 0;
                        }
                        dat_Uart1.pTxBuf[0]     = 0xAA;
                        dat_Uart1.pTxBuf[1]     = 0x55;
                        dat_Uart1.pTxBuf[0 + 2] = dat_Uart1.pRxBuf[0] + 0x10;
                        dat_Uart1.pTxBuf[1 + 2] = motor_id;
                        switch (pos_pid_index) {
                            case 0:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&pos_pid_int[0], 12);
                                break;
                            case 1:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&pos_pid_low_int[0], 12);
                                break;
                            case 2:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&pos_pid_mid_int[0], 12);
                                break;
                            case 3:
                                memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], (uint8_t *)&pos_pid_high_int[0], 12);
                                break;

                            default:
                                break;
                        }
                        dat_Uart1.pTxBuf[14 + 2] = motor_id;
                        dat_Uart1.pTxBuf[15 + 2] = bps_index;
                        dat_Uart1.pTxBuf[16 + 2] = mode_switch_flag;
                        dat_Uart1.pTxBuf[17 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 17);
                        UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 20);
                    }
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                           dat_Uart1.pRxBuf[0] == 0x45) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        uint32_t uid_words[3] = {LL_GetUID_Word0(), LL_GetUID_Word1(), LL_GetUID_Word2()};

                        HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                        usart_tx_delay = HAL_GetTick();
                        usart_tx_flag  = 1;
                        for (int i = 0; i < 20; i++) {
                            dat_Uart1.pTxBuf[i] = 0;
                        }
                        dat_Uart1.pTxBuf[0] = 0xAA;
                        dat_Uart1.pTxBuf[1] = 0x55;
                        dat_Uart1.pTxBuf[2] = 0x55;
                        dat_Uart1.pTxBuf[3] = motor_id;
                        memcpy(&dat_Uart1.pTxBuf[4], uid_words, sizeof(uid_words));
                        dat_Uart1.pTxBuf[16] = ROLLER_DEVICE_ID;
                        dat_Uart1.pTxBuf[17] = fm_version;
                        dat_Uart1.pTxBuf[19] = crc8_MAXIM(&dat_Uart1.pTxBuf[2], 17);
                        UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 20);
                    }
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                           dat_Uart1.pRxBuf[0] == 0x46) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        uint8_t payload[15] = {0};
                        int32_t speed_current_limit;
                        int32_t position_current_limit;
                        int32_t current_target;

                        uint32_t primask = __get_PRIMASK();
                        __disable_irq();
                        speed_current_limit    = max_speed_current;
                        position_current_limit = max_pos_current;
                        current_target         = current_point;
                        if (!primask) {
                            __enable_irq();
                        }

                        memcpy(&payload[0], &speed_current_limit, sizeof(speed_current_limit));
                        memcpy(&payload[4], &position_current_limit, sizeof(position_current_limit));
                        memcpy(&payload[8], &current_target, sizeof(current_target));
                        send_readback_response(dat_Uart1.pRxBuf[0], payload, sizeof(payload));
                    }
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 3) == dat_Uart1.pRxBuf[3] && dat_Uart1.pRxBuf[1] == motor_id &&
                           dat_Uart1.pRxBuf[0] == 0x47) {
                    if (!dat_Uart1.pRxBuf[2]) {
                        uint8_t payload[15] = {0};
                        int32_t speed_target;
                        int32_t legacy_position_target;

                        uint32_t primask = __get_PRIMASK();
                        __disable_irq();
                        speed_target           = speed_point;
                        legacy_position_target = pos_point;
                        payload[8]             = motor_output;
                        payload[9]             = over_vol_protect_mode;
                        payload[10]            = motor_stall_protection_flag;
                        payload[11]            = motor_overvalue_protection_flag;
                        payload[12]            = i2c_address[0];
                        payload[13]            = bps_index;
                        payload[14]            = mode_switch_flag;
                        if (!primask) {
                            __enable_irq();
                        }

                        memcpy(&payload[0], &speed_target, sizeof(speed_target));
                        memcpy(&payload[4], &legacy_position_target, sizeof(legacy_position_target));
                        send_readback_response(dat_Uart1.pRxBuf[0], payload, sizeof(payload));
                    }
                }
            } else if (dat_Uart1.LEN == 5) {
                if (crc8_MAXIM(dat_Uart1.pRxBuf, 4) == dat_Uart1.pRxBuf[4] && dat_Uart1.pRxBuf[1] == motor_id &&
                    dat_Uart1.pRxBuf[0] == 0x62) {
                    HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                    usart_tx_delay = HAL_GetTick();
                    usart_tx_flag  = 1;
                    for (int i = 0; i < 25; i++) {
                        dat_Uart1.pTxBuf[i] = 0;
                    }
                    uint8_t temp_data[16] = {0};
                    uint8_t i2c_address   = dat_Uart1.pRxBuf[2];
                    i2c_address           = ((i2c_address << 1) | 1);
                    uint8_t i2c_len       = dat_Uart1.pRxBuf[3];
                    uint16_t i2c_success  = 0;
                    if (i2c_len > 16) i2c_len = 16;
                    LL_I2C_Disable(I2C1);
                    I2C1_Start();
                    i2c_success = I2C1_ReceiveData(i2c_address, temp_data, i2c_len, 10);
                    i2c_success = !i2c_success;

                    dat_Uart1.pTxBuf[0]     = 0xAA;
                    dat_Uart1.pTxBuf[1]     = 0x55;
                    dat_Uart1.pTxBuf[0 + 2] = 0x72;
                    dat_Uart1.pTxBuf[1 + 2] = motor_id;
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &i2c_success, 2);
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[4 + 2], &i2c_len, 1);
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[8 + 2], &temp_data, 16);
                    dat_Uart1.pTxBuf[24 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 24);
                    UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 27);
                }
            } else if (dat_Uart1.LEN == 25) {
                if (crc8_MAXIM(dat_Uart1.pRxBuf, 24) == dat_Uart1.pRxBuf[24] && dat_Uart1.pRxBuf[1] == motor_id &&
                    dat_Uart1.pRxBuf[0] == 0x63) {
                    HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                    usart_tx_delay = HAL_GetTick();
                    usart_tx_flag  = 1;
                    for (int i = 0; i < 6; i++) {
                        dat_Uart1.pTxBuf[i] = 0;
                    }
                    uint8_t i2c_address  = dat_Uart1.pRxBuf[2];
                    i2c_address          = (i2c_address << 1);
                    uint16_t i2c_len     = 0;
                    uint32_t i2c_success = 0;
                    i2c_len              = dat_Uart1.pRxBuf[3];
                    uint8_t is_stop_bit  = dat_Uart1.pRxBuf[4];
                    if (i2c_len <= 16) {
                        LL_I2C_Disable(I2C1);
                        I2C1_Start();
                        if (is_stop_bit)
                            i2c_success = I2C1_TransmitData(i2c_address, &dat_Uart1.pRxBuf[8], i2c_len, 10);
                        else
                            i2c_success =
                                I2C1_TransmitData_RepeatedStart(i2c_address, &dat_Uart1.pRxBuf[8], i2c_len, 10);
                        i2c_success = !i2c_success;
                    }

                    dat_Uart1.pTxBuf[0]     = 0xAA;
                    dat_Uart1.pTxBuf[1]     = 0x55;
                    dat_Uart1.pTxBuf[0 + 2] = 0x73;
                    dat_Uart1.pTxBuf[1 + 2] = motor_id;
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &i2c_success, 1);
                    dat_Uart1.pTxBuf[3 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 3);
                    UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 6);
                } else if (crc8_MAXIM(dat_Uart1.pRxBuf, 24) == dat_Uart1.pRxBuf[24] &&
                           dat_Uart1.pRxBuf[1] == motor_id && dat_Uart1.pRxBuf[0] == 0x61) {
                    HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                    usart_tx_delay = HAL_GetTick();
                    usart_tx_flag  = 1;
                    for (int i = 0; i < 6; i++) {
                        dat_Uart1.pTxBuf[i] = 0;
                    }
                    uint8_t i2c_address     = dat_Uart1.pRxBuf[2];
                    i2c_address             = (i2c_address << 1);
                    uint8_t i2c_address_len = dat_Uart1.pRxBuf[3];
                    uint16_t i2c_len        = 0;
                    uint32_t i2c_success    = 0;
                    uint8_t i2c_reg         = 0;
                    if (!i2c_address_len) {
                        i2c_reg = dat_Uart1.pRxBuf[4];
                        i2c_len = dat_Uart1.pRxBuf[6];
                        if (i2c_len > 16) i2c_len = 16;
                        LL_I2C_Disable(I2C1);
                        I2C1_Start();
                        i2c_success = I2C_Write_Bytes(i2c_address, i2c_reg, &dat_Uart1.pRxBuf[8], i2c_len, 10);
                        i2c_success = !i2c_success;
                    } else {
                        i2c_reg = (dat_Uart1.pRxBuf[4] | (dat_Uart1.pRxBuf[5] << 8));
                        i2c_len = dat_Uart1.pRxBuf[6];
                        if (i2c_len > 16) i2c_len = 16;
                        LL_I2C_Disable(I2C1);
                        I2C1_Start();
                        i2c_success =
                            I2C_Write_16bits_reg_Bytes(i2c_address, i2c_reg, &dat_Uart1.pRxBuf[8], i2c_len, 10);
                        i2c_success = !i2c_success;
                    }

                    dat_Uart1.pTxBuf[0]     = 0xAA;
                    dat_Uart1.pTxBuf[1]     = 0x55;
                    dat_Uart1.pTxBuf[0 + 2] = 0x71;
                    dat_Uart1.pTxBuf[1 + 2] = motor_id;
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &i2c_success, 1);
                    dat_Uart1.pTxBuf[3 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 3);
                    UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 6);
                }
            } else if (dat_Uart1.LEN == 8) {
                if (crc8_MAXIM(dat_Uart1.pRxBuf, 7) == dat_Uart1.pRxBuf[7] && dat_Uart1.pRxBuf[1] == motor_id &&
                    dat_Uart1.pRxBuf[0] == 0x60) {
                    HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_SET);
                    usart_tx_delay = HAL_GetTick();
                    usart_tx_flag  = 1;
                    for (int i = 0; i < 25; i++) {
                        dat_Uart1.pTxBuf[i] = 0;
                    }
                    uint8_t temp_data[16]   = {0};
                    uint8_t i2c_address     = dat_Uart1.pRxBuf[2];
                    i2c_address             = ((i2c_address << 1) | 1);
                    uint8_t i2c_address_len = dat_Uart1.pRxBuf[3];
                    uint16_t i2c_len        = 0;
                    uint16_t i2c_success    = 0;
                    if (!i2c_address_len) {
                        uint8_t i2c_reg = dat_Uart1.pRxBuf[4];
                        i2c_len         = dat_Uart1.pRxBuf[6];
                        if (i2c_len > 16) i2c_len = 16;
                        LL_I2C_Disable(I2C1);
                        I2C1_Start();
                        i2c_success = I2C_Read_Bytes(i2c_address, i2c_reg, temp_data, i2c_len, 10);
                        i2c_success = !i2c_success;
                    } else {
                        uint16_t i2c_reg = (dat_Uart1.pRxBuf[4] | (dat_Uart1.pRxBuf[5] << 8));
                        i2c_len          = dat_Uart1.pRxBuf[6];
                        if (i2c_len > 16) i2c_len = 16;
                        LL_I2C_Disable(I2C1);
                        I2C1_Start();
                        i2c_success = I2C_Read_16bits_reg_Bytes(i2c_address, i2c_reg, temp_data, i2c_len, 10);
                        i2c_success = !i2c_success;
                    }

                    dat_Uart1.pTxBuf[0]     = 0xAA;
                    dat_Uart1.pTxBuf[1]     = 0x55;
                    dat_Uart1.pTxBuf[0 + 2] = 0x70;
                    dat_Uart1.pTxBuf[1 + 2] = motor_id;
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[2 + 2], &i2c_success, 2);
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[4 + 2], &i2c_len, 1);
                    memcpy((uint8_t *)&dat_Uart1.pTxBuf[8 + 2], &temp_data, 16);
                    dat_Uart1.pTxBuf[24 + 2] = crc8_MAXIM((uint8_t *)&dat_Uart1.pTxBuf[0 + 2], 24);
                    UART_DMA_Send(USART3, dat_Uart1.pTxBuf, 27);
                }
            }
        }
    }
    if (LL_USART_IsActiveFlag_TC(USARTx)) {
        LL_USART_DisableIT_TC(USARTx);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
        HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_RESET);
        if (change_baudrate_flag) {
            LL_USART_DeInit(USART3);
            user_usart3_uart_init();
            hard_uart_begin();
            change_baudrate_flag = 0;
        }
        usart_tx_flag = 0;
    }
}
