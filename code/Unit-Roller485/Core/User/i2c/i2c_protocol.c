#include "i2c_protocol.h"

#include <string.h>

#include "adc.h"
#include "dma.h"
#include "encoder.h"
#include "flash.h"
#include "i2c.h"
#include "i2c_ex.h"
#include "main.h"
#include "motordriver.h"
#include "myadc.h"
#include "mysys.h"
#include "rgb.h"
#include "smart_knob.h"
#include "spi.h"
#include "stm32g4xx_ll_utils.h"
#include "tim.h"
#include "usart.h"
#include "u8g2_disp_fun.h"

#define FIRMWARE_VERSION   (2)
#define UID_REG_ADDR_START (0xE0)
#define UID_REG_ADDR_END   (0xEB)
#define UID_REG_LENGTH     (UID_REG_ADDR_END - UID_REG_ADDR_START + 1)

volatile uint8_t fm_version          = FIRMWARE_VERSION;
static uint8_t g_uid[UID_REG_LENGTH] = {0};

static void read_uid(void)
{
    uint32_t uid0 = LL_GetUID_Word0();
    uint32_t uid1 = LL_GetUID_Word1();
    uint32_t uid2 = LL_GetUID_Word2();

    memcpy(&g_uid[0], &uid0, sizeof(uid0));
    memcpy(&g_uid[4], &uid1, sizeof(uid1));
    memcpy(&g_uid[8], &uid2, sizeof(uid2));
}

static uint8_t copy_i2c_register_write(uint8_t *data, uint8_t *mark, const uint8_t *rx_data, uint16_t len,
                                       uint8_t first_register, uint8_t last_register)
{
    uint16_t offset         = rx_data[0] - first_register;
    uint16_t data_len       = len - 1;
    uint16_t register_count = (uint16_t)last_register - first_register + 1;

    if (offset >= register_count || data_len > register_count - offset) {
        return 0;
    }

    memcpy(&data[offset], &rx_data[1], data_len);
    memset(&mark[offset], 1, data_len);
    return 1;
}

static void update_le32_bytes(void *destination, const uint8_t *data, const uint8_t *mark)
{
    uint8_t *destination_bytes = destination;

    for (uint8_t i = 0; i < 4; i++) {
        if (mark[i]) {
            destination_bytes[i] = data[i];
        }
    }
}

void i2c_protocol_init(void)
{
    read_uid();
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len)
{
    uint8_t rx_buf[16]  = {0};
    uint8_t tx_buf[16]  = {0};
    uint8_t rx_mark[16] = {0};

    if (len > 1) {
        if (rx_data[0] <= 0x0F) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x00, 0x0F)) {
                return;
            }

            if (rx_mark[0]) {
                motor_output = rx_data[1];
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
            }

            if (rx_mark[1]) {
                if (rx_buf[1] && rx_buf[1] < MODE_MAX && !err_stalled_flag) {
                    motor_mode = rx_buf[1];
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
                            PIDTuningsSet(&pid_ctrl_speed_t, speed_pid_plus_float[0], speed_pid_plus_float[1],
                                          speed_pid_plus_float[2]);
                            PIDTuningsSet(&pid_ctrl_pos_t, pos_pid_plus_float[0], pos_pid_plus_float[1],
                                          pos_pid_plus_float[2]);
                            pid_ctrl_speed_t.iTerm = 0;
                            pid_ctrl_pos_t.iTerm   = 0;
                        }
                        last_motor_mode = motor_mode;
                    }
                }
            }

            if (rx_mark[14]) {
                if (rx_buf[14]) {
                    mode_switch_flag = 1;
                } else {
                    mode_switch_flag = 0;
                }
            }

            if (rx_mark[10]) {
                if (rx_buf[10]) {
                    motor_overvalue_protection_flag = 1;
                } else {
                    motor_overvalue_protection_flag = 0;
                }
            }

            if (rx_mark[15]) {
                if (rx_buf[15]) {
                    motor_stall_protection_flag = 1;
                } else {
                    motor_stall_protection_flag = 0;
                }
            }

            if (rx_mark[11] && rx_buf[11]) {
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
        } else if (rx_data[0] == 0xF1) {
            if (rx_data[1]) {
                if (!over_vol_flag) MotorDriverSetMode(MDRV_MODE_ENC_CAL);
            }
        } else if (rx_data[0] == 0xF2) {
            if (rx_data[1]) {
                // set encoder offset 配置编码器偏移，此处实际使用需要校准之后开机均从flash读取
                angle_cal_offset = GetMotorDriverEncCalOffset();
                MotorDriverSetAngleOffset(angle_cal_offset);
                flash_data_write_back();
            }
        } else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x40, 0x43)) {
                return;
            }

            update_le32_bytes(&speed_point, rx_buf, rx_mark);

            if (speed_point > MY_INT32_MAX)
                speed_point = MY_INT32_MAX;
            else if (speed_point < MY_INT32_MIN)
                speed_point = MY_INT32_MIN;

            pid_ctrl_speed_t.setpoint = (float)speed_point / 100.0f;
        } else if (rx_data[0] >= 0x80 && rx_data[0] <= 0x83) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x80, 0x83)) {
                return;
            }

            update_le32_bytes(&pos_point, rx_buf, rx_mark);

            if (pos_point > MY_INT32_MAX)
                pos_point = MY_INT32_MAX;
            else if (pos_point < MY_INT32_MIN)
                pos_point = MY_INT32_MIN;

            set_pos_point_legacy_cdeg(pos_point);
        } else if (rx_data[0] >= 0x88 && rx_data[0] <= 0x8D) {
            int32_t target_turns;
            uint16_t target_angle_cdeg;

            get_target_position_snapshot(&target_turns, &target_angle_cdeg);
            memcpy(&rx_buf[0], &target_turns, sizeof(target_turns));
            memcpy(&rx_buf[4], &target_angle_cdeg, sizeof(target_angle_cdeg));

            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x88, 0x8D)) {
                return;
            }

            memcpy(&target_turns, &rx_buf[0], sizeof(target_turns));
            memcpy(&target_angle_cdeg, &rx_buf[4], sizeof(target_angle_cdeg));
            if (target_angle_cdeg > 35999U) {
                return;
            }

            set_pos_point_turns_angle(target_turns, (float)target_angle_cdeg / 100.0f);
        } else if (rx_data[0] >= 0xB0 && rx_data[0] <= 0xB3) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0xB0, 0xB3)) {
                return;
            }

            update_le32_bytes(&current_point, rx_buf, rx_mark);

            if (current_point > 120000)
                current_point = 120000;
            else if (current_point < -120000)
                current_point = -120000;

            float current_set = (float)current_point / 100.0f;

            MotorDriverSetCurrentReal(current_set);
        } else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x12) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x10, 0x12)) {
                return;
            }
            if (rx_mark[0]) {
                motor_id = rx_buf[0];
            }
            if (rx_mark[1]) {
                if (rx_buf[1] <= 2) {
                    bps_index = rx_buf[1];
                }
            }
            if (rx_mark[2]) {
                if (rx_buf[2] <= 100) {
                    brightness_index = rx_buf[2];
                    ws2812_show();
                }
            }
        } else if (rx_data[0] >= 0x50 && rx_data[0] <= 0x53) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x50, 0x53)) {
                return;
            }

            update_le32_bytes(&max_speed_current, rx_buf, rx_mark);
            max_speed_current = apply_pid_current_limit(&pid_ctrl_speed_t, max_speed_current);
        } else if (rx_data[0] >= 0x20 && rx_data[0] <= 0x23) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x20, 0x23)) {
                return;
            }

            update_le32_bytes(&max_pos_current, rx_buf, rx_mark);
            max_pos_current = apply_pid_current_limit(&pid_ctrl_pos_t, max_pos_current);
        } else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x7B) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x70, 0x7B)) {
                return;
            }

            update_le32_bytes(&speed_pid_int[0], &rx_buf[0], &rx_mark[0]);
            update_le32_bytes(&speed_pid_int[1], &rx_buf[4], &rx_mark[4]);
            update_le32_bytes(&speed_pid_int[2], &rx_buf[8], &rx_mark[8]);

            for (int i = 0; i < 3; i += 2) {
                speed_pid_float[i] = (float)speed_pid_int[i] / 100000;
            }
            speed_pid_float[1] = (float)speed_pid_int[1] / 10000000;
            if (speed_pid_index == 0) {
                PIDTuningsSet(&pid_ctrl_speed_t, speed_pid_float[0], speed_pid_float[1], speed_pid_float[2]);
                pid_ctrl_speed_t.iTerm = 0;
            }
        } else if (rx_data[0] >= 0xA0 && rx_data[0] <= 0xAB) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0xA0, 0xAB)) {
                return;
            }

            update_le32_bytes(&pos_pid_int[0], &rx_buf[0], &rx_mark[0]);
            update_le32_bytes(&pos_pid_int[1], &rx_buf[4], &rx_mark[4]);
            update_le32_bytes(&pos_pid_int[2], &rx_buf[8], &rx_mark[8]);

            for (int i = 0; i < 3; i += 2) {
                pos_pid_float[i] = (float)pos_pid_int[i] / 100000;
            }
            pos_pid_float[1] = (float)pos_pid_int[1] / 10000000;
            if (pos_pid_index == 0) {
                PIDTuningsSet(&pid_ctrl_pos_t, pos_pid_float[0], pos_pid_float[1], pos_pid_float[2]);
                pid_ctrl_pos_t.iTerm = 0;
            }
        } else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x33)) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x30, 0x33)) {
                return;
            }
            if (rx_mark[0]) {
                rgb_color_buffer[rgb_color_buffer_index] &= ~0x000000ff;
                rgb_color_buffer[rgb_color_buffer_index] |= rx_buf[0];
            }
            if (rx_mark[1]) {
                rgb_color_buffer[rgb_color_buffer_index] &= ~0x0000ff00;
                rgb_color_buffer[rgb_color_buffer_index] |= (rx_buf[1] << 8);
            }
            if (rx_mark[2]) {
                rgb_color_buffer[rgb_color_buffer_index] &= ~0x00ff0000;
                rgb_color_buffer[rgb_color_buffer_index] |= (rx_buf[2] << 16);
            }
            if (rx_mark[3]) {
                if (rx_buf[3])
                    rgb_show_mode = 1;
                else
                    rgb_show_mode = 0;
            }
            if (rx_mark[0] || rx_mark[1] || rx_mark[2]) {
                lastest_rgb_color = rgb_color_buffer[rgb_color_buffer_index];
                if (rgb_color_buffer_index < (RGB_BUFFER_SIZE - 1)) ++rgb_color_buffer_index;
            }
        } else if (rx_data[0] >= 0x3C && rx_data[0] <= 0x3F) {
            if (!copy_i2c_register_write(rx_buf, rx_mark, rx_data, len, 0x3C, 0x3F)) {
                return;
            }

            update_le32_bytes(&current_position, rx_buf, rx_mark);
        } else if (rx_data[0] == 0xF0) {
            if (rx_data[1] == 1) {
                flash_data_write_back();
            }
        } else if (rx_data[0] == 0xFF) {
            if (rx_data[1] >= 0x08 && rx_data[1] <= 0x77 && i2c_address[0] != rx_data[1]) {
                i2c_address[0] = rx_data[1];
                flash_data_write_back();
                user_i2c_init();
            }
        } else if (rx_data[0] == 0xFD) {
            if (rx_data[1] == 1) {
                LL_I2C_DeInit(I2C1);
                LL_I2C_DisableAutoEndMode(I2C1);
                LL_I2C_Disable(I2C1);
                LL_I2C_DisableIT_ADDR(I2C1);
                HAL_TIM_Base_DeInit(&htim1);
                HAL_TIM_Base_DeInit(&htim3);
                HAL_TIM_Base_MspDeInit(&htim1);
                HAL_TIM_PWM_MspDeInit(&htim3);
                HAL_SPI_DeInit(&hspi1);
                HAL_SPI_MspDeInit(&hspi1);
                HAL_ADC_DeInit(&hadc1);
                HAL_ADC_MspDeInit(&hadc1);
                LL_USART_DeInit(USART3);
                LL_USART_DisableIT_IDLE(USART3);

                LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
                LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_2);

                LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
                LL_USART_DisableDMAReq_RX(USART3);
                HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
                HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
                HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
                HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
                HAL_ADC_Stop_DMA(&hadc1);
                NVIC_SystemReset();
            }
        }
    } else if (len == 1) {
        /* Unknown register reads return zero instead of reusing the previous response. */
        tx_buf[0] = 0;
        i2c1_set_send_data(&tx_buf[0], 1);

        if (rx_data[0] >= UID_REG_ADDR_START && rx_data[0] <= UID_REG_ADDR_END) {
            uint8_t uid_offset = rx_data[0] - UID_REG_ADDR_START;
            i2c1_set_send_data(&g_uid[uid_offset], UID_REG_LENGTH - uid_offset);
        } else if (rx_data[0] == 0xF3) {
            uint8_t cal_status = IsMotorDriverEncCalBusy();
            i2c1_set_send_data(&cal_status, 1);
        } else if (rx_data[0] == 0xF4) {
            tx_buf[0] = ROLLER_DEVICE_ID;
            i2c1_set_send_data(&tx_buf[0], 1);
        } else if (rx_data[0] <= 0x0F) {
            motor_output ? (motor_output = 1) : (motor_output = 0);
            uint8_t motor_mode_temp = motor_mode;
            if (motor_mode == MODE_SPEED_ERR_PROTECT) {
                motor_mode_temp = MODE_SPEED;
            } else if (motor_mode == MODE_POS_ERR_PROTECT) {
                motor_mode_temp = MODE_POS;
            }
            tx_buf[0]    = motor_output;
            tx_buf[1]    = motor_mode_temp;
            tx_buf[0x0A] = motor_overvalue_protection_flag;
            tx_buf[0x0C] = sys_status;
            tx_buf[0x0D] = error_code;
            tx_buf[0x0E] = mode_switch_flag;
            tx_buf[0x0F] = motor_stall_protection_flag;
            i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0]], 0x0F - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x10 && rx_data[0] <= 0x12) {
            tx_buf[0] = motor_id;
            tx_buf[1] = bps_index;
            tx_buf[2] = brightness_index;
            i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0] - 0x10], 0x12 - rx_data[0] + 1);
        } else if ((rx_data[0] >= 0x30) && (rx_data[0] <= 0x3F)) {
            int32_t vol_int32 = (int32_t)vol_lpf;
            memcpy(&tx_buf[0], (uint8_t *)&lastest_rgb_color, 3);
            tx_buf[3] = rgb_show_mode;
            memcpy(&tx_buf[4], (uint8_t *)&vol_int32, 4);
            memcpy(&tx_buf[8], (uint8_t *)&internal_temp, 4);
            memcpy(&tx_buf[12], (uint8_t *)&current_position, 4);
            i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0] - 0x30], 0x3F - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x40 && rx_data[0] <= 0x43) {
            memcpy(&tx_buf[0], &speed_point, sizeof(speed_point));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x40], 0x43 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x50 && rx_data[0] <= 0x53) {
            memcpy(&tx_buf[0], &max_speed_current, sizeof(max_speed_current));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x50], 0x53 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x20 && rx_data[0] <= 0x23) {
            memcpy(&tx_buf[0], &max_pos_current, sizeof(max_pos_current));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x20], 0x23 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x60 && rx_data[0] <= 0x63) {
            int32_t motor_rpm_int = 0;
            motor_rpm_int         = motor_rpm * 100;
            memcpy(&tx_buf[0], &motor_rpm_int, sizeof(motor_rpm_int));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x60], 0x63 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x80 && rx_data[0] <= 0x83) {
            memcpy(&tx_buf[0], &pos_point, sizeof(pos_point));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x80], 0x83 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x88 && rx_data[0] <= 0x8D) {
            int32_t target_turns;
            uint16_t target_angle_cdeg;

            get_target_position_snapshot(&target_turns, &target_angle_cdeg);
            memcpy(&tx_buf[0], &target_turns, sizeof(target_turns));
            memcpy(&tx_buf[4], &target_angle_cdeg, sizeof(target_angle_cdeg));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x88], 0x8D - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x90 && rx_data[0] <= 0x93) {
            int32_t mechanical_angle_int = get_legacy_position_cdeg();
            memcpy(&tx_buf[0], &mechanical_angle_int, sizeof(mechanical_angle_int));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x90], 0x93 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x98 && rx_data[0] <= 0x9D) {
            int32_t actual_turns;
            uint16_t actual_angle_cdeg;

            get_mechanical_position_snapshot(&actual_turns, &actual_angle_cdeg);
            memcpy(&tx_buf[0], &actual_turns, sizeof(actual_turns));
            memcpy(&tx_buf[4], &actual_angle_cdeg, sizeof(actual_angle_cdeg));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0x98], 0x9D - rx_data[0] + 1);
        } else if (rx_data[0] >= 0xB0 && rx_data[0] <= 0xB3) {
            memcpy(&tx_buf[0], &current_point, sizeof(current_point));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0xB0], 0xB3 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0xC0 && rx_data[0] <= 0xC3) {
            int32_t ph_current_int = ph_crrent_lpf * 100;
            memcpy(&tx_buf[0], &ph_current_int, sizeof(ph_current_int));
            i2c1_set_send_data(&tx_buf[rx_data[0] - 0xC0], 0xC3 - rx_data[0] + 1);
        } else if (rx_data[0] >= 0x70 && rx_data[0] <= 0x7B) {
            switch (speed_pid_index) {
                case 0:
                    memcpy(tx_buf, (uint8_t *)&speed_pid_int[0], 12);
                    break;
                case 1:
                    memcpy(tx_buf, (uint8_t *)&speed_pid_low_int[0], 12);
                    break;
                case 2:
                    memcpy(tx_buf, (uint8_t *)&speed_pid_mid_int[0], 12);
                    break;
                case 3:
                    memcpy(tx_buf, (uint8_t *)&speed_pid_high_int[0], 12);
                    break;

                default:
                    break;
            }

            i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0] - 0x70], 0x7B - rx_data[0] + 1);
        } else if (rx_data[0] >= 0xA0 && rx_data[0] <= 0xAB) {
            switch (pos_pid_index) {
                case 0:
                    memcpy(tx_buf, (uint8_t *)&pos_pid_int[0], 12);
                    break;
                case 1:
                    memcpy(tx_buf, (uint8_t *)&pos_pid_low_int[0], 12);
                    break;
                case 2:
                    memcpy(tx_buf, (uint8_t *)&pos_pid_mid_int[0], 12);
                    break;
                case 3:
                    memcpy(tx_buf, (uint8_t *)&pos_pid_high_int[0], 12);
                    break;
            }

            i2c1_set_send_data((uint8_t *)&tx_buf[rx_data[0] - 0xA0], 0xAB - rx_data[0] + 1);
        } else if (rx_data[0] == 0xFE) {
            i2c1_set_send_data((uint8_t *)&fm_version, 1);
        } else if (rx_data[0] == 0xFF) {
            i2c1_set_send_data((uint8_t *)&i2c_address[0], 1);
        }
    }
}
