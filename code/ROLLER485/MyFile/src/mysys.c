
/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "mysys.h"
#include "motordriver.h"
#include "myadc.h"
#include "tim.h"
#include "tle5012b.h"

#include "oled.h"
#include "bmp.h"

#include "arm_const_structs.h"

#include "oled_u8g2.h"
#include "u8g2_disp_fun.h"
#include "encoder.h"
#include "ws2812.h"
#include "i2c.h"
#include "i2c_ex.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "pid_controller.h"
#include "smart_knob.h"
#include "arm_math.h"
#include "i2c.h"

#define MAX_RECORD_SIZE 256
#define MAX_STALLED_CURRENT 500

uint16_t counter_loop_foc,counter_loop_control;

uint16_t counter_rpm = 0, pid_compute_counter = 0;

float32_t vol_input,vol_lpf;

float32_t ph_current_rt,ph_crrent_lpf;
uint16_t disp_ph_current;

int32_t adc_current_pha,adc_current_phb,adc_current_phc;
uint16_t abs_pha,abs_phb,abs_phc,ph_current_max;

float32_t encoder_absolute_angle_new,encoder_absolute_angle_old;
float32_t mechanical_angle,mechanical_turns,last_mechanical_turns; 
float32_t mechanical_rad; 
int32_t diff_encoder_value = 0;
float32_t diff_encoder_value_lpf = 0.0f;
float32_t pos_output_lpf = 0.0f;
float32_t motor_rpm; 
float32_t motor_rps; 
float32_t angle_target,angle_error; 

float32_t angle_kp,angle_ki,angle_kd;
float32_t uq_limit;
uint16_t dbg1,dbg2,dbg3;

float32_t debug=4.0f;

uint32_t act_delay = 0;
uint32_t status_flag = 0;

uint8_t speed_pid_index = 0;
uint32_t speed_pid_int[3] = {1500000, 1000, 40000000};
float speed_pid_float[3] = {15.0f, 0.0001f, 400.0f};

uint32_t speed_pid_low_int[3] = {1500000, 1000, 40000000};
float speed_pid_low_float[3] = {15.0f, 0.0001f, 400.0f};
uint32_t speed_pid_mid_int[3] = {2500000, 30, 20000000};
float speed_pid_mid_float[3] = {25.0f, 0.000003f, 200.0f};
uint32_t speed_pid_high_int[3] = {2500000, 30, 40000000};
float speed_pid_high_float[3] = {25.0f, 0.000003f, 400.0f};

uint8_t pos_pid_index = 0;
uint32_t pos_pid_int[3] = {1500000, 30, 40000000};
float pos_pid_float[3] = {15.0f, 0.000003f, 400.0f};

uint32_t pos_pid_low_int[3] = {1500000, 30, 40000000};
float pos_pid_low_float[3] = {15.0f, 0.000003f, 400.0f};
uint32_t pos_pid_mid_int[3] = {1500000, 1, 400000000};
float pos_pid_mid_float[3] = {15.0f, 0.0000001f, 4000.0f};
uint32_t pos_pid_high_int[3] = {1500000, 1, 800000000};
float pos_pid_high_float[3] = {15.0f, 0.0000001f, 8000.0f};

uint32_t speed_pid_plus_int[3] = {1500000, 1000, 40000000};
float speed_pid_plus_float[3] = {15.0f, 0.0001f, 400.0f};
uint32_t pos_pid_plus_int[3] = {1200000, 100000, 200000000};
float pos_pid_plus_float[3] = {12.0f, 0.01f, 2000.0f};

PIDControl pid_ctrl_speed_t;
PIDControl pid_ctrl_pos_t;
int32_t speed_point = 0;
int32_t max_speed_current = 100000;
int32_t max_pos_current = 100000;
int32_t pos_point = 0;
int32_t current_point = 0;
float current_point_float = 0.0f;

uint8_t error_code = ERR_NONE;
uint8_t over_vol_protect_mode = 0;
uint8_t over_vol_protect_auto_flag = 0;
uint32_t over_vol_protect_auto_counter = 0;
uint8_t over_vol_flag = 0;

uint8_t over_value_flag = 0;

uint8_t err_recover_try_max = 5;
uint8_t err_stalled_flag = 0;
uint16_t speed_err_value = 0;
uint8_t speed_err_timeout = 0;
uint8_t speed_err_count_flag = 0;
uint8_t speed_err_auto_flag = 0;
uint32_t speed_err_counter = 0;
uint32_t speed_err_auto_counter = 0;
uint32_t speed_err_recover_counter = 0;
uint32_t speed_err_recover_try_counter = 0;

float speed_err_rate = 0.8f;
float pos_err_rate = 0.4f;

uint16_t pos_err_value = 0;
uint8_t pos_err_timeout = 0;
uint8_t pos_err_count_flag = 0;
uint8_t pos_err_auto_flag = 0;
uint32_t pos_err_counter = 0;
uint32_t pos_err_auto_counter = 0;
uint32_t pos_err_recover_counter = 0;
uint32_t pos_err_recover_try_counter = 0;

uint8_t sys_status = SYS_STANDBY;
uint8_t running_index = 0;

volatile uint32_t i2c_stop_timeout_delay = 0;

volatile uint32_t usart_tx_delay = 0;
uint8_t usart_tx_flag = 0;

uint8_t dis_show_flag = DIS_INFO;
uint8_t last_dis_show_flag = DIS_INFO;

uint8_t motor_mode = MODE_SPEED;
uint8_t last_motor_mode = MODE_SPEED;

uint8_t motor_id = 0;

uint16_t angle_cal_offset = 0;

uint8_t motor_output = 0;

float speed_record[MAX_RECORD_SIZE] = {0};
uint8_t record_index = 0;
volatile uint8_t avg_filter_level = 20;
float32_t uq_output = 0.0f;
float diff_pos_debug = 0.0f;

uint8_t mode_switch_flag = 0;
uint8_t motor_stall_protection_flag = 1;
uint8_t motor_overvalue_protection_flag = 0;

float rpm_rps_count_temp = 0;

uint32_t bps_list[3] = {115200, 19200, 9600};
uint8_t bps_index = 0;
float brightness_list[4] = {1.0f, 0.5f, 0.2f, 0.0f};
uint8_t brightness_index = 100;

uint8_t rgb_show_mode = 0;
uint32_t rgb_color_buffer[RGB_BUFFER_SIZE] = {0};
uint32_t rgb_color_buffer_index = 0;
uint32_t lastest_rgb_color = 0;

void Rpm_Count_100us(void);

void _sys_exit(int x){x = x;}

void init_pid(void)
{
  switch (speed_pid_index)
  {
  case 0:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_float[0], 
        speed_pid_float[1], 
        speed_pid_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  case 1:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_low_float[0], 
        speed_pid_low_float[1], 
        speed_pid_low_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  case 2:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_mid_float[0], 
        speed_pid_mid_float[1], 
        speed_pid_mid_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  case 3:
    PIDInit(&pid_ctrl_speed_t, 
        speed_pid_high_float[0], 
        speed_pid_high_float[1], 
        speed_pid_high_float[2], 1.0f, 
        -(float)max_speed_current/100, (float)max_speed_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_speed_t.setpoint = 0;
    break;
  
  default:
    break;
  }

  switch (pos_pid_index)
  {
  case 0:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_float[0], 
        pos_pid_float[1], 
        pos_pid_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_pos_t.setpoint = 0;  
    break;
  case 1:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_low_float[0], 
        pos_pid_low_float[1], 
        pos_pid_low_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_pos_t.setpoint = 0;  
    break;
  case 2:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_mid_float[0], 
        pos_pid_mid_float[1], 
        pos_pid_mid_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
    pid_ctrl_pos_t.setpoint = 0;  
    break;
  case 3:
    PIDInit(&pid_ctrl_pos_t, 
        pos_pid_high_float[0], 
        pos_pid_high_float[1], 
        pos_pid_high_float[2], 1.0f, 
        -(float)max_pos_current/100, (float)max_pos_current/100, AUTOMATIC, DIRECT);  
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
    if (fabsf(pid_ctrl_speed_t.point_error/pid_ctrl_speed_t.setpoint > 0.5f)) {
      rgb_flash_slow = 0;
    }
    else {
      rgb_flash_slow = 1;   
    }  
  }
  if (0) {
    pid_ctrl_speed_t.iTerm = 0;
    MotorDriverSetCurrentReal(0.0f);
  }
  else {
    PIDCompute(&pid_ctrl_speed_t);
    if (speed_err_value) {
      // 第一次堵转判断
      if (!speed_err_auto_flag) {
        if (fabsf(pid_ctrl_speed_t.point_error) >= speed_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (!speed_err_count_flag) {
            speed_err_counter = HAL_GetTick();
            speed_err_count_flag = 1;
          }
        }
        else {
          if (speed_err_count_flag == 1)
            speed_err_count_flag = 0;
        }

        // 跳转到堵转保护模式，2S后自动恢复
        if (speed_err_count_flag == 1 && HAL_GetTick() - speed_err_counter > speed_err_timeout * 1000) {
          motor_mode = MODE_SPEED_ERR_PROTECT;
          error_code |= ERR_STALLED;
          MotorDriverSetMode(MDRV_MODE_OFF);
          pid_ctrl_speed_t.iTerm = 0;
          speed_err_auto_counter = HAL_GetTick();
          speed_err_auto_flag = 1;
          return;
        }        
      }
      // 自动恢复后判断是否有堵转
      else {
        if (speed_err_count_flag == 2) {
          speed_err_recover_counter = HAL_GetTick();
          speed_err_count_flag = 1;
        }
        if (fabsf(pid_ctrl_speed_t.point_error) >= speed_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (HAL_GetTick() - speed_err_recover_counter > 500) {
            motor_mode = MODE_SPEED_ERR_PROTECT;
            error_code |= ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
            pid_ctrl_speed_t.iTerm = 0;
            speed_err_auto_counter = HAL_GetTick();          
          }
        }
        else {
          if (HAL_GetTick() - speed_err_recover_counter > 500) {
            speed_err_recover_try_counter = 0;
            error_code &= ~ERR_STALLED;
            speed_err_count_flag = 0;
            speed_err_auto_flag = 0; 
            err_stalled_flag = 0;       
          }          
        }
        if (motor_disable_flag) {
          if (motor_mode == MODE_SPEED_ERR_PROTECT) {
            MotorDriverSetMode(MDRV_MODE_OFF);
            err_stalled_flag = 1;
          }
          else {
            error_code &= ~ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
          }
          motor_disable_flag = 0;
        }        
      }

      MotorDriverSetCurrentReal(pid_ctrl_speed_t.output);
    }
    else {
      if (!err_stalled_flag) {
        speed_err_count_flag = 0;
        error_code &= ~ERR_STALLED;
        speed_err_auto_flag = 0;
      }
      else {
        pid_ctrl_speed_t.iTerm = 0;
      }
      MotorDriverSetCurrentReal(pid_ctrl_speed_t.output);
    }
  }
}

void pos_pid(void)
{
  static uint8_t i_overflow_count_flag = 0;
  static uint8_t i_overflow_counter = 0;

  pid_ctrl_pos_t.input = mechanical_angle;
  if ((pid_ctrl_pos_t.point_error <= 360.0f || pid_ctrl_pos_t.point_error >= -360.0f) && 
  (pid_ctrl_pos_t.iTerm > 30.0f || pid_ctrl_pos_t.iTerm < -30.0f) &&
  (pid_ctrl_pos_t.output < 30.0f || pid_ctrl_pos_t.output > -30.0f) && !i_overflow_count_flag) {
    i_overflow_count_flag = 1;
    i_overflow_counter = HAL_GetTick();
  }

  if (i_overflow_count_flag && HAL_GetTick() - i_overflow_counter > 100) {
    pid_ctrl_pos_t.iTerm = 0;
    i_overflow_count_flag = 0;
  } 
  if (sys_status == SYS_RUNNING) {
    if (fabsf(pid_ctrl_pos_t.point_error) > 10.0f) {
      rgb_flash_slow = 0;
    }  
    else {
      rgb_flash_slow = 1;
    }    
  }
  PIDCompute(&pid_ctrl_pos_t);
    if (pos_err_value) {
      // 第一次堵转判断
      if (!pos_err_auto_flag) {
        if (fabsf(pid_ctrl_pos_t.point_error) >= pos_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (!pos_err_count_flag) {
            pos_err_counter = HAL_GetTick();
            pos_err_count_flag = 1;
          }
        }
        else {
          if (pos_err_count_flag == 1)
            pos_err_count_flag = 0;
        }

        // 跳转到堵转保护模式，2S后自动恢复
        if (pos_err_count_flag == 1 && HAL_GetTick() - pos_err_counter > pos_err_timeout * 1000) {
          motor_mode = MODE_POS_ERR_PROTECT;
          error_code |= ERR_STALLED;
          MotorDriverSetMode(MDRV_MODE_OFF);
          pid_ctrl_pos_t.iTerm = 0;
          pos_err_auto_counter = HAL_GetTick();
          pos_err_auto_flag = 1;
          return;
        }        
      }
      // 自动恢复后判断是否有堵转
      else {
        if (pos_err_count_flag == 2) {
          pos_err_recover_counter = HAL_GetTick();
          pos_err_count_flag = 1;
        }
        if (fabsf(pid_ctrl_pos_t.point_error) >= pos_err_value && fabsf(ph_crrent_lpf) >= MAX_STALLED_CURRENT) {
          if (HAL_GetTick() - pos_err_recover_counter > 500) {
            motor_mode = MODE_POS_ERR_PROTECT;
            error_code |= ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
            pid_ctrl_pos_t.iTerm = 0;
            pos_err_auto_counter = HAL_GetTick();          
          }
        }
        else {
          if (HAL_GetTick() - pos_err_recover_counter > 500) {
            pos_err_recover_try_counter = 0;
            error_code &= ~ERR_STALLED;
            pos_err_count_flag = 0;
            pos_err_auto_flag = 0; 
            err_stalled_flag = 0;       
          }          
        }
        if (motor_disable_flag) {
          if (motor_mode == MODE_POS_ERR_PROTECT) {
            MotorDriverSetMode(MDRV_MODE_OFF);
            err_stalled_flag = 1;
          }
          else {
            error_code &= ~ERR_STALLED;
            MotorDriverSetMode(MDRV_MODE_OFF);
          }
          motor_disable_flag = 0;
        }         
      }

      MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
    }
    else {
      if (!err_stalled_flag) {
        pos_err_count_flag = 0;
        error_code &= ~ERR_STALLED;
        pos_err_auto_flag = 0;
      }
      else {
        pid_ctrl_pos_t.iTerm = 0;
      }
      MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
    }  
  MotorDriverSetCurrentReal(pid_ctrl_pos_t.output);
   
}

void pos_speed_pid(void)
{
  pid_ctrl_pos_t.input = mechanical_angle;
  PIDCompute(&pid_ctrl_pos_t);
  pos_output_lpf += (1.0f / (1.0f + 1.0f/(2.0f * 3.14f *0.00019642857f*2.0f)))*(pid_ctrl_pos_t.output - pos_output_lpf );
  pid_ctrl_speed_t.setpoint = pos_output_lpf;
}

uint8_t crc8_MAXIM(uint8_t *data, uint8_t len)
{
    uint8_t crc, i;
    crc = 0x00;

    while(len--)
    {
        crc ^= *data++;
        for(i = 0;i < 8;i++)
        {
            if(crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8c;
            }
                else crc >>= 1;
        }
    }
    return crc;
}

void InitMysys(void)
{

  encoder_absolute_angle_new=0;
  encoder_absolute_angle_old=0;
  mechanical_angle=0;
  mechanical_turns=0;
  angle_error=0;

  angle_target = 240.0f;
  angle_kp = 0.5f;

  uq_limit = 500.0f;

  counter_loop_foc=0;
  counter_loop_control=0;

  GPIOB->BSRR=1<<1;//enable DRV8311 to enable intrlnal current sensor

	MyADCInit();
	TIM1->CCR4=995;//Enable TIM1 CH4 for ADC trigger

	//Enable TIM1 channels for PWM generate
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  

	HAL_Delay(20);
  //TIM1 update interrupt for FOC and outside control loop
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

	MotorDriverInit();

	MyADCZeroCal();

	EncoderInit();
  HAL_Delay(300);

  // //wait for encoder calibration  等待编码器校准，此处临时使用阻塞式，实际使用不要阻塞
  // while(IsMotorDriverEncCalBusy())
  // {
  //   ;
  // }



  //start motor driver 配置驱动为正常输出模式
  // MotorDriverSetMode(MDRV_MODE_RUN);  

//以下为实际使用：
//正常运行时调用方式二选一均可

  
  //配置电机电流目标值，此为ADC值，范围-1500 ~ +1500,对应最大相电流约为+-1.2A(非母线电流，即电源输入电流)
  //数据类型int32_t

  //MotorDriverSetCurrentAdc(200);


  //配置电机电流目标值，此为实际值，范围-1200.0 ~ +1200.0, 对应最大相电流约为+-1.2A(非母线电流，即电源输入电流)
  //数据类型float32_t

  // MotorDriverSetCurrentReal(1000.0f);  
  init_flash_data();
  u8g2Init(&u8g2);  
  if (!HAL_GPIO_ReadPin(SYS_SW_GPIO_Port, SYS_SW_Pin)) {
    u8g2_disp_menu_init();
    u8g2_disp_menu_update();
  }  
  init_pid();
  if (comm_type == COMM_TYPE_I2C) {
    user_i2c_init();
    i2c1_it_enable();
  }
  else if (comm_type == COMM_TYPE_485) {
    user_usart3_uart_init();
    hard_uart_begin();
  }
  else if (comm_type == COMM_TYPE_485_I2C) {
    user_usart3_uart_init();
    user_i2c_init();
    I2C1_Start();
    hard_uart_begin();
  }  

  u8g2_disp_init();
  // pid_ctrl_speed_t.setpoint = 50;
  // pid_ctrl_pos_t.setpoint = 0.0f;
  // u8g2_disp_speed();
  // u8g2_disp_pos();
  // u8g2_disp_current();
  // u8g2_disp_all();
}




void LoopMysys(void)
{
    while(1)
    {	  
        // if (over_vol_protect_auto_flag && over_vol_protect_mode) {
        //   if (HAL_GetTick() - over_vol_protect_auto_counter > 2000) {
        //     over_vol_protect_auto_flag = 0;
        //     over_vol_flag = 0;
        //     error_code &= ~ERR_OVER_VOLTAGE;
        //     sys_status = SYS_STANDBY;
        //   }
        // }
        i2c_timeout_counter = 0;
        if (i2c_stop_timeout_flag) {
          if (i2c_stop_timeout_delay < HAL_GetTick()) {
            i2c_stop_timeout_counter++;
            i2c_stop_timeout_delay = HAL_GetTick() + 10;
          }
        }
        if (i2c_stop_timeout_counter > 50) {
          LL_I2C_DeInit(I2C1);
          LL_I2C_DisableAutoEndMode(I2C1);
          LL_I2C_Disable(I2C1);
          LL_I2C_DisableIT_ADDR(I2C1);     
          user_i2c_init();    
          i2c1_it_enable();
          HAL_Delay(500);
        }       
        // encoder_update();
        if (usart_fault_flag) {
          MX_USART3_UART_Init();
          hard_uart_begin();
          usart_fault_flag = 0;
        }
        button_update();
        u8g2_disp_update_mode();
        // u8g2_disp_update_status();
        u8g2_disp_update_page();
        u8g2_disp_update_comm();
        
        if (LL_USART_IsActiveFlag_ORE(USART3)) {
          LL_USART_ClearFlag_ORE(USART3);
          MX_USART3_UART_Init();
        }
        if (usart_tx_flag && HAL_GetTick() - usart_tx_delay >= 50) {
          HAL_GPIO_WritePin(GPIOB, RS485_DIR_Pin, GPIO_PIN_RESET);
          usart_tx_flag = 0;
        }
        if (my_button.was_click) {
          dis_show_flag++;
          if (dis_show_flag >= DIS_MAX)
            dis_show_flag = DIS_INFO;
          last_dis_show_flag = dis_show_flag;
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
        //get input voltage
        if(ph_crrent_lpf<0)
        {
          disp_ph_current = (uint16_t)(-ph_crrent_lpf);
        }
        else
        {
          disp_ph_current = (uint16_t)(ph_crrent_lpf);
        }

        if (over_vol_flag) {
          dis_show_flag = DIS_OVP;
        }
        else if (err_stalled_flag) {
          dis_show_flag = DIS_STALL;
        }
        else if (over_value_flag) {
          dis_show_flag = DIS_OVER_VALUE;
        }
        else {
          dis_show_flag = last_dis_show_flag;
        }
        if (rgb_color_buffer_index && rgb_show_mode) {
          uint32_t rgb_show_index = rgb_color_buffer_index;
          for (uint32_t i = 0; i < rgb_show_index; i++) {
            neopixel_set_color(0, rgb_color_buffer[i]);
            neopixel_set_color(1, rgb_color_buffer[i]);
            ws2812_show();
          }
          rgb_color_buffer_index = 0;
        } 
        switch (dis_show_flag)
        {
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
        // u8g2_disp_update_status();
        // u8g2_disp_update_vin(vol_input);
        //voltage
        // OLED_ShowNum(30,16, vol_input/100,2,8,1);
        // OLED_ShowNum(48,16, vol_input/10%10,1,8,1);

        // //current
        // OLED_ShowNum(30,24, ph_current_max/1000,1,8,1);
        // OLED_ShowNum(42,24, ph_current_max/10%100,2,8,1);
          
        // //angle
        // OLED_ShowNum(30,32,MotorDriverGetMechanicalAngle()/10,3,8,1);
        // OLED_ShowNum(53,32,MotorDriverGetMechanicalAngle()%10,1,8,1);
        // OLED_Refresh();

        // if (angle_error > 0.2f) {
        //   neopixel_set_color(0, ((0x10 << 16) | (0 << 8) | 0));   
        //   neopixel_set_color(1, ((0x10 << 16) | (0 << 8) | 0));   

        //   ws2812_show();           
        // } else {
        //   neopixel_set_color(0, ((0 << 16) | (0x10 << 8) | 0));   
        //   neopixel_set_color(1, ((0 << 16) | (0x10 << 8) | 0));   

        //   ws2812_show();          
        // }
        
        if (act_delay < HAL_GetTick()) {
          running_index++;
          if (running_index > 1)
            running_index = 0;
          if (status_flag) {
            status_flag = 0;
            // OLED_ShowString(0,0," M5 BLDC  ",8,1);
          }
          else {
            status_flag = 1;
            // OLED_ShowString(0,0," M5 BLDC *",8,1);
          }
          // if (act_flag == 1) {
          //   angle_target += 360.0f;
          // } else if (act_flag == 3) {
          //   angle_target = 180.0f;
          // }   
					// if (angle_target > 540.0f) {
					// 	angle_target = 180.0f;
					// }
          act_delay = HAL_GetTick() + 1000;
        }
	}
}

void Loop_FOC(void)
{
  GPIOB->BSRR=GPIO_PIN_9;
  MotorDriverProcess();
  MyAdcProcess();
  GPIOB->BRR=GPIO_PIN_9;

}

void Loop_Control(void)
{

  encoder_absolute_angle_old = encoder_absolute_angle_new;
  encoder_absolute_angle_new = MotorDriverGetMechanicalAngle()/10.0f;

  if(encoder_absolute_angle_new<90.0f && encoder_absolute_angle_old>180.0f )
  {
    mechanical_turns += 1.0f;
  }

  if(encoder_absolute_angle_new>180.0f && encoder_absolute_angle_old<90.0f )
  {
    mechanical_turns -= 1.0f;
  }

    mechanical_angle =  (360.0f * mechanical_turns) + encoder_absolute_angle_new;

    mechanical_rad =  mechanical_angle * PI / 180.0f;

    // angle_error =  angle_target - mechanical_angle ;

    // uq_output = angle_error * angle_kp;

    // if(uq_output > uq_limit) uq_output = uq_limit;
    // if(uq_output < -uq_limit) uq_output = -uq_limit;

    // MotorDriverSetCurrentReal(uq_output);

    //lpfdata += (1.0 / (1.0 + 1.0/(2.0f * 3.14f *T*fc)))*(rawdata - lpfdata );
    //lpfdata ： 滤波后的数据。
    //rawdata ： 滤波前的原始数据。
    //T： 数据的采样频率的倒数，即采样周期，单位是秒。
    //fc : 截止频率。截止频率就是超过该频率的数据（噪声）都被过滤掉，只保留低于该截止频率的数据。

    ph_current_rt = MotorDriverGetPhaseCurrentReal();
    ph_crrent_lpf += (1.0f / (1.0f + 1.0f/(2.0f * 3.14f *0.0002f*2.0f)))*(ph_current_rt - ph_crrent_lpf );

    speed_encoder_update();

    diff_encoder_value = speed_encoder_value_t.encoder_value - speed_encoder_value_t.last_encoder_value;
    diff_encoder_value_lpf += (1.0f / (1.0f + 1.0f/(2.0f * 3.14f *0.00017857142857f*2.0f)))*(diff_encoder_value - diff_encoder_value_lpf );

    rpm_rps_count_temp = diff_encoder_value_lpf / 16383.0f;
    motor_rpm = rpm_rps_count_temp * 336000;
    motor_rps = rpm_rps_count_temp * 2016000 * PI / 180.0f;
    speed_encoder_value_t.last_encoder_value = speed_encoder_value_t.encoder_value;  

    //get input voltage
    vol_input = (MyAdcGetVal(1,4)*330*6.4545454545f)/4095;//adc1_in4, e.g. 1036 = 10.36v
    vol_lpf += (1.0f / (1.0f + 1.0f/(2.0f * 3.14f *0.0002f*2.0f)))*(vol_input - vol_lpf );

    if (!over_vol_flag) {
      if (vol_lpf > 1800) {
        over_vol_flag = 1;
        error_code |= ERR_OVER_VOLTAGE;
        MotorDriverSetMode(MDRV_MODE_OFF);
        if (!over_vol_protect_auto_flag && over_vol_protect_mode) {
          over_vol_protect_auto_counter = HAL_GetTick();
          over_vol_protect_auto_flag = 1;
        }
      }
    }
    else {
      if (vol_lpf <= 1750) {
        over_vol_flag = 0;
        error_code &= ~ERR_OVER_VOLTAGE;
        sys_status = SYS_STANDBY;
        if (!over_vol_protect_auto_flag && over_vol_protect_mode) {
          over_vol_protect_auto_counter = HAL_GetTick();
          over_vol_protect_auto_flag = 1;
        }
      }
      else {
        over_vol_flag = 1;
        error_code |= ERR_OVER_VOLTAGE;
        MotorDriverSetMode(MDRV_MODE_OFF);        
      }      
    }
    if (motor_overvalue_protection_flag) {
      int32_t mechanical_angle_int = mechanical_angle * 100;
      if (abs(mechanical_angle_int) > MY_INT32_MAX) {
        over_value_flag = 1;
        error_code |= ERR_OVER_VALUE;
        MotorDriverSetMode(MDRV_MODE_OFF);      
      }
      else {
        if (over_value_flag) {
          over_value_flag = 0;
          error_code &= ~ERR_OVER_VALUE;
          sys_status = SYS_STANDBY;
        }
      }
    }
    else {
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

  if (speed_encoder_value_t.encoder_value != speed_encoder_value_t.last_encoder_value) {
    diff_encoder_value = speed_encoder_value_t.encoder_value - speed_encoder_value_t.last_encoder_value;
    diff_encoder_value_lpf += (1.0f / (1.0f + 1.0f/(2.0f * 3.14f *0.00017857142857f*2.0f)))*(diff_encoder_value - diff_encoder_value_lpf );
    // if (avg_filter_level != 0) {
    //   speed_record[record_index] = diff_encoder_value_lpf;
    //   record_index++;
    //   if (record_index >= avg_filter_level) {
    //     record_index = 0;
    //   }
    //   diff_encoder_value_lpf = avg_filter(speed_record, avg_filter_level);
    // }       
    motor_rpm = diff_encoder_value_lpf / 16383.0f * 336000;
    speed_encoder_value_t.last_encoder_value = speed_encoder_value_t.encoder_value;
  }
  else {
    motor_rpm = 0;
  }
}


void TIM1_UP_TIM16_IRQHandler(void)
{
  //HAL_TIM_IRQHandler(&htim1);
  
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  //MotorDriverProcess();
  if(counter_loop_foc<2)
  {
    counter_loop_foc+=1;
  }
  else
  {
    counter_loop_foc=0;
    Loop_FOC();
  }
 

  if(counter_loop_control<9)
  {
    counter_loop_control+=1;
  }
  else
  {
    counter_loop_control=0;
    Loop_Control();
  }

  if(pid_compute_counter<10)
  {
    pid_compute_counter+=1;
  }
  else
  {
    pid_compute_counter=0;
    switch (motor_mode)
    {
    case MODE_SPEED:
      if (sys_status == SYS_RUNNING) {
        if (motor_stall_protection_flag) {
          speed_err_value = abs((int32_t)(speed_err_rate * pid_ctrl_speed_t.setpoint));
          speed_err_timeout = 3;
        }
        else {
          speed_err_value = 0;
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
          if (abs((int32_t)pid_ctrl_pos_t.setpoint) > 10)
            pos_err_value = 10;
          else
            pos_err_value = abs((int32_t)(pos_err_rate * pid_ctrl_pos_t.setpoint));
          pos_err_timeout = 3;
        }
        else {
          pos_err_value = 0;
          pos_err_timeout = 0;          
        }
        pos_pid();
      }
      break;
    case MODE_SPEED_ERR_PROTECT:
      if (HAL_GetTick() - speed_err_auto_counter > 2000 && speed_err_count_flag) {
        if (!err_stalled_flag && err_recover_try_max && speed_err_recover_try_counter <= err_recover_try_max - 1) {
          speed_err_recover_try_counter++;
          MotorDriverSetMode(MDRV_MODE_RUN);
          motor_mode = MODE_SPEED;
          speed_err_count_flag = 2;
        }
        else {
          err_stalled_flag = 1;
        }
      }
      break;
    case MODE_POS_ERR_PROTECT:
      if (HAL_GetTick() - pos_err_auto_counter > 2000 && pos_err_count_flag) {
        if (!err_stalled_flag && err_recover_try_max && pos_err_recover_try_counter <= err_recover_try_max - 1) {
          pos_err_recover_try_counter++;
          MotorDriverSetMode(MDRV_MODE_RUN);
          motor_mode = MODE_POS;
          pos_err_count_flag = 2;
        }
        else {
          err_stalled_flag = 1;
        }
      }
      break;
    case MODE_CURRENT:
      if (sys_status == SYS_RUNNING) {
        current_point_float = (float)current_point / 100.0f;
        if (fabsf(ph_crrent_lpf / current_point_float) < 0.90f) {
          rgb_flash_slow = 1;
        }
        else {
          rgb_flash_slow = 0;
        }
      }
      break;
    case MODE_DIAL:
      if (sys_status == SYS_RUNNING || sys_status == SYS_STANDBY)
        handle_smart_knob();
      break;      
    
    default:
      break;
    }
    
  }
}

