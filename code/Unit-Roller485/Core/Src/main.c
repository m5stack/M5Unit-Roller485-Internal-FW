/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mysys.h"
#include "rgb.h"
#include "flash.h"
#include "i2c_ex.h"
#include "motordriver.h"
#include "encoder.h"
#include "u8g2_disp_fun.h"
#include "myadc.h"
#include "smart_knob.h"
#include "i2c_protocol.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS         (0x64)
#define APPLICATION_ADDRESS ((uint32_t)0x08002000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flash_data[FLASH_DATA_SIZE] = {0};
uint8_t i2c_address[1]              = {I2C_ADDRESS};
uint8_t motor_disable_flag          = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set(void)
{
    extern uint32_t g_pfnVectors[];

    SCB->VTOR = (uint32_t)g_pfnVectors;
    __DSB();
    __ISB();
}

__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
    return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

static uint32_t getCurrentMicros(void)
{
    /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
    GXT_SYSTICK_IsActiveCounterFlag();
    uint32_t m         = HAL_GetTick();
    const uint32_t tms = SysTick->LOAD + 1;
    __IO uint32_t u    = tms - SysTick->VAL;
    if (GXT_SYSTICK_IsActiveCounterFlag()) {
        m = HAL_GetTick();
        u = tms - SysTick->VAL;
    }
    return (m * 1000 + (u * 1000) / tms);
}

// 获取系统时间，单位us
uint32_t micros(void)
{
    return getCurrentMicros();
}

void init_flash_data(void)
{
    if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
        i2c_address[0] = I2C_ADDRESS;
        flash_data[0]  = i2c_address[0];
        flash_data[1]  = motor_mode;
        flash_data[2]  = angle_cal_offset;
        flash_data[3]  = (angle_cal_offset >> 8);
        flash_data[4]  = motor_id;
        // speed p
        flash_data[5] = speed_pid_int[0];
        flash_data[6] = speed_pid_int[0] >> 8;
        flash_data[7] = speed_pid_int[0] >> 16;
        flash_data[8] = speed_pid_int[0] >> 24;
        // speed i
        flash_data[9]  = speed_pid_int[1];
        flash_data[10] = speed_pid_int[1] >> 8;
        flash_data[11] = speed_pid_int[1] >> 16;
        flash_data[12] = speed_pid_int[1] >> 24;
        // speed d
        flash_data[13] = speed_pid_int[2];
        flash_data[14] = speed_pid_int[2] >> 8;
        flash_data[15] = speed_pid_int[2] >> 16;
        flash_data[16] = speed_pid_int[2] >> 24;
        // pos p
        flash_data[17] = pos_pid_int[0];
        flash_data[18] = pos_pid_int[0] >> 8;
        flash_data[19] = pos_pid_int[0] >> 16;
        flash_data[20] = pos_pid_int[0] >> 24;
        // pos i
        flash_data[21] = pos_pid_int[1];
        flash_data[22] = pos_pid_int[1] >> 8;
        flash_data[23] = pos_pid_int[1] >> 16;
        flash_data[24] = pos_pid_int[1] >> 24;
        // pos d
        flash_data[25] = pos_pid_int[2];
        flash_data[26] = pos_pid_int[2] >> 8;
        flash_data[27] = pos_pid_int[2] >> 16;
        flash_data[28] = pos_pid_int[2] >> 24;
        flash_data[29] = comm_type;
        flash_data[30] = speed_pid_index;
        flash_data[31] = pos_pid_index;
        flash_data[32] = bps_index;
        flash_data[33] = brightness_index;
        flash_data[34] = rgb_show_mode;
        flash_data[35] = motor_stall_protection_flag;
        flash_data[36] = motor_overvalue_protection_flag;
        writeMessageToFlash(flash_data, FLASH_DATA_SIZE);
    } else {
        i2c_address[0]   = flash_data[0];
        motor_mode       = flash_data[1];
        angle_cal_offset = flash_data[2] | (flash_data[3] << 8);
        motor_id         = flash_data[4];
        // speed p
        memcpy((uint8_t *)&speed_pid_int[0], (uint8_t *)&flash_data[5], 4);
        // speed i
        memcpy((uint8_t *)&speed_pid_int[1], (uint8_t *)&flash_data[9], 4);
        // speed d
        memcpy((uint8_t *)&speed_pid_int[2], (uint8_t *)&flash_data[13], 4);
        // pos p
        memcpy((uint8_t *)&pos_pid_int[0], (uint8_t *)&flash_data[17], 4);
        // pos i
        memcpy((uint8_t *)&pos_pid_int[1], (uint8_t *)&flash_data[21], 4);
        // pos d
        memcpy((uint8_t *)&pos_pid_int[2], (uint8_t *)&flash_data[25], 4);
        comm_type                       = flash_data[29];
        speed_pid_index                 = flash_data[30];
        pos_pid_index                   = flash_data[31];
        bps_index                       = flash_data[32];
        brightness_index                = flash_data[33];
        rgb_show_mode                   = flash_data[34];
        motor_stall_protection_flag     = flash_data[35];
        motor_overvalue_protection_flag = flash_data[36];
        MotorDriverSetAngleOffset(angle_cal_offset);
        for (int i = 0; i < 3; i += 2) {
            pos_pid_float[i] = (float)pos_pid_int[i] / 100000;
        }
        pos_pid_float[1] = (float)pos_pid_int[1] / 10000000;
        for (int i = 0; i < 3; i += 2) {
            speed_pid_float[i] = (float)speed_pid_int[i] / 100000;
        }
        speed_pid_float[1] = (float)speed_pid_int[1] / 10000000;
    }

    if (motor_mode == MODE_SPEED_ERR_PROTECT) {
        motor_mode = MODE_SPEED;
    } else if (motor_mode == MODE_POS_ERR_PROTECT) {
        motor_mode = MODE_POS;
    }
}

void flash_data_write_back(void)
{
    if (motor_mode == MODE_SPEED_ERR_PROTECT) {
        motor_mode = MODE_SPEED;
    } else if (motor_mode == MODE_POS_ERR_PROTECT) {
        motor_mode = MODE_POS;
    }

    flash_data[0] = i2c_address[0];
    flash_data[1] = motor_mode;
    flash_data[2] = angle_cal_offset;
    flash_data[3] = (angle_cal_offset >> 8);
    flash_data[4] = motor_id;
    // speed p
    flash_data[5] = speed_pid_int[0];
    flash_data[6] = speed_pid_int[0] >> 8;
    flash_data[7] = speed_pid_int[0] >> 16;
    flash_data[8] = speed_pid_int[0] >> 24;
    // speed i
    flash_data[9]  = speed_pid_int[1];
    flash_data[10] = speed_pid_int[1] >> 8;
    flash_data[11] = speed_pid_int[1] >> 16;
    flash_data[12] = speed_pid_int[1] >> 24;
    // speed d
    flash_data[13] = speed_pid_int[2];
    flash_data[14] = speed_pid_int[2] >> 8;
    flash_data[15] = speed_pid_int[2] >> 16;
    flash_data[16] = speed_pid_int[2] >> 24;
    // pos p
    flash_data[17] = pos_pid_int[0];
    flash_data[18] = pos_pid_int[0] >> 8;
    flash_data[19] = pos_pid_int[0] >> 16;
    flash_data[20] = pos_pid_int[0] >> 24;
    // pos i
    flash_data[21] = pos_pid_int[1];
    flash_data[22] = pos_pid_int[1] >> 8;
    flash_data[23] = pos_pid_int[1] >> 16;
    flash_data[24] = pos_pid_int[1] >> 24;
    // pos d
    flash_data[25] = pos_pid_int[2];
    flash_data[26] = pos_pid_int[2] >> 8;
    flash_data[27] = pos_pid_int[2] >> 16;
    flash_data[28] = pos_pid_int[2] >> 24;
    flash_data[29] = comm_type;
    flash_data[30] = speed_pid_index;
    flash_data[31] = pos_pid_index;
    flash_data[32] = bps_index;
    flash_data[33] = brightness_index;
    flash_data[34] = rgb_show_mode;
    flash_data[35] = motor_stall_protection_flag;
    flash_data[36] = motor_overvalue_protection_flag;
    writeMessageToFlash(flash_data, FLASH_DATA_SIZE);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    IAP_Set();
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_SPI1_Init();
    MX_USART3_UART_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    i2c_protocol_init();
    InitMysys();
    sk6812_init(PIXEL_MAX);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        LoopMysys();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN            = 21;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
