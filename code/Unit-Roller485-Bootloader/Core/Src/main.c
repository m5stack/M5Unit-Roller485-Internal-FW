/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Bootloader main program
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include "crc.h"
#include "gpio.h"
#include "i2c.h"

/* USER CODE BEGIN Includes */
#include "stm32g4xx_hal_flash_ex.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);

typedef enum {
    I2C_EVENT_NONE = 0,
    I2C_EVENT_PACKET_READY,
    I2C_EVENT_TRANSMITTING,
    I2C_EVENT_RECEIVING
} I2C_EventStatus;

typedef enum {
    BOOT_STATUS_OK = 0,
    BOOT_STATUS_I2C_ERROR,
    BOOT_STATUS_RX_OVERFLOW,
    BOOT_STATUS_BAD_PACKET,
    BOOT_STATUS_BAD_LENGTH,
    BOOT_STATUS_BAD_ADDRESS,
    BOOT_STATUS_BAD_ALIGNMENT,
    BOOT_STATUS_FLASH_UNLOCK,
    BOOT_STATUS_FLASH_ERASE,
    BOOT_STATUS_FLASH_PROGRAM,
    BOOT_STATUS_FLASH_VERIFY,
    BOOT_STATUS_FLASH_CONFIG,
    BOOT_STATUS_BAD_FIRMWARE
} BootStatus;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define APPLICATION_ADDRESS ((uint32_t)0x08002000)
#define FW_END_ADDRESS      ((uint32_t)0x0801D800)
#define FW_LENGTH           (FW_END_ADDRESS - APPLICATION_ADDRESS)
#define FW_CRC_ADDR         (FW_END_ADDRESS - sizeof(uint32_t))
#define SRAM_START_ADDRESS  ((uint32_t)0x20000000)
#define SRAM_END_ADDRESS    ((uint32_t)0x20008000)

#define IAP_HEADER_SIZE         ((uint16_t)8)
#define IAP_PAGE_SIZE           ((uint16_t)0x800)
#define IAP_RX_BUFFER_SIZE      (IAP_HEADER_SIZE + IAP_PAGE_SIZE)
#define FLASH_RETRY_COUNT       ((uint8_t)3)
#define FLASH_OPERATION_TIMEOUT ((uint32_t)50)
#define BOOT_ENTRY_DELAY_MS     ((uint32_t)500)
#define UPDATE_TIMEOUT_MS       ((uint32_t)500000)
#define CLOCK_TIMEOUT_MS        ((uint32_t)100)

#define OPC_WREN  ((uint8_t)0x06)
#define OPC_USRCD ((uint8_t)0x77)

_Static_assert((APPLICATION_ADDRESS % IAP_PAGE_SIZE) == 0U, "Application address must be page aligned");
_Static_assert((FW_LENGTH % IAP_PAGE_SIZE) == 0U, "Firmware region must contain whole pages");
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static volatile uint16_t i2c_receive_counter = 0;
static volatile uint16_t i2c_packet_length   = 0;
static volatile uint8_t i2c_receive_overflow = 0;
static volatile uint8_t receive_buffer[IAP_RX_BUFFER_SIZE];
static volatile I2C_EventStatus i2c_event = I2C_EVENT_NONE;
static volatile BootStatus boot_status    = BOOT_STATUS_OK;
static uint32_t app_jump_deadline         = 0;
static uint8_t app_validation_pending     = 1;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static uint8_t time_reached(uint32_t now, uint32_t deadline);
static uint8_t compute_fw_crc32(void);
static uint8_t application_vectors_valid(void);
static uint8_t jump_to_application(void);
static BootStatus write_code(uint16_t packet_length);
static void process_i2c_packet(uint16_t packet_length, uint8_t overflow);
static void reset_all_peripherals(void);
static void iap_i2c(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static uint8_t time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static uint8_t compute_fw_crc32(void)
{
    uint32_t crc_read = *(const uint32_t *)FW_CRC_ADDR;
    uint32_t crc_sum;

    crc_sum = HAL_CRC_Calculate(&hcrc, (uint32_t *)APPLICATION_ADDRESS, FW_LENGTH - sizeof(uint32_t)) ^ 0xFFFFFFFFU;
    return (crc_read == crc_sum) ? 1U : 0U;
}

static uint8_t application_vectors_valid(void)
{
    uint32_t stack_pointer = *(const uint32_t *)APPLICATION_ADDRESS;
    uint32_t reset_vector  = *(const uint32_t *)(APPLICATION_ADDRESS + sizeof(uint32_t));
    uint32_t reset_address = reset_vector & ~1UL;

    if (FLASH_SIZE < (FW_END_ADDRESS - FLASH_BASE)) {
        return 0;
    }
    if ((stack_pointer < SRAM_START_ADDRESS) || (stack_pointer > SRAM_END_ADDRESS) || ((stack_pointer & 0x7U) != 0U)) {
        return 0;
    }
    if (((reset_vector & 1U) == 0U) || (reset_address < APPLICATION_ADDRESS) || (reset_address >= FW_END_ADDRESS)) {
        return 0;
    }
    return 1;
}

void I2C1_EV_UserHandler(void)
{
    uint32_t status = I2C1->ISR;

    if ((status & I2C_ISR_ADDR) != 0U) {
        I2C1->ICR = I2C_ICR_ADDRCF;
        I2C1->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_TXIE);
        I2C1->CR1 |= I2C_CR1_STOPIE;

        if ((status & I2C_ISR_DIR) != 0U) {
            i2c_event = I2C_EVENT_TRANSMITTING;
            I2C1->CR1 |= I2C_CR1_TXIE;
        } else {
            I2C1->CR2 &= ~I2C_CR2_NACK;
            i2c_receive_counter  = 0;
            i2c_receive_overflow = 0;
            i2c_event            = I2C_EVENT_RECEIVING;
            I2C1->CR1 |= I2C_CR1_RXIE;
        }
    }

    if ((status & I2C_ISR_RXNE) != 0U) {
        uint8_t data = (uint8_t)I2C1->RXDR;
        if (i2c_receive_counter < IAP_RX_BUFFER_SIZE) {
            receive_buffer[i2c_receive_counter++] = data;
        } else {
            i2c_receive_overflow = 1;
            I2C1->CR2 |= I2C_CR2_NACK;
        }
    }

    if ((status & I2C_ISR_TXIS) != 0U) {
        I2C1->TXDR = (uint8_t)boot_status;
    }
    if ((status & I2C_ISR_NACKF) != 0U) {
        I2C1->ICR = I2C_ICR_NACKCF;
    }

    if ((status & I2C_ISR_STOPF) != 0U) {
        I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
        I2C1->CR1 &= ~(I2C_CR1_STOPIE | I2C_CR1_RXIE | I2C_CR1_TXIE);

        if (i2c_event == I2C_EVENT_RECEIVING) {
            i2c_packet_length = i2c_receive_counter;
            i2c_event         = I2C_EVENT_PACKET_READY;
            LL_I2C_DisableIT_ADDR(I2C1);
        } else {
            i2c_event = I2C_EVENT_NONE;
            LL_I2C_EnableIT_ADDR(I2C1);
        }
    }
}

void I2C1_ER_UserHandler(void)
{
    LL_I2C_Disable(I2C1);
    I2C1->ICR = I2C_ICR_ADDRCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF |
                I2C_ICR_PECCF | I2C_ICR_TIMOUTCF | I2C_ICR_ALERTCF;
    I2C1->CR1 &= ~(I2C_CR1_STOPIE | I2C_CR1_RXIE | I2C_CR1_TXIE);
    i2c_receive_counter  = 0;
    i2c_receive_overflow = 0;
    i2c_event            = I2C_EVENT_NONE;
    boot_status          = BOOT_STATUS_I2C_ERROR;
    LL_I2C_Enable(I2C1);
    LL_I2C_EnableIT_ADDR(I2C1);
}

static void reset_all_peripherals(void)
{
    uint32_t index;

    LL_I2C_DisableIT_ADDR(I2C1);
    LL_I2C_Disable(I2C1);
    LL_I2C_DeInit(I2C1);
    HAL_CRC_DeInit(&hcrc);

    for (index = 0; index < 8U; index++) {
        NVIC->ICER[index] = 0xFFFFFFFFU;
        NVIC->ICPR[index] = 0xFFFFFFFFU;
    }

    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    SCB->ICSR     = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
}

static uint8_t jump_to_application(void)
{
    uint32_t jump_address;
    pFunction jump_function;

    if ((!application_vectors_valid()) || (!compute_fw_crc32())) {
        boot_status = BOOT_STATUS_BAD_FIRMWARE;
        return 0;
    }

    jump_address  = *(const uint32_t *)(APPLICATION_ADDRESS + sizeof(uint32_t));
    jump_function = (pFunction)jump_address;
    reset_all_peripherals();
    SCB->VTOR = APPLICATION_ADDRESS;
    __DSB();
    __ISB();
    __set_MSP(*(const uint32_t *)APPLICATION_ADDRESS);
    jump_function();
    while (1) {
    }
}

static BootStatus write_code(uint16_t packet_length)
{
    FLASH_EraseInitTypeDef erase_config = {0};
    uint32_t page_error                 = 0;
    uint32_t flash_address;
    uint16_t data_length;
    uint16_t data_offset;
    uint8_t retry;
    HAL_StatusTypeDef hal_status = HAL_ERROR;

    if (packet_length < IAP_HEADER_SIZE) {
        return BOOT_STATUS_BAD_PACKET;
    }
    if (FLASH_SIZE < (FW_END_ADDRESS - FLASH_BASE)) {
        return BOOT_STATUS_FLASH_CONFIG;
    }

    flash_address = ((uint32_t)receive_buffer[1] << 24) | ((uint32_t)receive_buffer[2] << 16) |
                    ((uint32_t)receive_buffer[3] << 8) | (uint32_t)receive_buffer[4];
    data_length   = ((uint16_t)receive_buffer[5] << 8) | (uint16_t)receive_buffer[6];

    if ((data_length == 0U) || (data_length > IAP_PAGE_SIZE) ||
        (packet_length < (uint16_t)(IAP_HEADER_SIZE + data_length))) {
        return BOOT_STATUS_BAD_LENGTH;
    }
    if ((flash_address < APPLICATION_ADDRESS) || (flash_address > (FW_END_ADDRESS - IAP_PAGE_SIZE))) {
        return BOOT_STATUS_BAD_ADDRESS;
    }
    if (((flash_address - APPLICATION_ADDRESS) % IAP_PAGE_SIZE) != 0U) {
        return BOOT_STATUS_BAD_ALIGNMENT;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    if ((FLASH_WaitForLastOperation(FLASH_OPERATION_TIMEOUT) != HAL_OK) || (HAL_FLASH_Unlock() != HAL_OK)) {
        return BOOT_STATUS_FLASH_UNLOCK;
    }

    erase_config.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_config.NbPages   = 1;

#if defined(FLASH_OPTR_DBANK)
    /* G431 has 2-KB pages only in dual-bank mode; page numbers are bank-relative. */
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) == 0U) {
        HAL_FLASH_Lock();
        return BOOT_STATUS_FLASH_CONFIG;
    }
    if (flash_address < (FLASH_BASE + FLASH_BANK_SIZE)) {
        erase_config.Banks = FLASH_BANK_1;
        erase_config.Page  = (flash_address - FLASH_BASE) / IAP_PAGE_SIZE;
    } else {
        erase_config.Banks = FLASH_BANK_2;
        erase_config.Page  = (flash_address - (FLASH_BASE + FLASH_BANK_SIZE)) / IAP_PAGE_SIZE;
    }
#else
    erase_config.Banks = FLASH_BANK_1;
    erase_config.Page  = (flash_address - FLASH_BASE) / IAP_PAGE_SIZE;
#endif

    for (retry = 0; retry < FLASH_RETRY_COUNT; retry++) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        if (HAL_FLASHEx_Erase(&erase_config, &page_error) == HAL_OK) {
            hal_status = HAL_OK;
            break;
        }
    }
    if (hal_status != HAL_OK) {
        HAL_FLASH_Lock();
        return BOOT_STATUS_FLASH_ERASE;
    }

    for (data_offset = 0; data_offset < data_length; data_offset += sizeof(uint64_t)) {
        uint64_t data = ~(uint64_t)0;
        uint8_t byte_index;
        uint16_t remaining    = data_length - data_offset;
        uint8_t bytes_in_word = (remaining > sizeof(uint64_t)) ? sizeof(uint64_t) : (uint8_t)remaining;

        for (byte_index = 0; byte_index < bytes_in_word; byte_index++) {
            data &= ~((uint64_t)0xFFU << (byte_index * 8U));
            data |= (uint64_t)receive_buffer[IAP_HEADER_SIZE + data_offset + byte_index] << (byte_index * 8U);
        }

        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_address + data_offset, data) != HAL_OK) {
            HAL_FLASH_Lock();
            return BOOT_STATUS_FLASH_PROGRAM;
        }
        if (*(const volatile uint64_t *)(flash_address + data_offset) != data) {
            HAL_FLASH_Lock();
            return BOOT_STATUS_FLASH_VERIFY;
        }
    }

    HAL_FLASH_Lock();
    return BOOT_STATUS_OK;
}

static void process_i2c_packet(uint16_t packet_length, uint8_t overflow)
{
    if (overflow != 0U) {
        boot_status = BOOT_STATUS_RX_OVERFLOW;
        return;
    }
    if (packet_length == 0U) {
        boot_status = BOOT_STATUS_BAD_PACKET;
        return;
    }

    switch (receive_buffer[0]) {
        case OPC_WREN:
            app_jump_deadline      = HAL_GetTick() + UPDATE_TIMEOUT_MS;
            app_validation_pending = 1;
            boot_status            = write_code(packet_length);
            break;

        case OPC_USRCD:
            app_validation_pending = 0;
            jump_to_application();
            break;

        default:
            boot_status = BOOT_STATUS_BAD_PACKET;
            break;
    }
}

static void iap_i2c(void)
{
    while (1) {
        if ((app_validation_pending != 0U) && time_reached(HAL_GetTick(), app_jump_deadline)) {
            app_validation_pending = 0;
            jump_to_application();
        }

        if (i2c_event == I2C_EVENT_PACKET_READY) {
            uint16_t packet_length;
            uint8_t overflow;

            NVIC_DisableIRQ(I2C1_EV_IRQn);
            packet_length = i2c_packet_length;
            overflow      = i2c_receive_overflow;
            i2c_event     = I2C_EVENT_NONE;
            process_i2c_packet(packet_length, overflow);
            LL_I2C_EnableIT_ADDR(I2C1);
            NVIC_EnableIRQ(I2C1_EV_IRQn);
        }
    }
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_CRC_Init();

    /* USER CODE BEGIN 2 */
    iap_gpio_init();
    LL_mDelay(30);

    if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) &&
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET)) {
        app_jump_deadline = HAL_GetTick() + BOOT_ENTRY_DELAY_MS;
    } else {
        app_jump_deadline = HAL_GetTick();
    }

    MX_I2C1_Init();
    LL_I2C_Enable(I2C1);
    LL_I2C_EnableIT_ERR(I2C1);
    LL_I2C_EnableIT_ADDR(I2C1);
    iap_i2c();
    /* USER CODE END 2 */

    while (1) {
    }
}

void SystemClock_Config(void)
{
    uint32_t deadline;

    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
    }

    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSI_Enable();
    deadline = HAL_GetTick() + CLOCK_TIMEOUT_MS;
    while (LL_RCC_HSI_IsReady() != 1U) {
        if (time_reached(HAL_GetTick(), deadline)) {
            Error_Handler();
        }
    }

    LL_RCC_HSI_SetCalibTrimming(64);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 21, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    deadline = HAL_GetTick() + CLOCK_TIMEOUT_MS;
    while (LL_RCC_PLL_IsReady() != 1U) {
        if (time_reached(HAL_GetTick(), deadline)) {
            Error_Handler();
        }
    }

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    deadline = HAL_GetTick() + CLOCK_TIMEOUT_MS;
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
        if (time_reached(HAL_GetTick(), deadline)) {
            Error_Handler();
        }
    }

    for (__IO uint32_t delay = (170U >> 1); delay != 0U; delay--) {
    }

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(168000000U);

    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
