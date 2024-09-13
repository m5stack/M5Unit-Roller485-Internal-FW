
#include "arm_math.h"
#include "arm_const_structs.h"

#include "encoder.h"
#include "spi.h"

#define ENCODER_SPI_CS_H ENCODER_SPI_CS_GROUP->BSRR	=	ENCODER_SPI_CS_PIN
#define ENCODER_SPI_CS_L ENCODER_SPI_CS_GROUP->BRR	=	ENCODER_SPI_CS_PIN


ENCODER_SPI_Signal_Typedef	encoder_spi;

	uint16_t data_t[2];
	uint16_t data_r[2];

uint8_t SPI_TransmitReceive(SPI_HandleTypeDef * hspi, uint16_t TxData, uint16_t *RxData)            
{
  volatile uint32_t cnt = 0;
    
   while ((hspi->Instance->SR & SPI_SR_TXE) == 0)
   {
	;
   }  
    hspi->Instance->DR = TxData;
	                        
    while ((hspi->Instance->SR & SPI_SR_RXNE)==0) 
     {
	;
   	}  
        if((hspi->Instance->SR & SPI_SR_RXNE))
        {
            *RxData = hspi->Instance->DR;
            return 0;
        }
        cnt++;
      
       
    return 1; 
	while ((hspi->Instance->SR & SPI_SR_TXE) == 0);                               
}

void EncoderInit(void)
{
	encoder_spi.sample_data = 0;
	encoder_spi.angle = 0;
	ENCODER_SPI_CS_H;
	__HAL_SPI_ENABLE(&hspi1);
}

void EncoderGetData(void)
{

	uint8_t h_count;
	data_t[0] = 0x8021;
	data_t[1] = 0xffff;

		//读取SPI数据
		ENCODER_SPI_CS_L;
		ENCODER_SPI_CS_L;
		//HAL_SPI_TransmitReceive(&ENCODER_SPI_HW, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, HAL_MAX_DELAY);
		SPI_TransmitReceive(&hspi1, data_t[0], &data_r[0]);
		SPI_TransmitReceive(&hspi1, data_t[1],&data_r[1]);
		//HAL_Delay(1);

		//HAL_SPI_TransmitReceive(&ENCODER_SPI_HW, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, HAL_MAX_DELAY);
		//SPI_TransmitReceive(&hspi1, data_t[1],&data_r[1]);
		//HAL_Delay(1);
		ENCODER_SPI_CS_H;
		ENCODER_SPI_CS_H;
		//encoder_spi.sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);
		encoder_spi.sample_data = data_r[1];



		encoder_spi.angle = (((encoder_spi.sample_data & 0x7fff) << 1) >> 2);//&0X7FFF;
		//encoder_spi.no_mag_flag = (bool)(encoder_spi.sample_data & (0x0001 << 1));
	
}

ENCODER_Typedef	encoder;

float32_t EncoderGetAngle(void)
{
	EncoderGetData();
	encoder.angle_data = encoder_spi.angle;   
	return encoder.angle_data;
}