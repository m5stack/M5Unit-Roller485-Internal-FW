
#include "arm_math.h"
#include "arm_const_structs.h"

#include "mt6816.h"
#include "spi.h"

#define MT6816_SPI_CS_H MT6816_SPI_CS_GROUP->BSRR	=	MT6816_SPI_CS_PIN
#define MT6816_SPI_CS_L MT6816_SPI_CS_GROUP->BRR	=	MT6816_SPI_CS_PIN


MT6816_SPI_Signal_Typedef	mt6816_spi;

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

void MT6816Init(void)
{
	mt6816_spi.sample_data = 0;
	mt6816_spi.angle = 0;
	MT6816_SPI_CS_H;
	__HAL_SPI_ENABLE(&hspi1);
}

void MT6816GetData(void)
{

	uint8_t h_count;
	data_t[0] = 0x8300;
	data_t[1] = 0x8400;

		//读取SPI数据
		MT6816_SPI_CS_L;
		MT6816_SPI_CS_L;
		//HAL_SPI_TransmitReceive(&MT6816_SPI_HW, (uint8_t*)&data_t[0], (uint8_t*)&data_r[0], 1, HAL_MAX_DELAY);
		SPI_TransmitReceive(&hspi1, data_t[0], &data_r[0]);
		//HAL_Delay(1);
		MT6816_SPI_CS_H;
		MT6816_SPI_CS_H;
		
		MT6816_SPI_CS_L;
		MT6816_SPI_CS_L;
		//HAL_SPI_TransmitReceive(&MT6816_SPI_HW, (uint8_t*)&data_t[1], (uint8_t*)&data_r[1], 1, HAL_MAX_DELAY);
		SPI_TransmitReceive(&hspi1, data_t[1],&data_r[1]);
		//HAL_Delay(1);
		MT6816_SPI_CS_H;
		MT6816_SPI_CS_H;
		mt6816_spi.sample_data = ((data_r[0] & 0x00FF) << 8) | (data_r[1] & 0x00FF);



		mt6816_spi.angle = mt6816_spi.sample_data >> 2;
		mt6816_spi.no_mag_flag = (bool)(mt6816_spi.sample_data & (0x0001 << 1));
	
}

MT6816_Typedef	mt6816;

float32_t MT6816GetAngle(void)
{
	MT6816GetData();
	mt6816.angle_data = mt6816_spi.angle;   
	return mt6816.angle_data;
}

float32_t MT6816GeteEAngle360(void)
{
	float32_t eangle;
	MT6816GetData();
	eangle = (mt6816_spi.angle / 7) % 360;
	return eangle;
}