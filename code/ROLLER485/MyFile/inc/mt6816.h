#ifndef __MT6816_SPI_H
#define __MT6816_SPI_H
#include "stm32g4xx.h"
#include "arm_const_structs.h"

#include <string.h>		
#include <stdlib.h>	
#include <stdbool.h>			

#define MT6816_SPI_CS_GROUP	     GPIOA
#define MT6816_SPI_CS_PIN		 GPIO_PIN_4
#define MT6816_SPI_HW		     (hspi1)
//#define MT6816_Mode_SPI		        (0x03)	


typedef struct{
	uint16_t	sample_data;	
	uint16_t	angle;				
	bool		no_mag_flag;	
	bool		pc_flag;			
}MT6816_SPI_Signal_Typedef;



typedef struct{
	uint16_t	angle_data;			
	uint16_t	rectify_angle;		
	bool			rectify_valid;		
}MT6816_Typedef;

extern MT6816_Typedef	mt6816;

void MT6816Init(void);	
float32_t MT6816GetAngle(void);		
float32_t MT6816GeteEAngle360(void);		

#endif

