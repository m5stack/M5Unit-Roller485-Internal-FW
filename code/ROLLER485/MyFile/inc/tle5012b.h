#ifndef __TLE5012B_H
#define __TLE5012B_H
#include "stm32g4xx.h"
#include "arm_const_structs.h"

#include <string.h>		
#include <stdlib.h>	
#include <stdbool.h>			

#define ENCODER_SPI_CS_GROUP	     GPIOA
#define ENCODER_SPI_CS_PIN		 GPIO_PIN_4
#define ENCODER_SPI_HW		     (hspi1)
//#define ENCODER_Mode_SPI		        (0x03)	


typedef struct{
	uint16_t	sample_data;	
	uint16_t	angle;				
	bool		no_mag_flag;	
	bool		pc_flag;			
}ENCODER_SPI_Signal_Typedef;



typedef struct{
	uint16_t	angle_data;			
	uint16_t	rectify_angle;		
	bool			rectify_valid;		
}ENCODER_Typedef;

extern ENCODER_Typedef	encoder;

void EncoderInit(void);	
float32_t EncoderGetAngle(void);		

#endif

