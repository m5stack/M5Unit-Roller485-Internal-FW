
#include "myadc.h"
#include "adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "tim.h"

#include "arm_math.h"
#include "arm_const_structs.h"

__IO uint16_t adc1_convbuf[10];
__IO uint16_t adc2_convbuf[10];

uint16_t ia_adc,ib_adc,ic_adc,vin_adc;
uint16_t ia_offset,ib_offset,ic_offset;
int32_t ia,ib,ic;


float32_t theta,bx1,by1,bx2,by2;

float32_t ha_f,hb_f,angle_atan;

int32_t internal_temp = 0;

uint8_t adc_cal_ok=0;

void MyADCInit(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc1_convbuf, 6);

    ia=0;ib=0;ic=0;
   
}

void MyADCZeroCal(void)
{
    ia_offset=adc1_convbuf[0];
    ib_offset=adc1_convbuf[1];
    ic_offset=adc1_convbuf[2];
    
    adc_cal_ok=1;

    //ia_offset=2048;
   // ib_offset=2048;
   // ic_offset=2048;

}

void MyAdcProcess(void)
{


        ia_adc=adc1_convbuf[0];
        ib_adc=adc1_convbuf[1];
        ic_adc=adc1_convbuf[2];

        vin_adc=adc1_convbuf[3];

        internal_temp = __HAL_ADC_CALC_TEMPERATURE(__HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc1_convbuf[5], ADC_RESOLUTION_12B), adc1_convbuf[4], ADC_RESOLUTION_12B);
    

    ia=ia_adc - ia_offset;
    ib=ib_adc - ib_offset;
    ic=ic_adc - ic_offset;
}

uint16_t MyAdcGetVal(uint8_t adc_hw,uint8_t channel)
{
    uint16_t adc_val;
    switch(adc_hw)
    {
        case 1:
        {
            switch(channel)
            {
                case 1:{ adc_val = adc1_convbuf[0]; break;}
                case 2:{ adc_val = adc1_convbuf[1]; break;}
                case 3:{ adc_val = adc1_convbuf[2]; break;}
                case 4:{ adc_val = adc1_convbuf[3]; break;}
            }
            break;
        }

        case 2:
        {
            switch(channel)
            {
                case 1:{ adc_val = adc2_convbuf[0]; break;}
                case 2:{ adc_val = adc2_convbuf[1]; break;}
                case 3:{ adc_val = adc2_convbuf[2]; break;}
                case 4:{ adc_val = adc2_convbuf[3]; break;}
            }
        }
    }
    return adc_val;
}

int32_t MyAdcGetCurrent(uint8_t channel)
{
    int32_t current_val;
    switch(channel)
    {
    case 1:{ current_val = ia; break;}
    case 2:{ current_val = ib; break;}
    case 3:{ current_val = ic; break;}

    default: break;

    }
    return current_val;
}




