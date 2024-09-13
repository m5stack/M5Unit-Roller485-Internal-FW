
#include "motordriver.h"
#include "tim.h"

#include "tle5012b.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "myadc.h"

#include "mysys.h"

#include "u8g2_disp_fun.h"
#include "ws2812.h"

#define     LIMIT_UDC       24.0f
#define     SQRT3           1.732050808f
#define     TS              1000
#define     SQRT3_MULT_TS   (float32_t)((float32_t)TS * SQRT3)
#define     LIMIT           (float32_t)(0.9f / SQRT3)

uint8_t currentloop_enable;
uint8_t motor_driver_cal_flag;
uint8_t motor_driver_cal_init,motor_driver_cal_busy;
uint32_t motor_driver_cal_timer_count;
uint16_t motor_driver_cal_encoder_offset;


uint8_t sectormap[7] = {0, 2, 6, 1, 4, 3, 5};
uint16_t channel1, channel2, channel3;
float32_t eAngle_360,eAngle_add;
uint16_t angle_force;

int16_t dbg_id,dbg_iq;


extern int32_t ia,ib,ic;

float32_t sinVal, cosVal;
float32_t iAlpha, iBeta;
float32_t id, iq;
float32_t uAlpha, uBeta;
float32_t ud, uq, vbus;



float32_t id_curr_pi_kp = 0.002f;
float32_t id_curr_pi_ki = 0.00005f;

float32_t id_curr_pi_target;
float32_t id_curr_pi_value;
float32_t id_curr_pi_error;

float32_t id_curr_pi_errMin = 0.0f;
float32_t id_curr_pi_errSum;
float32_t id_curr_pi_errSumMax = 6.0f;
    
float32_t id_curr_pi_result;




float32_t iq_curr_pi_kp = 0.0002f;
float32_t iq_curr_pi_ki = 0.0005f;

float32_t iq_curr_pi_target;
float32_t iq_curr_pi_value;
float32_t iq_curr_pi_error;

float32_t iq_curr_pi_errMin = 0.0f;
float32_t iq_curr_pi_errSum;
float32_t iq_curr_pi_errSumMax = 6.0f;
    
float32_t iq_curr_pi_result;

float32_t angle_get,angle_offset;
float32_t angle_corrected;
float32_t last_angle_corrected;
uint16_t eangle_get;

void MotorDriverSetMode(uint8_t val);




static void Current_PI_Cal_Id(float32_t resultMax)
{
    id_curr_pi_error = id_curr_pi_target - id_curr_pi_value;
    
    id_curr_pi_errSum += id_curr_pi_error * id_curr_pi_ki;
    
    if(id_curr_pi_errSum > id_curr_pi_errSumMax)
        id_curr_pi_errSum = id_curr_pi_errSumMax;
    else if(id_curr_pi_errSum < -id_curr_pi_errSumMax)
        id_curr_pi_errSum = -id_curr_pi_errSumMax;   
    
    id_curr_pi_result = id_curr_pi_kp * id_curr_pi_error + id_curr_pi_errSum;
    
    if(id_curr_pi_result > resultMax)
        id_curr_pi_result = resultMax;
    else if(id_curr_pi_result < -resultMax)
        id_curr_pi_result = -resultMax;
}


static void Current_PI_Cal_Iq(float32_t resultMax)
{
    iq_curr_pi_error = iq_curr_pi_target - iq_curr_pi_value;

    iq_curr_pi_errSum += iq_curr_pi_error * iq_curr_pi_ki;
    
    if(iq_curr_pi_errSum > iq_curr_pi_errSumMax)
    iq_curr_pi_errSum = iq_curr_pi_errSumMax;
    else if(iq_curr_pi_errSum < -iq_curr_pi_errSumMax)
    iq_curr_pi_errSum = -iq_curr_pi_errSumMax;   
    
    iq_curr_pi_result = iq_curr_pi_kp * iq_curr_pi_error + iq_curr_pi_errSum;
    
    if(iq_curr_pi_result > resultMax)
        iq_curr_pi_result = resultMax;
    else if(iq_curr_pi_result < -resultMax)
        iq_curr_pi_result = -resultMax;
}



static void CurrentLoopCalc(float32_t resultMax)
{
    id_curr_pi_value = id;
    iq_curr_pi_value = iq;
    Current_PI_Cal_Id(resultMax);
    Current_PI_Cal_Iq(resultMax);
    ud = id_curr_pi_result;
    uq = iq_curr_pi_result;

}

static void Clarke_Park(int32_t currA, int32_t currB, int32_t currC)
{   
    sinVal=sinf(eAngle_360 *0.01745f);
    cosVal=cosf(eAngle_360 *0.01745f);


    iAlpha = (float32_t)currA;
    iBeta = (float32_t)(currB - currC) * 0.5773502692f;

    id = iAlpha * cosVal + iBeta * sinVal;
    iq = -iAlpha * sinVal + iBeta * cosVal;
}


static void InversePark(void)
{   
    uAlpha = ud * cosVal - uq * sinVal;
    uBeta = uq * cosVal + ud * sinVal;
}


static void SVMGenerate(float32_t udc)
{
    float32_t U1, U2, U3;
    uint8_t a, b, c, n = 0;
    uint16_t channel1 = 0, channel2 = 0, channel3 = 0;
    
    U1 = uBeta;
    U2 = (SQRT3 * uAlpha - uBeta) / 2.0f;
    U3 = (-SQRT3 * uAlpha - uBeta) / 2.0f;
    
    if(U1 > 0.0f)
        a = 1;
    else 
        a = 0;
    if(U2 > 0.0f)
        b = 1;
    else 
        b = 0;
    if(U3 > 0.0f)
        c = 1;
    else 
        c = 0;
    
    n = (c << 2) + (b << 1) + a;
    
    switch(sectormap[n])
    {
        case 0:
        {
            channel1 = TS / 2;
            channel2 = TS / 2;
            channel3 = TS / 2;
        }break;
        
        case 1:
        {
            int16_t t4 = SQRT3_MULT_TS * U2 / udc;
            int16_t t6 = SQRT3_MULT_TS * U1 / udc;
            int16_t t0 = (TS - t4 - t6) / 2;
            
            channel1 = t4 + t6 + t0;
            channel2 = t6 + t0;
            channel3 = t0;
        }break;
        
        case 2:
        {
            int16_t t2 = -SQRT3_MULT_TS * U2 / udc;
            int16_t t6 = -SQRT3_MULT_TS * U3 / udc;
            int16_t t0 = (TS - t2 - t6) / 2;
            
            channel1 = t6 + t0;
            channel2 = t2 + t6 + t0;
            channel3 = t0;
        }break;
        
        case 3:
        {
            int16_t t2 = SQRT3_MULT_TS * U1 / udc;
            int16_t t3 = SQRT3_MULT_TS * U3 / udc;
            int16_t t0 = (TS - t2 - t3) / 2;
            
            channel1 = t0;
            channel2 = t2 + t3 + t0;
            channel3 = t3 + t0;
        }break;
        
        case 4:
        {
            int16_t t1 = -SQRT3_MULT_TS * U1 / udc;
            int16_t t3 = -SQRT3_MULT_TS * U2 / udc;
            int16_t t0 = (TS - t1 - t3) / 2;
            
            channel1 = t0;
            channel2 = t3 + t0;
            channel3 = t1 + t3 + t0;
        }break;
        
        case 5:
        {
            int16_t t1 = SQRT3_MULT_TS * U3 / udc;
            int16_t t5 = SQRT3_MULT_TS * U2 / udc;
            int16_t t0 = (TS - t1 - t5) / 2;
            
            channel1 = t5 + t0;
            channel2 = t0;
            channel3 = t1 + t5 + t0;
        }break;
        
        case 6:
        {
            int16_t t4 = -SQRT3_MULT_TS * U3 / udc;
            int16_t t5 = -SQRT3_MULT_TS * U1 / udc;
            int16_t t0 = (TS - t4 - t5) / 2;
            
            channel1 = t4 + t5 + t0;
            channel2 = t0;
            channel3 = t5 + t0;
        }break;
        
        default:
            break;
    }
    
    
    if(channel1 > TS)
        channel1 = TS;
    if(channel2 > TS)
        channel2 = TS;
    if(channel3 > TS)
        channel3 = TS;
    
    
    TIM1->CCR1 = channel3;
    TIM1->CCR2 = channel2;
    TIM1->CCR3 = channel1;
    
}


static void vbusLimitCalc( float32_t udc)
{
    float32_t limitUD = 0.0f;
    float32_t limitUQ = 0.0f;
    
    if(udc > LIMIT_UDC)
    limitUD = LIMIT * LIMIT_UDC;
    else
        limitUD = LIMIT * udc;

    arm_sqrt_f32((limitUD * limitUD - ud * ud), &limitUQ);
    
    if(ud > limitUD)
        ud = limitUD;
    else if(ud < -limitUD)
        ud = -limitUD;
    if(uq > limitUQ)
        uq = limitUQ;
    else if(uq < -limitUQ)
        uq = -limitUQ;
}

static void VoltageLimit( float32_t udc)
{
    float32_t limitUD = 0.0f;
    float32_t limitUQ = 0.0f;
    
    if(udc > LIMIT_UDC)
        limitUD = LIMIT * LIMIT_UDC;
    else
        limitUD = LIMIT * udc;
    
    arm_sqrt_f32((limitUD * limitUD - ud * ud), &limitUQ);
    
    if(ud > limitUD)
        ud = limitUD;
    else if(ud < -limitUD)
        ud = -limitUD;
    if(uq > limitUQ)
        uq = limitUQ;
    else if(uq < -limitUQ)
        uq = -limitUQ;
}

static void VoltageControl(float32_t udc)
{
    VoltageLimit(udc);
    
    InversePark();
    
    SVMGenerate(udc);
}



void MotorDriverInit(void)
{
    currentloop_enable=0;
	eAngle_360=0;
    vbus=12.0f;
	ud=0.0f; uq=0.0f;
    iq_curr_pi_target=0;
    id_curr_pi_target=0;
    angle_offset=0;

    motor_driver_cal_flag=0;
    motor_driver_cal_busy=0;
}

void MotorDriverProcess(void)
{
    angle_get=16383-EncoderGetAngle();
    if( angle_get>angle_offset )
    {
        angle_corrected = angle_get-angle_offset;
    }
    else
    {
        angle_corrected = 16383-(angle_offset - angle_get);
    }
     eangle_get =(((((uint16_t)angle_corrected)*360*7*7)/16383) / 7) % 360;

        if(eangle_get<270)
        {
            eangle_get+=90;
        }
        else
        {
            eangle_get = (eangle_get+90)-360;
        }

    switch(motor_driver_cal_flag)
    {
        case 0 :
        {   
            eAngle_360=eangle_get;
            break;
        }

        case 1 :
        {
            if(motor_driver_cal_init ==0 )
            {
                motor_driver_cal_init = 1 ;
                motor_driver_cal_timer_count=0;;
            }

            if(motor_driver_cal_timer_count<28000)
            {
                motor_driver_cal_timer_count +=1;
                eAngle_360 = 0;
                iq_curr_pi_target=1200.0f;
            }
            else
            {
                motor_driver_cal_encoder_offset = (uint16_t)angle_get;
                iq_curr_pi_target=0.0f;
                MotorDriverSetMode(MDRV_MODE_OFF);
                motor_driver_cal_busy=0;
                motor_driver_cal_flag=0;
            }
            break;
        }
    }


    Clarke_Park(ia,ib,ic);

	if(currentloop_enable)
    {
        CurrentLoopCalc(6.4f);
    }			
    
       
	vbusLimitCalc(vbus);
    InversePark();
    SVMGenerate(vbus);
	
}

void MotorDriverSetAngleOffset(float32_t offset)
{
    angle_offset = offset;
}

uint8_t IsMotorDriverEncCalBusy(void)
{
    return motor_driver_cal_busy;
}

uint16_t GetMotorDriverEncCalOffset(void)
{
    return motor_driver_cal_encoder_offset;
}

//Set mode such as off/calibration  encoder/run1
//  MDRV_MODE_OFF:          Release motor,Turn off all of the driver mosfets and reset current loop p-i values.
//  MDRV_MODE_ENC_CAL:      Calibrate encoder by inject a 0 deg iq and read the encoder value, 
//please check busy while calibrating, after calibration, mode will be set to MDRV_MODE_OFF and release motor.
//  MDRV_MODE_RUN:          Normally run mode with current loop.
void MotorDriverSetMode(uint8_t val)
{
    switch(val)
    {
        case MDRV_MODE_OFF:
        {
            rgb_flash_flag = 0;
            switch (motor_mode)
            {
            case MODE_SPEED:
            case MODE_SPEED_ERR_PROTECT:
                rgb_color = 0x003200;
                break;
            case MODE_POS:
            case MODE_POS_ERR_PROTECT:
                rgb_color = 0x000032;
                break;
            case MODE_CURRENT:
                rgb_color = 0x323200;
                break;
            
            default:
                break;
            } 
            if (sys_status == SYS_ERROR) {
                rgb_color = 0x320000;
            }          
            // neopixel_set_color(0, rgb_color);
            // neopixel_set_color(1, rgb_color); 
            // ws2812_show();           
            if (error_code)
                sys_status = SYS_ERROR;
            else
                sys_status = SYS_STANDBY;
            GPIOB->BRR=1<<2; //DISABLE Driver Output

            currentloop_enable =0;//Disable current loop

            iq_curr_pi_value    =0;
            iq_curr_pi_error    =0;
            iq_curr_pi_errSum   =0;
            iq_curr_pi_result   =0;

            id_curr_pi_value    =0;
            id_curr_pi_error    =0;
            id_curr_pi_errSum   =0;
            id_curr_pi_result   =0;

            break;
        }


        case MDRV_MODE_RUN:
        {
            rgb_flash_flag = 1;
            switch (motor_mode)
            {
            case MODE_SPEED:
            case MODE_SPEED_ERR_PROTECT:
                rgb_color = 0x003200;
                break;
            case MODE_POS:
            case MODE_POS_ERR_PROTECT:
                rgb_color = 0x000032;
                break;
            case MODE_CURRENT:
                rgb_color = 0x323200;
                break;
            
            default:
                break;
            } 
            if (sys_status == SYS_ERROR) {
                rgb_color = 0x320000;
            }             
            // neopixel_set_color(0, rgb_color);
            // neopixel_set_color(1, rgb_color); 
            // ws2812_show();               
            sys_status = SYS_RUNNING;
            GPIOB->BSRR=1<<2; //ENABLE Driver Output
            currentloop_enable =1;//Enable current loop
            break;
        }

        case MDRV_MODE_ENC_CAL:
        {
            sys_status = SYS_STANDBY;
            GPIOB->BSRR=1<<2;           //ENABLE Driver Output
            currentloop_enable =1;      //Enable current loop
            motor_driver_cal_flag =1;   //Enable Encoder calibration, switch electrical angle and iq to manual
            motor_driver_cal_busy=1;
            break;
        }

    }

}

//Set motor phase current by ADC value
//data type: int32_t
//range: -1500 ~ 1500, 0means 0current, motor spins free.
// e.g.:  1500 means about 1.5A
void MotorDriverSetCurrentAdc(int32_t phase_current)
{
    if(motor_driver_cal_busy ==0 )
    {
        if(phase_current> 1500)//1.2A
        phase_current= 1500;
        if(phase_current< -1500)//-1.2A
        phase_current= -1500;

        iq_curr_pi_target = (float32_t) phase_current;
    }
}


//Set motor phase current by real mA value
//data type: float32_t
//range: -1200.0f ~ 1200.0f, 0means 0current, motor spins free.
// e.g.:  1.0f = 1mA, 1000.0f means about 1.0A
void MotorDriverSetCurrentReal(float32_t phase_current)
{
    float32_t iq_calc = 0.0f;
    if(motor_driver_cal_busy ==0 )
    {
        if(phase_current> 1200.0f)//1.2A
        phase_current= 1200.0f;
        if(phase_current< -1200.0f)//-1.2A
        phase_current= -1200.0f;
        iq_calc = phase_current * 1.25f ;
        iq_curr_pi_target = iq_calc;
    }
}

//Get motor real-time phase current by ADC value
float32_t MotorDriverGetPhaseCurrentAdc(void)
{
   return iq; 
}

//Get motor real-time phase current by mA value
float32_t MotorDriverGetPhaseCurrentReal(void)
{
   return (iq/1.25f); 
}

void mt6816_update(void)
{
    EncoderGetAngle();    
}

void MotorDriverSetUq(float32_t uq_set)
{
    if(uq_set<8)
    {
        uq=uq_set;
    }
}

uint16_t MotorDriverGetMechanicalAngle(void)
{
    uint16_t mangle;
    mangle=(angle_corrected)*3600/16383;
    return mangle;
}


void MotorDriverInterrupt(void)
{
	
}


