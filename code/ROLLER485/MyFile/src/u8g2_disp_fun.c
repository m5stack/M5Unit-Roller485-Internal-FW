/*

  u8g2_disp_fun.c

  Monochrome minimal user interface: Glue code between mui and u8g2.

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2021, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/
#include "u8g2_disp_fun.h"
#include "motordriver.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "ws2812.h"
#include "smart_knob.h"

void (*current)(void);

unsigned char funIndex = 0;
uint8_t is_menu_flag = 0;
uint8_t last_btn_status = 0;
uint8_t comm_flash_count = 0;
uint8_t comm_type = COMM_TYPE_I2C;
uint32_t rgb_color = 0;
uint32_t rgb_flash_color = 0;
uint32_t rgb_flash_slow = 0;
uint32_t rgb_flash_flag = 0;
int32_t last_dial_counter = 0;
int cx=10; //x center
int cy=24; //y center
int radius=13; //radius
int radius2=8; //radius
int percent=80; //needle percent
int pot = 0;

const unsigned char gImage_lun[] = {
0x00,0x00,0x00,0x00,0x00,0x0E,0x00,0x07,0x00,0x27,0x00,0x3F,0x00,0x3F,0x80,0x0F,
0xF0,0x01,0xFC,0x00,0xFC,0x00,0xE4,0x00,0xE0,0x00,0x70,0x00,0x00,0x00,0x00,0x00,
};

const unsigned char gImage_mode_back[] = {
0x7E,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7E,
};

const unsigned char gImage_comm[] = {
0x04,0x02,0xFF,0x00,0xFF,0x40,0x20,
};

const unsigned char gImage_comm_i2c[] = {
0x18,0x10,0x10,0x10,0x10,0x10,0x38,0x00,
0x02,0xFF,0x00,0xFF,0x40,
};

const unsigned char gImage_comm_back[] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x02,0xFF,0x00,0xFF,0x40,
};

const unsigned char gImage_comm_485[] = {
0x00,0x18,0x24,0x10,0x08,0x04,0x3C,0x00,
0x02,0xFF,0x00,0xFF,0x40,
};

const unsigned char gImage_comm_back_11x8[] =
{
0xFE,0x03,0xFF,0x07,0xFF,0x07,0xFF,0x07,0xFF,0x07,0xFF,
0x07,0xFF,0x07,0xFE,0x03,
};

// const unsigned char gImage_logo[] =
// {
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,
// 0xE1,0xFF,0x01,0x00,0x00,0x00,0x06,0x80,0x21,0x00,0x00,
// 0x00,0x00,0x00,0x0E,0xC0,0x21,0x00,0x00,0x00,0x00,0x00,
// 0x1E,0xC0,0x21,0x00,0x00,0x00,0x00,0x00,0x3E,0xF0,0x21,
// 0x00,0x00,0x00,0x00,0x00,0x62,0x30,0x21,0x00,0x00,0x00,
// 0x00,0x00,0x62,0x18,0x21,0x00,0x00,0x00,0x00,0x00,0xC2,
// 0x18,0x21,0x00,0x00,0x00,0x00,0x00,0x82,0x0F,0xE1,0x3F,
// 0x00,0x00,0x00,0x00,0x82,0x07,0x01,0xE0,0x00,0x00,0x00,
// 0x00,0x02,0x03,0x01,0x80,0x00,0x00,0x00,0x00,0x02,0x00,
// 0x01,0x80,0x01,0x00,0x00,0x00,0x02,0x00,0x01,0x00,0x01,
// 0x00,0x00,0x00,0x02,0x00,0x01,0x00,0x01,0x00,0x00,0x00,
// 0x02,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x02,0x00,0x01,
// 0x00,0x01,0x00,0x00,0x00,0x02,0x00,0x01,0x80,0x00,0x00,
// 0x00,0x00,0x02,0x00,0x01,0x80,0x00,0x00,0x00,0x00,0x02,
// 0x00,0x01,0x20,0x00,0x00,0x00,0x00,0x02,0x00,0xE1,0x1F,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x00,0xE0,0xFF,0xFF,0xFF,0xFF,0x1F,
// 0x00,0x00,0xE0,0xFF,0xFF,0xFF,0xFF,0x1F,0x00,0x00,0xE0,
// 0xFF,0xFF,0xFF,0xFF,0x1F,0x00,0x00,0xE0,0x03,0xF3,0x83,
// 0x8F,0x1F,0x00,0x00,0xE0,0x03,0xF2,0x03,0x03,0x1F,0x00,
// 0x00,0xE0,0x73,0xF2,0x33,0x32,0x1F,0x00,0x00,0xE0,0x73,
// 0xF2,0x73,0xF2,0x1F,0x00,0x00,0xE0,0x03,0xF2,0x73,0xF2,
// 0x1F,0x00,0x00,0xE0,0x03,0xF2,0x73,0xF2,0x1F,0x00,0x00,
// 0xE0,0x73,0xF2,0x73,0xF2,0x1F,0x00,0x00,0xE0,0x73,0xF2,
// 0x73,0x32,0x1F,0x00,0x00,0xE0,0x03,0x02,0x02,0x03,0x1F,
// 0x00,0x00,0xE0,0x03,0x03,0x02,0x87,0x1F,0x00,0x00,0xE0,
// 0xFF,0xFF,0xFF,0xFF,0x1F,0x00,0x00,0xE0,0xFF,0xFF,0xFF,
// 0xFF,0x1F,0x00,0x00,0xE0,0xFF,0xFF,0xFF,0xFF,0x1F,0x00,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
// };

const unsigned char gImage_logo[] =
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x18,0xFF,0xFF,0xFF,0x03,0xE0,0xFF,0xFF,0x18,0xFF,0xFF,
0xFF,0x03,0x80,0xFF,0xFF,0x18,0xFF,0xFF,0xFF,0x03,0x00,
0xFF,0xFF,0x18,0xFF,0xFF,0xFF,0xC3,0x1F,0xFE,0xFF,0x18,
0xFF,0xFF,0xFF,0xC3,0x1F,0xFE,0xFF,0x18,0xFF,0xFF,0xFF,
0xC3,0x3F,0x1E,0xF8,0x18,0x1F,0x7C,0xC6,0xC3,0x3F,0x0E,
0xF0,0x18,0x07,0x30,0xC0,0xC3,0x3F,0x06,0xC0,0x18,0x07,
0x20,0xC0,0xC3,0x1F,0xC2,0xC7,0x18,0xE3,0x23,0xF0,0xC3,
0x07,0xE3,0x87,0x18,0xF1,0x03,0xFC,0x03,0x80,0xE3,0x8F,
0x18,0xF1,0x07,0xFC,0x03,0xC0,0xE1,0x8F,0x18,0xF0,0x07,
0xFC,0x03,0xC0,0xF1,0x0F,0x18,0x00,0x00,0xFC,0xC3,0xC7,
0xE1,0x0F,0x18,0x00,0x00,0xFC,0xC3,0x87,0xE1,0x8F,0x18,
0xF1,0x3F,0xFC,0xC3,0x8F,0xE1,0x8F,0x18,0xF1,0x3F,0xFC,
0xC3,0x0F,0xE3,0x8F,0x18,0xE1,0x3F,0xFC,0xC3,0x1F,0xC2,
0xC7,0x18,0xE3,0x27,0xFC,0xC3,0x1F,0x06,0xC0,0x18,0x83,
0x00,0xFC,0xC3,0x3F,0x0C,0xE0,0x18,0x0F,0x20,0xFC,0xC3,
0x3F,0x3C,0xF0,0x18,0x1F,0x38,0xFC,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x21,0x13,
0x0F,0x60,0xF0,0xE0,0x03,0xF0,0x27,0x93,0x3F,0x20,0x98,
0xF1,0x03,0x30,0x24,0x93,0x21,0x10,0x08,0x31,0x00,0x30,
0x24,0x93,0x21,0x18,0x08,0x31,0x00,0x30,0x24,0x93,0x21,
0x08,0x98,0xB1,0x01,0x30,0x66,0x91,0x31,0x4C,0xF0,0x70,
0x03,0xF0,0x63,0x94,0x1F,0x46,0xF0,0x30,0x06,0x30,0xC0,
0x9C,0x09,0x42,0x88,0x01,0x04,0x30,0xC0,0x8C,0x19,0x7E,
0x0C,0x01,0x04,0x30,0xC0,0x8C,0x11,0x40,0x0C,0x11,0x04,
0x30,0xC0,0x8C,0x31,0x40,0x88,0x31,0x06,0x30,0xC0,0x8C,
0x21,0x40,0xF0,0xE0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};


typedef struct
{
	uint8_t current;
	uint8_t up;//选择索引号
	uint8_t down;//选择索引号
	uint8_t exit;//退出索引号
	uint8_t enter;//确认索引号
	void (*current_operation)();
} key_table;

key_table table[30]=
{
	//第0层
	{0,10,1,0,0,(*u8g2_disp_menu_0_1)},
	{1,0,2,1,11,(*u8g2_disp_menu_0_2)},
	{2,1,3,2,12,(*u8g2_disp_menu_0_3)},
	{3,2,4,3,13,(*u8g2_disp_menu_0_4)},
	{4,3,5,4,14,(*u8g2_disp_menu_0_5)},
	{5,4,6,5,15,(*u8g2_disp_menu_0_6)},
	{6,5,7,6,16,(*u8g2_disp_menu_0_7)},
	{7,6,8,7,17,(*u8g2_disp_menu_0_8)},
	{8,7,9,8,18,(*u8g2_disp_menu_0_9)},
	{9,8,10,9,19,(*u8g2_disp_menu_0_10)},
	{10,9,0,10,20,(*u8g2_disp_menu_0_11)},
	
  //第1层
	{11,11,11,1,11,(*u8g2_disp_menu_2_1)},					
	{12,12,12,2,12,(*u8g2_disp_menu_3_1)},
	{13,13,13,3,13,(*u8g2_disp_menu_1_1)},						                	
	{14,14,14,4,14,(*u8g2_disp_menu_4_1)},						                	
	{15,15,15,5,15,(*u8g2_disp_menu_5_1)},						                	
	{16,16,16,6,16,(*u8g2_disp_menu_6_1)},						                	
	{17,17,17,7,17,(*u8g2_disp_menu_7_1)},						                	
	{18,18,18,8,18,(*u8g2_disp_menu_8_1)},						                	
	{19,19,19,9,19,(*u8g2_disp_menu_9_1)},						                	
	{20,20,20,10,20,(*u8g2_disp_menu_10_1)},						                	
	// {8,8,3,8,(*page_setup_model_0)},
	// {9,9,4,9,(*page_setup_pairs_0)},					
	// {10, 9,11,21,(*fun_b22)},
	// {11,10,12,22,(*fun_b23)},						                	
	// {12,11, 9, 2,(*fun_b24)},
	
	// {13,16,14,23,(*fun_c21)},					
	// {14,13,15,24,(*fun_c22)},				                	
	// {15,14,16,25,(*fun_c23)},				                	
	// {16,15,13, 3,(*fun_c24)},
	
  //   //第3层
	// {17,17,17,5,(*fun_a31)},			                	
	// {18,18,18,6,(*fun_a32)},		                	
	// {19,19,19,7,(*fun_a33)},
	
	// {20,20,20, 9,(*fun_b31)},				                	
	// {21,21,21,10,(*fun_b32)},			                	
	// {22,22,22,11,(*fun_b33)},
	
	// {23,23,23,13,(*fun_c31)},			                	
	// {24,24,24,14,(*fun_c32)},			                	
	// {25,25,25,15,(*fun_c33)},								
};

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    long res = 0;
    res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (res > out_max)
        return out_max;
    else if (res < out_min)
        return out_min;
    else
        return res;
}

void u8g2_disp_init(void)
{
    u8g2_ClearBuffer(&u8g2); 
    u8g2_DrawXBMP(&u8g2, 0, 0, 64, 48, gImage_logo);
    u8g2_SendBuffer(&u8g2);     
    HAL_Delay(1000);    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFontMode(&u8g2, 1); /*字体模式选择*/
    u8g2_SetFontDirection(&u8g2, 0); /*字体方向选择*/
   
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_SetFont(&u8g2, u8g2_font_7x13_t_symbols);
    // u8g2_DrawGlyph(&u8g2, 0, 8, 0x25EF);     
    u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
    u8g2_DrawGlyph(&u8g2, 1, 20+2, 0x0053); 
    u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
    u8g2_DrawStr(&u8g2, 2, 26+2-1, ".");
    u8g2_DrawStr(&u8g2, 2, 32+2-2, "."); 
    u8g2_DrawStr(&u8g2, 2, 38+2-2, ".");
    // u8g2_DrawGlyph(&u8g2, 2, 26+2, 0x0053); 
    // u8g2_DrawGlyph(&u8g2, 2, 32+2, 0x0053); 
    // u8g2_DrawGlyph(&u8g2, 1, 32, 0x25A1);      
    // u8g2_DrawGlyph(&u8g2, 1, 38, 0x25A1); 
    // u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawBox(&u8g2, 0, 38, 8, 8);
    // u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
    // u8g2_SetDrawColor(&u8g2, 0);     
    // u8g2_DrawStr(&u8g2, 2, 45, "C");
    // u8g2_SetDrawColor(&u8g2, 1);         
    u8g2_DrawVLine(&u8g2, 9+2, 0, 38); 
    u8g2_SendBuffer(&u8g2);     
}

void u8g2_disp_speed(void)
{
    for (int i = 0; i < 1500; i++){
        u8g2_ClearBuffer(&u8g2);
        Drawgauge(cx,cy,radius,percent,i,-1500,1500);
        u8g2_SendBuffer(&u8g2);
        HAL_Delay(100);
    }
}

void u8g2_disp_pos(void)
{
    for (int i = 0; i < 360; i++) {
        u8g2_ClearBuffer(&u8g2);
        DrawPos(32, 24, 10, percent, i, 0, 360);
        u8g2_SendBuffer(&u8g2);
        HAL_Delay(10);
    }
}

void u8g2_disp_current(void)
{
    for (int i = 0; i < 32767; i++) {
        u8g2_ClearBuffer(&u8g2);
        DrawCurrent(64-11, 24, 10, percent, i, -32767, 32767);
        u8g2_SendBuffer(&u8g2);
    }
}

void u8g2_disp_all(void)
{
    char pos_char_buf[50] = {0};
    char turn_char_buf[50] = {0};
    int pos_nums = 0, turn_nums = 0;
    uint8_t turn_x = 0;

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0+10+2, 0, 54, 17+3);
    u8g2_DrawBox(&u8g2, 0+10+2, 17+3, 54, 31);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawHLine(&u8g2, 0+12+2, 17+3, 54);  
    u8g2_DrawVLine(&u8g2, 36+2, 20+3, 48-20);  
    // u8g2_DrawHLine(&u8g2, 0+10, 20, 64);
    // u8g2_DrawVLine(&u8g2, 32, 10, 10);
    // u8g2_DrawVLine(&u8g2, 42, 0, 10);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
    u8g2_DrawStr(&u8g2, 22+2, 26+3, "S");
    u8g2_DrawStr(&u8g2, 64-15+2, 26+3, "C");
    
    // u8g2_DrawStr(&u8g2, 36, 18, "curent");
    Drawgauge(23+2,24+12+3,12,percent,(int)motor_rpm,-1200,1200);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
    u8g2_DrawStr(&u8g2, 0+12+2, 7, "P");    
    float dis_angle = (float)MotorDriverGetMechanicalAngle() / 10.0f;
    if (dis_angle < 10.0f)
        pos_nums = snprintf(pos_char_buf, 50, "  %.1f", dis_angle);
    else if (dis_angle < 100.0f)
        pos_nums = snprintf(pos_char_buf, 50, " %.1f", dis_angle);
    else
        pos_nums = snprintf(pos_char_buf, 50, "%.1f", dis_angle);
    snprintf(turn_char_buf, 50, "%.0fr",mechanical_turns);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
    u8g2_DrawStr(&u8g2, 46-6+2, 16+2, ".");
    u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
    u8g2_DrawStr(&u8g2, 50-6, 16+2, &pos_char_buf[pos_nums-1]);
    pos_char_buf[pos_nums-2] = 0; 
    u8g2_DrawStr(&u8g2, 31-6+2, 16+2, pos_char_buf); 
    u8g2_DrawCircle(&u8g2, 55-6+2, 11+2, 1, U8G2_DRAW_ALL);
    
    if (mechanical_turns >= 0) {
        if (mechanical_turns < 10)
            turn_x = 36;
        else if (mechanical_turns < 100)
            turn_x = 36-3;
        else if (mechanical_turns < 1000)
            turn_x = 36-5;
        else if (mechanical_turns < 10000)
            turn_x = 36-8;
        else if (mechanical_turns < 100000)
            turn_x = 36-10;
        else if (mechanical_turns < 1000000)
            turn_x = 36-13;
        else if (mechanical_turns < 10000000)
            turn_x = 36-15;
        else if (mechanical_turns < 100000000)
            turn_x = 36-18;
        else if (mechanical_turns < 1000000000)
            turn_x = 36-20;
        else
            turn_x = 36-25;
    }
    else {
        if (mechanical_turns > -10)
            turn_x = 36-3;
        else if (mechanical_turns > -100)
            turn_x = 36-5;
        else if (mechanical_turns > -1000)
            turn_x = 36-8;
        else if (mechanical_turns > -10000)
            turn_x = 36-10;
        else if (mechanical_turns > -100000)
            turn_x = 36-13;
        else if (mechanical_turns > -1000000)
            turn_x = 36-15;
        else if (mechanical_turns > -10000000)
            turn_x = 36-18;
        else if (mechanical_turns > -100000000)
            turn_x = 36-20;
        else
            turn_x = 36-25;            
    }

    u8g2_DrawStr(&u8g2, turn_x, 8, turn_char_buf); 
    // if (fabs(mechanical_turns) < 100) {
    //     if ((int32_t)mechanical_turns >= 0)
    //         turn_nums = snprintf(turn_char_buf, 50, " %.0f",mechanical_turns);
    //     else
    //         turn_nums = snprintf(turn_char_buf, 50, "%.0f",mechanical_turns);
    //     u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
    //     u8g2_DrawStr(&u8g2, 12+3, 13, turn_char_buf);
    //     u8g2_DrawStr(&u8g2, 12+3+5*turn_nums, 11, ".");
    //     u8g2_DrawStr(&u8g2, 12+3+5*turn_nums+6, 13, pos_char_buf); 
    //     u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
    //     u8g2_DrawCircle(&u8g2, 12+3+5*turn_nums+6+5*pos_nums, 8, 1, U8G2_DRAW_ALL);
    //     // u8g2_DrawStr(&u8g2, 12+3+5*turn_nums+6+5*pos_nums, 7, "o");        
    // }
    // else {
    //     u8g2_SetFont(&u8g2, u8g2_font_5x8_mf);
    //     if (mechanical_turns > 0)
    //         turn_nums = snprintf(turn_char_buf, 50, " oo");
    //     else
    //         turn_nums = snprintf(turn_char_buf, 50, "-oo");
    //     u8g2_DrawStr(&u8g2, 12+3, 12, turn_char_buf);
    //     u8g2_DrawStr(&u8g2, 12+3+5*turn_nums, 11, ".");
    //     u8g2_DrawStr(&u8g2, 12+3+5*turn_nums+6, 13, pos_char_buf); 
    //     u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
    //     u8g2_DrawCircle(&u8g2, 12+3+5*turn_nums+6+5*pos_nums, 8, 1, U8G2_DRAW_ALL);
    //     // u8g2_DrawStr(&u8g2, 12+3+5*turn_nums+6+5*pos_nums, 7, "o");         
    // }
    // DrawPos(32, 24, 10, percent, eangle_get, 0, 360);
    DrawCurrent(64-14+2, 24+12+3, 12, percent, (int)ph_crrent_lpf, -1200, 1200);
    
    u8g2_SendBuffer(&u8g2);
}

void u8g2_disp_char(void)
{
    char vin_char_buf[50] = {0};
    char dis_char_buf[50] = {0};
    char target_char_buf[50] = {0};
    int pos_nums = 0, target_pos_nums = 0;
    int dis_char_num = 0, target_char_num = 0;

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0+10+2, 0, 54, 48);
    u8g2_SetFont(&u8g2, u8g2_font_micro_tr); /*字库选择*/

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf); /*字库选择*/
    u8g2_DrawStr(&u8g2, 0+12+2, 23, "T");
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf); /*字库选择*/
    switch (motor_mode)
    {
    case MODE_SPEED:
    case MODE_SPEED_ERR_PROTECT:
        dis_char_num = snprintf(dis_char_buf, 50, "%.0f", motor_rpm);
        target_char_num = snprintf(target_char_buf, 50, "%.0f", pid_ctrl_speed_t.setpoint);
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        u8g2_DrawStr(&u8g2, 11+8+2+5*6+1, 21, "rp"); 
        u8g2_DrawStr(&u8g2, 11+8+2+5*6+1, 29, "m");
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        u8g2_DrawStr(&u8g2, 11+8+2+5*6+1, 38, "rp"); 
        u8g2_DrawStr(&u8g2, 11+8+2+5*6+1, 46, "m");   
        u8g2_SetFont(&u8g2, u8g2_font_6x12_tf); /*字库选择*/               
        break;
    case MODE_POS:
    case MODE_POS_SPEED:
    case MODE_POS_ERR_PROTECT:
        pos_nums = snprintf(dis_char_buf, 50, "%.1f", mechanical_angle);
        target_pos_nums = snprintf(target_char_buf, 50, "%.1f", pid_ctrl_pos_t.setpoint);
        break;
    case MODE_CURRENT:
        snprintf(dis_char_buf, 50, "%.0fmA", ph_crrent_lpf);
        snprintf(target_char_buf, 50, "%.0fmA", (float)current_point / 100.0f);
        break;
    case MODE_DIAL:
        snprintf(dis_char_buf, 50, "%d", current_position);
        break;
    
    default:
        break;
    }
    u8g2_DrawStr(&u8g2, 11+8+2, 27, target_char_buf);  
    u8g2_DrawLine(&u8g2, 0+12+2, 29, 64, 29);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf); /*字库选择*/
    u8g2_DrawStr(&u8g2, 0+12+2, 39, "N");
    u8g2_SetFont(&u8g2, u8g2_font_6x12_tf); /*字库选择*/
    u8g2_DrawStr(&u8g2, 11+8+2, 44, dis_char_buf); 
    if (motor_mode == MODE_POS) {
        u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
        u8g2_DrawCircle(&u8g2, 2+11+8+6*target_pos_nums, 20, 1, U8G2_DRAW_ALL);
        u8g2_DrawCircle(&u8g2, 2+11+8+6*pos_nums, 36, 1, U8G2_DRAW_ALL);
        // u8g2_DrawStr(&u8g2, 11+11+6*target_pos_nums, 20, "o");
        // u8g2_DrawStr(&u8g2, 11+11+6*pos_nums, 36, "o");
    } 

    u8g2_DrawLine(&u8g2, 12+2, 14, 64, 14);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
    // u8g2_DrawStr(&u8g2, 0+12+2, 8, "V");
    // u8g2_DrawStr(&u8g2, 47, 10, "M");
    u8g2_DrawLine(&u8g2, 0+12+2, 29, 64, 29);
    int vol_pos = snprintf(vin_char_buf, 50, "%.1f", vol_lpf/100.0f);
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_SetFont(&u8g2, u8g2_font_6x12_tf);
    u8g2_DrawStr(&u8g2, 0+12+2+2, 7, vin_char_buf);
    u8g2_SetFont(&u8g2, u8g2_font_7x14_tf);
    u8g2_DrawStr(&u8g2, 34, 11, "/");
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
    u8g2_DrawStr(&u8g2, 23, 13, "V");
    memset(vin_char_buf, 0, 50);
    snprintf(vin_char_buf, 50, "%d", (int)ph_crrent_lpf);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
    u8g2_DrawStr(&u8g2, 39, 13, vin_char_buf); // 0+12+2+vol_pos*5+7-2
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
    u8g2_DrawStr(&u8g2, 49, 6, "mA");
    u8g2_SendBuffer(&u8g2); 
    
}

void u8g2_disp_pid(void)
{
    char p_char_buf[50] = {0};
    char i_char_buf[50] = {0};
    char d_char_buf[50] = {0};
    char target_char_buf[50] = {0};
    int pos_nums = 0, target_pos_nums = 0;
    int dis_char_num = 0, target_char_num = 0;

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0+10+2, 0, 54, 48);
    u8g2_SetFont(&u8g2, u8g2_font_micro_tr); /*字库选择*/

    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf); /*字库选择*/
    u8g2_DrawStr(&u8g2, 0+12+2, 23, "I");
    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf); /*字库选择*/
    switch (motor_mode)
    {
    case MODE_SPEED:
    case MODE_SPEED_ERR_PROTECT:
        switch (speed_pid_index)
        {
        case 0:
            snprintf(p_char_buf, 50, "%.7f", speed_pid_float[0]);
            snprintf(i_char_buf, 50, "%.7f", speed_pid_float[1]);
            snprintf(d_char_buf, 50, "%.7f", speed_pid_float[2]);
            break;
        case 1:
            snprintf(p_char_buf, 50, "%.7f", speed_pid_low_float[0]);
            snprintf(i_char_buf, 50, "%.7f", speed_pid_low_float[1]);
            snprintf(d_char_buf, 50, "%.7f", speed_pid_low_float[2]);
            break;
        case 2:
            snprintf(p_char_buf, 50, "%.7f", speed_pid_mid_float[0]);
            snprintf(i_char_buf, 50, "%.7f", speed_pid_mid_float[1]);
            snprintf(d_char_buf, 50, "%.7f", speed_pid_mid_float[2]);
            break;
        case 3:
            snprintf(p_char_buf, 50, "%.7f", speed_pid_high_float[0]);
            snprintf(i_char_buf, 50, "%.7f", speed_pid_high_float[1]);
            snprintf(d_char_buf, 50, "%.7f", speed_pid_high_float[2]);
            break;
        
        default:
            break;
        }
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 11+8, 11, p_char_buf);         
        u8g2_DrawStr(&u8g2, 11+8, 27, i_char_buf);                        
        u8g2_DrawStr(&u8g2, 11+8, 44, d_char_buf);                        
        break;
    case MODE_POS:
    case MODE_POS_ERR_PROTECT:
        switch (pos_pid_index)
        {
        case 0:
            snprintf(p_char_buf, 50, "%.7f", pos_pid_float[0]);
            snprintf(i_char_buf, 50, "%.7f", pos_pid_float[1]);
            snprintf(d_char_buf, 50, "%.7f", pos_pid_float[2]);
            break;
        case 1:
            snprintf(p_char_buf, 50, "%.7f", pos_pid_low_float[0]);
            snprintf(i_char_buf, 50, "%.7f", pos_pid_low_float[1]);
            snprintf(d_char_buf, 50, "%.7f", pos_pid_low_float[2]);
            break;
        case 2:
            snprintf(p_char_buf, 50, "%.7f", pos_pid_mid_float[0]);
            snprintf(i_char_buf, 50, "%.7f", pos_pid_mid_float[1]);
            snprintf(d_char_buf, 50, "%.7f", pos_pid_mid_float[2]);
            break;
        case 3:
            snprintf(p_char_buf, 50, "%.7f", pos_pid_high_float[0]);
            snprintf(i_char_buf, 50, "%.7f", pos_pid_high_float[1]);
            snprintf(d_char_buf, 50, "%.7f", pos_pid_high_float[2]);
            break;
        
        default:
            break;
        }
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 11+8, 11, p_char_buf);         
        u8g2_DrawStr(&u8g2, 11+8, 27, i_char_buf);                        
        u8g2_DrawStr(&u8g2, 11+8, 44, d_char_buf);
        break;
    case MODE_CURRENT:
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 11+8, 11, "0.0020000");         
        u8g2_DrawStr(&u8g2, 11+8, 27, "0.0000500");                        
        u8g2_DrawStr(&u8g2, 11+8, 44, "0.0000000");
        break;
    case MODE_DIAL:
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 11+8+10, 11, "2500.0");         
        u8g2_DrawStr(&u8g2, 11+8+10, 27, "0.0000");                        
        u8g2_DrawStr(&u8g2, 11+8+10, 44, "10.000");
        break;
    case MODE_POS_SPEED:
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 11+8+10, 11, "NA");         
        u8g2_DrawStr(&u8g2, 11+8+10, 27, "NA");                        
        u8g2_DrawStr(&u8g2, 11+8+10, 44, "NA");
        break;
    
    default:
        break;
    }
    u8g2_DrawStr(&u8g2, 11+8+2, 27, target_char_buf);  
    u8g2_DrawLine(&u8g2, 0+12+2, 29, 64, 29);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf); /*字库选择*/
    u8g2_DrawStr(&u8g2, 0+12+2, 39, "D");
    u8g2_DrawLine(&u8g2, 12+2, 14, 64, 14);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
    u8g2_DrawStr(&u8g2, 0+12+2, 8, "P");
    u8g2_DrawLine(&u8g2, 0+12+2, 29, 64, 29);
    u8g2_SendBuffer(&u8g2); 
    
}

void u8g2_disp_info(void)
{
    char vin_char_buf[50] = {0};
    char dis_char_buf[50] = {0};
    char target_char_buf[50] = {0};
    int pos_nums = 0, target_pos_nums = 0;

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0+10+2, 0, 54, 48);

    u8g2_SetDrawColor(&u8g2, 1);

    u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
    if (comm_type == COMM_TYPE_I2C) {   
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf); /*字库选择*/
        switch (motor_mode)
        {
        case MODE_SPEED:
        case MODE_SPEED_ERR_PROTECT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:SPEED");
            break;
        case MODE_POS:
        case MODE_POS_ERR_PROTECT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:POS");
            break;
        case MODE_CURRENT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:CURRENT");
            break;
        case MODE_DIAL:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:ENCODER");
            break;
        case MODE_POS_SPEED:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:POS+");
            break;
        
        default:
            break;
        }
        u8g2_DrawLine(&u8g2, 12+2, 12, 64, 12);         

        u8g2_DrawStr(&u8g2, 0+12+2+4, 21, "COM:");
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 0+12+2+4+16, 21, "I2C");  
        u8g2_DrawLine(&u8g2, 0+12+2, 24, 64, 24);

        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
        u8g2_DrawStr(&u8g2, 0+12+2, 33, "ADDR:");
        snprintf(dis_char_buf, 50, "0x%02X", i2c_address[0]);
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
        u8g2_DrawStr(&u8g2, 0+12+2+8+12, 33, dis_char_buf);       
        u8g2_DrawLine(&u8g2, 0+12+2, 36, 64, 36); 
    }   
    else if (comm_type == COMM_TYPE_485) {
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf); /*字库选择*/
        switch (motor_mode)
        {
        case MODE_SPEED:
        case MODE_SPEED_ERR_PROTECT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:SPEED");
            break;
        case MODE_POS:
        case MODE_POS_ERR_PROTECT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:POS");
            break;
        case MODE_CURRENT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:CURRENT");
            break;
        case MODE_DIAL:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:ENCODER");
            break;
        case MODE_POS_SPEED:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:POS+");
            break;            
        
        default:
            break;
        }
        u8g2_DrawLine(&u8g2, 12+2, 12, 64, 12);    

        u8g2_DrawStr(&u8g2, 0+12+2+4, 21, "COM:");
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 0+12+2+4+16, 21, "485");  
        u8g2_DrawLine(&u8g2, 0+12+2, 24, 64, 24);

        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
        u8g2_DrawStr(&u8g2, 0+12+2+8, 33, "ID:");
        snprintf(dis_char_buf, 50, "0x%02X", motor_id);
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
        u8g2_DrawStr(&u8g2, 0+12+2+8+12, 33, dis_char_buf);       
        u8g2_DrawLine(&u8g2, 0+12+2, 36, 64, 36); 

        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 0+16+2, 45, "BPS:");
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        snprintf(dis_char_buf, 50, "%d", bps_list[bps_index]);
        u8g2_DrawStr(&u8g2, 11+17+6, 45, dis_char_buf);        
    }
    else if (comm_type == COMM_TYPE_485_I2C) {
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf); /*字库选择*/
        switch (motor_mode)
        {
        case MODE_SPEED:
        case MODE_SPEED_ERR_PROTECT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:SPEED");
            break;
        case MODE_POS:
        case MODE_POS_ERR_PROTECT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:POS");
            break;
        case MODE_CURRENT:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:CURRENT");
            break;
        case MODE_DIAL:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:ENCODER");
            break;
        case MODE_POS_SPEED:
            u8g2_DrawStr(&u8g2, 0+12+2, 9, "MODE:POS+");
            break;            
        
        default:
            break;
        }
        u8g2_DrawLine(&u8g2, 12+2, 12, 64, 12);    

        u8g2_DrawStr(&u8g2, 0+12+2, 21, "COM:");
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 0+12+2+15, 21, "485"); 
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);  
        u8g2_DrawStr(&u8g2, 0+12+2+15+15, 21, "-");  
        u8g2_DrawStr(&u8g2, 0+12+2+15+15+3, 21, ">");  
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
        u8g2_DrawStr(&u8g2, 0+12+2+15+15+3+2, 21, "I2C");  
        u8g2_DrawLine(&u8g2, 0+12+2, 24, 64, 24);

        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
        u8g2_DrawStr(&u8g2, 0+12+2+8, 33, "ID:");
        snprintf(dis_char_buf, 50, "0x%02X", motor_id);
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf);
        u8g2_DrawStr(&u8g2, 0+12+2+8+12, 33, dis_char_buf);       
        u8g2_DrawLine(&u8g2, 0+12+2, 36, 64, 36); 

        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf); /*字库选择*/
        u8g2_DrawStr(&u8g2, 0+16+2, 45, "BPS:");
        u8g2_SetFont(&u8g2, u8g2_font_5x7_tf); /*字库选择*/
        snprintf(dis_char_buf, 50, "%d", bps_list[bps_index]);
        u8g2_DrawStr(&u8g2, 11+17+6, 45, dis_char_buf);   
    }    

    u8g2_SendBuffer(&u8g2); 
    
}

void u8g2_disp_ovp(void)
{
    char dis_char_buf[50] = {0};

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0+10+2, 0, 54, 48);

    u8g2_SetDrawColor(&u8g2, 1);

    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_2x_t);
    u8g2_DrawGlyph(&u8g2, 13+2+2, 20, 0x0079);
    u8g2_SetFont(&u8g2, u8g2_font_6x13B_mf);
    snprintf(dis_char_buf, 50, "E:%d", error_code);
    u8g2_DrawStr(&u8g2, 13+23+2, 17, dis_char_buf);    
    u8g2_DrawStr(&u8g2, 11+6+2, 33, "Over");    
    u8g2_DrawStr(&u8g2, 11+6+2, 43, "Voltage");    


    u8g2_SendBuffer(&u8g2); 
    
}

void u8g2_disp_stall(void)
{
    char dis_char_buf[50] = {0};

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0+10+2, 0, 54, 48);

    u8g2_SetDrawColor(&u8g2, 1);

    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_2x_t);
    u8g2_DrawGlyph(&u8g2, 13+2+2, 20, 0x0079);
    u8g2_SetFont(&u8g2, u8g2_font_6x13B_mf);
    snprintf(dis_char_buf, 50, "E:%d", error_code);
    u8g2_DrawStr(&u8g2, 13+23+2, 17, dis_char_buf);    
    u8g2_DrawStr(&u8g2, 11+6+2, 33, "Motor");    
    u8g2_DrawStr(&u8g2, 11+6+2, 43, "Jam");    


    u8g2_SendBuffer(&u8g2); 
    
}

void u8g2_disp_over_value(void)
{
    char dis_char_buf[50] = {0};

    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0+10+2, 0, 54, 48);

    u8g2_SetDrawColor(&u8g2, 1);

    u8g2_SetFont(&u8g2, u8g2_font_open_iconic_all_2x_t);
    u8g2_DrawGlyph(&u8g2, 13+2+2, 20, 0x0079);
    u8g2_SetFont(&u8g2, u8g2_font_6x13B_mf);
    snprintf(dis_char_buf, 50, "E:%d", error_code);
    u8g2_DrawStr(&u8g2, 13+23+2, 17, dis_char_buf);    
    u8g2_DrawStr(&u8g2, 11+6+2, 33, "Over");    
    u8g2_DrawStr(&u8g2, 11+6+2, 43, "Range");    


    u8g2_SendBuffer(&u8g2); 
    
}

void u8g2_disp_update_status(void)
{
    if (sys_status == SYS_STANDBY) {
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawBox(&u8g2, 0, 0, 9, 10);
        u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
        // u8g2_DrawGlyph(&u8g2, 1, 4, 0x0051); 
        // u8g2_DrawGlyph(&u8g2, 1, 12, 0x0051); 
        // u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
        // u8g2_DrawStr(&u8g2, 0, 9, "*");
        // u8g2_DrawStr(&u8g2, 5, 9, "*");
        u8g2_SendBuffer(&u8g2);  
    }
    else if (sys_status == SYS_RUNNING) {
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawBox(&u8g2, 0, 0, 9, 10);
        u8g2_SetDrawColor(&u8g2, 1);
        u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
        switch (running_index)
        {
        case 0:
            u8g2_DrawStr(&u8g2, 0, 9, "*"); 
            break;
        case 1:
            u8g2_DrawStr(&u8g2, 5, 9, "*"); 
            break;
        // case 2:
        //     u8g2_DrawGlyph(&u8g2, 0, 8, 0x25D1); 
        //     break;
        // case 3:
        //     u8g2_DrawGlyph(&u8g2, 0, 8, 0x25D5); 
        //     break;
        // case 4:
        //     u8g2_DrawGlyph(&u8g2, 0, 8, 0x25CF); 
        //     break;
        // case 5:
        //     u8g2_DrawGlyph(&u8g2, 0, 8, 0x25D5); 
        //     break;
        // case 6:
        //     u8g2_DrawGlyph(&u8g2, 0, 8, 0x25D1); 
        //     break;
        // case 7:
        //     u8g2_DrawGlyph(&u8g2, 0, 8, 0x25D4); 
        //     break;
        
        default:
            break;
        }
        u8g2_SendBuffer(&u8g2);
    }
}

void u8g2_disp_update_comm(void)
{
    static uint32_t comm_delay = 0;
    static uint8_t comm_flag = 0;

    if (comm_type == COMM_TYPE_I2C) {
        rgb_flash_color = 0x001B082C;
    }
    else if (comm_type == COMM_TYPE_485) {
        rgb_flash_color = 0x00002532;
    }

    if (comm_flash_count) {
        if (comm_flag) {
            if (comm_type == COMM_TYPE_I2C) {
                u8g2_SetDrawColor(&u8g2, 0);
                u8g2_DrawBox(&u8g2, 0, 48-8, 9+2, 8);
                // u8g2_DrawXBMP(&u8g2, 0, 48-8, 9+2, 8, gImage_comm_back_11x8);
                u8g2_SetDrawColor(&u8g2, 1);
                u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
                u8g2_DrawStr(&u8g2, 0, 48-1, "I2C");
            }
            else {
                u8g2_SetDrawColor(&u8g2, 0);
                u8g2_DrawBox(&u8g2, 0, 48-8, 9+2, 8);
                // u8g2_DrawXBMP(&u8g2, 0, 48-8, 9+2, 8, gImage_comm_back_11x8);
                u8g2_SetDrawColor(&u8g2, 1);
                u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
                u8g2_DrawStr(&u8g2, 0, 48-1, "485");
            }
            // neopixel_set_color(0, rgb_flash_color);
            // neopixel_set_color(1, rgb_flash_color);
        }
        else {
            if (comm_type == COMM_TYPE_I2C) {
                u8g2_SetDrawColor(&u8g2, 1);
                u8g2_DrawBox(&u8g2, 0, 48-8, 9+2, 8);
                // u8g2_DrawXBMP(&u8g2, 0, 48-8, 9+2, 8, gImage_comm_back_11x8);
                u8g2_SetDrawColor(&u8g2, 0);
                u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
                u8g2_DrawStr(&u8g2, 0, 48-1, "I2C");
                u8g2_SetDrawColor(&u8g2, 1);
            }
            else {
                u8g2_SetDrawColor(&u8g2, 1);
                u8g2_DrawBox(&u8g2, 0, 48-8, 9+2, 8);
                // u8g2_DrawXBMP(&u8g2, 0, 48-8, 9+2, 8, gImage_comm_back_11x8);
                u8g2_SetDrawColor(&u8g2, 0);
                u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
                u8g2_DrawStr(&u8g2, 0, 48-1, "485");
                u8g2_SetDrawColor(&u8g2, 1);
            }
            // neopixel_set_color(0, rgb_color);
            // neopixel_set_color(1, rgb_color);            
        }
    } 
    else {
        // u8g2_SetDrawColor(&u8g2, 0);
        // u8g2_DrawBox(&u8g2, 0, 48-8, 9, 13);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
        if (comm_type == COMM_TYPE_I2C) {
            u8g2_SetDrawColor(&u8g2, 1);
            u8g2_DrawBox(&u8g2, 0, 48-8, 9+2, 8);
            // u8g2_DrawXBMP(&u8g2, 0, 48-8, 9+2, 8, gImage_comm_back_11x8);
            u8g2_SetDrawColor(&u8g2, 0);
            u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
            u8g2_DrawStr(&u8g2, 0, 48-1, "I2C");
            u8g2_SetDrawColor(&u8g2, 1);
        }
        else {
            u8g2_SetDrawColor(&u8g2, 1);
            u8g2_DrawBox(&u8g2, 0, 48-8, 9+2, 8);
            // u8g2_DrawXBMP(&u8g2, 0, 48-8, 9+2, 8, gImage_comm_back_11x8);
            u8g2_SetDrawColor(&u8g2, 0);
            u8g2_SetFont(&u8g2, u8g2_font_micro_tr);
            u8g2_DrawStr(&u8g2, 0, 48-1, "485");
            u8g2_SetDrawColor(&u8g2, 1);
        }
        // neopixel_set_color(0, rgb_color);
        // neopixel_set_color(1, rgb_color);                 
    }   
    // ws2812_show();

    if (comm_delay < HAL_GetTick()) {
        if (comm_flag)
            comm_flag = 0;
        else
            comm_flag = 1;
        if (comm_flash_count)
            comm_flash_count--;
        comm_delay = HAL_GetTick() + 250;
    }
}

void ws2812_flash(void)
{
    static uint32_t flash_delay = 0;
    static uint8_t flash_flag = 0;


    if (sys_status == SYS_ERROR) {
        if (!rgb_show_mode) {
            rgb_color = 0x320000;
            neopixel_set_color(0, rgb_color);
            neopixel_set_color(1, rgb_color); 					
            ws2812_show();
        }
    }
    else {
        switch (motor_mode)
        {
        case MODE_SPEED:
        case MODE_SPEED_ERR_PROTECT:
            rgb_color = 0x003200;
            break;
        case MODE_POS:
        case MODE_POS_SPEED:
        case MODE_POS_ERR_PROTECT:
            rgb_color = 0x000032;
            break;
        case MODE_CURRENT:
            rgb_color = 0x323200;
            break;
        case MODE_DIAL:
            rgb_flash_slow = 1;
            rgb_color = 0x100010;
            break;
    
        default:
            break;
        }

        if (rgb_flash_flag) {
            if (rgb_flash_slow) {
                if (flash_delay < HAL_GetTick()) {
                    if (flash_flag)
                        flash_flag = 0;
                    else
                        flash_flag = 1;
                    flash_delay = HAL_GetTick() + 500;
                }
            }
            else {
                if (flash_delay < HAL_GetTick()) {
                    if (flash_flag)
                        flash_flag = 0;
                    else
                        flash_flag = 1;               
                    flash_delay = HAL_GetTick() + 150;
                }          
            }

            if (!rgb_show_mode) {
                if (flash_flag) {
                    neopixel_set_color(0, rgb_color);
                    neopixel_set_color(1, rgb_color);         
                }
                else {
                    neopixel_set_color(0, 0);
                    neopixel_set_color(1, 0);        
                }
                ws2812_show(); 
            }
        }
        else {
            if (!rgb_show_mode) {
                neopixel_set_color(0, rgb_color);
                neopixel_set_color(1, rgb_color);
                ws2812_show();  
            }
        }
    }
}

void u8g2_disp_update_page(void)
{       
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 0, 10, 9, 64-10-30+4);
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawVLine(&u8g2, 9, 0, 48);
    if (dis_show_flag < DIS_MAX) {
        switch (dis_show_flag)
        {
        case DIS_GRAPHY:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 26+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 20+2-2-4, ".");
            u8g2_DrawStr(&u8g2, 2, 32+2-1-4, ".");             
            u8g2_DrawStr(&u8g2, 2, 38+2-1-4, ".");             
            break;
        case DIS_INFO:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 20+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 26+2-1-4, ".");
            u8g2_DrawStr(&u8g2, 2, 32+2-2-4, "."); 
            u8g2_DrawStr(&u8g2, 2, 38+2-2-4, "."); 
            break;
        case DIS_CHAR:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 32+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 20+2-2-4, ".");
            u8g2_DrawStr(&u8g2, 2, 26+2-2-4, ".");             
            u8g2_DrawStr(&u8g2, 2, 38+2-2-4, ".");             
            break;
        case DIS_PID:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 38+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 20+2-2-4, ".");
            u8g2_DrawStr(&u8g2, 2, 26+2-2-4, ".");             
            u8g2_DrawStr(&u8g2, 2, 32+2-2-4, ".");             
            break;
        
        default:
            break;
        }
    }
    else {
        switch (last_dis_show_flag)
        {
        case DIS_CHAR:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 26+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 20+2-2-4, ".");
            u8g2_DrawStr(&u8g2, 2, 32+2-1-4, ".");             
            u8g2_DrawStr(&u8g2, 2, 38+2-1-4, ".");             
            break;
        case DIS_GRAPHY:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 20+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 26+2-1-4, ".");
            u8g2_DrawStr(&u8g2, 2, 32+2-2-4, "."); 
            u8g2_DrawStr(&u8g2, 2, 38+2-2-4, "."); 
            break;
        case DIS_INFO:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 32+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 20+2-2-4, ".");
            u8g2_DrawStr(&u8g2, 2, 26+2-2-4, ".");             
            u8g2_DrawStr(&u8g2, 2, 38+2-2-4, ".");             
            break;
        case DIS_PID:
            u8g2_SetFont(&u8g2, u8g2_font_m2icon_5_tf);
            u8g2_DrawGlyph(&u8g2, 1, 38+2-4, 0x0053); 
            u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
            u8g2_DrawStr(&u8g2, 2, 20+2-2-4, ".");
            u8g2_DrawStr(&u8g2, 2, 26+2-2-4, ".");             
            u8g2_DrawStr(&u8g2, 2, 32+2-2-4, ".");             
            break;
        
        default:
            break;
        }        
    }

}

void u8g2_disp_update_vin(float vin)
{
    char vin_char_buf[50] = {0};
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawBox(&u8g2, 8+10, 0, 30, 14);
    snprintf(vin_char_buf, 50, "%.1fV", vin/100.0f);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_6x13B_mf);
    u8g2_DrawStr(&u8g2, 8+10, 13, vin_char_buf);
    u8g2_SendBuffer(&u8g2);
}

void u8g2_disp_update_mode(void)
{
    uint8_t draw_color_flag = 0;
    u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
    if (sys_status == SYS_STANDBY || sys_status == SYS_ERROR) {
        draw_color_flag = 0;
    }
    else if (sys_status == SYS_RUNNING) {
        switch (running_index)
        {
        case 0:
            draw_color_flag = 0; 
            break;
        case 1:
            draw_color_flag = 1;
            break;

        default:
            break;
        }
    }
    
    switch (motor_mode)
    {
    case MODE_SPEED:
    case MODE_SPEED_ERR_PROTECT:
        u8g2_SetDrawColor(&u8g2, !draw_color_flag);
        if (!draw_color_flag)
            u8g2_DrawXBMP(&u8g2, 0, 0+1, 8, 8, gImage_mode_back);
        else
            u8g2_DrawBox(&u8g2, 0, 0+1, 8, 8);
        u8g2_SetDrawColor(&u8g2, draw_color_flag);     
        u8g2_DrawStr(&u8g2, 2, 7+1, "S");
        u8g2_SetDrawColor(&u8g2, 1);
        break;
    case MODE_POS:
    case MODE_POS_SPEED:
    case MODE_POS_ERR_PROTECT:
        u8g2_SetDrawColor(&u8g2, !draw_color_flag);
        if (!draw_color_flag)
            u8g2_DrawXBMP(&u8g2, 0, 0+1, 8, 8, gImage_mode_back);
        else
            u8g2_DrawBox(&u8g2, 0, 0+1, 8, 8);
        u8g2_SetDrawColor(&u8g2, draw_color_flag);     
        u8g2_DrawStr(&u8g2, 2, 7+1, "P");
        u8g2_SetDrawColor(&u8g2, 1);
        break;
    case MODE_CURRENT:
        u8g2_SetDrawColor(&u8g2, !draw_color_flag);
        if (!draw_color_flag)
            u8g2_DrawXBMP(&u8g2, 0, 0+1, 8, 8, gImage_mode_back);
        else
            u8g2_DrawBox(&u8g2, 0, 0+1, 8, 8);
        u8g2_SetDrawColor(&u8g2, draw_color_flag);     
        u8g2_DrawStr(&u8g2, 2, 7+1, "C");
        u8g2_SetDrawColor(&u8g2, 1);
        break;
    case MODE_DIAL:
        u8g2_SetDrawColor(&u8g2, !draw_color_flag);
        if (!draw_color_flag)
            u8g2_DrawXBMP(&u8g2, 0, 0+1, 8, 8, gImage_mode_back);
        else
            u8g2_DrawBox(&u8g2, 0, 0+1, 8, 8);
        u8g2_SetDrawColor(&u8g2, draw_color_flag);     
        u8g2_DrawStr(&u8g2, 2, 7+1, "E");
        u8g2_SetDrawColor(&u8g2, 1);
        break;
    
    default:
        break;
    }
}

void u8g2_disp_menu_init(void)
{
    neopixel_set_color(0, 0x003200);
    neopixel_set_color(1, 0x003200); 
    ws2812_show();   
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 12, "> Quit"); 
    switch (comm_type)
    {
    case COMM_TYPE_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(I2C)");
        break;
    case COMM_TYPE_485:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485)");
        break;
    case COMM_TYPE_485_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485+)");
        break;        
    
    default:
        break;
    }  
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 36, "  I2C ADDR");
    u8g2_DrawStr(&u8g2, 0, 48, "  485 ID");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    is_menu_flag = 1;
    while (!HAL_GPIO_ReadPin(SYS_SW_GPIO_Port, SYS_SW_Pin));
}

void u8g2_disp_menu_update(void)
{
    MotorDriverSetMode(MDRV_MODE_RUN);
    init_smart_knob();
    motor_mode = MODE_DIAL;
    while (1)
    {
        // encoder_update();
        if (last_dial_counter != current_position) {
            if (current_position > last_dial_counter) {
                encoder_value_t.encoder_down = 1;
                last_dial_counter = current_position;
            } else {
                encoder_value_t.encoder_up = 1;
                last_dial_counter = current_position;
            }
        }
        button_update();
        current = table[funIndex].current_operation;//根据需要获取对应需要执行的函数
        (*current)();  
        if (!is_menu_flag) {
            MotorDriverSetMode(MDRV_MODE_OFF);
            motor_mode = flash_data[1];
            break;
        }
    }  
}

void u8g2_disp_menu_0_1(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 12, "> Quit"); 
    switch (comm_type)
    {
    case COMM_TYPE_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(I2C)");
        break;
    case COMM_TYPE_485:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485)");
        break;
    case COMM_TYPE_485_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485+)");
        break;        
    
    default:
        break;
    }  
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 36, "  I2C ADDR");
    u8g2_DrawStr(&u8g2, 0, 48, "  485 ID");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 

    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }
    if (my_button.was_click) {
        my_button.was_click = 0;
        is_menu_flag = 0;
    }
}

void u8g2_disp_menu_0_2(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 12, "  Quit"); 
    switch (comm_type)
    {
    case COMM_TYPE_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "> COM(I2C)");
        break;
    case COMM_TYPE_485:
        u8g2_DrawStr(&u8g2, 0, 24, "> COM(485)");
        break;
    case COMM_TYPE_485_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "> COM(485+)");
        break;        
    
    default:
        break;
    }  
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 36, "  I2C ADDR");
    u8g2_DrawStr(&u8g2, 0, 48, "  485 ID");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }   
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_0_3(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 12, "  Quit"); 
    switch (comm_type)
    {
    case COMM_TYPE_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(I2C)");
        break;
    case COMM_TYPE_485:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485)");
        break;
    case COMM_TYPE_485_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485+)");
        break;        
    
    default:
        break;
    }  
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 36, "> I2C ADDR");
    u8g2_DrawStr(&u8g2, 0, 48, "  485 ID");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }   
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_0_4(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 12, "  Quit"); 
    switch (comm_type)
    {
    case COMM_TYPE_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(I2C)");
        break;
    case COMM_TYPE_485:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485)");
        break;
    case COMM_TYPE_485_I2C:
        u8g2_DrawStr(&u8g2, 0, 24, "  COM(485+)");
        break;        
    
    default:
        break;
    }  
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 36, "  I2C ADDR");
    u8g2_DrawStr(&u8g2, 0, 48, "> 485 ID");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }   
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_0_5(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    switch (comm_type)
    {
    case COMM_TYPE_I2C:
        u8g2_DrawStr(&u8g2, 0, 12, "  COM(I2C)");
        break;
    case COMM_TYPE_485:
        u8g2_DrawStr(&u8g2, 0, 12, "  COM(485)");
        break;
    case COMM_TYPE_485_I2C:
        u8g2_DrawStr(&u8g2, 0, 12, "  COM(485+)");
        break;
    
    default:
        break;
    }    
    u8g2_DrawStr(&u8g2, 0, 24, "  I2C_ADDR"); 
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 36, "  485 ID");
    u8g2_DrawStr(&u8g2, 0, 48, "> POS PID");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }  
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_0_6(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
  
    u8g2_DrawStr(&u8g2, 0, 12, "  I2C_ADDR"); 
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 24, "  485 ID");
    u8g2_DrawStr(&u8g2, 0, 36, "  POS PID");
    u8g2_DrawStr(&u8g2, 0, 48, "> SPEED");
    u8g2_DrawStr(&u8g2, 46, 48, "PID");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }  
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_0_7(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
  
    // u8g2_DrawStr(&u8g2, 0, 12, "  I2C_ADDR"); 
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 12, "  485 ID");
    u8g2_DrawStr(&u8g2, 0, 24, "  POS PID");
    u8g2_DrawStr(&u8g2, 0, 36, "  SPEED");
    u8g2_DrawStr(&u8g2, 46, 36, "PID");
    u8g2_DrawStr(&u8g2, 0, 48, "> BPS");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }  
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_0_8(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
  
    // u8g2_DrawStr(&u8g2, 0, 12, "  I2C_ADDR"); 
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 12, "  POS PID");
    u8g2_DrawStr(&u8g2, 0, 24, "  SPEED");
    u8g2_DrawStr(&u8g2, 46, 24, "PID");
    u8g2_DrawStr(&u8g2, 0, 36, "  BPS");
    u8g2_DrawStr(&u8g2, 0, 48, "> RGB%");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }  
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_0_9(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
  
    // u8g2_DrawStr(&u8g2, 0, 12, "  I2C_ADDR"); 
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 12, "  SPEED");
    u8g2_DrawStr(&u8g2, 46, 12, "PID");
    u8g2_DrawStr(&u8g2, 0, 24, "  BPS");
    u8g2_DrawStr(&u8g2, 0, 36, "  RGB%");
    u8g2_DrawStr(&u8g2, 0, 48, "> RGB");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }  
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_0_10(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
  
    // u8g2_DrawStr(&u8g2, 0, 12, "  I2C_ADDR"); 
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 12, "  BPS");
    u8g2_DrawStr(&u8g2, 0, 24, "  RGB%");
    u8g2_DrawStr(&u8g2, 0, 36, "  RGB");
    u8g2_DrawStr(&u8g2, 0, 48, "> JAM");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }  
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_0_11(void)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
  
    // u8g2_DrawStr(&u8g2, 0, 12, "  I2C_ADDR"); 
    // u8g2_DrawStr(&u8g2, 33, 36, " Load"); 
    u8g2_DrawStr(&u8g2, 0, 12, "  RGB%");
    u8g2_DrawStr(&u8g2, 0, 24, "  RGB");
    u8g2_DrawStr(&u8g2, 0, 36, "  JAM");
    u8g2_DrawStr(&u8g2, 0, 48, "> RANGE");
    // u8g2_DrawVLine(&u8g2, 17, 0, 48);
    // u8g2_DrawXBMP(&u8g2, 0, 17, 16, 16, gImage_lun);
    u8g2_SendBuffer(&u8g2); 
    if (encoder_value_t.encoder_down) {
        funIndex = table[funIndex].down;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up) {
        funIndex = table[funIndex].up;
        encoder_value_t.encoder_up = 0;
    }  
    if (my_button.was_click) {
        funIndex = table[funIndex].enter;
        last_btn_status = my_button.button_status;        
        my_button.was_click = 0;
    }       
}

void u8g2_disp_menu_1_1(void)
{
    char dis_buffer[20] = {0};
    static uint8_t opt_index = 0;
    static uint32_t button_delay = 0;
    static uint8_t is_press_init = 0;
  
    if (encoder_value_t.encoder_down)  {
        motor_id++;
        if (motor_id > 255)
            motor_id = 0;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (motor_id > 0)
					motor_id--;
				else
					motor_id = 255;
        encoder_value_t.encoder_up = 0;
    }
    snprintf(dis_buffer, 14, "ID=%03d", motor_id);
		
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 17, 12, "<485>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 14, 34, dis_buffer);

    u8g2_SendBuffer(&u8g2); 
    
    if (my_button.was_click) {
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }             
}

void u8g2_disp_menu_2_1(void)
{
    char dis_buffer_line_1[20] = {0};
    char dis_buffer_line_2[20] = {0};
    char dis_buffer_line_3[20] = {0};
    static uint8_t opt_index = 0;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        opt_index = comm_type;
        init_flag = 1;
    }
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 17, 12, "<COM>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    switch (comm_type)
    {
    case COMM_TYPE_I2C:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), "*I2C");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " 485");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " 485->I2C");
        break;
    case COMM_TYPE_485:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " I2C");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), "*485");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " 485->I2C");
        break;
    case COMM_TYPE_485_I2C:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " I2C");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " 485");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), "*485->I2C");
        break;
    
    default:
        break;
    }

    if (encoder_value_t.encoder_down)  {
        opt_index++;
        if (opt_index >= COMM_TYPE_MAX) {
            opt_index = COMM_TYPE_I2C;
        }
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (opt_index > COMM_TYPE_I2C)
            opt_index--;
        else
            opt_index = COMM_TYPE_485_I2C;
        encoder_value_t.encoder_up = 0;
    }    

    switch (opt_index)
    {
    case COMM_TYPE_I2C:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 16, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 25, ">");
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case COMM_TYPE_485:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 35, ">");
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case COMM_TYPE_485_I2C:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 45, ">");
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    
    default:
        break;
    }
     
   


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        comm_type = opt_index;
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_3_1(void)
{
    char dis_buffer[20] = {0};
    static uint8_t opt_index = 0;
    static uint32_t button_delay = 0;
    static uint8_t is_press_init = 0;
  
    if (encoder_value_t.encoder_down)  {
        i2c_address[0]++;
        if (i2c_address[0] > 127)
            i2c_address[0] = 0;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (i2c_address[0] > 0)
					i2c_address[0]--;
				else
					i2c_address[0] = 127;
        encoder_value_t.encoder_up = 0;
    }
    snprintf(dis_buffer, 20, "ADDR=0x%02X", i2c_address[0]);
		
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 17, 12, "<I2C>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 5, 34, dis_buffer); 


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }         
}

void Drawgauge(int x, int y, int r, int p, int v, int minVal, int maxVal) 
{
    char speed_char_buf[50] = {0};
    uint8_t dis_char_pos = 0;
    int v_abs = 0;

    if (v < 0)
        v_abs = -v;
    else
        v_abs = v;

    // u8g2_ClearBuffer(&u8g2);
    // u8g2_DrawCircle(&u8g2, x, y, r, U8G2_DRAW_UPPER_LEFT|U8G2_DRAW_UPPER_RIGHT);
    // u8g2_DrawCircle(&u8g2, x, y, radius2, U8G2_DRAW_UPPER_LEFT|U8G2_DRAW_UPPER_RIGHT);
    // u8g2_DrawLine(&u8g2, x-r, y, x-radius2, y);
    // u8g2_DrawLine(&u8g2, x+r, y, x+radius2, y);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
    snprintf(speed_char_buf, 50, "%d", v);
    if (v_abs >= 1000)
        dis_char_pos = x - 8;
    else if (v_abs >= 100)
        dis_char_pos = x - 5;
    else if (v_abs >= 10)
        dis_char_pos = x - 4;
    else
        dis_char_pos = x - 1;
    u8g2_DrawStr(&u8g2, dis_char_pos, y+12-3, speed_char_buf); 
    int val = map(v, minVal, maxVal, 13, 37); 
    if (val > 25) {
        for (int a = 25; a < val; a+=1) {
            u8g2_DrawBox(&u8g2, a, 33-(a-25), 1, (a-25)*2);                
        }    
    }
    else if (val < 25) {
        for (int a = 25; a > val; a-=1) {
            u8g2_DrawBox(&u8g2, a, 33-(25-a), 1, (25-a)*2);                
        }           
    }

    
    // u8g2_SetDrawColor(&u8g2, 0);
    // u8g2_DrawFilledEllipse(&u8g2, x, y, 9, 9, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_UPPER_LEFT);
    // u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_SendBuffer(&u8g2);
}

void u8g2_disp_menu_4_1(void)
{
    char dis_buffer_line_1[20] = {0};
    char dis_buffer_line_2[20] = {0};
    char dis_buffer_line_3[20] = {0};
    char dis_buffer_line_4[20] = {0};
    static uint8_t opt_index = 0;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        opt_index = pos_pid_index;
        init_flag = 1;
    }
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 5, 12, "<POS PID>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
    switch (pos_pid_index)
    {
    case 0:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), "*User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " Mid Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), " Heavy Load");
        break;
    case 1:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), "*Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " Mid Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), " Heavy Load");
        break;
    case 2:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), "*Mid Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), " Heavy Load");    
        break;
    case 3:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " Mid-Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), "*Heavy-Load");    
        break;
    
    default:
        break;
    }

    if (encoder_value_t.encoder_down)  {
        opt_index++;
        if (opt_index >= 4) {
            opt_index = 0;
        }
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (opt_index > 0)
            opt_index--;
        else
            opt_index = 3;
        encoder_value_t.encoder_up = 0;
    }    

    switch (opt_index)
    {
    case 0:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 16, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 25, ">");
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 1:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 35, ">");
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 2:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 36, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 45, ">");
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);         
        break;
    case 3:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 36, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 45, ">");
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_4);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_2); 
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_3);         
        break;
    
    default:
        break;
    }
     
   


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        pos_pid_index = opt_index;
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_5_1(void)
{
    char dis_buffer_line_1[20] = {0};
    char dis_buffer_line_2[20] = {0};
    char dis_buffer_line_3[20] = {0};
    char dis_buffer_line_4[20] = {0};
    static uint8_t opt_index = 0;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        opt_index = speed_pid_index;
        init_flag = 1;
    }
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 0, 12, "<SPEED PID>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
    switch (speed_pid_index)
    {
    case 0:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), "*User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " Mid Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), " Heavy Load");
        break;
    case 1:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), "*Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " Mid Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), " Heavy Load");
        break;
    case 2:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), "*Mid Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), " Heavy Load");    
        break;
    case 3:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " User-Def");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " Light Load");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " Mid Load");
        snprintf(dis_buffer_line_4, sizeof(dis_buffer_line_4), "*Heavy Load");    
        break;
    
    default:
        break;
    }

    if (encoder_value_t.encoder_down)  {
        opt_index++;
        if (opt_index >= 4) {
            opt_index = 0;
        }
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (opt_index > 0)
            opt_index--;
        else
            opt_index = 3;
        encoder_value_t.encoder_up = 0;
    }    

    switch (opt_index)
    {
    case 0:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 16, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 25, ">");
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 1:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 35, ">");
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 2:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 36, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 45, ">");
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);         
        break;
    case 3:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 36, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 45, ">");
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_4);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_2); 
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_3);         
        break;
    
    default:
        break;
    }
     
   


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        speed_pid_index = opt_index;
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_6_1(void)
{
    char dis_buffer_line_1[20] = {0};
    char dis_buffer_line_2[20] = {0};
    char dis_buffer_line_3[20] = {0};
    char dis_buffer_line_4[20] = {0};
    static uint8_t opt_index = 0;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        opt_index = bps_index;
        init_flag = 1;
    }
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 17, 12, "<BPS>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    switch (bps_index)
    {
    case 0:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), "*115200");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " 19200");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " 9600");
        break;
    case 1:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " 115200");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), "*19200");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), " 9600");
        break;
    case 2:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " 115200");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " 19200");
        snprintf(dis_buffer_line_3, sizeof(dis_buffer_line_3), "*9600");
        break;
    
    default:
        break;
    }

    if (encoder_value_t.encoder_down)  {
        opt_index++;
        if (opt_index >= 3) {
            opt_index = 0;
        }
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (opt_index > 0)
            opt_index--;
        else
            opt_index = 2;
        encoder_value_t.encoder_up = 0;
    }    

    switch (opt_index)
    {
    case 0:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 16, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 25, ">");
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 1:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 35, ">");
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 2:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 36, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 45, ">");
        u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 10, 25, dis_buffer_line_1); 
        u8g2_DrawStr(&u8g2, 10, 35, dis_buffer_line_2);         
        break;
    
    default:
        break;
    }
     
   


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        bps_index = opt_index;
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_7_1(void)
{
    char dis_buffer[20] = {0};
    static uint8_t opt_index = 0;
    static uint32_t button_delay = 0;
    static uint8_t is_press_init = 0;

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 17, 12, "<RGB%>"); 
    u8g2_SetDrawColor(&u8g2, 1); 
  
    if (encoder_value_t.encoder_down)  {
        brightness_index++;
        if (brightness_index > 100)
            brightness_index = 0;
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (brightness_index > 0)
					brightness_index--;
				else
					brightness_index = 100;
        encoder_value_t.encoder_up = 0;
    }

    neopixel_set_color(0, 0x003200);
    neopixel_set_color(1, 0x003200);
    ws2812_show();  

    snprintf(dis_buffer, 14, "BRIGHT=%d", brightness_index);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 5, 34, dis_buffer);

    u8g2_SendBuffer(&u8g2); 
    
    if (my_button.was_click) {
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }             
}

void u8g2_disp_menu_8_1(void)
{
    char dis_buffer_line_1[20] = {0};
    char dis_buffer_line_2[20] = {0};
    char dis_buffer_line_3[20] = {0};
    char dis_buffer_line_4[20] = {0};
    static uint8_t opt_index = 0;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        opt_index = rgb_show_mode;
        init_flag = 1;
    }
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 17, 12, "<RGB>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
    switch (rgb_show_mode)
    {
    case 0:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), "*sys-default");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " user-define");
        break;
    case 1:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " sys-default");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), "*user-define");
        break;
    
    default:
        break;
    }

    if (encoder_value_t.encoder_down)  {
        opt_index++;
        if (opt_index >= 2) {
            opt_index = 0;
        }
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (opt_index > 0)
            opt_index--;
        else
            opt_index = 1;
        encoder_value_t.encoder_up = 0;
    }    

    switch (opt_index)
    {
    case 0:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 16, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 25, ">");
        u8g2_DrawStr(&u8g2, 5, 25, dis_buffer_line_1);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 5, 35, dis_buffer_line_2); 
        // u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 1:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 35, ">");
        u8g2_DrawStr(&u8g2, 5, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 5, 25, dis_buffer_line_1); 
        // u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    
    default:
        break;
    }
     
   


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        rgb_show_mode = opt_index;
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_9_1(void)
{
    char dis_buffer_line_1[20] = {0};
    char dis_buffer_line_2[20] = {0};
    char dis_buffer_line_3[20] = {0};
    char dis_buffer_line_4[20] = {0};
    static uint8_t opt_index = 0;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        opt_index = motor_stall_protection_flag;
        init_flag = 1;
    }
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 17, 12, "<JAM>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
    switch (motor_stall_protection_flag)
    {
    case 0:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), "*jam-off");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " jam-on");
        break;
    case 1:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " jam-off");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), "*jam-on");
        break;
    
    default:
        break;
    }

    if (encoder_value_t.encoder_down)  {
        opt_index++;
        if (opt_index >= 2) {
            opt_index = 0;
        }
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (opt_index > 0)
            opt_index--;
        else
            opt_index = 1;
        encoder_value_t.encoder_up = 0;
    }    

    switch (opt_index)
    {
    case 0:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 16, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 25, ">");
        u8g2_DrawStr(&u8g2, 5, 25, dis_buffer_line_1);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 5, 35, dis_buffer_line_2); 
        // u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 1:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 35, ">");
        u8g2_DrawStr(&u8g2, 5, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 5, 25, dis_buffer_line_1); 
        // u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    
    default:
        break;
    }
     
   


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        motor_stall_protection_flag = opt_index;
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }     
}

void u8g2_disp_menu_10_1(void)
{
    char dis_buffer_line_1[20] = {0};
    char dis_buffer_line_2[20] = {0};
    char dis_buffer_line_3[20] = {0};
    char dis_buffer_line_4[20] = {0};
    static uint8_t opt_index = 0;
    static uint8_t init_flag = 0;

    if (!init_flag) {
        opt_index = motor_overvalue_protection_flag;
        init_flag = 1;
    }
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_DrawBox(&u8g2, 0, 0, 64, 15);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_tf);
    u8g2_SetDrawColor(&u8g2, 0);
    u8g2_DrawStr(&u8g2, 11, 12, "<RANGE>"); 
    u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_DrawHLine(&u8g2, 0, 15, 64);

    u8g2_SetFont(&u8g2, u8g2_font_5x8_tf);
    switch (motor_overvalue_protection_flag)
    {
    case 0:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), "*range-off");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), " range-on");
        break;
    case 1:
        snprintf(dis_buffer_line_1, sizeof(dis_buffer_line_1), " range-off");
        snprintf(dis_buffer_line_2, sizeof(dis_buffer_line_2), "*range-on");
        break;
    
    default:
        break;
    }

    if (encoder_value_t.encoder_down)  {
        opt_index++;
        if (opt_index >= 2) {
            opt_index = 0;
        }
        encoder_value_t.encoder_down = 0;
    }
    if (encoder_value_t.encoder_up)  {
        if (opt_index > 0)
            opt_index--;
        else
            opt_index = 1;
        encoder_value_t.encoder_up = 0;
    }    

    switch (opt_index)
    {
    case 0:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 16, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 25, ">");
        u8g2_DrawStr(&u8g2, 5, 25, dis_buffer_line_1);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 5, 35, dis_buffer_line_2); 
        // u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    case 1:
        // u8g2_SetDrawColor(&u8g2, 1);
        // u8g2_DrawBox(&u8g2, 0, 26, 64, 10);
        // u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, 0, 35, ">");
        u8g2_DrawStr(&u8g2, 5, 35, dis_buffer_line_2);
        // u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawStr(&u8g2, 5, 25, dis_buffer_line_1); 
        // u8g2_DrawStr(&u8g2, 10, 45, dis_buffer_line_3);         
        break;
    
    default:
        break;
    }
     
   


     
    u8g2_SendBuffer(&u8g2); 

    if (my_button.was_click) {
        motor_overvalue_protection_flag = opt_index;
        MotorDriverSetMode(MDRV_MODE_OFF);
        motor_mode = flash_data[1];
        flash_data_write_back();
        motor_mode = MODE_DIAL;
        MotorDriverSetMode(MDRV_MODE_RUN);
        funIndex = table[funIndex].exit;        
        my_button.was_click = 0;
    }     
}

void DrawPos(int x, int y, int r, int p, int v, int minVal, int maxVal) 
{
    char speed_char_buf[50] = {0};
    uint8_t dis_char_pos = 0;
    int dis_v = 0;

    if (v > 360)
        dis_v = v % 360;
    else
        dis_v = v;

    // u8g2_ClearBuffer(&u8g2);
    u8g2_DrawCircle(&u8g2, x, y, r, U8G2_DRAW_ALL);
    // u8g2_DrawCircle(&u8g2, x, y, radius2, U8G2_DRAW_ALL);
    // u8g2_DrawLine(&u8g2, cx-r, cy, cx-radius2, cy);
    // u8g2_DrawLine(&u8g2, cx+r, cy, cx+radius2, cy);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
    snprintf(speed_char_buf, 50, "%d", v);
    if (v >= 1000)
        dis_char_pos = x - 8;
    else if (v >= 100)
        dis_char_pos = x - 5;
    else if (v >= 10)
        dis_char_pos = x - 4;
    else
        dis_char_pos = x - 1;
    u8g2_DrawStr(&u8g2, dis_char_pos, y+18, speed_char_buf);
    int val = map(dis_v, minVal, maxVal, 0, 360); 
    float val_f = (float)val*3.14f/180.0f -1.572f;
    int xp = x+(sinf(val_f) * radius);
    int yp = y-(cosf(val_f) * radius);
    u8g2_DrawLine(&u8g2,x,y,xp,yp);
    u8g2_DrawLine(&u8g2,x-1,y-1,xp,yp);
    u8g2_DrawLine(&u8g2,x-1,y,xp,yp);
    u8g2_DrawLine(&u8g2,x+1,y,xp,yp);
    // u8g2_SendBuffer(&u8g2);
}

void DrawCurrent(int x, int y, int r, int p, int v, int minVal, int maxVal) 
{
    char current_char_buf[50] = {0};
    uint8_t dis_char_pos = 0;

    // u8g2_ClearBuffer(&u8g2);
    // u8g2_DrawCircle(&u8g2, x, y, r, U8G2_DRAW_UPPER_LEFT|U8G2_DRAW_UPPER_RIGHT);
    // u8g2_DrawCircle(&u8g2, x, y, radius2, U8G2_DRAW_UPPER_LEFT|U8G2_DRAW_UPPER_RIGHT);
    // u8g2_DrawLine(&u8g2, x-r, y, x-radius2, y);
    // u8g2_DrawLine(&u8g2, x+r, y, x+radius2, y);
    u8g2_SetDrawColor(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
    if (v >= 10000)
        dis_char_pos = x - 9;
    else if (v >= 1000)
        dis_char_pos = x - 8;
    else if (v >= 100)
        dis_char_pos = x - 5;
    else if (v >= 10)
        dis_char_pos = x - 4;
    else if (v >= 0)
        dis_char_pos = x - 1;
    else if (v <= -10000)
        dis_char_pos = x - 12;
    else if (v <= -1000)
        dis_char_pos = x - 9;
    else if (v <= -100)
        dis_char_pos = x - 8;
    else if (v <= -10)
        dis_char_pos = x - 5;
    else if (v < 0)
        dis_char_pos = x - 4;
    else
        dis_char_pos = x - 1;
    if (v >= 0) {
        snprintf(current_char_buf, 50, "%d", v);
        u8g2_DrawStr(&u8g2, dis_char_pos, y+12-3, current_char_buf);
    }
    else {
        snprintf(current_char_buf, 50, "%d", -v);
        u8g2_SetFont(&u8g2, u8g2_font_micro_tr); 
        u8g2_DrawStr(&u8g2, dis_char_pos+2, y+12-3, "-");
        u8g2_SetFont(&u8g2, u8g2_font_4x6_mf);
        u8g2_DrawStr(&u8g2, dis_char_pos+5, y+12-3, current_char_buf);        
    }
        
    
    int val = map(v, minVal, maxVal, 39, 63); 
    if (val > 51) {
        for (int a = 51; a < val; a+=1) {
            u8g2_DrawBox(&u8g2, a, 33-(a-51), 1, (a-51)*2);                
        }    
    }
    else if (val < 51) {
        for (int a = 51; a > val; a-=1) {
            u8g2_DrawBox(&u8g2, a, 33-(51-a), 1, (51-a)*2);                
        }           
    }    
    // if (val > 90) {
    //     for (int a = 90; a < val; a+=5) {
    //         float val_f = (float)a*3.14f/180.0f -1.572f;
    //         int xp = x+(sinf(val_f) * radius);
    //         int yp = y-(cosf(val_f) * radius);
    //         u8g2_DrawLine(&u8g2,x,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x-1,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x-2,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x+1,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x+2,y,xp,yp);
    //     }
    // }
    // else if (val < 90) {
    //     for (int a = 90; a > val; a-=5) {
    //         float val_f = (float)a*3.14f/180.0f -1.572f;
    //         int xp = x+(sinf(val_f) * radius);
    //         int yp = y-(cosf(val_f) * radius);
    //         u8g2_DrawLine(&u8g2,x,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x-1,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x-2,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x+1,y,xp,yp);
    //         u8g2_DrawLine(&u8g2,x+2,y,xp,yp);
    //     }
    // }
    // u8g2_SetDrawColor(&u8g2, 0);
    // u8g2_DrawFilledEllipse(&u8g2, x, y, 9, 9, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_UPPER_LEFT);          
    // u8g2_SetDrawColor(&u8g2, 1);
    // u8g2_SendBuffer(&u8g2);
}