#ifndef __GPIO_H_
#define __GPIO_H_

#include "main.h"
#include "gpio.h"

#define BUTTON_FILTER 500
#define BUTTON_FILTER_TIMEROUT BUTTON_FILTER*3

typedef struct
{
    uint8_t button_status;
    unsigned long button_delay;
    uint8_t is_pressed;
    uint8_t is_longlongpressed;
    uint8_t was_click;
    uint8_t was_longpress;
    uint8_t was_longlongpress;
} button_t;

extern button_t my_button;

void button_update(void);

#endif