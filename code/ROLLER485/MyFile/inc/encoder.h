#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "tle5012b.h"

typedef struct{
    int32_t encoder_value;
    int32_t last_encoder_value;
    int32_t encoder_up_down_value;
    int32_t last_encoder_up_down_value;
    uint8_t encoder_down;
    uint8_t encoder_up;	
    uint16_t last_feedback_position;
}encoder_value_typedef;

extern encoder_value_typedef encoder_value_t;
extern encoder_value_typedef speed_encoder_value_t;

void encoder_update(void);
void speed_encoder_update(void);

#endif