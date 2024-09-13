    
#include "encoder.h"
#include "motordriver.h"

uint8_t encoder_enable_flag = 1;
uint8_t encoder_init_flag = 0;
uint8_t speed_encoder_init_flag = 0;

encoder_value_typedef encoder_value_t;
encoder_value_typedef speed_encoder_value_t;

void encoder_update(void)
{
    if (encoder_enable_flag) {
      if(angle_corrected < encoder_value_t.last_feedback_position && 
      encoder_value_t.last_feedback_position > 12384 && angle_corrected < 4000){
        encoder_value_t.encoder_value += (16383 - encoder_value_t.last_feedback_position);
        encoder_value_t.encoder_value += angle_corrected;
        encoder_value_t.last_feedback_position = angle_corrected;
      } else if(angle_corrected > encoder_value_t.last_feedback_position && 
      encoder_value_t.last_feedback_position < 4000 && angle_corrected > 12384){
        encoder_value_t.encoder_value -= (16383 - angle_corrected);
        encoder_value_t.encoder_value -= encoder_value_t.last_feedback_position;
        encoder_value_t.last_feedback_position = angle_corrected;
      } else if (angle_corrected != encoder_value_t.last_feedback_position) {
        encoder_value_t.encoder_value += (angle_corrected - encoder_value_t.last_feedback_position);
        encoder_value_t.last_feedback_position = angle_corrected;
      }

      encoder_value_t.encoder_up_down_value = encoder_value_t.encoder_value / 200;   
      if (encoder_value_t.last_encoder_up_down_value != encoder_value_t.encoder_up_down_value) {
        if (encoder_value_t.encoder_up_down_value - encoder_value_t.last_encoder_up_down_value > 6) {
          encoder_value_t.encoder_down = 1;
          encoder_value_t.last_encoder_up_down_value = encoder_value_t.encoder_up_down_value;
        } else if (encoder_value_t.encoder_up_down_value - encoder_value_t.last_encoder_up_down_value < -6) {
          encoder_value_t.encoder_up = 1;
          encoder_value_t.last_encoder_up_down_value = encoder_value_t.encoder_up_down_value;
        }
      }      

      if(!encoder_init_flag) {
          encoder_value_t.last_feedback_position = angle_corrected;
          encoder_value_t.encoder_value = 0;
          encoder_value_t.encoder_down = 0;
          encoder_value_t.encoder_up = 0;
          encoder_init_flag = 1;
      }            
    }
}

void speed_encoder_update(void)
{
  if(!speed_encoder_init_flag) {
      speed_encoder_value_t.last_feedback_position = angle_corrected;
      speed_encoder_value_t.encoder_value = 0;
      speed_encoder_init_flag = 1;
  }

  if(angle_corrected < speed_encoder_value_t.last_feedback_position && 
  speed_encoder_value_t.last_feedback_position > 12384 && angle_corrected < 4000){
    speed_encoder_value_t.encoder_value += (16383 - speed_encoder_value_t.last_feedback_position);
    speed_encoder_value_t.encoder_value += angle_corrected;
    speed_encoder_value_t.last_feedback_position = angle_corrected;
  } else if(angle_corrected > speed_encoder_value_t.last_feedback_position && 
  speed_encoder_value_t.last_feedback_position < 4000 && angle_corrected > 12384){
    speed_encoder_value_t.encoder_value -= (16383 - angle_corrected);
    speed_encoder_value_t.encoder_value -= speed_encoder_value_t.last_feedback_position;
    speed_encoder_value_t.last_feedback_position = angle_corrected;
  } else if (angle_corrected != speed_encoder_value_t.last_feedback_position) {
    speed_encoder_value_t.encoder_value += (angle_corrected - speed_encoder_value_t.last_feedback_position);
    speed_encoder_value_t.last_feedback_position = angle_corrected;
  }
}    
