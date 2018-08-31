#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

#include <stdint.h>

#define POT_ZERO                1724.0f         
#define POT_READ_PER_DEGREE     18.8056f                     //Max degrees of rotation above and below zero (horizontal)

#define ERROR_BUFF_SIZE         10

#define ERROR_DELAY             1

class RingBuffer {
  uint8_t buffer_index;
  float ring_buffer_array[ERROR_BUFF_SIZE];
public:
  RingBuffer();
  void push(float value);
  float get(uint8_t offset = 0);
};


typedef struct{
    float motor_speed;
    float  error_signal;
    float  error_derivative;
    uint16_t  motor_cmd;
    float  pot_angle;
    uint16_t headroom;
    RingBuffer error_buffer;
    float error_sum;
} ProcessDataStructType;

typedef struct
{
    float     set_point;
    float     feedback_rate;
    float     deriv_rate;
    float     integral_rate;
    float     sum_max;
    float     direct;
    uint8_t   first_time;               //boolean
    
} ControllerDataStructType;



// Auxiliary functions
uint16_t atoi(char* str);
float atofloat(char* str);
uint8_t floatToUByte(float num);
uint8_t floatToByte(float num);
uint16_t floatToWord(float num);

// Controller functions
void initController(void);
void cntr_getProcessData();
uint16_t cntr_control(void);

#endif