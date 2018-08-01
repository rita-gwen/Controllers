#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

#include <stdint.h>

typedef struct{
    float motor_speed;
    float  error_signal;
    uint16_t  motor_cmd;
    int16_t  pot_angle;
    uint16_t headroom;
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

#endif