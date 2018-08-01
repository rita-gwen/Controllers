#include "controller.h"
#include <stdint.h>
#include <math.h>



uint16_t atoi(char* str)
{
    uint16_t res = 0;
    uint16_t i = 0;
    while(str[i] >= '0' & str[i] <= '9')
    {
      res = res*10+(str[i++] - '0');
    }
    return res;
}

uint8_t floatToUByte(float num)
{
    if(num > (float)0xFF)
      return (uint8_t)0xFF;
    if(num < (float)0)
      return (uint8_t)0;
      
    return (uint8_t)round(num);
}
    
uint8_t floatToByte(float num)
{
      if(num > (float)0x7F)
        return (uint8_t)0x7F;
      if(num < -(float)0x80)
        return (uint8_t)0x80;
    
      return (uint8_t)round(num);
}

float atofloat(char* str)
{
  float sign;
  if(*str == '-'){
    sign = -1;
    str++;
  }
  else if(*str == '+'){
    sign = 1;
    str++;
  }
  else 
    sign = 1;
  
  float res = 0;
  float decPos = 1;
  uint8_t pointFlag = 0;
  while(*str >='0' & *str <= '9' | *str =='.')
  {
    if(*str == '.')
      pointFlag = 1;
    else
    {
      res = res * 10 + (*str - '0');
      if(pointFlag)     decPos *= 10.;
    }
    str++;
  }
  return sign * res/decPos;
}