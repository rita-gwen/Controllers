#include "bsp.h"

void SetLED(BOOLEAN On)
{
    if (On) {
        GPIO_SetBits(GPIO_PORT_LD2, GPIO_PIN_LD2);
      } else {
        /* The high 16 bits of BSRR reset the pin */
        GPIO_ResetBits(GPIO_PORT_LD2, GPIO_PIN_LD2);
     }
}

void SetPin(int pin_nr, BOOLEAN On)
{
  GPIO_TypeDef* port = (GPIO_TypeDef*)0;
  uint16_t pin;
  switch(pin_nr)
  {
  case 1:
     port = GPIOA;
     pin = GPIO_PIN_TESTD2;
     break;
  case 2:
     port = GPIOB;
     pin = GPIO_PIN_TESTD4;
     break;
  default:
    return;
  }
  if (On) {
      GPIO_SetBits(port, pin);
    } else {
      /* The high 16 bits of BSRR reset the pin */
      GPIO_ResetBits(port, pin);
   }
}