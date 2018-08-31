/**********************************************************
FILE NAME   : hw_init.c
DESCRIPTION : Init the hardware

Copyright   : Dave Allegre et al
            : 2015
	    : All Rights Reserved
**********************************************************/
/* Includes ------------------------------------------------------------------*/
#include "nucleoboard.h"
#include "bsp.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private prototype ---------------------------------------------------------*/
void hw_system_clock(void);
void hw_gpio_init(void);
void hw_uart_init(void);
void hw_nvic_init(void);


/**********************************************************
Function Name	: hw_init
Description	: Calls functions to "boot up" processor
                : 
Parameters	:
Return value	:					*/
void Hw_init(void) {
  
  hw_system_clock();            // Clock up..
  hw_gpio_init();               // System I/O config
  hw_uart_init();               // USART config

  hw_ic_tim_init();
  hw_pwm_tim_init();
  
  NVIC_SetPriority(PendSV_IRQn, 0xFF); // Lowest possible priority
}

/*******************************************************************************
Function Name	: hw_system_clock
 * Clock fan out is not well documented, see stm32f401 reference manual, section 6.2 
 * Examine how HSI gets to the GPIO block (GPIO is on AHB1)
 * see section 6.2.6
 * The system can switch to the PLL to go faster, see PLLCFGR, 6.3.2
 * There is no need for that here, so leave SYSCLK == HSI
 * Leave AHB1 == SYSCLK (PPRE2 = 0)
Parameters	: None
Return value	: None
Author          : Geoff Smith
*/
void hw_system_clock(void) {
uint32_t reg;
     
    /* Enable used clocks, see 6.3.9 */
    reg = RCC->APB1ENR;
    reg |= RCC_APB1ENR_PWREN;
    RCC->APB1ENR = reg;    
    
    reg= RCC->AHB1ENR;
    reg |= RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOEEN;
    RCC->AHB1ENR = reg;
    
    reg= RCC->APB2ENR;
    reg |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB2ENR = reg;

}
/*******************************************************************************
Function Name	: hw_gpio_init
Description	: Init the needed GPIO
Parameters	: None
Return value	: None
Author          : Dave Allegre  */
void hw_gpio_init(void) {
GPIO_InitTypeDef GPIO_InitStructure;

  // Enable the GPIO_PORT_LD2 peripheral clock 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LD2;           // LED
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIO_PORT_LD2, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_TESTD2;           // monitoring pin
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIO_PORT_LD2, &GPIO_InitStructure);           // Same GPIOA as for the LED

}

