/**********************************************************
FILE NAME  	: nucleoboard.h
DESCRIPTION     : Board specific definitions live here
                : These generally come from the Nucleo schematics
Copyright	: David Allegre
		: 2015
		: All Rights Reserved
**********************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NUCLEOBOARD_H
#define __NUCLEOBOARD_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"

/* Exported define -----------------------------------------------------------*/
// Clock speed
#define CLOCK_HSI               HSI_VALUE

// User button
#define GPIO_PIN_USER_BUTTON    GPIO_Pin_13
#define GPIO_PORT_USER_BUTTON   GPIOC

// Green LED
#define GPIO_PIN_LD2            GPIO_Pin_5
#define GPIO_PORT_LD2           GPIOA
#define GPIO_PIN_TESTD2         GPIO_Pin_10           //monitoring pin for debugging purposes
#define GPIO_PIN_TESTD4         GPIO_Pin_5           //monitoring pin for debugging purposes

#endif