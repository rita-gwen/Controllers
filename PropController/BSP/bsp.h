#ifndef __BSP_H
#define __BSP_H

// Add interfaces to the specific hardware for use by application code
//#include <os_cpu.h>
//#include <os_cfg.h>
#include <app_cfg.h>
#include <ucos_ii.h>

#include "nucleoboard.h"
#include "hw_init.h"
#include "uart.h"
#include "adc.h"
#include "tim.h"
#include "uart.h"
#include "controller.h"

#define ARM_MATH_CM4
#define LAB4


void SetLED(BOOLEAN On);

#define TEST_PIN1       1
#define TEST_PIN2       2

void SetPin(int pin_nr, BOOLEAN On);

#endif /* __BSP_H */