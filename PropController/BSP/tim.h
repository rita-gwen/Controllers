#ifndef __TIM_H
#define __TIM_H


// PB0 maps to TIM3_CH3, pin A3 on Arduino connector. Use for motor speed CC

// motor speed measurement peripherals
#define MSREAD_GPIO_PORT        GPIOB
#define MSREAD_GPIO_PIN         GPIO_Pin_0
#define MSREAD_GPIO_PIN_SRC     GPIO_PinSource0
#define MSREAD_TIM              TIM3

// motor PWM (MPWM) output peripherals
#define MPWM_TIM                TIM4            //channel 3
#define MPWM_GPIO_PORT          GPIOB
#define MPWM_GPIO_PIN           GPIO_Pin_8      //pin PB8 (D15)
#define MPWM_GPIO_PIN_SRC       GPIO_PinSource8


#define MSREAD_COUNT_PRESCALER  1000       //prescaler for the timer clock to measure motor speed.
#define MPWM_COUNT_PRESCALER  5            //prescaler for the timer clock to generate motor PWM. Allows for duty cycle setting from 0 to 1000 
#define MPWM_COUNT_PERIOD      500         //about 32kHz PWM frequency

#define MSREAD_FLAG_REGULAR     1               //Flag for signalling IC event

typedef struct {
  uint16_t ic_value;       
  uint16_t ic_raw_count;       
  uint16_t ic_raw_count_prev;       
  uint16_t ic_error_flag;
  uint32_t ic_timestamp;       //timestamp of the last input capture (interrupt call)
  uint32_t ic_sum;              //variables to accumulate and average the reading 
  uint16_t ic_count;
  OS_FLAG_GRP* ic_flags;
} MSREAD_ResultType;


void hw_ic_tim_init(void);
void hw_pwm_tim_init(void);
void SetMotorCommand(uint16_t pwm_duty);

#ifdef __cplusplus
  extern "C" {
#endif 
   
void TIM3IrqHandler(void);

#ifdef __cplusplus
  }
#endif 
   


#endif /* __TIM_H */