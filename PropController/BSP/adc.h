#ifndef __ADC_H
#define __ADC_H


//ADC
#define GPIO_PORT_ADC           GPIOA
#define GPIO_PIN_ADC            GPIO_Pin_4      // pot ADC connection
#define ADC_Channel_POT         ADC_Channel_4

//Flags to identify what type of conversion happened
#define ADC_FLAG_REGULAR        1
#define ADC_FLAG_INJECTED       2
#define ADC_FLAG_ERROR          4

#define ADC_CONVERSION_FREQUENCY        20             // in Hz
#define ADC_TIM                         TIM2
#define ADC_GPIO_MONITOR_PIN            GPIO_Pin_1      //output triggering PWM signal to this pin for debugging (TIM2_CH2). Pin A0 in Arduino connector
#define ADC_GPIO_MONITOR_PIN_SRC        GPIO_PinSource1



typedef struct {
  uint16_t adc_value;       
  uint16_t adc_error_flag;
  uint32_t adc_timestamp;       //timestamp of the last ADC conversion (interrupt call)
  uint8_t  adc_conversion_type; //indicates if the last conversion was regular or injected
  OS_FLAG_GRP* adc_flags;
} ADC_ResultType;

void hw_adc_timer_init(void);
void hw_adc_init(void);

#endif /* __ADC_H */