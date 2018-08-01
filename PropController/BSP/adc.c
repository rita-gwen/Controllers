#include "nucleoboard.h"
#include "bsp.h"

ADC_ResultType pot_data_struct;

ADC_ResultType *pot_data = &pot_data_struct;


//ADC interrupt handler
void ADCIrqHandler(void){
    OS_CPU_SR  cpu_sr;
    INT8U err;

    OSIntEnter();
    OS_ENTER_CRITICAL();                                        /* Tell uC/OS-II that we are starting an ISR            */
    pot_data->adc_error_flag = 0;
    pot_data->adc_timestamp = TIM_GetCounter(ADC_TIM);
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET){
      pot_data->adc_value = ADC_GetConversionValue(ADC1);
      OSFlagPost(pot_data->adc_flags, ADC_FLAG_REGULAR, OS_FLAG_SET, &err);
      ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
    else{
      pot_data->adc_error_flag = 1;
      OSFlagPost(pot_data->adc_flags, ADC_FLAG_ERROR, OS_FLAG_SET, &err);
    }
    OS_EXIT_CRITICAL();

    OSIntExit();                                                /* Tell uC/OS-II that we are leaving the ISR            */
}

// Initialize the timer to trigger ADC conversions
void hw_adc_timer_init(void){

    //initialize GPIO pin for the timer PWM output
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ADC_GPIO_MONITOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIO_PORT_ADC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIO_PORT_ADC, ADC_GPIO_MONITOR_PIN_SRC, GPIO_AF_TIM2);
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);        //start TIM2 clock
    //initialize timebase
    TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_BaseInitStruct);
    TIM_BaseInitStruct.TIM_Prescaler = 0x1;
    TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseInitStruct.TIM_Period = CLOCK_HSI/ADC_CONVERSION_FREQUENCY;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(ADC_TIM, &TIM_BaseInitStruct);
    
    //initialize PWM
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0x200;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCNPolarity_High;

    TIM_OC2Init(ADC_TIM, &TIM_OCInitStruct);
    //TIM_OC2PreloadConfig(ADC_TIM, TIM_OCPreload_Enable); 
    
    TIM_Cmd(ADC_TIM, ENABLE);  
}


void hw_adc_init(void){
  
      // ADC channel 4 is on PA4 pin
      // the GPIO port clock is already enabled
      GPIO_InitTypeDef GPIO_InitStructure;
      GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ADC;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
      GPIO_Init(GPIO_PORT_ADC, &GPIO_InitStructure);

      //configure NVIC to handle ADC interrupts
      NVIC_InitTypeDef NVIC_InitStructure;

      NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
    
      //enable ADC clock
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

      /* ADC1 configuration ---------------------------------*/
      ADC_InitTypeDef  ADC_InitStructure;
      ADC_StructInit(&ADC_InitStructure);
      ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
      ADC_InitStructure.ADC_ScanConvMode = DISABLE;
      ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
      ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
      ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
      ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
      ADC_InitStructure.ADC_NbrOfConversion = 1;
      ADC_Init(ADC1, &ADC_InitStructure);
      
      ADC_CommonInitTypeDef ADC_CommonInitStructure;
      ADC_CommonStructInit(&ADC_CommonInitStructure);
      ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
      ADC_CommonInitStructure.ADC_Prescaler = 1;
      ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;

      /* ADC1 regular channels configuration */ 
      ADC_CommonInit(&ADC_CommonInitStructure);
      ADC_RegularChannelConfig(ADC1, ADC_Channel_POT, 1, ADC_SampleTime_15Cycles);

      /* Configure and enable ADC interrupts */
      ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

      //Enable ADC
      ADC_Cmd(ADC1, ENABLE);
      
}


