#include "nucleoboard.h"
#include "bsp.h"


MSREAD_ResultType motor_data_struct;
MSREAD_ResultType *motor_data = &motor_data_struct;

void SetMotorCommand(uint16_t pwm_duty){
    if(pwm_duty <= MPWM_COUNT_PERIOD)
      TIM_SetCompare1(MPWM_TIM, (uint32_t)pwm_duty);
}


//Interrupt handler for motor speed CC
void TIM3IrqHandler(void){
    OS_CPU_SR  cpu_sr;
    INT8U err;

    OSIntEnter();
    OS_ENTER_CRITICAL();                                        /* Tell uC/OS-II that we are starting an ISR            */

    motor_data->ic_raw_count_prev = motor_data->ic_raw_count;
    motor_data->ic_raw_count = TIM_GetCapture3(MSREAD_TIM);
    motor_data->ic_value = motor_data->ic_raw_count - motor_data->ic_raw_count_prev;
    motor_data->ic_timestamp = OSTime;
    motor_data->ic_error_flag = 0;
    OSFlagPost(motor_data->ic_flags, MSREAD_FLAG_REGULAR, OS_FLAG_SET, &err);
    TIM_ClearITPendingBit(MSREAD_TIM, TIM_IT_CC3);
//    TIM_ClearITPendingBit(MSREAD_TIM, TIM_IT_Update);
    
    OS_EXIT_CRITICAL();

    OSIntExit();                                                /* Tell uC/OS-II that we are leaving the ISR            */

}


//Configure TIM3 to read motor speed
void hw_ic_tim_init(void){
  
    GPIO_PinAFConfig(MSREAD_GPIO_PORT, MSREAD_GPIO_PIN_SRC, GPIO_AF_TIM3);
  
    //initialize GPIO pin for the timer input capture input (TIM3_CH3)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MSREAD_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  
    GPIO_Init(MSREAD_GPIO_PORT, &GPIO_InitStructure);

    //Enable timer interrupt in NVIC
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);        //start TIM3 clock
    //initialize timebase
    TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_BaseInitStruct);
    TIM_BaseInitStruct.TIM_Prescaler = MSREAD_COUNT_PRESCALER;
    TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    //TIM_BaseInitStruct.TIM_Period = 0x10;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(MSREAD_TIM, &TIM_BaseInitStruct);
    
    //Initialize input capture mode
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(MSREAD_TIM, &TIM_ICInitStructure);
    
    TIM_ITConfig(MSREAD_TIM, TIM_IT_CC3, ENABLE);
    //TIM_ITConfig(MSREAD_TIM, TIM_IT_Update, ENABLE);
          
    TIM_Cmd(MSREAD_TIM, ENABLE);
}

void hw_pwm_tim_init(void){
    //initialize GPIO pin for the timer PWM output
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MPWM_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
    GPIO_Init(MPWM_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(MPWM_GPIO_PORT, MPWM_GPIO_PIN_SRC, GPIO_AF_TIM5);
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);        //start TIM5 clock
    //initialize timebase
    TIM_TimeBaseInitTypeDef TIM_BaseInitStruct;
    TIM_TimeBaseStructInit(&TIM_BaseInitStruct);
    TIM_BaseInitStruct.TIM_Prescaler = 0x0;
    TIM_BaseInitStruct.TIM_CounterMode = TIM_CounterMode_Down;
    TIM_BaseInitStruct.TIM_Period = MPWM_COUNT_PERIOD;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(MPWM_TIM, &TIM_BaseInitStruct);
    
    //initialize PWM
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0x2;                   //start low
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OC1Init(MPWM_TIM, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(MPWM_TIM, TIM_OCPreload_Enable);       //allows for updating duty cycle on the fly
    
    TIM_Cmd(MPWM_TIM, ENABLE);    
}