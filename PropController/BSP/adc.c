#include "nucleoboard.h"
#include "bsp.h"
#include "arm_math.h"

AngleSensor angle_sensor_instance = AngleSensor();

AngleSensor *angle_sensor = &angle_sensor_instance;

//-------------------- IRQ handlers -----------------------
//ADC interrupt handler (inactive)
void ADCIrqHandler(void){

    OS_CPU_SR  cpu_sr;
    INT8U err;

    OSIntEnter();
  
    OS_ENTER_CRITICAL();                                        /* Tell uC/OS-II that we are starting an ISR            */
    angle_sensor->m_adc_timestamp = TIM_GetCounter(ADC_TIM);
    if(ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET){
      angle_sensor->m_adc_value[0] = ADC_GetConversionValue(ADC1);
      OSFlagPost(angle_sensor->m_adc_flags, ADC_FLAG_REGULAR, OS_FLAG_SET, &err);
      ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
    OS_EXIT_CRITICAL();

    OSIntExit();                                                /* Tell uC/OS-II that we are leaving the ISR            */
}
// DMA interrupt handler
void ADCDMAIrqHandler(void)
{
  OS_CPU_SR  cpu_sr;
  INT8U err;
  
  OSIntEnter();
  OS_ENTER_CRITICAL();

  if(DMA_GetITStatus(ADC_DMA_STREAM, DMA_IT_TCIF0))
  {
    angle_sensor->m_adc_value[0] = angle_sensor->m_dma_buffer[0];
    angle_sensor->m_adc_value[1] = angle_sensor->m_dma_buffer[1];
    OSFlagPost(angle_sensor->m_adc_flags, ADC_FLAG_REGULAR, OS_FLAG_SET, &err);
    DMA_ClearITPendingBit(ADC_DMA_STREAM, DMA_IT_TCIF0);
  }

  OS_EXIT_CRITICAL();

  OSIntExit();  
}

//-----------------------------------------------------------------

float AngleSensor::m_arctg[] = {0.0, 5.7106, 11.3099, 16.6992, 21.8014, 26.5651, 30.9638, 34.9920, 38.6598, 41.9872, 45.};
uint16_t adc_limits[] = {590, 590, 3615, 3615};

void AngleSensor::init_sensor(uint16_t conversionFrequency) 
{
  INT8U err;

  m_conversion_freq = conversionFrequency;    

  m_adc_flags = OSFlagCreate((OS_FLAGS)(0x00), &err);
  m_adc_value[0] = 0;
  m_adc_value[1] = 0;
  m_enabled = DISABLE;
  
  m_adc_min_value = adc_limits;
  m_adc_max_value = &(adc_limits[2]);
  m_current_func_type = 0;
  m_current_interval = -90.;
  m_current_angle = -90.;
  static_correction = +3.8f;     // without it the result reads -96 degrees when the arm is hanging still.
  
  //initialize the hardware
  hw_adc_timer_init();
  hw_adc_init();
  hw_adc_dma_init();
}


// Initialize the timer to trigger ADC conversions
void AngleSensor::hw_adc_timer_init(void){

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
    TIM_BaseInitStruct.TIM_Period = CLOCK_HSI/m_conversion_freq;
    TIM_BaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(ADC_TIM, &TIM_BaseInitStruct);
    
    //initialize PWM
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0x200;                         //TODO: Replace with some more intelligent value
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCNPolarity_High;

    TIM_OC2Init(ADC_TIM, &TIM_OCInitStruct);
}


void AngleSensor::hw_adc_init(void){
      // ADC channel 4 is on PA4 pin, channel 1 is on PA1
      // the GPIO port clock is already enabled
      GPIO_InitTypeDef GPIO_InitStructure;
      GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ADC_1;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
      GPIO_Init(GPIO_PORT_ADC, &GPIO_InitStructure);

      GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ADC_2;
      GPIO_Init(GPIO_PORT_ADC, &GPIO_InitStructure);

      /*
      //configure NVIC to handle ADC interrupts
      NVIC_InitTypeDef NVIC_InitStructure;

      NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
    */
      //enable ADC clock
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

      /* ADC1 configuration ---------------------------------*/
      ADC_InitTypeDef  ADC_InitStructure;
      ADC_StructInit(&ADC_InitStructure);
      ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
      ADC_InitStructure.ADC_ScanConvMode = ENABLE;
      ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
      ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
      ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
      ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
      ADC_InitStructure.ADC_NbrOfConversion = NUMBER_OF_ADC_CHANNELS;
      ADC_Init(ADC1, &ADC_InitStructure);
      
      ADC_CommonInitTypeDef ADC_CommonInitStructure;
      ADC_CommonStructInit(&ADC_CommonInitStructure);
      ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
      ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
      ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;

      /* ADC1 regular channels configuration */ 
      ADC_CommonInit(&ADC_CommonInitStructure);
      ADC_RegularChannelConfig(ADC1, ADC_Channel_SIN, 1, ADC_SampleTime_112Cycles);
      ADC_RegularChannelConfig(ADC1, ADC_Channel_COS, 2, ADC_SampleTime_112Cycles);

      /* Configure and enable ADC interrupts */
      //ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
      ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
}


void AngleSensor::hw_adc_dma_init()
{
    //enable DMA2 clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_Cmd(ADC_DMA_STREAM, DISABLE);   //make sure the stream is disabled before configuring

    //create DMA structure
    DMA_InitTypeDef  DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_DeInit(ADC_DMA_STREAM);
    
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(m_dma_buffer);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  
    DMA_InitStructure.DMA_BufferSize = NUMBER_OF_ADC_CHANNELS;  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    //send values to DMA registers
    DMA_Init(ADC_DMA_STREAM, &DMA_InitStructure);

    // Enable DMA1 Channel Transfer Complete interrupt
    DMA_ITConfig(ADC_DMA_STREAM, DMA_IT_TC, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    //Enable DMA2 channel IRQ Channel */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  

}

void AngleSensor::start_sensor(FunctionalState p_enable)
{
    INT8U err;

  //do nothing if old state is same as new state
  if(!m_enabled && p_enable){

    m_enabled = p_enable;
    //make sure the flags are cleared
    OSFlagPost(m_adc_flags, ADC_FLAG_REGULAR, OS_FLAG_CLR, &err);
    
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);              //Enable ADC
    DMA_Cmd(ADC_DMA_STREAM, ENABLE);    //Enable the DMA channel
    TIM_Cmd(ADC_TIM, ENABLE);           //start the trigger timer
  }
  
  if(m_enabled && !p_enable){
    m_enabled = p_enable;
    TIM_Cmd(ADC_TIM, DISABLE);           //stop the trigger timer
    DMA_Cmd(ADC_DMA_STREAM, DISABLE);
    //wait till it is actually switches off
    while(DMA_GetCmdStatus(ADC_DMA_STREAM)){};
    ADC_Cmd(ADC1, DISABLE);
  }
}


#define SIN_IDX         0
#define COS_IDX         1

float AngleSensor::calculate_angle(float *adc_value)
{

  // normalize raw ADC readings into -1:1 range 
  float norm_value[NUMBER_OF_ADC_CHANNELS];
  for(uint8_t i = 0; i < NUMBER_OF_ADC_CHANNELS; i++)
    norm_value[i] = 2.0f * ((float)(adc_value[i] - m_adc_min_value[i]))/(float)(m_adc_max_value[i] - m_adc_min_value[i]) - 1;
  
  // if abs(cos)>abs(sin) metric = sin/cos, func_type = 0 (tangent)
  int8_t func_type = 0;
  float metric = 0;
  if(abs(norm_value[COS_IDX]) > abs(norm_value[SIN_IDX]) ){
    metric = norm_value[SIN_IDX]/norm_value[COS_IDX];
    func_type = 0;
  }
  // else metric = cos/sin, func_type = 1 (cotangent)
  else{
    metric = norm_value[COS_IDX]/norm_value[SIN_IDX];
    func_type = 1;
  }
  float metric_sign = 1;
  if(metric < 0) metric_sign = -1;
      
  // angle = lookup(abs(metric))
  float angle = lookup_value(abs(metric))/2.f;
  
  // if func_type = 0 set angle sign same as metric
  if(func_type == 0)
    angle *= metric_sign;
  // else  opposite to the metric
  else
    angle *= metric_sign * (-1.f);
  
  // if (new_func_type - prev_func_type)*sign(angle) > 0
  if( (func_type - m_current_func_type)*metric > 0)
  {
    m_current_interval += 45.;  //    add pi/4 to the interval center
  }
  // if <0
  else if( (func_type - m_current_func_type)*metric < 0)
  {
    m_current_interval -= 45.;  //    subtract pi/4 from the interval center
  }
  m_current_func_type = func_type;
  // result = interval center + angle
  m_current_angle = m_current_interval + angle + static_correction;

  return m_current_angle;
}

//metric is a number between 0. and 1. inclusive
float AngleSensor::lookup_value(float metric)
{
  if(metric < 0. || metric > 1.)
    return 0.f;
  
  float grid_step = 10.0f;
  uint8_t l_idx = (uint8_t)floor(metric*grid_step);
  return (m_arctg[l_idx + 1] - m_arctg[l_idx])*grid_step * (metric - ((float)l_idx)/grid_step) + m_arctg[l_idx];
  
}

// this method blocks till the next reading is available and returns the new value
float AngleSensor::read_angle()
{
  INT8U err;
  OS_CPU_SR  cpu_sr;
  float adc_value[NUMBER_OF_ADC_CHANNELS];
  
  //wait for the next set of data available
  OSFlagPend(m_adc_flags, ADC_FLAG_REGULAR, OS_FLAG_WAIT_SET_ANY, 0, &err);
  OSFlagPost(m_adc_flags, ADC_FLAG_REGULAR, OS_FLAG_CLR, &err);

  OS_ENTER_CRITICAL(); 
  //grab live ADC values into the local variable ASAP to avoid data corruption by the next DMA transfer
  adc_value[0] = m_adc_value[0];
  adc_value[1] = m_adc_value[1];
  OS_EXIT_CRITICAL(); 
  
  return calculate_angle(adc_value);
}

// non-blocking, returns the most recently read value
float AngleSensor::current_angle()
{
  return m_current_angle;
}


