#ifndef __ADC_H
#define __ADC_H


//ADC
#define GPIO_PORT_ADC             GPIOA
//magnetic sensor ADC connections for sin() and cos() signals
#define GPIO_PIN_ADC_1            GPIO_Pin_4      // sin() signal (A2 pin on the Arduino connector)
#define ADC_Channel_SIN           ADC_Channel_4
#define GPIO_PIN_ADC_2            GPIO_Pin_1      // cos() signal (A1 pin on the Arduino connector)
#define ADC_Channel_COS           ADC_Channel_1

#define NUMBER_OF_ADC_CHANNELS        2
#define ADC_DMA_STREAM          DMA2_Stream0

//Flags to identify what type of conversion happened
#define ADC_FLAG_REGULAR        1
#define ADC_FLAG_ERROR          4

#define ADC_CONVERSION_FREQUENCY        1000             // in Hz
#define ADC_TIM                         TIM2
#define ADC_GPIO_MONITOR_PIN            GPIO_Pin_9      //output triggering PWM signal to this pin for debugging (TIM2_CH2). Pin D8 in Arduino connector
#define ADC_GPIO_MONITOR_PIN_SRC        GPIO_PinSource9

#define ADC_MAX 0xFFF

#ifdef __cplusplus
  extern "C" {
#endif 
   
    void ADCIrqHandler(void);
    void ADCDMAIrqHandler(void);
    
#ifdef __cplusplus
  }
#endif     


// The class that incapsulates all the ADC angle reading functionality including
// hardware initialization and data conversion
class AngleSensor{

  //class private members
  
  uint16_t m_dma_buffer[NUMBER_OF_ADC_CHANNELS];       //memory where DMA loads ADC data

  uint16_t m_adc_value[NUMBER_OF_ADC_CHANNELS];       //working area
  
  uint32_t m_adc_timestamp;       //timestamp of the last ADC conversion (interrupt call)
  OS_FLAG_GRP* m_adc_flags;       //read sinchronisation flags
  uint16_t m_conversion_freq;
  FunctionalState m_enabled;
  float m_current_angle;          //most recentrly read angle in degrees
  float m_current_interval;       //center point of the current approximmation interval
  int8_t   m_current_func_type;
  float static_correction;        // small correction factor to add to the angle for the off-vertical arm and sensor deviations
  
  // Static table of arctangent values for inverse lookup and interpolation. 
  // Values are given on the regular-spaced grid from 0 to 1 with 0.1 step
  const static float m_arctg[11];
  
  //sensor calibration data
  uint16_t *m_adc_max_value;
  uint16_t *m_adc_min_value;
  
  //hardware initialisation methods
  void hw_adc_timer_init(void); //ADC timebase trigger init
  void hw_adc_init(void);       // Actual ADC init
  void hw_adc_dma_init();       // Init DMA to read ADC data
  
  float calculate_angle(float *adc_value);       //calculates the angle from the most recent measurement
  float lookup_value(float metric);     //lookup and interpolation of the arctan value using the tabulated values.

public:
  friend void ADCIrqHandler(void);
  friend void ADCDMAIrqHandler(void);
  
  void init_sensor(uint16_t conversionFrequency);
  
  // start/stop the whole pipeline
  void start_sensor(FunctionalState newState);      
  //starts/stops the GPIO tracking output (for debugging)
  void enable_heartbeat(FunctionalState newState);
  //this method blocks till the next reading is available and returns the new value
  float read_angle();   
  //non-blocking, returns the most recently read value
  float current_angle();
};


#endif /* __ADC_H */