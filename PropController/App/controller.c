
#include "nucleoboard.h"
#include "bsp.h"
#include "arm_math.h"

ProcessDataStructType proc_data;
ControllerDataStructType cntr_data;
static float error_buffer_array[ERROR_BUFF_SIZE];

extern ADC_ResultType *pot_data;
extern MSREAD_ResultType *motor_data;

void initController(void)
{
    //initialize controller data
    cntr_data.direct = 0.0f;
    cntr_data.feedback_rate = 0.0f;
    cntr_data.set_point = 0.0f;
    cntr_data.first_time = 1;
    
    //initialize error buffer
    proc_data.error_history = error_buffer_array;
    //make sure the array is initialized to zeros to avoid startup artefacts
    for(uint8_t i = 0; i < ERROR_BUFF_SIZE; i++) error_buffer_array[i] = 0;     
    proc_data.errhist_index = ERROR_BUFF_SIZE;

    proc_data.error_sum = 0.0f;
  
}

//adds new value to the buffer
void cntr_errhist_push(float err_value)
{
  if(proc_data.errhist_index == ERROR_BUFF_SIZE)
    proc_data.errhist_index = 0;
  else
    proc_data.errhist_index++;
  proc_data.error_history[proc_data.errhist_index] = err_value;
}

// Retrieves a value from the buffer at the given offset from the current value
// If offset parameter is larger than ERROR_BUFF_SIZE retorns the earliest 
// value (the most remote from the current one)
float cntr_errhist_get(uint8_t offset)
{
  uint8_t idx;
  if(offset >= ERROR_BUFF_SIZE)
    offset = ERROR_BUFF_SIZE - 1;
  if(offset <= proc_data.errhist_index)
    idx = proc_data.errhist_index - offset;
  else if(offset < ERROR_BUFF_SIZE)
    idx = ERROR_BUFF_SIZE - (offset - proc_data.errhist_index);

  return proc_data.error_history[idx];
}

// collects and formats the process data and put it into the structure
void cntr_getProcessData()
{
  //   motor speed statistics calculation
  float32_t  mean = 0;
  proc_data.motor_speed = 0;
  if(motor_data->ic_count)
  {
    //6 spokes. Actual speed is up to 60 rps. Multiplying by 4 to scale it 
    //to the reporting interval and for better precision.
    mean = ((float)motor_data->ic_sum)/((float)motor_data->ic_count);
    proc_data.motor_speed = CLOCK_HSI/MSREAD_COUNT_PRESCALER/((float)mean*6)*4;
  }
  //reset for the new cycle  
  motor_data->ic_count = 0;
  motor_data->ic_sum = 0;
  
  proc_data.pot_angle = (int16_t)round(((float)pot_data->adc_value - POT_ZERO)/POT_READ_PER_DEGREE);

#ifdef LAB1
  proc_data.error_signal = cntr_data.set_point - proc_data.motor_speed;
#endif
  
#ifdef LAB3
  proc_data.error_signal = cntr_data.set_point - (float)proc_data.pot_angle;
#endif

  cntr_errhist_push( proc_data.error_signal );          //push the current error into the buffer
  
  proc_data.error_derivative = (cntr_errhist_get(0) - cntr_errhist_get(ERROR_DELAY))*ADC_CONVERSION_FREQUENCY;
  
  uint32_t period = CLOCK_HSI/ADC_CONVERSION_FREQUENCY;
  proc_data.headroom = (uint16_t)round(1000.0f - (float)TIM_GetCounter(ADC_TIM)/(float)period * 1000.0f);
  
  proc_data.motor_cmd = cntr_control();
}

// Function implementing the actual controller logic. Returns new motor command value based on the current 
// process and controller data
uint16_t cntr_control(void)
{
  float cmd = round(proc_data.error_signal * cntr_data.feedback_rate + proc_data.error_derivative * cntr_data.deriv_rate + cntr_data.direct);
  uint16_t pwm_cmd = 0;
  if((int16_t)cmd > MPWM_COUNT_PERIOD>>1)
    pwm_cmd = MPWM_COUNT_PERIOD>>1;
  else 
  {
    if(cmd > 0)
      pwm_cmd = (uint16_t)cmd;
    else 
      pwm_cmd = 0;
  }
  
  SetMotorCommand(pwm_cmd<<1);
  
  return pwm_cmd;
}
