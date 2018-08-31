/************************************************************************************

Module Name:

    tasks.c

Module Description:

    Exercises several uCOS services

History:

    2015/1 Nick Strathy wrote it
    2016/2 Ported to Cortex-M4/IAR

************************************************************************************/

#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "bsp.h"
#include "print.h"
#include "arm_math.h"

#define BUFSIZE 150
#define GUI_UPDATE_FREQ         50       //relative to the reporting loop frequency. 2 means every other iteration


/************************************************************************************

   Allocate the stacks for each task.
   The maximum number of tasks the application can have is defined by OS_MAX_TASKS in os_cfg.h

************************************************************************************/

static OS_STK   TaskControllerStk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK   TaskCommStk[APP_CFG_TASK_START_STK_SIZE];
static OS_STK   TaskSpeedDataStk[APP_CFG_TASK_START_STK_SIZE];


// Task prototypes
void TaskController(void* pdata);
void TaskComm(void* pdata);
void TaskSpeedData(void* data);

// Useful functions
void printWithBuf(char *buf, int size, char *format, ...);
void printAndDie(INT8U err, char *msg);
uint16_t atoi(char* str);
void getProcessData();
void printProcessData();
void packProcessData();
void unpackData(void);
uint16_t controller(void);

OS_EVENT * semPrint;

extern AngleSensor *angle_sensor;
extern MSREAD_ResultType *motor_data;
//USART buffers
extern USART_ReceiveBufferType * receive_buffer;
extern USART_TransmitBufferType* transmit_buffer;

//allocate memory for the process and controller data
extern ProcessDataStructType proc_data;
extern ControllerDataStructType cntr_data;

#ifdef LAB1
char* config_message = "&A~Desired~5&C&S~K_P~P~0~10~0.05&S~Direct~O~0~260~0.01&S~Desired~A~0~260~1&T~MotorSpeed~U1~0~260&T~Error~S1~-150~150&T~MotorCmd~U1~0~260&H~2&";
#endif

#ifdef LAB2
 char* config_message = "&A~Desired~5&C&S~K_P~P~0~10~0.05&S~Direct~O~0~260~0.01&S~Desired~A~-130~130~1&T~ArmAngle~S1~-130~130&T~Error~S1~-150~150&T~MotorCmd~U1~0~260&H~2&";
#endif

#ifdef LAB3
char* config_message = "&A~Desired~5&C&S~K_P~P~0~20~0.05&S~K_D~D~0~10~0.05&S~Direct~O~0~260~0.01&S~Desired~A~-130~130~1&T~ArmAngle~S1~-130~130&T~Error~S1~-150~150&T~Delta~S2~-1000~1000&T~MotorCmd~U1~0~260&H~2&";
#endif

#ifdef LAB4
char* config_message = "&A~Desired~5&C&S~K_P~P~0~20~0.05&S~K_D~D~0~10~0.05&S~K_I~I~0~20~0.05&S~SumMax~S~0~5000~1&S~Direct~O~0~260~0.01&S~Desired~A~-130~130~1&T~ArmAngle~S1~-130~130&T~Error~S1~-150~150&T~Delta~S2~-1000~1000&T~Sum~S2~-1000~1000&T~MotorCmd~U1~0~260&H~2&";
#endif




/************************************************************************************

   This task is the initial task running, started by main(). It starts
   the system tick timer and creates all the other tasks. Then it deletes itself.

************************************************************************************/
void StartupTask(void* pdata)
{
    INT8U err;
    // TODO semPrint 01: add code here to create semaphore semPrint as a binary semaphore
    semPrint = OSSemCreate(1);          //moving semaphore creation up to enable printWithBuf to work
    //initialize interrupt events
    receive_buffer->usart_flags = OSFlagCreate((OS_FLAGS)(0x00), &err);
    receive_buffer->eol_char = '\n';
    transmit_buffer->trans_semaphore = OSSemCreate(1);
    motor_data->ic_flags = OSFlagCreate((OS_FLAGS)(0x00), &err);
    motor_data->ic_sum =0;
    motor_data->ic_count = 0; 

    initController();

    //char buf[BUFSIZE];
    //printWithBuf(buf, BUFSIZE, "StartupTask: Begin\n");
    //printWithBuf(buf, BUFSIZE, "StartupTask: Starting timer tick\n");

    // Start the system tick
    SysTick_Config(CLOCK_HSI / OS_TICKS_PER_SEC);
    angle_sensor->init_sensor(ADC_CONVERSION_FREQUENCY);
    angle_sensor->start_sensor(ENABLE);     //start the ADC data flow

    //printWithBuf(buf, BUFSIZE, "StartupTask: Creating application tasks\n");
    

    // The maximum of tasks the application can have is defined by OS_MAX_TASKS in os_cfg.h
    INT8U pri = APP_TASK_START_PRIO + 1;
    OSTaskCreate(TaskController, (void*)0, &TaskControllerStk[APP_CFG_TASK_START_STK_SIZE-1], pri++);
    OSTaskCreate(TaskComm, (void*)0, &TaskCommStk[APP_CFG_TASK_START_STK_SIZE-1], pri++);
    OSTaskCreate(TaskSpeedData, (void*)0, &TaskSpeedDataStk[APP_CFG_TASK_START_STK_SIZE-1], pri++);
    

    // Delete ourselves, letting the work be done in the new tasks.
    //printWithBuf(buf, BUFSIZE, "StartupTask: deleting self\n");
    OSTaskDel(OS_PRIO_SELF);
}

/************************************************************************************
Controller loop task: implements the controller logic and sends updates to the host.
************************************************************************************/

void TaskController(void* pdata)
{
    INT8U err;
    uint8_t     loop_count = 0;

    //USART_Print("Controller: starting\n");

    while(1)
    {

      //The loop is synchronized by the pot ADC event. It updates the motor command and other data
      //transmits the data to the host (maybe every other iteration or less and waits for the next ADC event.
        if(cntr_data.first_time)
          USART_Print("%s\n\r", config_message);        //TODO: Add start and end markers [] so that the server can verify message integrity.
      
        angle_sensor->read_angle();
        //SetPin(TEST_PIN1, 1);
        
        if(!cntr_data.first_time)
        {
          cntr_getProcessData();
          //printProcessData();
          packProcessData();
          if(++loop_count == GUI_UPDATE_FREQ)
          {
            OSSemPend(transmit_buffer->trans_semaphore, 0, &err);
            USART_StartTransmission();
            loop_count = 0;
          }
          
        }
    }
    
}

/***************************************************
 Communication task. Data is loaded into UART buffer by UART ISR and a flag is set on 
receipt of the EOL. The thread then wakes up, parses up the data and updates the controller parameters (K and direct) in a critical section
to avoid incorrect reads by the controller.
****************************************************/
void TaskComm(void* data){

    INT8U err;
    char* gui_init_mark = "SET\n";
  
    while(1)
    {
      //initialize GUI
      while(cntr_data.first_time)
      {
        receive_buffer->buffer_pointer = 0;
        OSFlagPend(receive_buffer->usart_flags, USART_EOL_RECEIVED_FLAG, OS_FLAG_WAIT_SET_ALL, 0, &err );
        OSFlagPost(receive_buffer->usart_flags, USART_EOL_RECEIVED_FLAG, OS_FLAG_CLR, &err);
        if(!strcmp(receive_buffer->buffer, gui_init_mark))
        {
          cntr_data.first_time = 0;
        }
      }
    
      while(!cntr_data.first_time)
      {
          receive_buffer->buffer_pointer = 0;
          OSFlagPend(receive_buffer->usart_flags, USART_EOL_RECEIVED_FLAG, OS_FLAG_WAIT_SET_ALL, 0, &err );
          OSFlagPost(receive_buffer->usart_flags, USART_EOL_RECEIVED_FLAG, OS_FLAG_CLR, &err);
          unpackData();
      }
    }
}


/************************************************************************************/
void TaskSpeedData(void* data)
{
    INT8U err;

    while(1)
    {
        OSFlagPend(motor_data->ic_flags, MSREAD_FLAG_REGULAR, OS_FLAG_WAIT_SET_ALL, 0, &err );
        OSFlagPost(motor_data->ic_flags, MSREAD_FLAG_REGULAR, OS_FLAG_CLR, &err);
        if(motor_data->ic_value > 16)   //ignore too short counts
        {
          motor_data->ic_sum += motor_data->ic_value;
          motor_data->ic_count++;
        }
    }
}

/************************************************************************************

   Print a formated string with the given buffer.
   Each task should use its own buffer to prevent data corruption.

************************************************************************************/
void printWithBuf(char *buf, int size, char *format, ...)
{
	INT8U err;
    va_list args;
    va_start(args, format);
    vsnprintf(buf, size, format, args);

    // TODO semPrint 02: place the call to PrintString in a critical section using semaphore semPrint
    OSSemPend(semPrint, 0, &err);
    //printAndDie(err, "Cannot wait on the semaphore semPrint.\n");

    PrintString(buf);
    
    OSSemPost(semPrint);

    va_end(args);
}

//extacts data from the receive buffer
void unpackData(void)
{
  if(receive_buffer->buffer_pointer > 0)
  {
    char* ptr = receive_buffer->buffer;
    float val = atofloat(ptr+2);
    switch(*ptr)
    {
    case 'P':           /*      K_P - proportional feedback contribution        */
      cntr_data.feedback_rate = val;
      break;
    case 'O':
      cntr_data.direct = val;
      break;
    case 'A':
      cntr_data.set_point = val;
      break;
    case 'D':           /*       K_D - derivative contribution      */
      cntr_data.deriv_rate = val;
      break;
    case 'I':           /*       K_I - integral contribution      */
      cntr_data.integral_rate = val;
      break;
    case 'S':           /*       SumMax       */
      cntr_data.sum_max = val;
      break;
    case '~':
      cntr_data.first_time = 1;
      break;
    default:
      break;
    }
  }
  
}

// packs data from the process data struct into the transmit buffer
void packProcessData()
{
  
  char* ptr = transmit_buffer->buffer;
  *ptr = (char)0;               //start byte
#ifdef  LAB1            // packing sequence should be redefined for each lab according to the config string signature
  ptr++;
  *ptr = floatToUByte(proc_data.motor_speed);
  ptr++;
  *ptr = floatToByte(proc_data.error_signal);
  ptr++;
  *ptr = floatToUByte(proc_data.motor_cmd);
  ptr++;
#endif

#ifdef  LAB3
  ptr++;
  *ptr = floatToByte(proc_data.pot_angle);
  ptr++;
  *ptr = floatToByte(proc_data.error_signal);

  uint16_t deriv_ptr = floatToWord(proc_data.error_derivative);
  ptr++;
  *ptr = *((uint8_t*)&deriv_ptr);
  ptr++;
  *ptr = *(((uint8_t*)&deriv_ptr) + 1);
  
  ptr++;
  *ptr = floatToUByte(proc_data.motor_cmd);
#endif

  uint16_t* hdr_ptr = &(proc_data.headroom);
  ptr++;
  *ptr = *((uint8_t*)hdr_ptr);
  ptr++;
  *ptr = *((uint8_t*)hdr_ptr+1);
  ptr++;
  *ptr = (uint8_t)0xFF;         //end byte
  
  transmit_buffer->char_num = ptr - transmit_buffer->buffer + 1;
}


void printProcessData(){
  USART_Print("Process Data: \n\r\tmotor_speed: %d \n\r\terror_signal: %d \n\r\t motor_cmd:: %d\n\r\theadroom:%d\n\r", proc_data.motor_speed, proc_data.error_signal, proc_data.motor_cmd, proc_data.headroom);
}