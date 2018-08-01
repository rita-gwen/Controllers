
#include "nucleoboard.h"
#include "bsp.h"
#include <stdarg.h>
#include <string.h>

//allocate initial USART buffer
USART_ReceiveBufferType usart_receive_buffer_struct;
USART_ReceiveBufferType * receive_buffer = &usart_receive_buffer_struct;
static char usart_buffer_array[USART_BUFFER_LENGTH];

USART_TransmitBufferType transmit_buffer_struct;
USART_TransmitBufferType* transmit_buffer = &transmit_buffer_struct;
static char transmit_buffer_array[USART_BUFFER_LENGTH];

//-----------------------------------------------------------------------
// Initialize USART peripheral
void hw_uart_init(void) {
USART_InitTypeDef USART_InitStructure;
   INT8U err;

  //initialize the buffer variables
  receive_buffer->buffer = usart_buffer_array;
  //receive_buffer->usart_flags = OSFlagCreate((OS_FLAGS)(0x00), &err);         //do it later after OS is initialized
  
  transmit_buffer->buffer = transmit_buffer_array;
  //transmit_buffer->trans_semaphore = OSSemCreate(1);
  transmit_buffer->transmit_ptr = 0;
  transmit_buffer->char_num = 0;
  transmit_buffer->is_transmitted = 1;
  
  // Configure USART GPIO pins
  GPIO_PinAFConfig(GPIO_PORT_USART, GPIO_TX_AF_SOURCE, GPIO_USART_AF);
  GPIO_PinAFConfig(GPIO_PORT_USART, GPIO_RX_AF_SOURCE, GPIO_USART_AF);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_TX;            // USART_TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIO_PORT_USART, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RX;            // USART_RX
  GPIO_Init(GPIO_PORT_USART, &GPIO_InitStructure);
  
  //Enable USART clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  /* USARTx configuration -----------------------------------*/
   /* USARTx configured as follows:
         - BaudRate = from BAUD_RATE  
         - Word Length = 8 Bits
         - Two Stop Bit
         - Odd parity
         - Hardware flow control disabled (RTS and CTS signals)
         - Receive and transmit enabled
   */
   USART_InitStructure.USART_BaudRate = BAUD_RATE;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
   USART_Init(COMM, &USART_InitStructure);
   
   //Configure USART receive interrupt
   NVIC_InitTypeDef NVIC_InitStructure;

   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
    
   USART_ITConfig(COMM, USART_IT_RXNE, ENABLE);
   //Enable USART
   USART_Cmd(COMM, ENABLE);
}

//-----------------------------------------------------------------------
void USART2IrqHandler(void){
    OS_CPU_SR  cpu_sr;
    INT8U err;

    OSIntEnter();
    OS_ENTER_CRITICAL();                                        /* Tell uC/OS-II that we are starting an ISR            */

    //receiving
    if(USART_GetITStatus(COMM, USART_IT_RXNE) == SET){
        //receive interrupt, a characted was received
        if(receive_buffer->buffer_pointer == USART_BUFFER_LENGTH-1)     //reset the buffer pointer if we ran out of memory
          receive_buffer->buffer_pointer = 0;                         //to prevent corrupting other variables
        uint16_t i = ++(receive_buffer->buffer_pointer);              //increment the buffer pointer
        receive_buffer->buffer[i-1] = (char)USART_ReceiveData(COMM);    //move received data into the buffer
        if(receive_buffer->buffer[i-1] == receive_buffer->eol_char) {   //set a flag if we received EOL
          OSFlagPost(receive_buffer->usart_flags, USART_EOL_RECEIVED_FLAG, OS_FLAG_SET, &err);
          receive_buffer->buffer[i] = '\0';                             //add null line-terminator
        }
        if(receive_buffer->buffer_pointer == USART_BUFFER_LENGTH-1)     //set a flag if we are out of buffer memory
        {
          receive_buffer->buffer[i] = '\0';                             //add null line-terminator
          OSFlagPost(receive_buffer->usart_flags, USART_BUFFER_OVERRUN_FLAG, OS_FLAG_SET, &err);
        }
        USART_ClearITPendingBit(COMM, USART_IT_RXNE);
    }
    //transmitting
    else if(USART_GetFlagStatus(COMM, USART_FLAG_TXE) == SET){  
        //write interrupt, ready to transmit next character
        if(!transmit_buffer->is_transmitted){
          if(transmit_buffer->transmit_ptr < transmit_buffer->char_num-1){
            uint16_t i = ++(transmit_buffer->transmit_ptr);
            USART_SendData(COMM, transmit_buffer->buffer[i]);               //write next character to USART_DR
          }
          else{
            transmit_buffer->is_transmitted = 1;
            USART_ITConfig(COMM, USART_IT_TXE, DISABLE);
            if(transmit_buffer->trans_semaphore->OSEventCnt == 0)         //unblock the semaphor only if it is currently blocking
              OSSemPost(transmit_buffer->trans_semaphore);            
          }
        }
        USART_ClearITPendingBit(COMM, USART_IT_TXE);
    }
      
    OS_EXIT_CRITICAL();

    OSIntExit();                                                /* Tell uC/OS-II that we are leaving the ISR            */
    
}



/**
  * @brief  Print a character on the HyperTerminal
  * @param  c: The character to be printed
  * @retval None
  */
void PrintByte(char c)
{
  USART_SendData(COMM, c);
  while (USART_GetFlagStatus(COMM, USART_FLAG_TXE) == RESET)
  {
  }
}

void USART_StartTransmission()
{
    INT8U err;

    transmit_buffer->is_transmitted = 0;
    transmit_buffer->transmit_ptr = 0;

    USART_ClearFlag(COMM, USART_FLAG_TXE);
//    USART_ITConfig(COMM, USART_IT_TC, ENABLE);
    
    //transmit the first byte in the buffer. ISR picks up the rest. 
    if(transmit_buffer->char_num > 0){
      USART_SendData(COMM, transmit_buffer->buffer[0]);
    }
    USART_ITConfig(COMM, USART_IT_TXE, ENABLE);
}

void USART_Print(char *format, ...)
{
    INT8U err;
    va_list args;
    va_start(args, format);
    OSSemPend(transmit_buffer->trans_semaphore, 0, &err);

    transmit_buffer->char_num = vsnprintf(transmit_buffer->buffer, USART_BUFFER_LENGTH, format, args);
    transmit_buffer->is_transmitted = 0;
    transmit_buffer->transmit_ptr = 0;

    // acquire the buffer semaphore before sending data. It is released by the interrupt handler.
    USART_StartTransmission();
    
    va_end(args);
}