
#ifndef __UART_H
#define __UART_H

#define USART_BUFFER_LENGTH  256

#define BAUD_RATE               57600
#define COMM                    USART2

// USART GPIO
#define GPIO_TX_AF_SOURCE       GPIO_PinSource2
#define GPIO_RX_AF_SOURCE       GPIO_PinSource3
#define GPIO_USART_AF           GPIO_AF_USART2
#define GPIO_PIN_TX             GPIO_Pin_2
#define GPIO_PIN_RX             GPIO_Pin_3
#define GPIO_PORT_USART         GPIOA

typedef struct {
    uint16_t    buffer_pointer;
    char* buffer;
    OS_FLAG_GRP* usart_flags;
    char        eol_char;
} USART_ReceiveBufferType;

#define USART_EOL_RECEIVED_FLAG         1
#define USART_BUFFER_OVERRUN_FLAG       2

typedef struct {
    uint16_t    char_num;
    uint16_t    transmit_ptr;
    char* buffer;
    OS_EVENT* trans_semaphore;
    uint8_t     is_transmitted;
} USART_TransmitBufferType;


void hw_uart_init(void);
void PrintByte(char c);
void USART_StartTransmission();
void USART_Print(char *format, ...);


#endif /* __UART_H */