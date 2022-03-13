/* Functions to initialize, send, receive over USART */

/* Clock Speed */
#define FOSC 16000000  

/* if not defined elsewhere set a safe default baud rate */
#ifndef BAUD                         
#define BAUD  9600                     
#endif

/* Calculate bit-clock multiplier for the ATMega328p */
#define MYUBRR (FOSC/16/BAUD-1)  

/* Configures the hardware USART */
void USART_Init(void);

/* Blocking transmit and receive functions.
   When you call receiveByte() your program will hang until
   data comes through.  We'll improve on this later. */
void transmitByte(uint8_t data);
uint8_t receiveByte(void);

/* needed for tiny_printf() - data type char needed in argument */
void tfp_putchar(char);

