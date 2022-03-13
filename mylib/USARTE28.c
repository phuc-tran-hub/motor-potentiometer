/* Simple USART library to make serial communication work.

   Note that receiveByte() blocks -- it sits and waits _forever_ for
   a byte to come in.  If you're doing anything that's more interesting,
   you'll want to implement this with interrupts.

   Adapted from Williams' USART library.
*/

#include <avr/io.h>
#include "USARTE28.h"

void USART_Init(void) {
  /*Set baud rate */
  UBRR0 = MYUBRR;
  /* Enable receiver and transmitter */ 
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
  /* Set frame format: 8data, 1stop bit */ 
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}

void transmitByte(uint8_t data) {
  /* Wait for empty transmit buffer */
  while ( (UCSR0A & (1 << UDRE0)) == 0 ) {} 
  UDR0 = data;    /* send data */                                        
}

/* Same as transmitByte except for the data type of the argument. 
   Needed for tinpy_printf(). */
void tfp_putchar(char c) {
	transmitByte((uint8_t) c);
}

uint8_t receiveByte(void) {
  /* Wait for incoming data */
  while ( (UCSR0A & (1 << RXC0)) == 0) {} 
  return UDR0;   /* return register value */
}
