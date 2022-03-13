// Functions for ADC communication

#include <avr/io.h>		          // All the port definitions are here
#include <util/delay.h>         // So that you can use _delay...
#include <USARTE28.h>           // UART initializations
#include <ioE28.h>              // Basic read/write and tiny printf
#include <avr/interrupt.h> 

// Set the ADC input (0-5)
void ADC_setChannel(uint8_t channel){

  // clearing all the input bits
  ADMUX &= ~(1 << MUX0) & ~(1 << MUX1) & ~(1 << MUX2) & ~(1 << MUX3);

  // Setting the new input
  ADMUX |= channel;

}      

// Set the ADC voltage reference
void ADC_setReference(uint8_t Vref){

  // clearing all the REFS bit
  ADMUX &= ~(1 << REFS0) & ~(1 << REFS1);

  // Turn on REFS0 according to the Vref
	if (Vref == 5){
		ADMUX  |= (1 << REFS0);                      
	}
	else if (Vref == 1){
		ADMUX  |= (1 << REFS1) | (1 << REFS0);        
	}

}
 // Initialize the ADC
void ADC_Init(void){

  // Default: Vcc to 5V
  ADC_setReference(5);

  // Default: Set the input channel to 0
  ADC_setChannel(0);
  
  // this is setting ADLAR bit to 0 for 10-bit accuracy
  ADMUX |= (0 << ADLAR);

  // second, setting to a lower clockspeed 
  ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2); 

  // Prescale by 128
  ADCSRA |= (1 << ADEN);    	
}                     

// Initialize conversion, return result
uint16_t ADC_getValue(void){                                    
    // Initiate a conversion and return the result
    ADCSRA |= (1 << ADSC);       // Start conversion
    while ((ADCSRA & (1 << ADSC)) != 0){}  
    // Wait for completion
    uint16_t value = ADC;       // Read the result
    return value;
}          