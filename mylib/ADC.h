/* ADC.h
 * Device driver for for ADC communication
 * E.W. Hansen, Engs 28, 21W, based on P. Bonfert-Taylor's code from 19W
 * Phuc Tran, modified to have a modular ADC communication library
 */

#include <avr/io.h>

void ADC_setChannel(uint8_t channel);       // Set the ADC input (0-5)

void ADC_setReference(uint8_t Vref);        // Set the ADC voltage reference

void ADC_Init(void);                        // Initialize the ADC

uint16_t ADC_getValue(void);                // Initialize conversion, return result
