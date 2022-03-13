/* Taking a potentiometer's ADC value and convert that into a PWM signal for a motor
 * Taking in a sensor to calculate the RPM and display that onto a SevenSeg
 * 
 * Name:			Phuc Tran
 * Date created:	17 Feb 2022
 *
 * I/O pins:  (what ports are used, and what are they wired to?)
 * 
 * Digital Inputs:
 * 1. Bit 1 of I/O Port B: PWMA
 * 2. Bit 2 of I/O Port B: AIN1
 * 3. Bit 3 of I/O Port B: AIN2
 * 4. Bit 4 of I/O Port B: STDBY
 * 5. Bit 7 of I/O Port D: Sensor Vout
 * 6. SDA/SCL: TB6612
 * Analog Inputs:
 * 1. Bit 0 of I/O Port A: Potentiometer
 * 2. Bit 1 of I/O Port A: Potentiometer
 * 
 * Target device:	ATmega328p / Arduino UNO
 * Dependencies:	i2c.c, SevenSeg.c, ADC.c, USARTE28.c, io28.c, interrupt.c
 */

/* DEFINE VARIABLES */
#define PWM_TIMER_MAX	  1250			// 1.6kHz with prescale 8
#define PWM_TIMER_MIN	  -1250			// 1.6kHz with prescale 8 	
#define V_MAX	  5000							// 10-bit, 5V scale		
#define V_MIN	  0									// 10-bit, 5V scale	
#define SPEED_UPDATE	  500				// 5 seconds
#define VCC	  					5000			
#define BITRES	  			1024			

/* INCLUDE FILES */
#include <avr/io.h>								// All the port definitions are here
#include <USARTE28.h>							// USART communication
#include <ioE28.h>								// registers
#include <tb6612.h>								// motor microcontroller
#include <avr/interrupt.h> 				// Interrupt library
#include <ADC.h>									// ADC library
#include <i2c.h>									// Williams' code
#include <SevenSeg.h>							// SevenSeg library
#include <math.h>									// Math library

/* GLOBAL VARIABLES */
volatile uint8_t PWMtimerFlag;
volatile uint8_t ADCtimerFlag;
volatile uint8_t pulseFlag;

/* FUNCTIONS */
void ports_init(void);
void PWMtimer1_init(void); 
void ADCtimer0_init(uint32_t timeout);
void Pulsetimer0_init(uint16_t timeout);
void initPinChangeInterrupt(void);
void check_negative(int32_t counter_new, uint16_t *display_buffer);

// Main program
int main(void) {

	// what we want to display on our seven seg display
	uint16_t display_buffer[HT16K33_NBUF];
  int16_t TB6612_PWMA = 0;
  int32_t counter_new = 0;

  int32_t incrementPulse = 0;

	// Counting in increments of 0.008s 
	// Counting up to 7500
	int32_t oneSecTimer = 0;
  int8_t kConstant = 3;
  int32_t RPM = 0;

	// boundaries of the dead zone
	int8_t leftbound = -100;
	int8_t rightbound = 100;

	int8_t oneSecInterval = 125;

  // calculated voltage and temperature
	int32_t voltage;		
  const int16_t counter_max = PWM_TIMER_MAX;
  const int16_t counter_min = PWM_TIMER_MIN;

  const int16_t volt_max = V_MAX;
  const int16_t volt_min = V_MIN;

	// Finding the range of both values
	int32_t counter_range = counter_max - counter_min;
	int32_t volt_range = volt_max;

  const int16_t vcc = VCC;
  const int16_t bitResolution = BITRES;

	// initializing sensor ports
	ports_init();
	// initialize motor ports from the microcontroller
	motor_init();

	// Start i2c communication
  i2cInit();
	// Start ADC communication
	ADC_Init(); 
	// initialize USART communication
	USART_Init();  
	SevenSeg_init();  

	// initialize PWM timer1 communication
	PWMtimer1_init(); 
	// initialize ADC timer2 communication
	ADCtimer0_init(125);
	// Enable pin change interrupts for sensor
	initPinChangeInterrupt();
	// Global interrupt enable
	sei();

  printf("Converting potentiometer to motor's value\n\r");
  
  while(1) {

		// Occuring every pin change (high to low)
		if(pulseFlag){
			incrementPulse++;
			pulseFlag = 0;
		}

		// Occuring in 125Hz
		else if(ADCtimerFlag){

			uint32_t adc_value = ADC_getValue();

			// Calculating the step voltage from the ADC value and the temperature ouput from the calculated voltage
			voltage = ((int32_t) adc_value * vcc) / bitResolution;
			counter_new = ((int32_t)(voltage - volt_min) * counter_range / volt_range - counter_max);

			// Calculating RPM = 60 * PPS/20
			// Receive pulses for every 0.008 seconds, so we multiply those pulses 125 to get pulse every second
			oneSecTimer++;
			if (oneSecTimer == oneSecInterval){

				// Multiply PPS which is what we have by occuring this for 7500 times
				// Multiply60 for 60 seconds and dividing by 20
				RPM = kConstant * incrementPulse;

				if (counter_new >= 0){
					printf("RPM=%d, \t", RPM);
				} else{
					printf("RPM=%d, \t", -1 * RPM);
				}

				// i2C communication
				// we want to convert the display buffer's temp_output to an absolute value (use fabs() and then cast back to int32_t)
				// and then handle the negative sign later by arranging the buffer
				// lastly, we are writing either a C or F depending on the temp type
				SevenSeg_number((int32_t)(fabs(RPM)),display_buffer);
				check_negative(counter_new, display_buffer);
				SevenSeg_write(display_buffer);
				// Resetting incrementPulse and oneSecTimer
				incrementPulse = 0;	
				oneSecTimer = 0;		
			}
			ADCtimerFlag = 0;
		}	

		else{
			// Dictionary: 1 - FWD, 2 - REV, 3 - BRAKE, 4 - STOP, 5 - STDBY
			// If within this range, don't move the wheel
			if (counter_new > leftbound && counter_new < rightbound){
				counter_new = 0;
				motor_mode(3);
			}
				// set direction
			else if(counter_new >= 0) {
				motor_mode(1);
			} 
			else{
				motor_mode(2);
			}

			// Now set the magnitude
			TB6612_PWMA = (int32_t)(fabs(counter_new));		// magnitude (speed)
			motor_speed(TB6612_PWMA);						// Update the PWM
		}
	
  }
  return 0;                            /* This line is never reached */
}

// at pin change
ISR(PCINT2_vect) {
  // Set flag to notify main
	if ((PIND & (1 << PIND7)) == 0){
		pulseFlag = 1;	
	}
}

// ADC timer flag vector
ISR(TIMER0_COMPA_vect){
	ADCtimerFlag = 1;
}

void ports_init(void){
	DDRD  &= ~(1 << DDD7);			// Configure pin as input, explicitly
	PORTD |= (1 << PORTD7);			// Turn pullup on
}

void initPinChangeInterrupt(void) {
	PCICR  |= (1 << PCIE2); 		// Enable pin-change interrupt for D-pins
	PCMSK2 |= (1 << PD7); 			// Set pin mask for bit 7 of Port D
}

// FUNCTION implementations
// Set up PWM
void PWMtimer1_init(void) {
  TCCR1A |= (1 << WGM11); 							// fast pwm, using ICR1 as TOP
  TCCR1B |= (1 << WGM12) | (1 << WGM13); 
  TCCR1B |= (1 << CS11); 								// /8 prescale --> 2MHz clock
  ICR1    = PWM_TIMER_MAX;  						// TOP --> 1.6kHz PWM frequency
  TCCR1A |= (1 << COM1A1); 							// clear on compare match, set at bottom
  OCR1A   = 0;    											// set it to stopped, initially 
  DDRB   |= (1 << DDB1); 								// set PB1/OC1A to output 
}

// Setting up Timer 0 configuration for CTC, 125Hz (0.008s) in 1024 prescaler
void ADCtimer0_init(uint32_t timeout){

	// turn on CTC
	TCCR0A |= (1 << WGM01);

	// Compare Match A Interrupt
	TIMSK0 |= (1 << OCIE0A);
	
	// 125 clock cycles at 16MHz
	OCR0A = timeout;

	// Prescale 1024
	TCCR0B |= (1 << CS02) | (1 << CS00);
}


void check_negative(int32_t counter_new, uint16_t *display_buffer){
	// If it's negative 
	if (counter_new < 0){
		int16_t negative = numbertable[10];
		// update display buffer
		display_buffer[0] = negative;
	} else{
		display_buffer[0] = 0;
	}
}
