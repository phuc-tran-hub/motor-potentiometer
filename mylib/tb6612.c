// Functions for tb6612 communication

/* INCLUDE FILES */
#include <avr/io.h>				// All the port definitions are here
#include <tb6612.h>

#define PWM_TIMER_MAX	  1250		// 1.6kHz with prescale 8 		

// Set ports for PWM, IN1, IN2, STDBY
void motor_init(void){
    DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3);	// Outputs to PWM, IN1, and IN2
    DDRB |= (1 << DDB4);                                // Outputs to STDBY
    PORTB |= (1 << PORTB4);
}                          

// PWD, REV, BRAKE, STOP, STDBY
// Include a failsafe for PWD <-> REV?
void motor_mode(uint8_t mode){

    uint8_t TB6612_AIN1 = 0;
    uint8_t TB6612_AIN2 = 0;			// STOP mode

    switch(mode){
        // FWD: clockwise rotation
        case 1: 
           	TB6612_AIN1 = 1;
			TB6612_AIN2 = 0;
            break;

        // REV: counterclockwise rotation
        case 2: 
           	TB6612_AIN1 = 0;
			TB6612_AIN2 = 1;
            break;
        
        // BRAKE: a slow break while letting the motor spin
        case 3: 
           	TB6612_AIN1 = 1;
			TB6612_AIN2 = 1;
            break;   

        // STOP: a hard stop that disallows the motor from spinning
        case 4:
           	TB6612_AIN1 = 0;
			TB6612_AIN2 = 0;
            break;   

        // STDBY: stops the whole system as an emergency
        case 5:
            // Clear the HIGH bit to LOW
            DDRB &= ~(1 << DDB4);                     
            break; 
    }

    // Clearing AIN1 and AIN2
    PORTB = (PORTB & ~((1 << PB2) | (1 << PB3))) 
                | (TB6612_AIN1 << PB3) | (TB6612_AIN2 << PB2); 
}                 
                                              
 // Interacts with Timer1
void motor_speed(uint16_t pwm_value){
    // Safety first: Allow no pulse values outside the (0, PWM_TIMER_MAX-1 range!
	if (pwm_value>PWM_TIMER_MAX-1) 
		pwm_value = PWM_TIMER_MAX-1;
	else if (pwm_value < 0)
		pwm_value = 0;		
	OCR1A = pwm_value;
}