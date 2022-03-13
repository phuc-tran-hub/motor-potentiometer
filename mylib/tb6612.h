/* tb6612.h
 * A tb6612 device driver to initialize communication, select mode, and to select speed
 * Phuc Tran, modified to have a modular ADC communication library
 */

#include <avr/io.h>

void motor_init(void);                          // Set ports for PWM, IN1, IN2, STDBY

void motor_mode(uint8_t mode);                 // PWD, REV, BRAKE, STOP, STDBY
                                               // Include a failsafe for PWD <-> REV?

void motor_speed(uint16_t pwm_value);          // Interacts with Timer1
