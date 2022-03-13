# Phuc Tran

## January 2022

### Motor Potentiometer Controller

### Description

The program takes a potentiometer's ADC value and convert that into a PWM signal for a motor. Then, we take data in a light sensor to calculate the RPM and display that onto a SevenSeg

### Evaluation

The motor works well with my potentiometer! There was a gradually ramp in RPM as I was changing my potentiometer's value. I was able to incorporate an ADC communication with my potentiometer input port, an i2C communication bus for a SevenSeg display, and an interrupt system with Timer1 PWM and Timer0 counter. I also created a device driver for the tb6612 motor driver that incorporated initializes the pin/port set ups, change the motor mode, and update the motor speed. I was able to take many embedded programming concepts and apply them into a simple action!

### Execution

1. Complete the Arduino/Breadboard setup according to motor_potentiometer.c header comments

2. Power the Arduino to the computer as well as a external power source to the board (important so the motor doesn't draw current from the computer!)

3. Make flash the file onto the Arduino
