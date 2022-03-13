/* SevenSeg.c
 * Partial device driver for Adafruit seven segment display (HT16K33) 
 * E.W. Hansen, Engs 28, 21W, based on P. Bonfert-Taylor's code from 19W
 */
 
/* INCLUDE FILES */
#include <avr/io.h>					// All the port definitions are here
#include <util/delay.h>
#include <i2c.h>					// Williams' code
#include <SevenSeg.h>				// The new library

const uint8_t i2c_addr_7seg = HT16K33_ADDR;

// Initialize the display
void SevenSeg_init(void) {		
	i2cStart();									// Ping the backpack with address 	
	i2cSend((i2c_addr_7seg << 1));				// SLA+W; shift address left for R/W bit! 
	i2cSend(HT16K33_SYSTEM_CMD | HT16K33_OSC_ON); // Enable oscillator

	// Turn the display on
	i2cStart();									// Repeated start						
	i2cSend((i2c_addr_7seg << 1));			 	// SLA+W; shift address left for R/W bit!
	i2cSend(HT16K33_DISPLAY_CMD | HT16K33_DISPLAY_ON | HT16K33_BLINK_OFF);
  
  	// Set the display brightness
	i2cStart();									// Repeated start
	i2cSend((i2c_addr_7seg << 1));				// SLA+W; shift address left for R/W bit!  
	i2cSend(HT16K33_BRIGHT_CMD | 7 );			// PWM duty cycle, 0-15  
	i2cStop();									// Now we stop
}

// Blink the display
void SevenSeg_blink(uint8_t rate) {		

	// start i2c communication and start writing a byte
	i2cStart();
	i2cSend(i2c_addr_7seg << 1);

	// turn on the display command, display's screen, and the display's blinking rate
	i2cSend(HT16K33_DISPLAY_CMD | HT16K33_DISPLAY_ON | rate);

	// stop i2c communication
	i2cStop();
}

// Dim the display	
void SevenSeg_dim(uint8_t brightness) {

	// start i2c communication and start writing a byte
	i2cStart();
	i2cSend(i2c_addr_7seg << 1);

	// turn on the display command, and the brightness
	i2cSend(HT16K33_BRIGHT_CMD | brightness);

	// stop i2c communication
	i2cStop();
}				

// Write the entire display memory
// display_buffer is assumed to be of size HT16K33_NBUF 
void SevenSeg_write(uint16_t *display_buffer) {
  i2cStart();	
  i2cSend((i2c_addr_7seg << 1));		// SLA+W; note shift address left for R/W bit!
  i2cSend(0x00);						// start at address 0x00

  for (uint8_t i=0; i<HT16K33_NBUF; i++) {
	// iterate the display buffer array 
    i2cSend(display_buffer[i]);	  							// low byte  
    i2cSend(0);							 					// high byte (no data, but fills array correctly)   
  } 
	
  i2cStop();  
}

// Convert decimal value (0-9999) to segments for SevenSeg_write()
// This is a little higher than a purely device driver function.
// display_buffer is assumed to be of size HT16K33_NBUF 
void SevenSeg_number(uint16_t num, uint16_t *display_buffer) {
	if (num>9999) {
		for (uint8_t i = 0; i<5; i++) {
			if (i==2) {
				display_buffer[i] = 0x0000;				// skip the colon
			} else {
				display_buffer[i] = numbertable[0xE];	// E for error 
			}
		}
	} else {
		display_buffer[0] = numbertable[num/1000];		// 1000s place
		
		num %= 1000;
		display_buffer[1] = numbertable[num/100];		// 100s place
		
		display_buffer[2] = 0x0000;						// skip the colon
		
		num %= 100;
		display_buffer[3] = numbertable[num/10] ;		// 10s place
		
		num %= 10;
		display_buffer[4] = numbertable[num];			// 1s place
	}	

}				