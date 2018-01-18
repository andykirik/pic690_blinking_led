/*
 * File:   main_timer.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Using timers
 * 
 *  Board connection (PICKit 2 Low Count Demo):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC0 (DS1; J1->10)         LED
 * 
 */

#include <xc.h>

/* The __delay_ms() function is provided by XC8. 
It requires you define _XTAL_FREQ as the frequency of your system clock. 
We are using the internal oscillator at its default 4MHz, so _XTAL_FREQ is defined as 4000000. 
The compiler then uses that value to calculate how many cycles are required to give the requested delay. 
There is also __delay_us() for microseconds and _delay() to delay for a specific number of clock cycles. 
Note that __delay_ms() and __delay_us() begin with a double underscore whereas _delay() 
begins with a single underscore.
*/
#define _XTAL_FREQ 8000000

// CONFIG
// PIC16F690 Configuration Bit Settings
#pragma config FOSC 	= HS		// Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE 	= OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE 	= OFF      	// Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE 	= ON      	// MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP 		= OFF       // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD 		= OFF       // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN 	= ON       	// Brown-out Reset Selection bits (BOR enabled)
#pragma config IESO 	= ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN 	= ON       	// Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)


void system_init()
{
    OSCCON=0x70;          // Select 8 Mhz internal clock
    
	// I/O	
		// ANSELx registers
			ANSEL = 0x00;         // Set PORT ANS0 to ANS7 as Digital I/O
			ANSELH = 0x00;        // Set PORT ANS8 to ANS11 as Digital I/O
	  
		// TRISx registers (This register specifies the data direction of each pin)
			TRISA = 0x00;         // Set All on PORTB as Output
			TRISB = 0x00;         // Set All on PORTB as Output    
			TRISC = 0x00;         // Set All on PORTC as Output    
		
		// PORT registers (hold the current digital state of the digital I/O)
			PORTA = 0x00;         // Set PORTA all 0
			PORTB = 0x00;         // Set PORTB all 0
			PORTC = 0x00;         // Set PORTC all 0
        
	// Timer Setup - Timer 0
    /* A prescaler is a circuit that reduces the frequency of a clock using integer division. 
     *  The prescaler can be set anywhere from 1:2 to 1:256 for Timer 0.
     *  The clock we are slowing down is NOT the system clock Fosc (4MHz as in here). 
     *  It's the system's instruction clock Fcy, which is always Fosc/4.
     *  The timer expires when the TMR0 register rolls over. 
     *  The TMR0 register is an 8bit register, therefore it will roll over after 256 counts.
     *  Rollover Frequency = Fosc / (4 * prescaler * 256)
     *  In following case it would be 15.2588Hz or 0.0655 seconds per rollover.
    */
		OPTION_REGbits.PSA = 0; 	// Prescaler assigned to Timer 0
		OPTION_REGbits.PS = 0b111;  // Set the prescaler to 1:256
		OPTION_REGbits.T0CS = 0;    // Use the instruction clock (Fcy/4) as the timer clock. 
									//   Other option is an external oscillator or clock on the T0CKI pin.
}

void main(void) 
{
    system_init();
    
    while(1)
    {        
        while(INTCONbits.T0IF == 0); // Wait for the interrupt to occur. This happens when the TMR0 register rolls over.

        PORTCbits.RC0 = ~PORTCbits.RC0; // Toggle the LED
		
        INTCONbits.T0IF = 0;        // Clear the Timer 0 interrupt flag
        TMR0 = 0;                   // Load a value of 0 into the timer

    }
    
  return;
}
