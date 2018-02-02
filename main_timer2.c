/*
 * File:   main_timer.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Using Timer 2
 * 
 *  Board connection (PICKit 2 Low Count Demo; PIC16F690):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC0 (DS1; J1->10)         LED
 * 
 */

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

#include <xc.h>

void system_init()
{
    OSCCONbits.IRCF = 0b010;      // Select 250 kHz internal clock
    
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
        
	// Timer Setup - Timer 2
    /* The Timer2 module is an eight-bit timer.
     * -------------------T2CON---------------------------------------------
     * Bit#:  ---7----6-------5-------4-------3-------2------1-------0------
     * ANS:   -|---|TOUTPS3|TOUTPS2|TOUTPS1|TOUTPS0|TMR2ON|T2CKPS1|T2CKPS0|-
     * ---------------------------------------------------------------------
     * TOUTPS<3:0>: Timer2 Output Postscaler Select bits
     * TMR2ON: Timer2 On bit
     * T2CKPS<1:0>: Timer2 Clock Prescale Select bits
     * 
     * A prescaler is a circuit that reduces the frequency of a clock using integer division. 
     *  The prescaler can be set anywhere from 1:1 to 1:16 for Timer 2.
     *  The clock we are slowing down is NOT the system clock Fosc (250 kHz as in here). 
     *  It's the system's instruction clock Fcy, which is always Fosc/4.
     *  The postscaler has postscale options of 1:1 to 1:16 for Timer 2. 
     *  The output of the Timer2 postscaler is used to set the TMR2IF interrupt flag bit in the PIR1 register.
     *  The timer expires when the TMR2 register rolls over. 
     *  The TMR2 register is an 8 bit register, therefore it will roll over after 256 counts.
     *  Rollover Frequency = Fosc / (4 * prescaler * 256 * postscaler)
     *  In following case it would be 1.90 Hz or 0.52 seconds per rollover.
    */
		TMR2 = 0;                   // Start with zero Counter
        T2CON = 0b01110111;         // Postscaler: 1:8, Timer2=On, Prescaler: 1:16
}

void main(void) 
{
    system_init();
    
    while(1)
    {        
        while(PIR1bits.TMR2IF == 0){} // Wait until Timer 2 overflows(TMR2IF bit of the PIR1 register is set).

        PORTCbits.RC0 = ~PORTCbits.RC0; // Toggle the LED
		
        PIR1bits.TMR2IF = 0;        // Clear the Timer 2 interrupt flag
        TMR2 = 0;                   // Load a value of 2 into the timer

    }
    
  return;
}
