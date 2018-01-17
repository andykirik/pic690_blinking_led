/*
 * File:   main_timer_interrupt.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Using timer with interrupt
 * 
 *  Board connection (PICKit 2 Low Count Demo):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC0 (DS1; J1->10)         LED
 *  RC1 (DS2; J1->11)         LED
 *  RC2 (DS3; J1->12)         LED
 *  RC3 (DS4; J1->6)          LED
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
        
	// Timer Setup
	// Use Timer 0
		OPTION_REGbits.PSA = 0; 	// Prescaler assigned to Timer 0 (other option is to
									//   the Watchdog timer (WDT))
		OPTION_REGbits.PS = 0b111;  // Set the prescaler to 1:256
		OPTION_REGbits.T0CS = 0;    // Use the instruction clock (Fcy/4) as the timer
									//   clock. Other option is an external oscillator
									//   or clock on the T0CKI pin.
        INTCONbits.T0IE = 1;        // Enable the Timer 0 interrupt
		INTCONbits.T0IF = 0;        // Clear the Timer 0 interrupt flag
		TMR0 = 0;                   // Load a value of 0 into the timer

	// Interrupt setup
		INTCONbits.T0IE = 1;        // Enable the Timer 0 interrupt
		INTCONbits.GIE = 1;         // Set the Global Interrupt Enable
}

/* 
 * The PIC16F690 can only have one Interrupt Service Routine. 
 * Compiler should know which function is the interrupt handler. 
 * This is done by declaring the function with 'interrupt' prefix:
 */
void interrupt isr()
{
    INTCONbits.T0IF = 0;    // Clear the Timer 0 interrupt flag
    TMR0 = 0;               // Load a value of 0 into the timer
                            //   This is actually not necessary since the register will be at 0 anyway after rolling over

    PORTCbits.RC3 = ~PORTCbits.RC3; // Toggle the LED
}

void main(void) 
{
    system_init();
    
    while(1)
    {        
        PORTCbits.RC0 = ~PORTCbits.RC0;	// Toggle the LED
        __delay_ms(1000);   			// sleep 1 seconds

    }
    
  return;
}
