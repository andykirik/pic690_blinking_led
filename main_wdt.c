/*
 * File:   main.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Blinking LED
 * Digital I/O
 * 
 *  Board connection (PICKit 2 Low Count Demo; PIC16F690):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC0 (DS1; J1->10)         LED
 *  RC1 (DS2; J1->11)         LED
 *  RC2 (DS3; J1->12)         LED
 *  RC3 (DS4; J1->6)          LED
 *
 */

/* The __delay_ms() function is provided by XC8. 
It requires you define _XTAL_FREQ as the frequency of your system clock. 
The compiler then uses that value to calculate how many cycles are required to give the requested delay. 
There is also __delay_us() for microseconds and _delay() to delay for a specific number of clock cycles. 
Note that __delay_ms() and __delay_us() begin with a double underscore whereas _delay() 
begins with a single underscore.
*/
#define _XTAL_FREQ 8000000

// CONFIG
// PIC16F690 Configuration Bit Settings
#pragma config FOSC     = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE     = ON        // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE    = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE    = ON        // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP       = OFF       // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD      = OFF       // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN    = ON        // Brown-out Reset Selection bits (BOR enabled)
#pragma config IESO     = ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN    = ON        // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)

#include <xc.h>

void system_init()
{
    OSCCON=0x70;                // Select 8 Mhz internal clock    
    OPTION_REG = 0b00001111;    // Assign 1:128 prescaler to WDT (~2 sec)
	
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
}

void main(void) 
{
    system_init();
   
    PORTCbits.RC0 = 1;
    PORTCbits.RC1 = 0;
    PORTCbits.RC2 = 1;
    PORTCbits.RC3 = 0;

    //Enable Watchdog Timer
    WDTCONbits.SWDTEN = 1;

    SLEEP();

    PORTCbits.RC0 = 0;
    PORTCbits.RC1 = 1;
    PORTCbits.RC2 = 0;
    PORTCbits.RC3 = 1;

    WDTCONbits.SWDTEN = 0;
        
    while(1)
    {}
    
	return;
}
