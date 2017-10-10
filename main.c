/*
 * File:   main.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Blinking LED
 * 
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *   RC0		     	LED (+)
 *
 */

/* The __delay_ms() function is provided by XC8. 
It requires you define _XTAL_FREQ as the frequency of your system clock. 
We are using the internal oscillator at its default 4MHz, so _XTAL_FREQ is defined as 4000000. 
The compiler then uses that value to calculate how many cycles are required to give the requested delay. 
There is also __delay_us() for microseconds and _delay() to delay for a specific number of clock cycles. 
Note that __delay_ms() and __delay_us() begin with a double underscore whereas _delay() 
begins with a single underscore.
*/
#define _XTAL_FREQ 4000000ul  

#include <xc.h>

// CONFIG
// PIC16F690 Configuration Bit Settings
#pragma config FOSC 	= HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE 	= OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE 	= OFF      	// Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE 	= ON       	// MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP 		= OFF       // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD 		= OFF       // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN 	= ON       	// Brown-out Reset Selection bits (BOR enabled)
#pragma config IESO 	= ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN 	= ON       	// Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)

// INIT
void sys_init()
{
  ANSEL 	= 0x00;     // Set PORT ANS0 to ANS7 as Digital I/O
  ANSELH 	= 0x00;		// Set PORT ANS8 to ANS11 as Digital I/O
  TRISC 	= 0x00;     // Set All on PORTC as Output    
  PORTC 	= 0x00;     // Set PORTC all 0
}

// MAIN
void main(void) 
{
    sys_init();
    
    while(1)  
	{
        PORTC ^= 0x01;      // flip RC0
        __delay_ms(1000);   // sleep 1 seconds
    }
    
	return;
}
