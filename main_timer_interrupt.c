/*
 * File:   main_timer_interrupt.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Using timer with interrupt
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

#include <xc.h>

#define EXTENDED_DELAY

#ifdef EXTENDED_DELAY
#define TIMER_RESET_VALUE 6 // To set up the timer for a period of 1 ms (timerPeriod)
                            // Calculated by the formula:
                            // TMR0 = 256 - ((timerPeriod * Fosc) / (4 * prescaler)) + x
                            // TMR1 = 65536 - ((timerPeriod * Fosc) / (4 * prescaler)) + x

                            // In following case,   timerPeriod = 0.001s
                            //                      Fosc = 4,000,000
                            //                      prescaler = 4
                            //                      x = 0 because prescaler is > 2

                            // TMR0 = 256 - (0.001 * 4000000) / (4 * 4) = 6

int delayTime = 0;          // store the time that has elapsed
#endif

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
#ifdef EXTENDED_DELAY
        OPTION_REGbits.PS = 0b001;  // Set the prescaler to 1:4
#else
		OPTION_REGbits.PS = 0b111;  // Set the prescaler to 1:256
#endif
		OPTION_REGbits.T0CS = 0;    // Use the instruction clock (Fcy/4) as the timer clock. 
									//   Other option is an external oscillator or clock on the T0CKI pin.
        INTCONbits.T0IE = 1;        // Enable the Timer 0 interrupt
		INTCONbits.T0IF = 0;        // Clear the Timer 0 interrupt flag
#ifdef EXTENDED_DELAY
		TMR0 = TIMER_RESET_VALUE;   // Load the starting value back into the timer
#else
        TMR0 = 0;                   // Load a value of 0 into the timer
#endif

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
#ifdef EXTENDED_DELAY
    TMR0 = TIMER_RESET_VALUE;   // Load the starting value back into the timer
#else
    TMR0 = 0;               // Load a value of 0 into the timer
                            //   This is actually not necessary since the register will be at 0 anyway after rolling over
#endif
    
#ifdef EXTENDED_DELAY
    if(++delayTime >= 5000) // 5 seconds has elapsed
    {
        delayTime = 0;
#endif
        PORTCbits.RC3 = ~PORTCbits.RC3; // Toggle the LED
#ifdef EXTENDED_DELAY    
    }
#endif
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
