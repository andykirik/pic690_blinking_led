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
#pragma config WDTE     = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
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
    OSCCON=0x70;          // Select 8 Mhz internal clock
	
	// To control Digital I/O use three registers: ANSEL, TRIS and PORT
    // For digital I/O any port could be selected.
    // For analog I/O select ports AN0-AN13 (use ANSEL to make selection)
    // Any port could be either Input or Output (use TRIS to make selection)
    // For switches or button use ports which support pull up resistors
    //   And enable pull up resistors using RBPU in OPTION_REG
    // Input port could trigger interrupt on change. Use IOCB and INTCON bits to configure

    // ANSELx registers
	// ANSEL and ANSELH control the mode of AN0 through AN11:
	// 0 sets the pin to digital mode and 1 sets the pin to analog mode.
    /* 
	 * -------------------ANSEL--------------------------
     * Bit#:  ---7----6----5----4----3----2----1----0----
     * ANS:   -|ANS7|ANS6|ANS5|ANS4|ANS3|ANS2|ANS1|ANS0|-
     * --------------------------------------------------
	 * -------------------ANSELH-------------------------
     * Bit#:  ----7---6---5---4----3----2-----1----0-----
     * ANS:   --|---|---|---|---|ANS11|ANS10|ANS9|ANS8|--
     * --------------------------------------------------
     */
        ANSEL = 0x00;         // Set PORT ANS0 to ANS7  as Digital I/O
        ANSELH = 0x00;        // Set PORT ANS8 to ANS11 as Digital I/O
  
    // TRISx registers
	// This register specifies the data direction of each pin:
	// O - output, 1 - input
    /* 
	 * -------------------TRISA---------------------------------------------
     * Bit#:   ---7-------6------5------4------3------2------1------0-------
     * TRIS:   -|------|------|TRISA5|TRISA4|TRISA3|TRISA2|TRISA1|TRISA0|---
     * ---------------------------------------------------------------------
     * PORTA3 is Input only, so TRISA3 is read only
     * 
	 * -------------------TRISB---------------------------------------------
     * Bit#:   ----7------6------5------4------3------2------1------0-------
     * TRIS:   -|TRISB7|TRISB6|TRISB5|TRISB4|------|------|------|------|---
     * ---------------------------------------------------------------------
     * -------------------TRISC---------------------------------------------
     * Bit#:   ----7------6------5------4------3------2------1------0-------
     * TRIS:   --|TRISC7|TRISC6|TRISC5|TRISC4|TRISC3|TRISC2|TRISC1|TRISC0|--
     * ---------------------------------------------------------------------
     */
        TRISA = 0x00;         // Set All on PORTA as Output    
        TRISB = 0x00;         // Set All on PORTB as Output    
        TRISC = 0x00;         // Set All on PORTC as Output    
    
    // PORT registers 
	// Hold the current digital state of the digital I/O
    // If you read these registers, you can determine which pins are currently HIGH or LOW
    // Writing to the PORTX registers will set the digital output latches. 
    // Writing to a pin that is currently an input will have no effect on the pin because the output latch will be disabled.
        PORTA = 0x00;         // Set PORTA all 0
        PORTB = 0x00;         // Set PORTB all 0
        PORTC = 0x00;         // Set PORTC all 0
    
    // initial state of LEDs - OFF (redundant, just to show another method to set)
        PORTCbits.RC0 = 0;
        PORTCbits.RC1 = 0;
        PORTCbits.RC2 = 0;
        PORTCbits.RC3 = 0;
}

static int mode = 1; // 0 - blink; 1 - rotate (need to be recompiled to change modes)

void main(void) 
{
    system_init();
    
    while(1)
    {
        if(0 == mode)
        {
            // turn on and off
            while(1)  
            {
                PORTCbits.RC0 = ~PORTCbits.RC0;   // Toggle the LED 
                PORTCbits.RC1 = ~PORTCbits.RC1;   // Toggle the LED
                PORTCbits.RC2 = ~PORTCbits.RC2;   // Toggle the LED
                PORTCbits.RC3 = ~PORTCbits.RC3;   // Toggle the LED
                __delay_ms(2000);                 // sleep 2 seconds
            }
        }
        else
        {
            // rotate
            int index = 1;
            while(1)
            {
                PORTCbits.RC0 = index & 1;
                PORTCbits.RC1 = (index & 2) >> 1;
                PORTCbits.RC2 = (index & 4) >> 2;
                PORTCbits.RC3 = (index & 8) >> 3;

                index <<= 1;
                if(index >= 16)
                    index = 1;

                __delay_ms(500);                   // sleep 0.5 second
            }
        }
    }
    
	return;
}
