/*
 * File:   main_button.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Switch and a LED
 * Digital I/O
 * 
 *  Board connection (PICKit 2 Low Count Demo; PIC16F690):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC0 (DS1; J1->10)         LED
 * 
 *  RA3/MCLR/VPP (SW1, J1->3) BUTTON    Please note that because this pin has MCLRE function as well
 *                                      So this program might not work plugged in to programmer!!!
 *                                      It should be plugged in into internal power source
 *                                      and config MCLRE should be set to OFF
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
#pragma config FOSC 	= HS		// Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE 	= OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE 	= OFF      	// Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE 	= OFF      	//!!! MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP 		= OFF       // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD 		= OFF       // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN 	= ON       	// Brown-out Reset Selection bits (BOR enabled)
#pragma config IESO 	= ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN 	= ON       	// Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)

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
        ANSEL = 0x00;         // Set PORT ANS0 to ANS7 as Digital I/O
        ANSELH = 0x00;        // Set PORT ANS8 to ANS11 as Digital I/O
        ANSELbits.ANS3 = 0;   //digital switch (redundant, just to show another method to set)
  
    // TRISx registers (This register specifies the data direction of each pin)
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
        TRISA = 0b00001000;       // Set All on PORTB as Output, and A3 is set as Input   
        TRISB = 0x00;             // Set All on PORTB as Output    
        TRISC = 0x00;             // Set All on PORTC as Output    
        TRISAbits.TRISA3 = 1;     //switch as input (redundant, just to show another method to set)
    
    // PORT registers (hold the current digital state of the digital I/O)
    // If you read these registers, you can determine which pins are currently HIGH or LOW
    // Writing to the PORTX registers will set the digital output latches. 
    // Writing to a pin that is currently an input  will have no effect on the pin because the output latch will be disabled.
        PORTA = 0x00;         // Set PORTA all 0
        PORTB = 0x00;         // Set PORTB all 0
        PORTC = 0x00;         // Set PORTC all 0
        
    // initial state of LED - OFF (redundant, just to show another method to set)
        PORTCbits.RC0 = 0;
}

void main(void) 
{
    system_init();
    
    while(1)
    {        
        PORTCbits.RC0 = 0 == RA3 ? 1 : 0;       // RA3 == 0V (pressed)
        __delay_ms(50);                         // sleep 50 milliseconds

        /*if (0 == RA3) {                       //switch is normally open to 5V ...when pressed, RA3 is connected to GND
            __delay_ms(10);                     //debounce by delaying and checking if switch is still pressed
            if (0 == RA3)                       //check if still down
                PORTCbits.RC0 = 1;                           
        }
        else
            PORTCbits.RC0 = 0;
        */                          
    }
    
  return;
}
