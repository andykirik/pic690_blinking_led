/* 
 * File:   main_timer_interrupt.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Interrupt
 * Use Peripheral Interrupt.
 * On PICKit 2 Low Count Demo button is connected to RA3/MCLR/VPP, so we use this pin as interrupt source
 * LED 0 blinks once a second
 * LED 3 changes its state on button press
 * 
 *  Board connection (PICKit 2 Low Count Demo; PIC16F690):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC0 (DS1; J1->10)         LED
 *  RC3 (DS4; J1->6)          LED
 * 
 *  RA3/MCLR/VPP (SW1, J1->3) BUTTON    Please note that because this pin has MCLRE function as well
 *                                      So this program might not work plugged in to programmer!!!
 *                                      It should be plugged in into internal power source
 *                                      and config MCLRE should be set to OFF
 
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
    
	// I/O	
		// ANSELx registers
			ANSEL = 0x00;         // Set PORT ANS0 to ANS7 as Digital I/O
			ANSELH = 0x00;        // Set PORT ANS8 to ANS11 as Digital I/O
	  
		// TRISx registers (This register specifies the data direction of each pin)
			TRISA = 0b00001000;   // Set All on PORTB as Output, pin 3 as Input
			TRISB = 0x00;         // Set All on PORTB as Output    
			TRISC = 0x00;         // Set All on PORTC as Output    
		
		// PORT registers (hold the current digital state of the digital I/O)
			PORTA = 0x00;         // Set PORTA all 0
			PORTB = 0x00;         // Set PORTB all 0
			PORTC = 0x00;         // Set PORTC all 0
        
	// Interrupt setup
    /* 
     * -------------------INTCON----------------------------
     * Bit#:  ----7----6----5----4----3----2----1----0------
     *        --|GIE|PEIE|T0IE|INTE|RABIE|T0IF|INTF|RABIF|--
     * -----------------------------------------------------
     * GIE      - global interrupt enable bit
     * PEIE     - peripheral interrupt enable bit
     * T0IE     - Timer 0 overflow interrupt enable bit
     * INTE     - external interrupt enable bit
     * RABIE    - port change interrupt enable bit
     * T0IF     - Timer 0 overflow interrupt flag bit
     * INTF     - external interrupt flag bit
     * RABIF    - port change interrupt flag bit
     * 
     * -------------------OPTION_REG----------------------
     * Bit#:  ----7------6-----5----4----3---2---1---0----
     *        --|RABPU|INTEDG|T0CS|T0SE|PSA|PS2|PS1|PS0|--
     * ---------------------------------------------------
     * INTEDG   - interrupt edge select bit
     * 
     * 
     * In order to initialize the RA3/MCLR/VPP interrupt, the following operations must take place:
     *   1. Port A, pin 3 (RA3), must be initialized as Input.
     *   2. The interrupt source must be set to take place either on the falling or the rising edge of
     *      the signal using INTEDG bit of OPTION_REG.
     *   3. The external interrupt flag (RABIF in the INTCON Register) must be cleared.
     *   4. The Port Change on RA3 must be enabled by setting the RABIF bit in the INTCON
     *      Register and appropriate bin in IOCA register.
     *   5. Peripheral interrupts must be enabled by setting the PEIE bit in the INTCON Register.
     *   6. Global interrupts must be enabled by setting the GIE bit in the INTCON Register.
    */
        OPTION_REGbits.INTEDG   = 0;            // Interrupt on the rising edge
        IOCA                    = 0b00001000;   // Enable RA3 Interrupt
		INTCONbits.RABIF        = 0;            // Reset the Port Change Interrupt flag
        INTCONbits.RABIE        = 1;            // Set the Port Change Interrupt Enabled 
        INTCONbits.PEIE         = 1;            // Set the Peripheral Interrupt Enabled
		INTCONbits.GIE          = 1;            // Set the Global Interrupt Enabled
}

/* 
 * The PIC16F690 can only have one Interrupt Service Routine. 
 * Compiler should know which function is the interrupt handler. 
 * This is done by declaring the function with 'interrupt' prefix:
 */
void interrupt isr()
{
    if(INTCONbits.RABIF == 1)
    {
        PORTCbits.RC3 = ~PORTCbits.RC3; // Toggle the LED
        __delay_ms(200);                // Key debounce time

        INTCONbits.RABIF = 0;           // Clear the Port Change Interrupt flag
    }
    //else if(...)
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
