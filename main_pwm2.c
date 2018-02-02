/*
 * File:   main_pwm.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * PWM
 * 
 *  Board connection (PICKit 2 Low Count Demo; PIC16F690):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC2/P1D (DS3; J1->12)   LED
 *  RC3/P1C (DS4; J1->6)    LED
 * 
 *  RA0 (RP1)               POTENCIOMETER
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
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA4/OSC2/CLKOUT and RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

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
		
		// PORT registers
			PORTA = 0x00;         // Set PORTA all 0
			PORTB = 0x00;         // Set PORTB all 0
			PORTC = 0x00;         // Set PORTC all 0
                    
    // PWM setup
    // The PWM mode generates a Pulse-Width Modulated signal on the CCP1 pin. 
    // The duty cycle, period and resolution are determined by the following registers:
    // PR2, T2CON, CCPR1L, CCP1CON
    // In Pulse-Width Modulation (PWM) mode, the CCP module produces up to a 10-bit resolution 
    // PWM output on the CCP1 pin. Since the CCP1 pin is multiplexed with the PORT data latch, 
    // the TRIS for that pin must be cleared to enable the CCP1 pin output driver.
    // The PWM period is specified by the PR2 register of Timer2.
    // In PWM mode, the CCP1 pin can output a 10-bit resolution periodic digital waveform 
    // with programmable period and duty cycle. 
    // The duty cycle of the waveform to be generated is a 10-bit value of which 
    // the upper eight bits are stored in the CCPR1L register, 
    // whereas the lowest two bits are stored in bit 5 and bit 4 of the CCP1CON register.
    // The PIC PWM output ports P1A (RC5), P1B (RC4), P1C (RC3) and P1D (RC2).
    // The PWM output behavior is controlled by the CCP1CON, PWM1CON and PSTRCON registers
    // All the TRIS register for each of the PWM output ports: P1A, P1B, P1C and P1D should be set to the output mode.
    // PWM period = (PR2 + 1) x 4 x Tosc x (TMR2 prescale value) seconds
    // PWM frequency = 1 / PWM Period Hz
    // PWM width = (CCPR1L:CCP1CON<5:4>) x Tosc x (TMR2 prescale value) seconds.
    /* 
	 * -------------------CCP1CON----------------------------------
     * Bit#:  ---7----6----5------4-----3------2------1------0-----
     *        -|P1M1|P1M0|DC1B1|DC1B0|CCP1M3|CCP1M2|CCP1M1|CCP1M0|-
     * ------------------------------------------------------------
     * P1M<1:0>: PWM Output Configuration bits
     * DC1B<1:0>: PWM Duty Cycle Least Significant bits
     * CCP1M<3:0>: ECCP Mode Select bits
     * 
     * -------------------PSTRCON----------------------------------
     * Bit#:  ---7----6---5----4------3----2----1----0-------------
     *        -|---|---|----|STRSYNC|STRD|STRC|STRB|STRA|----------
     * ------------------------------------------------------------
     * STRSYNC  - Steering Sync bit
     * STRD     - Steering Enable bit D
     * STRC     - Steering Enable bit C
     * STRB     - Steering Enable bit B
     * STRA     - Steering Enable bit A
     * 
     * The following steps configure the CCP module for PWM operation:
     *   1. Establish the PWM period by writing to the PR2 register.
     *   2. Establish the PWM duty cycle by writing to the DCxB9:DCxB0 bits.
     *   3. Make the CCPx pin an output by clearing the appropriate TRIS bit.
     *   4. Establish the TMR2 prescale value and enable Timer2 by writing to T2CON.
     *   5. Configure the CCP module for PWM operation.
    */
    // Set the PWM period by loading the PR2 register.
        PR2 = 0x65;                 // Frequency: 4.90 kHz
        // PWM period = (101 + 1) x 4 x (1 / 8000000) x 4 = 0.000204 second
        // PWM frequency = 1 / PWM period = 1 / 0.000204 = 4901.96 Hz ~ 4.90 kHz
        PSTRCON = 0b00000100;       // Enable Pulse Steering on P1C (RC3)
        
    // Configure the CCP module for the PWM mode by loading the CCPxCON register with the appropriate values.
        //CCP1CON = 0b00001100;     // Single PWM mode; P1A, P1C active-high; P1B, P1D active-high
        CCP1CONbits.P1M = 0b00;     // Single output mode
        CCP1CONbits.DC1B = 0x00;    // Start with zero Duty Cycle (LSB)
        CCP1CONbits.CCP1M = 0b1100; // ECCP Mode PWM P1A, P1C active-high; P1B, P1D active-high
        CCPR1L = 0;                 // Start with zero Duty Cycle (MSB)
        
    // Configure and start Timer2:
        TMR2 = 0;                   // Start with zero Counter
        T2CON = 0b00000101;         // Postscale: 1:1, Timer2=On, Prescale = 1:4
        
    // Enable PWM output after a new PWM cycle has started:
        while(PIR1bits.TMR2IF == 0){} // Wait until Timer2 overflows(TMR2IF bit of the PIR1 register is set).
}

void main(void) 
{
    system_init();
    
    unsigned int ipwm = 0;
    unsigned char mode = 0;
    while(1)  
	{
        ipwm = 0;
        while (ipwm < 255) 
        {
          CCPR1L = ++ipwm;
          __delay_ms(5);     
        } 	                    

        ipwm = 0xff;
        while (ipwm > 0) {
            CCPR1L = --ipwm;
            __delay_ms(5);   
        }       

        __delay_ms(100);     
        
        if (0 == mode) 
        {
            mode = 1;
            PSTRCON=0b00001000;  // Enable Pulse Steering on P1D (RC2)
        } 
        else if (1 == mode) 
        {
            mode = 2;
            PSTRCON=0b00001100;  // Enable Pulse Steering on P1C and P1D (RC3 and RC2)
        } 
        else 
        {
            mode = 0;
            PSTRCON=0b00000100;  // Enable Pulse Steering on P1C (RC3)
        }
    }
    
    return;
}
