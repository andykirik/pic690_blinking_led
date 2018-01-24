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
 *  RC3 (DS4; J1->6)          LED
 * 
 *  RA0 (RP1)                 POTENCIOMETER
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

#define PIN_A0                    0
#define ACQ_US_DELAY              5

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
            
    // ADC setup
        ADCON0bits.ADFM = 1;   		// ADC result is right justified
        ADCON0bits.VCFG = 0;    	// Vref uses Vdd as reference
        ADCON1bits.ADCS = 0b001;	// Fosc/8 is the conversion clock
									//   This is selected because the conversion
									//   clock period (Tad) must be greater than 1.5us.
									//   With a Fosc of 4MHz, Fosc/8 results in a Tad
									//   of 2us.
        ADCON0bits.CHS = PIN_A0;	// Select analog input - AN0
        ADCON0bits.ADON = 1;    	// Turn on the ADC
        
    // PWM setup
    // The PWM mode generates a Pulse-Width Modulated signal on the CCP1 pin. 
    // The duty cycle, period and resolution are determined by the following registers:
    // PR2, T2CON, CCPR1L, CCP1CON
    // In Pulse-Width Modulation (PWM) mode, the CCP module produces up to a 10-bit resolution 
    // PWM output on the CCP1 pin. Since the CCP1 pin is multiplexed with the PORT data latch, 
    // the TRIS for that pin must be cleared to enable the CCP1 pin output driver.
    // The PWM period is specified by the PR2 register of Timer2.
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
    */
        PR2 = 0x65;                 // Frequency: 4.90 kHz
        T2CON = 0b00000101;         // Postscale: 1:1, Timer2=On, Prescale = 1:4
        CCPR1L = 0;                 // Start with zero Duty Cycle
        CCP1CON = 0b00001100;       // Single PWM mode; P1A, P1C active-high; P1B, P1D active-high
        TMR2 = 0;                   // Start with zero Counter
        PSTRCON = 0b00000100;       // Enable Pulse Steering on P1C (RC3)
}

uint16_t ADC_GetConversion()
{
    __delay_us(ACQ_US_DELAY);					// Acquisition time delay
    ADCON0bits.GO_nDONE = 1;					// Start the conversion
    while (ADCON0bits.GO_nDONE);				// Wait for the conversion to finish
    return ((uint16_t)((ADRESH << 8) + ADRESL));// Conversion finished, return the result
}

void main(void) 
{
    system_init();
    
    while(1)  
	{
        uint8_t adcResult = ADC_GetConversion();		//Start ADC conversion

        CCPR1H = adcResult >> 8;
        CCPR1L = adcResult;
        
		__delay_ms(50);                         // sleep 50 milliseconds
    }
    
  return;
}
