/*
 * File:   main_pwm.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * PWM
 * 
 *  Board connection (PICKit 2 Low Count Demo):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *  RC0 (DS1; J1->10)         LED
 *  RC1 (DS2; J1->11)         LED
 *  RC2 (DS3; J1->12)         LED
 *  RC3 (DS4; J1->6)          LED
 * 
 *  RA0 (RP1)                 POTENCIOMETER
 *
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

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
        CCP1CON = 0b00001100;       // Single PWM mode; P1A, P1C active-high; P1B, P1D active-high
        CCPR1L = 0;                 // Start with zero Duty Cycle
        T2CON = 0b00000101;         // Postscale: 1:1, Timer2=On, Prescale = 1:4
        PR2 = 0x65;                 // Frequency: 4.90 kHz
        TMR2 = 0;                   // Start with zero Counter
        PSTRCON = 0b00000100;       // Enable Pulse Steering on P1C (RC3)
        //state=0;                  // Start with state 1
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
        uint8_t adcResult = ADC_GetConversion() >> 6;		//Start ADC conversion
        adcResult &= 0x03FF;
        
        // Load duty cycle value
        /*/if(CCP1CONbits)
        {
            adcResult <<= 6;
            CCPR1H = adcResult >> 8;
            CCPR1L = adcResult;
        }
        //else*/
        {
            CCPR1H = adcResult >> 8;
            CCPR1L = adcResult;
        }
        
		__delay_ms(50);                         // sleep 50 milliseconds
    }
    
  return;
}
