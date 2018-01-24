/*
 * File:   main_adc.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Analog to Digital Converter (ADC)
 * 
 *  Board connection (PICKit 2 Low Count Demo; PIC16F690):
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
			ANSELbits.ANS0 = 1;   // Set RA0/AN0 to analog mode
	  
		// TRISx registers (This register specifies the data direction of each pin)
			TRISA = 0x00;         // Set All on PORTB as Output    
			TRISB = 0x00;         // Set All on PORTB as Output    
			TRISC = 0x00;         // Set All on PORTC as Output    
			TRISAbits.TRISA0 = 1; // Set RA0/AN0 as Input
		
		// PORT registers
			PORTA = 0x00;         // Set PORTA all 0
			PORTB = 0x00;         // Set PORTB all 0
			PORTC = 0x00;         // Set PORTC all 0
        
    // ADC setup
	// The PIC16F690 has a 10-bit ADC
	// It provides value 0 to 1023 as the input voltage rises from Vss to Vref
	// Vref is the analog reference voltage. 
	// It can either be Vdd or supplied externally to the RA1/AN1/Vref pin
	// The PIC16F690 has twelve analog input pins (AN0 through AN11). 
	// In order to enable the analog function for a pin, 
	// the corresponding bit must be set in the ANSEL or ANSELH register (see above).
    // Steps to configure:
    //  1. voltage reference in the ADCON1 register
    //  2. select ADC conversion clock in the ADCON0 register
    //  3. select one input channel CH0CH13 in the ADCON0 register
    //  4. select data format in the ADCON1 register (ADFM)
    //  5. enable AD converter in the ADCON0 register (ADON)
    //  optional configure interrupt:
    //  6. clear ADIF bit
    //  7. set ADIE, PEIE and GIE bits (see interrupts for more instructions)
    /* 
     * -------------------ADC---------------------------------
     * Bit#:  ----7----6----5----4----3----2----1-------0-----
     * ANS:   --|ADFM|VCFG|CHS3|CHS2|CHS1|CHS0|GO/DONE|ADON|--
     * -------------------------------------------------------
        ADFM - result format: 0 - left justified (default), 1 - right justified:
            * -------------------------ADRESH------------------------|-------------------ADRESL-----------------------
            * Bit#:  ---7-----6-----5-----4-----3-----2-----1-----0--|--7-----6-----5-----4-----3-----2-----1-----0---
            * bit:   |  0  |  0  |  0  |  0  |  0  |  0  |bit-9|bit-8|bit-7|bit-6|bit-5|bit-4|bit-3|bit-2|bit-1|bit-0|
            * --------------------------------------------------------------------------------------------------------
            result = (ADRESH<<8)+ADRESL;

            * -------------------------ADRESH------------------------|-------------------ADRESL-----------------------
            * Bit#:  ---7-----6-----5-----4-----3-----2-----1-----0--|--7-----6-----5-----4-----3-----2-----1-----0---
            * bit:   |bit-9|bit-8|bit-7|bit-6|bit-5|bit-4|bit-3|bit-2|bit-1|bit-0|  0  |  0  |  0  |  0  |  0  |  0  |
            * --------------------------------------------------------------------------------------------------------
            This is useful if only the 8 most significant bits are required. 
            In this case, we just simply copy the contens of the ADRESH register.
      
        VCFG - Voltage reference (Vref): 0 - Vdd, 1 - Vref pin     
      
        CHS<3:0> - channel select: This selects which voltage is fed into the ADC. 
            Setting this 0 to 11 selects AN0 to AN11 respectively. 
            Setting 12 feeds CVref (Comparator voltage reference). 
            Setting 13 connects the 0.6V fixed voltage reference from the
            Comparator module to the ADC.

        GO/DONE: 1 to start ADC conversion. 
            Be sure to wait at least 5us before doing this after changing 
            the input channel or performing a previous conversion.
            This bit is read to see if the current conversion is complete.
            The ADC will automatically reset this bit to 0 when conversion finishes.

        ADON - ADC enable bit: 0 - ADC disabled and consumes no current, 1 - ADC enabled.
     
     * -------------------ADCON1-------------------------
     * Bit#:  ----7----6-----5-----4----3---2---1---0----
     * ANS:   --| 0 |ADCS2|ADCS1|ADCS0| 0 | 0 | 0 | 0 |--
     * --------------------------------------------------
        ADCS<2:0> - ADC clock select:
            0b000 = Fosc/2
            0b001 = Fosc/8
            0b010 = Fosc/32
            0b011 or 0b111 = FRC (internal oscillator)
            0b100 = Fosc/4
            0b101 = Fosc/16
            0b110 = Fosc/64
          
     ADC could be user with interrupts
        In order to use the ADC interrupts, the GIE (Global Interrupt Enable) 
        and PEIE (Peripheral Interrupt Enable) bits in the INTCON register must be set to 1.
     
        * -------------------PIE1-----------------------------------
        * Bit#:  ----7----6----5---4-----3-----2------1------0------
        * ANS:   --| 0 |ADIE|RCIE|TXIE|SSPIE|CCP1IE|TMR2IE|TMR1IE|--
        * ----------------------------------------------------------
           ADIE - ADC interrupt enable. 
               Set to 1 to generate interrupts when ADC is completed
        
        * -------------------PIR1-----------------------------------
        * Bit#:  ----7----6---5----4----3------2------1------0------
        * ANS:   --| 0 |ADIF|RCIF|TXIF|SSPIF|CCP1IF|TMR2IF|TMR1IF|--
        * ----------------------------------------------------------
           ADIF - ADC interrupt flag. 
               This will be set to 1 by the ADC when a conversion completes. 
               This must be cleared by software in the interrupt service routine 
               to prepare the interrupt flag for the next conversion.
    */
        ADCON0bits.ADFM = 1;   		// ADC result is right justified
        ADCON0bits.VCFG = 0;    	// Vref uses Vdd as reference
        ADCON1bits.ADCS = 0b001;	// Fosc/8 is the conversion clock
									//   This is selected because the conversion
									//   clock period (Tad) must be greater than 1.5us.
									//   With a Fosc of 4MHz, Fosc/8 results in a Tad
									//   of 2us.
        ADCON0bits.CHS = PIN_A0;	// Select analog input - AN0
        ADCON0bits.ADON = 1;    	// Turn on the ADC
        
    // initial state of LED - OFF (redundant, just to show another method to set)
        PORTCbits.RC0 = 0;
        PORTCbits.RC1 = 0;
        PORTCbits.RC2 = 0;
        PORTCbits.RC3 = 0;
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
        uint16_t adcResult = ADC_GetConversion();
                
        //PORTCbits.RC0 = adcResult > 512 ? 1 : 0;// Turn on the LED if the input voltage is above Vdd/2
		//__delay_ms(50);                         // sleep 50 milliseconds
		
		PORTCbits.RC0 = 0;
        PORTCbits.RC1 = 0;
        PORTCbits.RC2 = 0;
        PORTCbits.RC3 = 0;
		
        if(adcResult > 256) {
            PORTCbits.RC0 = 1;
        } 
        if(adcResult > 512)  {
            PORTCbits.RC1 = 1;
        } 
        if(adcResult > 768)  {
            PORTCbits.RC2 = 1;
        }
        if(adcResult > 1000)  {
            PORTCbits.RC3 = 1;      
        }
		__delay_ms(50);                         // sleep 50 milliseconds
    }
    
  return;
}
