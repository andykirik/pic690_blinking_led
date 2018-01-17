/*
 * File:   main.c
 * Author: akirik
 *
 * Created on February 23, 2016, 12:52 PM
 * 
 * Blinking LED
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
    
    // ANSELx registers
        ANSEL = 0x00;         // Set PORT ANS0 to ANS7 as Digital I/O
        ANSELH = 0x00;        // Set PORT ANS8 to ANS11 as Digital I/O
        ANSELbits.ANS0 = 1;   // Set RA0/AN0 to analog mode
  
    // TRISx registers (This register specifies the data direction of each pin)
        TRISA = 0x00;         // Set All on PORTB as Output    
        TRISB = 0x00;         // Set All on PORTB as Output    
        TRISC = 0x00;         // Set All on PORTC as Output    
        TRISAbits.TRISA0 = 1; // Disable the output driver for pin RA0/AN0
    
    // PORT registers
        PORTA = 0x00;         // Set PORTA all 0
        PORTB = 0x00;         // Set PORTB all 0
        PORTC = 0x00;         // Set PORTC all 0
        
    // set the ADC to the options selected in the User Interface
        ADCON0bits.ADFM = 1;    //ADC result is right justified
        ADCON0bits.VCFG = 0;    //Vdd is the +ve reference
        ADCON1bits.ADCS = 0b001;//Fosc/8 is the conversion clock
                                //This is selected because the conversion
                                //clock period (Tad) must be greater than 1.5us.
                                //With a Fosc of 4MHz, Fosc/8 results in a Tad
                                //of 2us.
        ADCON0bits.CHS = PIN_A0;//select analog input, AN0
        ADCON0bits.ADON = 1;    //Turn on the ADC
        ADRESL = 0x00;
        ADRESH = 0x00;
        
    // initial state of LEDs (off)
        PORTCbits.RC0 = 0;
        PORTCbits.RC1 = 0;
        PORTCbits.RC2 = 0;
        PORTCbits.RC3 = 0;
}

uint16_t ADC_GetConversion()
{
    // Acquisition time delay
    __delay_us(ACQ_US_DELAY);

    // Start the conversion
    ADCON0bits.GO_nDONE = 1;

    // Wait for the conversion to finish
    while (ADCON0bits.GO_nDONE);

    // Conversion finished, return the result
    return ((uint16_t)((ADRESH << 8) + ADRESL));
}

void main(void) 
{
    system_init();
    
    uint8_t adcResult;
    while(1)  
	{
        //Get the top 4 MSBs and display it on the LEDs
        adcResult = ADC_GetConversion();// >> 12;
        
        if(adcResult > 512)
            PORTCbits.RC0 = 1;      //turn on the LED if the input voltage is above Vdd/2
        else
            PORTCbits.RC0 = 0;      //otherwise turn off the LED

        /*/Determine which LEDs will light up
        PORTCbits.RC0 =  adcResult & 1;
        PORTCbits.RC1 = (adcResult & 2) >> 1;
        PORTCbits.RC2 = (adcResult & 4) >> 2;
        PORTCbits.RC3 = (adcResult & 8) >> 3;*/
        
        PORTCbits.RC3 = 1;
    }
    
  return;
}
