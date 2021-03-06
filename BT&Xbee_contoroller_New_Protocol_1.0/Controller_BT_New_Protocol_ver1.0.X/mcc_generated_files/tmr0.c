/**
  TMR0 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr0.c

  @Summary
    This is the generated driver implementation file for the TMR0 driver using MPLAB? Code Configurator

  @Description
    This source file provides APIs for TMR0.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC18F46K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

/**
  Section: Included Files
 */

#include <xc.h>
#include "tmr0.h"
#include "mcc.h"
extern bool waitCTS;

typedef union indataunion {

    struct {
        unsigned IO_LCrossU : 1;
        unsigned IO_LCrossD : 1;
        unsigned IO_LCrossR : 1;
        unsigned IO_LCrossL : 1;
        unsigned IO_LMid : 1;
        unsigned IO_LDown : 1;
        unsigned IO_LTactSw0 : 1;
        unsigned IO_LED : 1;
    };
    uint8_t byte;
} innerData;
extern innerData subdata;
#define IO_LCrossU_GetValue() (subdata.IO_LCrossU)
#define IO_LCrossD_GetValue() (subdata.IO_LCrossD)
#define IO_LCrossR_GetValue() (subdata.IO_LCrossR)
#define IO_LCrossL_GetValue() (subdata.IO_LCrossL)
#define IO_LMid_GetValue() (subdata.IO_LMid)
#define IO_LDown_GetValue() (subdata.IO_LDown)
#define IO_LTactSw0_GetValue() (subdata.IO_LTactSw0)
#define IO_LED_LAT subdata.IO_LED
/**
  Section: Global Variables Definitions
 */
volatile uint8_t timer0ReloadVal8bit;

/**
  Section: TMR0 APIs
 */


void TMR0_Initialize(void) {
    // Set TMR0 to the options selected in the User Interface

    // T08BIT 8-bit; T0SE Increment_hi_lo; T0PS 1:64; T0CS FOSC/4; TMR0ON enabled; PSA assigned; 
    T0CON = 0xD5;

    // TMR0H 0; 
    TMR0H = 0x80;

    // TMR0L 0; 
    TMR0L = 0x00;

    // Load TMR0 value to the 8-bit reload variable
    timer0ReloadVal8bit = 0x80;

    // Clear Interrupt flag before enabling the interrupt
    INTCONbits.TMR0IF = 0;

    // Enabling TMR0 interrupt.
    INTCONbits.TMR0IE = 1;

    // Start TMR0
    TMR0_StartTimer();
}

void TMR0_StartTimer(void) {
    // Start the Timer by writing to TMR0ON bit
    T0CONbits.TMR0ON = 1;
}

void TMR0_StopTimer(void) {
    // Stop the Timer by writing to TMR0ON bit
    T0CONbits.TMR0ON = 0;
}

uint8_t TMR0_Read8bitTimer(void) {
    uint8_t readVal;

    // read Timer0, low register only
    readVal = TMR0L;

    return readVal;
}

void TMR0_Write8bitTimer(uint8_t timerVal) {
    // Write to the Timer0 registers, low register only
    TMR0L = timerVal;
}

void TMR0_Reload8bit(void) {
    //Write to the Timer0 register
    TMR0L = timer0ReloadVal8bit;
}

void TMR0_ISR(void) {

    // clear the TMR0 interrupt flag
    INTCONbits.TMR0IF = 0;

    // reload TMR0
    TMR0L = timer0ReloadVal8bit;

    // ticker function call;
    // ticker is 1 -> Callback function gets called every time this ISR executes
    TMR0_CallBack();

    // add your TMR0 interrupt custom code
}

void TMR0_CallBack(void) {
    // Add your custom callback code here
    // this code executes every 1 TMR0 periods
    if (waitCTS == true) {
        if (IO_CTS_GetValue() == 0) {
            IO_LED_LAT = waitCTS = false;
            PIE1bits.TXIE = 1;
        }
    }
}
/**
  End of File
 */