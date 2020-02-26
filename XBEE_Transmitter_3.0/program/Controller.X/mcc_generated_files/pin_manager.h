/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB? Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC16F1939
        Version           :  1.01
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RLeverSwR aliases
#define IO_RLeverSwR_TRIS               TRISA0
#define IO_RLeverSwR_LAT                LATA0
#define IO_RLeverSwR_PORT               RA0
#define IO_RLeverSwR_ANS                ANSA0
#define IO_RLeverSwR_SetHigh()    do { LATA0 = 1; } while(0)
#define IO_RLeverSwR_SetLow()   do { LATA0 = 0; } while(0)
#define IO_RLeverSwR_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define IO_RLeverSwR_GetValue()         RA0
#define IO_RLeverSwR_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define IO_RLeverSwR_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define IO_RLeverSwR_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define IO_RLeverSwR_SetDigitalMode()   do { ANSA0 = 0; } while(0)
// get/set IO_RLeverSwL aliases
#define IO_RLeverSwL_TRIS               TRISA1
#define IO_RLeverSwL_LAT                LATA1
#define IO_RLeverSwL_PORT               RA1
#define IO_RLeverSwL_ANS                ANSA1
#define IO_RLeverSwL_SetHigh()    do { LATA1 = 1; } while(0)
#define IO_RLeverSwL_SetLow()   do { LATA1 = 0; } while(0)
#define IO_RLeverSwL_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define IO_RLeverSwL_GetValue()         RA1
#define IO_RLeverSwL_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define IO_RLeverSwL_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define IO_RLeverSwL_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define IO_RLeverSwL_SetDigitalMode()   do { ANSA1 = 0; } while(0)
// get/set channel_LHorizon aliases
#define channel_LHorizon_TRIS               TRISA2
#define channel_LHorizon_LAT                LATA2
#define channel_LHorizon_PORT               RA2
#define channel_LHorizon_ANS                ANSA2
#define channel_LHorizon_SetHigh()    do { LATA2 = 1; } while(0)
#define channel_LHorizon_SetLow()   do { LATA2 = 0; } while(0)
#define channel_LHorizon_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define channel_LHorizon_GetValue()         RA2
#define channel_LHorizon_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define channel_LHorizon_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define channel_LHorizon_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define channel_LHorizon_SetDigitalMode()   do { ANSA2 = 0; } while(0)
// get/set channel_LVertical aliases
#define channel_LVertical_TRIS               TRISA3
#define channel_LVertical_LAT                LATA3
#define channel_LVertical_PORT               RA3
#define channel_LVertical_ANS                ANSA3
#define channel_LVertical_SetHigh()    do { LATA3 = 1; } while(0)
#define channel_LVertical_SetLow()   do { LATA3 = 0; } while(0)
#define channel_LVertical_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define channel_LVertical_GetValue()         RA3
#define channel_LVertical_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define channel_LVertical_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define channel_LVertical_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define channel_LVertical_SetDigitalMode()   do { ANSA3 = 0; } while(0)
// get/set IO_LAnPush aliases
#define IO_LAnPush_TRIS               TRISA4
#define IO_LAnPush_LAT                LATA4
#define IO_LAnPush_PORT               RA4
#define IO_LAnPush_ANS                ANSA4
#define IO_LAnPush_SetHigh()    do { LATA4 = 1; } while(0)
#define IO_LAnPush_SetLow()   do { LATA4 = 0; } while(0)
#define IO_LAnPush_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define IO_LAnPush_GetValue()         RA4
#define IO_LAnPush_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define IO_LAnPush_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define IO_LAnPush_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define IO_LAnPush_SetDigitalMode()   do { ANSA4 = 0; } while(0)
// get/set IO_RDipSw aliases
#define IO_RDipSw_TRIS               TRISA5
#define IO_RDipSw_LAT                LATA5
#define IO_RDipSw_PORT               RA5
#define IO_RDipSw_ANS                ANSA5
#define IO_RDipSw_SetHigh()    do { LATA5 = 1; } while(0)
#define IO_RDipSw_SetLow()   do { LATA5 = 0; } while(0)
#define IO_RDipSw_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define IO_RDipSw_GetValue()         RA5
#define IO_RDipSw_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define IO_RDipSw_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

#define IO_RDipSw_SetAnalogMode()   do { ANSA5 = 1; } while(0)
#define IO_RDipSw_SetDigitalMode()   do { ANSA5 = 0; } while(0)
// get/set IO_CTS aliases
#define IO_CTS_TRIS               TRISA7
#define IO_CTS_LAT                LATA7
#define IO_CTS_PORT               RA7
#define IO_CTS_SetHigh()    do { LATA7 = 1; } while(0)
#define IO_CTS_SetLow()   do { LATA7 = 0; } while(0)
#define IO_CTS_Toggle()   do { LATA7 = ~LATA7; } while(0)
#define IO_CTS_GetValue()         RA7
#define IO_CTS_SetDigitalInput()    do { TRISA7 = 1; } while(0)
#define IO_CTS_SetDigitalOutput()   do { TRISA7 = 0; } while(0)

// get/set IO_RDown aliases
#define IO_RDown_TRIS               TRISB0
#define IO_RDown_LAT                LATB0
#define IO_RDown_PORT               RB0
#define IO_RDown_WPU                WPUB0
#define IO_RDown_ANS                ANSB0
#define IO_RDown_SetHigh()    do { LATB0 = 1; } while(0)
#define IO_RDown_SetLow()   do { LATB0 = 0; } while(0)
#define IO_RDown_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define IO_RDown_GetValue()         RB0
#define IO_RDown_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define IO_RDown_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define IO_RDown_SetPullup()    do { WPUB0 = 1; } while(0)
#define IO_RDown_ResetPullup()   do { WPUB0 = 0; } while(0)
#define IO_RDown_SetAnalogMode()   do { ANSB0 = 1; } while(0)
#define IO_RDown_SetDigitalMode()   do { ANSB0 = 0; } while(0)
// get/set IO_LUp aliases
#define IO_LUp_TRIS               TRISB1
#define IO_LUp_LAT                LATB1
#define IO_LUp_PORT               RB1
#define IO_LUp_WPU                WPUB1
#define IO_LUp_ANS                ANSB1
#define IO_LUp_SetHigh()    do { LATB1 = 1; } while(0)
#define IO_LUp_SetLow()   do { LATB1 = 0; } while(0)
#define IO_LUp_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define IO_LUp_GetValue()         RB1
#define IO_LUp_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define IO_LUp_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define IO_LUp_SetPullup()    do { WPUB1 = 1; } while(0)
#define IO_LUp_ResetPullup()   do { WPUB1 = 0; } while(0)
#define IO_LUp_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define IO_LUp_SetDigitalMode()   do { ANSB1 = 0; } while(0)
// get/set IO_RMid aliases
#define IO_RMid_TRIS               TRISB2
#define IO_RMid_LAT                LATB2
#define IO_RMid_PORT               RB2
#define IO_RMid_WPU                WPUB2
#define IO_RMid_ANS                ANSB2
#define IO_RMid_SetHigh()    do { LATB2 = 1; } while(0)
#define IO_RMid_SetLow()   do { LATB2 = 0; } while(0)
#define IO_RMid_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define IO_RMid_GetValue()         RB2
#define IO_RMid_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define IO_RMid_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define IO_RMid_SetPullup()    do { WPUB2 = 1; } while(0)
#define IO_RMid_ResetPullup()   do { WPUB2 = 0; } while(0)
#define IO_RMid_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define IO_RMid_SetDigitalMode()   do { ANSB2 = 0; } while(0)
// get/set IO_LED aliases
#define IO_LED_TRIS               TRISB3
#define IO_LED_LAT                LATB3
#define IO_LED_PORT               RB3
#define IO_LED_WPU                WPUB3
#define IO_LED_ANS                ANSB3
#define IO_LED_SetHigh()    do { LATB3 = 1; } while(0)
#define IO_LED_SetLow()   do { LATB3 = 0; } while(0)
#define IO_LED_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define IO_LED_GetValue()         RB3
#define IO_LED_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define IO_LED_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define IO_LED_SetPullup()    do { WPUB3 = 1; } while(0)
#define IO_LED_ResetPullup()   do { WPUB3 = 0; } while(0)
#define IO_LED_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define IO_LED_SetDigitalMode()   do { ANSB3 = 0; } while(0)
// get/set IO_RUp aliases
#define IO_RUp_TRIS               TRISB4
#define IO_RUp_LAT                LATB4
#define IO_RUp_PORT               RB4
#define IO_RUp_WPU                WPUB4
#define IO_RUp_ANS                ANSB4
#define IO_RUp_SetHigh()    do { LATB4 = 1; } while(0)
#define IO_RUp_SetLow()   do { LATB4 = 0; } while(0)
#define IO_RUp_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define IO_RUp_GetValue()         RB4
#define IO_RUp_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define IO_RUp_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define IO_RUp_SetPullup()    do { WPUB4 = 1; } while(0)
#define IO_RUp_ResetPullup()   do { WPUB4 = 0; } while(0)
#define IO_RUp_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define IO_RUp_SetDigitalMode()   do { ANSB4 = 0; } while(0)
// get/set IO_LDipSw aliases
#define IO_LDipSw_TRIS               TRISB5
#define IO_LDipSw_LAT                LATB5
#define IO_LDipSw_PORT               RB5
#define IO_LDipSw_WPU                WPUB5
#define IO_LDipSw_ANS                ANSB5
#define IO_LDipSw_SetHigh()    do { LATB5 = 1; } while(0)
#define IO_LDipSw_SetLow()   do { LATB5 = 0; } while(0)
#define IO_LDipSw_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define IO_LDipSw_GetValue()         RB5
#define IO_LDipSw_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define IO_LDipSw_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define IO_LDipSw_SetPullup()    do { WPUB5 = 1; } while(0)
#define IO_LDipSw_ResetPullup()   do { WPUB5 = 0; } while(0)
#define IO_LDipSw_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define IO_LDipSw_SetDigitalMode()   do { ANSB5 = 0; } while(0)
// get/set IO_RCrossR aliases
#define IO_RCrossR_TRIS               TRISC0
#define IO_RCrossR_LAT                LATC0
#define IO_RCrossR_PORT               RC0
#define IO_RCrossR_SetHigh()    do { LATC0 = 1; } while(0)
#define IO_RCrossR_SetLow()   do { LATC0 = 0; } while(0)
#define IO_RCrossR_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define IO_RCrossR_GetValue()         RC0
#define IO_RCrossR_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define IO_RCrossR_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

// get/set IO_LCrossU aliases
#define IO_LCrossU_TRIS               TRISC1
#define IO_LCrossU_LAT                LATC1
#define IO_LCrossU_PORT               RC1
#define IO_LCrossU_SetHigh()    do { LATC1 = 1; } while(0)
#define IO_LCrossU_SetLow()   do { LATC1 = 0; } while(0)
#define IO_LCrossU_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define IO_LCrossU_GetValue()         RC1
#define IO_LCrossU_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define IO_LCrossU_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

// get/set IO_LCrossL aliases
#define IO_LCrossL_TRIS               TRISC2
#define IO_LCrossL_LAT                LATC2
#define IO_LCrossL_PORT               RC2
#define IO_LCrossL_SetHigh()    do { LATC2 = 1; } while(0)
#define IO_LCrossL_SetLow()   do { LATC2 = 0; } while(0)
#define IO_LCrossL_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define IO_LCrossL_GetValue()         RC2
#define IO_LCrossL_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define IO_LCrossL_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

// get/set SCL aliases
#define SCL_TRIS               TRISC3
#define SCL_LAT                LATC3
#define SCL_PORT               RC3
#define SCL_SetHigh()    do { LATC3 = 1; } while(0)
#define SCL_SetLow()   do { LATC3 = 0; } while(0)
#define SCL_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define SCL_GetValue()         RC3
#define SCL_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define SCL_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

// get/set SDA aliases
#define SDA_TRIS               TRISC4
#define SDA_LAT                LATC4
#define SDA_PORT               RC4
#define SDA_SetHigh()    do { LATC4 = 1; } while(0)
#define SDA_SetLow()   do { LATC4 = 0; } while(0)
#define SDA_Toggle()   do { LATC4 = ~LATC4; } while(0)
#define SDA_GetValue()         RC4
#define SDA_SetDigitalInput()    do { TRISC4 = 1; } while(0)
#define SDA_SetDigitalOutput()   do { TRISC4 = 0; } while(0)

// get/set IO_LCrossD aliases
#define IO_LCrossD_TRIS               TRISC5
#define IO_LCrossD_LAT                LATC5
#define IO_LCrossD_PORT               RC5
#define IO_LCrossD_SetHigh()    do { LATC5 = 1; } while(0)
#define IO_LCrossD_SetLow()   do { LATC5 = 0; } while(0)
#define IO_LCrossD_Toggle()   do { LATC5 = ~LATC5; } while(0)
#define IO_LCrossD_GetValue()         RC5
#define IO_LCrossD_SetDigitalInput()    do { TRISC5 = 1; } while(0)
#define IO_LCrossD_SetDigitalOutput()   do { TRISC5 = 0; } while(0)

// get/set TX aliases
#define TX_TRIS               TRISC6
#define TX_LAT                LATC6
#define TX_PORT               RC6
#define TX_SetHigh()    do { LATC6 = 1; } while(0)
#define TX_SetLow()   do { LATC6 = 0; } while(0)
#define TX_Toggle()   do { LATC6 = ~LATC6; } while(0)
#define TX_GetValue()         RC6
#define TX_SetDigitalInput()    do { TRISC6 = 1; } while(0)
#define TX_SetDigitalOutput()   do { TRISC6 = 0; } while(0)

// get/set RX aliases
#define RX_TRIS               TRISC7
#define RX_LAT                LATC7
#define RX_PORT               RC7
#define RX_SetHigh()    do { LATC7 = 1; } while(0)
#define RX_SetLow()   do { LATC7 = 0; } while(0)
#define RX_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define RX_GetValue()         RC7
#define RX_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define RX_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

// get/set IO_LLeverSwR aliases
#define IO_LLeverSwR_TRIS               TRISD0
#define IO_LLeverSwR_LAT                LATD0
#define IO_LLeverSwR_PORT               RD0
#define IO_LLeverSwR_ANS                ANSD0
#define IO_LLeverSwR_SetHigh()    do { LATD0 = 1; } while(0)
#define IO_LLeverSwR_SetLow()   do { LATD0 = 0; } while(0)
#define IO_LLeverSwR_Toggle()   do { LATD0 = ~LATD0; } while(0)
#define IO_LLeverSwR_GetValue()         RD0
#define IO_LLeverSwR_SetDigitalInput()    do { TRISD0 = 1; } while(0)
#define IO_LLeverSwR_SetDigitalOutput()   do { TRISD0 = 0; } while(0)

#define IO_LLeverSwR_SetAnalogMode()   do { ANSD0 = 1; } while(0)
#define IO_LLeverSwR_SetDigitalMode()   do { ANSD0 = 0; } while(0)
// get/set IO_LLeverSwL aliases
#define IO_LLeverSwL_TRIS               TRISD1
#define IO_LLeverSwL_LAT                LATD1
#define IO_LLeverSwL_PORT               RD1
#define IO_LLeverSwL_ANS                ANSD1
#define IO_LLeverSwL_SetHigh()    do { LATD1 = 1; } while(0)
#define IO_LLeverSwL_SetLow()   do { LATD1 = 0; } while(0)
#define IO_LLeverSwL_Toggle()   do { LATD1 = ~LATD1; } while(0)
#define IO_LLeverSwL_GetValue()         RD1
#define IO_LLeverSwL_SetDigitalInput()    do { TRISD1 = 1; } while(0)
#define IO_LLeverSwL_SetDigitalOutput()   do { TRISD1 = 0; } while(0)

#define IO_LLeverSwL_SetAnalogMode()   do { ANSD1 = 1; } while(0)
#define IO_LLeverSwL_SetDigitalMode()   do { ANSD1 = 0; } while(0)
// get/set IO_LCrossR aliases
#define IO_LCrossR_TRIS               TRISD2
#define IO_LCrossR_LAT                LATD2
#define IO_LCrossR_PORT               RD2
#define IO_LCrossR_ANS                ANSD2
#define IO_LCrossR_SetHigh()    do { LATD2 = 1; } while(0)
#define IO_LCrossR_SetLow()   do { LATD2 = 0; } while(0)
#define IO_LCrossR_Toggle()   do { LATD2 = ~LATD2; } while(0)
#define IO_LCrossR_GetValue()         RD2
#define IO_LCrossR_SetDigitalInput()    do { TRISD2 = 1; } while(0)
#define IO_LCrossR_SetDigitalOutput()   do { TRISD2 = 0; } while(0)

#define IO_LCrossR_SetAnalogMode()   do { ANSD2 = 1; } while(0)
#define IO_LCrossR_SetDigitalMode()   do { ANSD2 = 0; } while(0)
// get/set IO_RCrossU aliases
#define IO_RCrossU_TRIS               TRISD3
#define IO_RCrossU_LAT                LATD3
#define IO_RCrossU_PORT               RD3
#define IO_RCrossU_ANS                ANSD3
#define IO_RCrossU_SetHigh()    do { LATD3 = 1; } while(0)
#define IO_RCrossU_SetLow()   do { LATD3 = 0; } while(0)
#define IO_RCrossU_Toggle()   do { LATD3 = ~LATD3; } while(0)
#define IO_RCrossU_GetValue()         RD3
#define IO_RCrossU_SetDigitalInput()    do { TRISD3 = 1; } while(0)
#define IO_RCrossU_SetDigitalOutput()   do { TRISD3 = 0; } while(0)

#define IO_RCrossU_SetAnalogMode()   do { ANSD3 = 1; } while(0)
#define IO_RCrossU_SetDigitalMode()   do { ANSD3 = 0; } while(0)
// get/set IO_RCrossL aliases
#define IO_RCrossL_TRIS               TRISD4
#define IO_RCrossL_LAT                LATD4
#define IO_RCrossL_PORT               RD4
#define IO_RCrossL_ANS                ANSD4
#define IO_RCrossL_SetHigh()    do { LATD4 = 1; } while(0)
#define IO_RCrossL_SetLow()   do { LATD4 = 0; } while(0)
#define IO_RCrossL_Toggle()   do { LATD4 = ~LATD4; } while(0)
#define IO_RCrossL_GetValue()         RD4
#define IO_RCrossL_SetDigitalInput()    do { TRISD4 = 1; } while(0)
#define IO_RCrossL_SetDigitalOutput()   do { TRISD4 = 0; } while(0)

#define IO_RCrossL_SetAnalogMode()   do { ANSD4 = 1; } while(0)
#define IO_RCrossL_SetDigitalMode()   do { ANSD4 = 0; } while(0)
// get/set IO_LDown aliases
#define IO_LDown_TRIS               TRISD5
#define IO_LDown_LAT                LATD5
#define IO_LDown_PORT               RD5
#define IO_LDown_ANS                ANSD5
#define IO_LDown_SetHigh()    do { LATD5 = 1; } while(0)
#define IO_LDown_SetLow()   do { LATD5 = 0; } while(0)
#define IO_LDown_Toggle()   do { LATD5 = ~LATD5; } while(0)
#define IO_LDown_GetValue()         RD5
#define IO_LDown_SetDigitalInput()    do { TRISD5 = 1; } while(0)
#define IO_LDown_SetDigitalOutput()   do { TRISD5 = 0; } while(0)

#define IO_LDown_SetAnalogMode()   do { ANSD5 = 1; } while(0)
#define IO_LDown_SetDigitalMode()   do { ANSD5 = 0; } while(0)
// get/set IO_RCrossD aliases
#define IO_RCrossD_TRIS               TRISD6
#define IO_RCrossD_LAT                LATD6
#define IO_RCrossD_PORT               RD6
#define IO_RCrossD_ANS                ANSD6
#define IO_RCrossD_SetHigh()    do { LATD6 = 1; } while(0)
#define IO_RCrossD_SetLow()   do { LATD6 = 0; } while(0)
#define IO_RCrossD_Toggle()   do { LATD6 = ~LATD6; } while(0)
#define IO_RCrossD_GetValue()         RD6
#define IO_RCrossD_SetDigitalInput()    do { TRISD6 = 1; } while(0)
#define IO_RCrossD_SetDigitalOutput()   do { TRISD6 = 0; } while(0)

#define IO_RCrossD_SetAnalogMode()   do { ANSD6 = 1; } while(0)
#define IO_RCrossD_SetDigitalMode()   do { ANSD6 = 0; } while(0)
// get/set IO_LMid aliases
#define IO_LMid_TRIS               TRISD7
#define IO_LMid_LAT                LATD7
#define IO_LMid_PORT               RD7
#define IO_LMid_ANS                ANSD7
#define IO_LMid_SetHigh()    do { LATD7 = 1; } while(0)
#define IO_LMid_SetLow()   do { LATD7 = 0; } while(0)
#define IO_LMid_Toggle()   do { LATD7 = ~LATD7; } while(0)
#define IO_LMid_GetValue()         RD7
#define IO_LMid_SetDigitalInput()    do { TRISD7 = 1; } while(0)
#define IO_LMid_SetDigitalOutput()   do { TRISD7 = 0; } while(0)

#define IO_LMid_SetAnalogMode()   do { ANSD7 = 1; } while(0)
#define IO_LMid_SetDigitalMode()   do { ANSD7 = 0; } while(0)
// get/set channel_RHorizon aliases
#define channel_RHorizon_TRIS               TRISE0
#define channel_RHorizon_LAT                LATE0
#define channel_RHorizon_PORT               RE0
#define channel_RHorizon_ANS                ANSE0
#define channel_RHorizon_SetHigh()    do { LATE0 = 1; } while(0)
#define channel_RHorizon_SetLow()   do { LATE0 = 0; } while(0)
#define channel_RHorizon_Toggle()   do { LATE0 = ~LATE0; } while(0)
#define channel_RHorizon_GetValue()         RE0
#define channel_RHorizon_SetDigitalInput()    do { TRISE0 = 1; } while(0)
#define channel_RHorizon_SetDigitalOutput()   do { TRISE0 = 0; } while(0)

#define channel_RHorizon_SetAnalogMode()   do { ANSE0 = 1; } while(0)
#define channel_RHorizon_SetDigitalMode()   do { ANSE0 = 0; } while(0)
// get/set channel_RVertical aliases
#define channel_RVertical_TRIS               TRISE1
#define channel_RVertical_LAT                LATE1
#define channel_RVertical_PORT               RE1
#define channel_RVertical_ANS                ANSE1
#define channel_RVertical_SetHigh()    do { LATE1 = 1; } while(0)
#define channel_RVertical_SetLow()   do { LATE1 = 0; } while(0)
#define channel_RVertical_Toggle()   do { LATE1 = ~LATE1; } while(0)
#define channel_RVertical_GetValue()         RE1
#define channel_RVertical_SetDigitalInput()    do { TRISE1 = 1; } while(0)
#define channel_RVertical_SetDigitalOutput()   do { TRISE1 = 0; } while(0)

#define channel_RVertical_SetAnalogMode()   do { ANSE1 = 1; } while(0)
#define channel_RVertical_SetDigitalMode()   do { ANSE1 = 0; } while(0)
// get/set IO_RAnPush aliases
#define IO_RAnPush_TRIS               TRISE2
#define IO_RAnPush_LAT                LATE2
#define IO_RAnPush_PORT               RE2
#define IO_RAnPush_ANS                ANSE2
#define IO_RAnPush_SetHigh()    do { LATE2 = 1; } while(0)
#define IO_RAnPush_SetLow()   do { LATE2 = 0; } while(0)
#define IO_RAnPush_Toggle()   do { LATE2 = ~LATE2; } while(0)
#define IO_RAnPush_GetValue()         RE2
#define IO_RAnPush_SetDigitalInput()    do { TRISE2 = 1; } while(0)
#define IO_RAnPush_SetDigitalOutput()   do { TRISE2 = 0; } while(0)

#define IO_RAnPush_SetAnalogMode()   do { ANSE2 = 1; } while(0)
#define IO_RAnPush_SetDigitalMode()   do { ANSE2 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */