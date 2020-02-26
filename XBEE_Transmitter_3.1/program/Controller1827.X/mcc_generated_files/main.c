/*
 * File:   main.c
 * Author: medama1221
 *
 * Created on 2015/11/11, 21:03
 */


#include "mcc.h"

extern bool tmrflag;
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
innerData subdata;
/*
                         Main application
 */
char str[16];

void main(void) {
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    __delay_ms(5);
                IO_LED_SetHigh();
                IO_LED_SetLow();
    while (1) {
        if (tmrflag) {
            IO_LED_LAT=IO_LMid_GetValue();
            tmrflag = false;
            EUSART_Write((IO_LED_GetValue() << 7) | (IO_TactSw00_GetValue() << 6) | (IO_LDown_GetValue() << 5) | (IO_LMid_GetValue() << 4)
                    | (IO_LCrossL_GetValue() << 3) | (IO_LCrossR_GetValue() << 2) | (IO_LCrossD_GetValue() << 1) | (IO_LCrossU_GetValue() << 0));
        }
        if(EUSART_DataReady){
            if(EUSART_Read()&0x80){
                IO_LED_SetHigh();
            }else{
                IO_LED_SetLow();
            }
        }
    }
}
/**
 End of File
 */