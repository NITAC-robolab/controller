/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB? Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC16F1939
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

#include "mcc_generated_files/mcc.h"
bool tmrFlag = false;
//IS=X,RE=X,SD=0
#define CLEARDISP  0x01
//IS=X,RE=0,SD=0
#define RETURNHOME  0x03
//IS=X,RE=0,SD=0
#define ENTRYMODE   0x07
#define CURSOR_INC  0x07
#define CURSOR_DEC  0x05
#define SHIFT_EN    0x07
#define SHIFT_DIS   0x06
//IS=X,RE=0,SD=0
#define DISP        0x0f
#define DSP_ON      0x0f
#define DSP_OFF     0x0b
#define CSR_ON  0x0f
#define CSR_OFF 0x0d
#define BLK_ON  0x0f
#define BLK_OFF 0x0e
//IS=0,RE=0,SD=0
#define SHIFT       0x1f
#define DISPLAY_SHT 0x1f
#define CURSOR_SFT  0x17
#define SHIFT_R     0x1f
#define SHIFT_L     0x1b
//IS=X,RE=0,SD=0
#define FUNCTION    0x3f
#define BIT_8       0x3f
#define BIT_4       0x2f
#define LINE_2      0x3f
#define LINE_1      0x37
#define FONT_11     0x3f
#define FONT_8      0x3b
//IS=0,RE=0,SD=0
#define SET_CGADD   0x40
//IS=0,RE=X,SD=0
#define SET_DDADD   0x80
/*
 */
#define INST_   0
#define DATA_   1

#define OLED_ADDR     (0x3c)
//void I2C_M_Write(uint8_t,uint8_t *,uint8_t);
void oled_cmd(uint8_t);
void oled_data(uint8_t);
void oled_init(void);
void oled_position(uint8_t, uint8_t);
void oled_char_disp(uint8_t, uint8_t, uint8_t);
void oled_str_disp(uint8_t, uint8_t, const char *);
void oled_contrast(uint8_t);
void oled_clear(void);
#define SLAVE_I2C_GENERIC_RETRY_MAX           2
#define SLAVE_I2C_GENERIC_DEVICE_TIMEOUT      20   // define slave timeout 

typedef union indataunion {

    struct {
        unsigned IO_LCrossU : 1;
        unsigned IO_LCrossD : 1;
        unsigned IO_LCrossR : 1;
        unsigned IO_LCrossL : 1;
        unsigned IO_LMid : 1;
        unsigned IO_LDown : 1;
        unsigned IO_TactSw00 : 1;
        unsigned IO_LED : 1;
    };
    uint8_t byte;
} innerData;
innerData subdata;
#define IO_LCrossU_GetValue() (subdata.IO_LCrossU)
#define IO_LCrossD_GetValue() (subdata.IO_LCrossD)
#define IO_LCrossR_GetValue() (subdata.IO_LCrossR)
#define IO_LCrossL_GetValue() (subdata.IO_LCrossL)
#define IO_LMid_GetValue() (subdata.IO_LMid)
#define IO_LDown_GetValue() (subdata.IO_LDown)
#define IO_TactSw00_GetValue() (subdata.IO_TactSw00)
#define IO_LED_LAT subdata.IO_LED

void I2C_M_write(uint8_t addr, uint8_t* wr, uint8_t length) {


    // write to an EEPROM Device


    static uint16_t timeOut, slaveTimeOut;

    static I2C1_MESSAGE_STATUS status;
    status = I2C1_MESSAGE_PENDING;




    // Now it is possible that the slave device will be slow.
    // As a work around on these slaves, the application can
    // retry sending the transaction
    timeOut = 0;
    slaveTimeOut = 0;

    while (status != I2C1_MESSAGE_FAIL) {
        // write one byte to EEPROM (3 is the number of bytes to write)
        I2C1_MasterWrite(wr,
                length,
                addr,
                &status);

        // wait for the message to be sent or status has changed.
        while (status == I2C1_MESSAGE_PENDING) {
            // add some delay here
            __delay_us(100);
            // timeout checking
            // check for max retry and skip this byte
            if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        if ((slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT) ||
                (status == I2C1_MESSAGE_COMPLETE))
            break;

        // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
        //               or I2C1_DATA_NO_ACK,
        // The device may be busy and needs more time for the last
        // write so we can retry writing the data, this is why we
        // use a while loop here

        // check for max retry and skip this byte
        if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX)
            break;
        else
            timeOut++;
    }

    if (status == I2C1_MESSAGE_FAIL) {
        return;
    }

}

/*
                         Main application
 */
void oled_cmd(uint8_t work) {
    static char write[2];
    write[0] = 0x00; // Co=0, RS=0
    write[1] = work;
    I2C_M_write(OLED_ADDR, (uint8_t*) write, 2);
    __delay_us(40);
}

void oled_data(uint8_t work) {
    static char write[2];
    write[0] = 0x40; // Co=0, RS=0
    write[1] = work;
    I2C_M_write(OLED_ADDR, (uint8_t*) write, 2);
    __delay_us(40);
}

void oled_init(void) {
    oled_cmd(CLEARDISP); // clear diaplay
    __delay_ms(2);
    oled_cmd(RETURNHOME); // to home position
    oled_cmd(DISP & DSP_ON & CSR_OFF & BLK_OFF); // display on
    oled_cmd(CLEARDISP);
    __delay_ms(2);
}

void oled_position(uint8_t li, uint8_t col) {
    oled_cmd(0x80 | (li << 5) | col); // line0 columun0 = 0x00, line1 columun0 = 0x20
}

void oled_char_disp(uint8_t li, uint8_t col, uint8_t ascii) {
    oled_position(li, col);
    oled_data(ascii);
}

void oled_str_disp(uint8_t li, uint8_t col, const char *string) {
    uint8_t i = 0;
    oled_position(li, col);
    while (((col + i) < 16) && string[i]) {
        //IO_RE5_Toggle();
        //UART2_Write((string[i]));
        oled_data(string[i]);
        //IO_RE5_Toggle();
        i++;
    }
}

void oled_contrast(uint8_t cont) {
    oled_cmd(0x2A); // RE = 1
    oled_cmd(0x79); // SD = 1
    oled_cmd(0x81); // contrast control
    oled_cmd(cont); // value
    oled_cmd(0x78); // SD = 0
    oled_cmd(0x28); // RE = 0
    __delay_ms(2);
}

void oled_clear(void) {
    oled_cmd(CLEARDISP);
    __delay_ms(2);
}

void EUSART_CompWrite(uint8_t c) {
    EUSART1_Write(c);
    EUSART1_Write(~c);
}
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
    
    //Explicit the program is a Xbee mode:LED extinction
    LATEbits.LATE2=0;
    //RA4 unused
    IO_LAnPush_PORT=0;
    
    __delay_ms(5);
    oled_init();
    oled_contrast(0xFF);
    oled_str_disp(0, 0, "OLED TEST");
    while (1) {
        if (tmrFlag) {
            tmrFlag = false;
            EUSART1_Write('S');
            EUSART1_Write('S');
            EUSART_CompWrite((IO_RCrossD_GetValue() << 7) | (IO_RCrossU_GetValue() << 6) | (IO_RCrossR_GetValue() << 5) | (IO_RCrossL_GetValue() << 4)
                    | (IO_LCrossD_GetValue() << 3) | (IO_LCrossU_GetValue() << 2) | (IO_LCrossR_GetValue() << 1) | (IO_LCrossL_GetValue() << 0));
            EUSART_CompWrite((IO_RLeverSwR_GetValue() << 7) | (IO_RLeverSwL_GetValue() << 6) | (IO_LLeverSwR_GetValue() << 5) | (IO_LLeverSwL_GetValue() << 4)
                    | (IO_RMid_GetValue() << 3) | (IO_RUp_GetValue() << 2) | (IO_LMid_GetValue() << 1) | (IO_LUp_GetValue() << 0));

            EUSART_CompWrite(0xff - (ADC_GetConversion(channel_LHorizon) >> 2));
            EUSART_CompWrite(0xff - (ADC_GetConversion(channel_LVertical) >> 2));

            EUSART_CompWrite(0xff - (ADC_GetConversion(channel_RHorizon) >> 2));
            EUSART_CompWrite(0xff - (ADC_GetConversion(channel_RVertical) >> 2));

            EUSART_CompWrite((IO_RDown_GetValue() << 7) | (IO_LDown_GetValue() << 6) | ((1 - IO_RDipSw_GetValue()) << 1) | (IO_LDipSw_GetValue() << 0));
            EUSART_CompWrite((IO_TactSw12_GetValue() << 5) | (IO_TactSw11_GetValue() << 4) | (IO_TactSw10_GetValue() << 3) | (IO_TactSw02_GetValue() << 2) | (IO_TactSw01_GetValue() << 1) | (IO_TactSw00_GetValue() << 0));
            EUSART1_Write('E');
            EUSART1_Write('E');
            //sprintf(str, "%2x%2x%2x%2x%2x%2x", IO_LLeverSwL_GetValue(), IO_LLeverSwR_GetValue(), IO_RLeverSwL_GetValue(), IO_RLeverSwR_GetValue(), IO_LAnPush_GetValue(), IO_RAnPush_GetValue());
            //oled_str_disp(1, 0, str);
            // Add your application code
        }
        if (EUSART2_DataReady) {
            subdata.byte = EUSART2_Read();
        }
        
/**********UART通信規格_対コントローラ**************
 * (0)'S’
 * (1)'S'
 * (2)右十字[7]D[6]U[5]R[4]L 左十字[3]D[2]U[1]R[0]L
 * (3)(2)のNOT
 * (4)右レバー[7]RLR[6]RLL 左レバー[5]LLR [4]LLL　右上中[3]RM[2]RU 左上中[1]LM[0]LU
 * (5)(4)のNOT
 * (6)左アナログ水平方向[0]~[7]LH
 * (7)(6)のNOT
 * (8)左アナログ垂直方向[0]~[7]LV
 * (9)(8)のNOT
 * (10)右アナログ水平方向[0]~[7]RH
 * (11)(10)のNOT
 * (12)右アナログ垂直方向[0]~[7]RV
 * (13)(12)のNOT
 * (14)右下[7],左下[6]　0[5]　0[4]　右アナログ押込[3]R3　左アナログ押込[2]L3　右モードSW[1]RMS　左モードSW[0]LMS　
 * (15)(14)のNOT
 * (16)0[7]　0[6] タクトスイッチ右下[5] タクトスイッチ中下[4] タクトスイッチ左下[3] タクトスイッチ右上[2] タクトスイッチ中上[1] タクトスイッチ左上[0]
 * (17)(16)のNOT
 * (18)'E'
 * (19)'E'
 *
 *
 *
 *
 *
******************************************/
        if (EUSART1_DataReady) {
            static uint8_t recieveddata;
            static uint8_t recievedbefore = '\0';
            static uint8_t columun = 0;
            static uint8_t line = 0;
            static uint8_t escaped = false;
            uint8_t i = 0;
            recieveddata = EUSART1_Read();
            if (!escaped) {
                switch (recievedbefore) {
                    case '\0':
                        if (recieveddata == '\0') {
                            oled_clear();
                            oled_position(line = 0, columun = 0);
                            escaped = true;
                        }else{
                            escaped = false;
                        }
                        break;
                    case '\x11':
                        oled_position(line = recieveddata, columun = 0);
                        escaped = true;
                        break;
                    case '\x12':
                        oled_position(line, columun = recieveddata);
                        escaped = true;
                        break;
                    default:
                        escaped = false;
                        break;
                }
            } else {
                escaped = false;
            }
            if (!escaped) {
                switch (recieveddata) {
                    case '\r':
                    case '\0':
                        oled_position(line, columun);
                        for (i = 0; i < 16 - columun; i++) {
                            oled_data(' ');
                        }
                    case '\x11':
                    case '\x12':
                        break;
                    case '\n':
                        if (recievedbefore == '\r' && line == 0) {
                            oled_position(line = 1, columun = 0);
                        }
                        break;
                    default:
                        oled_data(recieveddata);
                        columun++;
                        break;
                }
            }
            recievedbefore = recieveddata;
        }
    }
}
/**
 End of File
 */