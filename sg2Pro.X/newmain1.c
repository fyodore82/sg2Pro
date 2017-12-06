/* 
 * File:   newmain.c
 * Author: fedor
 *
 * Created on 30 ??????? 2012 ?., 14:35
 */

#include <stdio.h>
#include <stdlib.h>
//#include <pic16f690.h>
#include <htc.h>
#include "i2c_my.h"
#include "PinsCheck.h"
#include "EngBinSt.h"

//#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
__CONFIG(FOSC_INTRCIO & WDTE_OFF & PWRTE_ON & MCLRE_ON & CP_OFF & CPD_OFF & BOREN_OFF & IESO_ON & FCMEN_OFF);
/*
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Selection bits (BOR disabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
*/
#define _XTAL_FREQ 4000000

#define FRM_OFF     1   // OFF format, 5 bytes
#define FRM_PINS    2   // PINS format, 5 bytes

#define _I2C_PICADDR    0b10010000

#define _I2C_NOCMD  0
#define _I2C_DAY    1
#define _I2C_HOUR   2
#define _I2C_MIN    3
#define _I2C_SEC    4
#define _I2C_ADDRH  5
#define _I2C_ADDRL  6
#define _I2C_UBATT  7
#define _I2C_TENG   8
#define _I2C_PINSTATE   9
#define _I2C_BINST      10
#define _I2C_ENGST      11
#define _I2C_INTSVCSYNC 12
#define _I2C_NEWBINTMR  13
#define _I2C_NEWASRTMR  14
#define _I2C_ASRONTMR   15
#define _I2C_ENGSTARTTMR    16
#define _I2C_UBATTTMR   17
#define _I2C_TEB        18
#define _I2C_DI21       19
#define _I2C_AI23       20
#define _I2C_AI24       21
#define _I2C_DI25       22
#define _I2C_DO26       23
#define _I2C_DO27       24
#define _I2C_AI28       25
#define _I2C_AI29       26
#define _I2C_AI210      27
#define _I2C_DI1416     28
#define _I2C_DO1819     29
#define _I2C_UTRHLD     30
#define _I2C_TTRHLD     31
#define _I2C_CMDREBOOT  32
#define _I2C_EEPROMRW   33



#define ASRTOST     1   // ASR should be started (CN_25 is 1, wait when it will be 0 and start)
#define ASROFFON    2   // ASR should be turned ON (gave negative impulse to ASR, but not acknoleged that Eng started)
#define ENGASRON    3   // Engine started by ASR
#define ASRONOFF    4   // ASR should be turned OFF (gave negative impulse to ASR, but not acknoleged that Eng turned OFF)
#define ASRTOSTOP   5   // Got STOP command (CN_25 is 0, wait when it will be 1 and stop)

#define ENGON       7   // Engine started by key
#define OFF         0   // Everything OFF
#define ERR         0xFF    // Error

#define BINTOST     0x11   // Binar should be started (CN_25 is 1, wait when it will be 0 and start)
#define BINOFFON    0x12   // Binar is starting now
#define BINON       0x13   // Binar is ON
#define BINONOFF    0x14   // Binar is turning OFF now
#define BINTOSTOP   0x15   // Got STOP command (CN_25 is 0, wait when it will be 1 and stop)

#define UBT_TMRST   0xF8    // When UbattTmt is equal to this value, start engine

#define TEB_HIGH    0x84    // Highest value for TEB (Engine Start)
#define TEB_LOW     0x7B    // Lowest value for TEB (Binag Start)

unsigned char IntSvcSync;   //  We'll sync main routine with InterruptService

#define ISS_NEWSEC  0b00000001
#define ISS_PINIFS  0b00000100
#define ISS_NEWERR  0b00001000      // We identified ERR state first time
#define ISS_AUTOST  0b00010000      // Eng/Binar started automatically, ie by Voltage (not by command)
#define ISS_NEWASRON    0b00100000  // We just switched from ASROFFOM to ASRENGON
#define ISS_TEBHL   0b01000000      // TEB goes to TEB_LOW

// StateTest - we got STOP command and will send DO=1 if we will start engine
unsigned char StateTest;

#define     ST_NONE     0
#define     ST_TEST     1

// ASRChkTmr - After starting by ASR check if Engine works for at least ASRChkTmr. If less, go to ERR
// EngChkTmr - After starting engine check if it works for at least EngChkTmr. If less don't clear ERR and go to ERR
// NewBinTmr, NewASRTmr - Timers, which are turned on after Binar or ASR switched to new states
// EngStartTmr - Time after Eng switched to OFF during Eng can be started and turned OFF withou ERR
// ASRONTmr - After this timer ASR will be started. Timer turned on after Binar stopped
// UbattTmr - Timer decremented if battery is low
// TEB - Start Engine or Binar. If more or equal to 0x80 - Engine, if less or equal to 0x7F - Binar
unsigned char BinSt, EngSt, NewBinTmr, NewASRTmr, ASRONTmr, EngStartTmr, UbattTmr, TEB;

unsigned char addrh, addrl;

unsigned char day, sec, min, hour;

/*
 * 
 */


void OutRet (char addr, char *ret, char Read)
{
    char gie = INTCONbits.GIE;
    INTCONbits.GIE = 0x0;               // Disable ALL interrupts
    EEADR = addr;               // Address EEPROM
    EECON1bits.EEPGD = 0x0;     // ?????? ? ??????
    if (Read)
    {
        EECON1bits.RD = 1;
        *ret = EEDAT;
    }
    else
    {
        EEDAT = *ret;
        EECON1bits.WREN = 0x1;      // Write enabled
        EECON2 = 0x55;              // Magic sequence
        EECON2 = 0xAA;
        EECON1bits.WR = 0x1;

        // EECON1bits.WREN = 0x0;
        while(EECON1bits.WR);
        _delay(100000);     // Safe Delay
    }
    INTCONbits.GIE = gie;            // Enable Interrupts back
}

void interrupt InterruptService (void)
{
    if (INTCONbits.RABIF)
    {
        char n = DI21;      // Read DI ports
        n = DI25;
        INTCONbits.RABIF = 0;
        ToCheck = 0xFF;     // Check all
    }
    // Timer 1 expired
    if (PIR1bits.TMR1IF)
    {
        PIR1bits.TMR1IF = 0;
      //  DO1819 = !DO1819;
        sec += 16;
        IntSvcSync |= ISS_NEWSEC;
    }
}

void WriteLog (void)        // Writes to I2C memory
// start format:
//  0b1------- - PINS format
//  0b01010101 - STOP byte
//  0b00111--- - OFF format
//  0b00010--- - BinSt = ERR format
//  0b00001--- - EngSt = ERR format
{
    char d[6];
    d[5] = 0b01010101;  // STOP byte

    d[3] = Ubatt;
    d[4] = (Teng & 0b01111111) | ((TEB >= 0x80) << 7);

    if ((BinSt == OFF && EngSt == OFF) || BinSt == ERR || EngSt == ERR)
        // Format when everything OFF
        //  start | day | hour | min | Ubatt  | TEB | Teng
        //    5   |  8  |  5   |  6  |   8    |  1  |  7
    {
        d[0] = 0b00111000 | (day >> 5);
        if (BinSt == ERR)
        {
            d[0] = 0b00010000 | (day >> 5);
        }
        if (EngSt == ERR)
        {
            d[0] = 0b00001000 | (day >> 5);
        }
        if (BinSt == ERR && EngSt == ERR)
        {
            d[0] = 0b00011000 | (day >> 5);
        }
        d[1] = (day << 3) | (hour >> 2);
        d[2] = (hour << 6) | min;
    }
    else                        // PINS format when something ON
                                //  start | hour | min | ENGST | BINST | pins | Ubatt | TEB | Teng
                                //    1   |  5   |  6  |   3   |   3   |   6  |   8   |  1  |  7
    {
        d[0] = 0b10000000 | (hour << 2) | (min >> 4);
        d[1] = (min << 4) | ((EngSt & 0b00000111) << 1) | ((BinSt & 0b00000111) >> 2);
        d[2] = ((BinSt & 0b00000111) << 6) | ((PinState & 0b01101110) >> 1) | (DO1819 << 4);
    }

    char i = 3;
    while (TXNbytes (addrh, addrl, 6, d) && i > 0)
    {
        __delay_ms(10);
        i--;
    }
    addrl += 5;
    if (STATUSbits.C)
        addrh += 1;
}

void SearchStop (void)  // Search for stop byte
{
    unsigned char d = 0xFF;
    addrh = 0;
    addrl = 0;
    while (d != 0b01010101 && addrh != 0xFF)
    {

        if (d & 0b11111000)
        {
            EngSt = OFF;
            BinSt = OFF;
        }
        else
        {
            EngSt = ERR;
            BinSt = ERR;
            IntSvcSync |= ISS_NEWERR;       // Set ERR flag as we read ERR state from LOG. We don't need to write this state to log once again
        }

        addrl += 4;
        if (STATUSbits.C)
            addrh += 1;

        RXbyte (addrh, addrl, &d);
        if (0b10000000 & d)
            TEB = TEB_HIGH;
        else
            TEB = TEB_LOW;

        addrl += 1;
        if (STATUSbits.C)
            addrh += 1;

        RXbyte (addrh, addrl, &d);

    }
    if (addrh == 0xFF)
    {
        addrh = 0;
        addrl = 0;
    }
}

void EngBinState (void)
{

    if ((PinState & (1 << CN_23)) && (BinSt == ERR || EngSt == ERR))    // ASR IG is ON and some Error
    {
        BinSt = OFF;
        EngSt = OFF;    // Set EngSt to OFF to be able to start EngChkTmr
    }

    if (BinSt == ERR || EngSt == ERR)       // Binar or Eng is in error state, exit
        return;

    // --------------------------------------------------------------------------
    // ENGINE
    // --------------------------------------------------------------------------

    if (PinState & (1 << CN_210))       // IG is ON
    {
        if (EngSt != ASRONOFF && EngSt != ASRTOSTOP)
        {
            ASRONTmr = 0xFF;    // STOP ASR timer as engine is already started
 //           NewASRTmr = 0xFF;

            if (!(IntSvcSync & ISS_AUTOST))
                IntSvcSync &= ~ISS_TEBHL;


            if (PinState & (1 << CN_23))    // ASR IG is ON
            {
                if (EngSt != ENGON && EngSt != ENGASRON && EngStartTmr == 0xFF)
                    NewASRTmr = 0x10;
                    //EngChkTmr = 0x10;   // Start EngChkTmr timer to check if Engine will work for at least EngChkTmr
                                        //  and only if Engine has been switched OFF more than EngStartTmr ago
                EngSt = ENGON;
                //if (EngChkTmr == 0x10)  // BinSt is just switched to ENGON state
                if (NewASRTmr == 0x10)
                    WriteLog();

//                ASRChkTmr = 0xFF;

                IntSvcSync &= ~ISS_AUTOST;      // NOT autostarted
                IntSvcSync &= ~ISS_TEBHL;       // NOT send TEBHL signal
            }
            else
            {
                if (EngSt == ASROFFON)   // We just started ASR, check if Engine will work for at lease ASRChkTmr
                    NewASRTmr = 0x10;
                    //ASRChkTmr = 0x10;
                
                EngSt = ENGASRON;

                //if (ASRChkTmr == 0x10)
                if (NewASRTmr == 0x10)
                    WriteLog();
            }
        }
    }
    else    // IG is OFF
    {
        if (EngSt != ASROFFON && EngSt != ASRTOST && EngSt != OFF)
        {
            EngStartTmr = 0x10;
//            NewASRTmr = 0xFF;
            EngSt = OFF;
        }
 //       if (ASRChkTmr != 0xFF)
 //       {
 //           ASRChkTmr = 0xFF;
 //           EngSt = ERR;
 //       }
//        if (EngChkTmr != 0xFF)
//        {
//            EngChkTmr = 0xFF;
//            EngSt = ERR;
//        }
    }


    // --------------------------------------------------------------------------
    // BINAR state
    // --------------------------------------------------------------------------

    if (PinState & (1 << CN_1416))  // Binar is ON or turning OFF
    {
        if (BinSt != BINONOFF && BinSt != BINTOSTOP && BinSt != BINON)
        {
            NewBinTmr = 0xFF;        // Turn OFF new state timer
            ASRONTmr = 0xFF;        // Turn OFF ASR timer
            BinSt = BINON;
            WriteLog ();
        }
    }
    else
    {
        if (BinSt != BINOFFON && BinSt != BINTOST && BinSt != OFF)  // Binar is not turning ON and not just got START command
        {
            if (BinSt == BINON)     // Binag was ON (if it turned OFF don't start engine)
                ASRONTmr = 16;      // If Engine already started this timer will be turned OFF
            NewBinTmr = 0xFF;
            BinSt = OFF;
            DO1819 = 0;
            WriteLog();
        }
    }

    // --------------------------------------------------------------------------
    // STOP command
    // --------------------------------------------------------------------------

    if (!(PinState & (1 << CN_21)))    // STOP command received
    {
        // If Eng had been started automatically OR everything OFF
//        if ((IntSvcSync & ISS_AUTOST) || (BinSt == OFF && EngSt == OFF))
        if (EngSt != ENGON)     // if ENG is started we woudn't respond to request
        {
            if (TEB >= 0x7F)
            {
                StateTest = ST_TEST;
            }
        }
/*        else
        {
            if (BinSt != OFF && DO1819 == 1)    // Binag is started by relay, so we can turn it OFF
            {
                NewBinTmr = 0x4;    // Should get
                BinSt = BINTOSTOP;
            }
            if (EngSt == ENGASRON)      // Eng is started by ASR, so we can turn it OFF
            {
                EngSt = ASRTOSTOP;
                NewASRTmr = 0x4;    // Should get
            }
        }*/
    }
    else
    {
        if (StateTest == ST_TEST)
        {
            StateTest = ST_NONE;
            __delay_ms(2000);
            DO27 = 1;
            __delay_ms(2000);
            DO27 = 0;
        }
        if (BinSt == BINTOSTOP)
        {
            DO1819 = 0;
            BinSt = BINONOFF;
            NewBinTmr = 0x4;
        }
        if (EngSt == ASRTOSTOP)
        {
            EngSt = ASRONOFF;
            NewASRTmr = 0x4;
            DO26 = 1;
            __delay_ms(500);
            DO26 = 0;
        }
    }

    // --------------------------------------------------------------------------
    // START command
    // --------------------------------------------------------------------------

    if (!(PinState & (1 << CN_25)))    // START command received
    {
        if (BinSt == OFF && EngSt == OFF &&
            ASRONTmr == 0xFF && NewBinTmr == 0xFF && NewASRTmr == 0xFF)       // Binar is OFF, Eng is OFF, ASRONTmr is OFF now
        {
            IntSvcSync &= ~ISS_AUTOST;  // Start by command (NOT automatically, by low voltage)

            if (TEB <= 0x7F)  // TEB <= 0x7F - Binar
            {
                BinSt = BINTOST;
                NewBinTmr = 0x4;    // Binar should be started within this time
            }
            else
            {
                EngSt = ASRTOST;
                NewASRTmr = 0x4;    // Binar should be started within this time
            }
//            WriteLog ();
        }
    }
    else        // Channel 2 is in 0 state
    {
        if (BinSt == BINTOST)   // Binar should be started
        {
            DO1819 = 1;
            NewBinTmr = 0x4;
            BinSt = BINOFFON;
            WriteLog ();
        }
        if (EngSt == ASRTOST)
        {
            EngSt = ASROFFON;
            NewASRTmr = 0x4;
            DO26 = 1;
            __delay_ms(500);
            DO26 = 0;
            WriteLog();
        }
    }

    // --------------------------------------------------------------------------
    // ASR -> ON by timer
    // --------------------------------------------------------------------------

    if (ASRONTmr == 0)      // ASR timer expired, start ASR
    {
        ASRONTmr = 0xFF;
        if (EngSt == OFF && BinSt == OFF)
        {
            EngSt = ASROFFON;
            DO26 = 1;
            __delay_ms(500);
            DO26 = 0;
            NewASRTmr = 0x4;     // Engine should start during this time
            WriteLog ();
        }
    }

    // --------------------------------------------------------------------------
    // Start by low voltage
    // --------------------------------------------------------------------------

    if (UbattTmr == UBT_TMRST && BinSt == OFF && EngSt == OFF &&
        ASRONTmr == 0xFF && NewBinTmr == 0xFF && NewASRTmr == 0xFF)
    {
        IntSvcSync |= ISS_AUTOST;   // We started by low Voltage (NOT command)
        if (TEB <= 0x7F)  // TEB <= 0x7F - Binar
        {
            DO1819 = 1;
            NewBinTmr = 0x4;
            BinSt = BINOFFON;
            WriteLog ();
        }
        else
        {
            EngSt = ASROFFON;
            NewASRTmr = 0x4;
            DO26 = 1;
            __delay_ms(500);
            DO26 = 0;
            WriteLog ();
        }
    }
}

void main(void) {

    //sec = 0, min = 15, hour = 4; day = 26;
    sec = 0;
    min = 0;
    hour = 0;
    day = 0;

    addrh = 0;
    addrl = 0x0;

    EngSt = OFF;
    BinSt = OFF;

    ASRONTmr = 0xFF;
    NewBinTmr = 0xFF;
    NewASRTmr = 0xFF;
//    ASRChkTmr = 0xFF;
//    EngChkTmr = 0xFF;
    EngStartTmr = 0xFF;
    UbattTmr = 0xFF;
    
    IntSvcSync = 0;
    StateTest = ST_NONE;

    PinState = 0;

    T1CON = 0;      // Set Timer1 to 0
    INTCON = 0;     // Set ALL interrupts to 0

    // -----------------------------------------
    //  Ports init & ADC init
    // -----------------------------------------

    PORTA = 0x0;        // Clear PortA
    PORTB = 0x0;
    PORTC = 0x0;

    // V 17 - RA2 = 1 (2/5)
    // 2, 3: RA4,5 - input for TM1 cristall
    TRISA = 0b11111111;
    WPUA = 0b00000000;
    IOCAbits.IOCA2 = 1; // RA2 IOC ON

    // V 10 - RB7 = 1, Binar start sence
    // V 11 - RB6 = 1 (SCL)
    // 12 - RB5 = 1 (2/1)
    // V 13 - RB4 = 1 (SDA)
    TRISB = 0b11111111;
    WPUB = 0b10000000;
    IOCBbits.IOCB5 = 1;     // RB5 IOC ON

    // 9 - RC7 - 
    // V 8 - RC6 = 0, (AN8) = 2/6, ASR-202m command
    // V 5 - RC5 = 0 1/18-19 - binar start
    // V 6 - RC4 = 0, dop canal sign
    // 7 - RC3 -
    // V 14 - RC2 = AN6
    // 15 - RC1 - 
    // ! 16 - RC0 (AN4) = 1, = 2/3
    TRISC = 0b10001111;

    OPTION_REGbits.nRABPU = 0;  // Weak Pull Ups enabled

    // ------------------------------------
    //  ADC init
    // ------------------------------------

    ANSELH = 0b00000010;    // AN9 = 1 (29)
    ANSEL = 0b11110000; // AN7 = 1 (210), AN6 = 1 (28), AN5 = 1 (24), AN4 = 1 (23)

    // ADC configuration
    ADCON1 = 0b00110000;     // Frc clock for convertion
    ADCON0 = 0b10011001;     // ADFM = 1 (right justify), VDD as ref0, AN6 selected, enable ADC

    PORTA = 0x0;        // Clear PortA
    PORTB = 0x0;
    PORTC = 0x0;


    // ------------------------------------
    // i2c init & pins init
    // ------------------------------------

    i2c_init (_I2C_PICADDR);
    PinCheckInit ();

    // ----------- TEST --------------------
    /*
    char d;

    char addrh = 0x1, addrl = 0x0;
    char dt[16], ret;

    char i = 0x2F, j = 0, k = 0, b = 0x10;
    for (j = 0; j < 16; j++)
    {
        dt[j] = 0x50 + j;
    }
    while(0)
    {
        __delay_ms(1);
 //       if (k >= 5)
 //           OutRet (0xAA, &k, 0);
        if (DI25)
        {
            if (k == 0)
            {
                if (i2c_start(0) || i2c_tx(0b10100000) || i2c_tx(addrh) || i2c_tx(addrl))
                    i2c_stop (0);
                else
                    k = 1;
            }
            else
            {
                if (i2c_tx(dt[0]))
                {
                    i2c_stop(0);
                    k = 0;
                }
                else
                    dt[0]++;
            }
            __delay_ms(200);
        }
        else
        {
            if (k == 1)
            {
                i2c_stop(0);
                k = 0;
            }
        }
    }


    ret = i2c_start(0);
//    OutRet (0x22, &ret, 0);

    if (1)
    {
    ret = i2c_tx(0b10100010);
//    OutRet (0x23, &ret, 0);

    ret = i2c_tx(0x1D);
//    OutRet (0x24, &ret, 0);
    ret = i2c_tx(0x2E);
//    OutRet (0x25, &ret, 0);

    }
    else
    {

    ret = i2c_tx(0b10100011);
    OutRet (0x23, &ret, 0);

    if ((ret = i2c_rx(1, &d)))
        OutRet (0x24, &ret, 0);
    else
        OutRet (0x24, &d, 0);
    if ((ret = i2c_rx(0, &d)))
        OutRet (0x25, &ret, 0);
    else
        OutRet (0x25, &d, 0);
    }
    ret = i2c_stop(0);
    OutRet (0x26, &ret, 0);
        while (1);
*/
/*    DO1819 = 0;
    while (1)
    {
        DO1819 = !DO1819;
        __delay_ms(2000);
    }
*/
    // ------------ TEST ENDS ---------------

    // ------------------------------------
    //  Copy I2C memory contents to EEPROM
    // ------------------------------------

    if (0)
    {
        char i, j, ret;
        char d[16];
        addrh = 0x0;
        addrl = 0;
        DO27 = 0;
        for (i = 0; i <= 16; i++)
        {
            //__delay_ms(1000);
            ret = RX16bytes (addrh, (i<<4), d);
            for (j = 0; j <= 15; j++)
            {
                
                if (ret > 0)
                {
                    OutRet ((i << 4) + j, &ret, 0);
                }
                else
                {
                    OutRet ((i << 4) + j, &d[j], 0);
                }
            }
            DO27 = 1;
            __delay_ms (2000);
            DO27 = 0;
        }
        DO27 = 0;
        __delay_ms (500);
        DO27 = 1;
        __delay_ms (500);
        DO27 = 0;
        while (1);
    }
        // Clear I2C memory

/*    char d[16];
    while (addrh != 0xFF)
    {
        TXNbytes (addrh, addrl, 16, d);
        while (TXWaitACK (20));
        addrl += 16;
        if (STATUSbits.C)
            addrh++;

    }*/

    

//    SearchStop ();   // Search for STOP byte
    TEB = TEB_HIGH;

    // ----------------------------------------------
    //   Timer init
    // ----------------------------------------------

    TMR1H = 0x0;
    TMR1L = 0x0;

    //  Timer1 Configuration
    T1CON = 0b00111110;         // T1CKPS<1:0> = 00 (1:8 Prescaler Value, for every 16 seconds)
                                // T1OSCEN<3> = 1 (1 = LP oscillator is enabled for Timer1 clock)
                                // T1SYNC<2> = 0 - Synchronize external clock input
                                // TMR1CS<1> = 1 - External clock from T1CKI pin (on the rising edge)
                                // TMR1ON = 1 (1 = Enables Timer1)
                                // RA4,5 cannot be used
    __delay_ms (1000);
    T1CONbits.TMR1ON = 1;
    __delay_ms (1000);


    // ---------------------------------------
    //   Loacal vars init
    // ---------------------------------------

    char BINtmr = 0, i2cHoldTmr = 0;    // BINtmr - how many times we'll write to log after identifing ASROFFON EngSt
                                        // i2cHoldTmr - if we got first bit = 1 in i2cCmd, we'll not count any timers in main cycle till i2cHoldTmr doen't expire
    unsigned char i2cCmd = _I2C_NOCMD;
    unsigned char i2cEEPROMAddr = 0x0;      // Address in EEPROM to read

    // --------------------------------------
    //   Interrupts init just before main cycle
    // --------------------------------------

    PIR1 = 0x0;
    PIR2 = 0x0;
    PIE1 = 0b00001001;  // ADIE<6> = 1: ADC interrupt enable,
                        // SSPIE<3> = 1: SSP (I2C) module recieve complete interrupt
                        // TMR1IE<0> = 1: Timer1 Overflow Interrupt Enable bit
    INTCON = 0b11001000;    // <7> = GIE, <6> = PEIE, <3> = RABIE


    char ret;
    while (1)
    {
        ToCheck = 0b11111111;

        while (ToCheck)
        {
            StartCheck (0);
            CheckRes();
        }

        if (!(min == 0 && hour == 0))
            EngBinState ();

        WriteLog ();

        OutRet (0x31, &EngSt, 0);
        OutRet (0x32, &BinSt, 0);
//        OutRet (0x33, &ASRONTmr, 0);
//        OutRet (0x34, &NewBinTmr, 0);
 //       OutRet (0x35, &NewASRTmr, 0);
//        OutRet (0x36, &ASRChkTmr, 0);
        OutRet (0x41, &Ubatt, 0);
        OutRet (0x42, &Teng, 0);
//        OutRet (0x43, &IntSvcSync, 0);
//        OutRet (0x44, &ASRtimes, 0);
//        OutRet (0x45, &EngChkTmr, 0);
//        OutRet (0x46, &EngStartTmr, 0);
//

   /*     if (PIR1bits.SSPIF)     // Something received on I2C bus
        {
            PIR1bits.SSPIF = 0;
            if (SSPSTATbits.R_W)    // Master wants to receive, READ (we WRITE to master)
            {
                if (!SSPSTATbits.D_nA)      // First byte is address
                {
                    char i2cAddr = SSPBUF;
                }
                //SSPBUF = ;  // Load data
                SSPCONbits.CKP = 1;
            }
            else                        // Master wants to transmit (we READ from Master)
            {
                if (SSPSTATbits.BF)
                {
                    if (!SSPSTATbits.D_nA)  // First byte is address
                    {
                        char i2cAddr = SSPBUF;
                        i2cCmd = _I2C_NOCMD;    // Clear command to prepare for receiving command from Master
                    }
                    else
                    {
                        switch(i2cCmd & 0b01111111)
                        {
                            case _I2C_NOCMD:
                                i2cCmd = SSPBUF;    // Got first byte of data, command
                                break;
                            case _I2C_EEPROMRW:
                                if (i2cEEPROMAddr == 0)
                                    i2cEEPROMAddr = SSPBUF;
                                else
                                {
                                    char ret = SSPBUF;
                                    OutRet (i2cEEPROMAddr, &ret, 0);
                                    i2cCmd = _I2C_NOCMD;
                                    i2cEEPROMAddr = 0x0;
                                }
                                break;
                        }
                    }
                }
            }
        }*/


        // ------------------------------------
        //  TIMER subroutines
        // ------------------------------------

        if (IntSvcSync & ISS_NEWSEC)
        {
            IntSvcSync &= ~ISS_NEWSEC;

            if (sec > 59)
            {
                min++;
                sec -= 60;
                if (min == 60)
                {
                    hour++;
                    min = 0;
                    // Write to LOG every hour
                    if (hour == 24)
                    {
                        day++;
                        hour = 0;
                    }
                    if (hour & 0b00000001)  // We will write ro log only on odd hours
                        WriteLog ();
                }

                // We will write to LOG every 3 minutes when BINAR is ON
                if (BinSt == BINON)
                {
                    if (BINtmr == 6)
                    {
                        BINtmr = 0;
                        WriteLog ();
                    }
                    else
                        BINtmr++;
                }
                else
                    BINtmr = 0;

            }

            // --------------
            // !!! TIMERS !!!
            // --------------
            if (NewBinTmr != 0xFF)
            {
                NewBinTmr--;
                if (NewBinTmr == 0)
                {
                    BinSt = ERR;
                    NewBinTmr = 0xFF;
                }
            }
            if (NewASRTmr != 0xFF)
            {
                NewASRTmr--;
                if (EngSt == OFF)   // Engine is turned OFF and timer still not epired
                {
                    EngSt = ERR;
                    NewASRTmr = 0xFF;
                }
                if (NewASRTmr == 0)
                {
                    if (EngSt != ENGON && EngSt != ENGASRON)
                        EngSt = ERR;
                    NewASRTmr = 0xFF;
                }
            }

            if (ASRONTmr != 0xFF)
                ASRONTmr--;

//            if (ASRChkTmr != 0xFF)
//                ASRChkTmr--;

//            if (EngChkTmr != 0xFF)
//                EngChkTmr--;

            if (EngStartTmr != 0xFF)
                EngStartTmr--;

            if (BinSt == OFF && EngSt == OFF &&
                ASRONTmr == 0xFF && NewBinTmr == 0xFF && NewASRTmr == 0xFF)// && ASRChkTmr == 0xFF)
            {
                if (Ubatt > 0x80 && Ubatt <= 0x93)   // 0x92 = ~11.28 V or lower.
                    // 0x92 is 0x49 previously, when we used (ADRESH << 6) | (ADRESL >> 2) to get voltage
                    // 0x80 (0x40 previously) = 10V, lowest voltage. If below, we won't try to start
                {   
                    if (UbattTmr > UBT_TMRST)
                    {
                        UbattTmr--;
                    }
                }
                else
                {
                    if (UbattTmr != 0xFF)
                    {
                        UbattTmr++;
                    }
                }
            }
            else
            {
                if (UbattTmr != 0xFF)
                {
                    UbattTmr = 0xFF;
                }
            }

            // Check Eng Temp and increase/decrease TEB
            //if (Teng < 0x69)      // Teng > 0,5 and Teng < 5,1 V
            {
                if (Teng < 0xEE)        // 0xEC (0xFB previouslvy, when we used (ADRESH << 6) | (ADRESL >> 2) to get Tb = 2.83V, or lower, so air is hot
                                        // 0xEB from 09.03.14
                                        // 0xEE from next program cycle
                {
                    if (TEB < TEB_HIGH)
                        TEB++;
                }
                else
                {
                    if (TEB > TEB_LOW)
                    {
                        if (TEB == TEB_HIGH && !(IntSvcSync & ISS_TEBHL) &&
                            EngSt == OFF && BinSt == OFF)        // Send signal to brelok that T is going low and this is last chance to start Engine without Binar
                        {
                            //OutRet (0x19, &IntSvcSync, 0);
                            DO27 = 1;
                            __delay_ms(1000);
                            DO27 = 0;
                            IntSvcSync |= ISS_TEBHL;
                            WriteLog ();
                        }
                        TEB--;
                    }
                }
            }

            // ----------------
            // !!!TIMER ENDS!!!
            // ----------------

        }       
        // ------------------
        // End New 16 Seconds
        // ------------------

     /*   if (EngSt == ENGASRON)
        {
            if (ASRtimes != 0xFF)
            {
                WriteLog ();
                ASRtimes--;
            }
        }
        else
            ASRtimes = 0xFF;

        if (EngSt == ENGASRON && (IntSvcSync & ISS_NEWASRON))
        {
            ASRtimes = 0x9;
        }

        if (EngSt == ASROFFON)      // Delay for 1 second and write to log 10 times
            IntSvcSync |= ISS_NEWASRON;
        else
            IntSvcSync &= ~ISS_NEWASRON;
*/
        // ------------------------------------------
        // Urgently wirte to log ERR state
        // ------------------------------------------

        if (EngSt == ERR || BinSt == ERR)
        {
            DO27 = 0;
            DO26 = 0;
            DO1819 = 0;
            if (!(IntSvcSync & ISS_NEWERR))   // Something are in ERR state - Write to LOG
            {
                WriteLog ();
                IntSvcSync |= ISS_NEWERR;
            }
        }
        else        // Everithing works fine - Write to LOG
        {
            if ((IntSvcSync & ISS_NEWERR))
            {
                WriteLog ();
                IntSvcSync &= ~ISS_NEWERR;
            }
        }

        if (!ToCheck)  // We can sleep
        {
                
            if (0)
            {
                asm ("sleep");
            }
            else
            {
               // WriteLog ();
                __delay_ms (2000);
                sec += 16;
                IntSvcSync |= ISS_NEWSEC;
            }
        }
    }
}

