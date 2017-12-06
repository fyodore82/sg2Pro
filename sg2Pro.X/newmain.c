/* 
 * File:   newmain.c
 * Author: fedor
 *
 * Created on 30 ??????? 2012 ?., 14:35
 */

#include <stdio.h>
#include <stdlib.h>
//#include <pic16f690.h> 
//#include <htc.h>
#include "../../i2c/i2c_my.h"
#include "PinsCheck.h"
#include "EngBinSt.h"
#include "EEPROMfunc.h"
#include "ResetReasons.h"
#include <xc.h>
#include "../../i2c/i2cCmds.h"
//#include <.\i2c.h>

// PIC16F690 Configuration Bit Settings

// CONFIG
//__CONFIG(FOSC_INTRCIO & WDTE_OFF & PWRTE_ON & MCLRE_ON & CP_OFF & CPD_OFF & BOREN_OFF & IESO_ON & FCMEN_OFF);


#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON      // Brown-out Reset Selection bits (BOR disabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode is enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)

#define _I2C_PICADDR    0b01000010

#define _XTAL_FREQ 4000000


// Teng in 0x10 - 0x14
// Ubatt in 0x15 - 0x19
//const unsigned int abc1 @ 0x2110 = 0x5566;
//const unsigned int abc2 @ 0x2112 = 0x5566;
//const unsigned int abc3 @ 0x2114 = 0x5566;
//const unsigned int abc4 @ 0x2116 = 0x5566;
//const unsigned int abc5 @ 0x2118 = 0x5566;
__EEPROM_DATA(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
__EEPROM_DATA(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
__EEPROM_DATA(0xD6, 0xD6, 0xD6, 0xD6, 0xD6, 0x94, 0x94, 0x94);
__EEPROM_DATA(0x94, 0x94, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);



unsigned char addrh, addrl;

unsigned char day, sec, min, hour;

unsigned char i2cCmd, i2cEEPROMAddr, i2cSSPIFTmr, i2cHoldTmr;

// TEngTemp - Engine temp, if lower - start binar
// UbattMin - Minimum battery voltage, start engine if lower
unsigned char TEngTemp, UbattMin;


void I2CSlaveSR (void);

/*
 * 
 */
/*
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
        __delay_ms(100);     // Safe Delay - _dealy(100000), which is equal to 100 ms on 4MHz
    }
    INTCONbits.GIE = gie;            // Enable Interrupts back
}
*/


// stack = 0

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
        //DO1819 = !DO1819;
        sec += 16;
        IntSvcSync |= ISS_NEWSEC;
    }
    if (PIR1bits.SSPIF)
    {
        I2CSlaveSR ();
//        i2cSSPIFTmr = 0x5;
    }
}

// stack = 3 (TXNBytes)
void WriteLog (unsigned char Event, unsigned char ResetReason)        // Writes to I2C memory
// start format:
//  0b1------- - PINS format
//  0b01010101 - STOP byte
//  0b001x1--- - OFF format = 0x38 OR 0x28
//  0b000x1--- - ERR format = 0x18 OR 0x08

//  0b00000111 - Reset detected
//  0b0000001- - New ERR / ALL OK = 0x02
//  0b00000001 - TEB Low -> High (or High -> Low)
//  0b00000000 - Start by low voltage
{
    char d[6];

    if (Event == WL_NOLOG)
        return;

    d[3] = Ubatt;
    d[4] = Teng;
    d[5] = 0b01010101;  // STOP byte

    if (Event == WL_DEFAULT)
    {
        Event = WL_PINS;
        if (BinSt == OFF && EngSt == OFF)
            Event = WL_OFF;
        if ((BinSt&ERR_BINPATTERN) || (EngSt&ERR_ENGPATTERN))
            Event = WL_ERR;
    }

    switch (Event)
    {
        case WL_RESETREASON:
            d[0] = 0b00000111;
            d[1] = ResetReason;
            d[2] = ResetReason;
            d[3] = 0x0;
            d[4] = 0x55;
            break;
        case WL_OFF:
            // Format when everything OFF or ERR
            //  start | TEB | start | day | hour | ENGST | BINST | Ubatt | Teng
            //    3   |  1  |   1   |  8  |  5   |   3   |   3   |   8   |   8
            d[0] = 0b00101000 | ((TEB >= 0x80) << 4) | (day >> 5);
            d[1] = (day << 3) | (hour >> 2);
            d[2] = (hour << 6) | ((EngSt & 0b00000111) << 3) | (BinSt & 0b00000111);
            break;
        case WL_ERR:
            d[0] = 0b00001000 | ((TEB >= 0x80) << 4) | (day >> 5);
            d[1] = (day << 3) | (hour >> 2);
            d[2] = (hour << 6) | ((EngSt & 0b00000111) << 3) | (BinSt & 0b00000111);
            break;
        case WL_NEWERR:
        case WL_ALLOK:
            // NEW ERR / ALL OK format
            //  start | hour | min | ENGST | BINST | Ubatt | Teng
            //    7   |  5   |  6  |   3   |   3   |   8   |   8
            d[0] = 0b00000010 | (hour >> 4);
            d[1] = (hour << 4) | (min >> 2);
            d[2] = (min << 6) | ((EngSt & 0b00000111) << 3) | (BinSt & 0b00000111);
            break;
        case WL_PINS:
            // PINS format when something ON
            //  start | hour | min | ENGST | BINST | pins | Ubatt | Teng
            //    1   |  5   |  6  |   3   |   3   |   6  |   8   |   8
            d[0] = 0b10000000 | (hour << 2) | (min >> 4);
            d[1] = (min << 4) | ((EngSt & 0b00000111) << 1) | ((BinSt & 0b00000111) >> 2);
            // Pins bits: X29(Teng-X);210;25;X24-X(DO1819);23;14-16;21;X28(Ubatt, X)
            d[2] = ((BinSt & 0b00000111) << 6) | ((PinState & 0b01101110) >> 1) | (DO1819 << 4);
            break;
        case WL_STARTBYLOWVOLTAGE:
            // PINS format when something ON
            //  start | hour | min | ENGST | BINST | pins | 0b0 | Teng
            //    8   |  5   |  6  |   3   |   3   |   6  |  1  |   8
            d[0] = 0b00000000;
            d[1] = (hour << 3) | (min >> 3);
            d[2] = (min << 5) | ((EngSt & 0b00000111) << 2) | ((BinSt & 0b00000111) >> 1);
            // Pins bits: X29(Teng-X);210;25;X24-X(DO1819);23;14-16;21;X28(Ubatt, X)
            d[3] = ((BinSt & 0b00000111) << 7) | (PinState & 0b01101110) | (DO1819 << 4);
            break;
        case WL_TEBHL:
        case WL_TEBLH:
            //  start | hour | min | New TEB | 0b0000 | Ubatt | Teng
            //    8   |  5   |  6  |    1    |    4   |   8   |   8
            d[0] = 0b00000001;
            d[1] = (hour << 3) | (min >> 3);
            d[2] = (min << 5) | ((Event == WL_TEBHL ? 0 : 1) << 4);
            break;

    }
    char i = 3;
    while (TXNbytes (addrh, addrl, 6, d) && i > 0)      // stack = 2
    {
        __delay_ms(10);
        i--;
    }
    addrl += 5;
    if (STATUSbits.C)
        addrh += 1;
}

// stack = 3 (RXByte)
void SearchStop (void)  // Search for stop byte
{
        unsigned char d = 0xFF;
        unsigned char addrhs = 0, addrls = 0;
        addrh = 0;
        addrl = 0;
        while (d != 0b01010101 && addrh != 0xFF)
        {
            addrl += 5;
            if (STATUSbits.C)
                addrh += 1;

            RXbyte (addrh, addrl, &d);

            if ((d & 0b11001000) == 0b00001000)
            {
                addrhs = addrh;
                addrls = addrl;
            }
        }

        if (d != 0b01010101)
        {
            addrh = 0;
            addrl = 0;
        }
        else
        {
            if (addrhs != 0 || addrls != 0)
            {
                RXbyte (addrhs, addrls, &d);

                if (0b00010000 & d)
                    TEB = TEB_HIGH;
                else
                    TEB = TEB_LOW;

                if (0b00100000 & d)
                {
                    BinSt = OFF;
                    EngSt = OFF;
                }
                else
                {
                    BinSt = ERR_BINPATTERN;
                    EngSt = ERR_ENGPATTERN;
                }

                day = (d & 0b00000111) << 5;

                addrls += 1;
                if (STATUSbits.C)
                    addrhs += 1;
                RXbyte (addrhs, addrls, &d);

                day |= ((d & 0b11111000) >> 3);
                hour = (d & 0b00000111) << 2;

                addrls += 1;
                if (STATUSbits.C)
                    addrhs += 1;
                RXbyte (addrhs, addrls, &d);

                hour |= (d & 0b11000000) >> 6;
 //   OutRet (0x33, &addrhs, 0);
 //   OutRet (0x34, &addrls, 0);
//    OutRet (0x35, &day, 0);
//    OutRet (0x36, &hour, 0);

                hour += 2;
                if (hour == 24 || hour == 25)
                {
                    hour -= 24;
                    day++;
                }
                min = 1;

//    OutRet (0x31, &addrh, 0);
//    OutRet (0x32, &addrl, 0);

            }
        }
}

// stack = 4 (WriteLog)
unsigned char EngBinState (void)
{
    unsigned char WrLogEvent = WL_NOLOG;

    if (((PinState & (1 << CN_210)) && (PinState & (1 << CN_23))) && ((BinSt&ERR_BINPATTERN) || (EngSt&ERR_ENGPATTERN)))    // ASR IG is ON and some Error
    {
        BinSt = OFF;
        EngSt = OFF;    // Set EngSt to OFF to be able to start EngChkTmr
    }

    if ((BinSt&ERR_BINPATTERN) || (EngSt&ERR_ENGPATTERN))       // Binar or Eng is in error state, exit
        return (WrLogEvent);

    // --------------------------------------------------------------------------
    // ENGINE
    // --------------------------------------------------------------------------

    if (PinState & (1 << CN_210))       // IG is ON
    {
        if (EngSt != ASRONOFF && EngSt != ASRTOSTOP)
        {
            ASRONTmr = 0xFF;    // STOP ASR timer as engine is already started

            if (!(IntSvcSync & ISS_AUTOST))
                IntSvcSync &= ~ISS_TEBHL;


            if (PinState & (1 << CN_23))    // ASR IG is ON (It means that ENG is ON)
            {
                if (EngSt != ENGON && EngSt != ENGASRON && EngStartTmr == 0xFF)
                {
                    NewASRTmr = 0x10;
                                        // Start EngChkTmr timer to check if Engine will work for at least EngChkTmr
                                        //  and only if Engine has been switched OFF more than EngStartTmr ago
                    EngSt = ENGON;      // WriteLog will write correct EngSt
 //                   WriteLog(0, 0);    // stack = 3
                    WrLogEvent = 0;
                }
                EngSt = ENGON;

                IntSvcSync &= ~ISS_AUTOST;      // NOT autostarted
                IntSvcSync &= ~ISS_TEBHL;       // NOT send TEBHL signal
            }
            else
            {
                if (EngSt == ASROFFON)   // We just started ASR, check if Engine will work for at lease ASRChkTmr
                {
                    NewASRTmr = 0x10;
                    EngSt = ENGASRON;
                    WrLogEvent = 0;
                    //WriteLog(0, 0);
                }
                EngSt = ENGASRON;
            }
        }
    }
    else    // IG is OFF
    {
        if (EngSt != ASROFFON && EngSt != ASRTOST && EngSt != OFF)
        {
            EngStartTmr = 0x10;
            EngSt = OFF;
        }
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
            WrLogEvent = 0;
            //WriteLog (0, 0);
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
            WrLogEvent = 0;
            //WriteLog(0, 0);
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

    if (!(PinState & (1 << CN_25)))    // START command received (CN_25 == 0
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
            WrLogEvent = 0;
            //WriteLog (0, 0);
        }
        if (EngSt == ASRTOST)
        {
            EngSt = ASROFFON;
            NewASRTmr = 0x4;
            DO26 = 1;
            __delay_ms(500);
            DO26 = 0;
            WrLogEvent = 0;
            //WriteLog(0, 0);
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
            WrLogEvent = 0;
            //WriteLog (0, 0);
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
            WrLogEvent = WL_STARTBYLOWVOLTAGE;
            //WriteLog (WL_STARTBYLOWVOLTAGE, 0);
        }
        else
        {
            EngSt = ASROFFON;
            NewASRTmr = 0x4;
            DO26 = 1;
            __delay_ms(500);
            DO26 = 0;
            WrLogEvent = WL_STARTBYLOWVOLTAGE;
//            WriteLog (WL_STARTBYLOWVOLTAGE, 0);
        }
    }
    return (WrLogEvent);
}
// stack = 0
void I2CReset (void)
{
    SSPCONbits.SSPEN = 0;
    __delay_ms(10);
    i2c_init(_I2C_PICADDR);
}

// stack = 1 (OutRet)
void I2CSlaveSR (void)
{
    char x;

    if (PIR1bits.SSPIF == 0)
        return;

    SSPCONbits.SSPOV = 0;

    if (SSPSTATbits.R_W)    // Master wants to receive, READ (we WRITE to master)
    {
/*        if (!SSPSTATbits.D_nA)      // First byte is address
        {
            x = SSPBUF;
        }
        else
        {*/
            if (SSPCONbits.CKP == 0)    // Clock stretch after address match and we TRANSMIT (WRITE to master)
            {
                switch(i2cCmd)
                {
                    case _I2C_EEPROMRW:
                        OutRet (i2cEEPROMAddr, &x, 1); // Use i2cCmd to get data as we won't use it any more
                        SSPBUF = x;
                        i2cEEPROMAddr = 0x0;
                        break;
                    case _I2C_DAY:
                        SSPBUF = day;
                        break;
                    case _I2C_HOUR:
                        SSPBUF = hour;
                        break;
                    case _I2C_MIN:
                        SSPBUF = min;
                        break;
                    case _I2C_SEC:
                        SSPBUF = sec;
                        break;
                    case _I2C_ADDRH:
                        SSPBUF = addrh;
                        break;
                    case _I2C_ADDRL:
                        SSPBUF = addrl;
                        break;
                    case _I2C_UBATT:
                        SSPBUF = Ubatt;
                        break;
                    case _I2C_TENG:
                        SSPBUF = Teng;
                        break;
                    case _I2C_PINSTATE:
                        SSPBUF = PinState;
                        break;
                    case _I2C_BINST:
                        SSPBUF = BinSt;
                        break;
                    case _I2C_ENGST:
                        SSPBUF = EngSt;
                        break;
                    case _I2C_INTSVCSYNC:
                        SSPBUF = IntSvcSync;
                        break;
                    case _I2C_NEWBINTMR:
                        SSPBUF = NewBinTmr;
                        break;
                    case _I2C_NEWASRTMR:
                        SSPBUF = NewASRTmr;
                        break;
                    case _I2C_ASRONTMR:
                        SSPBUF = ASRONTmr;
                        break;
                    case _I2C_ENGSTARTTMR:
                        SSPBUF = EngStartTmr;
                        break;
                    case _I2C_UBATTTMR:
                        SSPBUF = UbattTmr;
                        break;
                    case _I2C_TEB:
                        SSPBUF = TEB;
                        break;
                    case _I2C_DI21:
                        SSPBUF = DI21;
                        break;
                    case _I2C_DI23:
                        SSPBUF = DI23;
                        break;
                    case _I2C_DI24:
                        SSPBUF = DI24;
                        break;
                    case _I2C_DI25:
                        SSPBUF = DI25;
                        break;
                    case _I2C_AI28:
                        SSPBUF = AI28;
                        break;
                    case _I2C_AI29:
                        SSPBUF = AI29;
                        break;
                    case _I2C_DI210:
                        SSPBUF = DI210;
                        break;
                    case _I2C_DI1416:
                        SSPBUF = DI1416;
                        break;
                    case _I2C_DO26:
                    case _I2C_DO27:
                    case _I2C_DO1819:
                        break;
                    case _I2C_UTRHLD:
                        SSPBUF = UbattMin;
                        break;
                    case _I2C_TTRHLD:
                        SSPBUF = TEngTemp;
                        break;
                    case _I2C_CMDREBOOT:
                    default:
                        SSPBUF = 0x0;    // Command unrecognized - send 0x0
                        break;
                }
                SSPCONbits.CKP = 1;
                i2cCmd = _I2C_NOCMD;
            
            }   // if SSPCONbits.CKP == 0
        //}   // else !SSPSTATbits.D_nA
    }   // if SSPSTATbits.R_W
    else                        // Master wants to transmit (we READ from Master)
    {
        if (SSPSTATbits.BF) // Always set for RECEPTION (READ from Master)
        {
            if (!SSPSTATbits.D_nA)  // First byte is address
            {
                x = SSPBUF;
                i2cCmd = _I2C_NOCMD;    // Clear command to prepare for receiving command from Master
            }
            else
            {
                switch(i2cCmd)
                {
                    case _I2C_DAY:
                        day = SSPBUF;
                        break;
                    case _I2C_HOUR:
                        hour = SSPBUF;
                        break;
                    case _I2C_MIN:
                        min = SSPBUF;
                        break;
                    case _I2C_SEC:
                        sec = SSPBUF;
                        break;
                    case _I2C_ADDRH:
                        addrh = SSPBUF;
                        break;
                    case _I2C_ADDRL:
                        addrl = SSPBUF;
                        break;
                    case _I2C_UBATT:
                        Ubatt = SSPBUF;
                        TEngUbattRW (UBATT_EEPROM, &Ubatt, 0);
                        break;
                    case _I2C_TENG:
                        Teng = SSPBUF;
                        TEngUbattRW (TENG_EEPROM, &Teng, 0);
                        break;
                    case _I2C_PINSTATE:
                    case _I2C_BINST:
                        BinSt = SSPBUF;
                        break;
                    case _I2C_ENGST:
                        EngSt = SSPBUF;
                        break;
                    case _I2C_INTSVCSYNC:
                        x = SSPBUF;
                        break;
                    case _I2C_NEWBINTMR:
                        NewBinTmr = SSPBUF;
                        break;
                    case _I2C_NEWASRTMR:
                        NewASRTmr = SSPBUF;
                        break;
                    case _I2C_ASRONTMR:
                        ASRONTmr = SSPBUF;
                        break;
                    case _I2C_ENGSTARTTMR:
                        EngStartTmr = SSPBUF;
                        break;
                    case _I2C_UBATTTMR:
                        UbattTmr = SSPBUF;
                        break;
                    case _I2C_TEB:
                        TEB = SSPBUF;
                        break;
                    case _I2C_DI21:
                    case _I2C_DI23:
                    case _I2C_DI24:
                    case _I2C_DI25:
                    case _I2C_AI28:
                    case _I2C_AI29:
                    case _I2C_DI210:
                    case _I2C_DI1416:
                        x = SSPBUF;
                        break;
                    case _I2C_DO26:
                        x = SSPBUF;
                        DO26 = x;
                        break;
                    case _I2C_DO27:
                        x = SSPBUF;
                        DO27 = x;
                        break;
                    case _I2C_DO1819:
                        x = SSPBUF;
                        DO1819 = x;
                        break;
                    case _I2C_UTRHLD:
                        x = SSPBUF;
                        UbattMin = x;
                        TEngUbattRW (UBATT_EEPROM, &x, 0);  // Read TEng treshold temp
                        break;
                    case _I2C_TTRHLD:
                        x = SSPBUF;
                        TEngTemp = x;    // 0xEC (0xFB previouslvy, when we used (ADRESH << 6) | (ADRESL >> 2) to get Tb = 2.83V, or lower, so air is hot
                        TEngUbattRW (TENG_EEPROM, &x, 0);  // Read TEng treshold temp
                        break;
                    case _I2C_CMDREBOOT:
                        x = SSPBUF;
                        break;
                    case _I2C_EEPROMRW:
                        if (i2cEEPROMAddr == 0x0)
                        {
                            i2cEEPROMAddr = SSPBUF;
                        }
                        else
                        {
                            x = SSPBUF;
                            OutRet (i2cEEPROMAddr, &x, 0);
                            i2cEEPROMAddr = 0x0;
                        }
                        break;
                    case _I2C_HOLDTMR:
                        i2cHoldTmr = SSPBUF;   // About 2 minutes
                        break;
                    default:
                        x = SSPBUF;
                        break;
                }

                if (i2cCmd == _I2C_NOCMD)
                {
                    i2cCmd = SSPBUF;    // Got first byte of data, command
                }
                else
                {
                    if (i2cEEPROMAddr == 0)       // On EEPROMRW we should get second byte as command
                    {
                        i2cCmd = _I2C_NOCMD;
                        //I2CReset();   // Reset I2C module in case of some error
                    }
                }
            } // else !SSPSTATbits.D_nA
        }   // if SSPSTATbits.BF
    }

    PIR1bits.SSPIF = 0;

}

// Stack = 4 (WriteLog)
unsigned char TmrRoutines (unsigned char *pBINtmr)
{
    unsigned char WrLogEvent = WL_NOLOG;

    if (!(IntSvcSync & ISS_NEWSEC))     // There is not new 16 seconds, return
        return (WrLogEvent);

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
                WrLogEvent = 0;//WriteLog (0, 0);
        }

        // We will write to LOG every 3 minutes when BINAR is ON
        if (BinSt == BINON)
        {
            if (*pBINtmr == 6)
            {
                *pBINtmr = 0;
                WrLogEvent = 0;//WriteLog (0, 0);
            }
            else
                *pBINtmr++;
        }
        else
            *pBINtmr = 0;

    }

    // --------------
    // !!! TIMERS !!!
    // --------------
    if (i2cHoldTmr != 0xFF)
    {
        i2cHoldTmr--;
    }
    else
    {
#ifndef __DEBUG         // if in DEBUG don't reset I2C module as we cannot send anything
        I2CReset();     // Reset I2C module every 16 seconds.
                        // To NOT reset, set i2cHoldTmr
#endif

        if (NewBinTmr != 0xFF)
        {
            NewBinTmr--;

            if (NewBinTmr == 0)
            {
                switch(BinSt)
                {
                    case BINTOST:
                        BinSt = ERR_BINTOST;
                        break;
                    case BINOFFON:
                        BinSt = ERR_BINOFFON;
                        break;
                    case BINON:
                        BinSt = ERR_BINON;
                        break;
                    case BINONOFF:
                        BinSt = ERR_BINONOFF;
                        break;
                    case BINTOSTOP:
                        BinSt = ERR_BINTOSTOP;
                        break;
                    case OFF:
                        BinSt = ERR_BINOFF;
                        break;
                    default:
                        BinSt = ERR_BINPATTERN;
                        break;
                }
                NewBinTmr = 0xFF;
            }
        }
        if (NewASRTmr != 0xFF)
        {
            NewASRTmr--;

            if (EngSt == OFF)   // Engine is turned OFF and timer still not expired
            {
                EngSt = ERR_ASRTMR_ENGOFF;
                NewASRTmr = 0xFF;
            }

            if (NewASRTmr == 0)
            {
                if (EngSt != ENGON && EngSt != ENGASRON)
                {
                    switch(EngSt)
                    {
                        case ASRTOST:
                            EngSt = ERR_ASRTOST;
                            break;
                        case ASROFFON:
                            EngSt = ERR_ASROFFON;
                            break;
                        case ENGASRON:
                            EngSt = ERR_ASRON;
                            break;
                        case ASRONOFF:
                            EngSt = ERR_ASRONOFF;
                            break;
                        case ASRTOSTOP:
                            EngSt = ERR_ASRTOSTOP;
                            break;
                        case ENGON:
                            EngSt = ERR_ASRTMR_ENGON;
                            break;
                        case OFF:
                            EngSt = ERR_ASRTMR_ENGOFF;
                            break;
                        default:
                            EngSt = ERR_ENGPATTERN;
                            break;

                    }
                }
                NewASRTmr = 0xFF;
            }
        }

        if (ASRONTmr != 0xFF)
            ASRONTmr--;

        if (EngStartTmr != 0xFF)
            EngStartTmr--;

        if (i2cSSPIFTmr != 0xFF)
            i2cSSPIFTmr--;

        if (BinSt == OFF && EngSt == OFF &&
            ASRONTmr == 0xFF && NewBinTmr == 0xFF && NewASRTmr == 0xFF) // && ASRChkTmr == 0xFF)
        {
            if (Ubatt > 0x80 && Ubatt <= UbattMin)
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
        if (Teng < TEngTemp)
        {
            if (TEB < TEB_HIGH)
            {
                TEB++;
                if (TEB == TEB_HIGH && TEB != TEBPrev)
                {
                    WrLogEvent = WL_TEBLH;//WriteLog (WL_TEBLH, 0);
                    TEBPrev = TEB_HIGH;
                }
            }
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
                }

                TEB--;
                if (TEB == TEB_LOW && TEB != TEBPrev)
                {
                    WrLogEvent = WL_TEBHL;//WriteLog (WL_TEBHL, 0);
                    TEBPrev = TEB_LOW;
                }
            }
        }
    }
    return (WrLogEvent);
    // ----------------
    // !!!TIMER ENDS!!!
    // ----------------

}
// Stack = 0
char DetectReset (void)
{
    if (!PCONbits.nPOR && STATUSbits.nTO && STATUSbits.nPD)     // POR
        return POR;
    if (!PCONbits.nBOR && STATUSbits.nTO && STATUSbits.nPD)     // BOR
        return BOR;
    if (!STATUSbits.nTO && !STATUSbits.nPD)     // WDT wake up
        return WDTWAKEUP;
    if (STATUSbits.nTO && !STATUSbits.nPD)      // MCLR reset during sleep
        return MCLRSLEEP;
    if (!STATUSbits.nTO)
        return WDTRESET;
    return MCLRRESET;

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
    EngStartTmr = 0xFF;

    UbattTmr = 0xFF;
    IntSvcSync = 0;
    StateTest = ST_NONE;

    CheckedNow = CN_NONE;
    PinState = 0;
    NewState = 0;

    T1CON = 0;      // Set Timer1 to 0
    INTCON = 0;     // Set ALL interrupts to 0


    // -----------------------------------------
    //  Ports init
    // -----------------------------------------

    PORTA = 0x0;        // Clear PortA
    PORTB = 0x0;
    PORTC = 0x0;

    // 17 - RA2 = 1 (210) - input
    // 2, 3: RA4,5 - input for TM1 cristall
    TRISA = 0b11111111;
    WPUA = 0b00000000;
    //IOCAbits.IOCA2 = 1; // RA2 IOC ON

    // 10 - RB7 = 1 (21)
    // 11 - RB6 = 1 (SCL)
    // 12 - RB5 = 1 (25)
    // 13 - RB4 = 1 (SDA)
    TRISB = 0b11111111;
    WPUB = 0b00000000;
    IOCBbits.IOCB7 = 1;     // RB7 IOC ON
    IOCBbits.IOCB5 = 1;     // RB5 IOC ON

    // 9 - RC7 = AN9 = 1 (29) - Teng
    // 8 - RC6 = 1 (14-16) - Binar start sense
    // 5 - RC5 = 1 (23) - ASR +12
    // 6 - RC4 = 1 (24) - ACC
    // 7 - RC3 = 0 (18-19) - Binar start
    // 14 - RC2 = 0 (27) - dop kanal,
    // 15 - RC1 = 0 (26) - Engine start
    // 16 - RC0 = AN4 = 1 (28) - Ubatt
    TRISC = 0b11110001;
    //OPTION_REGbits.nRABPU = 0;  // Weak Pull Ups enabled

    // ------------------------------------
    //  ADC init
    // ------------------------------------

    ANSEL = 0b00010000;     // AN4 = 1 (28)
    ANSELH = 0b00000010;    // AN9 = 1 (29)

    // ADC configuration
    ADCON1 = 0b00110000;     // Frc clock for convertion
    ADCON0 = 0b10010001;     // ADFM = 1 (right justify), VDD as ref0, AN4 selected, enable ADC

    PORTA = 0x0;        // Clear PortA
    PORTB = 0x0;
    PORTC = 0x0;

    // Set WDT prescaler
    asm ("CLRWDT");
    TMR0 = 0;
    OPTION_REGbits.PSA = 1;
    asm ("CLRWDT");
    OPTION_REGbits.PS = 0b000;
    OPTION_REGbits.PS = 0b101;  // WDT prescaler 1:32
    WDTCON = 0b00010111;        // Turn on WDT with 1:65536 postscaler
                                // WDT elapses after 67 seconds

    __delay_ms(1000);
    if (TEngUbattRW (UBATT_EEPROM, &UbattMin, 1))  // Read TEng treshold temp
    {   // Error - use standard Ubatt
        UbattMin = 0x93;    // 0x92 = ~11.28 V or lower.
                        // 0x92 is 0x49 previously, when we used (ADRESH << 6) | (ADRESL >> 2) to get voltage
                        // 0x80 (0x40 previously) = 10V, lowest voltage. If below, we won't try to start
    }
    if (TEngUbattRW (TENG_EEPROM, &TEngTemp, 1))  // Read TEng treshold temp
    {   // Error - use standard Temp
        TEngTemp = 0xd6;    // 0xEC (0xFB previouslvy, when we used (ADRESH << 6) | (ADRESL >> 2) to get Tb = 2.83V, or lower, so air is hot
                        // 0xEB from 09.03.14
                        // 0xEE from next program cycle
    }
    //OutRet (0x33, &TEngTemp, 0);
    // ------------------------------------
    // i2c init
    // ------------------------------------

    i2c_init (_I2C_PICADDR);

    // Search for STOP byte
    TEB = TEB_HIGH;

    SearchStop();

    TEBPrev = TEB;

    // ----------------------------------------------
    //  Last reset detection
    // ----------------------------------------------
    char ResetReason = DetectReset();
    WriteLog (WL_RESETREASON, ResetReason);

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

    unsigned char BINtmr = 0;   // BINtmr - how many times we'll write to log after identifing ASROFFON EngSt
                                        
    i2cCmd = _I2C_NOCMD;
    i2cEEPROMAddr = 0x0;        // Address in EEPROM to read
//    i2cSSPIFTmr = 0xFF;
    i2cHoldTmr = 0xFF;          // i2cHoldTmr - if we got first bit = 1 in i2cCmd, we'll not count any timers in main cycle till i2cHoldTmr doen't expire

    // --------------------------------------
    //   Interrupts init just before main cycle
    // --------------------------------------

    PIR1 = 0x0;
    PIR2 = 0x0;
    PIE1 = 0b00001001;  // ADIE<6> = 1: ADC interrupt enable, SSPIE<3> = 1 - I2C interrupts, TMR1IE<0> = 1: Timer1 Overflow Interrupt Enable bit
    PIE1bits.SSPIE = 1;
    INTCON = 0b11001000;    // <7> = GIE, <6> = PEIE, <3> = RABIE

    while (1)
    {
        asm ("CLRWDT");
        ToCheck = 0b11111111;

        while (ToCheck)
        {
            StartCheck (0);
            CheckRes();
        }

       /* if (!(PinState & (1 << CN_25)))
        {
            DO27=1;
        }
        else
        {
            DO27=0;
        }

        if (!(PinState & (1 << CN_21)))
        {
            DO26=1;
        }
        else
        {
            DO26=0;
        }
        if (!(PinState & (1 << CN_1416)))
        {
            DO1819=1;
        }
        else
        {
            DO1819=0;
        }

    }
    {*/
        if (i2cHoldTmr == 0xFF)
        {
            unsigned char resbin = EngBinState ();
            WriteLog (resbin, 0);
        }
 //               OutRet (0x41, &UbattMin, 0);
 //       OutRet (0x42, &TEngTemp, 0);
/*        OutRet (0x30, &ToCheck, 0);
        OutRet (0x31, &EngSt, 0);
        OutRet (0x32, &BinSt, 0);
        OutRet (0x33, &ASRONTmr, 0);
        OutRet (0x34, &NewBinTmr, 0);
        OutRet (0x35, &NewASRTmr, 0);
        OutRet (0x41, &Ubatt, 0);
        OutRet (0x42, &Teng, 0);
        OutRet (0x43, &IntSvcSync, 0);
        OutRet (0x44, &PinState, 0);
        OutRet (0x46, &i2cHoldTmr, 0);
*/
        // ------------------------------------
        //  New 16 Seconds
        // ------------------------------------

        if (IntSvcSync & ISS_NEWSEC)
        {
            unsigned char restmr = TmrRoutines (&BINtmr);
            WriteLog (restmr, 0);
            IntSvcSync &= ~ISS_NEWSEC;
        }       
        // ------------------
        // End New 16 Seconds
        // ------------------

        // ------------------------------------------
        // Urgently wirte to log ERR state
        // ------------------------------------------

        if ((BinSt&ERR_BINPATTERN) || (EngSt&ERR_ENGPATTERN))
        {
            DO27 = 0;
            DO26 = 0;
            DO1819 = 0;
            if (!(IntSvcSync & ISS_NEWERR))   // Something are in ERR state - Write to LOG
            {
                WriteLog (WL_NEWERR, 0);
                IntSvcSync |= ISS_NEWERR;
            }
        }
        else        // Everithing works fine - Write to LOG
        {
            if ((IntSvcSync & ISS_NEWERR))
            {
                WriteLog (WL_ALLOK, 0);
                IntSvcSync &= ~ISS_NEWERR;
            }
        }

        if (!ToCheck/* && i2cSSPIFTmr == 0xFF*/)  // We can sleep
        {
            asm ("CLRWDT");
#ifndef __DEBUG
            asm ("sleep");
#elif __DEBUG
            // WriteLog (0);
            __delay_ms (1000);
            sec += 16;
            IntSvcSync |= ISS_NEWSEC;
#endif
         }
    }
}

