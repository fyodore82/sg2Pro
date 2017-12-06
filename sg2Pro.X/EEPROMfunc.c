#include "EEPROMfunc.h"
#include <xc.h>

// stack = 0
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

unsigned char TEngUbattRW (unsigned char StartAddr, char *Teng, char Read)
{
    char addr, lTeng, RTimes, EqTimes;
    if (!Read)      // We should WRITE
    {
        for (addr = StartAddr; addr < StartAddr + 5; addr++)
        {
            OutRet (addr, Teng, Read);
        }
    }
    else
    {
        for (RTimes = 5; RTimes > 0; RTimes--)
        {
            addr = StartAddr + RTimes - 1;
            OutRet (addr, Teng, Read);      // Read or write first
            lTeng = *Teng;
            EqTimes = 1;
            do
            {
                addr--;
                OutRet (addr, Teng, Read);
                EqTimes += (lTeng == *Teng);
            } while (addr >= StartAddr);
            if (EqTimes >= 3)
            {
                *Teng = lTeng;
                return 0;
            }
            else
            {
                return 1;
            }
        }
    }
    return 0;
}
