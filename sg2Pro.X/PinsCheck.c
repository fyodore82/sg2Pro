#include "PinsCheck.h"
#include <htc.h>

#define _XTAL_FREQ 4000000

//char i;

// Stack = 2 (ADCStart)
char StartCheck (char OnceMore)
{
  //  ToCheck &= 0b01111110;      // Set bits 7 & 0 to zero as there is not corresponding pins
    if (!ToCheck || (!OnceMore && CheckedNow != CN_NONE))     //
        return SC_DONE;


    if (CheckTimes == 0)    // Not checked before
    {
        CheckTimes = CT;
        NewState = NS_ZERO;
    }

    if (ToCheck & (1 << CN_21))   // 21 OR 25 - priority check
    {
        CheckedNow = CN_21;
        return SC_INPR;
    }
    if (ToCheck & (1 << CN_25))   // 21 OR 25 - priority check
    {
        CheckedNow = CN_25;
        return SC_INPR;
    }

    if (ToCheck & (1 << CN_1416))   // 21 OR 25 - priority check
    {
        CheckedNow = CN_1416;
        return SC_INPR;
    }

    if (ToCheck & (1 << CN_23))   // Check 23
    {
        CheckedNow = CN_23;
        //ADC_Start(AI23);
        return SC_INPR;
    }
    if (ToCheck & (1 << CN_24))   // Check 23
    {
        CheckedNow = CN_24;
//        ADC_Start(AI24);
        return SC_INPR;
    }
    if (ToCheck & (1 << CN_210))   // Check 23
    {
        CheckedNow = CN_210;
//        ADC_Start(AI210);
        return SC_INPR;
    }
    if (ToCheck & (1 << CN_28))   // Check 28, Ubatt
    {

        CheckedNow = CN_28;
        ADC_Start(AI28);
        Ubatt = 0;
        UbattLow = 0;
        return SC_INPR;
    }
    if (ToCheck & (1 << CN_29))     // 29 (Teng) should be checked just after start command)
    {
        CheckedNow = CN_29;
        ADC_Start(AI29);
        Teng = 0;
        return SC_INPR;
    }
    
    return SC_DONE;
}
// Stack = 3 (StartCheck)
char decr_check_times (void)
{
    CheckTimes--;
    if (!CheckTimes)
    {
        if (NewState >= NS_ZERO + CT)
            bitset (PinState, CheckedNow);
        if (NewState <= NS_ZERO - CT)
            bitclr (PinState, CheckedNow);

        bitclr (ToCheck, CheckedNow);       // Clear ToCheck bit, as we just checked that bit
        CheckedNow = CN_NONE;
        return 0;
    }
    else
    {
        __delay_ms (WAITMS/CT);
        return 1;
    }
}
// Stack = 4 (decr_check_times)
void CheckRes (void)
{
    if (CheckedNow == CN_21 || CheckedNow == CN_25 || CheckedNow == CN_23 || CheckedNow == CN_24 || CheckedNow == CN_210 || CheckedNow == CN_1416)
    {
        switch (CheckedNow)
        {
            case CN_21:
                if (DI21)
                    NewState++;
                else
                    NewState--;
                break;
            case CN_25:
                if (DI25)
                    NewState++;
                else
                    NewState--;
                break;
            case CN_23:
                if (!DI23)      // Invert DI23, DI24, DI210, DI1416, as they in "1" when "0" on input
                    NewState++;
                else
                    NewState--;
                break;
            case CN_24:
                if (!DI24)
                    NewState++;
                else
                    NewState--;
                break;
            case CN_210:
                if (!DI210)
                    NewState++;
                else
                    NewState--;
                break;
            case CN_1416:
                if (!DI1416)
                    NewState++;
                else
                    NewState--;
                break;
        }
        if (decr_check_times ())
        {
            StartCheck (1);  // Start Check again as port hadn't been checked CT times
                          //  Set OnceMore = 1 as we started again with the same parameters

        }

    }


    if (PIR1bits.ADIF)
    {
        PIR1bits.ADIF = 0;
        if (CheckedNow == CN_28 || CheckedNow == CN_29)     // Checking Teng or Ubatt
        {
            unsigned char Ub, UbLow, Te;
            if (CheckedNow == CN_28)
            {
                Ub = (ADRESH << 7) | (ADRESL >> 1);
                
                if (Ubatt == 0)
                    Ubatt = Ub;
                else
                    Ubatt = (Ub + Ubatt) / 2;
                
                UbLow = (ADRESL << 7);
                
                if (UbattLow == 0)
                    UbattLow = UbLow;
                else
                    UbattLow = (UbLow + UbattLow) / 2;
            }
            if (CheckedNow == CN_29)
            {
                Te = (ADRESH << 7) | (ADRESL >> 1);
                if (Teng == 0)
                    Teng = Te;
                else
                    Teng = (Te + Teng) / 2;
            }
        }
/*        else
        {
            if (ADRESH > 1 || (ADRESH = 1 && ADRESL > 0x32))   // > 1,5B
                NewState++;
            else
                NewState--;
        }*/
        if (decr_check_times ())
        {
            StartCheck (1);  // Start Check again as port hadn't been checked CT times
                          //  Set OnceMore = 1 as we started again with the same parameters

        }
    }

    return;
}
/*
void TMR2_Cfg (void)     // Configs Timer2 for digital inputs check
{
    T2CON = 0;

    char posc = WAITMS*3.9/CT;
    if (posc > 16)
    {
        T2CON |= (posc / 16 - 1) << 4;
        T2CON |= 0b10;  // 1:16 prescaler
    }
    else
        T2CON |= (posc - 1) << 4;
}
*/

// Stack = 1
void ADC_Start (char ANCh)      // ANCh - Analog channel num
{
    if (ADCON0bits.GO_DONE)
        return;

    ADRESH = 0;
    ADRESL = 0;
    if ((ADCON0 & 0b00111100) != (ANCh << 2))
    {
        ADCON0 &= 0b11000011;
        ADCON0 |= (ANCh << 2);
        __delay_us(16);     // ADC aqusition time
    }
    PIR1bits.ADIF = 0;
    ADCON0bits.GO_DONE = 1;
}

