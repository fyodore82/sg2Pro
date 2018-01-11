/* 
 * File:   PinsCheck.h
 * Author: fedor
 *
 * Created on 2 ???? 2013 ?., 14:49
 */

#ifndef PINSCHECK_H
#define	PINSCHECK_H

#ifdef	__cplusplus
extern "C" {
#endif

                // BitNum 7  6  5  4   3    2   1  0
char PinState;  // Bits: 29;210;25;24; 23;14-16;21;28
                // 29 - engine temperature
                // 28 - Batt U
char NewState;  // Starts from NS_ZERO. -1 for Low State, +1 for High State. If >= NS_ZERO + 3 -> write to PinState
char Teng;
char Ubatt;

#define CT  3
#define NS_ZERO 0x80    // NewState = Zero
char CheckTimes;    // =3 and -1 with any check try
char ToCheck;   // Bits

#define WAITMS  3*CT   // In each check cycle we should wait a little to ensure that signal stable during WAITMS ms

#define CN_NONE 9       // None
#define CN_28   0       // Analog: Ubatt
#define CN_21   1       // Digital
#define CN_1416 2       // Digital
#define CN_23   3       // Digital
#define CN_24   4       // Digital
#define CN_25   5       // Digital
#define CN_210  6       // Digital
#define CN_29   7       // Analog: Teng
char CheckedNow;

#define SC_DONE 0;
#define SC_INPR 1;

#define bitset(var, bitno)    ((var) |= 1UL << (bitno))
#define bitclr(var, bitno)    ((var) &= ~(1UL << (bitno)))

#define DI21    PORTBbits.RB7 //PORTBbits.RB5
#define DI23    PORTCbits.RC5   //4
#define DI24    PORTCbits.RC4   //5
#define DI25    PORTBbits.RB5 //PORTAbits.RA2
#define DO26    PORTCbits.RC1 //PORTCbits.RC6
#define DO27    PORTCbits.RC2 //PORTCbits.RC4
#define AI28    4             //6
#define AI29    9
#define DI210   PORTAbits.RA2 //7
#define DI1416  PORTCbits.RC6 //PORTBbits.RB7
#define DO1819  PORTCbits.RC3 //PORTCbits.RC5

char StartCheck (char OnceMore);
char decr_check_times (void);
void CheckRes (void);
/*
void TMR2_Cfg (void);     // Configs Timer2 for digital inputs check
*/
void ADC_Start (char ANCh);      // ANCh - Analog channel num

void OutRet (char, char *, char);


#ifdef	__cplusplus
}
#endif

#endif	/* PINSCHECK_H */

