/* 
 * File:   EngBinSt.h
 * Author: fedor
 *
 * Created on 16 ?????? 2013 ?., 20:52
 */

#ifndef ENGBINST_H
#define	ENGBINST_H

#ifdef	__cplusplus
extern "C" {
#endif

#define ASRTOST     1   // ASR should be started (CN_25 is 1, wait when it will be 0 and start)
#define ASROFFON    2   // ASR should be turned ON (gave negative impulse to ASR, but not acknoleged that Eng started)
#define ENGASRON    3   // Engine started by ASR
#define ASRONOFF    4   // ASR should be turned OFF (gave negative impulse to ASR, but not acknoleged that Eng turned OFF)
#define ASRTOSTOP   5   // Got STOP command (CN_25 is 0, wait when it will be 1 and stop)

#define ENGON       7   // Engine started by key
#define OFF         0   // Everything OFF


#define ERR_BINPATTERN 0x80
#define ERR_ENGPATTERN 0xC0

// We have only 3 bits for Eng/Bin State/Error
#define ERR_BINOFF      0x87

#define ERR_BINTOST     0x81
#define ERR_BINONOFF    0x82
#define ERR_BINON       0x83
#define ERR_BINOFFON    0x84
#define ERR_BINTOSTOP   0x85

// We have only 3 bits for Eng/Bin State/Error
#define ERR_ASRTMR_ENGOFF   0xC6    // Engine is turned off but Timer is not expired (engine hadn't worked for at least 0x10*16 seconds

#define ERR_ASRTOST     0xC1
#define ERR_ASROFFON    0xC2
#define ERR_ASRON       0xC3
#define ERR_ASRONOFF    0xC4
#define ERR_ASRTOSTOP   0xC5

#define ERR_ASRTMR_ENGON    0xC7

#define BINTOST     0x11   // Binar should be started (CN_25 is 0, wait when it will be 1 and start)
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
#define ISS_TEBHL   0b01000000      // TEB goes to TEB_LOW. Used to send signal to brelok only once

// StateTest - we got STOP command and will send DO=1 if we will start engine
unsigned char StateTest;

#define     ST_NONE     0
#define     ST_TEST     1

#define WL_DEFAULT 0
#define WL_ERR      0x11
#define WL_OFF      0x12
#define WL_PINS     0x13
#define WL_RESETREASON  1
#define WL_NEWERR   2
#define WL_ALLOK    3
#define WL_TEBHL    4
#define WL_TEBLH    5
#define WL_STARTBYLOWVOLTAGE 6
#define WL_NOLOG 0xFF   // Logging not needed

// ASRChkTmr - After starting by ASR check if Engine works for at least ASRChkTmr. If less, go to ERR
// EngChkTmr - After starting engine check if it works for at least EngChkTmr. If less don't clear ERR and go to ERR
// NewBinTmr, NewASRTmr - Timers, which are turned on after Binar or ASR switched to new states
// EngStartTmr - Time after Eng switched to OFF during Eng can be started and turned OFF withou ERR
// ASRONTmr - After this timer ASR will be started. Timer turned on after Binar stopped
// UbattTmr - Timer decremented if battery is low
// TEB - Start Engine or Binar. If more or equal to 0x80 - Engine, if less or equal to 0x7F - Binar
// TEBPrev - Previous value of TEB. If TEB != TEBPrev we identified that TEB gone to High from Low or vice versa. We need to log and make TEBPrev = TEB
unsigned char BinSt, EngSt, NewBinTmr, NewASRTmr, ASRONTmr, EngStartTmr, UbattTmr, TEB, TEBPrev;


#ifdef	__cplusplus
}
#endif

#endif	/* ENGBINST_H */

