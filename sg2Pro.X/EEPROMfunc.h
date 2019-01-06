/* 
 * File:   EEPROMfunc.h
 * Author: fedor
 *
 * Created on 19 ?????? 2014 ?., 17:06
 */

#ifndef EEPROMFUNC_H
#define	EEPROMFUNC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define TENG_EEPROM 0x10        // First byte, where Teng storen in EEPROM
#define UBATT_EEPROM TENG_EEPROM + 5    // First byte, where Ubatt is stored in EEPROM

#define _XTAL_FREQ 4000000

unsigned char TEngUbattRW (unsigned char StartAddr, unsigned char *Teng, unsigned char Read);
void OutRet (unsigned char addr, unsigned char *ret, unsigned char Read);


#ifdef	__cplusplus
}
#endif

#endif	/* EEPROMFUNC_H */

