#ifndef _XC_H
#define _XC_H

#define _UNIT_TEST_

struct {
  unsigned int RC1;
  unsigned int RC2;
  unsigned int RC3;
  unsigned int RC4;
  unsigned int RC5;
  unsigned int RC6;
} PORTCbits; 

struct {
  unsigned int RB5;
  unsigned int RB7;
} PORTBbits; 

struct {
  unsigned int RA2;
} PORTAbits; 

void __delay_ms(int value) {}

#endif