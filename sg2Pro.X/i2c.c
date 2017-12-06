

#define _XTAL_FREQ 4000000

#define SCL TRISBbits.TRISB6
#define SDA TRISBbits.TRISB4
#define SCL_IN PORTBbits.RB6
#define SDA_IN PORTBbits.RB4

#define I2CND    0   // 0 - not detected
#define I2CBUSY  1   // 1 - Busy
#define I2CBUSYME 2  // Set busy by me
#define I2CFREE  3   // 2 - Free

char i2c_detect (void);  // Detect i2c status after enable
char i2c_start (void)   //  Start I2C. Return - 0 started, 1 - failed to start:
                        //    - I2C bus busy (multi master)
{
    /* NOT CHECKED !!! */// if (i2cstatus == I2CBUSYME ||
//                                  (i2cstatus == I2CFREE && (SSPSTATbits.P || !(SSPSTATbits.P | SSPSTATbits.S))))  // Ether STOP detected or both START & STOP = 0
    {
                              SDA = 1;
                              SCL = 1;
                              _delay (3);
                              SDA = 0b0;
                              SDA_IN = 0;   // This MAGIC MUST BE HERE
                              _delay (3);
                              SCL = 0b0;
                              SCL_IN = 0;   // This MAGIC MUST BE HERE
                              i2cstatus = I2CBUSYME;
                              return (0);
    }
                          return(1);
}

char i2c_stop (void)    // STOPS i2c transmission 0 - success
{
    char i = 10;
    SDA = 0;
    _delay (3);
    SCL = 1;
    /* NOT TESTED!*/ while (SCL_IN == 0 && i > 0)    // Is SCL high? (No slave holds SCL down for write to com1plete)
                     {
                         i--;
                         _delay(3);
                     }
    if (SCL_IN)
    {
        SDA = 1;
        i2cstatus = I2CFREE;
        return (0);
    }
    return (1);
}

char i2c_tx (unsigned char dt)
{
    char i;

    for (i = 8; i; i--)
    {
        if (dt&0x80)
            SDA = 1;
        else
            SDA = 0;
        dt <<= 1;
        SCL = 1;
        _delay (3);
        SCL = 0;
    }
    SDA = 1;
    SCL = 1;
    _delay (3);
    i = (char)SDA_IN;     // Possible ACK bit
    SCL = 0;
    return (i);
}

unsigned char i2c_rx (char ack, char *d)
{
    char i, j;
    *d = 0;
    SDA = 1;
    for(i = 0; i < 8; i++)
    {
        *d <<= 1;
        j = 3;
        do {
            SCL = 1;
            _delay(3);
            j--;
        }
/* SCL_IN NOT CHECKED */
        while(SCL_IN == 0 && j > 0);    // wait for any SCL clock stretching
                                            // no more than j - 1 times
        if (j == 0)
            return 1;

        if (SDA_IN)
            *d |= 1;
        SCL = 0;
   }
   if (ack)
       SDA = 0;
   else
       SDA = 1;
   SCL = 1;
   _delay(3);             // send (N)ACK bit
   SCL = 0;
   SDA = 1;
   return 0;
 }

char TXbyte (char addrh, char addrl, char data)     // Transmit 1 byte of data
{
    char ret = 0;
    char i = 4;

    if (i2c_start ())      // Cannot start
    {
        ret = 1;
        goto end;
    }
    if (i2c_tx(0b10100000))
    {
        ret = 2;
        goto end;
    }
    if (i2c_tx(addrh))
    {
        ret = 3;
        goto end;
    }
    if (i2c_tx(addrl))
    {
        ret = 4;
        goto end;
    }
    if (i2c_tx(data))
    {
        ret = 5;
        goto end;
    }
    end:
    while (i2c_stop() && --i > 0)
        _delay (3);
    if (i == 0)
        ret = 0x10;
    return (ret);
}

char TXWaitACK (char cyclestowait)
{
    char ret = 0;
    char i = 4;

    do
    {
        if (i2c_start())
        {
            ret = 5;
            goto end;
        }
        ret = i2c_tx(0b10100000);
        _delay (3);
    }
    while (cyclestowait > 0 && ret > 0);

    end:
    while (i2c_stop() && --i > 0)
        _delay (3);

    if (i == 0)
        ret = 0x10;
    return ret;

}

char RXbyte (char addrH, char addrL, char *d)
{
    char ret = 0;
    char i = 4;

    if (i2c_start())
    {
        ret = 1;
        goto end;
    }
    if (i2c_tx(0b10100000))
    {
        ret = 2;
        goto end;
    }
    if (i2c_tx(addrH))
    {
        ret = 3;
        goto end;
    }
    if (i2c_tx(addrL))
    {
        ret = 4;
        goto end;
    }
    if (i2c_start())
    {
        ret = 5;
        goto end;
    }
    if (i2c_tx(0b10100001))
    {
        ret = 6;
        goto end;
    }
    if (i2c_rx(0, d))
    {
        ret = 7;
        goto end;
    }
    end:
    while (i2c_stop() && --i > 0)
        _delay (3);
    if (i == 0)
        ret = 0x10;
    return ret;
}

