# Coordinator between car pre-heater and automatic engine start

This is firmware for embedded device which links car's engine pre-heater and automatic engine start. 
In cold wheter conditions, engine pre-heater (like Webasto) can be used for easy engine start. This firmware developed for unique device, which controls pre-heater operation and engine start.

Main functions are:
 - Determine current engine temperature and desides wheter to turn on pre-heater or engine (In winter pre-heater strats first, in summer - engine)
 - Turns on pre-heater and waits when it stops. (Pre-heater has it own module to control how long should it work).
 - After pre-heater completes engine heating, engine is started to recharge battery
 - Monitors car's battery charge levew. If it goes lower then specified threshold, engine (or pre-heater + engine) is started to recharge bettery
 - Logs all activity in on-board i2c memory module for later troubleshoouting
 - Accepts commands by i2c bus for external devices.