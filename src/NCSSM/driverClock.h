#ifndef __driverClock_h
#define __driverClock_h

extern volatile unsigned long elapsedTime;

/**
 * called on clock interrupt.
 */
void driverClock_interruptHandler( void );

/**
 * returns the current elapsed driverClock time since startup
 */
unsigned long driverClock_getElapsedTime( void );

#endif

