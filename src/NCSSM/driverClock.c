#include "ifi_picdefs.h"
#include "driverClock.h" 

volatile unsigned long elapsedTime = 0;

void driverClock_interruptHandler(void)
{
	// this function will be called when a timer 0 interrupt occurs
	// Note:  Initilize_Timer_0() sets the prescaler to 8:1 and intitial value to 0x82
	//        and the counter is running immediately.
	// 	  Increment the total elapsedTime variable and set the sampleAndHold flag.
	// 	  Setting sampleAndHold tells the PID Control routines its time to update
	// 	  with new control data  PID needs to run at a _consistent_ sample rate.
	// mcfalls - timer0 interrupt routine

	
	++elapsedTime;
}

unsigned long driverClock_getElapsedTime() {
	return elapsedTime;
}

