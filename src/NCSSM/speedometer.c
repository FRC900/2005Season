#define __speedometer_c

#include "speedometer.h"
#include "constants.h"
#include "driverClock.h"
#include <stdio.h>

/*** local constants ***/

// distance traveled in one encoder click (mm)
#define leftEncoderDistance   8.47
#define rightEncoderDistance  8.47

// what timer value indicates a velocity of zero
// with value of 2000, any speed below 42mm/s is === 0
#define zeroVelocityTimerValue  2000

/*** global variables ***/
volatile speedometer_State speedometer_left; 
volatile speedometer_State speedometer_right;



/*** internal functions ***/

/**
 * returns the last known speed of the component.
 * speed is returned in the units specified in the
 * constant EncoderDistance, per second.  This is,
 * typically, mm/s.
 *
 * @param instance 
 *   the speedometer instance that should be read.
 */
unsigned int speedometer_getSpeed( speedometer_State *instance ) {
	unsigned long tCurrent = driverClock_getElapsedTime();
	signed long tDelta = tCurrent - (instance->tLast);
	if( tCurrent <= zeroVelocityTimerValue || tDelta > zeroVelocityTimerValue ) {
		instance->speed = 0;
	}
	return instance->speed;
}

/**
 * handleEncoderClick is called from the encoder interrupt
 * whenever the encoder raises its trigger.  This causes the
 * last known speed to be computed, based upon the elapsed
 * time since the last encoder click, and the distance per
 * encoder click as represented by the constant EncoderDistance.
 *
 * this function depends upon the driverClock for time
 * measurements.  It also uses the constant driverClock_period
 * to convert the driver clock time into seconds.
 */
void speedometer_handleEncoderClick( speedometer_State *instance ) {
	unsigned long tCurrent = driverClock_getElapsedTime();

    instance->tDelta = tCurrent - instance->tLast;
    instance->speed = (unsigned int)(instance->scaleFactor/instance->tDelta);
	instance->distance += instance->encoderDistance;
	instance->tLast = tCurrent;

	//printf( "encoder click. speed is: %d, %d, %ld \r", instance->speed, instance->scaleFactor, tDelta );
}

void speedometer_moduleInitialize( void ) {
	speedometer_left.speed = 0;
	speedometer_left.distance = 0;
	speedometer_left.tLast = 0;
	speedometer_left.tDelta = 0;
	speedometer_left.scaleFactor = 84700L; //(int)(leftEncoderDistance/driverClock_period);
	speedometer_left.encoderDistance = 847;//(int)(leftEncoderDistance*100);
	speedometer_right.speed = 0;
	speedometer_right.distance = 0;
	speedometer_right.tLast = 0;
	speedometer_right.tDelta = 0;
	speedometer_right.scaleFactor =84700L; //(int)(rightEncoderDistance/driverClock_period);
	speedometer_right.encoderDistance = 847;//(int)(rightEncoderDistance*100);
}

/**
 * returns the total distance that the encoder has traveled since startup.
 */
unsigned long speedometer_getDistance( speedometer_State *instance ) {
	return instance->distance / 100;
}


/*** averaging speedometer ***/




volatile AvgSpeedometer avgSpeed_left;
volatile AvgSpeedometer avgSpeed_right;

void avgSpeed_moduleInitialize( void ) {
	avgSpeed_left.reset = 0;
	avgSpeed_left.numSamples = 0;
	avgSpeed_left.currentIndex = 0;
	avgSpeed_left.Tlast = 0;
	avgSpeed_left.scaleFactor = 84700L; //(int)(leftEncoderDistance/driverClock_period);
//	avgSpeed_left.encoderDistance = 847;//(int)(leftEncoderDistance*100);

	avgSpeed_right.reset - 0;
	avgSpeed_right.numSamples = 0;
	avgSpeed_right.currentIndex = 0;
	avgSpeed_right.Tlast = 0;
	avgSpeed_right.scaleFactor =84700L; //(int)(rightEncoderDistance/driverClock_period);
//	avgSpeed_right.encoderDistance = 847;//(int)(rightEncoderDistance*100);
}

static void storeSample( unsigned long sample, AvgSpeedometer *instance ) {
	unsigned int current;

	if( instance->reset ) {
		instance->numSamples = 0;
		instance->currentIndex = 0;
		instance->reset = 0;
		current = 0;
	} else {
		current = instance->currentIndex;
		++current;
		if( current == max_samples ) {
			current = 0;
		}
	}

	instance->samples[current] = sample;

	if( instance->numSamples < max_samples ) {
		++(instance->numSamples);
	}
	instance->currentIndex = current;
}

static unsigned long computeAvg( AvgSpeedometer *instance ) {
	unsigned int numSamples = instance->numSamples;
	unsigned long sum = 0;
	unsigned int i = 0;

	for( i = 0; i < numSamples; ++i ) {
		sum += instance->samples[i];
	}

	return (unsigned long)(sum/numSamples);
}

void avgSpeed_handleInterrupt( AvgSpeedometer *instance ) {

	unsigned long Tcurrent = driverClock_getElapsedTime();
	unsigned long Tsample = Tcurrent - instance->Tlast;

	storeSample( Tsample, instance );

	instance->Tlast = Tcurrent;
}

unsigned int avgSpeed_getSpeed( AvgSpeedometer *instance ) {
	unsigned long Tcurrent = driverClock_getElapsedTime();
	signed long Tdelta = Tcurrent - instance->Tlast;

	if( (Tdelta > zeroVelocityTimerValue) || (Tcurrent < zeroVelocityTimerValue) ) {
		instance->reset = 1;
		return 0;
	} else {
		return (int)(instance->scaleFactor/computeAvg( instance ));
	}
}
 
