/***************************************
 *  Arm Driver
 *  File: arm.c
 *  Version 1.9 3/16/05
 *
 ***************************************/
#define arm_c

#ifndef _FRC2004_
	#include <stdio.h>
#else
	#include "printf_lib.h"
#endif

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "ifi_picdefs.h"
#include "armOutput.h"

#include "arm.h"
//*** Variables and defines

// Set the finger and shoulder ADC to milliradian scaling coefficients
// #define Kf 8
#define Ks 4


// Define Shoulder Comparator hysteresis values for "in position"
// FIXME - these need to be correctly sized based on the # of locking pin positions
#define shoulderSmallThresh 20
#define shoulderLargeThresh 45


#define STATE_IDLE 0
#define STATE_UP 1
#define STATE_DOWN 2
#define STATE_P1 3
#define STATE_P2 4
#define STATE_P3 5
#define STATE_P4 6

#define P1_POS 4
#define P2_POS 180
#define P3_POS 676
#define P4_POS 1140

int  shoulderPosSensor;

static unsigned int state = STATE_IDLE;
static char shoulderIsInPosition; // Flag
static int  fingerDriveCounter;
static char fingerIsInPosition;   // Flag

static unsigned char outputShoulderPWM;  //  This will be arm_UP, arm_DOWN, arm_STOP
static unsigned char outputFingerPWM;    //  This will be arm_UP, arm_DOWN, arm_STOP

// Note:  This may change with the arm_moduleInitialize() 
// routine. it may be calculated from an offset from 
// "non-zero zero".
static const int maxShoulderAngle = 2845; // milliradians


/**
 * computes the absolute value of an Integer
 */
static int abs(int number){
	if(number < 0){
		return(-1*number);
	}
	else{
		return(number);
	}
}

void arm_setShoulderSensor( int sensorValue ) {
	shoulderPosSensor = sensorValue;
}

/**
 * This is an output handling function - typically this is only called internal to arm_doMain()
 * Provides the position of the arm based on an ADC reading of a potentiometer position sensor.
 */
int arm_getShoulderPosition( void ) {
	return (int)Ks*(shoulderADC_MAX - shoulderPosSensor);
}

/**
 * The Comparator function is called locally only to arm.c
 * The comparator does most of the actual work - the "PID" module
 * is folded into the Comparator - a single value for a speed will be used to
 * drive the arm motor into position, thus no PID - "bang bang" controller.
 */
static int computeShoulderError( int desiredShoulderPosition) {
	static char state = 0;
	static int counter = 0;

	int error = 0;
	int errorMagnitude = 0;

	error = desiredShoulderPosition - arm_getShoulderPosition();
	errorMagnitude = abs(error);

	if( counter == 0 ) {
		printf("shoulderERROR [state]: %d [%d]\r", error, (int)state);
		++counter;
	} else if( counter == 100 ) {
		counter = 0;
	} else {
		++counter;
	}

	// 5 < error < 50
	// error < 5 ( and 50 ) 
	// error > 50 (and also > 5 )
	// error < 5 and greater than 50 - cannot happen

	switch ( state ) { // 2 States - InPosition=0, OutPosition=1
		case 0:
			if( errorMagnitude > shoulderLargeThresh ){
				state = 1;
				return error;
			}
			return 0;
		break;

		case 1:
			if (errorMagnitude < shoulderSmallThresh) {
				state = 0;
				return 0;
			}
			return error;
		break;

		default: 
			// Should never get here...
			return error;
	}
}

int arm_seekShoulderPosition( int pos ) {
	int error = computeShoulderError( pos );

	if( error > 0 ) {
		armOutput_moveShoulderUp();
	} else if( error < 0 ) {
		armOutput_moveShoulderDown();
	} else {
		armOutput_stopShoulder();
	}

	return error;
}

void arm_doShoulderOI( 
	unsigned int toggleUp, 
	unsigned int toggleDown,
	unsigned int preset1,
	unsigned int preset2,
	unsigned int preset3,
	unsigned int preset4 ) 
{
	unsigned char isPreset = (preset1 || preset2 || preset3 || preset4);
	int error = 0;

	switch( state ) {
		case STATE_IDLE:
			if( !isPreset ) {
				if( toggleUp && !toggleDown ) {
					armOutput_moveShoulderUp();
					state = STATE_UP;
				} 
				else if( !toggleUp && toggleDown ) {
					armOutput_moveShoulderDown();
					state = STATE_DOWN;
				}
				break;
			} else {
				if( !toggleUp && !toggleDown ) { 
					if( preset1 ) {
						state = STATE_P1;
					} else if( preset2 ) {
						state = STATE_P2;
					} else if( preset3 ) {
						state = STATE_P3;
					} else if( preset4 ) {
						state = STATE_P4;
					}
				}
				break;
			}

		case STATE_UP:
			if( !toggleUp ) {
				armOutput_stopShoulder();
				state = STATE_IDLE;
			}
			break;

		case STATE_DOWN:
			if( !toggleDown ) {
				armOutput_stopShoulder();
				state = STATE_IDLE;
			}
			break;

		case STATE_P1:
			if( preset2 || preset3 || preset4 || toggleUp || toggleDown ) {
				state = STATE_IDLE;
				arm_reset();
			} else {
				error = arm_seekShoulderPosition( P1_POS );
				if( error == 0 ) {
					state = STATE_IDLE;
				}
			}
			break;

		case STATE_P2:
			if( preset1 || preset3 || preset4 || toggleUp || toggleDown ) {
				state = STATE_IDLE;
				arm_reset();
			} else {
				error = arm_seekShoulderPosition( P2_POS );
				if( error == 0 ) {
					state = STATE_IDLE;
				}
			}
			break;

		case STATE_P3:
			if( preset1 || preset2 || preset4 || toggleUp || toggleDown ) {
				state = STATE_IDLE;
				arm_reset();
			} else {
				error = arm_seekShoulderPosition( P3_POS );
				if( error == 0 ) {
					state = STATE_IDLE;
				}
			}
			break;

		case STATE_P4:
			if( preset1 || preset2 || preset3 || toggleUp || toggleDown ) {
				state = STATE_IDLE;
				arm_reset();
			} else {
				error = arm_seekShoulderPosition( P4_POS );
				if( error == 0 ) {
					state = STATE_IDLE;
				}
			}
			break;


		default:
			state = STATE_IDLE;
	}
}

void arm_doFingerOI( int moveUp, int moveDown ) {
	if( moveUp ) {
		armOutput_moveFingerUp();

	} else if( moveDown ) {
		armOutput_moveFingerDown();

	} else {
		armOutput_stopFinger();
	}	
}


void arm_moduleInitialize( void ){
	arm_reset();
}

void arm_reset( void ) {
	armOutput_stopShoulder();
	armOutput_stopFinger();
	state = STATE_IDLE;
}
