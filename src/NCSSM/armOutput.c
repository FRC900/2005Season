#define _armOutput_c

#include "armOutput.h"
#include "arm.h"
#include <ifi_aliases.h>
#include <ifi_default.h>

#define STATE_LOCKED 0
#define STATE_BUMPUP 1
#define STATE_BUMPDOWN 2
#define STATE_UNLOCKED 3

#define BUMP_TIME_SHOULDER_UP 1
#define BUMP_TIME_SHOULDER_DOWN 1
#define PWM_SHOULDER_STOPPED 127
#define PWM_SHOULDER_BUMP_UP 127
#define PWM_SHOULDER_BUMP_DOWN 127

#define BUMP_TIME_FINGER_UP 3
#define BUMP_TIME_FINGER_DOWN 3
#define PWM_FINGER_STOPPED 127
#define PWM_FINGER_BUMP_UP 170
#define PWM_FINGER_BUMP_DOWN 115

// Define PWM output Values
#define PWM_SHOULDER_UP 190
#define PWM_SHOULDER_DOWN  117

#define PWM_FINGER_UP 190
#define PWM_FINGER_DOWN 110


#define sOut pwm03
#define fOut pwm04
#define sLockOut relay4_fwd
#define sUnlockOut relay5_fwd
#define fLockOut relay6_fwd
#define fUnlockOut relay7_fwd

static short int sLockState;
static short int fLockState;
static short int sBumpCounter;
static short int fBumpCounter;

static unsigned char suggestedShoulderPWM = 127;
static unsigned char suggestedFingerPWM = 127;

static void unlockShoulder( void ) {
	sUnlockOut = 1;
	sLockOut = 0;
}

static void lockShoulder( void ) {
	sLockOut = 1;
	sUnlockOut = 0;
}


static void unlockFinger( void ) {
	fUnlockOut = 1;
	fLockOut = 0;
}

static void lockFinger( void ) {
	fLockOut = 1;
	fUnlockOut = 0;
}

void armOutput_moveShoulderUp() {
	suggestedShoulderPWM = PWM_SHOULDER_UP;
}

void armOutput_moveShoulderDown() {
	suggestedShoulderPWM = PWM_SHOULDER_DOWN;
}

void armOutput_moveFingerDown() {
	suggestedFingerPWM = PWM_FINGER_DOWN;
}

void armOutput_moveFingerUp() {
	suggestedFingerPWM = PWM_FINGER_UP;
}

void armOutput_stopFinger() {
	suggestedFingerPWM = PWM_FINGER_STOPPED;
}

void armOutput_stopShoulder() {
	suggestedShoulderPWM = PWM_SHOULDER_STOPPED;
}

void armOutput_reset( void ) {
	sBumpCounter = 0;
	fBumpCounter = 0;
	sLockState = STATE_LOCKED;
	fLockState = STATE_LOCKED;

	lockShoulder();
	lockFinger();

	sOut = PWM_SHOULDER_STOPPED;
	fOut = PWM_FINGER_STOPPED;
}

void armOutput_moduleInitialize()
{
	armOutput_reset();
}


void armOutput_doShoulderOutput() {

	char isInPosition = (suggestedShoulderPWM == PWM_SHOULDER_STOPPED);

	switch( sLockState ) {
		case STATE_LOCKED:
			if( ! isInPosition ) {
				unlockShoulder();
				sOut = PWM_SHOULDER_STOPPED;
				sBumpCounter = BUMP_TIME_SHOULDER_UP;
				sLockState = STATE_BUMPUP;
			} else {
				sOut = PWM_SHOULDER_STOPPED;
			}
		break;


		case STATE_BUMPUP:
			if( sBumpCounter != 0 && !shoulderMaxLimit) {
				sOut = PWM_SHOULDER_BUMP_UP;
				--sBumpCounter;
			} else {
				sOut = PWM_SHOULDER_STOPPED;
				sBumpCounter = BUMP_TIME_SHOULDER_DOWN;
				sLockState = STATE_BUMPDOWN;
			}
		break;


		case STATE_BUMPDOWN:
			if( sBumpCounter != 0 && !shoulderMinLimit ) {
				sOut = PWM_SHOULDER_BUMP_DOWN;
				--sBumpCounter;
			} else {
				sOut = PWM_SHOULDER_STOPPED;
				sLockState = STATE_UNLOCKED;
			}		
		break;


		case STATE_UNLOCKED:
			if( isInPosition ) {
				sOut = PWM_SHOULDER_STOPPED;
				lockShoulder();
				sLockState = STATE_LOCKED;
			} else {
				if( suggestedShoulderPWM > PWM_SHOULDER_STOPPED ) {
					// moving up
					if( shoulderMaxLimit ) {
						suggestedShoulderPWM = PWM_SHOULDER_STOPPED;
						sOut = PWM_SHOULDER_STOPPED;
						lockShoulder();
						sLockState = STATE_LOCKED;
					} else {
						sOut = suggestedShoulderPWM;
					}
				} else {
					// moving down
					if( shoulderMinLimit ) {
						suggestedShoulderPWM = PWM_SHOULDER_STOPPED;
						sOut = PWM_SHOULDER_STOPPED;
						lockShoulder();
						sLockState = STATE_LOCKED;
					} else {
						sOut = suggestedShoulderPWM;
					}
				}
			}
		break;


		default:
			sLockState = STATE_LOCKED;
	}
}


void armOutput_doFingerOutput() {

	char isInPosition = (suggestedFingerPWM == PWM_FINGER_STOPPED);

	switch( fLockState ) {
		case STATE_LOCKED:
			if( ! isInPosition ) {
				unlockFinger();
				fOut = PWM_FINGER_STOPPED;
				fBumpCounter = BUMP_TIME_FINGER_UP;
				fLockState = STATE_BUMPUP;
			} else {
				fOut = PWM_FINGER_STOPPED;
			}
		break;


		case STATE_BUMPUP:
			if( (fBumpCounter != 0) && !fingerMaxLimit) {
				fOut = PWM_FINGER_BUMP_UP;
				--fBumpCounter;
			} else {
				fOut = PWM_FINGER_STOPPED;
				fBumpCounter = BUMP_TIME_FINGER_DOWN;
				fLockState = STATE_BUMPDOWN;
			}
		break;


		case STATE_BUMPDOWN:
			if( (fBumpCounter != 0) && !fingerMinLimit) {
				fOut = PWM_FINGER_BUMP_DOWN;
				--fBumpCounter;
			} else {
				fOut = PWM_FINGER_STOPPED;
				fLockState = STATE_UNLOCKED;
			}		
		break;


		case STATE_UNLOCKED:
			if( isInPosition ) {
				fOut = PWM_FINGER_STOPPED;
				lockFinger();
				fLockState = STATE_LOCKED;
			} else {
				if( (fingerMaxLimit && (suggestedFingerPWM > PWM_FINGER_STOPPED)) || (fingerMinLimit && (suggestedFingerPWM < PWM_FINGER_STOPPED)) ) 
				{
					suggestedFingerPWM = PWM_FINGER_STOPPED;
					lockFinger();
					fLockState = STATE_LOCKED;
				} 
				fOut = suggestedFingerPWM;
			}
		break;


		default:
			fLockState = STATE_LOCKED;
	}
}
