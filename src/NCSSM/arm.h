/***************************************
 * Arm Driver
 * File: arm.h
 * Version 1.9 3/16/05
 *
 ****************************************/

#ifndef __arm_h
#define __arm_h

#include "ifi_aliases.h"
#include "ifi_picdefs.h"


// Define limitSwitch Variable aliases
#define shoulderADC_MAX 695

#define fingerMinLimit    0
#define fingerMaxLimit    0
#define fingerCenterLimit 0

#define shoulderMinLimit   (shoulderPosSensor >= shoulderADC_MAX-20)
#define shoulderMaxLimit   (shoulderPosSensor <= shoulderADC_MAX-272)

extern int shoulderPosSensor;

//*** function prototypes ***/

// recomputes pwm values based on feedback
// and current arm position inputs
// called once every 26ms
// Arm position is sampled every 26ms
// this method must not allow the arm or finger to move beyond
// allowed limits.  This means that it must analyze the values
// of any limit switches, and respond accordingly.
// 
// This method must compute suggested pwm values for shoulder
// and arm mechanisms, and make them available via calls to
// arm_getShoulderPWM() and arm_getFingerPWM().  An external
// state machine will use this information to manage the locking
// mechanisms, and user-controls, before passing the pwm values
// along to the hardware. This function should, however, manage
// interaction with the position sensors (pot's) internally.
//
// this method *must* insure that the mechanism is driven to the actual
// position requested, before it raises the "isInPosition" flag.
// however, once that flag has been raised, it must insure that
// the mechanism's actual position is allowed to deviate from 
// the setpoint by a small amount, without trying to drive 
// it back to position, and without lowering the isInPosition flag.
// 
// This is important, as locking points are at discrete positions
// along the continuous path of the mechanisms.  The mechanisms
// must be driven past the locking point, and then allowed to
// "fall into" the locking point, without causing the system
// to become unstable.  The locking state machine will rely on
// the value of isInPosition, to manage the state of the lock.
void arm_doMain( void );

// Temorary workaround to get suggestedPWM to putput driver w/o extern
void arm_doShoulderOI( 
	unsigned int toggleUp, 
	unsigned int toggleDown,
	unsigned int preset1,
	unsigned int preset2,
	unsigned int preset3,
	unsigned int preset4 ) ;

// changes the set-point for the robot's shoulder position
// specified in radians10-3.  
// 0 = min rotation, 2845milliradians = max rotation (163 degrees)
int arm_seekShoulderPosition( int desiredPosition );

// Gets the actual position of the shoulder
// specified in radians10-3.  
// 1 degree = ~17 milliRadians
// 0 = min rotation, shoulder joint max rotation is 163 degrees 20pi = max rotation
// NOTE:  The ADC on the Contoller returns a 10 bit number  Vref = 5V, so 0x3FF = 5V
// The voltage swing on the shoulder will be 0V to 3.40V; therefore 0 to 695 for a 10 bit number
//  Converting the ADC "tic" number to milliradians:  163 = 2845 millirad, Ks*695 = 2845, Ks = 4 millirad/tic
//  This yields a measured shoulder angle that is within 3.72 degrees of actual
int arm_getShoulderPosition( void );
void arm_setShoulderSensor( int sensorValue );

// This module needs to ensure that the arm doesn't change position wildly
// at startup due to desired position inputs being wildly different than the
// actual arm position at startup
void arm_moduleInitialize( void );

void arm_reset( void );

void arm_doFingerOI( int moveUp, int moveDown );

#endif

