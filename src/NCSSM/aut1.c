#define _aut1_c

#include "aut1.h"
#include "speedometer.h"
#include "arm.h"
#include "user_routines.h"
#include "wheels.h"

#define STATE_INITIAL 0
#define WAIT_TURN1 1
#define ARM_TO_POS0 3
#define SEEK_FINGER_CENTER 4
#define ARM_TO_POS1 6
#define ARM_TO_POS2 7
#define END_IDLE 8


#define INITIAL_TURN_TARGET   0  // ? needs to be 90deg left
#define INITIAL_TURN_SPEED    0  // ?
#define SHOULDER_POSITION_0   500  // ?? should be our start position
#define SHOULDER_POSITION_1   1000 // ?? way up - needs to bump the tetra
#define SHOULDER_POSITION_2   900  // ?? a little bump down from pos1

typedef struct {
	int turnMonitor_initial;
	int turnMonitor_target;
	int dist_initial;
	int dist_target;
	char aut_state;
} aut1_struct;

static aut1_struct aut1_data;

/**
 * target is target delta between wheel distance.
 * left turn is positive, right turn is negative
 *
 */
static void turnMonitor_start( int target ) {
	aut1_data.turnMonitor_initial = speedometer_getDistance( &speedometer_right ) - speedometer_getDistance( &speedometer_left );
	aut1_data.turnMonitor_target = aut1_data.turnMonitor_initial + target;
}

static char turnMonitor_isDone( void ) {
	int actualDelta = speedometer_getDistance( &speedometer_right ) - speedometer_getDistance( &speedometer_left );
	if( aut1_data.turnMonitor_target > 0 ) {
		return (actualDelta >= aut1_data.turnMonitor_target);
	} else if (aut1_data.turnMonitor_target < 0 ) {
		return (actualDelta <= aut1_data.turnMonitor_target);
	} else {
		return (actualDelta==0);
	}
}

/**
 * target is delta between avg(left,right) current and avg(left,right) target
 * roughly target is is distance to be travled.
 * forward is positive, reverse is negative
 */
static void distance_start( int target ) {
	aut1_data.dist_initial = 
		(speedometer_getDistance( &speedometer_right ) + speedometer_getDistance( &speedometer_left ))/2;
	aut1_data.dist_target = aut1_data.dist_initial + target;
}

static char distance_isDone( void ) {
	int actualDelta =
		(speedometer_getDistance( &speedometer_right ) + speedometer_getDistance( &speedometer_left ))/2;
	if( aut1_data.dist_target > 0 ) {
		return (actualDelta >= aut1_data.dist_target);
	} else if (aut1_data.turnMonitor_target < 0 ) {
		return (actualDelta <= aut1_data.dist_target);
	} else {
		return (actualDelta == 0);
	}
}

void aut1_reset( void ) {
	aut1_data.aut_state = STATE_INITIAL;
}

void aut1_moduleInitialize( void ) {
	aut1_data.turnMonitor_initial = 0;
	aut1_data.turnMonitor_target = 0;
	aut1_reset();
}


void aut1_doMain( void ) {
	if( ur_data.aut_reset ) {
		ur_data.aut_reset = 0;
		aut1_reset();
	}

	switch( aut1_data.aut_state ) {
		case STATE_INITIAL:
			turnMonitor_start( INITIAL_TURN_TARGET );
			wheels_setTurningSpeed( INITIAL_TURN_SPEED );
			aut1_data.aut_state = WAIT_TURN1;
		break;

		case WAIT_TURN1:
			if( turnMonitor_isDone() ) {
				wheels_setTurningSpeed( 0 );
				aut1_data.aut_state = ARM_TO_POS1;
			}
		break;

		case ARM_TO_POS1:
			if( 0 == arm_seekShoulderPosition( SHOULDER_POSITION_1 ) ) {
				aut1_data.aut_state = ARM_TO_POS2;
			}
		break;
		
		case ARM_TO_POS2:
			if( 0 == arm_seekShoulderPosition( SHOULDER_POSITION_2 ) ) {
				aut1_data.aut_state = ARM_TO_POS1;
			}
		break;

		default:
		break;
	}
}
