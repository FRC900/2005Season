#define wheels_c

#ifndef _FRC2004_
	#include <stdio.h>
#else
	#include "printf_lib.h"
#endif

#include "FRC2005/ifi_aliases.h"
#include "FRC2005/ifi_default.h"
#include "FRC2005/ifi_utilities.h"
#include "FRC2005/ifi_picdefs.h"

#include "wheels.h"
#include "speedometer.h"

/** Local Prototypes **/
long limitTurningSpeed(long a_turningSpeed);
int wheels_getDriveDirection(char a_pwm);
int safeAdd(int one, int two);
unsigned char safePWMs(long one, long two);
long safeAddLong( long one, long two );
/*** exported globals ***/





/*** constants and defines ***/
#define PoutScale 640L

/***********************************************************
 *  Set up Forward Velocity Coefficients
 *  Ts = 0.1ms 
 *  1/Ts = TsINV = 10000
 *  Ex = Inches/tooth = 1/3 = 8.47mm
 *  Uv = Scale from (teeth/Ts) to (mm/s) = 8.47/0.0001 = 84700(mm/sec)
 *  Max speed = 113 (in/sec) = 2872 (mm/sec)
 *  Min speed = 20mm/sec
 ************************************************************/
#define Smax  2872
#define Smin  20


/*************************************************************
 * Set up Turning Velocity Coefficients and limits
 * The Robot is 27.25inches wide wheel to wheel
 * When pivoting around center (i.e. left reverse, right forward)
 * the wheels create a circle with radius R = 13.625"
 * The max forward velocity is 113 inches/sec
 * The Forward Velocity is related to the rotation as follows:
 * R = 13.625 inches = 346mm
 * Vf = forward velocity --> maximum is 113 inches/sec
 * Vr = rotational Velocity
 * 1 rotation = 2Pi Radians
 * C = Circumference = 2Pi*R inches
 * Vr = Vf/2Pi*R (rotations/second)
 * Vr = Vf*2Pi/2Pi*R = Vf/R (radians/second)
 *
 * At Vf = 113inches/second Vr = 1.32 rotations/sec or 8.29 radians/sec (too fast)
 * Limit to 1/4 turn per second = Pi/2 radians/sec and solve for new Vf
 * Vrmax = Pi/2 = Vf/R;  Vfmax = 21 inches/second = 533.4mm/sec
 * Vrmax = 10*(Pi/2)(e-1)Radians/sec = 16 (tenths of radians/sec)  16 ~= 15.7 and is a "nice" base2 number
 * NOTE:  All calculations done using "Tenths of radians" must be divided by 10 to get the right answer.
 *  
 * Vfturnmax = R*Vrmax 
 *
 * The fastest we will ever turn will be to modify the left and right wheel speeds by Vfturnmax/2
 * Speedleft = Vfleft + Vfturnmax/2
 * Speedright = Vfright - Vfturnmax/2
 * 
 * ***********************************************************/

#define R 346
#define Vrmax 16
#define radScale 10


/*
 *  Connect the right drive motors to PWM01 on the RC.
 *  Connect the left  drive motors to PWM02 on the RC.
 */

#define rightPWM pwm01
#define leftPWM pwm02
#define JoystickOffset 127

/*** Define joystick driver variables ***/
// Define the gear ratio for high vs low gear (low is 1.59x slower
#define gearRatio 1.59



/*** local types ***/
typedef struct {
    signed long Esum;
    signed long Elast;
	signed int  VtargetLast;
} PIDState;

/*** static globals ***/
static int speedEhist = 0;
static int targetSpeed = 0;
static int targetTurningSpeed = 0;

static PIDState leftWheelState = {0,0, 0};
static PIDState rightWheelState = {0,0, 0};
static PIDState turningState = {0,0, 0};

static signed int iterateSpeedPID( signed int Vtarget, signed int Vactual, PIDState *state ) {
//	static const long Kp = 10L;
//	static const long Ki = 1L; //2L;
//	static const long Kd = -2L; //-2L;

	static const long Kp = 10L;
	static const long Ki = 1L; //2L;
	static const long Kd = -5L; //-2L;

	signed long E;
 	signed long Pout;
	signed int Pclamp;
	signed long Esum;


	E =  (signed long)(Vtarget - Vactual);

	
	if( (E < speedEhist) && (E > -speedEhist) ) {
		E = 0;
	}

	if( Vtarget == 0 && Vactual < 10 && Vactual > -10) {
		state->Esum = 0;
	}

	Esum = safeAddLong(state->Esum,E);

	Pout = (signed long)(Kp*E + Ki*(Esum) + Kd*(E-state->Elast));
	state->Elast = E;

	// scales 4000 into 127, then clamps us into -128..127
//	Pout = Pout/240L;
	Pout = Pout/1200L;
	if( Pout > 127L ) {
		Pclamp = 127;
	} else if( Pout < -127L ) {
		Pclamp = -127;
	} else {
		Pclamp = (signed int)Pout;
		state->Esum = Esum;
	}

	if( Pclamp < 1 && Vtarget >= 0 ) {
		Pclamp = 0;
	} else if( Pclamp > -1 && Vtarget <= 0 ) {
		Pclamp = 0;
	} 


	return Pclamp;

}


/*** functions ***/
void wheels_doMain(void) {

	static int iters =0;

	static const int Kp_turn = 1;
	static const int Ki_turn = 0;
	static const int Kd_turn = 0;

	int Vfturn  =0;
	int PLout   =0;
	int PRout   =0;
	signed int tempInt = 0;

	signed long E=0;

	// Guess at Current wheel direction based on current PWM values
//	int Rd = 1; //wheels_getDriveDirection(rightPWM);
//	int Rl = 1; //wheels_getDriveDirection(leftPWM);	

	// get actual speeds. (without direction)
#ifndef _USE_AVGSPEED
	int leftActualSpeed  = (signed int)speedometer_getSpeed(&speedometer_left);
	int rightActualSpeed = (signed int)speedometer_getSpeed(&speedometer_right);
#else
	int leftActualSpeed = (signed int)avgSpeed_getSpeed( &avgSpeed_left );
	int rightActualSpeed = (signed int)avgSpeed_getSpeed( &avgSpeed_right );
#endif
	

	signed int VTactual = leftActualSpeed - rightActualSpeed;
	
	long lTemp = (long)(targetTurningSpeed*1000L);
	int VTtarget = (int)(lTemp/289L);

	int VLtarget = targetSpeed;
	int VRtarget = targetSpeed;

//	//TurningSpeed PID

//	Vfturn = (long)iterateTurnPID( VTtarget, VTactual, &turningState );
//	VLtarget += Vfturn/2;
//	VRtarget += Vfturn/2;

	VLtarget -= VTtarget/2;
	VRtarget += VTtarget/2;
	//Left Speed PID 
	
	// compute next iteration of PID - returns -128..127
	//	adjust( &VLtarget, &leftActualSpeed, &leftWheelState );

	// correct the sign of actual speed if necessary
	if( VLtarget < 0 ) {
		leftActualSpeed = -leftActualSpeed;
	}

	// minimum speed
	if( VLtarget > 50  &&  VLtarget < 500 ) {
		VLtarget = 500;
	} else if( VLtarget < -50 && VLtarget > -500 ) {
		VLtarget = -500;
	} else if( VLtarget >= -50  &&  VLtarget <= 50 ){
		VLtarget = 0;
	}
	PLout = iterateSpeedPID( VLtarget, leftActualSpeed, &leftWheelState );
	// convert into range 0..255, and convert to unsigned char
	tempInt = PLout + (int)127;
	leftPWM = (unsigned char)(tempInt);
	if( ((leftPWM< 127) && (PLout > 0)) || ((leftPWM>127) && (PLout<0)) ) {
		leftPWM= 127;
	}
	

	//Right Speed PID 

	// compute next iteration of PID - returns -128..127
	//	adjust( &VRtarget, &rightActualSpeed, &rightWheelState );

	// correct the sign of actual speed if necessary
	if( VRtarget < 0 ) {
		rightActualSpeed = -rightActualSpeed;
	}

	// minimum speed
	if( VRtarget > 50  &&  VRtarget < 500 ) {
		VRtarget = 500;
	} else if( VRtarget < -50 && VRtarget > -500 ) {
		VRtarget = -500;
	} else if( VRtarget >= -50  &&  VRtarget <= 50 ) {
		VRtarget = 0;
	}

	PRout = iterateSpeedPID( VRtarget, rightActualSpeed, &rightWheelState );

	// convert into range 0..255, and convert to unsigned char
	tempInt = PRout + (int)127;
	rightPWM = (unsigned char)(tempInt);
	if( ((rightPWM < 127) && (PRout > 0)) || ((rightPWM>127) && (PRout<0)) ) {
		rightPWM = 127;
	}


	++iters;
	if( iters == 50 ) {
//		printf( "Lpwm, Rpwm, rSpeed, lSpeed, lESum, rESum: %d, %d, %d, %d, %d, %d\r", (unsigned int)leftPWM, (unsigned int)rightPWM,(int)rightActualSpeed, (int)leftActualSpeed, (int)leftWheelState.Esum, (int)rightWheelState.Esum );
		iters = 0;
	}

	if( ((VLtarget < 0) && (leftPWM > 127)) || ((VLtarget > 0) && (leftPWM < 127)) ) {
		printf("LEFT sign-inverse! VTrad, VTtarget, VTactual: %d, %d, %d \r L(Vtarget, Vactual, Pout, pwm): %d, %d, %d, %d \r R(Vtarget, Vactual, Pout, pwm): %d, %d, %d, %d \r\r", 
			targetTurningSpeed, VTtarget, VTactual, 
			VLtarget, leftActualSpeed, PLout, (unsigned char)leftPWM, 
			VRtarget, rightActualSpeed, PRout, (unsigned char)rightPWM
		);
	}
	if( ((VRtarget < 0) && (rightPWM > 127)) || ((VRtarget > 0) && (rightPWM < 127)) ){
		printf("RIGHT sign-inverse! VTrad, VTtarget, VTactual: %d, %d, %d \r L(Vtarget, Vactual, Pout, pwm): %d, %d, %d, %d \r R(Vtarget, Vactual, Pout, pwm): %d, %d, %d, %d \r\r", 
			targetTurningSpeed, VTtarget, VTactual, 
			VLtarget, leftActualSpeed, PLout, (unsigned char)leftPWM, 
			VRtarget, rightActualSpeed, PRout, (unsigned char)rightPWM
		);
	}
//		iters=0;
//	}

}

void wheels_setDriveSpeed( int a_targetSpeed ) {
	signed long Htemp = a_targetSpeed;

	if( a_targetSpeed != targetSpeed ) {
		Htemp = Htemp * Htemp;

		if( a_targetSpeed < 0 ) {
			Htemp = ((long)Htemp)/((int)((-a_targetSpeed)+1));
		} else {
			Htemp = ((long)Htemp)/((int)(a_targetSpeed+ (int)1));
		}
	
		speedEhist = (int) (Htemp/30L);

	}

	targetSpeed = a_targetSpeed;
}

void wheels_setTurningSpeed( int a_turningSpeed ) {
	targetTurningSpeed = a_turningSpeed;
}

void wheels_oneStickDriver( unsigned char a_x, unsigned char a_y, char a_isLowGear ) {
	static unsigned int iters = 0;

	static int highGearStick2mm = Smax/128;
	static int lowGearStick2mm = Smax/128/1.59;

	int x = (signed int)(((int)a_x) - ((signed int)JoystickOffset));
	int y = (signed int)(((int)a_y) - ((signed int)JoystickOffset));
	int speed;
	int turn;

	if( a_isLowGear ) {
		speed = y*lowGearStick2mm;
	} else {
		speed = y*highGearStick2mm;
	}
	turn = (int)((x*100)/((int)30));

	if( (speed < 100) && (speed > -100)) {
		speed = 0;
	}

	if( turn < 20 && turn > -20 ) {
		turn = 0;
	}

	if( iters == 50 ) {
//		printf("STICK (Ox, x, Oy, y): %u, %d, %u, %d, \r", (unsigned char)a_x, (signed int)x, (unsigned char)a_y, (signed int)y);
//		printf("(speed, turn): %d, %d \r", (int)speed, (int)turn);
		iters=0;
	}

	wheels_setDriveSpeed( speed );
	wheels_setTurningSpeed( turn*3 );

	++iters;
}

void wheels_twoStickDriver(char a_leftStick, char a_rightStick, char a_lowGear){
	// Scale factor to scale the max joystick value into mm/sec
	// highGearStick2mm ~= 11 = 2872/128
	// i.e. setSpeed = stickValue*highGearStick2mm in mm/sec
	static int highGearStick2mm = Smax/128;
	static int lowGearStick2mm = Smax/128/1.59;

	int turnSpeed=0;
	int speed=0;
	speed = (int)(((a_leftStick-JoystickOffset) + (a_rightStick-JoystickOffset)) >> 1);
	turnSpeed = (int)((a_leftStick-JoystickOffset) - (a_rightStick-JoystickOffset));

	if(a_lowGear){
		speed = speed*lowGearStick2mm;
		turnSpeed = turnSpeed*lowGearStick2mm*10/R;  // Convert turn speed to radians-1/sec
		// Stick range is -39,0,39 for equivalent of Vf = -544,0,544, and Vr = -16, 0 , 16
        // Note:  Turning speed currently uses about 20% of the total Vf scale before saturating at 16rad-1/sec
		// So, an additional scale factor may be needed to force this value scale larger
	}
	else{
		speed = speed*highGearStick2mm;
		turnSpeed = turnSpeed*highGearStick2mm*10/R;  // Convert turn speed to radians-1/sec
		// Stick range is -25, 0, 25 for equivalent of Vf = -544,0,544, and Vr = -16, 0 , 16
		// Note:  Turning speed currently uses about 20% of the total Vf scale before saturating at 16rad-1/sec
		// So, an additional scale factor may be needed to force this value scale larger
	}

	if( (speed < 50) && (speed > -50) ) {
		speed = 0;
	}

	wheels_setDriveSpeed( speed );
	wheels_setTurningSpeed( turnSpeed );
}


// Use this on the output of the turning speed PID to limit how fast we can turn to Vrmax
long limitTurningSpeed(long a_turningSpeed){
	if(a_turningSpeed < -Vrmax ){
		return(-Vrmax);
		}
	else if( a_turningSpeed > Vrmax ){
		return(Vrmax);
	}
	else{
		return(a_turningSpeed);
	    }
}


// Guess the actual wheel direction based on the current pwm values
int wheels_getDriveDirection(char a_pwm) {
	if( a_pwm > 127 ) {
		return (int)1;
	} else if ( a_pwm < 127 ) {
		return (int)-1;
	} else {
		return 0;
	}

}

long safeAddLong( long one, long two ) {
	long sum = one + two;

	if((two > 0) && (sum < one))
	{
		return(0x7FFFFFFFL);
	}
	else if((two < 0) && (sum > one))
	{
		return(0x80000000L);
	}			
	else
	{ 
		return(sum);
	}
}

// Accumulate safely without overflow signed numbers
int safeAdd(int one, int two)
{
	int sum;


	sum = one + two;

	if((two > 0) && (sum < one))
	{
		return(32767);
	}
	else if((two < 0) && (sum > one))
	{
		return(-32768);
	}			
	else
	{ 
		return(sum);
	}
}

// Shift signed chars to unsigned pwm 0-255 range unsigned chars
unsigned char safePWMs(long one, long two)
{
	long sum;

	sum = one + two;

	if(sum < 0)
	{
		return((unsigned char) 0);
	}
	else if(sum > 255 )
	{
		return((unsigned char) 255);
	}			
	else
	{ 
		return((unsigned char) sum);
	}
}

void wheels_moduleInitialize( void ) {
}
