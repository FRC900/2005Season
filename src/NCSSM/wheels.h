#ifndef __wheels_h
#define __wheels_h

#define PIDDataTableSize 25
typedef struct {
	unsigned int currentOffset;
	signed long Pout[ PIDDataTableSize ];
	unsigned char PWM[ PIDDataTableSize ];
	signed long Esum[ PIDDataTableSize ];
} PIDDataTable;

//*** function prototypes ***/

// recomputes pwm values based on feedback
// and current driver inputs.
// called once every driver cycle.
// Driver cycle speed is defined by the Ts clock (currently .0001s)
void wheels_doMain( void );
void wheels_doMainTest( PIDDataTable * );

// changes the set-point for the robot's speed.
// speed is specified in mm/s.  positive values
// drive the robot forward, negative values drive
// the robot backward.  Use 0, to request a complete
// stop.
void wheels_setDriveSpeed( int a_targetSpeed );

// changes the set-point for the robot's turning speed.
// turning speed is computed based on this number and it 
// is reflected as a diference between the left 
// wheel speed and the right wheel speed.  Turning speed
// should be specified in 10^-1Radians/second.
// Use 0 for no turning
// negative turns left
// positive turns right
void wheels_setTurningSpeed( int a_turningSpeed );

// returns the current drive direction of the robot
// this is derrived from the current value of the
// actual PWM pins.  
// returns:
//   0: if the robot is stopped, or pivoting
//   1: if the robot is being driven forward
//  -1: if the robot is being driven backward
int wheels_getDriveDirection( char a_pwm );	

//  Two joystick driver which sets the forward/reverse speed and
//  the turning speed from "y" inputs of two joysticks
void wheels_twoStickDriver(char a_leftStick, char a_rightStick, char lowGear);
void wheels_oneStickDriver(unsigned char a_x, unsigned char a_y, char lowGear);

void wheels_moduleInitialize( void );

#endif

