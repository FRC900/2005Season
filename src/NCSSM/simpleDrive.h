#ifndef __simpleDrive_h
#define __simpleDrive_h

//*** function prototypes ***/

// accepts a left and right PWM value, and drives
// each wheel at the specified value.  Any needed "buffer zone"
// around 127 (stopped) should be included as logic inside
// this function.     
void simpleDrive_drive( unsigned char pwmLeft, unsigned char pwmRight );

// returns the current drive direction of the robot
// this is derrived from the current value of the
// actual PWM pins.  
// returns:
//   0: if the robot is stopped, or pivoting
//   1: if the robot is being driven forward
//  -1: if the robot is being driven backward
int simpleDrive_getDriveDirection( unsigned char a_pwm );	

// accepts values for the 'y' axis of left stick and right stick,
// and scales those values according to the internal ramping table.
// a different table should be used if isLowGear is true.  This
// allows for a different response in low gear vs. high gear.
//
// the scaled values should then be sent to the wheels by calling
// simpleDrive_drive( leftPWM, rightPWM ).
//
// This method will be called from the main driver loop, every
// ~26.6 ms.
void simpleDrive_handleTwoStickInput(unsigned char a_leftStick, unsigned char a_rightStick, unsigned char isLowGear);

// this is called once at system setup.  It should initialize
// any necessary internal state, and then call the simpleDrive_reset()
// method.
void simpleDrive_moduleInitialize( void );

// this method should reset the internal state of this module (if any)
// so that the robot is fully stopped.  This method may be called
// at any time.  Specifically, this method may be called upon
// transition from autonomous mode, to manual mode.
void simpleDrive_reset( void );

#endif

