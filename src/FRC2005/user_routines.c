/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#ifndef _FRC2004_
  #include "user_Serialdrv.h"
  #include <stdio.h>
#else
  #include "printf_lib.h"
#endif

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "NCSSM/interrupts.h"
#include "NCSSM/speedometer.h"
#include "NCSSM/wheels.h"
#include "NCSSM/arm.h"
#include "NCSSM/armOutput.h"
#include "NCSSM/aut1.h"

#define MID_COUNTER_VALUE 50

// analog values corresponding to each position on the aut mode selector
#define AMODE1 102
#define AMODE2 308
#define AMODE3 512
#define AMODE4 716
#define AMODE5 923

static unsigned int midCounter = MID_COUNTER_VALUE;
user_struct ur_data;

void SetupSystem( void ) {
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */
}


void SetupSystemModules( void ) {
	printf("initializing system modules...\r");

	interrupts_moduleInitialize();

	printf("system modules initialized.\r");
}

void SetupDriverModules( void ) {
	printf("initializing driver modules...\r");

#ifndef _USE_AVGSPEED
	speedometer_moduleInitialize();
#else
	avgSpeed_moduleInitialize();
#endif

	wheels_moduleInitialize();

	arm_moduleInitialize();
	armOutput_moduleInitialize();

	printf("driver modules initialized.\r");
}

void SetupControllerModules( void ) {
	printf("setting up controller modules...\r");

	printf("controller modules initialized.\r");
}

void EnableSystem( void ) {
	printf("enabling system...\r");

	interrupts_enable();

	printf("system enabled.\r");
}

/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
	/*** must call SetupSystem first ***/
	SetupSystem();

	SetupSystemModules();
	SetupDriverModules();
	SetupControllerModules();

	/* Add any other initialization code here. */
	
	EnableSystem();

	/* Add any other initialization code below here. */

	// initial gear is low
	relay1_fwd = 0;
	relay2_fwd = 1;
	relay1_rev = 0;
	relay2_rev = 0;

	// initialize ur_data
	// set aut_enable = 0 to disable all autonomous code
	// set simulate_aut = 1 to simulate autonomous during normal mode operation
	ur_data.aut_enable = 0;
	ur_data.simulate_aut = 0;
	ur_data.aut_reset = 1;
	ur_data.finger_preset_timer = 88;

	/* Add any other initialization code above here. */


  /*** required system setup code... place no user code below this line ***/
  Putdata(&txdata);             /* DO NOT CHANGE! */

#ifndef _FRC2004_
  Serial_Driver_Initialize();
#endif

  printf("\r\rIFI 2005 User Processor Initialized ...\r\r");  /* Optional - Print initialization message. */
  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
	static char initialPressure = 0;
	static char autMode = 0;
	int autModeSelector;
	static char state = 0;
	int sMaxLimitTemp;
	int sMinLimitTemp;



	Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */

	/* Add code here that you want to be executed every program loop. */
	
	arm_setShoulderSensor( Get_Analog_Value( ADC_CH0 ));

	switch( state ) {

		case 0:
			// reset state.

			arm_reset();
			armOutput_reset();
			wheels_setDriveSpeed(0);
			wheels_setTurningSpeed(0);

			if( autonomous_mode ) {
				state = 1;
			} else {
				state = 2;
			}
		break;

		case 1:
		if( autonomous_mode ) {
				// autonomous state
			if( ur_data.finger_preset_timer != 0 ) {
				// set the finger.
	
				if( initialPressure ) {
					--(ur_data.finger_preset_timer);
					armOutput_moveFingerDown();
				}
			} else {
				armOutput_stopFinger();
				if( autMode == 0 ) {
					autModeSelector = Get_Analog_Value( ADC_CH1 );
					if( autModeSelector <= AMODE1 ) {
						autMode = 1;
					} else if( autModeSelector > AMODE1 && autModeSelector <= AMODE2 ) {
						autMode = 2;
					} else if( autModeSelector > AMODE2 && autModeSelector <= AMODE3 ) {
						autMode = 3;
					} else if( autModeSelector > AMODE3 && autModeSelector <= AMODE4 ) {
						autMode = 4;
					} else if( autModeSelector > AMODE4 && autModeSelector <= AMODE5 ) {
						autMode = 5;
					} else {
						autMode = 6;
					}
					printf("selected aut mode: %d", autMode);
				} else {
					switch( autMode ) {
						case 1:
							aut1_doMain();
							break;
	
						case 2:
	//						aut2_doMain();
							break;
	
						case 3:
	//						aut3_doMain();
							break;
	
						case 4:
	//						aut4_doMain();
							break;
	
						case 5:
	//						aut5_doMain();
							break;
					}
				}
			}
		} else {
			state = 0;
		}
		break;

		case 2:
		if( !autonomous_mode ) {
			autMode = 0;
			// manual state
			// do user-mode input
	
			wheels_oneStickDriver( p1_x, p1_y, 1 );
	
			/**
			* using the following pins for arm and preset inputs
			*  shoulder:
			*    toggle up - (p2_y > 140) - thumbpad up
		    *    toggle down - (p2_y < 100) - thumbpad down
		    *  finger:
		    *    toggle up - (p2_wheel >140) - button 5
		    *    toggle down - (p2_aux >140) - button 6
		    *  presets:
		    *    P1 - p2_sw_trig - button 1
		    *    p2 - p2_sw_top  - button 2
			*    P3 - p2_sw_aux1 - button 3
			*    P4 - P2_sw_aux2 - button 4
			**/
			arm_doShoulderOI(
				(p2_y > 140), (p2_y < 100),
				p2_sw_trig, p2_sw_top, p2_sw_aux1, p2_sw_aux2
			);
		
			arm_doFingerOI(
				(p2_wheel > 140), (p2_aux > 140)
			);

		} else {
			state = 0;
		}
		break;

		default:
			state = 0;
	}

	// do output sequences...
	armOutput_doShoulderOutput();
	armOutput_doFingerOutput();
	wheels_doMain();

// Compressor code
	/**
	 * rc_dig_in03 - signals low pressure in system
	 * relay3_fwd - turns on compressor
	 */
	if(!rc_dig_in03)
	{
		relay3_fwd=1;
		relay3_rev=0;	
	}
	else
	{
		initialPressure = 1;
		relay3_fwd=0;
		relay3_rev=0;
	}
// end Compressor Code


	if( midCounter == 0 ) {
		/** This runs every (MID_COUNTER_VALUE*26.6)ms
		 * you can put print statements in here, and anything
		 * else that you want to run periodically
		 */

		/** P2 controller printouts **/
//		printf("(x,y) %d,%d\r", (unsigned int)p2_x, (unsigned int)p2_y);
/**		printf("(wheel) %d\r", (unsigned int)p2_wheel);
		printf("(aux, sw_aux1, sw_aux2) %d,%d,%d\r", (unsigned int)p2_aux, (unsigned int)p2_sw_aux1, (unsigned int)p2_sw_aux2);
		printf("(sw_trig, sw_top) %d, %d\r\r", (unsigned int)p2_sw_trig, (unsigned int)p2_sw_top);
*/

		sMaxLimitTemp = (int)shoulderMaxLimit;
		sMinLimitTemp = (int)shoulderMinLimit;

		/** SHOULDER PRINTOUTS **/
		printf("S: PWM,Pos[Sens],[l/u], [L+,L-]: \r   %d, %d[%d] [%d/%d] [%d,%d]\r", (int)pwm03, (int)arm_getShoulderPosition(), (int)shoulderPosSensor, (unsigned int)relay4_fwd, (unsigned int)relay5_fwd, sMaxLimitTemp, sMinLimitTemp);

		/** FINGER PRINTOUTS **/
		printf("F: PWM [l/u]:\r   %d [%d/%d]\r", (int)pwm04, (unsigned int)relay6_fwd, (unsigned int)relay7_fwd);

		//printf("speed: (l,r) %d, %d", (int)speedometer_getSpeed(&speedometer_left), (int)speedometer_getSpeed(&speedometer_right));
	}


	// MID COUNTER
	if( midCounter == 0 ) {
		midCounter = MID_COUNTER_VALUE;
	} else {
		--midCounter;
	}

	Putdata(&txdata);             /* DO NOT CHANGE! */
}



