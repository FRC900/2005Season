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

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "printf_lib.h"
#include "receiver.h"


/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/
extern unsigned long Wheel_Clock;					//fast clock for finding encoder stripe periods
extern unsigned int wheel_r_count;					//count of lines on right encoder
extern unsigned int wheel_l_count;					//count of lines on left encoder
unsigned char p1_x_old=127;							//x coordinate of last loop
unsigned char p1_y_old=127;							//y coordinate of last loop
unsigned char HATCommand=0;							//hate toggle value
unsigned char casterToggle=0;						//caster toggle value
unsigned char whinchCommand=0;						//winch stopper toggle value
unsigned char upToggle=0;							//winch upward toggle value
unsigned char downToggle=0;							//winch downward toggle value
unsigned char winch_piston_toggle=0;				//winch piston toggle value
unsigned char whinchToggle=0;						//winch manual down toggle
unsigned char whinchCounter=0;						//counter variable for winch encoder
unsigned char whinchColor_old=0;					//last color of line on encoder
unsigned long counter=0;							//holds loop counts
unsigned char debug_cycle = 0;						//is true every so many loops and tells the program to print out debug info
unsigned int XPER=78;								//Percentage knockoff variable for x
unsigned int YPER=80;								//Percentage knockoff variable for y

//variables for PID calcs
unsigned int setPointL_old=0;						//old speed of left motor that it was supposed to be					
unsigned int setPointR_old=0;						//old speed of right motor that it was supposed to be
unsigned int processPointL_old=0;					//old calculated speed of left motor
unsigned int processPointR_old=0;					//old calculated speed of right motor
int errorL_old=0;									//old difference between desired speed and real speed on left side
int errorR_old=0;									//old difference between desired speed and real speed on right side
unsigned char pwm15_old=127;						//old value of left motors
unsigned char pwm13_old=127;						//old value of right motors
unsigned char periodTable[14];						//interpolation table for pwm speed -> period
/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}

//---------------------------------------------------------------------------------
void inputModifier(unsigned char *p1_x_un,unsigned char *p1_y_un)
{
	unsigned int x=*p1_x_un;
	unsigned int y=*p1_y_un;	
	unsigned int constk=(int)20*ACCCOEFF;
	unsigned int xdist=(signed int)x-127;
	unsigned int ydist=(signed int)y-127;

	unsigned int int1;
	unsigned int int2;
	unsigned int int3;
	
		if(x<(unsigned int)127)
		{
			xdist=((unsigned int)255-x)-(unsigned int)127;
			x=(unsigned int)127-((unsigned int)XPER*xdist/(unsigned int)100);
		}
		else
		{
			int1=(unsigned int)XPER*(unsigned int)xdist;
			int2=int1/(unsigned int)100;
			int3=int2+(unsigned int)127;
	
			if(debug_cycle) printf("int1: %d\n",int1);
			if(debug_cycle) printf("int2: %d\n",int2);
			if(debug_cycle) printf("int3: %d\n",int3);
	
			x=int3;
		}
		
		x=quadMod(x);

	//if(relay4_fwd==1)
	//{
		if(y<(unsigned int)127)
		{
			ydist=((unsigned int)255-y)-(unsigned int)127;
			y=(unsigned int)127-((unsigned int)YPER*ydist/(unsigned int)100);
		}
		else
		{
			int1=(unsigned int)YPER*(unsigned int)ydist;
			int2=int1/(unsigned int)100;
			int3=int2+(unsigned int)127;
		
			if(debug_cycle) printf("int1: %d\n",int1);
			if(debug_cycle) printf("int2: %d\n",int2);	
			if(debug_cycle) printf("int3: %d\n",int3);
	
			y=int3;
		}
	//}

	//if((x<142) && (x>102)) x=127;
	//if((y<142) && (y>102)) y=127;

	//if(debug_cycle) printf("Xo:	%d	Yo:	%d\n",(int)p1_x_old,(int)p1_y_old);
	//if(debug_cycle) printf("Xb4:	%d	Yb4:	%d\n",(int)x,(int)y);

	y=p1_y_old+(ACCCOEFF*(2000+y-p1_y_old)/100) - constk;  //Ramping function
	x=p1_x_old+(ACCCOEFF*(2000+x-p1_x_old)/100) - constk;  //Ramping function

	//if(debug_cycle) printf("Xa:	%d	Ya:	%d\n",(int)x,(int)y);
	p1_y_old=y;	
	p1_x_old=x;

    //if(debug_cycle) printf("X1: %d  Y1: %d",(int)x,(int)y);

    // remove the deadband of from 124..132 (dead zone code
    recalc_for_deadband(&x);
    recalc_for_deadband(&y);

    //if(debug_cycle) printf("  X2: %d  Y2: %d\n",(int)x,(int)y);

	*p1_x_un=x;
	*p1_y_un=y;
}

#define HALF_DEADBAND 8

void recalc_for_deadband(unsigned int *n)
{
    unsigned int v = *n;

   //if (debug_cycle) printf("v => %d\n", (unsigned int)v );

    if (v < (unsigned int)(127-HALF_DEADBAND))
    {
       //if (debug_cycle) printf("A\n");
       v = v + HALF_DEADBAND;
    }
    else if (v > (unsigned int)((int)127+HALF_DEADBAND))
    {
       //if (debug_cycle) printf("B\n");
       v = v - HALF_DEADBAND;
    }
    else
    {
		v = 127;
    }

    *n = v;
}
//-------------------------------------------------------------------------------
unsigned int quadMod(unsigned int x)
{
	long temp1=x-127L;
	long temp2=temp1*temp1;
	long temp3=temp2/127L;
	unsigned int temp4=255-((unsigned int)(temp3+127L));
	if(x<127) temp4=255-temp4;
	return(temp4);
}
//-------------------------------------------------------------------------------
/*******************************************************************************
* FUNCTION NAME: motorModifier
* PURPOSE:       This is a modifier for the given motor speed to make the stick
*				 or less sensitive as desired
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument				Type    		IO	Description
*     --------				-----------		--	-----------
*     unModified			unsiged char	I	---	
* RETURNS:       void
*******************************************************************************/
unsigned char motorModifier(unsigned char unModified)
{
	return(unModified);												//linear
	
	/*long temp1=unModified-127L;
	long temp2=temp1*temp1;
	long temp3=temp2/127L;
	unsigned char temp4=255-((unsigned char)(temp3+127L));
	if(unModified<127) temp4=255-temp4;
	//return(temp4);*/

	/*long temp1=unModified-127L;//(int)(((float)unModified-127.0)*((float)unModified-127.0)*((float)unModified-127.0))/16129L;
	long temp2=temp1*temp1*temp1;
	long temp3=temp2/16129L;
	//return(255-((unsigned char) (temp3+127L)));	//cubic*/
}
//-----------------------------------------------------------------------------------
void relayControl(void)
{
//-----------Compressor Code--------------
	if(!rc_dig_in01)
	{
		relay6_fwd=1;
		relay6_rev=0;	
	}
	else
	{
		relay6_fwd=0;
		relay6_rev=0;
	}
//-----------Button Handler---------------

//	if(p1_sw_trig)
//	{
//		relay4_fwd=1;
//		relay4_rev=0;
//	}
//	else
//	{
//		relay4_fwd=0;
//		relay4_rev=1;
//	}

	if(p1_sw_trig)
	{
		//printf("P1 Trig\n");
		if(casterToggle<2)
		{
			if(!casterToggle)
			{
				relay4_fwd=1;
				relay4_rev=0;
				casterToggle=3;
			}
			else
			{
				relay4_fwd=0;
				relay4_rev=1;
				casterToggle=2;
			}
		}
		
	}
	else
	{
		if(casterToggle>1)casterToggle=casterToggle-2;
	}
	
	if(p1_sw_top)
	{
	//printf("P1 Top\n");
		XPER=78;
		YPER=80;
	}

	if(p1_sw_aux1)
	{
		//printf("P1 Aux1\n");
		YPER=100;
		XPER=78;
	}

	if(p1_sw_aux2)
	{
		//printf("P1 Aux2\n");
		XPER=60;
		YPER=50;
	}
/*
	HATCommand Reference
	0-Center
	1-Top
	2-Right
	3-Down-Right
	4-Down
	5-Down-Left
	6-Left
*/
	if(p1_wheel>40 && p1_wheel<50)		//HAT Center
	{
		HATCommand=0;
	}
	
	if(p1_wheel>250 && p1_wheel<260)		//HAT Top Row
	{
		if(HATCommand!=1)
		{
			//printf("P1 HAT Top\n");
			relay2_fwd=1;
			relay2_rev=0;
			relay3_fwd=1;
			relay3_rev=0;
		}
		HATCommand=1;
	}
	
	if(p1_wheel>90 && p1_wheel<100)		//HAT Left
	{
		if(HATCommand!=6)
		{
			//printf("P1 HAT Left\n");
			if(relay2_fwd==0)
			{
				relay2_fwd=1;
				relay2_rev=0;
			}	
			else
			{
				relay2_fwd=0;
				relay2_rev=1;
			}
		}
		HATCommand=6;
	}
	
	if(p1_wheel>190 && p1_wheel<209)		//HAT Right
	{
		if(HATCommand!=2)
		{
			//printf("P1 HAT Right\n");
			if(relay3_fwd==0)
			{
				relay3_fwd=1;
				relay3_rev=0;
			}
			else
			{
				relay3_fwd=0;
				relay3_rev=0;
			}
		}
		HATCommand=2;
	}
	
	if(p1_wheel>140 && p1_wheel<160)		//HAT Down
	{	
		if(HATCommand!=4)
		{
			//printf("P1 HAT Down\n");
			relay2_fwd=0;
			relay2_rev=1;
			relay3_fwd=0;
			relay3_rev=0;
		}
		HATCommand=4;
	}
	
	if(p1_wheel>160 && p1_wheel<170)		//HAT Down-Left
	{
		if(HATCommand!=5)
		{
			//printf("P1 HAT Down-Left\n");
		}
		HATCommand=5;
	}
	
	if(p1_wheel>210 && p1_wheel<215)		//HAT Down-Right
	{
		if(HATCommand!=3)
		{
			//printf("P1 HAT Down-Right\n");
		}
		HATCommand=3;
	}

	if(p2_x>140)
	{
		//printf("P2 X\n");
		if(whinchToggle==0)
		{
			if(relay1_fwd==1)
			{
				relay1_fwd=0;
				relay1_rev=0;
			}
			else
			{
				relay1_fwd=1;
				relay1_rev=0;
			}
		}
		whinchToggle=1;
	}
	else
	{
		whinchToggle=0;
	}
	
	if(p2_y>140)
	{
		
	}

	if(p2_wheel>140)
	{
		//printf("P2 Wheel\n");
		relay5_fwd=1;
		relay5_rev=0;
	}
	else
	{
		relay5_fwd=0;
		relay5_rev=0;
	}
	
	if(p2_aux>140)
	{
		if(winch_piston_toggle<2)
		{
			if(!winch_piston_toggle)
			{
				relay3_fwd=0;
				relay3_rev=1;
				winch_piston_toggle=3;
			}
			else
			{
				relay3_fwd=1;
				relay3_rev=0;
				winch_piston_toggle=2;
			}
		}
	}
else
{
	if(winch_piston_toggle>1)winch_piston_toggle=winch_piston_toggle-2;
	}

	if(p2_sw_trig)
	{
		//printf("P2 Trig\n");
		pwm01=0;
	}
	else
	{
		if(whinchCommand==0)
		{
			pwm01=127;
		}
	}
	
	if(p2_sw_top) //if pressing this button
	{
		if(!p2_sw_aux2) //and not pressing this button at same time
		{
			if(downToggle==0) //only clicking once, toggle (ignores other 'clicks' until button is released)
			{
				if(pwm01==127) 
				{
					whinchCommand=2;  //winch command for going down (1 is going up) and (0 is stop)
				}
				else
				{
					whinchCommand=0;  //else that you stop it
				}
			}
		}
		else
		{
			whinchCommand=0;  //if that button is being pressed at the same time then stop
		}
		downToggle=1; //toggles anytime the button is hit
	}
	else
	{
		downToggle=0; //otherwise toggle is set to 0
	}
	
	if(p2_sw_aux1)
	{
		//printf("P2 Aux1\n");
	}
	
	if(p2_sw_aux2)
	{
		printf("P2 Aux2\n");
		if(!p2_sw_top)
		{
			if(upToggle==0)
			{
				if(pwm01==127)
				{
					whinchCommand=1;
				}
				else
				{
					whinchCommand=0;
				}
			}
		}
		upToggle=1;
	}
	else
	{
		upToggle=0;
	}

	if(whinchCommand!=0)
	{
		if(whinchCommand==1)
		{
			if(whinchCounter<158) //this is where the number of 'ticks' gets changed
			{
				pwm01=254;
				if(whinchColor_old!=rc_dig_in06) whinchCounter++;
			}
			else
			{
				pwm01=127;
				whinchCommand=0;
			}
		}
		else
		{
			if(whinchCounter>0)
			{
				pwm01=0;
				if(whinchColor_old!=rc_dig_in06) whinchCounter--;
			}
			else
			{
				pwm01=127;
				whinchCommand=0;
			}
		}
	}
	else
	{
		pwm01=127;
	}
	
	if(p2_sw_aux1)
	{
		pwm01=0;
		if(whinchColor_old!=rc_dig_in06 && whinchCounter!=0) whinchCounter--;
	}
	else
	{
		if(whinchCommand==0)
		{
			pwm01=127;
		}
	}

	if(debug_cycle) printf("winch: %d",(int)whinchCounter);
	whinchColor_old=rc_dig_in06;
}
//-----------------------------------------------------------------------------------
void relayInit(void)
{
	relay1_fwd=1;		//Spool Pin
	relay1_rev=0;
	relay2_fwd=0;		//Left Wing
	relay2_rev=1;
	relay3_fwd=0;		//Right Wing
	relay3_rev=0;
	relay4_fwd=0;		//Caster
	relay4_rev=1;
	relay5_fwd=0;		//10pt Knocker
	relay5_rev=0;
	relay6_fwd=0;		//Compressor
	relay6_rev=0;
	whinchColor_old=rc_dig_in06;
}
//------------------------------------------------------------------------------------
unsigned char period(unsigned int period, unsigned char isForward)
{
	unsigned char i=0;
	unsigned char i_hold=0;
	unsigned int step;
	unsigned int start;
	unsigned char toReturn=127;
	while((unsigned int)periodTable[i]>period && i<13)
	{
		i++;
	}
	step=(periodTable[i-1]-periodTable[i])/10;
	start=periodTable[i-1];
	i_hold=i-1;
	i=0;
	while(start+(unsigned int)i*step>period)
	{
		i++;
	}
	if(isForward==1)
	{
		toReturn=127+i_hold*10+i;
	}
	else
	{
		toReturn=127-i_hold*10-i;
	}
	return(toReturn);
}
//------------------------------------------------------------------------------------
void encoderMod(void)
{
	unsigned int count=(unsigned int)(Wheel_Clock);
	unsigned char processPointL;
	unsigned char processPointR;
	unsigned int setPointL=pwm15;
	unsigned int setPointR=pwm13;
	unsigned char isForward=1;
	float Ki=1;
	float Kp=1;
	int errorL;
	int PL;
	int IL;
	int errorR;
	int PR;
	int IR;

	printf("Left Period: %d\n",(int)count/wheel_l_count);
	printf("Right Period: %d\n",(int)count/wheel_r_count);

	if(wheel_l_count>0)
	{
		if(pwm15<127)
		{
			isForward=0;
		}
		processPointL=period((unsigned int)(count/wheel_l_count),isForward);
		isForward=1;
	}
	else
	{
		processPointL=period((unsigned int)512,(unsigned char)1);
	}
	if(wheel_r_count>0)
	{
		if(pwm13<127)
		{
			isForward=0;
		}
		processPointR=period((unsigned int)(count/wheel_r_count),isForward);
		isForward=1;
	}
	else
	{
		processPointR=period((unsigned int)512, 1);
	}
	errorL=setPointL-processPointL;
	PL=Kp*(errorL-errorL_old);
	errorL_old=errorL;
	IL=Ki*Wheel_Clock*errorL;
	pwm15=pwm16=pwm15_old+PL+IL;


	errorR=setPointR-processPointR;
	PR=Kp*(errorR-errorR_old);
	errorR_old=errorR;
	IR=Ki*Wheel_Clock*errorR;
	pwm13=pwm14=pwm13_old+PR+IR;

	Wheel_Clock=0;
	wheel_l_count=wheel_r_count=0;
}
//-----------------------------------------------------------------------------------------------------
void tableInit()
{
	periodTable[0]=512;
	periodTable[1]=512;
	periodTable[2]=392;
	periodTable[3]=202;
	periodTable[4]=102;
	periodTable[5]=96;
	periodTable[6]=92;
	periodTable[7]=88;
	periodTable[8]=80;
	periodTable[9]=76;
	periodTable[10]=72;
	periodTable[11]=65;
	periodTable[12]=61;
	periodTable[13]=61;
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
  rom const	char *strptr = "IFI User Processor Initialized ...";

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

  /* Add any other initialization code here. */
//initialize interupts here

  relayInit();
  tableInit();
	
  Initialize_Serial_Comms();   

  Putdata(&txdata);             /* DO NOT CHANGE! */

  printf("%s\n", strptr);       /* Optional - Print initialization message. */

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
  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */
  if(!autonomous_mode) Disable_Receiver();

  counter++;
  if ((counter&0xf)==0)
  {
	debug_cycle = 1;
    printf("\n-----------\n");
  }
  else
	debug_cycle = 0;

  //if(debug_cycle) printf("X: %d  Y: %d\n",(int)p1_x,(int)p1_y);
  if(debug_cycle) printf("Dist from front: %d\n",(int)Get_Analog_Value(rc_ana_in01));

  Default_Routine();  /* Optional.  See below. */

  relayControl();

  if(counter%3==0 && counter!=0) 
  {
     //encoderMod();
  }

  Generate_Pwms(pwm13,pwm14,pwm15,pwm16);
  
  Putdata(&txdata);             /* DO NOT CHANGE! */
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{ 
 /*---------- 1 Joystick Drive ----------------------------------------------
  *--------------------------------------------------------------------------
  *  This code mixes the Y and X axis on Port 1 to allow one joystick drive. 
  *  Joystick forward  = Robot forward
  *  Joystick backward = Robot backward
  *  Joystick right    = Robot rotates right
  *  Joystick left     = Robot rotates left
  *  Connect the right drive motors to PWM13 and/or PWM14 on the RC.
  *  Connect the left  drive motors to PWM15 and/or PWM16 on the RC.
  */ 
   if(debug_cycle)printf("Xraw: %d	Yraw: %d\n",(int)p1_x,(int)p1_y);
   inputModifier(&p1_x,&p1_y);
   if(debug_cycle)printf("Xo: %d	Yo: %d\n",(int)p1_x,(int)p1_y);
   pwm16 = pwm15 = motorModifier(Limit_Mix(2000 + p1_x + p1_y - 127));
   pwm14 = pwm13 = motorModifier(Limit_Mix(2000 + p1_x - p1_y + 127));
   
   pwm13=pwm14=255-pwm14;

   //if(debug_cycle)printf("X: %d	Y: %d\n",(int)p1_x,(int)p1_y);
   //if(debug_cycle)printf("Left: %d	Right: %d\n",(int)pwm01,(int)pwm02);
 
 /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *------------------------------------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
  
  if (user_display_mode == 0) /* User Mode is Off */
    
  { /* Check position of Port 1 Joystick */
    if (p1_y >= 0 && p1_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON  */
    }
    else if (p1_y >= 125 && p1_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON */
    }
    else if (p1_y >= 216 && p1_y <= 255)
    {                     /* Joystick is in full forward position*/
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON  */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }  /*END Check position of Port 1 Joystick
    
    /* Check position of Port 2 Y Joystick 
           (or Port 1 X in Single Joystick Drive Mode) */
    if (p2_y >= 0 && p2_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm2_green  = 0;    /* Turn pwm2 green LED - OFF */
      Pwm2_red  = 1;      /* Turn pwm2 red LED   - ON  */
    }
    else if (p2_y >= 125 && p2_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON */
      Pwm2_red  = 1;      /* Turn PWM2 red LED   - ON */
    }
    else if (p2_y >= 216 && p2_y <= 255)
    {                     /* Joystick is in full forward position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON  */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm2_green  = 0;    /* Turn PWM2 green LED - OFF */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }  /* END Check position of Port 2 Joystick */
    
    /* This drives the Relay 1 and Relay 2 "Robot Feedback" lights on the OI. */
    Relay1_green = relay1_fwd;    /* LED is ON when Relay 1 is FWD */
    Relay1_red = relay1_rev;      /* LED is ON when Relay 1 is REV */
    Relay2_green = relay2_fwd;    /* LED is ON when Relay 2 is FWD */
    Relay2_red = relay2_rev;      /* LED is ON when Relay 2 is REV */

    Switch1_LED = (int)relay4_fwd;
    Switch2_LED = (int)relay4_fwd;
    Switch3_LED = (int)relay4_fwd;
    
  } /* (user_display_mode = 0) (User Mode is Off) */
  
  else  /* User Mode is On - displays data in OI 4-digit display*/
  {
    User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
  }   
  
} /* END Default_Routine(); */


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
