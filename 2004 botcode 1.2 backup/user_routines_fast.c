/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "receiver.h"
#include "printf_lib.h"

/*******************************************************************************
*
*	FUNCTION:		InterruptVectorLow()
*
*	PURPOSE:		Installs the low priority interrupt code at the low
*					priority interrupt vector, which is a fixed place in
*					memory where the microcontroller will start executing
*					code when it detects an interrupt condition. Because
*					this place in memory, at address 24/0x18, is intended 
*					to contain only a very small amount of code, general
*					practice is to place a "goto" instruction here that
*					will point to the real interrupt handler somewhere else
*					in memory. More information on interrupts can be found
*					in section nine [87] of the PIC18F8520 data sheet.
* 
*	CALLED FROM:	Called in response to a hardware generated interrupt
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR

void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}

#pragma code

/*******************************************************************************
*
*	FUNCTION:		InterruptHandlerLow()
*
*	PURPOSE:		Determines which individual interrupt handler
*					should be called, clears the interrupt flag and
*					then calls the interrupt handler.
* 
*	CALLED FROM:	InterruptVectorLow()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
* 
*******************************************************************************/
#pragma interruptlow InterruptHandlerLow

void InterruptHandlerLow ()     
{                               
	unsigned char Port_B;
	unsigned char Port_B_Delta;       
	static unsigned char Old_Port_B = 0xFF; // state of port b the last time
											// this function was called  
	
	if (INTCONbits.TMR0IF) // timer 0 Interrupt
	{
		INTCONbits.TMR0IF = 0; // clear the interrupt flag
		//Timer_0_Int_Handler();
	}

	else if (PIR1bits.TMR1IF) // timer 1 interrupt - used by the IR receiver
	{
		PIR1bits.TMR1IF = 0; // clear the interrupt flag
		Timer_1_Int_Handler(); // call the timer 1 interrupt handler (in receiver.c)
	}  

	else if(PIR1bits.TMR2IF)
	{
		PIR1bits.TMR2IF = 0;
		Timer_2_Int_Handler();
	}  

	else if (INTCON3bits.INT2IF) // external interrupt 1
	{ 
		INTCON3bits.INT2IF = 0; // clear the interrupt flag
		// if used, call the interrupt 1 handler here
	}

	else if (INTCON3bits.INT3IF) // external interrupt 2
	{
		INTCON3bits.INT3IF = 0; // clear the interrupt flag
		// if used, call the interrupt 2 handler here
	}

	else if (INTCONbits.RBIF) // external interrupts 3 through 6
	{
		Port_B = PORTB; // remove the "mismatch condition" by reading port b            
		INTCONbits.RBIF = 0; // clear the interrupt flag
		Port_B_Delta = Port_B ^ Old_Port_B; // determine which bits have changed
		Old_Port_B = Port_B; // save a copy of port b for next time around
		
		if(Port_B_Delta & 0x10) // did external interrupt 3 change state? - IR sensor 1
		{
			Int_3_Handler(Port_B & 0x10 ? 1 : 0); // call the interrupt 3 handler (in receiver.c)
		}

		if(Port_B_Delta & 0x20) // did external interrupt 4 change state? - IR sensor 2
		{
			Int_4_Handler(Port_B & 0x20 ? 1 : 0); // call the interrupt 4 handler (in receiver.c)
		}

		if(Port_B_Delta & 0x40) // did external interrupt 5 change state? - IR sensor 3
		{
			Int_5_Handler(Port_B & 0x40 ? 1 : 0); // call the interrupt 5 handler (in receiver.c)
		}
		
		if(Port_B_Delta & 0x80) // did external interrupt 6 change state? - IR sensor 4
		{
			Int_6_Handler(Port_B & 0x80 ? 1 : 0); // call the interrupt 6 handler (in receiver.c)
		}
	}
}

void set_left_motors(unsigned char n)
{
	pwm15 = n;
	pwm16 = n;
}

void set_right_motors(unsigned char n)
{
	pwm13 = n;
	pwm14 = n;
}

// use Jeff finite state machine code
#define TUNING_JB 1

// use previous method
//#define TUNING_JB 0

/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
//INSIDE user_routines_fast.c
void User_Autonomous_Code(void)
{
#define ACTION_GO_FORWARD 0
#define ACTION_EXTEND_RAM 2
#define ACTION_LEAVE_RAM_OUT 3
#define ACTION_RETRACT_RAM 4
#define ACTION_CHARGE_FORWARD 5
#define ACTION_STOP_FORWARD 6
  unsigned char task=ACTION_GO_FORWARD;
  unsigned int old_clock[7];
  unsigned int clock=0;
  unsigned int clock2=0;
  unsigned char old_pwmleft=127;
  unsigned char old_pwmright=127;
  unsigned char hitBall=0;
#if TUNING_JB
#define STATE_GO_FOR_AWHILE 0
#define STATE_GO_UNTIL_LINE 1
#define STATE_EXTEND_RAM 2
#define STATE_LEAVE_RAM_OUT 3
#define STATE_RETRACT_RAM 4
#define STATE_CHARGE_FORWARD 5
#define STATE_ALL_STOP 7
  unsigned char state;
  unsigned char line_seen = 0;
  unsigned char banner_count = 0;
#endif

  for(task;task<7;task++)
  {
  	old_clock[task]=0;
  }

#if TUNING_JB
  state = STATE_GO_FOR_AWHILE;
  printf("entering STATE_GO_FOR_AWHILE\n");
#else
  task=ACTION_GO_FORWARD;
#endif

  while(autonomous_mode)
  {
#if TUNING_JB
	if (rc_dig_in03)
	{
		if (banner_count<16)
			banner_count++;
		else
			line_seen = 1;
	}
	else
	{
		banner_count = 0;
	}
#endif

    if(statusflag.NEW_SPI_DATA) //26.2ms loop
    {
        Getdata(&rxdata);   // bad things will happen if you move or delete this


#define TUNING_POKE_BALL 1
 
//'left' need to be greater than 'right' values to go straight

#if 0 /* set for playing on the left position */
#  define SLOWED_MOTOR_LEFT 162
#  define SLOWED_MOTOR_RIGHT 160
#else
#  if 1 /* set for playing on the right position */
#    define SLOWED_MOTOR_LEFT 165
#    define SLOWED_MOTOR_RIGHT 160
#  else
	/* old values */
#    define SLOWED_MOTOR_LEFT 180
#    define SLOWED_MOTOR_RIGHT 240
#  endif
#endif

#define STOPPED_MOTOR_LEFT 127
#define STOPPED_MOTOR_RIGHT 127

#define FAST_MOTOR_LEFT 200
#define FAST_MOTOR_RIGHT 196

#if TUNING_JB
		switch(state)
		{
		case STATE_GO_FOR_AWHILE:
#if TUNING_POKE_BALL
			set_left_motors(SLOWED_MOTOR_LEFT);
 			set_right_motors(SLOWED_MOTOR_RIGHT);
#else
			set_left_motors(FAST_MOTOR_LEFT);
			set_right_motors(FAST_MOTOR_RIGHT);
#endif
			banner_count = 0;
			line_seen = 0;

			if (clock2 > /*95*/ 105)
			{
				state = STATE_GO_UNTIL_LINE;
				printf("entering STATE_GO_UNTIL_LINE\n");
			}
			break;

		case STATE_GO_UNTIL_LINE:
#if TUNING_POKE_BALL
			set_left_motors(SLOWED_MOTOR_LEFT);
			set_right_motors(SLOWED_MOTOR_RIGHT);
#else
			set_left_motors(FAST_MOTOR_LEFT);
			set_right_motors(FAST_MOTOR_RIGHT);
#endif

			if (line_seen)
			{
				Disable_Receiver();

//Disables ball poker
#if TUNING_POKE_BALL
				state = STATE_EXTEND_RAM;
				printf("entering STATE_EXTEND_RAM\n");
				clock = 0;
#else
				state = STATE_CHARGE_FORWARD;
				printf("entering STATE_CHARGE_FORWARD\n");
				clock = 0;
#endif
			}
			break;

		case STATE_EXTEND_RAM:
			set_left_motors(STOPPED_MOTOR_LEFT);			//left motors
			set_right_motors(STOPPED_MOTOR_RIGHT);			//right motors
			relay5_fwd=1;
			relay5_rev=0;
			clock++;
			if (clock > 20)
			{
				state = STATE_LEAVE_RAM_OUT;
				printf("entering STATE_LEAVE_RAM_OUT\n");
				clock = 0;
			}
			break;

		case STATE_LEAVE_RAM_OUT:
			set_left_motors(STOPPED_MOTOR_LEFT);		//left motors
			set_right_motors(STOPPED_MOTOR_RIGHT);		//right motors
			relay5_fwd=1;
			relay5_rev=0;
			clock++;
			if (clock > 40)
			{
				state = STATE_RETRACT_RAM;
				printf("entering STATE_RETRACT_RAM\n");
				clock = 0;
			}
			break;

		case STATE_RETRACT_RAM:
			set_left_motors(STOPPED_MOTOR_LEFT);		//left motors
			set_right_motors(STOPPED_MOTOR_RIGHT);		//right motors
			relay5_fwd=0;
			relay5_rev=0;
			clock++;
			if (clock > 25)   /* 25 => 1/2 sec */
			{
#if 1
				state = STATE_CHARGE_FORWARD;
				printf("entering STATE_CHARGE_FORWARD\n");
				clock = 0;
#else
				state = STATE_ALL_STOP;
				printf("entering STATE_ALL_STOP\n");
				clock = 0;
#endif
			}
			break;

		case STATE_CHARGE_FORWARD:
			relay5_fwd=0;
			relay5_rev=0;
			set_left_motors(FAST_MOTOR_LEFT);
			set_right_motors(FAST_MOTOR_RIGHT);
			clock++;
			if (clock > 100)
			{
				state = STATE_ALL_STOP;
				printf("entering STATE_ALL_STOP\n");
				clock = 0;
			}
			break;

		case STATE_ALL_STOP:
			set_left_motors(STOPPED_MOTOR_LEFT);		//left motors
			set_right_motors(STOPPED_MOTOR_RIGHT);		//right motors
			break;
		}
#else		
		if(rc_dig_in03 && clock2>95)
		{
				hitBall=1;
		}

		if(hitBall)
		{
			if(clock==0)							//See the Light
			{
				printf("I saw the light, it was puurty\n");
				task=ACTION_EXTEND_RAM;
				clock++;
			}
			else if(clock<20)
			{
				task=ACTION_LEAVE_RAM_OUT;			//Hit the Ball
				clock++;
			}
			else if(clock<100)
			{
				task=ACTION_RETRACT_RAM;			//Pull Arm Back
				clock++;
			}
#if 0
			else if(clock<330)
			{
				task=ACTION_CHARGE_FORWARD;			//Go Forward Again
				clock++;
			}
#endif
			else
			{
				task=6;					//STOP
				clock++;
			}
		}
		else
		{
			task=ACTION_GO_FORWARD;
		}
		
		if((clock%6)==0 && clock!=0) printf("Task = %d\n",(int)task);
		if((clock%6)==0 && clock!=0) printf("Clock = %d\n",(int)clock);

		switch(task)
		{
			case ACTION_GO_FORWARD:	
																//Initial Movement
					pwm15=pwm16=162;  //15 and 16 need to be greater than 13 and 14 values to go straight
					pwm13=pwm14=160;
					//pwm15=pwm16=180;	//left motors
					//pwm13=pwm14=240;	//right motors
					break;
			case 1:												//Moving Forward To Get to Ball
					pwm15=pwm16=100;
					pwm13=pwm14=100;
					break;
			
			case ACTION_EXTEND_RAM:								//See the Light
					Disable_Receiver();
					pwm15=pwm16=127;			//left motors
					pwm13=pwm14=127;			//right motors
					relay5_fwd=1;
					relay5_rev=0;
					break;
			case ACTION_LEAVE_RAM_OUT:							//Hit the Ball
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					relay5_fwd=1;
					relay5_rev=0;
					break;
			case ACTION_RETRACT_RAM:							//Pull Arm Back
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					relay5_fwd=0;
					relay5_rev=0;
					break;
			case ACTION_CHARGE_FORWARD:												//Ram Forward Again
					relay5_fwd=0;
					relay5_rev=0;
					pwm15=pwm16=200;
					pwm13=pwm14=200;
					break;
			case ACTION_STOP_FOREWARD:
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					break;
			case 7:
					pwm15=pwm16=100;
					pwm13=pwm14=100;
		}
#endif

		pwm15=pwm16=(unsigned char)((unsigned int)old_pwmleft+((unsigned int)25*(2000+(unsigned int)pwm15-(unsigned int)old_pwmleft)/(unsigned int)100) - ((unsigned int)20*(unsigned int)25));
		pwm13=pwm14=(unsigned char)((unsigned int)old_pwmright+((unsigned int)25*(2000+(unsigned int)pwm13-(unsigned int)old_pwmright)/(unsigned int)100) - ((unsigned int)20*(unsigned int)25));

		old_pwmleft=pwm15;
		old_pwmright=pwm13;		
		
		if(task!=0 && old_clock[task]==0) old_clock[task]=clock;

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

		pwm13=pwm14=50*(pwm13-127)/100+pwm13;	//slow it down for testing
		pwm15=pwm16=50*(pwm15-127)/100+pwm15;	

		Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

		clock2++;

        Putdata(&txdata);   //even more bad things will happen if you mess with this
    }
  }
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */

}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
