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


static unsigned char hitBall=0;
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
			Int_3_Handler(Port_B & 0x10 ? 1 : 0, &hitBall); // call the interrupt 3 handler (in receiver.c)
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
void User_Autonomous_Code(void)
{
  unsigned char task=0;
  unsigned int old_clock[7];
  unsigned int clock=0;
  unsigned char old_pwmleft=127;
  unsigned char old_pwmright=127;

  for(task;task<7;task++)
  {
  	old_clock[task]=0;
  }
  task=0;

  while(autonomous_mode)
  {
    if(statusflag.NEW_SPI_DATA) //26.2ms loop
    {
        Getdata(&rxdata);   // bad things will happen if you move or delete this
		if(hitBall)
		{
			if(clock==0)							//See the Light
			{
				task=2;
				clock++;
			}
			else
			{
				if(clock<20)
				{
					task=3;							//Hit the Ball
					clock++;
				}
				else
				{
					if(clock<100)
					{
						task=4;						//Pull Arm Back
						clock++;
					}
					else
					{
						if(clock<330)
						{
							task=5;					//Go Forward Again
							clock++;
						}
						else
						{
							task=6;					//STOP
							clock++;
						}
					}
				}
			}
		}
		else
		{
			task=0;
		}
		
		if((clock&0xf)==0) printf("Task = %d\n",(int)task);
		if((clock&0xf)==0) printf("Clock = %d\n",(int)clock);

		switch(task)
		{
			case 0:	
																//Initial Movement
					if((clock&0xf)==0)printf("Dist = %d\n",(int)Get_Analog_Value(rc_ana_in01));
					pwm15=pwm16=170;
					pwm13=pwm14=180;
					//pwm15=pwm16=180;	//left motors
					//pwm13=pwm14=240;	//right motors
					break;
			case 1:												//Moving Forward To Get to Ball
					pwm15=pwm16=200;
					pwm13=pwm14=200;
					break;
			
			case 2:												//See the Light
					Disable_Receiver();
					pwm15=pwm16=127;			//left motors
					pwm13=pwm14=127;			//right motors
					relay5_fwd=1;
					relay5_rev=0;
					break;
			case 3:												//Hit the Ball
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					relay5_fwd=1;
					relay5_rev=0;
					break;
			case 4:												//Pull Arm Back
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					relay5_fwd=0;
					relay5_rev=0;
					break;
			case 5:												//Ram Forward Again
					relay5_fwd=0;
					relay5_rev=0;
					pwm15=pwm16=200;
					pwm13=pwm14=200;
					break;
			case 6:
					pwm15=pwm16=127;		//left motors
					pwm13=pwm14=127;		//right motors
					break;
		}

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
