/*******************************************************************************
*
*	TITLE:		receiver.c 
*
*	VERSION:	0.9 (Beta)                           
*
*	DATE:		13-Jan-2004
*
*	AUTHOR:		R. Kevin Watson
*
*	COMMENTS:	This file contains example 2004 FRC infrared receiver code.
*				This code is designed to run on both the EDU and full-size
*				robot controllers.
*
*				Numbers within brackets refer to the PIC18F8520 data sheet
*				page number where additional information can be found.
*
*				Assumptions:
*
*				For the EDU controller IR sensor 1 is on interrupt 3
*					         "         IR sensor 2 is on interrupt 4
*					         "         IR sensor 3 is on interrupt 5
*					         "         IR sensor 4 is on interrupt 6
*
*				For the 2004 FRC controller IR sensor 1 is on digital input 3
*					         "              IR sensor 2 is on digital input 4
*					         "              IR sensor 3 is on digital input 5
*					         "              IR sensor 4 is on digital input 6
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	28-Nov-2003  0.3  RKW Original
*	30-Nov-2003  0.4  RKW - Changed Sensor_Stats definition, making Beacon_Count
*	                  and Beacon_Quality an array.
*	04-Dec-2003  0.5  RKW - Changed timer 1 prescaler to 4:1 from 8:1 to double
*	                  the racker control rate.
*	07-Dec-2003  0.6  RKW - Turned Sensor_Stats into an array so that the code in
*	                  tracker.c can be simplified.
*	09-Dec-2003  0.7  RKW - Slight modifications to run on the 2004 full-size
*	                  robot controller
*	23-Dec-2003  0.8  RKW - Reassigned interrupts
*	13-Jan-2004  0.9  RKW - Added Disable_Receiver() function which should be
*	                  called when the beacon receiver is no longer needed.
*
*******************************************************************************/

#include "ifi_picdefs.h"
#include "ifi_aliases.h"
#include "printf_lib.h"
#include "receiver.h"
#include "user_routines.h"

unsigned int Clock = 0;	// upper 16 bits of the 32 bit system clock
						// timer 1 contains the lower 16 bits
						// each clock tick is worth 26.21 milliseconds
unsigned long Wheel_Clock=0;

unsigned int wheel_r_count=0;
unsigned int wheel_l_count=0;

/*******************************************************************************
*
*	FUNCTION:		Initialize_Receiver()
*
*	PURPOSE:		Initializes the infrared receiver code.
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/

void Initialize_Receiver(void)  
{
	// initialize and start timer 1
	IPR1bits.TMR1IP = 0;		// timer 1 will use the low priority interrupt [98]
	PIE1bits.TMR1IE = 1;		// enable timer1 roll-over interrupt [95]
	T1CONbits.RD16 = 1;			// use 16-bit read/write operations with timer 1 [135]
	T1CONbits.T1CKPS0 = 0;		// timer 1 clock will use a 4:1 prescale value [135]
	T1CONbits.T1CKPS1 = 1;
	T1CONbits.T1OSCEN = 0;		// turn-off the timer 1 oscillator to save power [135]
	T1CONbits.TMR1CS = 0;		// timer 1 will use the internal 10MHz clock [135]
	T1CONbits.TMR1ON = 1;		// start timer 1 [135]

	// initialize external interrupts 3-6 (KBI0 - KBI3 on user 18F8520)
	TRISBbits.TRISB4 = 1;		// make sure RB4/KBI0 is setup as an input [108]
	TRISBbits.TRISB5 = 1;		// make sure RB5/KBI1 is setup as an input [108]
	TRISBbits.TRISB6 = 1;		// make sure RB6/KBI2 is setup as an input [108]
	TRISBbits.TRISB7 = 1;		// make sure RB7/KBI3 is setup as an input [108]
	INTCON2bits.RBIP = 0;		// interrupts 3 through 6 will use the low priority interrupt [90]
	INTCONbits.RBIE = 1;		// enable interrupts 3 through 6 [89]
	
	Initialize_Timer_2();
}
/*******************************************************************************
*
*	FUNCTION:		Initialize_Timer_2()
*
*	PURPOSE:		Initializes the timer 2 hardware.
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		Place "#include "interrupts.h" in the includes section
*					of user_routines.c then call Initialize_Timer_2() in
*					user_routines.c/User_Initialization().
*
*					Timer 2 documentation starts on page 141 of the data sheet.
*
*******************************************************************************/
void Initialize_Timer_2(void)  
{
	TMR2 = 0x00;			// 8-bit timer 2 register (this is readable and writable)
							//
	PR2	= 0xFF;				// timer 2 period register - timer 2 increments to this 
							// value then resets to zero on the next clock and starts
							// all over again
							//
	T2CONbits.T2OUTPS0 = 0;	// T2OUTPS3 T2OUTPS2 T2OUTPS1 T2OUTPS0
	T2CONbits.T2OUTPS1 = 0;	//    0        0        0        0		1:1 postscaler
	T2CONbits.T2OUTPS2 = 0;	//    0        0        0        1		1:2 postscaler
	T2CONbits.T2OUTPS3 = 0;	//    0        0        1        0		1:3 postscaler
							//    0        0        1        1		1:4 postscaler
							//    0        1        0        0		1:5 postscaler
							//    0        1        0        1		1:6 postscaler
							//    0        1        1        0		1:7 postscaler
							//    0        1        1        1		1:8 postscaler
							//    1        0        0        0		1:9 postscaler
							//    1        0        0        1		1:10 postscaler
							//    1        0        1        0		1:11 postscaler
							//    1        0        1        1		1:12 postscaler
							//    1        1        0        0		1:13 postscaler
							//    1        1        0        1		1:14 postscaler
							//    1        1        1        0		1:15 postscaler
							//    1        1        1        1		1:16 postscaler
							//
	T2CONbits.T2CKPS0 = 1;	// T2CKPS1  T2CKPS0
	T2CONbits.T2CKPS1 = 1;	//    0        0	1:1 prescaler (clock = 10MHz/each tick=100ns)
							//    0        1	1:4 prescaler (clock = 2.5MHz/each tick=400ns)
							//    1        x	1:16 prescaler (clock = 625KHz/each tick=1.6us) (T2CKPS0 doesn't matter)
							//
	PIE1bits.TMR2IE = 1;	// 0: disable timer 2 interrupt on PR2 match
							// 1: enable timer 2 interrupt on PR2 match
							//    if the prescaler is enabled (i.e., greater than 1:1), this
							//    match will occur n times (where n is the postscaler value)
							//    before an interrupt will be generated
							//
	IPR1bits.TMR2IP = 0;	// 0: timer 2 interrupt is low priority (leave at 0 for IFI controllers)
							// 1: timer 2 interrupt is high priority
							//
	T2CONbits.TMR2ON = 1;	// 0: timer 2 is disabled
							// 1: timer 2 is enabled (running)
}

/*******************************************************************************
*
*	FUNCTION:		Disable_Receiver()
*
*	PURPOSE:		Disables the infrared receiver code.
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Disable_Receiver(void)  
{
	T1CONbits.TMR1ON = 0;		// stop timer 1 [135]
	//INTCONbits.RBIE = 0;		// disable interrupts 3 through 6 [89]
}



void disableInterrupts(void)
{
	T1CONbits.TMR1ON = 0;		// stop timer 1 [135]
	T2CONbits.TMR2ON = 0;
	INTCONbits.RBIE = 0;		// disable interrupts 3 through 6 [89]
}


void enableInterrupts(void)
{
	T1CONbits.TMR1ON = 1;		// stop timer 1 [135]
	T2CONbits.TMR2ON = 1;
	INTCONbits.RBIE = 1;		// disable interrupts 3 through 6 [89]
}
/*******************************************************************************
*
*	FUNCTION:		Timer_2_Int_Handler()
*
*	PURPOSE:		This code is called in response to a timer 1 
*					roll-over interrupt.
*
*	CALLED FROM:	User_Routines_Fast()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		With a 1:4 timer 1 prescale, this function is 
*					called every 26.21 milliseconds
*
*******************************************************************************/
void Timer_2_Int_Handler(void)
{
	Wheel_Clock++;
}
/*******************************************************************************
*
*	FUNCTION:		Timer_1_Int_Handler()
*
*	PURPOSE:		This code is called in response to a timer 1 
*					roll-over interrupt.
*
*	CALLED FROM:	User_Routines_Fast()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		With a 1:4 timer 1 prescale, this function is 
*					called every 26.21 milliseconds
*
*******************************************************************************/
void Timer_1_Int_Handler(void)
{	
	Clock++; // increment the upper 16-bits of the system clock
}

/*******************************************************************************
*
*	FUNCTION:		Int_3_Handler()
*
*	PURPOSE:		This code is called in response to an interrupt 3 request.
*					This is used by IR sensor 1. 
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		The digital input 3/interrupt 3 input uses the
*					PIC18F8520's RB4/KBI0 pin on port b.
*
*******************************************************************************/
void Int_3_Handler(unsigned char RB4_State, unsigned char *hitBall)
{
	unsigned char Bryan; // temporary data buffer
	unsigned int Int_3_Period; // pulse-width period
	unsigned int Int_3_Up_Edge_Count_Low; // lower 16-bits of the rising-edge time snapshot
	unsigned int Int_3_Up_Edge_Count_High; // upper 16-bits of the rising-edge time snapshot
	static unsigned int Int_3_Down_Edge_Count_Low; // lower 16-bits of the falling-edge time snapshot
	static unsigned int Int_3_Down_Edge_Count_High; // upper 16-bits of the falling-edge time snapshot
	

	if(!*hitBall)
	{
		/*switch(RB4_State) // current state determines how the function behaves
		{
		case 0: // falling-edge detected (beginning of the pulse)
			printf("Falling Edge\n");
			Int_3_Down_Edge_Count_High = Clock; // get a snapshot of the time
			Bryan = TMR1L; // TMR1L must be read before TMR1H
			Int_3_Down_Edge_Count_Low = TMR1H;
			Int_3_Down_Edge_Count_Low <<= 8;
			Int_3_Down_Edge_Count_Low += Bryan;
			break; // now wait for the rising-edge interrupt to happen...
	
		case 1: // rising-edge detected (end of the pulse)
			printf("Rising Edge\n");
			Int_3_Up_Edge_Count_High = Clock; // get a snapshot of the time
			Bryan = TMR1L;
			Int_3_Up_Edge_Count_Low = TMR1H;
			Int_3_Up_Edge_Count_Low <<= 8;
			Int_3_Up_Edge_Count_Low += Bryan;
			
			// determine the pulse-width period by determining the time 
			// difference between the falling-edge and rising-edge interrupts
			if (Int_3_Up_Edge_Count_High == Int_3_Down_Edge_Count_High)
			{
				// this is quicker because the 16-bit system clock hasn't changed and therefore has no effect on the outcome
				Int_3_Period = Int_3_Up_Edge_Count_Low - Int_3_Down_Edge_Count_Low;
			}	
			else
			{
				// this works because the pulse-width will always be less than one clock tick (= 65536 timer ticks)
				Int_3_Period = 65536 - Int_3_Down_Edge_Count_Low + Int_3_Up_Edge_Count_Low;
			}
		
			// now catagorize the pulse type and update the associated statistics variable
			if ((Int_3_Period >= BEACON_0_LOWER_BOUND) && (Int_3_Period <= BEACON_0_UPPER_BOUND))
			{
				printf("Head Towards the Light...\n");
				*hitBall=1;
			}
			break; // now wait for another falling-edge interrupt to happen...
		}*/

		printf("Head Towards the Light...\n");
		*hitBall=1;
	}
	else
	{
		Disable_Receiver();
	}
}
/*******************************************************************************
*
*	FUNCTION:		Int_4_Handler()
*
*	PURPOSE:		This code is called in response to a interrupt 4 request.
*					This is used by IR sensor 2. 
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		The digital input 4/interrupt 4 input uses the
*					PIC18F8520's RB5/KBI1/PGM pin on port b.
*
*******************************************************************************/
void Int_4_Handler(unsigned char RB5_State)
{
	//Right Encoder Interrupt
    if(rc_dig_in04) wheel_r_count++;
}

/*******************************************************************************
*
*	FUNCTION:		Int_5_Handler()
*
*	PURPOSE:		This code is called in response to a interrupt 5 request.
*					This is used by IR sensor 3. 
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		The digital input 5/interrupt 5 input uses the
*					PIC18F8520's RB6/KBI2/PGC pin on port b.
*
*******************************************************************************/
void Int_5_Handler(unsigned char RB6_State)
{
	//Left Encoder Interrupt
	printf("GAAAA!\n");
    if(rc_dig_in05) wheel_l_count++;
}

/*******************************************************************************
*
*	FUNCTION:		Int_6_Handler()
*
*	PURPOSE:		This code is called in response to a interrupt 6 request.
*					This is used by IR sensor 4. 
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		The digital input 6/interrupt 6 input uses the
*					PIC18F8520's RB7/KBI3/PGD pin on port b.
*
*******************************************************************************/
void Int_6_Handler(unsigned char RB7_State)
{
	if(rc_dig_in06) wheel_l_count++;
}
