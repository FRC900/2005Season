
#ifndef _FRC2004_
	#include <stdio.h>
	#include "user_Serialdrv.h"
#else
	#include "printf_lib.h"
#endif

#include <timers.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "driverClock.h"
#include "speedometer.h"

void InterruptHandlerLow (void);  /* DO NOT CHANGE! */


/*******************************************************************************
*
*	FUNCTION:		Initialize_Timer_1()
*
*	PURPOSE:		Initializes the timer 1 hardware.
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:		Place "#include "interrupts.h" in the includes section
*					of user_routines.c then call Initialize_Timer_1() in
*					user_routines.c/User_Initialization().
*
*					Timer 1 documentation starts on page 135 of the data sheet.
*
*******************************************************************************/
void Initialize_Timer_1(void)  
{
	printf("initializing timer1...");

	TMR1L = 0x82;			// least significant 8-bits of the timer 1 register (this is readable and writable)
	TMR1H = 0xFF;			// most significant 8-bits of the timer 1 register (this is readable and writable)
							//
	T1CONbits.T1CKPS0 = 1;	// T1CSP1 T1CSP0
	T1CONbits.T1CKPS1 = 1;	//   0      0		1:1 prescaler (clock=10MHz/each tick=100ns)
							//   0      1		1:2 prescaler (clock=5MHz/each tick=200ns)
							//   1      0		1:4 prescaler (clock=2.5MHz/each tick=400ns)
							//   1      1		1:8 prescaler (clock=1.25MHz/each tick=800ns)
							//
	T1CONbits.T1OSCEN = 0;	// 0: timer 1 oscillator disabled (leave at 0 to allow the use of an external clock)
							// 1: timer 1 oscillator enabled (can't be used because of hardware constraints)
							//
	T1CONbits.TMR1CS = 0;	// 0: use the internal clock
							// 1: use an external clock on RC0/T1OSO/T13CLK (rc_dig_in14 on full-size controller)
							//
	T1CONbits.RD16 = 1;		// 0: timer 1 register operations are done in two 8-bit accesses
							// 1: timer 1 register operations are done in one 16-bit access
							//    In this mode, reading TMR1L will latch a copy of TMR1H into a buffer
							//    mapped to the TMR1H memory address. Conversely, a write to the buffer
							//    followed by a write to the TMR1L register will update the entire 16-bit
							//    timer at once. This solves the problem where the timer may overflow
							//    between two 8-bit accesses. Here's an example of how to do a 16-bit read:
							//
							//    unsigned char Temp_Buf; // 8-bit temporary buffer
							//    unsigned int Timer_Snapshot; // 16-bit variable
							//
							//    Temp_Buf = TMR1L; // TMR1L must be read before TMR1H
							//    Timer_Snapshot = TMR1H;
		 					//    Timer_Snapshot <<= 8; // move TMR1H data to the upper half of the variable
							//    Timer_Snapshot += Temp_Buf; // we now have all sixteen bits  
							// 
	IPR1bits.TMR1IP = 0;	// 0: timer 1 overflow interrupt is low priority (leave at 0 on IFI controllers)
							// 1: timer 1 overflow interrupt is high priority
							//
	PIR1bits.TMR1IF = 0;	// 0: timer 1 overflow hasn't happened (set to 0 before enabling the interrupt)
							// 1: timer 1 overflow has happened
							//
	PIE1bits.TMR1IE = 1;	// 0: disable timer 1 interrupt on overflow (i.e., a transition from FFFF->0)
							// 1: enable timer 1 interrupt on overflow (i.e., a transition from FFFF->0)
							//	
	T1CONbits.TMR1ON = 1;	// 0: timer 1 is disabled
							// 1: timer 1 is enabled (running)

	printf("done.\r");
}

static void InitializeInterrupt1( void ) {
	printf("...interrupt 1 initialize...");
	// initialize external interrupt 1 (INT2 on user 18F8520)
	TRISBbits.TRISB2 = 1;		// make sure the RB2/INT2 pin is configured as an input [108]
								//
	INTCON3bits.INT2IP = 0;		// 0: interrupt 1 is low priority (leave at 0 for IFI controllers) [91]
								// 1: interrupt 1 is high priority
								//
	INTCON2bits.INTEDG2 = 0;	// 0: trigger on the falling-edge [90]
								// 1: trigger on the rising-edge
								//
	INTCON3bits.INT2IF = 0;		// 0: external interrupt 1 hasn't happened (set to 0 before enabling the interrupt) [91]
								// 1: external interrupt 1 has happened
								//
	INTCON3bits.INT2IE = 0;		// 0: disable interrupt	1 [91]
								// 1: enable interrupt 1
	printf("done.\r");
}

static void InitializeInterrupt2( void ) {
	printf("...interrupt 2 initialize...");

	// initialize external interrupt 2 (INT3 on user 18F8520)
	TRISBbits.TRISB3 = 1;		// make sure the RB3/CCP2/INT3 pin is configured as an input [108]
								//
	INTCON2bits.INT3IP = 0;		// 0: interrupt 2 is low priority (leave at 0 for IFI controllers) [90]
								// 1: interrupt 2 is high priority
								//
	INTCON2bits.INTEDG3 = 0;	// 0: trigger on the falling-edge [90]
								// 1: trigger on the rising-edge
								//
	INTCON3bits.INT3IF = 0;		// 0: external interrupt 2 hasn't happened (set to 0 before enabling the interrupt) [91]
								// 1: external interrupt 2 has happened
								//
	INTCON3bits.INT3IE = 0;		// 0: disable interrupt	2 [91]
								// 1: enable interrupt 2
	printf("done.\r");
}


/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD /* You may want to save additional symbols. */

void InterruptHandlerLow ()     
{                               
 unsigned char int_byte;       
  //mcfalls - ADded timer interrupt handlers - currently only using timer0
	if (PIR1bits.TMR1IF && PIE1bits.TMR1IE) // timer 1 interrupt?
	{
		WriteTimer1( 0xFF82 );
		driverClock_interruptHandler();
		PIR1bits.TMR1IF = 0; // clear the timer 1 interrupt flag [92]
	}  
	else if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
	{ 
#ifndef _USE_AVGSPEED
		speedometer_handleEncoderClick( &speedometer_left );
#else
		avgSpeed_handleInterrupt( &avgSpeed_left );
#endif
		INTCON3bits.INT2IF = 0;
	}
	else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
	{
#ifndef _USE_AVGSPEED
		speedometer_handleEncoderClick( &speedometer_right );
#else
		avgSpeed_handleInterrupt( &avgSpeed_right );
#endif
		INTCON3bits.INT3IF = 0;
	}
	else
	{ 
#ifndef _FRC2004_
		CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
#endif
	}
}


void interrupts_moduleInitialize(void)  
{
	printf("interrupts_moduleInitialize()\r");

	Initialize_Timer_1();
	InitializeInterrupt1();
	InitializeInterrupt2();

	printf("interrupts_moduleInitialize()...done\r");
}						  

void interrupts_enable(void) {

  // INT 1
	INTCON3bits.INT2IE = 1;		// 0: disable interrupt	1 [91]
								// 1: enable interrupt 1
  // INT 2
	INTCON3bits.INT3IE = 1;		// 0: disable interrupt	2 [91]
								// 1: enable interrupt 2
}

void interrupts_disable( void ) {

	// INT 1
	INTCON3bits.INT2IE = 0;		// 0: disable interrupt	1 [91]
								// 1: enable interrupt 1
	// INT 2
	INTCON3bits.INT3IE = 0;		// 0: disable interrupt	2 [91]
								// 1: enable interrupt 2
}
