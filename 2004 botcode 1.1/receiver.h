/*******************************************************************************
*
*	TITLE:		receiver.h 
*
*	VERSION:	0.9 (Beta)                           
*
*	DATE:		13-Jan-2004
*
*	AUTHOR:		R. Kevin Watson
*
*	COMMENTS:
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
#ifndef _receiver_h
#define _receiver_h

struct encoder_s {
   unsigned long high;
   unsigned long last_high;
   unsigned long delta_high;
   unsigned char low;
   unsigned char last_low;
   unsigned char delta_low;
   unsigned char is_white;
   unsigned char last_is_white;
   int cnt;
   unsigned long period[16];
   unsigned char white[16];
};

extern struct encoder_s left_encoder;
extern struct encoder_s right_encoder;


// function prototypes
void Disable_Receiver(void);
void Initialize_Receiver(void);
void Initialize_Timer_2(void);
void Timer_1_Int_Handler(void);		// timer 1 interrupt handleroid
void Timer_2_Int_Handler(void);
void Int_3_Handler(unsigned char);	// IR sensor 1 interrupt handler
void Int_4_Handler(unsigned char);	// IR sensor 2 interrupt handler
void Int_5_Handler(unsigned char);	// IR sensor 3 interrupt handler
void Int_6_Handler(unsigned char);	// IR sensor 4 interrupt handler
void encoder_print(struct encoder_s *p);
void encoder_init(struct encoder_s *p);
void encoder_age_out(struct encoder_s *p);
void enableInterrupts(void);
void disableInterrupts(void);

// some handy macros
#define HIBYTE(value) ((unsigned char)(((unsigned int)(value)>>8)&0xFF))
#define LOBYTE(value) ((unsigned char)(value))

// pulse-width discrimination defines
// type 0 average = 2720 clock ticks
#define BEACON_0_LOWER_BOUND 2040 // average-25% 
#define BEACON_0_UPPER_BOUND 3400 // average+25%

// type 1 average = 5224 clock ticks
#define BEACON_1_LOWER_BOUND 3918 // average-25%
#define BEACON_1_UPPER_BOUND 6530 // average+25%

// state machine defines
#define WAITING_FOR_UP_EDGE 0
#define WAITING_FOR_DOWN_EDGE 1

// variable definitions
#endif // _receiver_h
