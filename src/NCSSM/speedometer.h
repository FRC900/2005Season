#ifndef _speedometer_h
#define _speedometer_h

typedef struct speedometer_State speedometer_State;

struct speedometer_State {
	volatile unsigned int speed;
	volatile unsigned long distance;
	volatile unsigned long tLast;
	volatile unsigned long tDelta;
	unsigned long scaleFactor;
    unsigned int encoderDistance;
};

unsigned int speedometer_getSpeed( speedometer_State *instance );
unsigned long speedometer_getDistance( speedometer_State *instance );
void speedometer_handleEncoderClick( speedometer_State *instance );
void speedometer_moduleInitialize( void );

#ifndef __speedometer_c
  extern speedometer_State speedometer_left;
  extern speedometer_State speedometer_right;
#endif 


/*** Avg Speedometer ***/
#define max_samples 5

volatile typedef struct {
	volatile unsigned long samples[max_samples];
	volatile unsigned int numSamples;
	volatile unsigned int currentIndex;
	volatile unsigned long Tlast;
	volatile char reset;
	unsigned long scaleFactor;
} AvgSpeedometer;

extern volatile AvgSpeedometer avgSpeed_left;
extern volatile AvgSpeedometer avgSpeed_right;

void avgSpeed_moduleInitialize( void );
void avgSpeed_handleInterrupt( AvgSpeedometer *instance );
unsigned int avgSpeed_getSpeed( AvgSpeedometer *instance );

#endif
