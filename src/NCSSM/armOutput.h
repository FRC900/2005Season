#ifndef _armOutput_h
#define _armOutput_h

void armOutput_moduleInitialize( void );
void armOutput_reset( void );

void armOutput_moveShoulderUp( void );
void armOutput_moveShoulderDown( void );
void armOutput_moveFingerDown( void );
void armOutput_moveFingerUp( void );
void armOutput_stopFinger( void );
void armOutput_stopShoulder( void );

void armOutput_doShoulderOutput( void);
void armOutput_doFingerOutput(void);

#endif

