/*******************************************************************************
* FILE NAME: ifi_utilities.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to ifi_utilities.c
*  It contains some aliases and function prototypes used in that file.
*
* USAGE:
*  This file should not be modified by the user.
*  DO NOT EDIT THIS FILE!
*******************************************************************************/

#ifndef __ifi_utilities_h_
#define __ifi_utilities_h_

#ifdef _SNOOP_ON_COM1    /* FOR FUTURE USE */
#define RXINTF              PIR3bits.RC2IF
#define RXINTE              PIE3bits.RC2IE
#define TXINTF              PIR3bits.TX2IF 
#define TXINTE              PIE3bits.TX2IE
#define RCSTAbits           RCSTA2bits
#define RCSTA               RCSTA2
#define TXSTA               TXSTA2
#define TXREG               TXREG2
#define RCREG               RCREG2
#define SPBRG               SPBRG2
#define OpenUSART           Open2USART
#else
#define RXINTF              PIR1bits.RCIF
#define RXINTE              PIE1bits.RCIE
#define TXINTF              PIR1bits.TXIF 
#define TXINTE              PIE1bits.TXIE
#define RCSTAbits           RCSTA1bits
#define RCSTA               RCSTA1
#define TXSTA               TXSTA1
#define TXREG               TXREG1
#define RCREG               RCREG1
#define SPBRG               SPBRG1
#define OpenUSART           Open1USART
#endif

/*******************************************************************************
                             MACRO DEFINITIONS
*******************************************************************************/

typedef enum
{
  baud_19 = 15,
  baud_38 = 64,
  baud_56 = 42,
  baud_115 = 21
} SERIAL_SPEED;


/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

void Hex_output(unsigned char temp);  /* located in ifi_library.lib */

#ifdef _FRC_BOARD
  /* located in ifi_library.lib */
void Generate_Pwms(unsigned char pwm_13,unsigned char pwm_14,
                   unsigned char pwm_15,unsigned char pwm_16);
#else
  /* located in ifi_library.lib */
void Generate_Pwms(unsigned char pwm_1,unsigned char pwm_2,
                   unsigned char pwm_3,unsigned char pwm_4,
                   unsigned char pwm_5,unsigned char pwm_6,
                   unsigned char pwm_7,unsigned char pwm_8);
#endif

/* These routines reside in ifi_utilities.c */
void Wait4TXEmpty(void);
void PrintByte(unsigned char odata);
void PrintWord(unsigned int odata);
void PrintString(char *bufr);
void DisplayBufr(unsigned char *bufr);
void PacketNum_Check(void);
void Initialize_Serial_Comms (void);
void Set_Number_of_Analog_Channels (unsigned char number_of_channels);
unsigned int Get_Analog_Value(unsigned char channel);

#endif


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
