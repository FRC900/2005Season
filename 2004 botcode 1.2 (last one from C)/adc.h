#ifndef __ADC_H
#define __ADC_H

/* PIC 17Cxxx and 18Cxxx A/D conversion peripheral library header */

/* There are two different forms of ADC peripheral interfaces, the old style is
 * 2 control registers (ADCON0/1) and port configuration bits combined with
 * voltage reference configuration bits.  The new style is 3 control registers
 * (ADCON0/1/2) and separate port / voltage reference configuration bits.
 * Parts that use the old style will define ADC_INTERFACE_OLD
 */
#if defined(__18C242) || defined(__18C252) || defined(__18C442) || defined(__18C452)
#define ADC_INTERFACE_OLD
#endif

#if defined(__18F242) || defined(__18F252) || defined(__18F442) || defined(__18F452)
#define ADC_INTERFACE_OLD
#endif

#if defined(__18F2439) || defined(__18F2539) || defined(__18F4439) || defined(__18F4539)
#define ADC_INTERFACE_OLD
#endif

#if defined(__18F248) || defined(__18F258) || defined(__18F448) || defined(__18F458)
#define ADC_INTERFACE_OLD
#endif

/* Configuration Bit Definitions */
#define ADC_INT_ON       0b11111111 // A/D interrupt on
#define ADC_INT_OFF      0b01111111 // A/D interrupt off

#if    __18CXX
#define ADC_FOSC_2       0b10001111 // A/D clock source Fosc/2
#define ADC_FOSC_4       0b11001111 // A/D clock source Fosc/4
#define ADC_FOSC_8       0b10011111 // A/D clock source Fosc/8
#define ADC_FOSC_16      0b11011111 // A/D clock source Fosc/16
#define ADC_FOSC_32      0b10101111 // A/D clock source Fosc/32
#define ADC_FOSC_64      0b11101111 // A/D clock source Fosc/64
#define ADC_FOSC_RC      0b11111111 // A/D clock source Internal RC OSC
#else /* 17CXX */
#define ADC_FOSC_8       0b10011111 // A/D clock source Fosc/8
#define ADC_FOSC_32      0b10111111 // A/D clock source Fosc/32
#define ADC_FOSC_64      0b11011111 // A/D clock source Fosc/64
#define ADC_FOSC_RC      0b11111111 // A/D clock source Internal RC OSC
#endif

/* Acquisition time select bits */
#if defined(__18F1220) || defined(__18F1320) || defined(__18F2220) || defined(__18F2320) || defined(__18F4220) || defined(__18F4320)
#define ADC_0_TAD        0b11110001
#define ADC_2_TAD        0b11110011
#define ADC_4_TAD        0b11110101
#define ADC_6_TAD        0b11110111
#define ADC_8_TAD        0b11111001
#define ADC_12_TAD       0b11111011
#define ADC_16_TAD       0b11111101
#define ADC_20_TAD       0b11111111
#endif

#if    __18CXX
#define ADC_RIGHT_JUST   0b11111111 // Right justify A/D result
#define ADC_LEFT_JUST    0b01111111 // Left justify A/D result
#else /* 17CXX */
#define ADC_RIGHT_JUST   0b11111111 // Right justify A/D result
#define ADC_LEFT_JUST    0b11101111 // Left justify A/D result
#endif

#if    __18CXX
/* Not applicable to 18Cxxx */
#else /* 17CXX */
#define ADC_VREF_EXT     0b11111111 // Use external pins as A/D Vref
#define ADC_VREF_INT     0b11111110 // Use Avdd pin as A/D Vreg
#endif

#if    __18CXX
#ifdef ADC_INTERFACE_OLD
#define ADC_8ANA_0REF    0b11110000 // VREF+=VDD VREF-=VSS all analog channels   (8/0)
#define ADC_7ANA_1REF    0b11110001 // AN3=VREF+  all analog channels except AN3 (7/1)
#define ADC_5ANA_0REF    0b11110010 // VREF+=VDD VREF-=VSS  (5/0)
#define ADC_4ANA_1REF    0b11110011 // AN3=VREF+      (4/1)
#define ADC_3ANA_0REF    0b11110100 // VREF+=VDD VREF-=VSS  (3/0)
#define ADC_2ANA_1REF    0b11110101 // AN3=VREF+         (2/1)
#define ADC_0ANA_0REF    0b11110111 // ALL DIGITAL I/O      (0/0)
#define ADC_6ANA_2REF    0b11111000 // AN3=VREF+ AN2=VREF-  (6/2)
#define ADC_6ANA_0REF    0b11111001 // VREF+=VDD VREF-=VSS  (6/0)
#define ADC_5ANA_1REF    0b11111010 // AN3=VREF+ VREF-=VSS  (5/1)
#define ADC_4ANA_2REF    0b11111011 // AN3=VREF+ AN2=VREF-  (4/2)
#define ADC_3ANA_2REF    0b11111100 // AN3=VREF+ AN2=VREF-  (3/2)
#define ADC_2ANA_2REF    0b11111101 // AN3=VREF+ AN2=VREF-  (2/2)
#define ADC_1ANA_0REF    0b11111110 // AN0 is analog input  (1/0)
#define ADC_1ANA_2REF    0b11111111 // AN3=VREF+ AN2=VREF- AN0=A (1/2)
#else  /* !ADC_INTERFACE_OLD */
/* Voltage reference configuration bits */
#define ADC_VREFPLUS_VDD   0b11111110 // VREF+ = AVDD
#define ADC_VREFPLUS_EXT   0b11111111 // VREF+ = external
#define ADC_VREFMINUS_VSS  0b11111101 // VREF- = AVDD
#define ADC_VREFMINUS_EXT  0b11111111 // VREF- = external
/* AD Port configuration bits */
#define ADC_0ANA   0b11111111 // All digital
#define ADC_1ANA   0b11111110 // analog: AN0      digital: AN1->15
#define ADC_2ANA   0b11111101 // analog: AN0->1   digital: AN2->15
#define ADC_3ANA   0b11111100 // analog: AN0->2   digital: AN3->15
#define ADC_4ANA   0b11111011 // analog: AN0->3   digital: AN4->15
#define ADC_5ANA   0b11111010 // analog: AN0->4   digital: AN5->15
#define ADC_6ANA   0b11111001 // analog: AN0->5   digital: AN6->15
#define ADC_7ANA   0b11111000 // analog: AN0->6   digital: AN7->15
#define ADC_8ANA   0b11110111 // analog: AN0->7   digital: AN8->15
#define ADC_9ANA   0b11110110 // analog: AN0->8   digital: AN9->15
#define ADC_10ANA  0b11110101 // analog: AN0->9   digital: AN10->15
#define ADC_11ANA  0b11110100 // analog: AN0->10  digital: AN11->15
#define ADC_12ANA  0b11110011 // analog: AN0->11  digital: AN12->15
#define ADC_13ANA  0b11110010 // analog: AN0->12  digital: AN13->15
#define ADC_14ANA  0b11110001 // analog: AN0->13  digital: AN14->15
#define ADC_16ANA  0b11110000 // All analog
#endif /* ADC_INTERFACE_OLD */
#else  /* 17CXX */
#define ADC_ALL_ANALOG   0b11110001 // AN0 - AN11 analog
#define ADC_ALL_DIGITAL  0b11111111 // AN0 - AN11 digital
#define ADC_11ANA_1DIG   0b11110011 // AN7 digital, all others analog
#define ADC_10ANA_2DIG   0b11110101 // AN6 & AN7 digital, all others analog
#define ADC_9ANA_3DIG    0b11110111 // AN5 - AN7 digital, all others analog
#define ADC_8ANA_4DIG    0b11111001 // AN4 - AN7 digital, all others analog
#define ADC_6ANA_6DIG    0b11111011 // AN3 - AN7 digital, all others analog
#define ADC_4ANA_8DIG    0b11111101 // AN2 - AN7 digital, all others analog
#endif

#if    __18CXX
#define ADC_CH0          0b10000111  // Channel 0
#define ADC_CH1          0b10001111  // Channel 1
#define ADC_CH2          0b10010111  // Channel 2
#define ADC_CH3          0b10011111  // Channel 3
#define ADC_CH4          0b10100111  // Channel 4
#define ADC_CH5          0b10101111  // Channel 5
#define ADC_CH6          0b10110111  // Channel 6
#define ADC_CH7          0b10111111  // Channel 7
#define ADC_CH8          0b11000111  // Channel 8
#define ADC_CH9          0b11001111  // Channel 9
#define ADC_CH10         0b11010111  // Channel 10
#define ADC_CH11         0b11011111  // Channel 11
#define ADC_CH12         0b11100111  // Channel 12
#define ADC_CH13         0b11101111  // Channel 13
#define ADC_CH14         0b11110111  // Channel 14
#define ADC_CH15         0b11111111  // Channel 15
#else /* 17CXX */
#define ADC_CH0          0b00000000  // Channel 0
#define ADC_CH1          0b00010000  // Channel 1
#define ADC_CH2          0b00100000  // Channel 2
#define ADC_CH3          0b00110000  // Channel 3
#define ADC_CH4          0b01000000  // Channel 4
#define ADC_CH5          0b01010000  // Channel 5
#define ADC_CH6          0b01100000  // Channel 6
#define ADC_CH7          0b01110000  // Channel 7
#define ADC_CH8          0b10000000  // Channel 8
#define ADC_CH9          0b10010000  // Channel 9
#define ADC_CH10         0b10100000  // Channel 10
#define ADC_CH11         0b10110000  // Channel 11
#endif


/* A/D Result Union */
union ADCResult
{
 int lr;    // 2 bytes as a long
 char br[2];   // 2 bytes as 2 chars
};


/* A/D Converter Function Prototypes */

#if    __18CXX
#define PARAM_SCLASS auto
#else /* 17CXX */
#define PARAM_SCLASS static
#endif

/* OpenADC
 * Configure A/D
 */
#if defined(__18F1220) || defined(__18F1320) || defined(__18F2220) || defined(__18F2320) || defined(__18F4220) || defined(__18F4320)
void OpenADC(PARAM_SCLASS unsigned char, PARAM_SCLASS unsigned char, PARAM_SCLASS unsigned char);
#else
void OpenADC(PARAM_SCLASS unsigned char, PARAM_SCLASS unsigned char);
#endif

/* ConvertADC
 * Start an A/D conversion
 */
void ConvertADC(void);

/* SetChanADC
 * Set A/D to specified channel
 */
void SetChanADC(PARAM_SCLASS unsigned char);

/* ReadADC
 * Read A/D result
 */
int ReadADC(void);

/* CloseADC
 * Turn off A/D
 */
void CloseADC(void);

/* BusyADC
 * Check status of A/D conversion
 */
char BusyADC(void);

#endif
