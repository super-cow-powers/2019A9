/* Function and struct defs go in this file.

   Pin Assignments:
   P.E (8-15): On-board buttons
   P.E (3-6) : Relays
   P.C 4     : ADC
   P.E 0    : Frequency. TIM4_CH1

 */

#define ARM_MATH_CM4
#include "stm32f4xx.h"
#include <arm_math.h> /* Include ARM CMSIS-DSP stuff for "quick maffs". */
#include "core_cm4.h" /* Include ARM's funky types. Word of warning: CMSIS-DSP seems to not like GCC too much - 
		       * Here Be Dragons! Check the -specs args in the makefile for how I made it go.
		       * Also, I can't get the static libs to work, so they have to be rebuilt each time. */
#include "stm32f4xx_gpio.h" //STM SPL GPIO lib
#include "stm32f4xx_exti.h" //STM SPL EXTI lib

#include <stdint.h>
#include <string.h> //Maybe I shouldn't on an MCU - but it has like, loads of memory and stuff.

extern volatile uint32_t msTicks; extern volatile uint32_t ADC_result; //micro second tick and ADC_Result
//Errm, nothing to see here - move along please...

typedef enum Modes {DC_V,AC_V,DC_I,AC_I,FRQ,RES} Modes;
typedef enum MeasureTypes {MEAN,MAX,RMS} MeasureTypes;
typedef enum Ranges {tenm,hundredm,one,ten} Ranges;

typedef struct { //Mode struct. Contains the basic state information for the meter 
  Modes MeasureMode;
  MeasureTypes MeasureType;
  Ranges CurrentRange;
} Mode;

void Init_Buttons(void);

void Switch_Relay(int relay);

void Init_Relays(void);

int HSE_CLK_Init(void);

void USART_Cust_Init(void);

void SerialWrite_Char(char data);

void SerialWrite_String(char *str);

void Initialise_IRQs(void);

void Initialise_ADCs(void);

void init_FRQ(void);

void toggle_ADCs(int state);

char* output_to_string(float value);

void update_Serial_out(float value, Mode* CurrentMode);

void redraw_display(float value, Mode CurrentMode);

void delay_us(u32 nTime); //MicroSecond delay. Not implemented due to slow HSE

void delay_ms(u32 nTime); //MilliSecond Delay. Not required currently

unsigned int init_vm(void); //Set the zero point (simple calibration)

int modeSwitch(Mode* CurrentMode, int pressed_button); //Update the current mode
