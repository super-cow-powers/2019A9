/* Function and struct defs

   Pin Assignments:
   P.E (8-15): On-board buttons
   P.E (3-6) : Relays
   P.C 4     : ADC
   P.E 0    : Frequency. TIM4_CH1

 */

#define ARM_MATH_CM4
#include "stm32f4xx.h"
#include <arm_math.h> //Include ARM CMSIS-DSP stuff for "quick maffs".
#include "core_cm4.h"
#include "stm32f4xx_gpio.h" //STM SPL GPIO lib
#include "stm32f4xx_exti.h" //STM SPL EXTI li

#include <stdint.h>
#include <string.h>

extern volatile uint32_t msTicks; extern volatile uint32_t ADC_result; //micro second tick and ADC_Result

typedef enum Modes {DC_V,AC_V,DC_I,AC_I,FRQ,RES} Modes;
typedef enum MeasureTypes {MEAN,MAX,RMS} MeasureTypes;
typedef enum Ranges {tenm,hundredm,one,ten} Ranges;

typedef struct {
  Modes MeasureMode;
  MeasureTypes MeasureType;
  Ranges CurrentRange;
} Mode;

/*
typedef struct {
  float32_t ADC_Vals[Max_ADC_Vals];
  int current_value;
} ADC_Buffer;
*/

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

void redraw_display(char* buffer, Mode CurrentMode);

void delay_us(u32 nTime);

void delay_ms(u32 nTime);

unsigned int init_vm(void);

int modeSwitch(Mode* CurrentMode, int pressed_button);
