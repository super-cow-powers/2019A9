#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h" //STM SPL GPIO lib
#include "stm32f4xx_exti.h" //STM SPL EXTI li
#include <stdlib.h>
#include <stdio.h>

extern volatile uint32_t msTicks; extern volatile uint32_t ADC_result; //micro second tick and ADC_Result

enum Modes {DC_V,AC_V,DC_I,AC_I,FRQ};
enum Ranges {tenm,hundredm,one,ten};

void Init_Buttons(void);

void Switch_Relay(int relay);

void Init_Relays(void);

int HSE_CLK_Init(void);

void USART_Cust_Init(void);

void SerialWrite_Char(char data);

void SerialWrite_String(char *str);

void Initialise_IRQs(void);

void Initialise_ADCs(void);

void toggle_ADCs(int state);

void redraw_display(char* buffer);

void delay_us(u32 nTime);

void delay_ms(u32 nTime);

unsigned int init_vm(void);

int modeSwitch(enum Modes* CurrentMode, enum Ranges* CurrentRange, int pressed_button);
