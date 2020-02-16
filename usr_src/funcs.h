#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include <stdlib.h>
#include <stdio.h>

void Switch_Relay(int relay, GPIO_InitTypeDef* GPIO_Struct);

GPIO_InitTypeDef* Init_Relays(void);

int HSE_CLK_Init(void);

void USART_Cust_Init(void);

void SerialWrite_Char(char data);

void SerialWrite_String(char *str);
