#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include <stdlib.h>
#include <stdio.h>

void Switch_Relay(int relay, GPIO_InitTypeDef* GPIO_Struct);

GPIO_InitTypeDef* Init_Relays(void);

int HSE_CLK_Init();

int USART_Cust_Init();
