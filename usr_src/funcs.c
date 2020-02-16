#include "funcs.h"
#include "PB_LCD_Drivers.h"

GPIO_InitTypeDef* Init_Relays(void){
  GPIO_InitTypeDef* GPIO_Struct=malloc(sizeof(GPIO_InitTypeDef));

  int relay_pins = (GPIO_Pin_5); //Set to required pins with |
  //Enable clock on gpio E
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  
  // configure port E for driving Relays
  GPIO_Struct->GPIO_Pin = relay_pins;
  GPIO_Struct->GPIO_Mode = GPIO_Mode_OUT;    // output
  GPIO_Struct->GPIO_OType = GPIO_OType_PP; //push-pull mode
  GPIO_Struct->GPIO_Speed = 0x03;   // high speed
  GPIO_Struct->GPIO_PuPd = GPIO_PuPd_DOWN; //Pull Down
  GPIO_Init(GPIOE, GPIO_Struct) ;             // initialize port

  return GPIO_Struct;
}

void Switch_Relay(int relay, GPIO_InitTypeDef* GPIO_Struct){ //Switch relays on Port A
  int relay_pin_OS=3; //Relays start at PE.3
  
  if (relay<=3){
  GPIOE->ODR=0;
  GPIOE->ODR=(0b1<<(relay+relay_pin_OS));
  }
  else{
    //Currently do nothing
  }

}

void USART_Cust_Init(void){
  unsigned int bit = 2;
  unsigned long bitMask = ~(3UL << 2*bit);
  
  RCC->AHB1ENR |= RCC_APB1ENR_USART2EN; //Enable USART2 clock

  USART2->BRR=(52<<4);
  USART2->CR1=(1<<2)|(1<<3)|(1<<13);//Enable RX, TX, UART

  // Set-up PA2 as an output, and configure PA2 to take input
  // from USART2 (alternate function mode):
  GPIOA->AFR[0] = (GPIOA->AFR[0] & 0xFFFFF0FF) | (0x7 << 8);
  GPIOA->MODER = (GPIOA->MODER & bitMask) | (2UL << 2*bit);
  
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA clock
}

void SerialWrite_Char(char data){
  while (!(USART2->SR & USART_SR_TXE));
  USART2->DR = data;
}

void SerialWrite_String(char *str)
{
    // Send a string
  while (*str) //do for all characters
    {
        SerialWrite_Char(*str++);
    }
}


int HSE_CLK_Init(void){
  RCC->CR |= RCC_CR_HSEON;
  
  while (!(RCC->CR & RCC_CR_HSERDY)){}

  RCC->CFGR &= ~(RCC_CFGR_SW_0 | RCC_CFGR_SW_1);
  RCC->CFGR |= RCC_CFGR_SW_HSE;

  return 0;
}
