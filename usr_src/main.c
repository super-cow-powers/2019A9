#include "funcs.h"
#include "PB_LCD_Drivers.h"
//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct;

volatile uint32_t msTicks = 0;                          /* Variable to store millisecond ticks */

int setup(void){
  PB_LCD_Init();
  PB_LCD_WriteString ("Init Meter...",13);
  HSE_CLK_Init();//Init External 8Mhz clock
  Relay_GPIO_Struct=Init_Relays();
  USART_Cust_Init();
  Initialise_IRQs();
  
  
  
  return 0;
}

int main(void){
  setup();
  PB_LCD_Clear();
  PB_LCD_WriteString ("Init Done",9);
  char buff[50];
  //sprintf(buff,"Clock CFG: %ld %ld\n\r",(RCC->CFGR & (RCC_CFGR_SW_1)),(RCC->CFGR & (RCC_CFGR_SW_0)));
  //SerialWrite_String(buff);

  
  while (1){ //Main control loop
    //Switch_Relay(1, Relay_GPIO_Struct);
    sprintf(buff,"mS: %ld : %ld\n\r",msTicks, SystemCoreClock);
    SerialWrite_String(buff);
    PB_LCD_WriteString(buff,25);
    PB_LCD_Clear();
  
    
  }
}


void SysTick_Handler(void)  {                               /* SysTick interrupt Handler. */
  msTicks++;                                                /* See startup file startup_LPC17xx.s for SysTick vector */ 
}
