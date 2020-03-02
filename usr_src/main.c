#include "funcs.h"
#include "PB_LCD_Drivers.h"
//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct;

volatile uint32_t ADC_result = 0, msTicks = 0;                        /* Variable to store millisecond ticks */
volatile int pressed_button=0;

int setup(void){
  SystemInit();//Initialise System
  HSE_CLK_Init();//init external clock
  
  USART_Cust_Init(); //init UART
  Init_Relays(); //Init relay pins
  Initialise_ADCs();
  PB_LCD_Init(); //Init Screen
  Initialise_IRQs(); //setup IRQs
  
  Init_Buttons();
  
  return 0;
}

int main(void){
  setup();
  PB_LCD_Clear();
  
  char buff[50];
  int sTicks=0;
  enum Modes CurrentMode=DC_V;
  enum Ranges CurrentRange=one;
  
  unsigned int zero_point=init_vm();

  toggle_ADCs(1);
  redraw_display(buff);
  Switch_Relay(3);
  while (1){ //Main control loop
    
    if (!(msTicks % 200)){
      
      sTicks++;
      sprintf(buff,"ADC:%d, %d, %d",(int)(ADC_result),CurrentMode,CurrentRange);
      SerialWrite_String(buff);
      SerialWrite_String("\n\r");
        
      redraw_display(buff);      
    } 
    
    pressed_button=modeSwitch(&CurrentMode,&CurrentRange,pressed_button);
    
  }
}





//IRQ Stuff. BEGIN:


void SysTick_Handler(void)  {                               /* SysTick interrupt Handler. */
  msTicks++;                                                /* See startup file startup_DEVICE.s for SysTick vector */ 
}

void ADC_IRQHandler (void) {
  //SerialWrite_String("ADC\n\r");
  ADC_result = ADC1->DR;
  ADC1->CR2 |= ((0b1<<30));//Start new conversion
}

void EXTI9_5_IRQHandler (void) {
  switch (EXTI->PR){ //Find which button has been pressed
  case (1<<8):
    pressed_button=1;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0100);
    break;
  case (1<<9):
    pressed_button=2;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0200);
    break;
  }
  EXTI->PR = (0xFFFF);//Clear irq
}

void EXTI15_10_IRQHandler(void) {
  /* Make sure that interrupt flag is set */
  switch (EXTI->PR){ //Find which button has been pressed
  case (1<<10):
    pressed_button=3;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0400);
    break;
  case (1<<11):
    pressed_button=4;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0800);
    break;
  case (1<<12):
    pressed_button=5;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x1000);
    break;
  case (1<<13):
    pressed_button=6;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x2000);
    break;
  case (1<<14):
    pressed_button=7;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x4000);
    break;
  case (1<<15):
    pressed_button=8;
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x8000);
    break;
  }
  
  EXTI->PR = (0xFFFF);//Clear irq
}

