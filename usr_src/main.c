#include "funcs.h"
#include "PB_LCD_Drivers.h"
//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct;

volatile uint32_t ADC_result = 0, usTicks = 0;                        /* Variable to store millisecond ticks */

int setup(void){
  SystemInit();//Initialise System
  HSE_CLK_Init();//init external clock
  
  USART_Cust_Init(); //init UART
  Relay_GPIO_Struct=Init_Relays(); //Init relay pins
  Initialise_ADCs();

  Initialise_IRQs(); //setup IRQs
  
  PB_LCD_Init(); //Init Screen
  
  return 0;
}

int main(void){
  setup();
  PB_LCD_Clear();
  
  char buff[50];
  int sTicks=0;

  
  while (1){ //Main control loop
    //Switch_Relay(1, Relay_GPIO_Struct);
    int size;
    size=sizeof(buff);
    if ((usTicks==100000)){//increment seconds, and reset useconds each second.
      
      sprintf(buff,"S: %d | %d",sTicks,size);
      sTicks++;
      redraw_display(buff);
      SerialWrite_String(buff);
      SerialWrite_String("\n\r");
      usTicks=0;
      
    } else if (usTicks==50000){ //Update display twice per second
      sprintf(buff,"S: %d | %d",sTicks,size);
      redraw_display(buff);
    }  
    
  }
}


void SysTick_Handler(void)  {                               /* SysTick interrupt Handler. */
  usTicks++;                                                /* See startup file startup_DEVICE.s for SysTick vector */ 
}

void ADC_IRQHandler (void) {
  //SerialWrite_String("ADC\n\r");
  ADC_result = ADC1->DR;
  ADC1->CR2 |= ((0b1<<30));//Start new conversion
}
