#include "funcs.h"
#include "PB_LCD_Drivers.h"
#include <stdlib.h>
#include <stdio.h>


//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct; 
const int ADC_BUFF_SIZE = 1024;
volatile uint32_t ADC_result = 0, msTicks = 0; /*msTicks increments each millisecond */
volatile int pressed_button = 1, new_ADC_val = 0;
volatile char UART_input = '0';
volatile int counter_overflow=0;

int setup(void){
  SystemInit();//Initialise System
  HSE_CLK_Init();//init external clock
  
  USART_Cust_Init(); //init UART
  Init_Relays(); //Init relay pins
  Initialise_ADCs();
  PB_LCD_Init(); //Init Screen
  Initialise_IRQs(); //setup IRQs
  init_FRQ();
  Init_Buttons();
  
  return 0;
}

int main(void){
  setup();
  PB_LCD_Clear();
  
  int Current_ADC_Val_Index=0;
  float32_t* stats_output=(float32_t* )malloc(sizeof(float32_t));//ARM DSP libs has some funky types
  float32_t ADC_Vals_Vect[ADC_BUFF_SIZE]; //These are 32 bit fixed precision floats for example.
  float32_t ADC_Vals_Vect_TST[10]={4,76.54,-34,6.4,3,7,2,9,65,1}; //Test data for stats stuff
  
  Mode CurrentMode;
  CurrentMode.MeasureMode=DC_V;
  CurrentMode.MeasureType=MEAN;
  CurrentMode.CurrentRange=one;
  
  modeSwitch(&CurrentMode,pressed_button);
  
  unsigned int zero_point=init_vm();

  redraw_display(0, CurrentMode);
  
  while (1){ //Main control loop
    
    if (!(msTicks % 200)){
      update_Serial_out(*stats_output,&CurrentMode);
      //sprintf(buff,"%d %d",(int)*stats_output,counter_overflow);
      //SerialWrite_String(buff);
      //SerialWrite_String("\n\r");  
      redraw_display(*stats_output, CurrentMode);      
    } 
    if (!(msTicks % 10)){
      pressed_button=modeSwitch(&CurrentMode,pressed_button); //Switches mode, setting the pressed button back to 0 (default).
    } //Also effectively acts as debouncing for the buttons.
    
    if (Current_ADC_Val_Index == ADC_BUFF_SIZE){ //if the ADC buffer is full, do stats to it using "quick maffs".
      Current_ADC_Val_Index = 0;
      switch (CurrentMode.MeasureType){
      case MEAN:
	arm_mean_f32(ADC_Vals_Vect,ADC_BUFF_SIZE,stats_output); //Find mean
	break;
      case MAX:
	arm_max_f32(ADC_Vals_Vect,ADC_BUFF_SIZE,stats_output,NULL); //Find max
	break;
      case RMS:
	arm_rms_f32(ADC_Vals_Vect,ADC_BUFF_SIZE,stats_output); //Find RMS
	break;
      }
    }//Do stats stuff to readings using CMSIS-DSP libs. See here for more details: https://arm-software.github.io/CMSIS_5/DSP/html/group__groupStats.html
      if (new_ADC_val == 1){ //copy new adc value into buffer if there is one, after adjusting for the zero offset.
	ADC_Vals_Vect[Current_ADC_Val_Index] =(float32_t)((int)(ADC_result)-(int)(zero_point));
	new_ADC_val = 0;
	Current_ADC_Val_Index++;
      }
  }
}

//Begin IRQ Stuff:


void SysTick_Handler(void)  {                               /* SysTick interrupt Handler. */
  msTicks++;                                                /* See startup file startup_DEVICE.s for SysTick vector */ 
}

void ADC_IRQHandler (void) {
  //SerialWrite_String("ADC\n\r");
  ADC_result = ADC1->DR;
  new_ADC_val = 1;
  ADC1->CR2 |= ((0b1<<30));//Start new conversion
}

void EXTI9_5_IRQHandler (void) {
  switch (EXTI->PR){ //Find which button has been pressed
  case (1<<8):
    pressed_button=1;
    break;
  case (1<<9):
    pressed_button=2;
    break;
  }
  EXTI->PR = (0xFFFF);//Clear irq
}

void EXTI15_10_IRQHandler(void) {
  /* Make sure that interrupt flag is set */
  switch (EXTI->PR){ //Find which button has been pressed
  case (1<<10):
    pressed_button=3;
    break;
  case (1<<11):
    pressed_button=4;
    break;
  case (1<<12):
    pressed_button=5;
    break;
  case (1<<13):
    pressed_button=6;
    break;
  case (1<<14):
    pressed_button=7;
    break;
  case (1<<15):
    pressed_button=8;
    break;
  }
  
  EXTI->PR = (0xFFFF);//Clear irq
}

void USART2_IRQHandler (void) {
  if (USART2->SR & USART_SR_RXNE) {
    UART_input = USART2->DR;
  }
}

void TIM4_IRQHandler (void) {
  TIM4->SR &= ~(1); //Clear interrupt flags
  TIM4->SR &= ~(0b1<<6); //Restart
  counter_overflow += 1;
  
}
