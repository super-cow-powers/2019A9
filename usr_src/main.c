#include "funcs.h"
#include "PB_LCD_Drivers.h"


//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct; 

volatile uint32_t ADC_result = 0, msTicks = 0;                        /* Variable to store millisecond ticks */
volatile int pressed_button = 0, new_ADC_val = 0;
volatile char UART_input = '0';

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
  int sTicks=0, Current_ADC_Val_Index=0;
  float32_t stats_output=0;
  ADC_Buffer ADC_Vals;

  Mode CurrentMode;
  CurrentMode.MeasureMode=DC_V;
  CurrentMode.MeasureType=MEAN;
  CurrentMode.CurrentRange=one;
  
  modeSwitch(&CurrentMode,pressed_button); //
  
  unsigned int zero_point=init_vm();

  redraw_display(buff, CurrentMode);

  while (1){ //Main control loop
    
    if (!(msTicks % 200)){
      
      sTicks++;
      sprintf(buff,"ADC:%d, %d, %d",(int)(ADC_result-zero_point),CurrentMode.MeasureMode,CurrentMode.CurrentRange);
      SerialWrite_String(buff);
      SerialWrite_String("\n\r");
        
      redraw_display(buff, CurrentMode);      
    } 
    if (!(msTicks % 10)){
      pressed_button=modeSwitch(&CurrentMode,pressed_button); //Switches mode, setting the pressed button back to 0 (default).
    } //Also effectively acts as debouncing for the buttons.
    
    if (ADC_Vals.current_value == Max_ADC_Vals){ //if the ADC buffer is full, do stats to it using "quick maffs".
      switch (CurrentMode.MeasureType){
      case MEAN:
	arm_mean_f32(ADC_Vals.ADC_Vals,Max_ADC_Vals,&stats_output); //Find mean
	break;
      case MAX:
	arm_max_f32(ADC_Vals.ADC_Vals,Max_ADC_Vals,&stats_output,NULL); //Find max
	break;
      case RMS:
	arm_rms_f32(ADC_Vals.ADC_Vals,Max_ADC_Vals,&stats_output); //Find RMS
	break;
      }
      
      //Do stats stuff to readings using CMSIS-DSP libs. See here for more details: https://arm-software.github.io/CMSIS_5/DSP/html/group__groupStats.html
      Current_ADC_Val_Index = 0;
    }
    if (new_ADC_val == 1){ //copy new adc value into buffer if there is one, after adjusting for the zero offset.
      ADC_Vals.ADC_Vals[ADC_Vals.current_value] = ADC_result-zero_point;
      new_ADC_val = 0;
      ADC_Vals.current_value++;
    }
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
