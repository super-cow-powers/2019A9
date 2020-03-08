#include "funcs.h"
#include "PB_LCD_Drivers.h"


//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct; 

volatile uint32_t ADC_result = 0, msTicks = 0;                        /* Variable to store millisecond ticks */
volatile int pressed_button=0;
volatile int new_ADC_val=0;  

float32_t ADC_Vals[Max_ADC_Vals];

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
  float32_t RMS=0,MEAN=0,MAX=0;

  Mode CurrentMode;
  
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
    
    if (Current_ADC_Val_Index == Max_ADC_Vals){
      arm_rms_f32(ADC_Vals,Max_ADC_Vals,&RMS); //Find RMS
      //Do something from here: https://arm-software.github.io/CMSIS_5/DSP/html/group__groupStats.html
      Current_ADC_Val_Index = 0;
    }
    if (new_ADC_val == 1){
      ADC_Vals[Current_ADC_Val_Index]=ADC_result-zero_point;
      new_ADC_val = 0;
      Current_ADC_Val_Index++;
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

