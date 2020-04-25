#include "funcs.h"
#include "PB_LCD_Drivers.h"


void Init_Buttons(void){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  //Buttons are on PORT:E.(8-15)
  
  GPIO_InitTypeDef GPIO_Struct;
  GPIO_InitTypeDef LED_Struct;

  
  int button_pins = (GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); //pin 8-15
  
  
  GPIO_Struct.GPIO_Pin = button_pins;
  GPIO_Struct.GPIO_Mode = GPIO_Mode_IN;    // output
  GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL; //Pull up
  GPIO_Init(GPIOE, &GPIO_Struct);             // initialize port

  LED_Struct.GPIO_Pin = button_pins;
  LED_Struct.GPIO_Mode = GPIO_Mode_OUT;    // output
  LED_Struct.GPIO_OType = GPIO_OType_PP; //push-pull mode
  LED_Struct.GPIO_Speed = 0x03;   // high speed
  LED_Struct.GPIO_PuPd = GPIO_PuPd_DOWN; //Pull Down
  GPIO_Init(GPIOD, &LED_Struct) ;             // initialize port
  
  SYSCFG->EXTICR[2] &= ~0x4444; //enable the correct lines to Port E
  SYSCFG->EXTICR[2] = 0x4444;
  SYSCFG->EXTICR[3] &= ~0x4444;
  SYSCFG->EXTICR[3] = 0x4444;

  EXTI->IMR |= (0XFF<<8); //enable interrupt on line
  EXTI->FTSR &= ~(0XFF<<8); //Disable falling edge
  EXTI->RTSR |= (0XFF<<8); //Enable rising edge

}

void Init_Relays(void){
  GPIO_InitTypeDef GPIO_Struct;

  int relay_pins = (GPIO_Pin_6|GPIO_Pin_5|GPIO_Pin_3|GPIO_Pin_4); //GPIO pins for relays
  //Enable clock on gpio E
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  
  // configure port E for driving Relays
  GPIO_Struct.GPIO_Pin = relay_pins;
  GPIO_Struct.GPIO_Mode = GPIO_Mode_OUT;    // output
  GPIO_Struct.GPIO_OType = GPIO_OType_PP; //push-pull mode
  GPIO_Struct.GPIO_Speed = 0x03;   // high speed
  GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP; //Pull Down
  GPIO_Init(GPIOE, &GPIO_Struct) ;             // initialize port

}

void Switch_Relay(int relay){ //Switch relays on Port A (only one on at once)
  int relay_pin_OS=3; //Relays start at PE.3
  
  if (relay<=3){
    GPIOE->ODR &= ~(0xF<<3); //Clear relays
    GPIOE->ODR |= (0b1<<(relay+relay_pin_OS));
  }
  else{
    SerialWrite_String("BAD RELAY NUMBER\n");
  }

}

void USART_Cust_Init(void){

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;          /* Enable GPIOA clock */
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;    /* Enable USART2 clock */
    
  USART2->BRR=52<<2; //CHECK THIS. POSSIBLE CLOCK ISSUES
  //USART2->BRR|=(0b0010);
 
  /* Configure PA2, PA3 for USART2 TX, RX */
  GPIOA->AFR[0] &= ~0xFF00;
  GPIOA->AFR[0] |=  0x7700;   /* alt7 for USART2 */
  GPIOA->MODER  &= ~0x00F0;
  GPIOA->MODER  |=  0x00A0;   /* enable alt. function for PA2, PA3 */
    

  USART2->CR1|=USART_CR1_RE|USART_CR1_TE|USART_CR1_UE;//Enable RX, TX, UART
}

void SerialWrite_Char(char data){
  while (!(USART2->SR & USART_SR_TXE)){};
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

  SystemCoreClockUpdate(); //Update core clock value

  return 0;
}

void Initialise_IRQs(void){
  NVIC_EnableIRQ(ADC_IRQn); //Enable NVIC for ADCs
  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  NVIC_SetPriority(EXTI9_5_IRQn,-1);//higher priority than ADCs
  NVIC_SetPriority(EXTI15_10_IRQn,-1);//higher priority than ADCs
  
  NVIC_SetPriority(ADC_IRQn, 0); //Set ADC priority.

  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_EnableIRQ(TIM4_IRQn); //Enable Timer 4
  SysTick_Config(SystemCoreClock / 1000); //Set SysTick to 1ms
}

void Initialise_ADCs(void){
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //Enable clock to ADC1
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;//Clock to gpio C (use pin PC.4/ADC12_IN14)
  GPIOC->MODER |= (GPIO_Mode_AN<<8);

  ADC1->CR1 &= ~0xFFFFFFFF;
  ADC1->CR1 |= ((1<<11)|(1<<5));

  ADC1->CR2 &= ~0xFFFFFFFF;
  ADC1->CR1 |= ((1<<10)|(1<<5));

  ADC1->SQR1 &= ~0xF00000; //L = 0000
  ADC1->SQR3 = 14; //Channel 14

  ADC1->CR2 |= ((0b1)); //Enable ADC1
  ADC1->CR2 |= (0b1<<30);
}


void init_FRQ(void){
  //Configure 16 bit Timer 4 to count from Pin P.E 0
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; //Enable GPIOD if not already
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  GPIOE->MODER &= ~(0b10); //Clear P.E 0 mode
  GPIOE->MODER |= (0b10); //Set P.E 0 to AF mode
  GPIOE->AFR[0] &= (0b0010); //Set P.E 0 to AF mode 2
  GPIOE->AFR[0] |= (0b0010);
  
  TIM4->SMCR &= ~(0b111);
  TIM4->SMCR |= (0b111); //Set External Clock Mode 1
  TIM4->SMCR |= (0b1<<14); //Enable external clock
  TIM4->SMCR &= ~(0b11<<13); //Disable Prescaler for now. Change in measurement fn if too fast
  TIM4->DIER = 0b1;
  TIM4->CR1 |= 0b1<<2; //Only an overflow/underflow can trigger IRQ
  TIM4->CR1 |= 0b1; //Enable the counter
  
}
char* output_to_string(float value){ //Converts the floating-point output to a string to be shown
  char* num_str = (char*)malloc(5*sizeof(char));
  int int_of_val = (int)value;//required because newlib-nano does not default allow floats in *printf 
  sprintf(num_str,"%d.%d%d",int_of_val,(int)((value-int_of_val)*10),(int)((value-int_of_val)*100));
  return num_str;
}

void update_Serial_out(float value, Mode* CurrentMode){
  char* number = output_to_string(value);
  char* mode = (char*)malloc(10*sizeof(char));
  if ((CurrentMode->MeasureMode != FRQ)&&(CurrentMode->MeasureMode != RES)){
    switch (CurrentMode->CurrentRange){
    case (tenm):
      strcpy(mode,"10m");
      break;
    case (hundredm):
      strcpy(mode,"100m");
      break;
    case (one):
      strcpy(mode,"");
      break;
    case (ten):
      strcpy(mode,"10");
      break;
    }
  } else {
    strcpy(mode,"");
  }
  
  switch (CurrentMode->MeasureMode){
  case (DC_V):
    strcat(mode,"V");
    break;
  case (AC_V):
    strcat(mode,"Vrms");
    break;
  case (DC_I):
    strcat(mode,"A");
    break;
  case (AC_I):
    strcat(mode,"Arms");
    break;
  case (FRQ):
    strcat(mode,"Hz");
    break;
  case (RES):
    strcat(mode,"Ohms");
    break;
  }
  SerialWrite_String(number);
  SerialWrite_String(mode);
  free(number);//Clean up after yourself.
  free(mode);
}

void redraw_display(float value, Mode CurrentMode){
  char* buffer = output_to_string(value);
  PB_LCD_WriteString("                ");
  PB_LCD_GoToXY((8-(strlen(buffer))),0);
  //PB_LCD_GoToXY(0,0);
  PB_LCD_WriteString(buffer);
  PB_LCD_GoToXY(0,1);
  
  switch (CurrentMode.MeasureMode){
  case (DC_V):
    PB_LCD_WriteString("DC V");
    break;
  case (AC_V):
    PB_LCD_WriteString("AC Vrms");
    break;
  case (DC_I):
    PB_LCD_WriteString("DC I");
    break;
  case (AC_I):
    PB_LCD_WriteString("AC Irms");
    break;
  case (FRQ):
    PB_LCD_WriteString("Frq");
    break;
  case (RES):
    PB_LCD_WriteString("Ohms");
    break;
  }
  if ((CurrentMode.MeasureMode != FRQ)&&(CurrentMode.MeasureMode != RES)){
  switch (CurrentMode.CurrentRange){
  case (tenm):
    PB_LCD_WriteString(" (10 milli)");
    break;
  case (hundredm):
    PB_LCD_WriteString(" (100 milli)");
    break;
  case (one):
    PB_LCD_WriteString(" (1)");
    break;
  case (ten):
    PB_LCD_WriteString(" (10)");
    break;
  }
  } else {
    PB_LCD_WriteString("              ");
  }
  switch (CurrentMode.MeasureType){
  case (MEAN):
    PB_LCD_WriteString(" MEAN                     ");
    break;
  case (MAX):
    PB_LCD_WriteString(" MAX                     ");
    break;
  case (RMS):
    PB_LCD_WriteString(" RMS                     ");
    break;
  }
  PB_LCD_GoToXY(0,0);
  free(buffer);
}

unsigned int init_vm(void){
  uint32_t target_ms = msTicks + 2000;
  int i=0;//number of samples
  uint32_t sum=0;
  while (msTicks <= target_ms){
    sum += ADC_result;
    i++;
  }
  return (sum/i);
}

int modeSwitch(Mode* CurrentMode, int pressed_button){
  int buffer=CurrentMode->CurrentRange;
  switch (pressed_button){ //Sets mode state according to button press
  case (0):
    break;
  case (1):
    CurrentMode->MeasureMode=DC_V;

    if (CurrentMode->MeasureType != MAX){
      CurrentMode->MeasureType=MEAN;}
    
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0100);
    break;
  case (2):
    CurrentMode->MeasureMode=AC_V;
    
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0200);
    break;
  case (3):
    CurrentMode->MeasureMode=DC_I;
    
    if (CurrentMode->MeasureType != MAX){
      CurrentMode->MeasureType=MEAN;}
    
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0400);
    break;
  case (4):
    CurrentMode->MeasureMode=AC_I;
    
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x0800);
    break;
  case (5):
    CurrentMode->MeasureMode=FRQ;
    
    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x1000);
    break;
  case (6):
    CurrentMode->MeasureMode=RES;
    CurrentMode->MeasureType=MEAN;

    GPIOD->ODR &= ~(0xFF00);
    GPIOD->ODR |= (0x2000);
    break;
  case (7):
    buffer=CurrentMode->CurrentRange;
    buffer++;

    if (buffer>3){
    buffer=0;}
    
    CurrentMode->CurrentRange=buffer;
    Switch_Relay(CurrentMode->CurrentRange);
    //GPIOD->ODR &= ~(0xFF00);
    //GPIOD->ODR |= (0x4000);
    break;
  case (8):
    if ((CurrentMode->MeasureMode == AC_V)|(CurrentMode->MeasureMode == AC_I)){
      buffer=CurrentMode->MeasureType;
      buffer++;

      if (buffer>2){
	buffer=0;}
    
      CurrentMode->MeasureType=buffer;
    } else if ((CurrentMode->MeasureMode == RES)|(CurrentMode->MeasureMode == FRQ)){
      CurrentMode->MeasureType=MEAN;
    } else {
      buffer=CurrentMode->MeasureType;
       buffer++;

      if (buffer>1){
	buffer=0;}
    
      CurrentMode->MeasureType=buffer;
    }
    //GPIOD->ODR &= ~(0xFF00);
    //GPIOD->ODR |= (0x8000);
    break;
  }

    return 0;
  }

