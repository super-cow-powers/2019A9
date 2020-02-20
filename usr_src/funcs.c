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
  NVIC_SetPriority(ADC_IRQn, 0); //Set ADCs to max priority. May change to polling later!!!
  //NVIC_SetPriority(SysTick_IRQn, -1);
  
  SysTick_Config(SystemCoreClock / 100000); //Set SysTick to 1ms
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
  ADC1->CR2 |= ((0b1<<30)); //take one conversion
}

void redraw_display(char* buffer){
  PB_LCD_WriteString("                ");
  PB_LCD_GoToXY(0,0);
  PB_LCD_WriteString(buffer);
  PB_LCD_GoToXY(0,1);
  PB_LCD_WriteString("TEST MODE");
  PB_LCD_GoToXY(0,0);
}
