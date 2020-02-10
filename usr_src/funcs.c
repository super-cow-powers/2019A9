#include "funcs.h"
#include "PB_LCD_Drivers.h"

GPIO_InitTypeDef* Init_Relays(void){
  GPIO_InitTypeDef* GPIO_Struct=malloc(sizeof(GPIO_InitTypeDef));

  int relay_pins = (GPIO_Pin_5);
  //Enable clock on gpio A
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  
  // configure port A for driving LEDs
  GPIO_Struct->GPIO_Pin = relay_pins;
  GPIO_Struct->GPIO_Mode = GPIO_Mode_OUT;    // output
  GPIO_Struct->GPIO_OType = GPIO_OType_PP; //push-pull mode
  GPIO_Struct->GPIO_Speed = 0x03;   // high speed
  GPIO_Struct->GPIO_PuPd = GPIO_PuPd_DOWN; //Pull Down
  GPIO_Init(GPIOA, GPIO_Struct) ;             // initialize port

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
