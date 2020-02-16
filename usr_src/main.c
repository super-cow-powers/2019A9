#include "funcs.h"
#include "PB_LCD_Drivers.h"
//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct;


int setup(void){
  //Init External 8Mhz clock
  HSE_CLK_Init();
  //Relay_GPIO_Struct=Init_Relays();
  PB_LCD_Init();
  PB_LCD_Clear();
  PB_LCD_Clear();
  return 0;
}

int main(void){
  setup();
  USART_Cust_Init();
  char buff[50];
  SerialWrite_String("INIT DONE\n");
  
  PB_LCD_WriteString ("This Is Not A String",22);
  while (1){ //Main control loop
    //Switch_Relay(1, Relay_GPIO_Struct);
    sprintf(buff,"%ld %ld",(RCC->CFGR & (RCC_CFGR_SW_1)),(RCC->CFGR & (RCC_CFGR_SW_0)));
    PB_LCD_WriteString (buff,10);
    PB_LCD_Clear();
    //PB_LCD_WriteString ("Mine",22);
    //PB_LCD_Clear();
  }
}
