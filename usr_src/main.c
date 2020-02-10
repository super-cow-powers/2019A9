#include "funcs.h"
#include "PB_LCD_Drivers.h"
//Global variable defs go here please
GPIO_InitTypeDef* Relay_GPIO_Struct;


int setup(void){
  //Relay_GPIO_Struct=Init_Relays();
  PB_LCD_Init();
  PB_LCD_WriteChar ('A');
  PB_LCD_Clear();
  return 0;
}

int main(void){
  setup();
  PB_LCD_WriteString ("This Is Not A String",22);
  while (1){ //Main control loop
    //Switch_Relay(1, Relay_GPIO_Struct);  
    
  }
}
