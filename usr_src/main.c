
#include "funcs.h"

//Global variable defs go here please
GPIO_InitTypeDef* GPIO_InitStructureRelays;


int setup(void){
  GPIO_InitStructureRelays=Init_Relays();
  
  return 0;
}

int main(void){
  setup();

  while (1){ //Main control loop
  Switch_Relay(5, GPIO_InitStructureRelays);  


  }
}
