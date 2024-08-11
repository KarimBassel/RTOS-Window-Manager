#include "TM4c123gh6pm.h"
#include "bitwise_operation.h"
#include "DIO.h"


void DIO_Init() {
  // initialize the clock for port F
  SYSCTL_RCGCGPIO_R |= 0x00000020;
  
  // wait for the clock
  while ((SYSCTL_PRGPIO_R & 0x00000020) == 0) {};
  
  // unlock port F
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  
  // set commit register
  GPIO_PORTF_CR_R = 0x1F;
  
  // set pins direction
  GPIO_PORTF_DIR_R = 0x0E;
  
  // set PUR for input pins
  GPIO_PORTF_PUR_R = 0x11;
  // enable pins using digital enable register
  GPIO_PORTF_DEN_R = 0x1F;
}

void DIO_WritePin(vulong32_ptr port, uint8 pin, uint8 value) {
  if (value == 1) Set_Bit(*port, pin);
  else Clear_Bit(*port, pin);
}
uint32 DIO_ReadPin(vulong32_ptr port, uint8 pin){
  return Get_Bit(*port,pin);
}
void DIO_WritePort(vulong32_ptr port, uint32 value) {
  *port = value;
}
void DIO_TogglePin(vulong32_ptr port,uint8 pin){
  Toggle_Bit(*port, pin)  ;
}