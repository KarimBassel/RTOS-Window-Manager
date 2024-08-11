#include "TM4c123gh6pm.h"
#include "uart.h"




void printString(char *String, void (*printChar)(char)){
	 // This loop will continue until we get the null character
  while(*String){
  
    printChar(*String);
    String++;  }
}

void printChar(char c){
	while((UART0_FR_R & (1<<5)) != 0); // Wait until the FIFO is Empty to put the new byte
    UART0_DR_R = c;

}

char UART0Rx(void){
	 char c;
    while((UART0_FR_R & (1<<4)) != 0); // Wait until the FIFO is not empty
    c= UART0_DR_R ;                    // Read The Recieved DATA
    return c;

}
