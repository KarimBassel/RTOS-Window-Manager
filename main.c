#include <stdint.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "TM4c123gh6pm.h"
#include "DIO.h"
#include "bitwise_operation.h"
#include "uart.h"
#include "queue.h"
#include "semphr.h"
#define PortA_IRQn 0
#define PortB_IRQn 1
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xMutex;
xQueueHandle xQueue;
int counter=0;
#define Manual_UP 2
#define Auto_UP 1
#define PortF_IRQn 30

void Motor_Init(){
 SYSCTL->RCGCGPIO |= 0x00000004; 
 GPIO_PORTC_LOCK_R = 0x4C4F434B;  
 GPIO_PORTC_CR_R = 0x30;  
 GPIO_PORTC_DIR_R = 0x30; 
	GPIO_PORTC_DEN_R = 0x30; 
}

void Switches_Init(){
	 SYSCTL->RCGCGPIO |= 0x00000008; 
	 GPIOD->LOCK = 0x4C4F434B;  
	 GPIOD->CR = 0xF;  
	 GPIOD->DIR = 0x00; 
	 GPIOD->PUR = 0xF;  
	 GPIOD->DEN = 0xF; 
}
void Switch_Init(){
 SYSCTL->RCGCGPIO |= 0x00000001; 
 GPIO_PORTA_LOCK_R = 0x4C4F434B;  
 GPIO_PORTA_CR_R = 0x8C; 
 GPIO_PORTA_DIR_R = 0x00; 
 GPIO_PORTA_DEN_R = 0x8C; 
 GPIO_PORTA_PUR_R = 0x8C;  
}
void IR_Init(){
		SYSCTL->RCGCGPIO |= 0x00000020; 
	 GPIOF->LOCK = 0x4C4F434B; // 2) unlock PortF PF0 
	 GPIOF->CR = 0x1F; // allow changes to PF4-0 
	 GPIOF->AMSEL= 0x00; // 3) disable analog function
	 GPIOF->PCTL = 0x00000000; // 4) GPIO clear bit PCTL 
	 GPIOF->DIR = 0x0E; // 5) PF4,PF0 input, PF3,PF2,PF1 output
		GPIOF->AFSEL = 0x00; // 6) no alternate function
	 GPIOF->PUR = 0x11; // enable pullup resistors on PF4,PF0 
	 GPIOF->DEN = 0x1F; // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	// Setup the interrupt on PortF
	GPIOF->ICR = 0x11; // Clear any Previous Interrupt 
	GPIOF->IM |=0x11; // Unmask the interrupts for PF0 and PF4
	GPIOF->IS |= 0x11; // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~0x11; // Sense on Low Level
	NVIC_PRI7_R |= (5<<21); 
	NVIC_EnableIRQ(PortF_IRQn); // Enable the Interrupt for PortF in NVIC
}
void MotorUp(){
	GPIO_PORTC_DATA_R=0x20;
}
void MotorDown(){
	GPIO_PORTC_DATA_R=0x10;
}
void MotorStop(){
	GPIO_PORTC_DATA_R=0x00;
}
void Task_Driver(){
portBASE_TYPE xStatus;
const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
int down=4;
int up=2;// up
int stop=1;//auto stop

	for(;;){
		xSemaphoreTake( xMutex, portMAX_DELAY );
		if((GPIO_PORTD_DATA_R & 0x4)==0x0){//driver up
			//vTaskPrioritySet(NULL,uxTaskPriorityGet(NULL)+1);
			while((GPIO_PORTD_DATA_R & 0x4)==0x0){
				xStatus = xQueueSendToBack( xQueue, &up, xTicksToWait );
				counter++;
				if( xStatus != pdPASS )
					{
					/* We could not write to the queue because it was full - this must
					be an error as the receiving task should make space in the queue
					as soon as both sending tasks are in the Blocked state. */
					//vPrintString( "Could not send to the queue.\n" );
					}
			}
			if(counter < 200000){
				while((GPIO_PORTA_DATA_R &0x4) != 0){
					xStatus = xQueueSendToBack( xQueue, &up, xTicksToWait );
					if( xStatus != pdPASS )
						{
						/* We could not write to the queue because it was full - this must
						be an error as the receiving task should make space in the queue
						as soon as both sending tasks are in the Blocked state. */
						//vPrintString( "Could not send to the queue.\n" );
						}
				}
			}
			counter=0;
			xStatus = xQueueSendToBack( xQueue, &stop, xTicksToWait );
					if( xStatus != pdPASS )
						{
						/* We could not write to the queue because it was full - this must
						be an error as the receiving task should make space in the queue
						as soon as both sending tasks are in the Blocked state. */
						//vPrintString( "Could not send to the queue.\n" );
						}
			
			//vTaskPrioritySet(NULL,uxTaskPriorityGet(NULL)-1);
		}
		/***************************************************************************************************************************/
		if((GPIO_PORTD_DATA_R & 0x1)== 0x0){//driver down
			while((GPIO_PORTD_DATA_R & 0x1)==0x0){
				xStatus = xQueueSendToBack( xQueue, &down, xTicksToWait );
				counter++;
				if( xStatus != pdPASS )
					{
					}
			}
			if(counter < 200000){
				while((GPIO_PORTA_DATA_R &0x8) != 0){
					xStatus = xQueueSendToBack( xQueue, &down, xTicksToWait );
					if( xStatus != pdPASS )
						{
						}
				}
			}
			counter=0;
			xStatus = xQueueSendToBack( xQueue, &stop, xTicksToWait );
			if( xStatus != pdPASS )
			{
			/* We could not write to the queue because it was full - this must
			be an error as the receiving task should make space in the queue
			as soon as both sending tasks are in the Blocked state. */
			//vPrintString( "Could not send to the queue.\n" );
			}
		}
		xSemaphoreGive( xMutex );
	}
}
void Task_Passenger(){
portBASE_TYPE xStatus;
const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
int down=4;
int up=2;// up
int stop=1;//auto stop
int counter=0;
	for(;;){
		xSemaphoreTake( xMutex, portMAX_DELAY );

		if(((GPIO_PORTD_DATA_R & 0x2)==0x0) && ((GPIO_PORTA_DATA_R & 0x80)==0x0)){//passenger up
			//vTaskPrioritySet(NULL,uxTaskPriorityGet(NULL)+1);
			while((GPIO_PORTD_DATA_R & 0x2)==0x0){
				xStatus = xQueueSendToBack( xQueue, &up, xTicksToWait );
				counter++;
				if( xStatus != pdPASS )
					{
					/* We could not write to the queue because it was full - this must
					be an error as the receiving task should make space in the queue
					as soon as both sending tasks are in the Blocked state. */
					//vPrintString( "Could not send to the queue.\n" );
					}
			}
			if(counter < 200000){
				while((GPIO_PORTA_DATA_R &0x4) != 0){
					xStatus = xQueueSendToBack( xQueue, &up, xTicksToWait );
					if( xStatus != pdPASS )
						{
						/* We could not write to the queue because it was full - this must
						be an error as the receiving task should make space in the queue
						as soon as both sending tasks are in the Blocked state. */
						//vPrintString( "Could not send to the queue.\n" );
						}
				}
			}
			counter=0;
			xStatus = xQueueSendToBack( xQueue, &stop, xTicksToWait );
					if( xStatus != pdPASS )
						{
						/* We could not write to the queue because it was full - this must
						be an error as the receiving task should make space in the queue
						as soon as both sending tasks are in the Blocked state. */
						//vPrintString( "Could not send to the queue.\n" );
						}
			
			//vTaskPrioritySet(NULL,uxTaskPriorityGet(NULL)-1);
		}
		/***************************************************************************************************************************/
		if(((GPIO_PORTD_DATA_R & 0x8)==0x0)&&((GPIO_PORTA_DATA_R & 0x80)==0x0)){//passenger down
			while((GPIO_PORTD_DATA_R & 0x8)==0x0){
				xStatus = xQueueSendToBack( xQueue, &down, xTicksToWait );
				counter++;
				if( xStatus != pdPASS )
					{
					/* We could not write to the queue because it was full - this must
					be an error as the receiving task should make space in the queue
					as soon as both sending tasks are in the Blocked state. */
					//vPrintString( "Could not send to the queue.\n" );
					}
			}
			if(counter < 200000){
				while((GPIO_PORTA_DATA_R &0x8) != 0){
					xStatus = xQueueSendToBack( xQueue, &down, xTicksToWait );
					if( xStatus != pdPASS )
						{
						/* We could not write to the queue because it was full - this must
						be an error as the receiving task should make space in the queue
						as soon as both sending tasks are in the Blocked state. */
						//vPrintString( "Could not send to the queue.\n" );
						}
				}
			}
			counter=0;
			xStatus = xQueueSendToBack( xQueue, &stop, xTicksToWait );
			if( xStatus != pdPASS )
			{
			/* We could not write to the queue because it was full - this must
			be an error as the receiving task should make space in the queue
			as soon as both sending tasks are in the Blocked state. */
			//vPrintString( "Could not send to the queue.\n" );
			}
		}
		xSemaphoreGive( xMutex );
	}
}
static void Task_Rec(){
	int rec;
	portBASE_TYPE xStatus;
	for( ;; )
		{
		if( uxQueueMessagesWaiting( xQueue ) != 1 )
		{
			//vPrintString( "Queue should have been full!\n" );
		}	
		xStatus = xQueueReceive( xQueue, &rec, portMAX_DELAY );
	
		if( xStatus == pdPASS )
		{
			if(rec == 2){
				MotorUp();
			}
			else if(rec ==1){
				MotorStop();
			}
			else if(rec==4){
				MotorDown();
			}
		}
		else
		{
		}
	}
}
void Task_Safety(){
	xSemaphoreTake( xBinarySemaphore, 0 );

    for( ;; )
    {
        xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
				MotorDown();
			  for(int i=0;i<1000000;i++);
				MotorStop();
				xSemaphoreGive( xBinarySemaphore );
    }
}
void vApplicationIdleHook(void){
	
}
int main( void ) 

{ 
	xQueue = xQueueCreate( 1, sizeof( int ) );
	Switch_Init();
	//IR_Init();
	Switches_Init();
	Motor_Init();
	xMutex = xSemaphoreCreateMutex();	
	xBinarySemaphore = xSemaphoreCreateBinary();
	xTaskCreate( Task_Driver, "Task Driver", 100, NULL, 1, NULL ); 
	xTaskCreate( Task_Passenger, "Task Passenger", 100, NULL, 1, NULL ); 
	xTaskCreate( Task_Rec, "Task Receive", 100, NULL, 3, NULL );
	xTaskCreate( Task_Safety, "Task Safety", 100, NULL, 4, NULL );
	if(xQueue !=NULL)
	vTaskStartScheduler();
} 
//void GPIOA_Handler(void){
//	GPIO_PORTA_ICR_R  =0x1;
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

//    /* 'Give' the semaphore to unblock the task. */
//    xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );


//    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}
void GPIOF_Handler(void){
	GPIO_PORTF_ICR_R  =0x1;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* 'Give' the semaphore to unblock the task. */
    xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );

    /* Clear the software interrupt bit using the interrupt controllers
    Clear Pending register. */
    //GPIO_PORTF__R  =0x1;

    /* Giving the semaphore may have unblocked a task - if it did and the
    unblocked task has a priority equal to or above the currently executing
    task then xHigherPriorityTaskWoken will have been set to pdTRUE and
    portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
    higher priority task.

    NOTE: The syntax for forcing a context switch within an ISR varies between
    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
    the Cortex M3 port layer for this purpose.  taskYIELD() must never be called
    from an ISR! */
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06




