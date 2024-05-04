/* FreeRTOS includes. */
#include "FreeRTOS.h" 
#include "FreeRTOSConfig.h"
#include "task.h" 
//#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdio.h>
//#include "uart.h"
//#include "dio.h"
#include "queue.h"
#include <stdlib.h>
#include "motor.h"
#include "semphr.h" //to enable semahphores
#define PortF_IRQn 30
/*********** TASKS DECLARATION *******************/

void mainController(void *pvParameters); //MAIN CONTROLLER FUNCTION
void driverControl (void *pvParameter); // This function for driver controlling passenger's window
void passengerControl (void *pvParameter); // This function for passengers controlling passenger's window


/*************************************************/
/*********** Other DECLARATION *******************/

void vToggle_Blue()
{
	GPIO_PORTF_DATA_R ^= 1<<2;
}
void vToggle_Red()
{
	GPIO_PORTF_DATA_R ^= 1<<1;
}
void vTurn_OFF()
{
    GPIO_PORTF_DATA_R  &= ~((1<<1)|(1<<2)|(1<<3));      
}

/*************************************************/

/*********** Semaphore and qeueue Handles *******************/
xQueueHandle xQueue_MainController; //Queue for main controller
xSemaphoreHandle xBinarySemaphore_MainController; // semaphore for main controller
xSemaphoreHandle xBinarySemaphore_Driver; // semaphore for main controller
xSemaphoreHandle xBinarySemaphore_Passenger; // semaphore for main controller



/*************************************************/


/*********** Tasks Handles *******************/

xTaskHandle driverControllerHandler;
xTaskHandle mainControllerHandler;
xTaskHandle passengerControllerHandler;


/*************************************************/


void vPORTF_init() 
{
	SYSCTL_RCGCGPIO_R |= 0x20;
  while(!(SYSCTL_PRGPIO_R& 0x20));
	  GPIO_PORTF_LOCK_R   = 0x4C4F434B;                       
    GPIO_PORTF_CR_R    |=  (1<<0|1<<1|1<<2|1<<3|1<<4);  
		GPIO_PORTF_AMSEL_R |= 0x00;    
	  GPIO_PORTF_PCTL_R = 0x00000000;  
	  GPIO_PORTF_AFSEL_R = 0x00;      
    GPIO_PORTF_DIR_R   &= ~((1<<0) |(1<<4));                
    GPIO_PORTF_DIR_R   |=  ((1<<1) | (1<<2) | (1<<3));       
    GPIO_PORTF_PUR_R   |=  ((1<<0)|(1<<4));                  
    GPIO_PORTF_DEN_R   |=  (1<<0|1<<1|1<<2|1<<3|1<<4);                             
    GPIO_PORTF_DATA_R  &= ~((1<<1)|(1<<2)|(1<<3));      
		GPIO_PORTF_ICR_R = 0x11;     // Clear any Previous Interrupt 
		GPIO_PORTF_IM_R |=0x11;      
// Unmask the interrupts for PF0 and PF4
		GPIO_PORTF_IS_R |= 0x11;     // Make bits PF0 and PF4 level sensitive
		GPIO_PORTF_IEV_R &= ~0x11;   // Sense on Low Level
		NVIC_EN0_R |= (1<<PortF_IRQn);        
}

void InitTask(void *pvParameters);
/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/
int main( void )
{
	//vPORTF_init();
	InitTask(NULL);
	xBinarySemaphore_Driver = xSemaphoreCreateBinary();
	xBinarySemaphore_Passenger = xSemaphoreCreateBinary();

  xQueue_MainController = xQueueCreate(1 , sizeof(long));

	if( xQueue_MainController != NULL && xBinarySemaphore_Driver!= NULL )
	{
	xTaskCreate(driverControl,"Driver Control",180,NULL,2,&driverControllerHandler);
	xTaskCreate(mainController,"main Control",180,NULL,1,&mainControllerHandler);
	xTaskCreate(passengerControl,"Passenger Control",180,NULL,2,&passengerControllerHandler);

	vTaskStartScheduler();	
	}

	for( ;; ); 
}





/*-----------------------------------------------------------*/
//This function is used to initialize UART and buttons
void InitTask(void *pvParameters) 
{
	    //UARTInit(0,1,1,104,11,0, 3);
			DIO_Init(PORT_F,0,IN);
			DIO_Init(PORT_F,1, OUT);
			DIO_Init(PORT_F,2, OUT);
			DIO_Init(PORT_F,3, OUT);
			DIO_Init(PORT_F,4,IN);
			motorInit();
			DIO_Init(PORT_B, 2, IN);
			DIO_Init(PORT_B,3, IN);

}
/*-----------------------------------------------------------*/
void mainController( void *pvParameters )
{
	uint32_t windowUP = 0x01;
	uint32_t windowDown = 0x00;
 for( ;; )
 {
	 
	 	 if((GPIO_PORTF_DATA_R & (1u << 0)) == 0) //Tiva button 0 is pressed
		{
			xQueueSendToBack(xQueue_MainController, &windowUP, 0);	
			xSemaphoreGive(xBinarySemaphore_Driver);
	
		}
		else if((GPIO_PORTF_DATA_R & (1u << 4)) == 0) //Tiva button 4 is pressed
		{
			xQueueSendToBack(xQueue_MainController, &windowDown, 0);	
			xSemaphoreGive(xBinarySemaphore_Driver);
	
		}
		else if((GPIO_PORTB_DATA_R & (1u << 3)) == 1) // PORT B, PIN 2
		{
			xQueueSendToBack(xQueue_MainController, &windowUP, 0);	
			xSemaphoreGive(xBinarySemaphore_Passenger);
	
		}
		else if((GPIO_PORTB_DATA_R & (1u << 2)) == 1) // PORT B, PIN 3
		{
			xQueueSendToBack(xQueue_MainController, &windowDown, 0);	
			xSemaphoreGive(xBinarySemaphore_Passenger);
		}
	 
 }
}

/*-----------------------------------------------------------*/
void driverControl (void *pvParameter) // This function for driver controlling passenger's window
{
			xSemaphoreTake(xBinarySemaphore_Driver,0);
	uint32_t windowsDirection;
	for(;;)
	{
		xSemaphoreTake(xBinarySemaphore_Driver,portMAX_DELAY);
		xQueueReceive(xQueue_MainController,&windowsDirection ,portMAX_DELAY);
		
	if(windowsDirection == 1){
		
		vToggle_Blue(); //MOTOR_UP
		motorUP();
		while((GPIO_PORTF_DATA_R & (1u << 0)) == 0);
		
		}
		
	else if(windowsDirection == 0){
		motorDOWN();
		vToggle_Red();//MOTOR_DOWN
		while((GPIO_PORTF_DATA_R & (1u << 4)) == 0);
		
	}
		vTurn_OFF();
		motorOFF();
		taskYIELD();

	}
	
	
} 
/*-----------------------------------------------------------*/
void passengerControl (void *pvParameter) // This function for passengers controlling passenger's window
{
	
			xSemaphoreTake(xBinarySemaphore_Passenger,0);
	uint32_t windowsDirection;
	for(;;)
	{
		xSemaphoreTake(xBinarySemaphore_Passenger,portMAX_DELAY);
		xQueueReceive(xQueue_MainController,&windowsDirection ,portMAX_DELAY);
		
	if(windowsDirection == 1){
		
		vToggle_Blue(); //MOTOR_UP
		while((GPIO_PORTB_DATA_R & (1u << 2)) == 0);
	}
	else if(windowsDirection == 0){
		
		vToggle_Red(); //MOTOR_DOWN
		while((GPIO_PORTB_DATA_R & (1u << 3)) == 0);
		
	}
		vTurn_OFF();
		taskYIELD();

	}
	
	
	
}
