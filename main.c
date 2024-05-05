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

/****
BLUE -> MOTOR UP
RED -> MOTOR DOWN
****/

/**************** INTERRUPT DEFINITIONS ***********************/
#define PortF_IRQn 30
#define PortD_IRQn 3

/************************PORT B******************************/
#define GPIO_PORTB_CLK_EN  0x02      //enable clock for PORTB
#define GPIO_PORTB_PIN1_EN 0x02 		 //enable pin 1 of PORTB
#define GPIO_PORTB_PIN2_EN 0x04			 //enable pin 2 of PORTB
#define GPIO_PORTB_PIN3_EN 0x08			 //enable pin 3 of PORTB
#define GPIO_PORTB_PIN6_EN 0x040			 //enable pin 6 of PORTB used for limit switch for window up
#define GPIO_PORTB_PIN7_EN 0x080			 //enable pin 7 of PORTB used to trigger jamming
/************************PORT D******************************/
//#define GPIO_PORTD_CLK_EN  0x08      //enable clock for PORTD
//#define GPIO_PORTD_PIN2_EN 0x04			 //enable pin 2 of PORTD

/************** INIT FUNCTIONS DECLARTIONS ********************/
void InitTask(void *pvParameters);
void DelayPORTB(unsigned int);
void PORTB_Init(void);
void PORTD_Init(void);

static void vJammingTask( void *pvParameters);

/*********************** TASKS DECLARATION *******************/
void mainController(void *pvParameters); //MAIN CONTROLLER FUNCTION
void driverControl (void *pvParameter); // This function for driver controlling passenger's window
void passengerControl (void *pvParameter); // This function for passengers controlling passenger's window



/****************** Other DECLARATION ************************/

//void vPORTD_init(); 

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


/*********** Semaphore and qeueue Handles *******************/
xQueueHandle xQueue_MainController; //Queue for main controller
xSemaphoreHandle xBinarySemaphore_MainController; // semaphore for main controller
xSemaphoreHandle xBinarySemaphore_Driver; // semaphore for main controller
xSemaphoreHandle xBinarySemaphore_Passenger; // semaphore for main controller
xSemaphoreHandle xJammingSemaphore;

/********************* Tasks Handles ************************/
xTaskHandle driverControllerHandler;
xTaskHandle mainControllerHandler;
xTaskHandle passengerControllerHandler;
xTaskHandle jammingHandler;


/*-----------------------------------------------------------*/
int main( void )
{
	//vPORTF_init();
//	__asm ("CPSID  I\n");
	InitTask(NULL);
	PORTB_Init();
	//PORTD_Init();
	//vPORTD_init();

	
	xBinarySemaphore_Driver = xSemaphoreCreateBinary();
	xBinarySemaphore_Passenger = xSemaphoreCreateBinary();
	xJammingSemaphore = xSemaphoreCreateBinary();
	
  xQueue_MainController = xQueueCreate(1 , sizeof(long));

	if( xQueue_MainController != NULL && xBinarySemaphore_Driver!= NULL )
	{
	xTaskCreate(driverControl,"Driver Control",128,NULL,2,&driverControllerHandler);
	xTaskCreate(mainController,"main Control",128,NULL,1,&mainControllerHandler);
	xTaskCreate(passengerControl,"Passenger Control",128,NULL,2,&passengerControllerHandler);
	xTaskCreate(vJammingTask,"Jamming Control",128,NULL,2,&jammingHandler);

	vTaskStartScheduler();	
	}

	for( ;; ); 
}

/*-----------------------------------------------------------*/
void mainController( void *pvParameters )
{
	uint32_t windowUP = 0x01;
	uint32_t windowDown = 0x00;
	unsigned int portB_Pin3, portB_Pin2;

 for( ;; )
 {
			//portB_Pin4 = GPIO_PORTB_DATA_R & 0x10;
			portB_Pin3 = GPIO_PORTB_DATA_R & 0x08;
			portB_Pin2 = GPIO_PORTB_DATA_R & 0x04;
			//portB_Pin1 = GPIO_PORTB_DATA_R & 0x02;
	 
	 	 if((GPIO_PORTF_DATA_R & (1u << 0)) == 0) //Tiva button 0 is pressed //DRIVER WINDOW UP
		{
			xQueueSendToBack(xQueue_MainController, &windowUP, 0);	
			xSemaphoreGive(xBinarySemaphore_Driver);
	
		}
		else if((GPIO_PORTF_DATA_R & (1u << 4)) == 0) //Tiva button 4 is pressed //DRIVER WINDOW DOWN
		{
			xQueueSendToBack(xQueue_MainController, &windowDown, 0);	
			xSemaphoreGive(xBinarySemaphore_Driver);
	
		}

		else if(!portB_Pin3) // PORT B, PIN 3 //PASSENGER WINDOW UP
		{
			xQueueSendToBack(xQueue_MainController, &windowUP, 0);	
			xSemaphoreGive(xBinarySemaphore_Passenger);
	
		}
		else if(!portB_Pin2) // PORT B, PIN 2 // PASSENGER WINDOW DOWN
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
		
		//__asm  ("CPSIE  I\n");
		vToggle_Blue(); //MOTOR_UP
		motorUP();
		while(((GPIO_PORTF_DATA_R & (1u << 0)) == 0) && ((GPIO_PORTB_DATA_R & (1u << 7)) != 0) && ((GPIO_PORTB_DATA_R & (1u << 6)) != 0));
		if((GPIO_PORTB_DATA_R & (1u << 7)) == 0)
		{
			xSemaphoreGive(xJammingSemaphore);	
			vTaskPrioritySet(jammingHandler, 3);
		}
		
		//__asm  ("CPSID  I\n");
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
		motorUP();		
		vToggle_Blue(); //MOTOR_UP
		while(((GPIO_PORTB_DATA_R & (1u << 3)) == 0)&& ((GPIO_PORTB_DATA_R & (1u << 7)) != 0) && ((GPIO_PORTB_DATA_R & (1u << 6)) != 0));
		if((GPIO_PORTB_DATA_R & (1u << 7)) == 0)
		{
			xSemaphoreGive(xJammingSemaphore);	
			vTaskPrioritySet(jammingHandler, 3);
		}
		
	}
	else if(windowsDirection == 0){
		motorDOWN();
		vToggle_Red(); //MOTOR_DOWN
		while((GPIO_PORTB_DATA_R & (1u << 2)) == 0); //PORT B pin 2
		
	}
		motorOFF();
		vTurn_OFF();
		taskYIELD();

	}
	
}

/*-----------------------------------------------------------*/


static void vJammingTask( void *pvParameters )
{
	xSemaphoreTake( xJammingSemaphore, 0 );
	for( ;; )
	{
		xSemaphoreTake( xJammingSemaphore, portMAX_DELAY );
		
		/** TURN OFF MOTOR FIRST **/
		motorOFF();	
		vTurn_OFF();
		DelayPORTB(100);
		
		/** TURN MOTOR DOWN FOR 0.5 SECONDS **/
		motorDOWN();
		vToggle_Red();	
		DelayPORTB(500);
		
		/** TURN OFF MOTOR AGAIN **/
		motorOFF();
		vTurn_OFF();
		DelayPORTB(100);

		/** SET BACK PRIORITY TO 2 **/	
		vTaskPrioritySet(NULL,2);	
		
		/** TASKYIELD **/
		taskYIELD();

	}
}

/************** INIT FUNCTIONS IMPLEMENTATIONS ****************/
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
			//DIO_Init(PORT_D, 0, IN);
			//DIO_Init(PORT_B,3, IN);
}

void DelayPORTB(unsigned int delay)
{
	volatile unsigned int i, counter;
	counter = delay * 4000;  // 1 second (1000 msec) needs 40000000 counter so 4000000/1000 = 4000
	for(i=0;i<counter;i++);
}

void PORTB_Init(void)
{
	SYSCTL_RCGCGPIO_R |= GPIO_PORTB_CLK_EN;           //activate clock for Port B
  DelayPORTB(10);           															//Delay 10 msec to allow clock to start on PORTB  
	GPIO_PORTB_LOCK_R = 0x4C4F434B;           				//unlock GPIO of PORTB
	GPIO_PORTB_CR_R = 0x01;                   				//Enable GPIOPUR register enable to commit
	GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN3_EN;   				//Enable Pull Up SW on PB3	
	GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN2_EN;   				//Enable Pull Up SW on PB2	
	GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN3_EN;        	  // Enable pin 3 of PORTB 
	GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN2_EN;        	  // Enable pin 2 of PORTB 

	GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN6_EN;   				//Enable Pull Up SW on PB6	
	GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN7_EN;   				//Enable Pull Up SW on PB7	
	GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN6_EN;        	  // Enable pin 6 of PORTB 
	GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN7_EN;        	  // Enable pin 7 of PORTB 
	
	
	GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN6_EN ; 				// Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN6_EN;        // Disable analog function on pin 6 of PORTB
	GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN6_EN;        // Regular port function
	GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN7_EN ; 				// Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN7_EN;        // Disable analog function on pin 7 of PORTB
	GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN7_EN;        // Regular port function
	
	GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN3_EN ; 				// Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN3_EN;        // Disable analog function on pin 3 of PORTB
	GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN3_EN;        // Regular port function
	GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN2_EN ; 				// Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN2_EN;        // Disable analog function on pin 2 of PORTB
	GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN2_EN;        // Regular port function
}




//void PORTD_Init(void)
//{
//	SYSCTL_RCGCGPIO_R |= GPIO_PORTD_CLK_EN;           //activate clock for Port D
//  DelayPORTB(10);           															//Delay 10 msec to allow clock to start on PORTD  
//	GPIO_PORTD_LOCK_R = 0x4C4F434B;           				//unlock GPIO of PORTD
//	GPIO_PORTD_CR_R = 0x01;                   				//Enable GPIOPUR register enable to commit
//	GPIO_PORTD_PUR_R |= GPIO_PORTD_PIN2_EN;   				//Enable Pull Up SW on PD2	
//	GPIO_PORTD_DEN_R |= GPIO_PORTD_PIN2_EN;        	  // Enable pin 2 of PORTD 

//	GPIO_PORTD_PCTL_R &= ~GPIO_PORTD_PIN2_EN ; 				// Regular GPIO of PORTD
//  GPIO_PORTD_AMSEL_R &= ~GPIO_PORTD_PIN2_EN;        // Disable analog function on pin 2 of PORTD
//	GPIO_PORTD_AFSEL_R &= ~GPIO_PORTD_PIN2_EN;        // Regular port function
//}





//void vPORTF_init() 
//{
//	SYSCTL_RCGCGPIO_R |= 0x20;
//  while(!(SYSCTL_PRGPIO_R& 0x20));
//	  GPIO_PORTF_LOCK_R   = 0x4C4F434B;                       
//    GPIO_PORTF_CR_R    |=  (1<<0|1<<1|1<<2|1<<3|1<<4);  
//		GPIO_PORTF_AMSEL_R |= 0x00;    
//	  GPIO_PORTF_PCTL_R = 0x00000000;  
//	  GPIO_PORTF_AFSEL_R = 0x00;      
//    GPIO_PORTF_DIR_R   &= ~((1<<0) |(1<<4));                
//    GPIO_PORTF_DIR_R   |=  ((1<<1) | (1<<2) | (1<<3));       
//    GPIO_PORTF_PUR_R   |=  ((1<<0)|(1<<4));                  
//    GPIO_PORTF_DEN_R   |=  (1<<0|1<<1|1<<2|1<<3|1<<4);                             
//    GPIO_PORTF_DATA_R  &= ~((1<<1)|(1<<2)|(1<<3));      
//		GPIO_PORTF_ICR_R = 0x11;     // Clear any Previous Interrupt 
//		GPIO_PORTF_IM_R |=0x11;      
//// Unmask the interrupts for PF0 and PF4
//		GPIO_PORTF_IS_R |= 0x11;     // Make bits PF0 and PF4 level sensitive
//		GPIO_PORTF_IEV_R &= ~0x11;   // Sense on Low Level
//		NVIC_EN0_R |= (1<<PortF_IRQn);        
//}

//void vPORTD_init() 
//{
//	SYSCTL_RCGCGPIO_R |= 0x08;
//  while(!(SYSCTL_PRGPIO_R& 0x08));
//	  GPIO_PORTD_LOCK_R   = 0x4C4F434B;                       
//    GPIO_PORTD_CR_R    |=  (1<<0);  
//		GPIO_PORTD_AMSEL_R |= 0x00;    
//	  GPIO_PORTD_PCTL_R = 0x00000000;  
//	  GPIO_PORTD_AFSEL_R = 0x00;      
//    GPIO_PORTD_DIR_R   &= ~((1<<0));                
//    //GPIO_PORTF_DIR_R   |=  ((1<<1) | (1<<2) | (1<<3));       
//    GPIO_PORTD_PUR_R   |=  ((1<<0));                  
//    GPIO_PORTD_DEN_R   |=  (1<<0);                             
//    //GPIO_PORTF_DATA_R  &= ~((1<<1)|(1<<2)|(1<<3));      
//		GPIO_PORTD_ICR_R = 0x11;     // Clear any Previous Interrupt 
//		GPIO_PORTD_IM_R |=0x11;      
//		GPIO_PORTD_IS_R |= 0x11;     // Make bits PF0 and PF4 level sensitive
//		GPIO_PORTD_IEV_R &= ~0x11;   // Sense on Low Level
//		NVIC_EN0_R |= (1<<PortD_IRQn);        
//}

///*-----------------------------------------------------------*/



//void GPIOD_Handler( void )
//{
//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//		vTaskPrioritySet(jammingHandler, 3);
//    /* 'Give' the semaphore to unblock the task. */
//    xSemaphoreGiveFromISR( xJammingSemaphore, &xHigherPriorityTaskWoken );
//    /* Clear the software interrupt bit using the interrupt controllers
//    Clear Pending register. */
//    GPIO_PORTD_ICR_R = 0x11;     // Clear any Previous Interrupt 

//    /* Giving the semaphore may have unblocked a task - if it did and the
//    unblocked task has a priority equal to or above the currently executing
//    task then xHigherPriorityTaskWoken will have been set to pdTRUE and
//    portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
//    higher priority task.

//    NOTE: The syntax for forcing a context switch within an ISR varies between
//    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
//    the Cortex M3 port layer for this purpose.  taskYIELD() must never be called
//    from an ISR! */
//    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
//}