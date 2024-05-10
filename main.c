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
#define GPIO_PORTB_CLK_EN 0x02 //enable clock for PORTB
#define GPIO_PORTB_PIN1_EN 0x02 //enable pin 1 of PORTB
#define GPIO_PORTB_PIN2_EN 0x04 //enable pin 2 of PORTB
#define GPIO_PORTB_PIN3_EN 0x08 //enable pin 3 of PORTB

#define GPIO_PORTB_PIN4_EN 0x010 //enable pin 4 of PORTB used for ON/OFF Switch
#define GPIO_PORTB_PIN5_EN 0x020 //enable pin 5 of PORTB used for limit switch for window down
#define GPIO_PORTB_PIN6_EN 0x040 //enable pin 6 of PORTB used for limit switch for window up
#define GPIO_PORTB_PIN7_EN 0x080 //enable pin 7 of PORTB used to trigger jamming
/************************PORT D******************************/
//#define GPIO_PORTD_CLK_EN  0x08      //enable clock for PORTD
//#define GPIO_PORTD_PIN2_EN 0x04			 //enable pin 2 of PORTD

/************** INIT FUNCTIONS DECLARTIONS ********************/
void InitTask(void * pvParameters);
void DelayPORTB(unsigned int);
void PORTB_Init(void);
void PORTD_Init(void);

static void vJammingTask(void * pvParameters);

/*********************** TASKS DECLARATION *******************/
void mainController(void * pvParameters); //MAIN CONTROLLER FUNCTION
void driverControl(void * pvParameter); // This function for driver controlling passenger's window
void passengerControl(void * pvParameter); // This function for passengers controlling passenger's window

/****************** Other DECLARATION ************************/

//void vPORTD_init(); 

void vToggle_Blue() {
  GPIO_PORTF_DATA_R ^= 1 << 2;
}
void vToggle_Red() {
  GPIO_PORTF_DATA_R ^= 1 << 1;
}
void vTurn_OFF() {
  GPIO_PORTF_DATA_R &= ~((1 << 1) | (1 << 2) | (1 << 3));
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
int main(void) {
  InitTask(NULL);
  PORTB_Init();
  xBinarySemaphore_Driver = xSemaphoreCreateBinary(); //Just Creating semaphores
  xBinarySemaphore_Passenger = xSemaphoreCreateBinary(); //Just Creating semaphores
  xJammingSemaphore = xSemaphoreCreateBinary();//Just Creating semaphores

  xQueue_MainController = xQueueCreate(1, sizeof(long));

  if (xQueue_MainController != NULL && xBinarySemaphore_Driver != NULL) {
    xTaskCreate(driverControl, "Driver Control", 128, NULL, 2, & driverControllerHandler); //Driver control task Priority -> 2
    xTaskCreate(mainController, "main Control", 128, NULL, 1, & mainControllerHandler);//Main control task Priority -> 1
    xTaskCreate(passengerControl, "Passenger Control", 128, NULL, 2, & passengerControllerHandler);//Passenger control task Priority -> 2
    xTaskCreate(vJammingTask, "Jamming Control", 128, NULL, 2, & jammingHandler);//Jamming control task Priority -> 2

    vTaskStartScheduler(); //Start scheduler
  }

  for (;;);
}

/*-----------------------------------------------------------*/
void mainController(void * pvParameters) { //Continous function of lowest priorty
  uint32_t windowUP = 0x01;  // 1 indicates window up
  uint32_t windowDown = 0x00; // 0 indicates window down
  unsigned int portB_Pin3, portB_Pin2;

  for (;;) {
    //portB_Pin4 = GPIO_PORTB_DATA_R & 0x10;
    portB_Pin3 = GPIO_PORTB_DATA_R & 0x08;
    portB_Pin2 = GPIO_PORTB_DATA_R & 0x04;
    //portB_Pin1 = GPIO_PORTB_DATA_R & 0x02;

    if ((GPIO_PORTF_DATA_R & (1u << 0)) == 0) //Check if Tiva button 0 is pressed //DRIVER WINDOW UP
    {
			//if PF0 is pressed we will send on our queue "WindowUP == 1" 
      xQueueSendToBack(xQueue_MainController, & windowUP, 0);
      xSemaphoreGive(xBinarySemaphore_Driver); //Then increment the xBinarySemaphore_Driver (give semaphore to driver task)

    } else if ((GPIO_PORTF_DATA_R & (1u << 4)) == 0) //Check Tiva button 4 is pressed //DRIVER WINDOW DOWN
    {
			//if PF4 is pressed we will send on our queue "WindowUP == 0" 
      xQueueSendToBack(xQueue_MainController, & windowDown, 0);
      xSemaphoreGive(xBinarySemaphore_Driver);//Then increment the xBinarySemaphore_Driver (give semaphore to driver task)

    } else if (!portB_Pin3) // Check if PORT B, PIN 3 is pressed //PASSENGER WINDOW UP
    {
			//if PB3 is pressed we will send on our queue "WindowUP == 1" 
      xQueueSendToBack(xQueue_MainController, & windowUP, 0);
      xSemaphoreGive(xBinarySemaphore_Passenger); //Then increment the xBinarySemaphore_Passenger (give semaphore to passenger task)

    } else if (!portB_Pin2) // Check if PORT B, PIN 2 is pressed //PASSENGER WINDOW DOWN
    {
			//if PB2 is pressed we will send on our queue "WindowUP == 0" 
      xQueueSendToBack(xQueue_MainController, & windowDown, 0);
      xSemaphoreGive(xBinarySemaphore_Passenger); //Then increment the xBinarySemaphore_Passenger (give semaphore to passenger task)
    }

  }
}

/*-----------------------------------------------------------*/
void driverControl(void * pvParameter) // This function for driver controlling passenger's window
{
  xSemaphoreTake(xBinarySemaphore_Driver, 0); //Just initialzing semaphore
  
	uint32_t windowsDirection;
  for (;;) {
  
		xSemaphoreTake(xBinarySemaphore_Driver, portMAX_DELAY); //This sempahore will be blocked untill it recieves the semaphore
		// or xBinarySemaphore_Driver == 1, which is given from the main control function when PF0 or PF4 is pressed

    xQueueReceive(xQueue_MainController, & windowsDirection, portMAX_DELAY);
		//We will recieve the queue here and check what value is sent from the main control function

    if (windowsDirection == 1) { //if the value recieved from queue is 1 (indicates that window is going up) and PF0 is pressed

      DelayPORTB(100); //Here is used to check for automatic/manual, it will delay for 100ms

      if ((GPIO_PORTF_DATA_R & (1u << 0)) == 0) { //if after 100ms the button is still pressed -> then it's manual

        vToggle_Blue(); //Blue Led to indicate motor up

        motorUP(); //rotate the motor clockwise
				
				/*
				This while will work as long as PF0 is pressed. If PB7 button is pressed (indicates jamming)
				or if PB6 button is pressed (upper limit switch)-> the while loop will break 
				*/
        while (((GPIO_PORTF_DATA_R & (1u << 0)) == 0) && ((GPIO_PORTB_DATA_R & (1u << 7)) != 0) && ((GPIO_PORTB_DATA_R & (1u << 6)) != 0));
        //PF0 is for motor UP, PB6 is for limit switch, PB7 is for jamming!
				
        if ((GPIO_PORTB_DATA_R & (1u << 7)) == 0) //if jamming occurs (PB7 is pressed) 
        {
          xSemaphoreGive(xJammingSemaphore); //it will give semaphore to the jamming

          vTaskPrioritySet(jammingHandler, 3);// and will increase the jamming task priorty in order to go to the jamming task
        }

      } else { // else if after 100ms the button is not pressed -> then it's automatic 

        int counter = 0;

        do {
          vToggle_Blue(); //Blue Led to indicate motor up

          motorUP();//rotate the motor clockwise
					
					/*
						This do while will work as the counter didn't reach 100,000.
						If PB7 button is pressed (indicates jamming)
						or if PB6 button is pressed (upper limit switch)-> the do while loop will break 
					*/
          if (counter > 100000 || (GPIO_PORTB_DATA_R & (1u << 7)) == 0 || (GPIO_PORTB_DATA_R & (1u << 6)) == 0) {
            break; // Break out of the loop if either GPIO_PORTB_DATA_R bit 7 or bit 6 is 0
          } else {
            counter += 1; //increments counter
          }
        } while (1);

        //PF0 is for motor UP, PB6 is for limit switch, PB7 is for jamming!
        if ((GPIO_PORTB_DATA_R & (1u << 7)) == 0) //if jamming occurs (PB7 is pressed) 
        {
          xSemaphoreGive(xJammingSemaphore);//it will give semaphore to the jamming

          vTaskPrioritySet(jammingHandler, 3);// and will increase the jamming task priorty in order to go to the jamming task
        }

      }
    } else if (windowsDirection == 0) {//else if the value recieved from queue is 0 (indicates that window is going down) and PF4 is pressed
			
			DelayPORTB(100); //Here is used to check for automatic/manual, it will delay for 100ms
      
			if ((GPIO_PORTF_DATA_R & (1u << 4)) == 0) { //if after 100ms the button is still pressed -> then it's manual
      
			motorDOWN(); //motor rotates anti-clockwise
      
			vToggle_Red(); //Red led to indicate motor down
      //PF4 is for motor down, PB5 is for limit switch
			/*
				This while will work as long as PF4 is pressed. If PB5 button is pressed (lower limit switch)-> the while loop will break 
			*/
      while (((GPIO_PORTF_DATA_R & (1u << 4)) == 0) && ((GPIO_PORTB_DATA_R & (1u << 5)) != 0));
			}
			else // else if after 100ms the button is not pressed -> then it's automatic 
			{
				int counter = 0;
      
				do {
          vToggle_Red(); //Red led to indicate motor down

          motorDOWN();//motor rotates anti-clockwise
					
					 /*
						This do while will work as long as the counter didn't reach 100,000.
						If PB5 button is pressed (lower limit switch)-> the do while loop will break 
					*/
          if (counter > 100000||(GPIO_PORTB_DATA_R & (1u << 5)) == 0) {
            break; // Break out of the loop if GPIO_PORTB_DATA_R bit 5 is 0
          } else {
            counter += 1; //increments counter
          }
        } while (1);
			}
    }
    vTurn_OFF(); //turn off leds at the end

    motorOFF(); //turn off motor at the end

    taskYIELD(); //Task yeild to return to main control or go to other tasks of same priority

  }

}
/*-----------------------------------------------------------*/
void passengerControl(void * pvParameter) // This function for passengers controlling passenger's window
{

  xSemaphoreTake(xBinarySemaphore_Passenger, 0); //Just initialzing semaphore
  
	uint32_t windowsDirection;
  
	for (;;) {
    
		xSemaphoreTake(xBinarySemaphore_Passenger, portMAX_DELAY); //This sempahore will be blocked untill it recieves the semaphore
		// or xBinarySemaphore_Passenger == 1, which is given from the main control function when PB3 or PB2 is pressed
    
		xQueueReceive(xQueue_MainController, & windowsDirection, portMAX_DELAY);
		//We will recieve the queue here and check what value is sent from the main control function

    if ((GPIO_PORTB_DATA_R & (1u << 4)) != 0) // if on/off switch on -> indicating passenger's control is deactivated
    {
			// will only reach this part if PB4 is not pressed 

      if (windowsDirection == 1) {//if the value recieved from queue is 1 (indicates that window is going up) and PB3 is pressed
				
			DelayPORTB(100); //Here is used to check for automatic/manual, it will delay for 100ms
				
				if ((GPIO_PORTB_DATA_R & (1u << 3)) == 0) { //if after 100ms the button is still pressed -> then it's manual
					
					motorUP();//rotate the motor clockwise
					
					vToggle_Blue(); //Blue Led to indicate motor up
					
					/*
					This while will work as long as PB3 is pressed. If PB7 button is pressed (indicates jamming)
					or if PB6 button is pressed (upper limit switch)-> the while loop will break 
					*/
					while (((GPIO_PORTB_DATA_R & (1u << 3)) == 0) && ((GPIO_PORTB_DATA_R & (1u << 7)) != 0) && ((GPIO_PORTB_DATA_R & (1u << 6)) != 0));
					// PB3 is for motor up, PB7 is for jamming, PB6 is for limit switch
					
					if ((GPIO_PORTB_DATA_R & (1u << 7)) == 0) //if jamming occurs (PB7 is pressed) 
					{
						xSemaphoreGive(xJammingSemaphore); //it will give semaphore to the jamming
						
						vTaskPrioritySet(jammingHandler, 3);// and will increase the jamming task priorty in order to go to the jamming task
					}
				}
				else // else if after 100ms the button is not pressed -> then it's automatic 
				{
						int counter = 0;
						do {
							vToggle_Blue();//Blue Led to indicate motor up
							
							motorUP();//rotate the motor clockwise
							
							/*
							This do while will work as the counter didn't reach 100,000.
							If PB7 button is pressed (indicates jamming)
							or if PB6 button is pressed (upper limit switch)-> the do while loop will break 
							*/
							if (counter > 100000 || (GPIO_PORTB_DATA_R & (1u << 7)) == 0 || (GPIO_PORTB_DATA_R & (1u << 6)) == 0) {
								break; // Break out of the loop if either GPIO_PORTB_DATA_R bit 7 or bit 6 is 0
							} else {
								counter += 1;
							}
						} while (1);

						//PF0 is for motor UP, PB6 is for limit switch, PB7 is for jamming!
						if ((GPIO_PORTB_DATA_R & (1u << 7)) == 0) //if jamming occurs (PB7 is pressed) 
						{
							xSemaphoreGive(xJammingSemaphore);//it will give semaphore to the jamming
							vTaskPrioritySet(jammingHandler, 3);// and will increase the jamming task priorty in order to go to the jamming task
						}
				}

		} 
				else if (windowsDirection == 0) //else if the value recieved from queue is 0 (indicates that window is going down) and PB2 is pressed
					{
						DelayPORTB(100); //Here is used to check for automatic/manual, it will delay for 100ms
      if ((GPIO_PORTB_DATA_R & (1u << 2)) == 0) { //if after 100ms the button is still pressed -> then it's manual
						
					motorDOWN(); //motor rotates anti-clockwise
					vToggle_Red();//Red led to indicate motor down
					/*
						This while will work as long as PB2 is pressed. If PB5 button is pressed (lower limit switch)-> the while loop will break 
					*/
					while (((GPIO_PORTB_DATA_R & (1u << 2)) == 0) && ((GPIO_PORTB_DATA_R & (1u << 5)) != 0));
        // PB2 is for motor down, PB5 is for limit switch
				}
			else{ // else if after 100ms the button is not pressed -> then it's automatic 
				int counter = 0;
        do {
          vToggle_Red();//Red led to indicate motor down
					
          motorDOWN();//motor rotates anti-clockwise
					 /*
						This do while will work as long as the counter didn't reach 100,000.
						If PB5 button is pressed (lower limit switch)-> the do while loop will break 
					*/
          if (counter > 100000||(GPIO_PORTB_DATA_R & (1u << 5)) == 0) {
            break; // Break out of the loop if GPIO_PORTB_DATA_R bit 5 is 0
          } else {
            counter += 1; //increments counter
          }
        } while (1);}
			}
    }
		//Will reach here immediately if PB4 is pressed (indicating that passenger control is deactivated)
    motorOFF(); //turn off motor at the end
    vTurn_OFF(); //turn off leds at the end
    taskYIELD(); //Task yeild to return to main control or go to other tasks of same priority

  }

}

/*-----------------------------------------------------------*/

static void vJammingTask(void * pvParameters) {
  xSemaphoreTake(xJammingSemaphore, 0); //just intialzing semaphore
  for (;;) { 
    xSemaphoreTake(xJammingSemaphore, portMAX_DELAY); //Task blocked untill sempahore is recieved
		
		//If semaphore recieved
		
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
    vTaskPrioritySet(NULL, 2);

    /** TASKYIELD **/
    taskYIELD();

  }
}

/************** INIT FUNCTIONS IMPLEMENTATIONS ****************/
//This function is used to initialize UART and buttons
void InitTask(void * pvParameters) {
  DIO_Init(PORT_F, 0, IN);
  DIO_Init(PORT_F, 1, OUT);
  DIO_Init(PORT_F, 2, OUT);
  DIO_Init(PORT_F, 3, OUT);
  DIO_Init(PORT_F, 4, IN);
  motorInit();
}

void DelayPORTB(unsigned int delay) {
  volatile unsigned int i, counter;
  counter = delay * 4000; // 1 second (1000 msec) needs 40000000 counter so 4000000/1000 = 4000
  for (i = 0; i < counter; i++);
}

void PORTB_Init(void) {
  SYSCTL_RCGCGPIO_R |= GPIO_PORTB_CLK_EN; //activate clock for Port B
  DelayPORTB(10); //Delay 10 msec to allow clock to start on PORTB  
  GPIO_PORTB_LOCK_R = 0x4C4F434B; //unlock GPIO of PORTB
  GPIO_PORTB_CR_R = 0x01; //Enable GPIOPUR register enable to commit
  GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN3_EN; //Enable Pull Up SW on PB3	
  GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN2_EN; //Enable Pull Up SW on PB2	
  GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN3_EN; // Enable pin 3 of PORTB 
  GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN2_EN; // Enable pin 2 of PORTB 

  GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN6_EN; //Enable Pull Up SW on PB6	
  GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN7_EN; //Enable Pull Up SW on PB7	
  GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN6_EN; // Enable pin 6 of PORTB 
  GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN7_EN; // Enable pin 7 of PORTB 

  GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN6_EN; // Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN6_EN; // Disable analog function on pin 6 of PORTB
  GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN6_EN; // Regular port function
  GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN7_EN; // Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN7_EN; // Disable analog function on pin 7 of PORTB
  GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN7_EN; // Regular port function

  GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN3_EN; // Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN3_EN; // Disable analog function on pin 3 of PORTB
  GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN3_EN; // Regular port function
  GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN2_EN; // Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN2_EN; // Disable analog function on pin 2 of PORTB
  GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN2_EN; // Regular port function

  GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN5_EN; //Enable Pull Up SW on PB5	
  GPIO_PORTB_PUR_R |= GPIO_PORTB_PIN4_EN; //Enable Pull Up SW on PB4	
  GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN5_EN; // Enable pin 5 of PORTB 
  GPIO_PORTB_DEN_R |= GPIO_PORTB_PIN4_EN; // Enable pin 4 of PORTB 

  GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN5_EN; // Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN5_EN; // Disable analog function on pin 5 of PORTB
  GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN5_EN; // Regular port function
  GPIO_PORTB_PCTL_R &= ~GPIO_PORTB_PIN4_EN; // Regular GPIO of PORTB
  GPIO_PORTB_AMSEL_R &= ~GPIO_PORTB_PIN4_EN; // Disable analog function on pin 4 of PORTB
  GPIO_PORTB_AFSEL_R &= ~GPIO_PORTB_PIN4_EN; // Regular port function
}

