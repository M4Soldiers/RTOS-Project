#include "motor.h"


void motorInit()
{
	DIO_Init(PORT_A,4, OUT); // PORT C PIN 4
	DIO_Init(PORT_A,3, OUT); // PORT c PIN 3

}


void motorUP(){

DIO_WritePin(PORT_A, 4, 0);	
	DIO_WritePin(PORT_A, 3, 1);	
}
void motorDOWN(){
	
	DIO_WritePin(PORT_A, 4, 1);	
	DIO_WritePin(PORT_A, 3, 0);	
}

void motorOFF(){
	
	
	DIO_WritePort(PORT_A,0x00);
}