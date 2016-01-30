/*
 * UART_messages_control.c
 *
 *  Created on: 11 lis 2015
 *      Author: Paulina Sadowska
 */

#include "UARTMessagesControl.h"

uint16_t connectionState = 0;
char timeToLive = TIME_TO_LIVE_MAX;

//helper variables used to write and decode message
//UART3
unsigned char inBuffer3[MESSAGE_LENGTH_IN]; //buffor to store readed message

//UART4
unsigned char inBuffer4[MESSAGE_LENGTH_IN]; //buffor to store readed message

unsigned char outBuffer[MESSAGE_LENGTH_OUT]; //buffor to store message to send
unsigned char* inBuffer; //temp buffer to store reference to the right inBuffer

unsigned int i = 0; //variable to menage char position in inBuffer array
bool MessageInProgress = false;

//data used on the outside
bool moveRobotFlag = false;
uint16_t drivingAngle = 0; //0-360
uint32_t velocity = 0;
bool UARTDataChanged = false;


//UART3 interrupt handler
void UART3IntHandler(void)
{
    uint32_t ui32Status;
    unsigned char temp;
    ui32Status = UARTIntStatus(UART3_BASE, true); 	//get interrupt status
    UARTIntClear(UART3_BASE, ui32Status); 			//clear the asserted interrupts
    while(UARTCharsAvail(UART3_BASE))				//loop while there are chars
    {
        temp = UARTCharGetNonBlocking(UART3_BASE); //gets character
        WriteCharToBuffer(temp, UART_BLUETOOTH_NR);  //check if character is part of frame and write it to buffer
    }
}

//UART4 interrupt handler
void UART4IntHandler(void)
{
    uint32_t ui32Status;
    unsigned char temp;
    ui32Status = UARTIntStatus(UART4_BASE, true); 	//get interrupt status
    UARTIntClear(UART4_BASE, ui32Status); 			//clear the asserted interrupts
    while(UARTCharsAvail(UART4_BASE))				//loop while there are chars
    {
        temp = UARTCharGetNonBlocking(UART4_BASE); //gets character
        WriteCharToBuffer(temp, UART_RASPBERRY_NR);  //check if character is part of frame and write it to buffer
    }
}

void SendMessage(unsigned char UARTNr)
{
	int j;
	if(UARTNr == UART_BLUETOOTH_NR)
	{
		for(j=0; j<MESSAGE_LENGTH_OUT; j++)
				UARTCharPutNonBlocking(UART3_BASE,outBuffer[j]);
	}
	else if(UARTNr == UART_RASPBERRY_NR)
	{
			for(j=0; j<MESSAGE_LENGTH_OUT; j++)
				UARTCharPutNonBlocking(UART4_BASE,outBuffer[j]);
	}

	if(timeToLive == 0)
	{
		connectionState = NOT_CONNECTED;
		moveRobotFlag = false; //NOT CONNECTED <- DONT MOVE
		OnUartDataChangedEvent(); //STOP ROBOT
	}
	else
		timeToLive -= 1;

}

void UARTDataChangedSubscribe(void(*uartDataChangedEventHandler)(void))
{
	UartDataChangedEventHandler = uartDataChangedEventHandler;
}

void OnUartDataChangedEvent()
{
	UartDataChangedEventHandler();
}

void DecodeMessage(unsigned char UARTNr)
{
	ChooseInBuffer(UARTNr);
	timeToLive = TIME_TO_LIVE_MAX;

	if(UARTNr == UART_BLUETOOTH_NR && connectionState != CONNECTED_RASPBERRY)
		connectionState = CONNECTED_BLUETOOTH;
	else if(UARTNr == UART_RASPBERRY_NR && connectionState != CONNECTED_BLUETOOTH)
		connectionState = CONNECTED_RASPBERRY;

	//control message
	//get angle and velocity from message
	if(inBuffer[INDEX_ROBOT_STOP_SIGN]==ROBOT_MOVE_SIGN)
		moveRobotFlag = true;
	else
		moveRobotFlag = false;

	drivingAngle = (inBuffer[INDEX_ANGLE]-48)*100 + (inBuffer[INDEX_ANGLE+1]-48)*10 + (inBuffer[INDEX_ANGLE+2]-48);
	velocity = (inBuffer[INDEX_VELOCITY]-48)*100 + (inBuffer[INDEX_VELOCITY+1]-48)*10 + (inBuffer[INDEX_VELOCITY+2]-48);
	OnUartDataChangedEvent();
}

void CodeMessage(int current1, int current2, unsigned char UARTNr)
{

	if(current1 < 0)
	{
		outBuffer[INDEX_CURRENT_1] = '-';
		current1 = (-1) * current1;
	}
	else
		outBuffer[INDEX_CURRENT_1] = '+';

	if(current2 < 0)
	{
		outBuffer[INDEX_CURRENT_2] = '-';
		current2 = (-1) * current2;
	}
	else
		outBuffer[INDEX_CURRENT_2] = '+';

	outBuffer[INDEX_START_BYTE] = START_BYTE;
	outBuffer[INDEX_STOP_BYTE] = STOP_BYTE;

	outBuffer[INDEX_CURRENT_1+1] = current1/1000+48;
	current1 = current1%1000;
	outBuffer[INDEX_CURRENT_1+2] =  current1/100+48;
	current1 = current1%100;
	outBuffer[INDEX_CURRENT_1+3] = current1/10+48;
	current1 = current1%10;
	outBuffer[INDEX_CURRENT_1+4] = current1+48;

	outBuffer[INDEX_CURRENT_2+1] = current2/1000+48;
	current2 = current2%1000;
	outBuffer[INDEX_CURRENT_2+2] =  current2/100+48;
	current2 = current2%100;
	outBuffer[INDEX_CURRENT_2+3] = current2/10+48;
	current2 = current2%10;
	outBuffer[INDEX_CURRENT_2+4] = current2+48;
}

void WriteCharToBuffer(unsigned char character, unsigned char UARTNr)
{
    if(character == START_BYTE)
    {
    	i=0;
    	MessageInProgress = true;
    	WriteChar(character, UARTNr);
    }
    else if(MessageInProgress && character == STOP_BYTE)
    {
    	i++;
    	MessageInProgress = false;
    	WriteChar(character, UARTNr);
    	DecodeMessage(UARTNr);
    }
    else if(MessageInProgress)
    {
    	i++;
    	WriteChar(character, UARTNr);
    }
}

void WriteChar(unsigned char character, unsigned char UARTNr)
{
	if(UARTNr == UART_BLUETOOTH_NR)
	   inBuffer3[i] = character;
	else if(UARTNr == UART_RASPBERRY_NR)
	   inBuffer4[i] = character;
}

void ChooseInBuffer(unsigned char UARTNr)
{
	//choose right out buffer basing on uart nr
	if(UARTNr == UART_BLUETOOTH_NR)
		inBuffer = inBuffer3;
	else if(UARTNr == UART_RASPBERRY_NR)
		inBuffer = inBuffer4;
}

