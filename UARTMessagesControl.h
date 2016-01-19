/*
 * UART_messages_control.h
 *
 *  Created on: 11 lis 2015
 *      Author:  Paulina Sadowska
 */

#ifndef UART3_MESSAGES_CONTROL_H_
#define UART3_MESSAGES_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

/** macros of data recieved from uart **/

//special characters
#define START_BYTE '{'
#define STOP_BYTE '}'
#define ROBOT_STOP_SIGN 'S'
#define ROBOT_MOVE_SIGN 'M'

#define CONNECT_SIGN 'C'
#define DISCONNECTED_SIGN 'D'
#define ACKNOWLEDGE_SIGN 'A'
#define ERROR_SIGN 'E'

//indexes on in message for informations
//connection in message
#define INDEX_CONNECTION_INFO 1
//control in message
#define INDEX_ROBOT_STOP_SIGN 1
#define INDEX_ANGLE 2
#define INDEX_VELOCITY  5
//out message
#define INDEX_START_BYTE 0
#define INDEX_CURRENT_1  1
#define INDEX_CURRENT_2  6
#define INDEX_STOP_BYTE  11

#define UART_BLUETOOTH_NR 3
#define UART_RASPBERRY_NR 4

 //length of the message
#define MESSAGE_LENGTH_IN 9 //maximum input message length (contains velocity and angle)
#define MESSAGE_CONNECTION_LENGTH_IN 3
#define MESSAGE_LENGTH_OUT 12 //output message length
#define MESSAGE_CONNECTION_LENGTH_OUT 3

//MAX time to live. determines how quickly state turns to disconnected after when recieving messages
#define TIME_TO_LIVE_MAX 5

//possible connection states
#define NOT_CONNECTED 0
#define CONNECTED_BLUETOOTH 30
#define CONNECTED_RASPBERRY 40

extern uint16_t connectionState; //determines state of connection
extern char timeToLive;

//BLUETOOTH BUFFERS
extern unsigned char inBuffer3[MESSAGE_LENGTH_IN]; //buffor to store readed message from bluetooth

//RASPBERRY BUFFERS
extern unsigned char inBuffer4[MESSAGE_LENGTH_IN]; //buffor to store readed message from raspberry

//SHARED BUFFERS
extern unsigned char outBuffer[MESSAGE_LENGTH_OUT]; //buffor to store message to send
extern unsigned char outBufferConnection[MESSAGE_CONNECTION_LENGTH_OUT]; //buffor to store message to send

extern unsigned char* inBuffer; //handler to currently used inBuffor

extern unsigned int i; //variable to manage char position in inBuffer array
extern bool MessageInProgress;

extern bool moveRobotFlag;
extern uint16_t drivingAngle;
extern uint32_t velocity;
extern bool UARTDataChanged;

//EventHandler
void(*UartDataChangedEventHandler)(void);

/*
 * gets character and create incoming message
 * checks start and stop bytes
 * then writes message into buffer
 * using function void WriteChar(unsigned char character, unsigned char UARTNr);
 * params:
 * unsigned char character - character to write
 * unsigned char UARTNr - number od UART device
 */

void WriteCharToBuffer(unsigned char character, unsigned char UARTNr);

/*
 * writes character to buffer given by UARTNr.
 * params:
 * unsigned char character - character to write
 * unsigned char UARTNr - number od UART device
 */
void WriteChar(unsigned char character, unsigned char UARTNr);

/*
 * Analyze message in device buffer
 * and decodes it
 * writing received value to variables:
 * moveRobotFlag, drivingAngle and velocity
 * params:
 * unsigned char UARTNr - number od UART device
 */
void DecodeMessage(unsigned char UARTNr);

/*
 * Gets current values, translate it to message to send
 * and write it in buffer given by device number (UARTNr)
 * params:
 * int current1 - readed current value nr 1
 * int current2 - readed current value nr 2
 * unsigned char UARTNr - number od UART device
 */
void CodeMessage(int current1, int current2, unsigned char UARTNr);
//void CodeAcknowlegeMessage(bool error, unsigned char UARTNr);

/*
 * Sends message from buffer via UART
 * to specific device
 * params:
 * unsigned char UARTNr - number od UART device
 * bool connectionMessage - if message is connection status message
 */
void SendMessage(unsigned char UARTNr, bool connectionMessage);

/*
 * event called whan Uart data changed
 */
void OnUartDataChangedEvent();
void UARTDataChangedSubscribe(void(*uartDataChangedEventHandler)(void));

/*
 * chooses right out buffer basing on UARTNr
 * params:
 * unsigned char UARTNr - number od UART device
 */
void ChooseInBuffer(unsigned char UARTNr);

#endif /* UART3_MESSAGES_CONTROL_H_ */
