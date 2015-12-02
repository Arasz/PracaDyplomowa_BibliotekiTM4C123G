/*
 * UART_conf.h
 *
 *  Created on: 9 lis 2015
 *      Author: Paulina Sadowska
 */

#ifndef UART_CONF_H_
#define UART_CONF_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"

//predefined baud rate
#define BAUD_RATE 9600

/* Calls functions which configure UART connections and interrupts
 */
void UCInitUART(void);

/* Configure UART3 connection
 * used to communicate with Android via Bluetooth
 * used pins: PC6 (RX) and PC7 (TX)
 */
void InitUART3(void);

/* Configure UART4 connection
 * used to communicate with Raspberry via UART
 * used pins: PC4 (RX) and PC5 (TX)
 */
void InitUART4(void);

/* Configure UART3 and UART 4 interrupts
 */
void InitUARTInterrupts(void);


#endif /* UART_CONF_H_ */
