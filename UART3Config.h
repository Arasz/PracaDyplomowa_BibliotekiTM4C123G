/*
 * UART3_conf.h
 *
 *  Created on: 9 lis 2015
 *      Author: palka
 */

#ifndef UART3_CONF_H_
#define UART3_CONF_H_

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

/* Configure UART3 connection
 * used to communicate with Android via Bluetooth
 * used pins: PC6 (RX) and PC7 (TX)
 */
void UCInitUART3(void);


#endif /* UART3_CONF_H_ */
