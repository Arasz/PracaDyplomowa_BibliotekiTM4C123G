/*
 * SendingData.h
 *
 *  Created on: 12 lis 2015
 *      Author: palka
 */

#ifndef SENDINGDATA_H_
#define SENDINGDATA_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h" //includes valid INT_ values for IntEnable function parameter
#include "driverlib/timer.h" //macros for Timer API

#include "UARTMessagesControl.h"
#include "CurrentSensing.h"

/* Initialize Timer 1 used to
 * periodically send data via UART
 * with given frequency
 * param frequency - frequency of sending data to device
 */
void TSDInitTimer1(unsigned int frequency);

#endif /* SENDINGDATA_H_ */
