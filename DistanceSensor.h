/*
 * DistanceSensor.h
 *
 *  Created on: 17 lis 2015
 *      Author: palka
 */

#ifndef DISTANCESENSOR_H_
#define DISTANCESENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include <inc/hw_timer.h>
#include "inc/hw_ints.h" //includes valid INT_ values for IntEnable function parameter
#include <driverlib/sysctl.h>
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h" //macros for Timer API

extern unsigned int DSDistanceCm;
extern uint32_t value;
/* Initialize timer 2A
 * used to trigger distance measurement
 * and timer 2B
 * used to measure time of echo
 * which determine distance
 */
void DSInitTimer2(unsigned int frequency);

/* Initialize GPIO
 * PB3 - trigger
 * PB4 - echo
 */
void DSInitGPIO();
/*
 * Initialize Distance Sensor
 */
void DSInit(unsigned int frequency);


#endif /* DISTANCESENSOR_H_ */
