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
#include "inc/hw_memmap.h" //memory map
#include "inc/hw_ints.h" //includes valid INT_ values for IntEnable function parameter
#include <driverlib/sysctl.h> //SYSCTL macros
#include "driverlib/gpio.h"	//GPIO functiuons
#include "driverlib/interrupt.h"//interrupts functions
#include "driverlib/pin_map.h" 	//includes GPIO_PB0_T2CCP0 definition
#include "driverlib/timer.h" 	//macros for Timer API

#define TIMER_B_LOAD 60000 // To = 1.5ms dla freq 40Mhz; 1.2ms dla 50Mhz
#define TIMER_B_PRESCALER 0x7F // 127; 127 * 1.5 = 190.5ms; 127 * 1.2 = 152.4 ms;


extern uint32_t  DSDistancemm; ///Value used outside library. Stores measured distance in mm

extern uint16_t value[2]; 	 ///Helper values used to store timer value on both edges of echo signal
extern volatile bool Rising; ///Flag which determine if edge is rising or falling.
uint16_t ClockFrequencyMHz;


/*
 * Initialize Distance Sensor
 * Only function called from outside!!
 */
void DSInit();

/* Initialize timer 2A
 * used to trigger distance measurement
 * and timer 2B
 * used to measure time of echo
 * which determine distance
 */
void DSInitTimer2();

/* Initialize GPIO
 * PB3 - trigger
 */
void DSInitGPIO();

/*
 * gets timer value difference and calculates DSDistancemm
 */
void DSUpdateDistance(int32_t diff);


#endif /* DISTANCESENSOR_H_ */
