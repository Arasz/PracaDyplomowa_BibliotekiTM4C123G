/*
 * MotorControl.h
 *
 *  Created on: 15 lis 2015
 *      Author: Rafa³
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"


/* Defined values */

#define PWM_FREQUENCY 20

/* Defined types */

typedef enum MotorType
{
	MotorA = PWM_OUT_0,
	MotorB = PWM_OUT_1,
} Motor;

typedef enum MotorStates
{
	SOFT_BREAK = 0x0,
	ROT_CW = 0x1,
	ROT_CCW = 0x2,
	HARD_BREAK = 0x3,
} MotorState;

/* Global values */

// Value from/to which PWM counter counts
extern uint32_t LoadValue;

/* Functions */
void MCInitGpio();
void MCInitPwm(uint32_t DutyCycle);
void MCInitControlHardware(uint32_t DutyCycle);

void MCPwmDutyCycleSet(Motor selectedMotor, uint32_t DutyCycle);
void MCChangeMotorState(Motor selectedMotor, MotorState newMotorState);

#endif /* MOTORCONTROL_H_ */
