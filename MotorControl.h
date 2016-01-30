/*
 * MotorControl.h
 *
 *  Created on: 15 lis 2015
 *      Author: Rafa³
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "driverlib/rom.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"


#include "MatrixCalculation.h"
#include "StateEstimation.h"


/* Defined values */

#define PWM_FREQUENCY 22000

/** Defined types */

typedef enum MotorType
{
	MotorLeft = PWM_OUT_2,
	MotorRight = PWM_OUT_3,
} Motor;

typedef enum MotorStateType
{
	ROT_CW = 0x1,
	ROT_CCW = 0x2,
	HARD_BREAK = 0x3, // 0x00
} MotorState;

typedef struct Contorler
{
	float Kp;  // proportional gain
	float Ki; // integral gain
	float Kd; // derivative gain
	float Sum; // sum of control errors
	float LastError; // last control error
	float Ts; // sampling time
	float OutputLimit; // control signal limit
	float AntiWindupLimit; // atni-windup calculus limit
}PIDControler;

/* Global values */

// Value from/to which PWM counter counts
extern uint32_t LoadValue;

/* Functions */
void MCInitGpio();
void MCInitPwm(uint32_t DutyCycle);
/// \brief Inits PWM and GPIO peripherals
void MCInitControlHardware(uint32_t DutyCycle);


void MCPwmDutyCycleSet(Motor selectedMotor, uint32_t dutyCycle);
void MCChangeMotorState(Motor selectedMotor, MotorState newMotorState);

float CalculateControlSignal(PIDControler* controler, float setpoint,
		float measure);
void InitControler(PIDControler* controler, float kp, float ki, float kd,
		float ts, float outputLimit, float antiWindupLimit);
void MCInitControlSoftware(float samplingPeriod);

void MCControlCalculations(double measuredCurrentLeft, double measuredCurrentRight,  double measuredInputVoltage,
		double rotationSpeedLeft, double rotationSpeedRight);

#endif /* MOTORCONTROL_H_ */
