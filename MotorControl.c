/*
 * MotorControl.c
 *
 *  Created on: 15 lis 2015
 *      Author: Rafa³
 */

#include "MotorControl.h"

#define DUTY_CYCLE_DIVIDER 100

uint32_t LoadValue; //< PWM counter load value

PIDControler LeftMotorControler, RigthMotorControler;

void MCInitPwm(uint32_t DutyCycle)
{
	// Clock PWM peripheral at SysClk/PWMDIV
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	// Enable peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure pin PD0 as PWM output
	ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
	ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0);
	ROM_GPIOPinConfigure(GPIO_PD1_M1PWM1);

	// Calculate PWM clock
	uint32_t PWMClock = SysCtlClockGet() / 64;
	// Value from/to which PWM counter counts (subtract 1 becouse counter counts from 0). This is PWM period
	LoadValue = (PWMClock/PWM_FREQUENCY) - 1;
	// Configure PWM
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, LoadValue);
	// Set PWM signal width
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, (DutyCycle*LoadValue)/DUTY_CYCLE_DIVIDER);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (DutyCycle*LoadValue)/DUTY_CYCLE_DIVIDER);
	// Enable PWM output 0 and 1
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT|PWM_OUT_1_BIT, true);
	// Enable PWM generator
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

void MCInitGpio()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// PIN1, PIN2 - In1B,In2B (MotorB) ; PIN4,PIN5 - In1A,In2A
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
}

void MCInitControlHardware(uint32_t DutyCycle)
{
	// Enable FPU for fast calculations
	ROM_FPULazyStackingEnable();
	ROM_FPUEnable();

	MCInitGpio();
	MCInitPwm(DutyCycle);
}

void MCInitControlSoftware(float samplingPeriod)
{
	InitControler(&LeftMotorControler, 1, 1, 1, 1, samplingPeriod, 1000, 1000);
	InitControler(&RigthMotorControler, 1, 1, 1, 1,samplingPeriod, 1000, 1000);
}

void MCPwmDutyCycleSet(Motor selectedMotor, uint32_t DutyCycle)
{
	ROM_PWMPulseWidthSet(PWM1_BASE, selectedMotor, (DutyCycle*LoadValue)/100 );
}

void MCChangeMotorState(Motor selectedMotor, MotorState newMotorState)
{

	switch(selectedMotor)
	{
		case MotorA:
			ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4|GPIO_PIN_5, newMotorState<<4);
			break;
		case MotorB:
			ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2, newMotorState<<1);
			break;
		default:
			ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5,
					newMotorState<<4|newMotorState<<1);
			break;
	}
}

void InitControler(PIDControler* controler, float Kp, float Ki, float Kd, float b, float dt, float posOutputLimit, float negOutputLimit)
{
	controler->Kp = Kp;
	controler->Ki = Ki;
	controler->Kd = Kd;
	controler->b = b;
	controler->dt = dt;
	controler->posOutputLimit = posOutputLimit;
	controler->negOutputLimit = negOutputLimit;
	controler->lastError = 0;
	controler->sum = 0;
}

float CalculateMotorControl(PIDControler controler, float setpoint, float measurement)
{
	float error = setpoint - measurement; // Control error

	controler.sum += error; // Update errors sum for future integral calculations

	float proportional = (controler.b*setpoint - measurement)*controler.Kp;
	float integral = (controler.sum*controler.dt)*controler.Ki;
	float controlSignal = 0;

	if(controler.Kd != 0) // Derivative term is less frequently calculated.
	{
		float deltaError = error - controler.lastError;
		float derivative = (deltaError/controler.dt)*controler.Kd;
		controler.lastError = error;
		controlSignal = derivative;
	}

	controlSignal += proportional + integral;

	// Anti wind-up
	if(controlSignal > controler.posOutputLimit)
		controlSignal = controler.posOutputLimit;
	else if(controlSignal < controler.negOutputLimit)
		controlSignal = controler.negOutputLimit;

	return controlSignal;
}

