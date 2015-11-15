/*
 * MotorControl.c
 *
 *  Created on: 15 lis 2015
 *      Author: Rafa³
 */

#include "MotorControl.h"

#define DUTY_CYCLE_DIVIDER 100

uint32_t LoadValue;

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
	MCInitGpio();
	MCInitPwm(DutyCycle);
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

