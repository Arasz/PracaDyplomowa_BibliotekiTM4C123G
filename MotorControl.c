/*
 * MotorControl.c
 *
 *  Created on: 15 lis 2015
 *      Author: Rafal
 */

#include "MotorControl.h"

#define DUTY_CYCLE_DIVIDER 100
#define MAX_MOTOR_VOLTAGE 7.4
#define MAX_MOTOR_CURRENT 1.6

uint32_t LoadValue; /// PWM counter load value

static uint32_t DutyCycleLeft=0; /// PWM duty cycle
static uint32_t DutyCycleRight=0; /// PWM duty cycle

static PIDControler currentControllerLeft, speedControlerLeft, currentControllerRight, speedControlerRight; /// Motor controlers

void MCInitPwm(uint32_t DutyCycle)
{
	// Enable peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Clock PWM peripheral at SysClk/PWMDIV
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	// Configure pin PD0 as PWM output
	ROM_GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7);
	ROM_GPIOPinConfigure(GPIO_PA6_M1PWM2);
	ROM_GPIOPinConfigure(GPIO_PA7_M1PWM3);

	// Calculate PWM clock
	uint32_t PWMClock = SysCtlClockGet() / 64;
	// Value from/to which PWM counter counts (subtract 1 becouse counter counts from 0). This is PWM period
	LoadValue = (PWMClock/PWM_FREQUENCY) - 1;
	// Configure PWM
	ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	ROM_PWMGenIntTrigEnable(PWM1_BASE, PWM_GEN_1, PWM_INT_CNT_ZERO | PWM_TR_CNT_LOAD);
	ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, LoadValue);
	// Set PWM signal width
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, (DutyCycle*LoadValue)/DUTY_CYCLE_DIVIDER);
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, (DutyCycle*LoadValue)/DUTY_CYCLE_DIVIDER);
	// Enable PWM output 0 and 1
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT|PWM_OUT_3_BIT, true);
	// Invert output - if true output is active low
	ROM_PWMOutputInvert(PWM1_BASE,PWM_OUT_2_BIT|PWM_OUT_3_BIT, false );
	// Set PWM Output update mode to local sync ( update when generator count reaches 0)
	ROM_PWMOutputUpdateMode(PWM1_BASE, PWM_OUT_2_BIT|PWM_OUT_3_BIT, PWM_OUTPUT_MODE_SYNC_LOCAL);
	// Enable PWM generator
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);

}

void MCInitGpio()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// PIN1, PIN2 - In1B,In2B (MotorB) ; PIN4,PIN5 - In1A,In2A
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
	ROM_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
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
	//left - motorA controlers initialization
	InitControler(&currentControllerLeft, 0, 0.18068, 0, samplingPeriod, MAX_MOTOR_CURRENT, 100);
	InitControler(&speedControlerLeft, 41.36, 108.43, 3.81, samplingPeriod, MAX_MOTOR_VOLTAGE, 100);
	//right - motorB controlers initialization
	InitControler(&currentControllerRight, 0, 0.18068, 0, samplingPeriod, MAX_MOTOR_CURRENT, 100);
	InitControler(&speedControlerRight, 41.36, 108.43, 3.81, samplingPeriod, MAX_MOTOR_VOLTAGE, 100);

	// init kalman filter (state observer)
	//InitKalmanFilter();
}

void MCPwmDutyCycleSet(Motor selectedMotor, uint32_t dutyCycle)
{

	if(dutyCycle>0)
		ROM_PWMPulseWidthSet(PWM1_BASE, selectedMotor, (dutyCycle*LoadValue)/DUTY_CYCLE_DIVIDER );
	else
		ROM_PWMPulseWidthSet(PWM1_BASE, selectedMotor, 1);

	switch(selectedMotor)
	{
	case MotorLeft:
		DutyCycleLeft = dutyCycle;
	case MotorRight:
		DutyCycleRight = dutyCycle;
	}
}

void MCChangeMotorState(Motor selectedMotor, MotorState newMotorState)
{

	switch(selectedMotor)
	{
		case MotorLeft: //left motor
			ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4|GPIO_PIN_5, newMotorState<<4);
			break;
		case MotorRight: //right motor
			ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2, newMotorState<<1);
			break;
		default:
			ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5,
					newMotorState<<4|newMotorState<<1);
			break;
	}
}

/**
 *
 */
void InitControler(PIDControler* controler, float kp, float ki, float kd,
		float ts, float outputLimit, float antiWindupLimit)
{
	controler->Kp = kp;
	controler->Ki = ki;
	controler->Kd = kd;
	controler->Ts = ts;
	controler->OutputLimit = outputLimit;
	controler->AntiWindupLimit = antiWindupLimit;
	controler->LastError = 0;
	controler->Sum = 0;
}

/**
 * @brief Calculates control dignal
 * @param controler PID controler which wil be used in calculations
 * @param setpoint desirable object output
 * @param measurement measured object output
 */
float CalculateControlSignal(PIDControler* controler, float setpoint,
		float measurement)
{
	// Calculate control error
	float error = setpoint - measurement;
	// Update errors sum for future integral calculations
	controler->Sum += error;
	// Calculate proportional term
	float proportional = error*controler->Kp;
	// Calculate integral term
	float integral = (controler->Sum*controler->Ts)*controler->Ki;

	// Windup compensation
	if(integral > controler->AntiWindupLimit)
		integral = controler->AntiWindupLimit;
	else if(integral < -controler->AntiWindupLimit)
		integral = - controler->AntiWindupLimit;

	float controlSignal = 0;

	if(controler->Kd != 0)
	{
		// Derivative term is less frequently calculated

		float deltaError = error - controler->LastError;
		float derivative = (deltaError/controler->Ts)*controler->Kd;
		controler->LastError = error;
		controlSignal = derivative;
	}

	controlSignal += proportional + integral;

	// Output limit
	if(controlSignal > controler->OutputLimit)
		controlSignal = controler->OutputLimit;
	else if(controlSignal < -controler->OutputLimit)
		controlSignal = -controler->OutputLimit;

	return controlSignal;
}


static Matrix estimatedStateLeft;
static Matrix estimatedStateRight;
void MCControlCalculations(double measuredCurrentLeft, double measuredCurrentRight,  double measuredInputVoltage,
		double rotationSpeedLeft, double rotationSpeedRight)
{
	// Left motor
	// Mean PWM voltage
	double meanPwmVoltageLeft = (DutyCycleLeft*measuredInputVoltage)/DUTY_CYCLE_DIVIDER;
	// At first estimate rotation speed
	CalculateEstimatedState(meanPwmVoltageLeft, measuredCurrentLeft, &estimatedStateLeft);
	double estimatedRotationSpeedLeft = GetElement(&estimatedStateLeft, 2, 1);
	// Calculate current with speed controller for current controller
	double calculatedCurrentLeft = CalculateControlSignal(&speedControlerLeft, rotationSpeedLeft, estimatedRotationSpeedLeft);
	// Calculate motor voltage with current controller
	double calculatedMotorVoltageLeft = CalculateControlSignal(&currentControllerLeft, calculatedCurrentLeft, measuredCurrentLeft);
	// Calculate duty cycle
	double pwmDutyCycleLeft = (calculatedMotorVoltageLeft/measuredInputVoltage)*100;

	//Right motor
	// Mean PWM voltage
	double meanPwmVoltageRight = (DutyCycleRight*measuredInputVoltage)/DUTY_CYCLE_DIVIDER;
	// At first estimate rotation speed
	CalculateEstimatedState(meanPwmVoltageRight, measuredCurrentRight, &estimatedStateRight);
	double estimatedRotationSpeedRight = GetElement(&estimatedStateRight, 2, 1);
	// Calculate current with speed controller for current controller
	double calculatedCurrentRight = CalculateControlSignal(&speedControlerRight, rotationSpeedRight, estimatedRotationSpeedRight);
	// Calculate motor voltage with current controller
	double calculatedMotorVoltageRight = CalculateControlSignal(&currentControllerLeft, calculatedCurrentRight, measuredCurrentRight);
	// Calculate duty cycle
	double pwmDutyCycleRight = (calculatedMotorVoltageRight/measuredInputVoltage)*100;


	if(pwmDutyCycleLeft < 0)
	{
		MCChangeMotorState(MotorLeft, ROT_CCW);
		MCPwmDutyCycleSet(MotorLeft, -pwmDutyCycleLeft);
	}
	else
	{
		MCChangeMotorState(MotorLeft, ROT_CW);
		MCPwmDutyCycleSet(MotorLeft, pwmDutyCycleLeft);
	}

	if(pwmDutyCycleRight < 0)
	{
		MCChangeMotorState(MotorRight, ROT_CCW);
		MCPwmDutyCycleSet(MotorRight, -pwmDutyCycleRight);
	}
	else
	{
		MCChangeMotorState(MotorRight, ROT_CW);
		MCPwmDutyCycleSet(MotorRight, pwmDutyCycleRight);
	}


}

