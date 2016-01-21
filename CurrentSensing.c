/*
 * adc_config.c
 *
 *  Created on: 8 lis 2015
 *      Author: Paulina Sadowska
 */

#include "CurrentSensing.h"

volatile bool BatteryStateOK = true;

volatile int32_t CurrentBiasLeft = 0;
volatile int32_t CurrentBiasRight = 0;


//APLHA BETA FILTER VARIABLES
volatile float CurrentPriLeft = 0.0;
volatile float CurrentPostLeft = 0.0;
volatile float ChangeCurrentPriLeft = 0.0;
volatile float ChangeCurrentPostLeft = 0.0;

volatile float CurrentPriRight = 0.0;
volatile float CurrentPostRight = 0.0;
volatile float ChangeCurrentPriRight = 0.0;
volatile float ChangeCurrentPostRight = 0.0;


void CSInit(void)
{
	CSInitADC0();
	CSInitTimer0();
	CSInitRedLed();
	CSEnableInterrupts();
}

void CSEnableInterrupts(void)
{
	IntMasterEnable();
	//ADC INTERRUPTS
	ADCIntClear(ADC0_BASE, 1);
	// Enable processor interrupts.
	ADCIntEnable(ADC0_BASE, 1);		//enable ADC0 interrupts
	IntEnable(INT_ADC0SS1);			//enable interrupt when ADC0 conversion is done

}

void CSInitADC0(void)
{
	//enable ADC0 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	//enable Hardware averaging
	//64 measurements being averaged together.
	//We will then average four of those samples together in our code for a total of 256.
	ADCHardwareOversampleConfigure(ADC0_BASE, 64);
	//configure the ADC sequencer.
	ADCSequenceConfigure(ADC0_BASE, 			//use ADC0
		 	 	 	 	 1, 					// sequencer 1
					     ADC_TRIGGER_TIMER,   	//pwm triggers sequence
					     0); 					//highest priority
	 //configure all four steps in the ADC sequencer. CH4 (PD3), CH5 (PD2) and CH6 (PD1)
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH5);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH6 | 		//Chanel 6 : PD1
							ADC_CTL_IE| 	//configure the interrupt flag (ADC_CTL_IE) to be set when the sample is done
							ADC_CTL_END);	//last conversion on sequencer

	//enable ADC0 sequencer
	ADCSequenceEnable(ADC0_BASE, 1);
}

/*
 * interrupt routine called when conversion is done
 * calculates average value obtained by sensor,
 * and motor current.
 * ** NOTE! **
 * to get ADC0 interrupt to work ADC0IntHandler MUST be placed in interrupt vector
 * (tm4c123gh6pm_startup_ccs.c file) and declared in this file under "extern void _c_int00(void);"
 */
void ADC0IntHandler(void) {

    // Clear the interrupt status flag.
    ADCIntClear(ADC0_BASE, 1);

	 //read the ADC value from the ADC Sample Sequencer 1 FIFO
	 ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
	 //calculate average voltage
	 ADC0ValueAvg_CH4 = ui32ADC0Value[0];
	 ADC0ValueAvg_CH5 = ui32ADC0Value[1];
	 ADC0ValueAvg_CH6 = ui32ADC0Value[2];

	 // mV per ADC code = (VREFP - VREFN) * value / 4096
	 VoltageHallMotorRight = 33000 * ADC0ValueAvg_CH5 / 4096; //[10-4V]
	 VoltageHallMotorLeft = 33000 * ADC0ValueAvg_CH4 / 4096; //[10-4V]
	 BatteryVoltageSensor = 247 * 330 * ADC0ValueAvg_CH6 / 4096; //[10-4V]


	 CurrentMotorRight = VoltageHallMotorRight * 36700 / SUPPLY_VOLTAGE_x10nV;
	 CurrentMotorRight = CurrentMotorRight - 18350;

	 CurrentMotorLeft = VoltageHallMotorLeft * 36700 / SUPPLY_VOLTAGE_x10nV;
	 CurrentMotorLeft = CurrentMotorLeft - 18350;

	if(!moveRobotFlag)
	{
		//calibrate measurement in 0mA
		CurrentBiasLeft = CurrentMotorLeft;
		CurrentBiasRight = CurrentMotorRight;
	}
	//check bettery voltage
	if(BatteryVoltageSensor<BATTERY_VOLTAGE_SENSOR_MIN)
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0xFF); //turn red led on
	}
	else
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00); //turn red led off
	}
	CurrentMotorRight = CurrentMotorRight - CurrentBiasRight;
	CurrentMotorLeft = CurrentMotorLeft - CurrentBiasLeft;

	 CSAlphaBetaFilter();
}

void CSInitRedLed(void)//init red build on led
{
	//init PF1 (red build on led
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00); //turn red led off
}

void CSInitTimer0(void)
{
	uint32_t ui32Period; //desired clock period

	// before calling any peripheral specific driverLib function we must enable the clock to that peripheral!!!
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	/*
	 * Timer 0 as a 32-bit timer in periodic mode. When Timer 0 is configured as a 32-bit timer,
	 * it combines the two 16-bit timers Timer 0A and Timer 0B.
	 * TIMER0_BASE is the start of the timer registers for Timer0 in the peripheral section of the memory map.
	 */
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	/*
	 * desired frequency = 1/dT
	 */
	ui32Period = (SysCtlClockGet() * dT) ;

	/*
	 * load calculated period into the Timers Interval Load register using the TimerLoadSet
	 * you have to subtract one from the timer period since the interrupt fires at the zero count
	 */
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

	// Enable triggering
	TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
	TimerEnable(TIMER0_BASE, TIMER_A); //enable timerA
}

void CSAlphaBetaFilter()
{
	//left current sensor
	CurrentPriLeft = CurrentPostLeft + dT * ChangeCurrentPostLeft;
	ChangeCurrentPriLeft = ChangeCurrentPostLeft;
	CurrentPostLeft = CurrentPriLeft + ALPHA * (CurrentMotorLeft - CurrentPriLeft);
	ChangeCurrentPostLeft = ChangeCurrentPriLeft + (BETA*(CurrentMotorLeft - CurrentPriLeft))/dT;

	CurrentMotorLeftFiltered = CurrentPostLeft;

	//right current sensor
	CurrentPriRight = CurrentPostRight + dT * ChangeCurrentPostRight;
	ChangeCurrentPriRight = ChangeCurrentPostRight;
	CurrentPostRight = CurrentPriRight + ALPHA * (CurrentMotorRight - CurrentPriRight);
	ChangeCurrentPostRight = ChangeCurrentPriRight + (BETA*(CurrentMotorRight - CurrentPriRight))/dT;

	CurrentMotorRightFiltered = CurrentPostRight;
}

