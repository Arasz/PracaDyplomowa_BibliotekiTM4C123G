/*
 * adc_config.h
 *
 *  Created on: 8 lis 2015
 *      Author: palka
 */

#ifndef ADC_ADC_TIMER_INTERRUPT_DEMO_ADC_CONFIG_H_
#define ADC_ADC_TIMER_INTERRUPT_DEMO_ADC_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "inc/hw_adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h" //includes valid INT_ values for IntEnable function parameter
#include "driverlib/adc.h" //definitions for using the ADC driverS
#include "driverlib/timer.h" //macros for Timer API
#include <math.h>
#include "driverlib/fpu.h"

#define SUPPLY_VOLTAGE_x10nV 32700 //supply voltage [10-4 V]

#define ALPHA 0.21
#define BETA 0.05
#define dT 0.01

	//Sequencer 1 generates 4 samples
	/* array that will be used for storing the data read from the ADC FIFO.
	 * Array must be as large as the FIFO for the sequencer in use.
	 *
	 * Sequencer 0 : 8 samples
	 * Sequencer 1 : 4 samples
	 * Sequencer 2 : 4 samples
	 * Sequencer 3 : 1 samples
	 *
	 * This library uses sequencer 1
	 */
	uint32_t ui32ADC0Value[4]; //data buffer

	volatile uint32_t ADC0ValueAvg_CH4; //average value readed from PD3
	volatile uint32_t ADC0ValueAvg_CH5;	//average value readed from PD2

	volatile uint32_t VoltageHallMotorLeft; //voltage on channel 4 (PD3)
	volatile uint32_t VoltageHallMotorRight; //voltage on channel 5 (PD2)

	volatile int32_t CurrentMotorLeft; //current on channel 4 (PD3)
	volatile int32_t CurrentMotorRight; //current on channel 5 (PD2)

	//extern volatile int32_t CurrentBiasLeft;
	//extern volatile int32_t CurrentBiasRight;


	volatile float CurrentMotorLeftFiltered;
	volatile float CurrentMotorRightFiltered;

	//APLHA BETA FILTER VARIABLES
	extern volatile float CurrentPriLeft;
	extern volatile float CurrentPostLeft;
	extern volatile float ChangeCurrentPriLeft;
	extern volatile float ChangeCurrentPostLeft;

	extern volatile float CurrentPriRight;
	extern volatile float CurrentPostRight;
	extern volatile float ChangeCurrentPriRight;
	extern volatile float ChangeCurrentPostRight;


    /*
	* Calls ADC0_Init, Timer0_Init and Interrupts_Enable function
	* most general function to be called from the outside
    */
	void CSInit(void);

    /*
	* Initialize ADC0 sequencer 1
	* sequencer 1 gets 4 samples (averaged)
	* samples 0 and 1 from PE3, samples 2, 3 from PE2
	* it calculates average voltage and store it in ui32VoltageMotorRight/Left
	* it also calculate it to get ui32CurrentMotorRight/Left
	* (results are stored in this variables)
    */
	void CSInitADC0(void);


    /*
	* Initialize processor, TIM0 and ADC0 interrupts
	* ** NOTE! **
	* to get ADC0 interrupt to work ADC0IntHandler MUST be placed in interrupt vector
	* (tm4c123gh6pm_startup_ccs.c file) and declared in this file under "extern void _c_int00(void);"
    */
	void CSEnableInterrupts(void);

	//Filter measured current
	void CSAlphaBetaFilter();

#endif /* ADC_ADC_TIMER_INTERRUPT_DEMO_ADC_CONFIG_H_ */
