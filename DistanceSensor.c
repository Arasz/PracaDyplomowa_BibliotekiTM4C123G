/*
 * DistanceSensor.c
 *
 *  Created on: 17 lis 2015
 *      Author: Paulina Sadowska
 */
#include "DistanceSensor.h"

uint32_t  DSDistancemm = 0;

uint16_t value[2] = {0, 0};
volatile bool Rising = false;

//Timer2A gets edge interrupt from PB0
void Timer2AIntHandler(void)
{

	  volatile uint32_t interruptStatus = TimerIntStatus(TIMER2_BASE, false);
	  TimerIntClear(TIMER2_BASE, interruptStatus);

	  if( interruptStatus & TIMER_CAPA_EVENT )
	  {
		  if(Rising)
		  {
			  value[1] = TimerValueGet(TIMER2_BASE, TIMER_B);
			  Rising = false;;
		  }
		  else
		  {
			  	int32_t timeDifference;
			    Rising = true;
				value[0] = TimerValueGet(TIMER2_BASE, TIMER_B);
				timeDifference = value[1] - value[0];
				if(timeDifference<0){
					timeDifference += 60000;
				}
				DSUpdateDistance(timeDifference);
		  	}
	  }
}

//recalciulate timer difference to time and distance
void DSUpdateDistance(int32_t diff)
{
	volatile uint32_t time = (diff * TIMER_B_PRESCALER) / ClockFrequencyMHz; //time ms
	DSDistancemm = (time * 34) / 200; //distance [mm]
}

//Timer2B triggers the measurement by setting PB0
void Timer2BIntHandler(void)
{
	  // clear our interrupt mask
	  TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT);
	  if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3))
		  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
	  else
		  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0xFF);
}

void DSInit()
{
	ClockFrequencyMHz = SysCtlClockGet()/1000000; // gets clock frequency in MHz.
	DSInitTimer2();
	DSInitGPIO();
}

void DSInitTimer2()
{
	  // Configure PB0 as T2CCP0
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	  GPIOPinConfigure(GPIO_PB0_T2CCP0);
	  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);

	  // enable timer 2
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

	  // timer A is a time capture event timer, timer B : periodic
	  TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_PERIODIC );

	  //desired timer period. Set load and prescaler
	  TimerLoadSet(TIMER2_BASE, TIMER_B, TIMER_B_LOAD); // To = 1.5 ms
	  TimerPrescaleSet(TIMER2_BASE, TIMER_BOTH, TIMER_B_PRESCALER); //T = 1.5ms * 126 = 190.5ms

	  // interrupt on timer a capture event and timeout
	  TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);

	  // both edges
	  TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

	  // register our interrupt
	  TimerIntRegister(TIMER2_BASE, TIMER_A, Timer2AIntHandler);
	  TimerIntRegister(TIMER2_BASE, TIMER_B, Timer2BIntHandler);

	  // enables the specific vector associated with Timer2A / Timer2B
	  IntEnable(INT_TIMER2A);
	  IntEnable(INT_TIMER2B);

	  // sets interrupt pririty
	  IntPrioritySet(INT_TIMER2A,0);
	  IntPrioritySet(INT_TIMER2B,1);

	  // int master enable
	  IntMasterEnable();

	  //turn timers on
	  TimerEnable(TIMER2_BASE, TIMER_A);
	  TimerEnable(TIMER2_BASE, TIMER_B);

}

void DSInitGPIO()
{
	//PB3 - trigger
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);

}
