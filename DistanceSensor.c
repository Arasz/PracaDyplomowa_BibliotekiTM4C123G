/*
 * DistanceSensor.c
 *
 *  Created on: 17 lis 2015
 *      Author: Paulina Sadowska
 */
#include "DistanceSensor.h"

uint32_t  DSDistanceCm = 0;
uint32_t value0 = 0;
uint32_t value = 0;
uint32_t diff[20];
uint32_t temp = 0;
uint32_t diffAverage;
volatile uint32_t InterruptErrorCnt;
volatile uint32_t Distance;
volatile uint32_t Overflow = 0;
volatile uint32_t  i = 0;
volatile uint8_t Rising = 0x1;
void Timer2AIntHandler(void)
{

	// save the timer config so we know which edge this is
	  volatile uint32_t interruptStatus = TimerIntStatus(TIMER2_BASE, false);
	  TimerIntClear(TIMER2_BASE, interruptStatus);

	  switch(interruptStatus)
	  {
	  case TIMER_TIMA_TIMEOUT:
		  Overflow++;
		  break;
	  case TIMER_CAPA_EVENT:
		  if(Rising)
		  {
			  value = TimerValueGet(TIMER2_BASE, TIMER_A);
			  Rising ^=0x1;
		  }
		  else
		  {
			    Rising ^=0x1;
				value0 = TimerValueGet(TIMER2_BASE, TIMER_A);

				diff[i] = 65535 * Overflow + value0 - value;
					temp = temp + diff[i];
					i++;
					if(i==10)
					{
						diffAverage = temp / 10;
						Distance = (diffAverage * 170) / 40000;
						temp = 0;
						Overflow = 0;
						i = 0;
					}
		  }
		  break;
	  default:
		  InterruptErrorCnt++;
		  break;
	  }

//	  if(interruptStatus & TIMER_TIMA_TIMEOUT)
//	  {
//		  // clear our interrupt mask
//	  }
//	  else if (interruptStatus & TIMER_EVENT_NEG_EDGE) {
//		// clear our interrupt mask
//	    // set to positive edge
//		    //TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
//	  }
//	  else
//	  {
//		  // clear our interrupt mask
//	    // Rising edge!
//	    // set to negative edge
//	    //TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
//	  }

}

void Timer2BIntHandler(void)
{
	  // clear our interrupt mask
	  TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT);
	  if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3))
		  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x00);
	  else
		  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0xFF);
}

void DSInit(unsigned int frequency)
{
	DSInitTimer2(frequency);
	DSInitGPIO();
	//IntMasterEnable();  //master interrupt enable API for all interrupts.
}

void DSInitTimer2(unsigned int frequency)
{
	 uint16_t ui16Period; //desired clock period
	  // Configure PB0 as T2CCP0
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	  GPIOPinConfigure(GPIO_PB0_T2CCP0);
	  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);

	  // enable timer 2
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

	  // timer a is a time capture event timer
	  TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR |TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_PERIODIC );
	  //TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | |TIMER_CFG_A_ACT_NONE );

	  /*
	  * desired frequency = 10Hz
	  * duty cycle = 50% (interrupt at 1/2 of the desired period)
	  */
	  ui16Period = (SysCtlClockGet() / 1000) / 2;

	  /*
	   * load calculated period into the Timer’s Interval Load register using the TimerLoadSet
	   * you have to subtract one from the timer period since the interrupt fires at the zero count
	   */
	  TimerLoadSet(TIMER2_BASE, TIMER_B, 65534);
	  TimerLoadSet(TIMER2_BASE, TIMER_A, 20534);

	  TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT | TIMER_TIMA_TIMEOUT|TIMER_TIMB_TIMEOUT); 	// interrupt on timer a capture event and timeout

	  // both edges
	  TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

	  // register our interrupt
	  TimerIntRegister(TIMER2_BASE, TIMER_A, Timer2AIntHandler);
	  // register our interrupt
	  TimerIntRegister(TIMER2_BASE, TIMER_B, Timer2BIntHandler);

	  IntEnable(INT_TIMER2A); // enables the specific vector associated with Timer2A
	  IntEnable(INT_TIMER2B); // enables the specific vector associated with Timer2B
	  IntPrioritySet(INT_TIMER2A,0);
	  IntPrioritySet(INT_TIMER2B,1);

	  //TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT | TIMER_TIMA_TIMEOUT|TIMER_TIMB_TIMEOUT); 	// interrupt on timer a capture event and timeout
	  //TimerIntEnable(TIMER2_BASE, TIMER_TIMB_TIMEOUT);  // interrupt on timeout of Timer 2B

	  // int master enable
	  IntMasterEnable();

	  // and turn it on
	  TimerEnable(TIMER2_BASE, TIMER_A);
	  TimerEnable(TIMER2_BASE, TIMER_B);

}

void DSInitGPIO()
{
	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//PB3 - trigger
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
}
