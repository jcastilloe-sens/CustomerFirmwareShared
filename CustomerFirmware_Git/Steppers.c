//*****************************************************************************
//
// Steppers.c - Functions used in e-SENS firmware to control valve and pump
//
// Author: Jason Castillo
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "inc/hw_hibernate.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/eeprom.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "Steppers.h"
#include "inc/tm4c123gh6pm.h"
#include "Amperometrics.h"
#include "Bluetooth.h"
#include "Components.h"
#include "Helper.h"
#include "main.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#ifdef MCU_ZXR
#include "PinMap.h"
#endif

//// Global Variables
//float gPump_Ratio = 1;

uint32_t g_CurrentValvePossition = NO_OF_VALVE_PORTS + 1;
int32_t g_ValveStepsTravelled = 0;
int32_t g_PumpStepsTravelled = 0xFFFFFFFF;
int32_t g_CurrentValveStep = 0;
uint32_t g_ValveADCAvg = 0;
uint8_t g_ValveIndex = 0;
uint32_t g_ValveADCAvg_FW = 0;
uint8_t g_ValveIndex_FW = 0;
uint32_t g_ValveADCAvg_BW = 0;
uint8_t g_ValveIndex_BW = 0;
uint8_t g_ValveHigh = 0;		// Determines polarity of magnet
uint8_t g_ValveDirection = BW;

#ifdef VALVE_STRUCT
struct ValvePort ValveSetup = {.Air = 1, .Samp = 2, .B1 = 3, .B2 = 4, .C2 = 5, .Clean = 6, .Cal_1 = 7, .Cal_2 = 8, .T1 = 9, .Rinse = 10};
#endif

//**************************************************************************
// Setup all the pins used when controlling the steppers and sets up ADC
// used for finding position
// Parameters:  NONE
// 12/27/2022: Added #ifdef MCU_ZXR to handle new MCU
//**************************************************************************
void InitSteppers(uint8_t Valve_home){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);			// Enable ADC0

    SysCtlDelay(300);									// Delay so all ports have time to power up

    // Configure Valve and Pump Pins
#ifdef MCU_ZXR
	GPIODirModeSet( IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, GPIO_DIR_MODE_OUT );	// VALVE_STEP Signal
	GPIOPadConfigSet( IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, GPIO_DIR_MODE_OUT );	// VALVE_DIR Signal
	GPIOPadConfigSet( IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, GPIO_DIR_MODE_OUT );	// PUMP_STEP
	GPIOPadConfigSet( IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, GPIO_DIR_MODE_OUT );	// PUMP_DIR
	GPIOPadConfigSet( IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, GPIO_DIR_MODE_OUT );   // PUMP_SLEEP
	GPIOPadConfigSet( IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, GPIO_DIR_MODE_OUT );	// VALVE_SLEEP
	GPIOPadConfigSet(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

	if(gBoard != CAL1)
	{
		// Analog ADC Inputs Configuration
		GPIOPinTypeADC(IO_VALVE_MAG_SENS_BASE, IO_VALVE_MAG_SENS_PIN);						// VALVE_MAG_SENSOR  - AIN0
		GPIOPinTypeADC(IO_PUMP_MAG_SENS_BASE, IO_PUMP_MAG_SENS_PIN);						// PUMP_MAG_SENSOR - AIN1

		ADCHardwareOversampleConfigure(ADC0_BASE, 16);						// Turns on hardware oversampling by 8
		ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
		ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
		ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

		ADCSequenceEnable(ADC0_BASE, 1);
		ADCIntClear(ADC0_BASE, 1);
	}
	else
	{
		GPIODirModeSet( IO_VALVE_MAG_SENS_BASE, IO_VALVE_MAG_SENS_PIN, GPIO_DIR_MODE_IN );
		GPIOPadConfigSet( IO_VALVE_MAG_SENS_BASE, IO_VALVE_MAG_SENS_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	}

	// Initial State
	GPIOPinWrite(IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, ~IO_VALVE_STEP_PIN);				// VALVE_STEP = LOW
	GPIOPinWrite(IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, IO_VALVE_DIR_PIN);				// VALVE_DIR  = HIGH
	GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// PUMP_STEP  = LOW
	GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// PUMP_DIR  = HIGH
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0x00);			// VALVE_SLEEP  = LOW (sleep mode)
	GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, ~IO_PUMP_SLEEP_PIN);				// PUMP_SLEEP   = LOW (sleep mode)
#else
	GPIODirModeSet( GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );	// VALVE_STEP Signal
	GPIOPadConfigSet( GPIO_PORTC_BASE, GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );	// VALVE_DIR Signal
	GPIOPadConfigSet( GPIO_PORTC_BASE, GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );	// PUMP_STEP
	GPIOPadConfigSet( GPIO_PORTC_BASE, GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );	// PUMP_DIR
	GPIOPadConfigSet( GPIO_PORTC_BASE, GPIO_PIN_5,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );   // PUMP_SLEEP
	GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );	// VALVE_SLEEP
	GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

	if(gBoard != CAL1)
	{
		// Analog ADC Inputs Configuration
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);						// VALVE_MAG_SENSOR  - AIN0
		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);						// PUMP_MAG_SENSOR - AIN1

		ADCHardwareOversampleConfigure(ADC0_BASE, 16);						// Turns on hardware oversampling by 8
		ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
		ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
		ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

		ADCSequenceEnable(ADC0_BASE, 1);
		ADCIntClear(ADC0_BASE, 1);
	}
	else
	{
		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_DIR_MODE_IN );						// I2C used by BT
		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	}

	// Initial State
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, ~GPIO_PIN_6);				// VALVE_STEP = LOW
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);				// VALVE_DIR  = HIGH
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// PUMP_STEP  = LOW
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// PUMP_DIR  = HIGH
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);					// VALVE_SLEEP  = LOW (sleep mode)
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// PUMP_SLEEP   = LOW (sleep mode)
#endif

	if(gBoard >= V6)
	{
		if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE && Valve_home != 0)
			FindPossitionOneValve();
		if(HibernateIsActive() == 1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)
		{
//			HibernateDataGet(&g_CurrentValvePossition, 1);

//			uint32_t HibernateVariables[9];
//			HibernateDataGet(HibernateVariables, 9);
			uint32_t HibernateVariables[8];
			HibernateDataGet(HibernateVariables, 8);

			g_CurrentValvePossition = (0 | HibernateVariables[0]);
			g_CurrentValveStep = (0 | HibernateVariables[1]);
			g_ValveIndex_FW = (0 | HibernateVariables[2]);
			g_ValveADCAvg_FW = (0 | HibernateVariables[3]);
			g_ValveIndex_BW = (0 | HibernateVariables[4]);
			g_ValveADCAvg_BW = (0 | HibernateVariables[5]);
			g_ValveDirection = (0 | HibernateVariables[6]);
			g_ValveHigh = (0 | HibernateVariables[7]);
//			gPump_Ratio = ((float) HibernateVariables[8]) / 1000;

//			UARTprintf("Wake up Pump Ratio: %d \n", (int) (gPump_Ratio * 1000));

//			UARTprintf("Wake up valve variables: \n");
//			UARTprintf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n", g_CurrentValvePossition, g_CurrentValveStep, g_ValveIndex_FW,
//					g_ValveADCAvg_FW, g_ValveIndex_BW, g_ValveADCAvg_BW, g_ValveDirection, g_ValveHigh);
		}

	}
	else
	{
		if((gBoard != CAL1) && Valve_home != 0)
			FindPossitionOneValve();
	}
}

//**************************************************************************
// Driver for stepper motors (move by a number of steps)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  motor (valve or pump)
//				Direction: FW or BW
//				Number of steps to move
//**************************************************************************
void StepperRun(uint8_t motor, uint8_t Direction, uint32_t NumberOfSteps){

	int Delay = 40000;			// starting delay
	int EndDelay = 11000;
	int DelayDecrease = 400;

	switch (motor) {

	case VALVE:
#ifdef MCU_ZXR
		GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, IO_VALVE_SLEEP_PIN);				// VALVE_SLEEP  = HIGH (turn ON)
		userDelay(1, 0);

		if(Direction == FW)
			GPIOPinWrite(IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, IO_VALVE_DIR_PIN);				// DIR  = HIGH
		else
			GPIOPinWrite(IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, 0x00);				// DIR  = LOW
		SysCtlDelay(5);

		while(NumberOfSteps > 0){
			GPIOPinWrite(IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, IO_VALVE_STEP_PIN);				// STEP = HIGH
			SysCtlDelay(Delay); //8000 works for PG20, 5000 works for PG15									// wait a bit
			GPIOPinWrite(IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, ~IO_VALVE_STEP_PIN);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
			NumberOfSteps--;
			Delay = Delay - DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}

		GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0x00);					// VALVE_SLEEP  = LOW (turn OFF)
#else
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);				// VALVE_SLEEP  = HIGH (turn ON)
		userDelay(1, 0);

		if(Direction == FW)
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);				// DIR  = HIGH
		else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00);				// DIR  = LOW
		SysCtlDelay(5);

		while(NumberOfSteps > 0){
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);				// STEP = HIGH
			SysCtlDelay(Delay); //8000 works for PG20, 5000 works for PG15									// wait a bit
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, ~GPIO_PIN_6);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
			NumberOfSteps--;
			Delay = Delay - DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}

		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);					// VALVE_SLEEP  = LOW (turn OFF)

#endif

		break;

	case PUMP:

#ifdef MCU_ZXR
		//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)

				if(Direction == FW)
					GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
				else
					GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
				SysCtlDelay(5);

				while(NumberOfSteps > 0){
					GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
					SysCtlDelay(5000);									// wait a bit
					GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
					SysCtlDelay(5000);									// wait a bit
					NumberOfSteps--;
				}

		//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);					// PUMP_SLEEP   = OFF (turn ON)

#else
		//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)

				if(Direction == FW)
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
				else
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
				SysCtlDelay(5);

				while(NumberOfSteps > 0){
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
					SysCtlDelay(5000);									// wait a bit
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
					SysCtlDelay(5000);									// wait a bit
					NumberOfSteps--;
				}

		//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);					// PUMP_SLEEP   = OFF (turn ON)

#endif

		break;

	default:
		break;
	}

	return;
}

#ifndef OAD
//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of seconds to move
//**************************************************************************
void PumpStepperRun(uint8_t Direction, float ulSeconds){

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
	userDelay(1, 0);

	int Delay = 8000;			// starting delay
	int EndDelay = 6000;
	int DelayDecrease = 300;
	int count = 0;

	int Minutes = 0;				// Number of minutes
	while(ulSeconds > 60)
	{
		Minutes++;				// Add up number of minutes to run pump
		ulSeconds -= 60;		// Find number of seconds remaining after removing full minutes
	}

	if(Direction == FW)
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
	else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
	SysCtlDelay(5);

	int i;
	for(i = 0; i < Minutes; i++)
	{
		TimerLoadSet(TIMER0_BASE, TIMER_A, (60 * SysCtlClockGet()));		// Timer set
		TimerEnable(TIMER0_BASE, TIMER_A);
		while (g_TimerInterruptFlag == 0){
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
			count++;
			Delay = Delay - DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}
		g_TimerInterruptFlag = 0;
	}

	TimerLoadSet(TIMER0_BASE, TIMER_A, (ulSeconds * SysCtlClockGet()));		// Timer set
	TimerEnable(TIMER0_BASE, TIMER_A);
	while (g_TimerInterruptFlag == 0){
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
		SysCtlDelay(Delay);									// wait a bit
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
		SysCtlDelay(Delay);									// wait a bit
		count++;
		Delay = Delay - DelayDecrease;
		if (Delay < EndDelay)
			Delay = EndDelay;
	}
	g_TimerInterruptFlag = 0;

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
}

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of seconds to move
//**************************************************************************
void PumpStepperRunFast(uint8_t Direction, float ulSeconds){

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
	userDelay(1, 0);

	int Delay = 8000;			// starting delay
	int EndDelay = 2500;
	int DelayDecrease = 10;
	int count = 0;

	int Minutes = 0;				// Number of minutes
	while(ulSeconds > 60)
	{
		Minutes++;				// Add up number of minutes to run pump
		ulSeconds -= 60;		// Find number of seconds remaining after removing full minutes
	}

	if(Direction == FW)
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
	else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
	SysCtlDelay(5);

	int i;
	for(i = 0; i < Minutes; i++)
	{
		TimerLoadSet(TIMER0_BASE, TIMER_A, (60 * SysCtlClockGet()));		// Timer set
		TimerEnable(TIMER0_BASE, TIMER_A);
		while (g_TimerInterruptFlag == 0){
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
			count++;
			Delay = Delay - DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}
		g_TimerInterruptFlag = 0;
	}

	TimerLoadSet(TIMER0_BASE, TIMER_A, (ulSeconds * SysCtlClockGet()));		// Timer set
	TimerEnable(TIMER0_BASE, TIMER_A);
	while (g_TimerInterruptFlag == 0){
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
		SysCtlDelay(Delay);									// wait a bit
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
		SysCtlDelay(Delay);									// wait a bit
		count++;
		Delay = Delay - DelayDecrease;
		if (Delay < EndDelay)
			Delay = EndDelay;
	}
	g_TimerInterruptFlag = 0;

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
}

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//**************************************************************************
void PumpStepperRunStep(uint8_t Direction, uint32_t NumberOfSteps){

	if((gui32Error & ABORT_ERRORS) == 0)
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
		userDelay(1, 0);

		int Delay = 8000;			// starting delay
		int EndDelay = 3500;		// 3000
		int DelayDecrease = 300;

		// Number of steps that it will be accelerating and decelerating
		uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
		if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
			Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

		uint32_t StepsToGo = NumberOfSteps;

		if(Direction == FW)
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
		else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
		SysCtlDelay(5);

		while(StepsToGo > 0){
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
			StepsToGo--;

			if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
			{
				Delay -= DelayDecrease;
				if (Delay < EndDelay)
					Delay = EndDelay;
			}
			else if(StepsToGo <= (Accel_Steps + 1))
			{
				Delay += DelayDecrease;
			}
//			Delay = Delay - DelayDecrease;
//			if (Delay < EndDelay)
//				Delay = EndDelay;

			if((gui32Error & ABORT_ERRORS) != 0)
				break;
		}

		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)

		if(Direction == FW)
			g_PumpStepsTravelled += NumberOfSteps;
		else
			g_PumpStepsTravelled -= NumberOfSteps;
	}


//	g_PumpStepsTravelled += NumberOfSteps;	// Doesn't do anything.. NumberOfSteps = 0 after moving
}
#endif

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//**************************************************************************
void PumpStepperRunStepSlow(uint8_t Direction, uint32_t NumberOfSteps){

	//		if(ESPUMP)
	//		{
	//			ESPumpStepperRunStepSlow(Direction, NumberOfSteps);
	//		}
	//		else
	//		{
#ifdef MCU_ZXR
	GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)
	userDelay(1, 0);

	int Delay = 8000;			// starting delay
	int EndDelay = 6000;		// 3000

	int DelayDecrease = 20;

	// Number of steps that it will be accelerating and decelerating
	uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
	if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
		Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

	uint32_t StepsToGo = NumberOfSteps;

	if(Direction == FW)
		GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
	else
		GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
	SysCtlDelay(5);

	while(StepsToGo > 0){
		GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
		SysCtlDelay(Delay);									// wait a bit
		GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
		SysCtlDelay(Delay);									// wait a bit
		StepsToGo--;

		if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
		{
			Delay -= DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}
		else if(StepsToGo <= (Accel_Steps + 1))
		{
			Delay += DelayDecrease;
		}
//		Delay = Delay - DelayDecrease;
//		if (Delay < EndDelay)
//			Delay = EndDelay;
	}

	if((gui32Error & ABORT_ERRORS) != 0)
		SysCtlDelay(SysCtlClockGet()/6000);	// Delay 1/2 ms before turning on pump sleep

	GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, ~IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = OFF (turn ON)

#else
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
	userDelay(1, 0);

	int Delay = 8000;			// starting delay
	int EndDelay = 6000;		// 3000

	int DelayDecrease = 20;

	// Number of steps that it will be accelerating and decelerating
	uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
	if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
		Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

	uint32_t StepsToGo = NumberOfSteps;

	if(Direction == FW)
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
	else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
	SysCtlDelay(5);

	while(StepsToGo > 0){
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
		SysCtlDelay(Delay);									// wait a bit
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
		SysCtlDelay(Delay);									// wait a bit
		StepsToGo--;

		if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
		{
			Delay -= DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}
		else if(StepsToGo <= (Accel_Steps + 1))
		{
			Delay += DelayDecrease;
		}
//		Delay = Delay - DelayDecrease;
//		if (Delay < EndDelay)
//			Delay = EndDelay;
	}

	if((gui32Error & ABORT_ERRORS) != 0)
		SysCtlDelay(SysCtlClockGet()/6000);	// Delay 1/2 ms before turning on pump sleep

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)

#endif

	// Only use this function in the FindPossitionZeroPump function don't add steps moved here as it throws off the pump drift calculation
//	if(Direction == FW)
//		g_PumpStepsTravelled += NumberOfSteps;
//	else
//		g_PumpStepsTravelled -= NumberOfSteps;

	//	g_PumpStepsTravelled += NumberOfSteps;	// Doesn't do anything.. NumberOfSteps = 0 after moving
	//		}
}

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//**************************************************************************
void PumpStepperRunStepSlow_AbortReady(uint8_t Direction, uint32_t NumberOfSteps){

	if((gui32Error & ABORT_ERRORS) == 0)
	{
#ifdef MCU_ZXR
		//		UARTprintf("Pump Starting at step: %d", g_PumpStepsTravelled);
				GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)
				userDelay(1, 0);

				int Delay = 8000;			// starting delay
				int EndDelay = 6000;		// 3000

				int DelayDecrease = 300;

				// Number of steps that it will be accelerating and decelerating
				uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
				if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
					Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

				uint32_t StepsToGo = NumberOfSteps;

				if(Direction == FW)
					GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
				else
					GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
				SysCtlDelay(5);

				while(StepsToGo > 0){
					GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
					SysCtlDelay(Delay);									// wait a bit
					GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
					SysCtlDelay(Delay);									// wait a bit
					StepsToGo--;

					if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
					{
						Delay -= DelayDecrease;
						if (Delay < EndDelay)
							Delay = EndDelay;
					}
					else if(StepsToGo <= (Accel_Steps + 1))
					{
						Delay += DelayDecrease;
					}
					//				Delay = Delay - DelayDecrease;
					//				if (Delay < EndDelay)
					//					Delay = EndDelay;

					if((gui32Error & ABORT_ERRORS) != 0)
						break;
				}

				GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, ~IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = OFF (turn ON)

#else
		//		UARTprintf("Pump Starting at step: %d", g_PumpStepsTravelled);
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
				userDelay(1, 0);

				int Delay = 8000;			// starting delay
				int EndDelay = 6000;		// 3000

				int DelayDecrease = 300;

				// Number of steps that it will be accelerating and decelerating
				uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
				if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
					Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

				uint32_t StepsToGo = NumberOfSteps;

				if(Direction == FW)
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
				else
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
				SysCtlDelay(5);

				while(StepsToGo > 0){
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
					SysCtlDelay(Delay);									// wait a bit
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
					SysCtlDelay(Delay);									// wait a bit
					StepsToGo--;

					if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
					{
						Delay -= DelayDecrease;
						if (Delay < EndDelay)
							Delay = EndDelay;
					}
					else if(StepsToGo <= (Accel_Steps + 1))
					{
						Delay += DelayDecrease;
					}
					//				Delay = Delay - DelayDecrease;
					//				if (Delay < EndDelay)
					//					Delay = EndDelay;

					if((gui32Error & ABORT_ERRORS) != 0)
						break;
				}

				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)

#endif

		if(g_PumpStepsTravelled != 0xFFFFFFFF)	// Don't adjust this if pump home position hasn't been found yet, assuming I never pump -1 steps...
		{
			if(Direction == FW)
				g_PumpStepsTravelled += NumberOfSteps;
			else
				g_PumpStepsTravelled -= NumberOfSteps;
		}


//		g_PumpStepsTravelled += NumberOfSteps;	// Doesn't do anything.. NumberOfSteps = 0 after moving
//		UARTprintf("Pump Starting at step: %d", g_PumpStepsTravelled);
	}
}

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//				EndDelay; Delay to end pump at to adjust speed
//**************************************************************************
void PumpStepperRunStepSpeed(uint8_t Direction, uint32_t NumberOfSteps, uint32_t EndDelay){

	//		if(ESPUMP)
	//		{
	//			ESPumpStepperRunStepSlow(Direction, NumberOfSteps);
	//		}
	//		else
	//		{
	gPumping = 1; // Set flag so interrupts know pumping is happening and they can be shortened

#ifdef MCU_ZXR
	GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)
	userDelay(1, 0);

	int Delay = 8000;			// starting delay
	//		int EndDelay = 6000;		// 3000
	if(EndDelay > Delay)
		Delay = EndDelay;

	int DelayDecrease = 300;

	// Number of steps that it will be accelerating and decelerating
	uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
	if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
		Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

	uint32_t StepsToGo = NumberOfSteps;

	if(Direction == FW)
		GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
	else
		GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
	SysCtlDelay(5);

	while(StepsToGo > 0){
		GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
		SysCtlDelay(Delay);									// wait a bit
		GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
		SysCtlDelay(Delay);									// wait a bit
		StepsToGo--;

		if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
		{
			Delay -= DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}
		else if(StepsToGo <= (Accel_Steps + 1))
		{
			Delay += DelayDecrease;
		}
//		Delay = Delay - DelayDecrease;
//		if (Delay < EndDelay)
//			Delay = EndDelay;
	}

	GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, ~IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = OFF (turn ON)

#else
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
	userDelay(1, 0);

	int Delay = 8000;			// starting delay
	//		int EndDelay = 6000;		// 3000
	if(EndDelay > Delay)
		Delay = EndDelay;

	int DelayDecrease = 300;

	// Number of steps that it will be accelerating and decelerating
	uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
	if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
		Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

	uint32_t StepsToGo = NumberOfSteps;

	if(Direction == FW)
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
	else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
	SysCtlDelay(5);

	while(StepsToGo > 0){
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
		SysCtlDelay(Delay);									// wait a bit
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
		SysCtlDelay(Delay);									// wait a bit
		StepsToGo--;

		if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
		{
			Delay -= DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;
		}
		else if(StepsToGo <= (Accel_Steps + 1))
		{
			Delay += DelayDecrease;
		}
//		Delay = Delay - DelayDecrease;
//		if (Delay < EndDelay)
//			Delay = EndDelay;
	}

	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)

#endif

	if(g_PumpStepsTravelled != 0xFFFFFFFF)	// Don't adjust this if pump home position hasn't been found yet, assuming I never pump -1 steps...
	{
		if(Direction == FW)
			g_PumpStepsTravelled += NumberOfSteps;
		else
			g_PumpStepsTravelled -= NumberOfSteps;
	}

	if(gPumping == 2)
		update_Battery(0);
	gPumping = 0; // Set flag so interrupts know pumping is happening and they can be shortened


	//	g_PumpStepsTravelled += NumberOfSteps;	// Doesn't do anything.. NumberOfSteps = 0 after moving
	//		}
}

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//				EndDelay; Delay to end pump at to adjust speed
//**************************************************************************
void PumpStepperRunStepSpeed_AbortReady(uint8_t Direction, uint32_t NumberOfSteps, uint32_t EndDelay){

	if((gui32Error & ABORT_ERRORS) == 0)
	{
//		UARTprintf("Pump Starting at step: %d\n", g_PumpStepsTravelled);
		gPumping = 1; // Set flag so interrupts know pumping is happening and they can be shortened
#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)
#else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
#endif
		userDelay(1, 0);

		// Fastest pump turns is with delay of 3000, needs (8000 - 3000)/300 = 16.66 steps
		int Delay = 8000;			// starting delay
		//		int EndDelay = 6000;		// 3000
		if(EndDelay > Delay)
			Delay = EndDelay;

		int DelayDecrease = 20; //100; //300;

		// Number of steps that it will be accelerating and decelerating
		uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
		if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
			Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

		uint32_t StepsToGo = NumberOfSteps;

#ifdef MCU_ZXR
		if(Direction == FW)
			GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
		else
			GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
#else
		if(Direction == FW)
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
		else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
#endif

		SysCtlDelay(5);

		while(StepsToGo > 0){
#ifdef MCU_ZXR
			GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
#else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
#endif
			SysCtlDelay(Delay);									// wait a bit
			StepsToGo--;

			if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
			{
				Delay -= DelayDecrease;
				if (Delay < EndDelay)
					Delay = EndDelay;
			}
			else if(StepsToGo <= (Accel_Steps + 1))
			{
				Delay += DelayDecrease;
			}
//			Delay = Delay - DelayDecrease;
//			if (Delay < EndDelay)
//				Delay = EndDelay;

			if((gui32Error & ABORT_ERRORS) != 0)
				break;
		}

		if((gui32Error & ABORT_ERRORS) != 0)
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us before turning on pump sleep

#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, ~IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = OFF (turn ON)
#else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
#endif

		if(g_PumpStepsTravelled != 0xFFFFFFFF)	// Don't adjust this if pump home position hasn't been found yet, assuming I never pump -1 steps...
		{
			if(Direction == FW)
				g_PumpStepsTravelled += NumberOfSteps;
			else
				g_PumpStepsTravelled -= NumberOfSteps;
		}

		if(gPumping == 2)
			update_Battery(0);
		gPumping = 0; // Set flag so interrupts know pumping is happening and they can be shortened

//		while(g_PumpStepsTravelled >= STEP_PER_REV_PUMP)
//			g_PumpStepsTravelled -= STEP_PER_REV_PUMP;
//		while(g_PumpStepsTravelled <= -STEP_PER_REV_PUMP)
//			g_PumpStepsTravelled += STEP_PER_REV_PUMP;
//		UARTprintf("Pump Ending at step: %d\n", g_PumpStepsTravelled);
	}
}

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Time in seconds; Max time = 171 seconds
//				EndDelay; Delay to end pump at to adjust speed
//**************************************************************************
void PumpStepperRunTimeSpeed_AbortReady(uint8_t Direction, uint32_t Time, uint32_t EndDelay){

	if((gui32Error & ABORT_ERRORS) == 0)
	{
//		UARTprintf("Pump Starting at step: %d\n", g_PumpStepsTravelled);
		gPumping = 1; // Set flag so interrupts know pumping is happening and they can be shortened
#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)
#else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
#endif
		userDelay(1, 0);

		// Fastest pump turns is with delay of 3000, needs (8000 - 3000)/300 = 16.66 steps
		int Delay = 8000;			// starting delay
		//		int EndDelay = 6000;		// 3000
		if(EndDelay > Delay)
			Delay = EndDelay;

		int DelayDecrease = 20; //100; //300;

		// Number of steps that it will be accelerating and decelerating
		uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
//		if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
//			Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

//		uint32_t StepsToGo = NumberOfSteps;

#ifdef MCU_ZXR
		if(Direction == FW)
			GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
		else
			GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
#else
		if(Direction == FW)
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
		else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
#endif

		SysCtlDelay(5);

		uint32_t i;
		uint32_t NumberOfSteps = 0;

		g_TimerInterruptFlag = 0;
		TimerLoadSet(TIMER0_BASE, TIMER_A, (Time * SysCtlClockGet()));		// Timer set
		TimerEnable(TIMER0_BASE, TIMER_A);
		while(g_TimerInterruptFlag == 0)
		{
#ifdef MCU_ZXR
			GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
#else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
#endif

			Delay -= DelayDecrease;
			if (Delay < EndDelay)
				Delay = EndDelay;

			NumberOfSteps++;

			if((gui32Error & ABORT_ERRORS) != 0)
				break;
		}

		// Decelerate the pump after pumping for the specified time
		for(i = 0; i < Accel_Steps; i++)
		{
#ifdef MCU_ZXR
			GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
#else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
			SysCtlDelay(Delay);									// wait a bit
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
			SysCtlDelay(Delay);									// wait a bit
#endif

			Delay += DelayDecrease;

			NumberOfSteps++;
		}

		if((gui32Error & ABORT_ERRORS) != 0)
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us before turning on pump sleep

#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, ~IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = OFF (turn ON)
#else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
#endif

		if(g_PumpStepsTravelled != 0xFFFFFFFF)	// Don't adjust this if pump home position hasn't been found yet, assuming I never pump -1 steps...
		{
			if(Direction == FW)
				g_PumpStepsTravelled += NumberOfSteps;
			else
				g_PumpStepsTravelled -= NumberOfSteps;
		}

		if(gPumping == 2)
			update_Battery(0);
		gPumping = 0; // Set flag so interrupts know pumping is happening and they can be shortened

		DEBUG_PRINT(UARTprintf("Travelled %d steps in %d seconds\n", NumberOfSteps, Time);)

//		while(g_PumpStepsTravelled >= STEP_PER_REV_PUMP)
//			g_PumpStepsTravelled -= STEP_PER_REV_PUMP;
//		while(g_PumpStepsTravelled <= -STEP_PER_REV_PUMP)
//			g_PumpStepsTravelled += STEP_PER_REV_PUMP;
//		UARTprintf("Pump Ending at step: %d\n", g_PumpStepsTravelled);
	}
}

//**************************************************************************
// Driver for PUMP (move for a number of seconds)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//				EndDelay; Delay to end pump at to adjust speed
//**************************************************************************
void PumpStepperMix(uint8_t StartDirection, uint32_t NumberOfSteps, uint32_t EndDelay, uint8_t cycles){
	uint8_t i;

	if((gui32Error & ABORT_ERRORS) == 0)
	{
#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)
#else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
#endif
		userDelay(1, 0);
	}

	gPumping = 1; // Set flag so interrupts know pumping is happening and they can be shortened

	for(i = 0; i < cycles; i++)
	{
		if((gui32Error & ABORT_ERRORS) == 0)
		{
			// Fastest pump turns is with delay of 3000, needs (8000 - 3000)/300 = 16.66 steps
			int Delay = 8000;			// starting delay
			//		int EndDelay = 6000;		// 3000
			if(EndDelay > Delay)
				Delay = EndDelay;

			int DelayDecrease = 20; //100 //300;

			// Number of steps that it will be accelerating and decelerating
			uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
			if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
				Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

			uint32_t StepsToGo = NumberOfSteps;

#ifdef MCU_ZXR
			if(StartDirection == FW)
				GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
			else
				GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
#else
			if(StartDirection == FW)
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
			else
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
#endif

			SysCtlDelay(5);

			while(StepsToGo > 0){
#ifdef MCU_ZXR
				GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
				SysCtlDelay(Delay);									// wait a bit
				GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
				SysCtlDelay(Delay);									// wait a bit
#else
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
				SysCtlDelay(Delay);									// wait a bit
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
				SysCtlDelay(Delay);									// wait a bit
#endif

				StepsToGo--;

				if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
				{
					Delay -= DelayDecrease;
					if (Delay < EndDelay)
						Delay = EndDelay;
				}
				else if(StepsToGo <= (Accel_Steps + 1))
				{
					Delay += DelayDecrease;
				}

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}

//			if((gui32Error & ABORT_ERRORS) != 0)
//				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us before turning on pump sleep
//
//			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
		}

		if((gui32Error & ABORT_ERRORS) == 0)
		{
			// Fastest pump turns is with delay of 3000, needs (8000 - 3000)/300 = 16.66 steps
			int Delay = 8000;			// starting delay
			//		int EndDelay = 6000;		// 3000
			if(EndDelay > Delay)
				Delay = EndDelay;

			int DelayDecrease = 20; //300;

			// Number of steps that it will be accelerating and decelerating
			uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
			if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
				Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half

			uint32_t StepsToGo = NumberOfSteps;

#ifdef MCU_ZXR
			if(StartDirection == BW)	// Flipped logic here so it pumps in opposite direction
				GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
			else
				GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
#else
			if(StartDirection == BW)	// Flipped logic here so it pumps in opposite direction
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
			else
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
#endif
			SysCtlDelay(16000);

			while(StepsToGo > 0){
#ifdef MCU_ZXR
				GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
				SysCtlDelay(Delay);									// wait a bit
				GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
				SysCtlDelay(Delay);
#else
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
				SysCtlDelay(Delay);									// wait a bit
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
				SysCtlDelay(Delay);
#endif
							// wait a bit
				StepsToGo--;

				if((NumberOfSteps - StepsToGo) <= (Accel_Steps + 1))
				{
					Delay -= DelayDecrease;
					if (Delay < EndDelay)
						Delay = EndDelay;
				}
				else if(StepsToGo <= (Accel_Steps + 1))
				{
					Delay += DelayDecrease;
				}

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}

//			if((gui32Error & ABORT_ERRORS) != 0)
//				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us before turning on pump sleep
//
//			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
		}
	}

	if((gui32Error & ABORT_ERRORS) != 0)
		SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us before turning on pump sleep

#ifdef MCU_ZXR
	GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, ~IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = OFF (turn ON)
#else
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
#endif

	if(gPumping == 2)
		update_Battery(0);
	gPumping = 0; // Set flag so interrupts know pumping is happening and they can be shortened

}

//*****************************************************************************
// This function finds possiton 0 of the pump stepper
//*****************************************************************************
uint8_t FindPossitionZeroPump(void){
	// Only run if we are not aborting
	if((gui32Error & ABORT_ERRORS) == 0)
	{
//		UARTprintf("Finding Home Position of Pump!\n");
//		UARTprintf("Pump steps traveled (should never be greater than 1000): %d\n", g_PumpStepsTravelled);
//		int HowMany = STEP_PER_REV_PUMP;
		int StepsFromLowest = 0;
		int StepsFromHighest = 0;
		int Stepstogo;
		float Lowest = 5000;
		float Highest = 0;
		uint32_t ADC0Value[3];

		uint32_t Avg_Sum = 0;
		#define Buffer_Size		13	// Use odd numbers so center of buffer is on a step, also math is assuming odd numbers, defined so I can make the array definitions dependent
		uint16_t Avg_Buffer[Buffer_Size];
		uint16_t Start_Buffer[Buffer_Size - 1];	// One less than buffer size because there will always be at least one point used from the other buffer
		int8_t Buffer_ptr = 0;

#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)
#else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
#endif
		userDelay(1, 0);

		ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
		while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
		ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
		ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);						// Get data from ADC

		userDelay(1, 1);	// Devices requires 1 ms between changing sleep and stepping pump

//		Avg_Sum += ADC0Value[1];
//
//		if(Lowest > ADC0Value[1]){
//			Lowest = ADC0Value[1];
//		}
//		if(Highest < ADC0Value[1]){
//			Highest = ADC0Value[1];
//		}

//		while(HowMany > 0){
		uint16_t i;
		uint8_t k;
		for(i = 0; i < STEP_PER_REV_PUMP; i++)
		{
			StepperRun(PUMP, FW, 1);
			StepsFromLowest++;
			StepsFromHighest++;
			SysCtlDelay(2000);
			ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
			while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
			ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
			ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);						// Get data from ADC

////			UARTprintf("%d\n", ADC0Value[1]);
//			Avg_Sum += ADC0Value[1];
//
//			if(Lowest > ADC0Value[1]){
//				Lowest = ADC0Value[1];
//				StepsFromLowest = 0;
//			}
//			if(Highest < ADC0Value[1])
//			{
//				Highest = ADC0Value[1];
//				StepsFromHighest = 0;
//			}
//			//			HowMany--;

			// Fill in the average buffer and increment the pointer
			// Built buffer to average last 5 points, set steps from highest/lowest to 2 less than current number to account for average being center of buffer not the front edge of buffer
			// Beginning edge case, set to i/2 so i = 0, 0; i = 1, 0; i = 2, 1; i = 3, 1
			// Ending edge case,
			Avg_Buffer[Buffer_ptr] = ADC0Value[1];
//			UARTprintf("%d\t", ADC0Value[1]);

			float Avg = 0;
			if(i < (Buffer_Size - 1))	// If I haven't filled the buffer yet, this is my beginning edge case
			{
				Start_Buffer[i] = ADC0Value[1];
				if(i >= (Buffer_Size))	// Wait until we have filled the buffer, we will average these points back for the first point
				{
					// Average Buffer
					for(k = 0; k < (i + 1); k++)
						Avg += Avg_Buffer[k];
					Avg /= Buffer_Size;
					Avg_Sum += Avg;

					if(Lowest > Avg){
						Lowest = Avg;
						StepsFromLowest = (Buffer_Size / 2);
					}
					if(Highest < Avg)
					{
						Highest = Avg;
						StepsFromHighest = (Buffer_Size / 2);
					}
				}
			}
			else // Have a full buffer for the rest of the points
			{
				for(k = 0; k < Buffer_Size; k++)
					Avg += Avg_Buffer[k];
				Avg /= Buffer_Size;
				Avg_Sum += Avg;

				if(Lowest > Avg){
					Lowest = Avg;
					StepsFromLowest = Buffer_Size / 2;	// Set to 2 because we are averaging 5 points, and I want center of buffer not leading edge of buffer
				}
				if(Highest < Avg)
				{
					Highest = Avg;
					StepsFromHighest = Buffer_Size / 2;	// Set to 2 because we are averaging 5 points, and I want center of buffer not leading edge of buffer
				}
			}

//			UARTprintf("Average:\t%d\t", Avg);

			if(i == (STEP_PER_REV_PUMP - 1))	// After collecting all the points, stitch together the beginning and end points to handle the edge cases
			{
				//
				uint8_t j;
				uint8_t set_flag_low = 0;	// Always want the first point that reads highest/lowest, so have flag for average of first points that I can set if the first points
				uint8_t set_flag_high = 0;	// Always want the first point that reads highest/lowest, so have flag for average of first points that I can set if the first points
				for(j = 0; j < Buffer_Size / 2; j++)	// Loop through last points that center of buffer doesn't reach
				{
					// Handle the first points by stitching together the beginning and ending data and using that to create a smooth average
					Avg = 0;
					for(k = 0; k < ((Buffer_Size / 2) + 1 + j); k++)	// Loop through the number of points that will be included in beginning case
					{
						// Add all the points going into this average, from the starting buffer
						Avg += Start_Buffer[k];
					}
					for(k = 0; k < ((Buffer_Size / 2) - j); k++) // Loop through the rest of the buffer size points that weren't included in the above loop
					{
						int8_t index = Buffer_ptr - k;
						if(index < 0)	// Check that we don't have a negative index, if we do loop back to the end of buffer
							index += Buffer_Size;
						Avg += Avg_Buffer[index];
					}

					Avg /= Buffer_Size;
					// Don't add to average sum, already have the 1000 points

					if(Lowest > Avg || (Lowest == Avg && set_flag_low == 0)){
						Lowest = Avg;
						StepsFromLowest = STEP_PER_REV_PUMP - (j + 1);	// Set to center of buffer including the starting points
						set_flag_low = 1;
					}
					if(Highest < Avg || (Highest == Avg && set_flag_high == 0))
					{
						Highest = Avg;
						StepsFromHighest = STEP_PER_REV_PUMP - (j + 1);	// Set to center of buffer including the starting points
						set_flag_high = 1;
					}

//					UARTprintf("Average:\t%d\t", Avg);

					// Handle the end case by including the points from the start buffer to create a smooth average
					Avg = 0;
					for(k = 0; k < (Buffer_Size - 1) - j; k++)	// Loop through the number of points that will be included in end case
					{
						// Add all the points going into this average, starting at last point and working backwards
						int8_t index = Buffer_ptr - k;
						if(index < 0)	// Check that we don't have a negative index, if we do loop back to the end of buffer
							index += Buffer_Size;
						Avg += Avg_Buffer[index];
					}
					for(k = 0; k < (1 + j); k++) // Loop through the rest of the buffer size points that weren't included in the above loop
					{
						Avg += Start_Buffer[k];
					}

					Avg /= Buffer_Size;
					// Don't add to average sum, already have the 1000 points

					if(Lowest > Avg){
						Lowest = Avg;
						StepsFromLowest = (Buffer_Size / 2) - (j + 1);	// Set to center of buffer including the starting points
					}
					if(Highest < Avg)
					{
						Highest = Avg;
						StepsFromHighest = (Buffer_Size / 2) - (j + 1);	// Set to center of buffer including the starting points
					}

//					UARTprintf("Average:\t%d\t", Avg);
				}
			}

//			UARTprintf("Highest:\t%d\n", StepsFromHighest);
			// Put this after the math so I can use the current pointer value
			Buffer_ptr++;
			if(Buffer_ptr >= Buffer_Size)
				Buffer_ptr = 0;

			// If abort is received return without finishing
			if((gui32Error & ABORT_ERRORS) != 0)
				break;
		} //while (how_many)

		uint32_t Avg_calc = Avg_Sum / STEP_PER_REV_PUMP;
//		UARTprintf("Avg: %d\t", Avg_calc);
//		UARTprintf("High: %d\t", (int) Highest);
//		UARTprintf("Low: %d\t", (int) Lowest);

		if((Highest - Avg_calc) > (Avg_calc - Lowest))
		{
			Stepstogo = (STEP_PER_REV_PUMP - StepsFromHighest) + 501;
		}
		else
		{
			Stepstogo = (STEP_PER_REV_PUMP - StepsFromLowest) + 501;
		}

		if(Stepstogo >= STEP_PER_REV_PUMP)
			Stepstogo -= STEP_PER_REV_PUMP;

		//	StepperRun(PUMP, FW, Stepstogo);
		PumpStepperRunStepSlow(FW, Stepstogo);

#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, 0x00);					// PUMP_SLEEP   = OFF (turn ON)
#else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);					// PUMP_SLEEP   = OFF (turn ON)
#endif

		if(g_PumpStepsTravelled == 0xFFFFFFFF)
		{
			DEBUG_PRINT(UARTprintf("First time finding pump home, can't calculate drift!\n");)
		}
		else
		{
			DEBUG_PRINT(UARTprintf("Pump traveled %d steps!\n", g_PumpStepsTravelled);)
			g_PumpStepsTravelled %= 1000;	// Really only care about the remainder of steps

			int Steps_Drifted = 0;
			if(g_PumpStepsTravelled < 0)
				Steps_Drifted = (g_PumpStepsTravelled * -1) - Stepstogo;
			else
				Steps_Drifted = (STEP_PER_REV_PUMP - g_PumpStepsTravelled) - Stepstogo;

			if(Steps_Drifted > 500)
			{
				Steps_Drifted = 1000 - Steps_Drifted;
				DEBUG_PRINT(UARTprintf("Pump Steps Drifted:\t -%d\n", Steps_Drifted);)
			}
			else
			{
				DEBUG_PRINT(UARTprintf("Pump Steps Drifted:\t %d\n", Steps_Drifted);)
			}

			if(Steps_Drifted > 25)
				gui32Error |= PUMP_DRIFT_FAIL;
		}


//		UARTprintf("Pump steps traveled (should never be greater than 1000): %d\n", g_PumpStepsTravelled);
//		UARTprintf("Pump Steps Drifted: %d\n", Steps_Drifted);

		g_PumpStepsTravelled = 0;

		if(Avg_calc > 2900 || Avg_calc < 100)
		{
			DEBUG_PRINT(UARTprintf("Pump magnet sensor is not reading correctly!\n");)
			gui32Error |= PUMP_DRIFT_FAIL;
			return 0;
		}
		else if(Highest - Lowest > 50)
			return 1;
		else
			return 0;
	}

	return 0;
}

//**************************************************************************
// Driver for VALVE stepper motor (move by a number of steps)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//**************************************************************************
void ValveStepperRunSlow(int Direction, int NumberOfSteps){

	int Delay = 12000;		// Beginning delay decreased to avoid stepper resonance frequency during ramp	// starting delay 40000
	int EndDelay = 9000;		//
	int DelayDecrease = 400;

#ifdef MCU_ZXR
	if(Direction == FW)
		GPIOPinWrite(IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, IO_VALVE_DIR_PIN);				// DIR  = HIGH
	else
		GPIOPinWrite(IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, 0x00);				// DIR  = LOW
#else
	if(Direction == FW)
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);				// DIR  = HIGH
	else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00);				// DIR  = LOW
#endif

	SysCtlDelay(100000);

	while(NumberOfSteps > 0){
#ifdef MCU_ZXR
		GPIOPinWrite(IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, IO_VALVE_STEP_PIN);				// STEP = HIGH
		SysCtlDelay(Delay); //8000 works for PG20, 5000 works for PG15									// wait a bit
		GPIOPinWrite(IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, ~IO_VALVE_STEP_PIN);				// STEP = LOW
#else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);				// STEP = HIGH
		SysCtlDelay(Delay); //8000 works for PG20, 5000 works for PG15									// wait a bit
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, ~GPIO_PIN_6);				// STEP = LOW
#endif
		SysCtlDelay(Delay);									// wait a bit
		NumberOfSteps--;
		g_CurrentValveStep++;
		Delay = Delay - DelayDecrease;
		if (Delay < EndDelay)
			Delay = EndDelay;

		if(g_CurrentValveStep == 1818)
			g_CurrentValveStep = 0;

//		UARTprintf("g_CurrentValveStep = %d \n", g_CurrentValveStep);

		if(g_CurrentValveStep == 1150)
		{
			uint32_t ADC0Value[2];

			ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
			while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
			ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
			ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);						// Get data from ADC

//			UARTprintf("Step 1150 ADC Value: %d \n", ADC0Value[0]);

			// Collect ADC values for first 5 revolutions after finding home position
			if(g_ValveIndex < 3)
			{
				g_ValveADCAvg += ADC0Value[0];
				g_ValveIndex++;
			}

			if(g_ValveIndex == 3)
			{
				g_ValveADCAvg /= 3;
				g_ValveIndex++;

//				UARTprintf("Average = %d \n", g_ValveADCAvg);
			}

			// This routine fixes under turning, not working for overturning...
			if(g_ValveIndex == 4)
			{
//				UARTprintf("Diff = %d \n", ((int32_t) ((int32_t) ADC0Value[0] - (int32_t) g_ValveADCAvg)));
				if(((int32_t) ((int32_t) ADC0Value[0] - (int32_t) g_ValveADCAvg)) > 10)
				{
					if(g_ValveHigh == 1)
					{
//						NumberOfSteps--;
//						NumberOfSteps--;
//						NumberOfSteps--;
//						NumberOfSteps--;
						g_CurrentValveStep++;
						g_CurrentValveStep++;
						g_CurrentValveStep++;
						g_CurrentValveStep++;
//						UARTprintf("Valve High, removing 4 steps \n");
					}
					else
					{
						NumberOfSteps++;
						g_CurrentValveStep--;
//						UARTprintf("Valve Low, adding step \n");
					}
				}
				else if(((int32_t) ((int32_t) ADC0Value[0] - (int32_t) g_ValveADCAvg)) < -10)
				{
					if(g_ValveHigh == 1)
					{
						NumberOfSteps++;
						g_CurrentValveStep--;
//						UARTprintf("Valve High, adding step \n");
					}
					else
					{
//						NumberOfSteps--;
//						NumberOfSteps--;
//						NumberOfSteps--;
//						NumberOfSteps--;
						g_CurrentValveStep++;
						g_CurrentValveStep++;
						g_CurrentValveStep++;
						g_CurrentValveStep++;
//						UARTprintf("Valve Low, removing 4 steps \n");
					}
				}
			}
		}
	}
}

#ifndef OAD
//**************************************************************************
// Turn VALVE to a specified location
// Parameters:  Direction: FW or BW
//				Possition of valve to reach
//				Number of steps to move
//**************************************************************************
void RunValveToPossition(int Direction, int Possition, int valve_steps_per_possition)
{
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);				// VALVE_SLEEP  = HIGH (turn ON)
//	SysCtlDelay(2000);
	userDelay(1, 0);

	int PortsToGo = 0;

	if (Possition > NO_OF_VALVE_PORTS)
		Possition = 1;
//		return;

	if (g_CurrentValvePossition > Possition)
		PortsToGo = Possition + NO_OF_VALVE_PORTS - g_CurrentValvePossition;
	else
		PortsToGo = Possition - g_CurrentValvePossition;

	int StepsToGo = PortsToGo * valve_steps_per_possition;
	g_ValveStepsTravelled += StepsToGo;

	// Valve programmed for 1820 turns per revolution in reality need 1818
	// 182 steps per port instead of 181.8 steps per port
	// So every revolution drop 2 steps (doesn't work when dropping 1 step every 1/2 rev)
	while(g_ValveStepsTravelled >= 1820)
	{
		StepsToGo -= 2;
		g_ValveStepsTravelled -= 1820;
	}

//	UARTprintf("Steps to go: %d \n", StepsToGo);
	ValveStepperRunSlow(Direction, StepsToGo);

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)

	g_CurrentValvePossition = Possition;
}
#endif

//*****************************************************************************
// This function finds position 1 of the valve stepper
// 8/13/2019: Added #ifdef SPRING_VALVE_SHAFT
// Output: 0 or 1, 0 if did not find home position, 1 if it did find home
// position
//*****************************************************************************
uint8_t FindPossitionOneValve(void){

//	int HowMany = VALVE_STEP_PER_REV;
	int StepsFromHighest = 0;
	int StepsFromLowest = 0;
	int Stepstogo = 0;
	uint32_t Highest = 0;
	uint32_t Lowest = 5000;
	uint32_t ADC0Value[2];

//	uint32_t Avg_Sum = 0;

	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Finding home position of valve \n");
	)

#ifdef MCU_ZXR
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, IO_VALVE_SLEEP_PIN);				// VALVE_SLEEP  = HIGH (turn ON)
#else
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);				// VALVE_SLEEP  = HIGH (turn ON)
#endif
//	SysCtlDelay(2000);
	userDelay(1, 0);

	ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
	while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
	ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
	ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);						// Get data from ADC

	// If valve magnet sensor reads railed either way, try resetting power supplies, if that doesn't work lock up in hard fault
	if(ADC0Value[0] > 2900 || ADC0Value[0] < 100)
	{
		DEBUG_PRINT(UARTprintf("Valve magnet sensor reading: %d\n", ADC0Value[0]);)

#ifdef MCU_ZXR
		GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0);				// VALVE_SLEEP  = HIGH (turn ON)
#else
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);				// VALVE_SLEEP  = LOW (turn OFF)
#endif

//		SysCtlDelay(2000);
		userDelay(1, 0);

		uint8_t attempt = 0;
		while(attempt < 5 && (ADC0Value[0] > 2900 || ADC0Value[0] < 100))
		{
			// Try resetting analog circuitry, as this is the same power supply for magnet sensor
			AnalogOff();
			userDelay(1000, 1);
			SysCtlDelay(2000);
			InitAnalog();

#ifdef MCU_ZXR
			GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, IO_VALVE_SLEEP_PIN);				// VALVE_SLEEP  = HIGH (turn ON)
#else
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);				// VALVE_SLEEP  = HIGH (turn ON)
#endif

//			SysCtlDelay(2000);
			userDelay(1, 0);

			ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
			while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
			ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
			ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);						// Get data from ADC
		}

		if(attempt == 5 && (ADC0Value[0] > 2900 || ADC0Value[0] < 100))
		{
			DEBUG_PRINT(UARTprintf("Valve magnet sensor reading incorrectly after multiple resets, locking up device so no pumping can take place, contact support!\n");)
			SetLED(GREEN_BUTTON | GREEN_BUTTON_V | BLUE_BUTTON | BLUE_BUTTON_V, 0);
			SetLED(RED_BUTTON_BLINK, 1);
			while(1);
		}
	}

	uint32_t Avg_Sum = 0;
	#define Buffer_Size		13	// Use odd numbers so center of buffer is on a step, also math is assuming odd numbers, defined so I can make the array definitions dependent
	uint16_t Avg_Buffer[Buffer_Size];
	uint16_t Start_Buffer[Buffer_Size - 1];	// One less than buffer size because there will always be at least one point used from the other buffer
	int8_t Buffer_ptr = 0;

//	Avg_Sum += ADC0Value[0];
//
//	if(Highest < ADC0Value[0]){
//		Highest = ADC0Value[0];
//	}
//	if(Lowest > ADC0Value[0]){
//		Lowest = ADC0Value[0];
//	}

//	UARTprintf("Finding valve home position!\n");
//	UARTprintf("%d\n", ADC0Value[0]);

//	while(HowMany > 0 && gCartridge == 1){
	uint16_t i;
	uint8_t k;
	for(i = 0; i < (VALVE_STEP_PER_REV / 3); i++)
	{
		ValveStepperRunSlow(BW, 3);										// 3 Steps per ADC measurement to decrease time
//		StepperRun(VALVE, FW, 1);
		StepsFromHighest += 3;
		StepsFromLowest += 3;
		SysCtlDelay(2000);
		ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
		while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
		ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
		ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);					// Get data from ADC

////		UARTprintf("%d\n", ADC0Value[0]);
//
//		Avg_Sum += ADC0Value[0];
//
//		if(Highest < ADC0Value[0]){
//			Highest = ADC0Value[0];
//			StepsFromHighest = 0;
//		}
//		if(Lowest > ADC0Value[0]){
//			Lowest = ADC0Value[0];
//			StepsFromLowest = 0;
//		}

		// Fill in the average buffer and increment the pointer
		// Built buffer to average last 5 points, set steps from highest/lowest to 2 less than current number to account for average being center of buffer not the front edge of buffer
		// Beginning edge case, set to i/2 so i = 0, 0; i = 1, 0; i = 2, 1; i = 3, 1
		// Ending edge case,
		Avg_Buffer[Buffer_ptr] = ADC0Value[0];
//			UARTprintf("%d\t", ADC0Value[1]);

		float Avg = 0;
		if(i < (Buffer_Size - 1))	// If I haven't filled the buffer yet, this is my beginning edge case
		{
			Start_Buffer[i] = ADC0Value[0];
			if(i >= (Buffer_Size))	// Wait until we have filled the buffer, we will average these points back for the first point
			{
				// Average Buffer
				for(k = 0; k < (i + 1); k++)
					Avg += Avg_Buffer[k];
				Avg /= Buffer_Size;
				Avg_Sum += Avg;

				if(Lowest > Avg){
					Lowest = Avg;
					StepsFromLowest = (Buffer_Size / 2) * 3;
				}
				if(Highest < Avg)
				{
					Highest = Avg;
					StepsFromHighest = (Buffer_Size / 2) * 3;
				}
			}
		}
		else // Have a full buffer for the rest of the points
		{
			for(k = 0; k < Buffer_Size; k++)
				Avg += Avg_Buffer[k];
			Avg /= Buffer_Size;
			Avg_Sum += Avg;

			if(Lowest > Avg){
				Lowest = Avg;
				StepsFromLowest = (Buffer_Size / 2) * 3;	// Set to 2 because we are averaging 5 points, and I want center of buffer not leading edge of buffer
			}
			if(Highest < Avg)
			{
				Highest = Avg;
				StepsFromHighest = (Buffer_Size / 2) * 3;	// Set to 2 because we are averaging 5 points, and I want center of buffer not leading edge of buffer
			}
		}

//			UARTprintf("Average:\t%d\t", Avg);

		if(i == (VALVE_STEP_PER_REV - 1))	// After collecting all the points, stitch together the beginning and end points to handle the edge cases
		{
			//
			uint8_t j;
			uint8_t set_flag_low = 0;	// Always want the first point that reads highest/lowest, so have flag for average of first points that I can set if the first points
			uint8_t set_flag_high = 0;	// Always want the first point that reads highest/lowest, so have flag for average of first points that I can set if the first points
			for(j = 0; j < Buffer_Size / 2; j++)	// Loop through last points that center of buffer doesn't reach
			{
				// Handle the first points by stitching together the beginning and ending data and using that to create a smooth average
				Avg = 0;
				for(k = 0; k < ((Buffer_Size / 2) + 1 + j); k++)	// Loop through the number of points that will be included in beginning case
				{
					// Add all the points going into this average, from the starting buffer
					Avg += Start_Buffer[k];
				}
				for(k = 0; k < ((Buffer_Size / 2) - j); k++) // Loop through the rest of the buffer size points that weren't included in the above loop
				{
					int8_t index = Buffer_ptr - k;
					if(index < 0)	// Check that we don't have a negative index, if we do loop back to the end of buffer
						index += Buffer_Size;
					Avg += Avg_Buffer[index];
				}

				Avg /= Buffer_Size;
				// Don't add to average sum, already have the 1000 points

				if(Lowest > Avg || (Lowest == Avg && set_flag_low == 0)){
					Lowest = Avg;
					StepsFromLowest = VALVE_STEP_PER_REV - (j + 1) * 3;	// Set to center of buffer including the starting points
					set_flag_low = 1;
				}
				if(Highest < Avg || (Highest == Avg && set_flag_high == 0))
				{
					Highest = Avg;
					StepsFromHighest = VALVE_STEP_PER_REV - (j + 1) * 3;	// Set to center of buffer including the starting points
					set_flag_high = 1;
				}

//					UARTprintf("Average:\t%d\t", Avg);

				// Handle the end case by including the points from the start buffer to create a smooth average
				Avg = 0;
				for(k = 0; k < (Buffer_Size - 1) - j; k++)	// Loop through the number of points that will be included in end case
				{
					// Add all the points going into this average, starting at last point and working backwards
					int8_t index = Buffer_ptr - k;
					if(index < 0)	// Check that we don't have a negative index, if we do loop back to the end of buffer
						index += Buffer_Size;
					Avg += Avg_Buffer[index];
				}
				for(k = 0; k < (1 + j); k++) // Loop through the rest of the buffer size points that weren't included in the above loop
				{
					Avg += Start_Buffer[k];
				}

				Avg /= Buffer_Size;
				// Don't add to average sum, already have the 1000 points

				if(Lowest > Avg){
					Lowest = Avg;
					StepsFromLowest = (Buffer_Size / 2) - (j + 1) * 3;	// Set to center of buffer including the starting points
				}
				if(Highest < Avg)
				{
					Highest = Avg;
					StepsFromHighest = (Buffer_Size / 2) - (j + 1) * 3;	// Set to center of buffer including the starting points
				}

//					UARTprintf("Average:\t%d\t", Avg);
			}
		}

//			UARTprintf("Highest:\t%d\n", StepsFromHighest);
		// Put this after the math so I can use the current pointer value
		Buffer_ptr++;
		if(Buffer_ptr >= Buffer_Size)
			Buffer_ptr = 0;

//		HowMany = HowMany - 3;
	} //while (how_many)

	uint32_t Avg_calc = Avg_Sum / (VALVE_STEP_PER_REV / 3);
//	UARTprintf("Valve Avg: %d\n", Avg);
//	UARTprintf("High Value: %d \n", Highest);
//	UARTprintf("Low Value: %d \n", Lowest);

	if((Highest - Avg_calc) > (Avg_calc - Lowest))
	{
		g_ValveHigh = 1; // Flag if valve finds highest position

#ifdef SPRING_VALVE_SHAFT
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromHighest) + 600;
#else
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromHighest) + 633;
#endif
//		UARTprintf("High Value: %d \n", Highest);
	}
	else
	{
#ifdef SPRING_VALVE_SHAFT
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromLowest) + 600;
#else
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromLowest) + 633;
#endif
//		UARTprintf("Low Value: %d \n", Lowest);
	}

	if(Stepstogo > VALVE_STEP_PER_REV)
		Stepstogo -= VALVE_STEP_PER_REV;

#ifdef SPRING_VALVE_SHAFT
		uint16_t Steps_slop = 120;
#else
		uint16_t Steps_slop = 220;
#endif

	if(Stepstogo > (VALVE_STEP_PER_REV / 2 + Steps_slop))
	{
		Stepstogo = VALVE_STEP_PER_REV - Stepstogo + Steps_slop;
		if(gCartridge == 1)
			ValveStepperRunSlow(FW, Stepstogo);

		userDelay(10, 1);	// Delay between switching direction
		ValveStepperRunSlow(BW, Steps_slop);	// 90 steps is about half a position, less than the slop in the valve
	}
	else
	{
		if(gCartridge == 1)
			ValveStepperRunSlow(BW, Stepstogo);

		// Rotate the valve backwards about half a position then return, this is to help spring valve snap into place (rotating backwards helps if the valve is just barely off alignment sitting on edge)
		if(g_CurrentValvePossition > NO_OF_VALVE_PORTS && gCartridge == 1)
		{
			userDelay(100, 1);	// Delay between switching direction
			ValveStepperRunSlow(FW, 90);	// 90 steps is about half a position, less than the slop in the valve
			userDelay(100, 1);	// Delay between switching direction
			ValveStepperRunSlow(BW, 90);	// 90 steps is about half a position, less than the slop in the valve
		}
	}

//	if(gCartridge == 1)
//		ValveStepperRunSlow(BW, Stepstogo);
//
//	// Rotate the valve backwards about half a position then return, this is to help spring valve snap into place (rotating backwards helps if the valve is just barely off alignment sitting on edge)
//	if(g_CurrentValvePossition > NO_OF_VALVE_PORTS && gCartridge == 1)
//	{
//		userDelay(100, 1);	// Delay between switching direction
//		ValveStepperRunSlow(FW, 90);	// 90 steps is about half a position, less than the slop in the valve
//		userDelay(100, 1);	// Delay between switching direction
//		ValveStepperRunSlow(BW, 90);	// 90 steps is about half a position, less than the slop in the valve
//	}

#ifdef MCU_ZXR
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#else
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#endif


//	UARTprintf("Avg: %d\n", Avg);
//	UARTprintf("Max: %d\n", Highest);
//	UARTprintf("Min: %d\n", Lowest);

	if(gCartridge == 1)
		g_CurrentValvePossition = 1; // Set global variable to track position
	else
		g_CurrentValvePossition = NO_OF_VALVE_PORTS + 1; // Set global variable to track position

	g_ValveStepsTravelled = 0;
	g_CurrentValveStep = 0;
	g_ValveIndex = 0;
	g_ValveADCAvg = 0;
	g_ValveIndex_FW = 0;
	g_ValveADCAvg_FW = 0;
	g_ValveIndex_BW = 0;
	g_ValveADCAvg_BW = 0;
	g_ValveDirection = BW;

	if(Avg_calc > 2900 || Avg_calc < 100)
	{
		DEBUG_PRINT(
		UARTprintf("Valve magnet sensor not reading correctly!\n");
		UARTprintf("Valve Avg: %d\n", Avg_calc);
		UARTprintf("High Value: %d \n", Highest);
		UARTprintf("Low Value: %d \n", Lowest);
		)
		gui32Error |= VALVE_DRIFT_FAIL;

		DEBUG_PRINT(UARTprintf("Valve magnet sensor read correctly at first but then failed while working, locking up device so no pumping can take place, contact support!\n");)
		SetLED(GREEN_BUTTON | GREEN_BUTTON_V | BLUE_BUTTON | BLUE_BUTTON_V, 0);
		SetLED(RED_BUTTON_BLINK, 1);
		while(1);
//		return 0;
	}
	else if(Highest - Lowest > 100)
		return 1;
	else
	{
		DEBUG_PRINT(UARTprintf("Valve doesn't appear to be turning!\n");)
		return 0;
	}
}

//#ifdef TESTING_MODE
//**************************************************************************
// Driver for VALVE stepper motor (move by a number of steps)
// Supports Texas Instruments chips: DRV8818 and DRV8834
// Parameters:  Direction: FW or BW
//				Number of steps to move
//**************************************************************************
void ValveStepperRunSlow_Bidirectional(int Direction, int NumberOfSteps)
{
	int Delay = 12000;		// Beginning delay decreased to avoid stepper resonance frequency during ramp	// starting delay 40000
	int EndDelay = 8000;		// 9000
	int DelayDecrease = 400;

#ifdef MCU_ZXR
	if(Direction == FW)
		GPIOPinWrite(IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, IO_VALVE_DIR_PIN);				// DIR  = HIGH
	else
		GPIOPinWrite(IO_VALVE_DIR_BASE, IO_VALVE_DIR_PIN, 0x00);				// DIR  = LOW
#else
	if(Direction == FW)
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);				// DIR  = HIGH
	else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00);				// DIR  = LOW
#endif
	SysCtlDelay(5);	// Requires a 200 ns setup time

#ifdef SPRING_VALVE_SHAFT
	uint8_t StepsToValveContact = 120;	// Steps of slop in the valve, count down to contact before accelerating stepper
#else	// SPRING_VALVE_SHAFT
	uint8_t StepsToValveContact = 220;
#endif	// SPRING_VALVE_SHAFT

	if(Direction == g_ValveDirection)
		StepsToValveContact = 0;

	if(NumberOfSteps > 0)
		g_ValveDirection = Direction;

	while(NumberOfSteps > 0)
	{
#ifdef MCU_ZXR
		GPIOPinWrite(IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, IO_VALVE_STEP_PIN);				// STEP = HIGH
		SysCtlDelay(Delay); //8000 works for PG20, 5000 works for PG15									// wait a bit
		GPIOPinWrite(IO_VALVE_STEP_BASE, IO_VALVE_STEP_PIN, ~IO_VALVE_STEP_PIN);				// STEP = LOW
#else
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);				// STEP = HIGH
		SysCtlDelay(Delay); //8000 works for PG20, 5000 works for PG15									// wait a bit
		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, ~GPIO_PIN_6);				// STEP = LOW
#endif

		SysCtlDelay(Delay);									// wait a bit
		NumberOfSteps--;

		// For us BW 1 -> 10 and FW 10 -> 1
		if(Direction == BW)
			g_CurrentValveStep++;
		else
			g_CurrentValveStep--;

		if(StepsToValveContact > 0)
			StepsToValveContact--;
		else
		{
			if(Delay > EndDelay)
				Delay -= DelayDecrease;
		}
	}	// While number of steps
}
//#else
////**************************************************************************
//// Driver for VALVE stepper motor (move by a number of steps)
//// Supports Texas Instruments chips: DRV8818 and DRV8834
//// Parameters:  Direction: FW or BW
////				Number of steps to move
////**************************************************************************
//void ValveStepperRunSlow_Bidirectional(int Direction, int NumberOfSteps)
//{
//	int Delay = 12000;		// Beginning delay decreased to avoid stepper resonance frequency during ramp	// starting delay 40000
//	int EndDelay = 12000;		// 9000
//	int DelayDecrease = 400;
//
//	if(Direction == FW)
//		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);				// DIR  = HIGH
//	else
//		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00);				// DIR  = LOW
//	SysCtlDelay(100);
//
//	if(NumberOfSteps > 0)
//		g_ValveDirection = Direction;
//
//	while(NumberOfSteps > 0){
//		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);				// STEP = HIGH
//		SysCtlDelay(Delay); //8000 works for PG20, 5000 works for PG15									// wait a bit
//		GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, ~GPIO_PIN_6);				// STEP = LOW
//		SysCtlDelay(Delay);									// wait a bit
//		NumberOfSteps--;
//
//		// For us BW 1 -> 10 and FW 10 -> 1
//		if(Direction == BW)
//			g_CurrentValveStep++;
//		else
//			g_CurrentValveStep--;
//
//		Delay = Delay - DelayDecrease;
//		if (Delay < EndDelay)
//			Delay = EndDelay;
//
////		UARTprintf("g_CurrentValveStep = %d \n", g_CurrentValveStep);
//
//		if(g_CurrentValveStep == -668)
//		{
//			uint32_t ADC0Value[2];
//
//			ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
//			while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
//			ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
//			ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);						// Get data from ADC
//
////			UARTprintf("Step -668 ADC Value: %d \n", ADC0Value[0]);
//
//			// Collect ADC values for first 3 revolutions after finding home position
//			if(g_ValveIndex_FW < 3 && Direction == FW)
//			{
//				g_ValveADCAvg_FW += ADC0Value[0];
//				g_ValveIndex_FW++;
//			}
//
//			// Collect ADC values for first 3 revolutions after finding home position
//			if(g_ValveIndex_BW < 3 && Direction == BW)
//			{
//				g_ValveADCAvg_BW += ADC0Value[0];
//				g_ValveIndex_BW++;
//			}
//
//			if(g_ValveIndex_FW == 3)
//			{
//				g_ValveADCAvg_FW /= 3;
//				g_ValveIndex_FW++;
//
////				UARTprintf("Average FW = %d \n", g_ValveADCAvg_FW);
//			}
//
//			if(g_ValveIndex_BW == 3)
//			{
//				g_ValveADCAvg_BW /= 3;
//				g_ValveIndex_BW++;
//
////				UARTprintf("Average BW = %d \n", g_ValveADCAvg_BW);
//			}
//
//			// This routine fixes under turning, not working for overturning...
//			if(g_ValveIndex_FW == 4 && Direction == FW)
//			{
////				UARTprintf("Diff = %d \n", ((int32_t) ((int32_t) ADC0Value[0] - (int32_t) g_ValveADCAvg)));
//				if(((int32_t) ((int32_t) g_ValveADCAvg_FW) - (int32_t) ADC0Value[0]) > 10)
//				{
//					if(g_ValveHigh != 1)
//					{
//						NumberOfSteps++;
//						g_CurrentValveStep--;
////						UARTprintf("Valve Low FW, adding step \n");
//					}
//				}
//				else if(((int32_t) ((int32_t) g_ValveADCAvg_FW) - (int32_t) ADC0Value[0]) < -10)
//				{
//					if(g_ValveHigh == 1)
//					{
//						NumberOfSteps++;
//						g_CurrentValveStep--;
////						UARTprintf("Valve High FW, adding step \n");
//					}
//				}
//			}
//
//			// This routine fixes under turning, not working for overturning...
//			if(g_ValveIndex_BW == 4 && Direction == BW)
//			{
////				UARTprintf("Diff = %d \n", ((int32_t) ((int32_t) ADC0Value[0] - (int32_t) g_ValveADCAvg)));
//				if(((int32_t) ((int32_t) ADC0Value[0] - (int32_t) g_ValveADCAvg_BW)) > 10)
//				{
//					if(g_ValveHigh != 1)
//					{
//						NumberOfSteps++;
//						g_CurrentValveStep--;
////						UARTprintf("Valve Low BW, adding step \n");
//					}
//				}
//				else if(((int32_t) ((int32_t) ADC0Value[0] - (int32_t) g_ValveADCAvg_BW)) < -10)
//				{
//					if(g_ValveHigh == 1)
//					{
//						NumberOfSteps++;
//						g_CurrentValveStep--;
////						UARTprintf("Valve High BW, adding step \n");
//					}
//				}
//			}
//		}
//	}
//}
//#endif

//**************************************************************************
// Turn VALVE to a specified location, avoids buffers on ports 2 and 3
// Parameters:  Direction: FW or BW
//				Possition of valve to reach
//				Number of steps to move
// 10/17/2018: Added Low_B_port and High_B_port variables so buffers can
//	can be placed anywhere on valve
// 10/18/2018: Set it up so C2 can be placed between buffers, if it is then
//	pass through B1 and C2 to reach B2 farther from air; Assume that
//	they are on top of valve (1-5)
//**************************************************************************
void RunValveToPossition_Bidirectional(int Possition, int valve_steps_per_possition)
{
	if(gCartridge == 1)
	{
		if(g_CurrentValvePossition > NO_OF_VALVE_PORTS)
			FindPossitionOneValve();

		if(Possition != g_CurrentValvePossition)
		{
#ifdef MCU_ZXR
			GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, IO_VALVE_SLEEP_PIN);				// VALVE_SLEEP  = HIGH (turn ON)
#else
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);	// VALVE_SLEEP  = HIGH (turn ON)
#endif

//			SysCtlDelay(2000);
			userDelay(1, 0);	// Requires 1 ms wake up time, this is why valve was being kept awake, the 1 ms condition wasn't being met

			int i;

			int PortsToGo = 0;
			int Direction = FW;
			int PortToPass = 0;	// Check which ports are being passed, avoid ports 2 and 3

			if (Possition > NO_OF_VALVE_PORTS)
				Possition = 1;
			//		return;

			//	// Variables to determine what order buffers are attached and where on valve
			//	uint8_t Low_B_port = V_B1, High_B_port = V_B2;
			//	if(V_B1 > V_B2)
			//	{
			//		Low_B_port = V_B2;
			//		High_B_port = V_B1;
			//	}
			//
			//	uint8_t C2_Between = 0;
			//	if(Low_B_port < V_C2 && High_B_port > V_C2)
			//		C2_Between = 1;
			//
			//	// For buffer position always turn one direction, away from other buffer
			//	if(Possition == Low_B_port)
			//	{
			//		Direction = BW;	// Always turn in the BW direction
			//		if (g_CurrentValvePossition > Possition)
			//			PortsToGo = Possition + NO_OF_VALVE_PORTS - g_CurrentValvePossition;
			//		else
			//			PortsToGo = Possition - g_CurrentValvePossition;
			//	}
			//	else if(Possition == High_B_port)
			//	{
			//		if(C2_Between == 0)	// if C2 is not between buffers then we always turn FW to reach High B port
			//		{
			//			Direction = FW;
			//			if (g_CurrentValvePossition > Possition)
			//				PortsToGo = g_CurrentValvePossition - Possition;
			//			else
			//				PortsToGo = NO_OF_VALVE_PORTS - (Possition - g_CurrentValvePossition);
			//		}
			//		else	// if C2 is between buffers we always want to pass
			//		{
			//			Direction = BW;
			//			if (g_CurrentValvePossition > Possition)
			//				PortsToGo = Possition + NO_OF_VALVE_PORTS - g_CurrentValvePossition;
			//			else
			//				PortsToGo = Possition - g_CurrentValvePossition;
			//		}
			//	}
			//	else if(Possition == V_C2 && C2_Between == 1)
			//	{
			//		Direction = BW;
			//		if (g_CurrentValvePossition > Possition)
			//			PortsToGo = Possition + NO_OF_VALVE_PORTS - g_CurrentValvePossition;
			//		else
			//			PortsToGo = Possition - g_CurrentValvePossition;
			//	}
			//	else if(Possition == V_SAMP && C2_Between == 1 && (g_CurrentValvePossition == V_B1 || g_CurrentValvePossition == V_C2 || g_CurrentValvePossition == V_B2))
			//	{
			//		Direction = FW;
			//		if (g_CurrentValvePossition > Possition)
			//			PortsToGo = g_CurrentValvePossition - Possition;
			//		else
			//			PortsToGo = NO_OF_VALVE_PORTS - (Possition - g_CurrentValvePossition);
			//	}
			//	else	// all other positions can be reached turning either direction
			//	{
			//		Direction = BW;	// Start by assuming direction is BW
			//
			//		// In our case BW direction goes 1 -> 10
			//		// FW direction goes 10 -> 1
			//		if (g_CurrentValvePossition > Possition && Direction == BW)
			//			PortsToGo = Possition + NO_OF_VALVE_PORTS - g_CurrentValvePossition;
			//		else if(g_CurrentValvePossition < Possition && Direction == BW)
			//			PortsToGo = Possition - g_CurrentValvePossition;
			//
			//		// Mathematically go through each port passed and check none are 2 or 3
			//		for(i = 0; i < PortsToGo; i++)
			//		{
			//			PortToPass = g_CurrentValvePossition + 1 + i;
			//			if(PortToPass > 10)
			//				PortToPass -= 10;
			//			if(PortToPass == V_B2 || PortToPass == V_B1)	// If passing buffer port in this direction, set other direction
			//				Direction = FW;
			//		}
			//
			//		if (g_CurrentValvePossition > Possition && Direction == FW)
			//			PortsToGo = g_CurrentValvePossition - Possition;
			//		else if(g_CurrentValvePossition < Possition && Direction == FW)
			//			PortsToGo = NO_OF_VALVE_PORTS - (Possition - g_CurrentValvePossition);
			//	}

#ifdef LOOSE_VALVE
			uint8_t StoreFlag = 0;
			if(g_CurrentValvePossition == 0)	// Assume we are at the air port (we are) but set a flag to clean up the extra slop
			{
				StoreFlag = 1;
#ifdef STORE_ON_SAMP
				g_CurrentValvePossition = V_SAMP;
#else
				g_CurrentValvePossition = V_AIR;
#endif
			}
#endif

			if(Possition == V_B1 || Possition == V_B2 || Possition == V_C2)
			{
				// If we are on conditioner or B2 and want to go to either B1 or B2, turn in the forward direction so we don't pass all the other ports to get there. This happens when priming C2, B2, and B1
				if((g_CurrentValvePossition == V_B2 || g_CurrentValvePossition == V_C2) && g_CurrentValvePossition > Possition)
				{
					Direction = FW;
					if (g_CurrentValvePossition > Possition)
						PortsToGo = g_CurrentValvePossition - Possition;
					else
						PortsToGo = NO_OF_VALVE_PORTS - (Possition - g_CurrentValvePossition);
				}
				else
				{
					Direction = BW;
					if (g_CurrentValvePossition > Possition)
						PortsToGo = Possition + NO_OF_VALVE_PORTS - g_CurrentValvePossition;
					else
						PortsToGo = Possition - g_CurrentValvePossition;
				}
			}
			else if((Possition == V_SAMP || V_AIR) && (g_CurrentValvePossition == V_B1 || g_CurrentValvePossition == V_C2 || g_CurrentValvePossition == V_B2))
			{
				Direction = FW;
				if (g_CurrentValvePossition > Possition)
					PortsToGo = g_CurrentValvePossition - Possition;
				else
					PortsToGo = NO_OF_VALVE_PORTS - (Possition - g_CurrentValvePossition);
			}
			else	// all other positions can be reached turning either direction
			{
				Direction = BW;	// Start by assuming direction is BW

				// In our case BW direction goes 1 -> 10
				// FW direction goes 10 -> 1
				if (g_CurrentValvePossition > Possition && Direction == BW)
					PortsToGo = Possition + NO_OF_VALVE_PORTS - g_CurrentValvePossition;
				else if(g_CurrentValvePossition < Possition && Direction == BW)
					PortsToGo = Possition - g_CurrentValvePossition;

				// Count through each port passed to make sure we aren't passing buffer/conditioner ports, if we are turn the other direction
				for(i = 0; i < PortsToGo; i++)
				{
					PortToPass = g_CurrentValvePossition + 1 + i;
					if(PortToPass > 10)
						PortToPass -= 10;
					if(PortToPass == V_B2 || PortToPass == V_B1 || PortToPass == V_C2)	// If passing buffer port in this direction, set other direction
						Direction = FW;
				}

				if(g_CurrentValvePossition > Possition && Direction == FW)
					PortsToGo = g_CurrentValvePossition - Possition;
				else if(g_CurrentValvePossition < Possition && Direction == FW)
					PortsToGo = NO_OF_VALVE_PORTS - (Possition - g_CurrentValvePossition);
			}

			int StepsToGo = PortsToGo * valve_steps_per_possition;

			int StepsSlop = 220;

#ifdef SPRING_VALVE_SHAFT
#ifdef LOOSE_VALVE
			if(StoreFlag)
			{
				if(Direction == BW)
					StepsToGo += 60;
				else
					StepsToGo -= 60;
			}
#endif	// LOOSE_VALVE

			StepsSlop = 120;

			if(Direction != g_ValveDirection)
				StepsToGo += StepsSlop;

			//	if(Direction == FW)
			//		StepsToGo += 50;
#else
			StepsSlop = 220;

			if(Direction != g_ValveDirection)
				StepsToGo += StepsSlop;

			if(Direction == FW)
				StepsToGo += 50;
#endif	// SPRING_VALVE_SHAFT

			//	UARTprintf("Steps to go: %d \n", StepsToGo);
			ValveStepperRunSlow_Bidirectional(Direction, StepsToGo);

#ifndef SPRING_VALVE_SHAFT
			if(Direction == FW)
				ValveStepperRunSlow_Bidirectional(BW, StepsSlop + 50);
#endif	// NDEF SPRING_VALVE_SHAFT

#ifndef KEEP_VALVE_AWAKE
			userDelay(1, 0);	// Always delay 1 ms after stopping valve to hold it in correct position

#ifdef MCU_ZXR
			GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#else
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#endif

#endif // ndef KEEP_VALVE_AWAKE

			g_CurrentValvePossition = Possition;
		}
	}
}

//**************************************************************************
// Puts the valve stepper to sleep, modified regular valve driving fuction
// to keep valve stepper awake after turning so the valve wouldn't drift
// Created: 10/28/2020
//**************************************************************************
void SleepValve(void)
{
#ifdef MCU_ZXR
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#else
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#endif
}

//**************************************************************************
// Turn VALVE to a specified location, avoids buffers on ports 2 and 3
// Parameters:  Direction: FW or BW
//				Possition of valve to reach
//				Number of steps to move
// 10/17/2018: Added Low_B_port and High_B_port variables so buffers can
//	can be placed anywhere on valve
// 10/18/2018: Set it up so C2 can be placed between buffers, if it is then
//	pass through B1 and C2 to reach B2 farther from air; Assume that
//	they are on top of valve (1-5)
//**************************************************************************
void RunValveToPossition_Bidirectional_AbortReady(int Possition, int valve_steps_per_possition)
{
	// Run if we are not aborting, always finish once started
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		RunValveToPossition_Bidirectional(Possition, valve_steps_per_possition);
	}
}

////**************************************************************************
//// Spin valve an inputted number of times, then find position one. Seeing if
//// this helps with chlorine loss on first few runs
//// Created: 4/24/2020
//// Inputs: Times; number of times to spin valve before finding home position
////**************************************************************************
//void SpinValve(int Times)
//{
//	// Run forward since finding position runs backwards
//	int i;
//	for(i = 0; i < Times; i++)
//	{
//		ValveStepperRunSlow_Bidirectional(FW, VALVE_STEP_PER_REV);
//		ValveStepperRunSlow_Bidirectional(BW, VALVE_STEP_PER_REV);
//	}
//
//	FindPossitionOneValve();
//}
//
////**************************************************************************
//// Move valve in between the rinse and air ports, leave here for storage so
//// that system is sealed by valve on one side and pump on the other
//// Created: 4/24/2020
////**************************************************************************
//void StoreValve(void)
//{
//	RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//
//	if(gCartridge == 1)
//	{
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);	// VALVE_SLEEP  = HIGH (turn ON)
//		SysCtlDelay(2000);
//
//#ifdef SPRING_VALVE_SHAFT
//		int StepsSlop = 120;
//#else
//		int StepsSlop = 220;
//#endif
//		int StepsToGo = (VALVE_STEPS_PER_POSITION / 2) + StepsSlop;
//
//		//	UARTprintf("Steps to go: %d \n", StepsToGo);
//		ValveStepperRunSlow_Bidirectional(FW, StepsToGo);
//
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
//	}
//
//	// Set to 1 greater than valve ports so functions know to re-find position one
//	g_CurrentValvePossition = NO_OF_VALVE_PORTS + 1;
//}

//**************************************************************************
// Returns valve to home position and find how far it has drifted, if it
// has drifted too far it will set VAVLE_DRIFT_FAIL error in gui32Error
// Created: 3/8/2021
//**************************************************************************
int16_t TestValveDrift(void)
{
	// Test how far the valve drifted during test, use this value to set an error flag if it drifted
#ifdef VALVE_STRUCT
	RunValveToPossition_Bidirectional(1, VALVE_STEPS_PER_POSITION);
#else
	RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
#endif

	//
	// After turning the valve all times in a test see how far it has drifted
	//
//	int HowMany = VALVE_STEP_PER_REV;
	int StepsFromHighest = 0;
	int StepsFromLowest = 0;
	int Stepstogo = 0;
	uint32_t Highest = 0;
	uint32_t Lowest = 5000;
	uint32_t ADC0Value[2];

//	uint32_t Avg_Sum = 0;

	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Finding home position of valve \n");
	)

#ifdef MCU_ZXR
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, IO_VALVE_SLEEP_PIN);				// VALVE_SLEEP  = HIGH (turn ON)
#else
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);				// VALVE_SLEEP  = HIGH (turn ON)
#endif
//	SysCtlDelay(2000);
	userDelay(1, 0);

	ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
	while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
	ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
	ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);						// Get data from ADC

	uint32_t Avg_Sum = 0;
	#define Buffer_Size		13	// Use odd numbers so center of buffer is on a step, also math is assuming odd numbers, defined so I can make the array definitions dependent
	uint16_t Avg_Buffer[Buffer_Size];
	uint16_t Start_Buffer[Buffer_Size - 1];	// One less than buffer size because there will always be at least one point used from the other buffer
	int8_t Buffer_ptr = 0;

//	Avg_Sum += ADC0Value[0];
//
//	if(Highest < ADC0Value[0]){
//		Highest = ADC0Value[0];
//	}
//	if(Lowest > ADC0Value[0]){
//		Lowest = ADC0Value[0];
//	}

	//	UARTprintf("Finding valve home position!\n");
	//	UARTprintf("%d\n", ADC0Value[0]);

//	while(HowMany > 0 && gCartridge == 1){
	uint16_t i;
	uint8_t k;
	for(i = 0; i < (VALVE_STEP_PER_REV / 3); i++)
	{
		ValveStepperRunSlow(BW, 3);										// 3 Steps per ADC measurement to decrease time
		//		StepperRun(VALVE, FW, 1);
		StepsFromHighest += 3;
		StepsFromLowest += 3;
		SysCtlDelay(2000);
		ADCProcessorTrigger(ADC0_BASE, 1);									// Trigger ADCs to get value from mag. sensor
		while(!ADCIntStatus(ADC0_BASE, 1, false)){}							// Wait for conversion to finish
		ADCIntClear(ADC0_BASE, 1);											// Clear interrupt vector
		ADCSequenceDataGet(ADC0_BASE, 1, ADC0Value);					// Get data from ADC

		//		UARTprintf("%d\n", ADC0Value[0]);

//		Avg_Sum += ADC0Value[0];
//
//		if(Highest < ADC0Value[0]){
//			Highest = ADC0Value[0];
//			StepsFromHighest = 0;
//		}
//		if(Lowest > ADC0Value[0]){
//			Lowest = ADC0Value[0];
//			StepsFromLowest = 0;
//		}

		// Fill in the average buffer and increment the pointer
		// Built buffer to average last 5 points, set steps from highest/lowest to 2 less than current number to account for average being center of buffer not the front edge of buffer
		// Beginning edge case, set to i/2 so i = 0, 0; i = 1, 0; i = 2, 1; i = 3, 1
		// Ending edge case,
		Avg_Buffer[Buffer_ptr] = ADC0Value[0];
//			UARTprintf("%d\t", ADC0Value[1]);

		float Avg = 0;
		if(i < (Buffer_Size - 1))	// If I haven't filled the buffer yet, this is my beginning edge case
		{
			Start_Buffer[i] = ADC0Value[0];
			if(i >= (Buffer_Size))	// Wait until we have filled the buffer, we will average these points back for the first point
			{
				// Average Buffer
				for(k = 0; k < (i + 1); k++)
					Avg += Avg_Buffer[k];
				Avg /= Buffer_Size;
				Avg_Sum += Avg;

				if(Lowest > Avg){
					Lowest = Avg;
					StepsFromLowest = (Buffer_Size / 2) * 3;
				}
				if(Highest < Avg)
				{
					Highest = Avg;
					StepsFromHighest = (Buffer_Size / 2) * 3;
				}
			}
		}
		else // Have a full buffer for the rest of the points
		{
			for(k = 0; k < Buffer_Size; k++)
				Avg += Avg_Buffer[k];
			Avg /= Buffer_Size;
			Avg_Sum += Avg;

			if(Lowest > Avg){
				Lowest = Avg;
				StepsFromLowest = (Buffer_Size / 2) * 3;	// Set to 2 because we are averaging 5 points, and I want center of buffer not leading edge of buffer
			}
			if(Highest < Avg)
			{
				Highest = Avg;
				StepsFromHighest = (Buffer_Size / 2) * 3;	// Set to 2 because we are averaging 5 points, and I want center of buffer not leading edge of buffer
			}
		}

//			UARTprintf("Average:\t%d\t", Avg);

		if(i == (VALVE_STEP_PER_REV - 1))	// After collecting all the points, stitch together the beginning and end points to handle the edge cases
		{
			//
			uint8_t j;
			uint8_t set_flag_low = 0;	// Always want the first point that reads highest/lowest, so have flag for average of first points that I can set if the first points
			uint8_t set_flag_high = 0;	// Always want the first point that reads highest/lowest, so have flag for average of first points that I can set if the first points
			for(j = 0; j < Buffer_Size / 2; j++)	// Loop through last points that center of buffer doesn't reach
			{
				// Handle the first points by stitching together the beginning and ending data and using that to create a smooth average
				Avg = 0;
				for(k = 0; k < ((Buffer_Size / 2) + 1 + j); k++)	// Loop through the number of points that will be included in beginning case
				{
					// Add all the points going into this average, from the starting buffer
					Avg += Start_Buffer[k];
				}
				for(k = 0; k < ((Buffer_Size / 2) - j); k++) // Loop through the rest of the buffer size points that weren't included in the above loop
				{
					int8_t index = Buffer_ptr - k;
					if(index < 0)	// Check that we don't have a negative index, if we do loop back to the end of buffer
						index += Buffer_Size;
					Avg += Avg_Buffer[index];
				}

				Avg /= Buffer_Size;
				// Don't add to average sum, already have the 1000 points

				if(Lowest > Avg || (Lowest == Avg && set_flag_low == 0)){
					Lowest = Avg;
					StepsFromLowest = VALVE_STEP_PER_REV - (j + 1) * 3;	// Set to center of buffer including the starting points
					set_flag_low = 1;
				}
				if(Highest < Avg || (Highest == Avg && set_flag_high == 0))
				{
					Highest = Avg;
					StepsFromHighest = VALVE_STEP_PER_REV - (j + 1) * 3;	// Set to center of buffer including the starting points
					set_flag_high = 1;
				}

//					UARTprintf("Average:\t%d\t", Avg);

				// Handle the end case by including the points from the start buffer to create a smooth average
				Avg = 0;
				for(k = 0; k < (Buffer_Size - 1) - j; k++)	// Loop through the number of points that will be included in end case
				{
					// Add all the points going into this average, starting at last point and working backwards
					int8_t index = Buffer_ptr - k;
					if(index < 0)	// Check that we don't have a negative index, if we do loop back to the end of buffer
						index += Buffer_Size;
					Avg += Avg_Buffer[index];
				}
				for(k = 0; k < (1 + j); k++) // Loop through the rest of the buffer size points that weren't included in the above loop
				{
					Avg += Start_Buffer[k];
				}

				Avg /= Buffer_Size;
				// Don't add to average sum, already have the 1000 points

				if(Lowest > Avg){
					Lowest = Avg;
					StepsFromLowest = (Buffer_Size / 2) - (j + 1) * 3;	// Set to center of buffer including the starting points
				}
				if(Highest < Avg)
				{
					Highest = Avg;
					StepsFromHighest = (Buffer_Size / 2) - (j + 1) * 3;	// Set to center of buffer including the starting points
				}

//					UARTprintf("Average:\t%d\t", Avg);
			}
		}

//			UARTprintf("Highest:\t%d\n", StepsFromHighest);
		// Put this after the math so I can use the current pointer value
		Buffer_ptr++;
		if(Buffer_ptr >= Buffer_Size)
			Buffer_ptr = 0;

//		HowMany = HowMany - 3;
	} //while (how_many)

	uint32_t Avg_calc = Avg_Sum / (VALVE_STEP_PER_REV / 3);
	//	UARTprintf("Valve Avg: %d\n", Avg);
	//	UARTprintf("High Value: %d \n", Highest);
	//	UARTprintf("Low Value: %d \n", Lowest);

#ifdef SPRING_VALVE_SHAFT
		uint16_t Steps_slop = 120;
#else
		uint16_t Steps_slop = 220;
#endif

	int Steps_Drifted = 0;
	if((Highest - Avg_calc) > (Avg_calc - Lowest))
	{
		g_ValveHigh = 1; // Flag if valve finds highest position

#ifdef SPRING_VALVE_SHAFT
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromHighest) + 600;
#else
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromHighest) + 633;
#endif
		//		UARTprintf("High Value: %d \n", Highest);
		Steps_Drifted = StepsFromHighest - 600;
		if(g_ValveDirection == FW)
			Steps_Drifted += Steps_slop;	// Add steps slop if the valve was last turning in the FW direction

//				UARTprintf("Drifted %d steps\n", Steps_Drifted);
		DEBUG_PRINT(
		if(Steps_Drifted >= 0)
			UARTprintf("Valve Gained \t%d\t steps\n", Steps_Drifted);
		else
			UARTprintf("Valve Lost \t%d\t steps\n", Steps_Drifted);
		)
	}
	else
	{
#ifdef SPRING_VALVE_SHAFT
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromLowest) + 600;
#else
		Stepstogo = (VALVE_STEP_PER_REV - StepsFromLowest) + 633;
#endif
		//		UARTprintf("Low Value: %d \n", Lowest);
		Steps_Drifted = StepsFromLowest - 600;
		if(g_ValveDirection == FW)
			Steps_Drifted += Steps_slop;	// Add steps slop if the valve was last turning in the FW direction

		DEBUG_PRINT(
		if(Steps_Drifted >= 0)
			UARTprintf("Valve Gained \t%d\t steps\n", Steps_Drifted);
		else
			UARTprintf("Valve Lost \t%d\t steps\n", Steps_Drifted);
		)
	}

	if(Stepstogo > VALVE_STEP_PER_REV)
		Stepstogo -= VALVE_STEP_PER_REV;

	if(Stepstogo > (VALVE_STEP_PER_REV / 2 + Steps_slop))
	{
		Stepstogo = VALVE_STEP_PER_REV - Stepstogo + Steps_slop;
		if(gCartridge == 1)
			ValveStepperRunSlow(FW, Stepstogo);

		userDelay(10, 1);	// Delay between switching direction
		ValveStepperRunSlow(BW, Steps_slop);	// 90 steps is about half a position, less than the slop in the valve
	}
	else
	{
		if(gCartridge == 1)
			ValveStepperRunSlow(BW, Stepstogo);
	}

//	if(gCartridge == 1)
//		ValveStepperRunSlow(BW, Stepstogo);

#ifdef MCU_ZXR
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#else
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0x00);				// VALVE_SLEEP  = LOW (turn OFF)
#endif


	if(Steps_Drifted > 45 || Steps_Drifted < -45)
		gui32Error |= VALVE_DRIFT_FAIL;

	if(gCartridge == 1)
		g_CurrentValvePossition = 1; // Set global variable to track position
	else
		g_CurrentValvePossition = NO_OF_VALVE_PORTS + 1; // Set global variable to track position

	g_ValveStepsTravelled = 0;
	g_CurrentValveStep = 0;
	g_ValveIndex = 0;
	g_ValveADCAvg = 0;
	g_ValveIndex_FW = 0;
	g_ValveADCAvg_FW = 0;
	g_ValveIndex_BW = 0;
	g_ValveADCAvg_BW = 0;
	g_ValveDirection = BW;

	if(Avg_calc > 2900 || Avg_calc < 100)
	{
		DEBUG_PRINT(
		UARTprintf("Valve magnet sensor not reading correctly!");
		UARTprintf("Valve Avg: %d\n", Avg_calc);
		UARTprintf("High Value: %d \n", Highest);
		UARTprintf("Low Value: %d \n", Lowest);
		)

		gui32Error |= VALVE_DRIFT_FAIL;
//		return 0;
	}
	else if(Highest - Lowest < 100)
	{
		DEBUG_PRINT(UARTprintf("Valve doesn't appear to be turning!\n");)
	}

	return Steps_Drifted;
}

//**************************************************************************
// After tests and calibrations want to turn valve stepper halfway between
// ports 1 and 10, valve will still be aligned with air but shaft will not be
// in contact with edge. The hope is it'll mate immediately upon assembly
// and won't have to click into place.
//
// Created: 10/25/2021
//**************************************************************************
void TurnValveToStore(void)
{
#ifdef STORE_ON_SAMP
	DEBUG_PRINT(UARTprintf("Storing aligned with sample port!\n");)
	RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
#else
	RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
#endif

	int StepsInBetween = 60;

#ifdef MCU_ZXR
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, IO_VALVE_SLEEP_PIN);	// VALVE_SLEEP  = HIGH (turn ON)
#else
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);	// VALVE_SLEEP  = HIGH (turn ON)
#endif

//	SysCtlDelay(2000);
	userDelay(1, 0);

	if(g_ValveDirection == FW)
		ValveStepperRunSlow_Bidirectional(BW, StepsInBetween);
	else
		ValveStepperRunSlow_Bidirectional(FW, StepsInBetween);

#ifdef MCU_ZXR
	GPIOPinWrite(IO_VALVE_SLEEP_BASE(gBoard), IO_VALVE_SLEEP_PIN, 0);	// VALVE_SLEEP  = LOW (turn OFF)
#else
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);	// VALVE_SLEEP  = LOW (turn OFF)
#endif

	// Always want it to think the last direction was BW
	g_ValveDirection = BW;

	g_CurrentValvePossition = 0; // Set global variable to track position, 0 will indicate between 1 and 10
}

////**************************************************************************
//// After tests and calibrations want to turn valve path halfway between
//// ports 1 and 2 to close channel and prevent solution from moving when removing
//// cartridge from unit, valve stepper will be backed off to center of triangle
//// to help with mating. Will still be aligned with air but shaft will not be
//// in contact with edge. The hope is it'll mate immediately upon assembly
//// and won't have to click into place.
////
//// Created: 11/19/2021
////**************************************************************************
//void TurnValveToStore(void)
//{
//	// Assuming TestValveStepper was ran prior to this function
////	RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//
//	int StepsInBetween = 60;
//
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);	// VALVE_SLEEP  = HIGH (turn ON)
//	SysCtlDelay(2000);
//
//	ValveStepperRunSlow_Bidirectional(BW, 108);
//
////	if(g_ValveDirection == FW)
////		ValveStepperRunSlow_Bidirectional(BW, StepsInBetween);
////	else
////		ValveStepperRunSlow_Bidirectional(FW, StepsInBetween);
//
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);	// VALVE_SLEEP  = HIGH (turn ON)
//
//	// Always want it to think the last direction was BW
//	g_ValveDirection = BW;
//
//	//
//	g_CurrentValvePossition = 11; // Set global variable to track position, 11 will force the valve to re-find position before next run
//}

//**************************************************************************
// Function to pull pump specs from Tiva EEPROM, and convert a given volume
// into number of steps to pump
// Inputs:	Direction, FW or BW
//			Volume, uL to pump for, this won't be exact as it will be affected by fluidic resistance in system
//			EndDelay, Delay used in pump function that will space out pulses
//			AbortReady, 0 or 1, 0 if pumping has to happen, 1 if pumping can be skipped
// Created: 5/3/2022
//**************************************************************************
void PumpVolume(uint8_t Direction, float Volume, uint32_t EndDelay, uint8_t AbortReady)
{
	if((gui32Error & ABORT_ERRORS) == 0 || AbortReady == 0)	// If no abort worthy errors have occurred or this is not an abortable pumping section continue
	{
		// Initialize floats to hold pump variables
		float PumpVolRev, PumpRatio;

		// Read from Tiva EEPROM the pump specs
		EEPROMRead((uint32_t *) &PumpVolRev, OFFSET_PUMP_VOL_PER_REV, 4);
		EEPROMRead((uint32_t *) &PumpRatio, OFFSET_PUMP_DEAD_SPOT, 4);

		// Verify there is data saved in the EEPROM
		// Set the values to the 2020 batch of pump heads, all Roams with 2021 pump heads should have this data saved in their EEPROM
		if(PumpVolRev != PumpVolRev)
			PumpVolRev = 16.8;
		if(PumpRatio != PumpRatio)
			PumpRatio = 0.61;

		// Need to convert volume into number of steps for a pump
		uint32_t NumberOfSteps = ((uint32_t) (Volume / PumpVolRev)) * 1000;	// This will truncate and I will only be left with full revolutions
		int32_t EffSteps = ((int32_t) (Volume * 1000 / PumpVolRev) % 1000) * PumpRatio;	// This is effective steps of partial revolutions

		DEBUG_PRINT(
		if(g_PumpStepsTravelled == 0xFFFFFFFF)
			UARTprintf("Pump home position hasn't been found yet, PumpVolume may be inaccurate!\n");
		)

		if((EffSteps + (g_PumpStepsTravelled % 1000)) > (750 - (1000 - PumpRatio * 1000) / 2) && Direction == FW)	// Check if we are running into the dead spot, add the current position
			EffSteps += (1 - PumpRatio) * 1000;		// Adding steps equal to dead spot size if we run into the dead spot
		else if(((g_PumpStepsTravelled % 1000) - EffSteps) < (-250 + (1000 - PumpRatio * 1000) / 2) && Direction == BW)	// Check if we are running into the dead spot, add the current position
			EffSteps += (1 - PumpRatio) * 1000;		// Adding steps equal to dead spot size if we run into the dead spot


		// Cannot hit the dead spot twice... No matter where you start any volume that would get you back to where you are is taken care of above, so any fraction should always end shorter than start position

		NumberOfSteps += EffSteps;	// Add the steps needed after accounting for dead spot

		//	UARTprintf("Steps to pump: %d\n", NumberOfSteps);

//		UARTprintf("Start Position: %d, Pump Volume: %d nL, Steps: %d steps\n", (g_PumpStepsTravelled % 1000), (int) (Volume * 1000), NumberOfSteps);
		if(AbortReady)
			PumpStepperRunStepSpeed_AbortReady(Direction, NumberOfSteps, EndDelay);
		else
			PumpStepperRunStepSpeed(Direction, NumberOfSteps, EndDelay);
	}
}
