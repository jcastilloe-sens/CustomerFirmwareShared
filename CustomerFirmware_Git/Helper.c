//*****************************************************************************
//
// Helper.c - Functions used in e-SENS firmware to support other functions and
// to initialize parts of the device not used for specific systems
//
// Author: Jason Castillo
//
//*****************************************************************************
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ssi.h"
#include "driverlib/adc.h"
#include "driverlib/eeprom.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "Amperometrics.h"
#include "Bluetooth.h"
#include "Communication.h"
#include "Helper.h"
#include "Steppers.h"
#include "Components.h"
#include "inc/tm4c123gh6pm.h"
#include "main.h"
#include "inc/hw_nvic.h"
#ifdef MCU_ZXR
#include "PinMap.h"
#endif

uint32_t g_TimerInterruptFlag = 0;
uint32_t g_TimerPeriodicInterruptFlag = 0;
uint32_t g_TimerTemperatureInterruptFlag = 0;
uint32_t gui32Error = 0;
uint8_t gui8MemConnected = 1;

uint32_t gBoard = V7;
uint32_t gABoard = AV7;

//
// Constants
//
//float K_T_pH_Cal_1 = -0.0112; //0.0063; // Temperature coefficient Cal 1
//float K_T_pH_Cal_2 = -0.0155; //0.0053; // Temperature coefficient Cal 2
//float K_T_pH_Rinse = -0.0127;
//float K_T_pH_Samp = -0.009;
////static float K_T_pH_Cal_3 = 0.0047; // Temperature coefficient Cal 3
////static float K_T_pH = -0.80; 	// mV/C pH temperature compensation coefficient
////static float pKa1 = 6.352;		// Constant used for alpha CO2 calculation
////static float pKa2 = 10.329;	// Constant used for aplha CO2 calculation
////static float K_T_CO2 = -0.10; 	// mV/C CO2 Temperature Compensation Coefficient
////static float K_T_Ca = 0.10;	// mV/C Calcium Temperature Compensation Coefficient
////static float K_T_TH = 0.10;	// mV/C THard Temperature Compensation Coefficient
////static float K_T_NH4 = -0.10;	// mV/C NH4 Temperature Compensation Coefficient
//float pKa_NH4 = 9.246;

//**************************************************************************
// Initializes all the components on power-up or reset, put in a function
// so that as updates are made individual codes don't need updates
// Created: 1/7/2019
// 10/12/2020: Modified to initialize LED IO extender before providing power
//	to analog board and LEDs
// Parameters:  NONE
//**************************************************************************
void Init_all(uint8_t Steppers)
{
	//	InitReset();		// Unlocks pin, sets high then pulses low to reset components
		// Initialize all modules
		InitEEPROM();		// Turns on EEPROM peripheral and initializes it to check for errors
//		gBoard = V7;
//		gABoard = AV7;
//		EEPROMProgram(&gBoard, OFFSET_DBOARD_V, 4);
//		EEPROMProgram(&gABoard, OFFSET_ABOARD_V, 4);

		EEPROMRead(&gBoard, OFFSET_DBOARD_V, 4);
		EEPROMRead(&gABoard, OFFSET_ABOARD_V, 4);
		if(gBoard == 0 || gBoard == 0xFFFFFFFF || gABoard == 0 || gABoard == 0xFFFFFFFF)
		{
	//		UARTprintf("Board versions not saved!");
	//		UARTprintf("Saving versions V7.2 and AV7.3 for now!\n");
			unsigned char Hardware_Rev[8];
			EEPROMRead((uint32_t *) Hardware_Rev, OFFSET_HARDWARE_REV, 8);
			if(Hardware_Rev[4] == 0x37 && Hardware_Rev[6] == 0x32)
			{
				gBoard = V7_2;
				gABoard = AV7_3;
			}
			else
			{
				gBoard = V7;
				gABoard = AV7;
			}

			EEPROMProgram(&gBoard, OFFSET_DBOARD_V, 4);
			EEPROMProgram(&gABoard, OFFSET_ABOARD_V, 4);
		}
	//	UARTprintf("Digital Board Version: %d\n", gBoard);
	//	UARTprintf("Analog Board Version:%d\n", gABoard);

		InitGPIOPins();		// Configures GPIO pins for use with button, LEDs, etc
	//	BuzzerSound(400);

		// Created mechanism to determine if we are trying to wake from hibernate multiple times, if we are...
		// this indicates a problem so clear the flag and boot as if reset
		if(HibernateIsActive() == 1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)	// If we are waking from hibernate
		{
			uint32_t Hibernating;
			EEPROMRead((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);

			if(Hibernating == 1)
			{
				Hibernating = 0;
				EEPROMProgram((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
			}
			else
			{
				InitHibernate(); 	// GPIO States are held, wakes on wake pin
			}
		}

		// 5/16/2022: Moved InitSPI to before resetting BT so interrupt can fire as soon as BT is booted up and ready for Tiva
		InitSPI();			// Sets up one master port and one slave port with interrupt

		if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
		{
#ifdef MCU_ZXR
			GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, IO_RESET_BT_PIN); // Reset BT at beginning to ensure proper boot sequence
	//		while(1);
			g_ui32DataRx0[0] = 0;	// Clear any previous BT commands, want to be ready to catch first command sent
			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
			GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, 0x00); // Reset BT at beginning to ensure proper boot sequence

#else
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT at beginning to ensure proper boot sequence
	//		while(1);
			g_ui32DataRx0[0] = 0;	// Clear any previous BT commands, want to be ready to catch first command sent
			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT at beginning to ensure proper boot sequence
#endif	// MCU_ZXR
		}

		InitFPU();			// Lazy stacking setup for interrupt handlers
		InitTimer();		// Configures two timers and interrupts for both
		InitConsole();		// Sets up UART communication to terminal program

		DEBUG_PRINT(UARTprintf("Digital Board Version: %d\n", gBoard);)
		DEBUG_PRINT(UARTprintf("Analog Board Version:%d\n", gABoard);)

//		InitSPI();			// Sets up one master port and one slave port with interrupt
		InitI2C();			// Enables I2C port as master and sets up pins
		InitBT();			// Verifies the BT chip is running and has finished I2C setup

		uint8_t Battery_Percent;
		if(gBoard >= V7_2)
		{
			userDelay(10, 0);
			Battery_Percent = BatteryRead(REP_SOC_REG);
			DEBUG_PRINT(UARTprintf("Battery Percent: %d\n", Battery_Percent);)
			if(gBoard < RV1_0)	// Roam Digital V1.0 switched from extender to direct GPIO controlled LEDs, initialized in InitGPIO()
				InitLED_Ext();

			if(Battery_Percent > 5)
			{
				if(gBoard >= V5)
				{
#ifdef MCU_ZXR
					GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On
#else
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
#endif	// MCU_ZXR
				}

				SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

				// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
#ifdef MCU_ZXR
				if(gABoard >= AV6_1)
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN); // Conductivity ADC CNV Pin
				else
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin

				// Toggle Reset Pin
				GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
				GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Set high for normal operation
				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
#else
				if(gABoard >= AV6_1)
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
				else
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin

				// Toggle Reset Pin
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
#endif	// MCU_ZXR

				InitIO_Ext();		// Sets up IO Extenders and sets initial values

				if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
					if(Battery_Percent >= MIN_BAT_LEVEL)
						SetLED(BLUE_BUTTON, 1);
			}

			InitBattery();		// Configures battery IC to alert, turns on initial LEDs and updates BT on battery status

			InitCartridge();	// Checks if there is a cartridge present (memory) and update BT on whether there is one or not

			if(Battery_Percent > 5)
			{
				InitDAC();			// Resets device and writes configuration register
				InitTurbidityADC();
				InitADC();			// Resets devices and configures all channels
				InitWaveGen(0);		// Sets up waveform generator to output at 1 kHz
			}

		}
		else
		{
			InitIO_Ext();		// Sets up IO Extenders and sets initial values

			InitLED_Ext();

			userDelay(10, 0);
			Battery_Percent = BatteryRead(REP_SOC_REG);
			DEBUG_PRINT(UARTprintf("Battery Percent: %d\n", Battery_Percent);)
			if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
				if(Battery_Percent >= MIN_BAT_LEVEL)
					SetLED(BLUE_BUTTON, 1);

			InitBattery();		// Configures battery IC to alert, turns on initial LEDs and updates BT on battery status

			InitCartridge();	// Checks if there is a cartridge present (memory) and update BT on whether there is one or not
			InitDAC();			// Resets device and writes configuration register
			InitTurbidityADC();
			InitADC();			// Resets devices and configures all channels
			InitWaveGen(0);		// Sets up waveform generator to output at 1 kHz
		}

		InitCPUTemp();		// Initializes ADC to read temperature select channel

		if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
		{
			//		update_Test(FindTestNumber());
			//		update_Cal(FindCalNumber());
			update_Auto_Cal();
		}

		if(Battery_Percent >= MIN_BAT_LEVEL)
			InitSteppers(Steppers & gCartridge);		// Sets up pins to control steppers and finds valve home position
		else
			InitSteppers(0);		// Sets up pins to control steppers and finds valve home position

		InitHibernate(); 	// GPIO States are held, wakes on wake pin. Must be last part of initilization because this clears hibernate status so other parts won't know if waking from hibernate or resetting
}

#ifdef MCU_ZXR
//**************************************************************************
// Initializes the GPIO pins not used for communication or steppers
// GPIO Pins will retain state during hibernate
// Device will wake when wake pin is asserted (pulled low)
// Parameters:  NONE
// 1/3/2019: Updated for digital V6.2 and analog V6.3
// 12/27/2022: Going to be basing all the pins off the PinMap.h file, leaving
// the old way of doing it intact under the #else MCU_ZXR
//**************************************************************************
void InitGPIOPins(void)
{
	// Enable GPIO Peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

	SysCtlDelay(300); // Delay so peripherals have time to power up

	// Unlock PD7 pin so pin can be controlled
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;

	// Set up direction and strength of GPIO Pins
	GPIODirModeSet(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN, GPIO_DIR_MODE_IN);						// I2C used by BT
	GPIOPadConfigSet(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	GPIODirModeSet(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, GPIO_DIR_MODE_OUT);						// I2C used by TIVA -> Tiva request
	GPIOPadConfigSet(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	GPIODirModeSet(IO_BUZZER_BASE(gBoard), IO_BUZZER_PIN, GPIO_DIR_MODE_OUT);						// Buzzer
	GPIOPadConfigSet(IO_BUZZER_BASE(gBoard), IO_BUZZER_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	GPIODirModeSet(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, GPIO_DIR_MODE_OUT);						// Conductivity ADC CNV Pin
	GPIOPadConfigSet(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(IO_RESET_BT_BASE, IO_RESET_BT_PIN, GPIO_DIR_MODE_OUT);						// Reset BT
	GPIOPadConfigSet(IO_RESET_BT_BASE, IO_RESET_BT_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
	GPIODirModeSet(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, GPIO_DIR_MODE_OUT );						// Reset analog
	GPIOPadConfigSet(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN,GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

	if(gBoard >= V7)	// Only set I2C used by Tiva on boards after V7, didn't exist before that
	{
		GPIODirModeSet(IO_I2C_USED_BY_TIVA_BASE, IO_I2C_USED_BY_TIVA_PIN, GPIO_DIR_MODE_OUT);						// Started on V7 I2C_USED_BY_TIVA, used to be battery charging, removed this signal
		GPIOPadConfigSet(IO_I2C_USED_BY_TIVA_BASE, IO_I2C_USED_BY_TIVA_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
		GPIOPinWrite(IO_I2C_USED_BY_TIVA_BASE, IO_I2C_USED_BY_TIVA_PIN, 0x00); // I2C used by TIVA
	}
	else	// Only set bat charging signal on boards before V7, completely removed after that
	{
		GPIODirModeSet(IO_BAT_CHARGING_BASE, IO_BAT_CHARGING_PIN, GPIO_DIR_MODE_IN);						// Was Battery charging before then became V7 -> I2C_USED_BY_TIVA
		GPIOPadConfigSet(IO_BAT_CHARGING_BASE, IO_BAT_CHARGING_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	}

	GPIODirModeSet(IO_BAT_ALERT_BASE, IO_BAT_ALERT_PIN, GPIO_DIR_MODE_IN);						// Battery alert
	GPIOPadConfigSet(IO_BAT_ALERT_BASE, IO_BAT_ALERT_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	GPIODirModeSet(IO_BUTTON_BASE, IO_BUTTON_PIN, GPIO_DIR_MODE_IN);						// Button
	GPIOPadConfigSet(IO_BUTTON_BASE, IO_BUTTON_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	if(gBoard >= V6_2)
	{
		GPIODirModeSet(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN, GPIO_DIR_MODE_IN);						// Power good _B
		GPIOPadConfigSet(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
		GPIODirModeSet(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, GPIO_DIR_MODE_OUT);						// LED Osc RST _B
		GPIOPadConfigSet(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

		GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, IO_LED_OSC_RST_B_PIN); // LED Osc RST _B
		SysCtlDelay(SysCtlClockGet()/3000000);
		GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B
		SysCtlDelay(SysCtlClockGet()/3000000);
		if(gBoard < V7_2)
			GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, IO_LED_OSC_RST_B_PIN); // LED Osc RST _B
	}

	if(gBoard >= RV1_0)	// Roam Digital V1.0 switched from extender to direct GPIO controlled LEDs
	{
		GPIODirModeSet(LED_BUT_R_H_SW_BASE, LED_BUT_R_H_SW_PIN, GPIO_DIR_MODE_OUT );						// Red Button H SW LED
		GPIOPadConfigSet(LED_BUT_R_H_SW_BASE, LED_BUT_R_H_SW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_BUT_R_H_SEL_BASE, LED_BUT_R_H_SEL_PIN, GPIO_DIR_MODE_OUT );						// Red Button H SEL LED
		GPIOPadConfigSet(LED_BUT_R_H_SEL_BASE, LED_BUT_R_H_SEL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet(LED_BUT_R_V_SW_BASE, LED_BUT_R_V_SW_PIN, GPIO_DIR_MODE_OUT );						// Red Button V SW LED
		GPIOPadConfigSet(LED_BUT_R_V_SW_BASE, LED_BUT_R_V_SW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_BUT_R_V_SEL_BASE, LED_BUT_R_V_SEL_PIN, GPIO_DIR_MODE_OUT );						// Red Button V SEL LED
		GPIOPadConfigSet(LED_BUT_R_V_SEL_BASE, LED_BUT_R_V_SEL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet(LED_BUT_B_H_SW_BASE, LED_BUT_B_H_SW_PIN, GPIO_DIR_MODE_OUT );						// Blue Button H SW LED
		GPIOPadConfigSet(LED_BUT_B_H_SW_BASE, LED_BUT_B_H_SW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_BUT_B_H_SEL_BASE, LED_BUT_B_H_SEL_PIN, GPIO_DIR_MODE_OUT );						// Blue Button H SEL LED
		GPIOPadConfigSet(LED_BUT_B_H_SEL_BASE, LED_BUT_B_H_SEL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet(LED_BUT_B_V_SW_BASE, LED_BUT_B_V_SW_PIN, GPIO_DIR_MODE_OUT );						// Blue Button V SW LED
		GPIOPadConfigSet(LED_BUT_B_V_SW_BASE, LED_BUT_B_V_SW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_BUT_B_V_SEL_BASE, LED_BUT_B_V_SEL_PIN, GPIO_DIR_MODE_OUT );						// Blue Button V SEL LED
		GPIOPadConfigSet(LED_BUT_B_V_SEL_BASE, LED_BUT_B_V_SEL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet(LED_BUT_G_H_SW_BASE, LED_BUT_G_H_SW_PIN, GPIO_DIR_MODE_OUT );						// Green Button H SW LED
		GPIOPadConfigSet(LED_BUT_G_H_SW_BASE, LED_BUT_G_H_SW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_BUT_G_H_SEL_BASE, LED_BUT_G_H_SEL_PIN, GPIO_DIR_MODE_OUT );						// Green Button H SEL LED
		GPIOPadConfigSet(LED_BUT_G_H_SEL_BASE, LED_BUT_G_H_SEL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet(LED_BUT_G_V_SW_BASE, LED_BUT_G_V_SW_PIN, GPIO_DIR_MODE_OUT );						// Green Button V SW LED
		GPIOPadConfigSet(LED_BUT_G_V_SW_BASE, LED_BUT_G_V_SW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_BUT_G_V_SEL_BASE, LED_BUT_G_V_SEL_PIN, GPIO_DIR_MODE_OUT );						// Green Button V SEL LED
		GPIOPadConfigSet(LED_BUT_G_V_SEL_BASE, LED_BUT_G_V_SEL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet(LED_CHG_R_SW_BASE, LED_CHG_R_SW_PIN, GPIO_DIR_MODE_OUT );						// Red Charge SW LED
		GPIOPadConfigSet(LED_CHG_R_SW_BASE, LED_CHG_R_SW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_CHG_R_SEL_BASE, LED_CHG_R_SEL_PIN, GPIO_DIR_MODE_OUT );						// Red Charge SEL LED
		GPIOPadConfigSet(LED_CHG_R_SEL_BASE, LED_CHG_R_SEL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet(LED_CHG_G_BASE, LED_CHG_G_PIN, GPIO_DIR_MODE_OUT );						// Green Charge LED
		GPIOPadConfigSet(LED_CHG_G_BASE, LED_CHG_G_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet(LED_CHG_Y_BASE, LED_CHG_Y_PIN, GPIO_DIR_MODE_OUT );						// Yellow Charge LED
		GPIOPadConfigSet(LED_CHG_Y_BASE, LED_CHG_Y_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	}
	else if(gBoard >= V6_2)	// Dropped the extender on Roam Digital V1.0
	{
		GPIODirModeSet( IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, GPIO_DIR_MODE_OUT );						// LED Ext RST _B
		GPIOPadConfigSet( IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, GPIO_DIR_MODE_OUT );						// LED Ext CS _B
		GPIOPadConfigSet( IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B

		GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
		SysCtlDelay(SysCtlClockGet()/3000000);
		GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, 0); // LED Ext RST _B
		SysCtlDelay(SysCtlClockGet()/3000000);
		GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
	}
	else if(gBoard < V6_2)
	{
		GPIODirModeSet( LED_CHG_RED_BASE, LED_CHG_RED_PIN, GPIO_DIR_MODE_OUT );						// Red charging LED
		GPIOPadConfigSet( LED_CHG_RED_BASE, LED_CHG_RED_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( LED_CHG_GREEN_BASE, LED_CHG_GREEN_PIN, GPIO_DIR_MODE_OUT );						// Green charging LED
		GPIOPadConfigSet( LED_CHG_GREEN_BASE, LED_CHG_GREEN_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet( LED_BUT_GREEN_BASE, LED_BUT_GREEN_PIN, GPIO_DIR_MODE_OUT );						// Green button LED
		GPIOPadConfigSet( LED_BUT_GREEN_BASE, LED_BUT_GREEN_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( LED_BUT_RED_BASE, LED_BUT_RED_PIN, GPIO_DIR_MODE_OUT );						// Red button LED
		GPIOPadConfigSet( LED_BUT_RED_BASE, LED_BUT_RED_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet( LED_CHG_YELLOW_BASE, LED_CHG_YELLOW_PIN, GPIO_DIR_MODE_OUT );						// Orange charging LED
		GPIOPadConfigSet( LED_CHG_YELLOW_BASE, LED_CHG_YELLOW_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( LED_BUT_BLUE_BASE, LED_BUT_BLUE_PIN, GPIO_DIR_MODE_OUT );						// Blue button LED
		GPIOPadConfigSet( LED_BUT_BLUE_BASE, LED_BUT_BLUE_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIOPinWrite(LED_CHG_RED_BASE, LED_CHG_RED_PIN, 0x00); // Red charging LED
		GPIOPinWrite(LED_CHG_GREEN_BASE, LED_CHG_GREEN_PIN, 0x00); // Green charging LED

		GPIOPinWrite(LED_BUT_GREEN_BASE, LED_BUT_GREEN_PIN, 0x00); // Green button LED
		GPIOPinWrite(LED_BUT_RED_BASE, LED_BUT_RED_PIN, 0x00); // Red button LED

		GPIOPinWrite(LED_CHG_YELLOW_BASE, LED_CHG_YELLOW_PIN, 0x00); // Yellow charging LED
		GPIOPinWrite(LED_BUT_BLUE_BASE, LED_BUT_BLUE_PIN, 0x00); // Blue button LED
	}

	GPIODirModeSet(IO_ACC_INT_1_BASE(gBoard), IO_ACC_INT_1_PIN, GPIO_DIR_MODE_IN);						// Acc Int 1
	GPIOPadConfigSet(IO_ACC_INT_1_BASE(gBoard), IO_ACC_INT_1_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	// Initial Values
	GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA -> V7 Tiva Request

	GPIOPinWrite(IO_BUZZER_BASE(gBoard), IO_BUZZER_PIN, 0x00); // Buzzer

	GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, 0x00); // Reset BT
	GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

	// Set pin low turn on analog board, then set high
	GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B

	if(gBoard >= V5)
	{
		GPIODirModeSet( IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, GPIO_DIR_MODE_OUT );						// Analog On
		GPIOPadConfigSet( IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		if(gBoard < V7_2)	// Needed analog on to talk to LED extender, no longer necessary from V7.2 onward
			GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On
		else
			GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, 0x00); // Analog On
	}
	if(gBoard == V6)
	{
		GPIODirModeSet(IO_LED_FULL_INTENSITY_BASE, IO_LED_FULL_INTENSITY_PIN, GPIO_DIR_MODE_OUT);						// LED full intensity
		GPIOPadConfigSet(IO_LED_FULL_INTENSITY_BASE, IO_LED_FULL_INTENSITY_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

		GPIOPinWrite(IO_LED_FULL_INTENSITY_BASE, IO_LED_FULL_INTENSITY_PIN, 0x00); // LED full intensity
	}

	if(gBoard < V7_2)
	{
		SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

		// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
		if(gABoard >= AV6_1)
			GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN); // Conductivity ADC CNV Pin
		else
			GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin

		// Toggle Reset Pin
		GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
		GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Set high for normal operation
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
	}
}
#else	// MCU_ZXR
//**************************************************************************
// Initializes the GPIO pins not used for communication or steppers
// GPIO Pins will retain state during hibernate
// Device will wake when wake pin is asserted (pulled low)
// Parameters:  NONE
// 1/3/2019: Updated for digital V6.2 and analog V6.3
//**************************************************************************
void InitGPIOPins(void)
{
	// Enable GPIO Peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	SysCtlDelay(300); // Delay so peripherals have time to power up

	// Unlock PD7 pin so pin can be controlled
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;

	// Set up direction and strength of GPIO Pins
	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN );						// I2C used by BT
	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );						// I2C used by TIVA -> Tiva request
	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

	GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Buzzer
	GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT );						// Conductivity ADC CNV Pin
	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT );						// Reset BT
	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );						// Reset analog
	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_7,GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );

	if(gBoard >= V7)
	{
		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// Battery charging, V7 -> I2C_USED_BY_TIVA
		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0x00); // I2C used by TIVA
	}
	else
	{
		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN );						// Battery charging, V7 -> I2C_USED_BY_TIVA
		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
	}

	GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN );						// Battery alert
	GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

	GPIODirModeSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_DIR_MODE_IN );						// Button
	GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

	if(gBoard < V6_2)
	{
		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Red charging LED
		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Green charging LED
		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// Green button LED
		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// Red button LED
		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Acc Int 1
		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN );						// Acc Int 2
		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Orange charging LED
		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Blue button LED
		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00); // Red charging LED
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // Green charging LED

		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // Green button LED
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00); // Red button LED

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00); // Yellow charging LED
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00); // Blue button LED
	}
	else if(gBoard >= V6_2)
	{
		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Acc Int 1
		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// LED Ext RST _B
		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// LED Ext CS _B
		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Power good _B
		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// LED Osc RST _B
		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
		SysCtlDelay(SysCtlClockGet()/3000000);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0); // LED Ext RST _B
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
		SysCtlDelay(SysCtlClockGet()/3000000);
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
		if(gBoard < V7_2)
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
	}

	// Initial Values
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA -> V7 Tiva Request

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x00); // Buzzer

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

	// Set pin low turn on analog board, then set high
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B

	if(gBoard >= V5)
	{
		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Analog On
		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
		if(gBoard < V7_2)
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
		else
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00); // Analog On
	}
	if(gBoard == V6)
	{
		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// LED full intensity
		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );

		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0x00); // LED full intensity
	}


	if(gBoard < V7_2)
	{
		SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

		// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
		if(gABoard >= AV6_1)
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
		else
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin

		// Toggle Reset Pin
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
	}
}
#endif	// MCU_ZXR

//#ifdef I2C_LED_IOEXT
////**************************************************************************
//// Initializes all the components on power-up or reset, put in a function
//// so that as updates are made individual codes don't need updates
//// Created: 1/7/2019
//// 10/12/2020: Modified to initialize LED IO extender before providing power
////	to analog board and LEDs
//// Parameters:  NONE
////**************************************************************************
//void Init_all(uint8_t Steppers)
//{
//	//	InitReset();		// Unlocks pin, sets high then pulses low to reset components
//		// Initialize all modules
//		InitEEPROM();		// Turns on EEPROM peripheral and initializes it to check for errors
//		EEPROMRead(&gBoard, OFFSET_DBOARD_V, 4);
//		EEPROMRead(&gABoard, OFFSET_ABOARD_V, 4);
//		if(gBoard == 0 || gBoard == 0xFFFFFFFF || gABoard == 0 || gABoard == 0xFFFFFFFF)
//		{
//	//		UARTprintf("Board versions not saved!");
//	//		UARTprintf("Saving versions V7.2 and AV7.3 for now!\n");
//			gBoard = V7_2;
//			gABoard = AV7_3;
//
//			EEPROMProgram(&gBoard, OFFSET_DBOARD_V, 4);
//			EEPROMProgram(&gABoard, OFFSET_ABOARD_V, 4);
//		}
//	//	UARTprintf("Digital Board Version: %d\n", gBoard);
//	//	UARTprintf("Analog Board Version:%d\n", gABoard);
//
//		InitGPIOPins();		// Configures GPIO pins for use with button, LEDs, etc
//	//	BuzzerSound(400);
//
//		// Created mechanism to determine if we are trying to wake from hibernate multiple times, if we are...
//		// this indicates a problem so clear the flag and boot as if reset
//		if(HibernateIsActive() == 1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)	// If we are waking from hibernate
//		{
//			uint32_t Hibernating;
//			EEPROMRead((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
//
//			if(Hibernating == 1)
//			{
//				Hibernating = 0;
//				EEPROMProgram((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
//			}
//			else
//			{
//				InitHibernate(); 	// GPIO States are held, wakes on wake pin
//			}
//		}
//
//		if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
//		{
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT at beginning to ensure proper boot sequence
//	//		while(1);
//			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT at beginning to ensure proper boot sequence
//		}
//
//		InitFPU();			// Lazy stacking setup for interrupt handlers
//		InitTimer();		// Configures two timers and interrupts for both
//		InitConsole();		// Sets up UART communication to terminal program
//
//		UARTprintf("Digital Board Version: %d\n", gBoard);
//		UARTprintf("Analog Board Version:%d\n", gABoard);
//
//		InitSPI();			// Sets up one master port and one slave port with interrupt
//		InitI2C();			// Enables I2C port as master and sets up pins
//		InitBT();			// Verifies the BT chip is running and has finished I2C setup
//
//		InitLED_Ext();
//
//		uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
//		if(Battery_Percent > 5)
//		{
//			if(gBoard >= V5)
//			{
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//			}
//
//			SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
//			// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
//			if(gABoard >= AV6_1)
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//			else
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
//
//			// Toggle Reset Pin
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
//			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
//			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//
//			InitIO_Ext();		// Sets up IO Extenders and sets initial values
//
//			if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
//				if(Battery_Percent >= MIN_BAT_LEVEL)
//					SetLED(BLUE_BUTTON, 1);
//		}
//
//		InitBattery();		// Configures battery IC to alert, turns on initial LEDs and updates BT on battery status
//
//		InitCartridge();	// Checks if there is a cartridge present (memory) and update BT on whether there is one or not
//
//		if(Battery_Percent > 5)
//		{
//			InitDAC();			// Resets device and writes configuration register
//			InitTurbidityADC();
//			InitADC();			// Resets devices and configures all channels
//			InitWaveGen(0);		// Sets up waveform generator to output at 1 kHz
//		}
//
//		InitCPUTemp();		// Initializes ADC to read temperature select channel
//
//		if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
//		{
//	//		update_Test(FindTestNumber());
//	//		update_Cal(FindCalNumber());
//			update_Auto_Cal();
//		}
//
//		if(Battery_Percent >= MIN_BAT_LEVEL)
//			InitSteppers(Steppers & gCartridge);		// Sets up pins to control steppers and finds valve home position
//		else
//			InitSteppers(0);		// Sets up pins to control steppers and finds valve home position
//
//		InitHibernate(); 	// GPIO States are held, wakes on wake pin. Must be last part of initilization because this clears hibernate status so other parts won't know if waking from hibernate or resetting
//
//}
//
//////**************************************************************************
////// Initializes all the components on power-up or reset, put in a function
////// so that as updates are made individual codes don't need updates
////// Created: 1/7/2019
////// 10/12/2020: Modified to initialize LED IO extender before providing power
//////	to analog board and LEDs
////// Parameters:  NONE
//////**************************************************************************
////void Init_all(uint8_t Steppers)
////{
//////	InitReset();		// Unlocks pin, sets high then pulses low to reset components
////	// Initialize all modules
////	InitEEPROM();		// Turns on EEPROM peripheral and initializes it to check for errors
////	EEPROMRead(&gBoard, OFFSET_DBOARD_V, 4);
////	EEPROMRead(&gABoard, OFFSET_ABOARD_V, 4);
////	if(gBoard == 0 || gBoard == 0xFFFFFFFF || gABoard == 0 || gABoard == 0xFFFFFFFF)
////	{
//////		UARTprintf("Board versions not saved!");
//////		UARTprintf("Saving versions V7.2 and AV7.3 for now!\n");
////		gBoard = V7_2;
////		gABoard = AV7_3;
////
////		EEPROMProgram(&gBoard, OFFSET_DBOARD_V, 4);
////		EEPROMProgram(&gABoard, OFFSET_ABOARD_V, 4);
////	}
//////	UARTprintf("Digital Board Version: %d\n", gBoard);
//////	UARTprintf("Analog Board Version:%d\n", gABoard);
////
////	InitGPIOPins();		// Configures GPIO pins for use with button, LEDs, etc
//////	BuzzerSound(400);
////
////	// Created mechanism to determine if we are trying to wake from hibernate multiple times, if we are...
////	// this indicates a problem so clear the flag and boot as if reset
////	if(HibernateIsActive() == 1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)	// If we are waking from hibernate
////	{
////		uint32_t Hibernating;
////		EEPROMRead((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
////
////		if(Hibernating == 1)
////		{
////			Hibernating = 0;
////			EEPROMProgram((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
////		}
////		else
////		{
////			InitHibernate(); 	// GPIO States are held, wakes on wake pin
////		}
////	}
////
////	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
////	{
////		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT at beginning to ensure proper boot sequence
//////		while(1);
////		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
////		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT at beginning to ensure proper boot sequence
////	}
////
////	InitFPU();			// Lazy stacking setup for interrupt handlers
////	InitTimer();		// Configures two timers and interrupts for both
////	InitConsole();		// Sets up UART communication to terminal program
////
////	UARTprintf("Digital Board Version: %d\n", gBoard);
////	UARTprintf("Analog Board Version:%d\n", gABoard);
////
////	InitSPI();			// Sets up one master port and one slave port with interrupt
////	InitI2C();			// Enables I2C port as master and sets up pins
////	InitBT();			// Verifies the BT chip is running and has finished I2C setup
////
////	InitLED_Ext();
////
////	if(gBoard >= V5)
////	{
////		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
////	}
////
////	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
////
////	// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
////	if(gABoard >= AV6_1)
////		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
////	else
////		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
////
////	// Toggle Reset Pin
////	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
////	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
////	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
////	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
////
////	InitIO_Ext();		// Sets up IO Extenders and sets initial values
////
////	uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
////	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
////		if(Battery_Percent >= MIN_BAT_LEVEL)
////			SetLED(BLUE_BUTTON, 1);
////
////	InitBattery();		// Configures battery IC to alert, turns on initial LEDs and updates BT on battery status
////
////	InitCartridge();	// Checks if there is a cartridge present (memory) and update BT on whether there is one or not
////	InitDAC();			// Resets device and writes configuration register
////	InitTurbidityADC();
////	InitADC();			// Resets devices and configures all channels
////	InitWaveGen(0);		// Sets up waveform generator to output at 1 kHz
////
////	InitCPUTemp();		// Initializes ADC to read temperature select channel
////
////	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
////	{
//////		update_Test(FindTestNumber());
//////		update_Cal(FindCalNumber());
////		update_Auto_Cal();
////	}
////
////	if(Battery_Percent >= MIN_BAT_LEVEL)
////		InitSteppers(Steppers & gCartridge);		// Sets up pins to control steppers and finds valve home position
////	else
////		InitSteppers(0);		// Sets up pins to control steppers and finds valve home position
////
////	InitHibernate(); 	// GPIO States are held, wakes on wake pin. Must be last part of initilization because this clears hibernate status so other parts won't know if waking from hibernate or resetting
////}
//
////**************************************************************************
//// Initializes the GPIO pins not used for communication or steppers
//// GPIO Pins will retain state during hibernate
//// Device will wake when wake pin is asserted (pulled low)
//// Parameters:  NONE
//// 1/3/2019: Updated for digital V6.2 and analog V6.3
////**************************************************************************
//void InitGPIOPins(void)
//{
//	// Enable GPIO Peripherals
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//
//	SysCtlDelay(300); // Delay so peripherals have time to power up
//
//	// Unlock PD7 pin so pin can be controlled
//	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
//	HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;
//
//	// Set up direction and strength of GPIO Pins
//	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN );						// I2C used by BT
//	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );						// I2C used by TIVA -> Tiva request
//	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Buzzer
//	GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT );						// Conductivity ADC CNV Pin
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT );						// Reset BT
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );						// Reset analog
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_7,GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//
//	if(gBoard >= V7)
//	{
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// Battery charging, V7 -> I2C_USED_BY_TIVA
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0x00); // I2C used by TIVA
//	}
//	else
//	{
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN );						// Battery charging, V7 -> I2C_USED_BY_TIVA
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	}
//
//	GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN );						// Battery alert
//	GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_DIR_MODE_IN );						// Button
//	GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	if(gBoard < V6_2)
//	{
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Red charging LED
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Green charging LED
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// Green button LED
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// Red button LED
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Acc Int 1
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN );						// Acc Int 2
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Orange charging LED
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Blue button LED
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00); // Red charging LED
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // Green charging LED
//
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // Green button LED
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00); // Red button LED
//
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00); // Yellow charging LED
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00); // Blue button LED
//	}
//	else if(gBoard >= V6_2)
//	{
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Acc Int 1
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// LED Ext RST _B
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// LED Ext CS _B
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Power good _B
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// LED Osc RST _B
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B
//
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//		SysCtlDelay(SysCtlClockGet()/3000000);
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0); // LED Ext RST _B
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
//		SysCtlDelay(SysCtlClockGet()/3000000);
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
////		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
//	}
//
//	// Initial Values
//	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA -> V7 Tiva Request
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x00); // Buzzer
//
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation
//
//	// Set pin low turn on analog board, then set high
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
//
//	if(gBoard >= V5)
//	{
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Analog On
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00); // Analog On
//	}
//	if(gBoard == V6)
//	{
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// LED full intensity
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0x00); // LED full intensity
//	}
//
////	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
////	// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
////	if(gABoard >= AV6_1)
////		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
////	else
////		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
////
////	// Toggle Reset Pin
////	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
////	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
////	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
////	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//}
//
//#else
////**************************************************************************
//// Initializes all the components on power-up or reset, put in a function
//// so that as updates are made individual codes don't need updates
//// Created: 1/7/2019
//// Parameters:  NONE
////**************************************************************************
//void Init_all(uint8_t Steppers)
//{
////	InitReset();		// Unlocks pin, sets high then pulses low to reset components
//	// Initialize all modules
//	InitEEPROM();		// Turns on EEPROM peripheral and initializes it to check for errors
//	InitGPIOPins();		// Configures GPIO pins for use with button, LEDs, etc
////	BuzzerSound(400);
//
//	// Created mechanism to determine if we are trying to wake from hibernate multiple times, if we are...
//	// this indicates a problem so clear the flag and boot as if reset
//	if(HibernateIsActive() == 1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)	// If we are waking from hibernate
//	{
//		uint32_t Hibernating;
//		EEPROMRead((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
//
//		if(Hibernating == 1)
//		{
//			Hibernating = 0;
//			EEPROMProgram((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
//		}
//		else
//		{
//			InitHibernate(); 	// GPIO States are held, wakes on wake pin
//		}
//	}
//
//	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
//	{
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT at beginning to ensure proper boot sequence
////		while(1);
//		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT at beginning to ensure proper boot sequence
//	}
//
//	InitFPU();			// Lazy stacking setup for interrupt handlers
//	InitTimer();		// Configures two timers and interrupts for both
//	InitConsole();		// Sets up UART communication to terminal program
//
//	InitSPI();			// Sets up one master port and one slave port with interrupt
//	InitI2C();			// Enables I2C port as master and sets up pins
//	InitBT();			// Verifies the BT chip is running and has finished I2C setup
//
////	SysCtlDelay(SysCtlClockGet());
//
////	SysCtlDelay(SysCtlClockGet());
////	SetLED(RED_CHARGE_BLINK, 1);
////
////	SetLED(GREEN_BUTTON, 1);
////
////	SetLED(GREEN_BUTTON, 0);
////
////	SetLED(RED_BUTTON, 1);
////
////	SetLED(RED_BUTTON, 0);
//
//	InitIO_Ext();		// Sets up IO Extenders and sets initial values
//
//	InitLED_Ext();
//
//	uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
//	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
//		if(Battery_Percent >= MIN_BAT_LEVEL)
//			SetLED(BLUE_BUTTON, 1);
//
//	InitBattery();		// Configures battery IC to alert, turns on initial LEDs and updates BT on battery status
//
//	InitCartridge();	// Checks if there is a cartridge present (memory) and update BT on whether there is one or not
//	InitDAC();			// Resets device and writes configuration register
//	InitTurbidityADC();
//	InitADC();			// Resets devices and configures all channels
//	InitWaveGen(0);		// Sets up waveform generator to output at 1 kHz
//
//	InitCPUTemp();		// Initializes ADC to read temperature select channel
//
//	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
//	{
////		update_Test(FindTestNumber());
////		update_Cal(FindCalNumber());
//		update_Auto_Cal();
//	}
//
//	if(Battery_Percent >= MIN_BAT_LEVEL)
//		InitSteppers(Steppers & gCartridge);		// Sets up pins to control steppers and finds valve home position
//	else
//		InitSteppers(0);		// Sets up pins to control steppers and finds valve home position
//
//	InitHibernate(); 	// GPIO States are held, wakes on wake pin. Must be last part of initilization because this clears hibernate status so other parts won't know if waking from hibernate or resetting
//}
//
////**************************************************************************
//// Initializes the GPIO pins not used for communication or steppers
//// GPIO Pins will retain state during hibernate
//// Device will wake when wake pin is asserted (pulled low)
//// Parameters:  NONE
//// 1/3/2019: Updated for digial V6.2 and analog V6.3
////**************************************************************************
//void InitGPIOPins(void)
//{
//	// Enable GPIO Peripherals
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//
//	SysCtlDelay(300); // Delay so peripherals have time to power up
//
//	// Unlock PD7 pin so pin can be controlled
//	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
//	HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;
//
//	// Set up direction and strength of GPIO Pins
//	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN );						// I2C used by BT
//	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );						// I2C used by TIVA -> Tiva request
//	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Buzzer
//	GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT );						// Conductivity ADC CNV Pin
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT );						// Reset BT
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );						// Reset analog
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_7,GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//
//	if(gBoard >= V7)
//	{
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// Battery charging, V7 -> I2C_USED_BY_TIVA
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0x00); // I2C used by TIVA
//	}
//	else
//	{
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN );						// Battery charging, V7 -> I2C_USED_BY_TIVA
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	}
//
//	GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN );						// Battery alert
//	GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_DIR_MODE_IN );						// Button
//	GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	if(gBoard < V6_2)
//	{
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Red charging LED
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Green charging LED
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// Green button LED
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// Red button LED
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Acc Int 1
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN );						// Acc Int 2
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Orange charging LED
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Blue button LED
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00); // Red charging LED
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // Green charging LED
//
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // Green button LED
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00); // Red button LED
//
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00); // Yellow charging LED
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00); // Blue button LED
//	}
//	else if(gBoard >= V6_2)
//	{
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Acc Int 1
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// LED Ext RST _B
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// LED Ext CS _B
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Power good _B
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// LED Osc RST _B
//		GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B
//
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//		SysCtlDelay(SysCtlClockGet()/3000000);
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0); // LED Ext RST _B
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
//		SysCtlDelay(SysCtlClockGet()/3000000);
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
//	}
//
//	// Initial Values
//	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA -> V7 Tiva Request
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x00); // Buzzer
//
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation
//
//	// Set pin low turn on analog board, then set high
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
//
//	if(gBoard >= V5)
//	{
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Analog On
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//	}
//	if(gBoard == V6)
//	{
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// LED full intensity
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0x00); // LED full intensity
//	}
//
//	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
//	// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
//	if(gABoard >= AV6_1)
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//	else
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
//
//	// Toggle Reset Pin
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
//	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
//	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//}
//#endif

//**************************************************************************
// Initializes digital board components separately from analog board, allows
// analog board to be set up when needed but not on power-up, worried that
// as battery gets low analog board power flickers and damages sensors
// Created: 3/22/2019
// Parameters:  Valve_home, sets whether to find the home position of valve
//				on power-up
//**************************************************************************
void InitDigital(uint8_t Valve_home)
{
	// Initialize all modules
	InitGPIOPins();		// Configures GPIO pins for use with button, LEDs, etc

	// Created mechanism to determine if we are trying to wake from hibernate multiple times, if we are...
	// this indicates a problem so clear the flag and boot as if reset
	if(HibernateIsActive() == 1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)	// If we are waking from hibernate
	{
		uint32_t Hibernating;
		EEPROMRead((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);

		if(Hibernating == 1)
		{
			Hibernating = 0;
			EEPROMProgram((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);
		}
		else
		{
			InitHibernate(); 	// GPIO States are held, wakes on wake pin
		}
	}

	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
	{
#ifdef MCU_ZXR
		GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, IO_RESET_BT_PIN); // Reset BT at beginning to ensure proper boot sequence
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
		GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, 0x00); // Reset BT at beginning to ensure proper boot sequence
#else //MCU_ZXR
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT at beginning to ensure proper boot sequence
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT at beginning to ensure proper boot sequence
#endif
	}

	InitFPU();			// Lazy stacking setup for interrupt handlers
	InitTimer();		// Configures two timers and interrupts for both
	InitConsole();		// Sets up UART communication to terminal program

	InitSPI();			// Sets up one master port and one slave port with interrupt
	InitI2C();			// Enables I2C port as master and sets up pins
	InitBT();			// Verifies the BT chip is running and has finished I2C setup
	InitEEPROM();		// Turns on EEPROM peripheral and initializes it to check for errors

	// LED extender and LEDs need "analog on" power to operate
//	InitLED_Ext();
//
//	SetLED(BLUE_BUTTON, 1);

	// TODO: Initialize cartridge here once switch that connects cartridge memory is always on
	InitCartridge();	// Checks if there is a cartridge present (memory) and update BT on whether there is one or not
	InitBattery();		// Configures battery IC to alert, turns on initial LEDs and updates BT on battery status
	InitCPUTemp();		// Initializes ADC to read temperature select channel

	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If we are not waking from hibernate
	{
		//		update_Test(FindTestNumber());
		//		update_Cal(FindCalNumber());
		update_Auto_Cal();
	}

	InitSteppers(Valve_home);		// Sets up pins to control steppers and finds valve home position
	InitHibernate(); 	// GPIO States are held, wakes on wake pin
}

//**************************************************************************
// Turns on power to and initializes analog board components, allows
// powering up analog board separately from digital so it can be turned on
// only when needed
// Created: 3/22/2019
// Parameters:  NONE
//**************************************************************************
void InitAnalog(void)
{
#ifdef MCU_ZXR
	if(gBoard == V6_2 || gBoard == V6_3)
	{
		// Need to turn off LED rst and CS pins so LED extender doesn't hold +3.3V motor up
		// Drop analog reset low so parts on analog board don't hold +3.3V up when power is off
		GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B
		GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
	}

	// Set pin low, turn on analog board, then set high
	GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B

	SysCtlDelay(SysCtlClockGet()/3000);	// Wait a ms after dropping cond ADC CNV pin _B before turning on analog board... may not be necessary, just trying

	GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

	userDelay(100, 0);

	GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

	// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
	if(gABoard >= AV6_1)
		GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN); // Conductivity ADC CNV Pin
	else
		GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin

	// Toggle Reset Pin
	GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
	GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Set high for normal operation
	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms

	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

	if(gBoard == V6_2 || gBoard == V6_3)
	{
		// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
		GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
		GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
	}
#else
	if(gBoard == V6_2 || gBoard == V6_3)
	{
		// Need to turn off LED rst and CS pins so LED extender doesn't hold +3.3V motor up
		// Drop analog reset low so parts on analog board don't hold +3.3V up when power is off
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
	}

	// Set pin low, turn on analog board, then set high
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B

	SysCtlDelay(SysCtlClockGet()/3000);	// Wait a ms after dropping cond ADC CNV pin _B before turning on analog board... may not be necessary, just trying

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

	userDelay(100, 0);

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

	// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
	if(gABoard >= AV6_1)
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
	else
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin

	// Toggle Reset Pin
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms

	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

	if(gBoard == V6_2 || gBoard == V6_3)
	{
		// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
	}
#endif	// MCU_ZXR

	InitIO_Ext();		// Sets up IO Extenders and sets initial values

	if(gBoard == V7)
	{
//		InitLED_Ext();
		SetLED(0, 1);	// Reset LEDs to the state they were when analog board was reset
	}
	if(gBoard == V6_4)
	{
	    // If any of the LEDs are blinking turn on the oscillator
	    if((LED_BLINK & gLED_State) != 0)
	    {
#ifdef MCU_ZXR
			GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B
			SysCtlDelay(SysCtlClockGet()/3000000);
			GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, IO_LED_OSC_RST_B_PIN); // LED Osc RST _B
#else
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
			SysCtlDelay(SysCtlClockGet()/3000000);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
#endif
	    }
	}
	else if(gBoard == V6_2 || gBoard == V6_3)
		InitLED_Ext();
	else if(gBoard < V6_2)
		SetLED(0, 1);

	InitDAC();				// Resets device and writes configuration register
	InitTurbidityADC();		// Configures ADC that checks conductivity frequency and will read turbidity
	InitADC();				// Resets devices and configures all channels
	InitWaveGen(0);			// Sets up waveform generator to output at 1 kHz
}

//**************************************************************************
// Turns off power to analog board
// Created: 3/22/2019
// Parameters:  NONE
//**************************************************************************
void AnalogOff(void)
{
#ifdef MCU_ZXR
	// Turn off power to analog board
	if(gBoard == V6_4)
	{
//		// Set pin low turn on analog board, then set high
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B

		// Don't need to change LED extender pins since it will be connected to always on +3.3V
		GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0x00); // LED Osc RST _B
	}
	else if(gBoard >= V6_2 && gBoard < V6_4)
	{
		// Need to turn off LED rst and CS pins so LED extender doesn't hold +3.3V motor up
		// Drop analog reset low so parts on analog board don't hold +3.3V up when power is off
		GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, 0); // LED Ext CS _B
		GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, 0); // LED Ext RST _B
	}

	GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
	GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, 0x00); // Analog On

//	userDelay(5000, 1);	// Delay 5 seconds to allow +5V and +3.3V power supplies to discharge
#else
	// Turn off power to analog board
	if(gBoard == V6_4)
	{
//		// Set pin low turn on analog board, then set high
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B

		// Don't need to change LED extender pins since it will be connected to always on +3.3V
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00); // LED Osc RST _B
	}
	else if(gBoard >= V6_2 && gBoard < V6_4)
	{
		// Need to turn off LED rst and CS pins so LED extender doesn't hold +3.3V motor up
		// Drop analog reset low so parts on analog board don't hold +3.3V up when power is off
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0); // LED Ext CS _B
		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0); // LED Ext RST _B
	}

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00); // Analog On

//	userDelay(5000, 1);	// Delay 5 seconds to allow +5V and +3.3V power supplies to discharge
#endif
}

////**************************************************************************
//// Initializes the GPIO pins not used for communication or steppers
//// GPIO Pins will retain state during hibernate
//// Device will wake when wake pin is asserted (pulled low)
//// Parameters:  NONE
////**************************************************************************
//void InitGPIOPins(void)
//{
//	// Enable GPIO Peripherals
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//
//	SysCtlDelay(300); // Delay so peripherals have time to power up
//
//	// Set up direction and strength of GPIO Pins
//	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN );						// I2C used by BT
//	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );						// I2C used by TIVA
//	GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Red charging LED
//	GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Green charging LED
//	GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Buzzer
//	GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT );						// Green button LED
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// Red button LED
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT );						// Conductivity ADC CNV Pin
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT );						// Reset BT
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN );						// Acc Int 1
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN );						// Acc Int 2
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN );						// Battery charging
//	GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN );						// Battery alert
//	GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_DIR_MODE_OUT );						// Orange charging LED
//	GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//	GPIODirModeSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_DIR_MODE_OUT );						// Blue button LED
//	GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	GPIODirModeSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_DIR_MODE_IN );						// Button
//	GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
////	SysCtlDelay(SysCtlClockGet()/3000);
//
//	// Initial Values
//	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0x00); // Red charging LED
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // Green charging LED
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x00); // Buzzer
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // Green button LED
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00); // Red button LED
//	if(gABoard >= AV6_1)
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//	else
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT
//	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00); // Orange charging LED
//	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00); // Blue button LED
//
//	if(gBoard >= V5)
//	{
//		GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_DIR_MODE_OUT );						// Analog On
//		GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//	}
//	if(gBoard >= V6)
//	{
//		GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT );						// LED full intensity
//		GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0x00); // LED full intensity
//	}
//
//	SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//}

////**************************************************************************
//// Enables Reset pin turning it low, then high and leaving it there
//// Parameters: NONE
////**************************************************************************
//void InitReset(void)
//{
////	// Configure Reset Pin
////	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//
//	// Unlock PD7 pin so pin can be controlled
//	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
//	HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;
//
//	GPIODirModeSet( GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_DIR_MODE_OUT );
//	GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_7,GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD );
//
//	// Toggle Reset Pin
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
//	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset all analog components
//	SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
//	SysCtlDelay(SysCtlClockGet()/3000 * 8);	// Delay 8ms
//}

//**************************************************************************
// Enable Floating Point Unit and set lazy stacking so that interrupt
// handlers only save floating point to stack when they are used
// Parameters: NONE
//**************************************************************************
void InitFPU(void)
{
	FPULazyStackingEnable();
	FPUEnable();
}

//**************************************************************************
// Enables and configures Timer0 and WTimer0
// Timer0 is a 32-bit timer, interrupt sets interrupt flag
// WTimer0 is a 64-bit timer, interrupt puts the device in hibernate mode
// Parameters: NONE
//**************************************************************************
void InitTimer()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);		// 32-bit timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);		// 32-bit timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);		// 32-bit timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);		// 64-bit timer for hibernate
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);		// 64-bit timer for cartridge temperature recording
	TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT);
	TimerConfigure(WTIMER1_BASE, TIMER_CFG_PERIODIC);

	// Enable interrupts for both timers
	IntMasterEnable();
	IntEnable(INT_TIMER0A);
	IntEnable(INT_TIMER1A);
	IntEnable(INT_TIMER2A);
	IntEnable(INT_WTIMER0A);
	IntEnable(INT_WTIMER1A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Change interrupt priority so we can communicate with BT right before entering hibernate
	IntPrioritySet(INT_TIMER2A, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits
	IntPrioritySet(INT_WTIMER0A, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits
	IntPrioritySet(INT_WTIMER1A, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits
}

//**************************************************************************
// Interrupt Handler for Timer0A timeout, sets flag
// Parameters:  NONE
//**************************************************************************
void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	g_TimerInterruptFlag = 1; // Set flag variable
}

//**************************************************************************
// Interrupt Handler for Timer1A timeout, sets flag
// Parameters:  NONE
//**************************************************************************
void Timer1IntHandler(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	g_TimerPeriodicInterruptFlag = 1; // Set flag variable
}

//**************************************************************************
// Interrupt Handler for Timer1A timeout, sets flag
// Parameters:  NONE
//**************************************************************************
void TempRecordTimerIntHandler(void)
{
	TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
	g_TimerTemperatureInterruptFlag = 1; // Set flag variable
}

//**************************************************************************
// Initializes the Hibernate module
// GPIO Pins will retain state during hibernate
// Device will wake when wake pin is asserted (pulled low)
// Parameters:  NONE
//**************************************************************************
void InitHibernate(void)
{
	if(gBoard >= V6)
	{

		//		if(HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)	// If we are waking from hibernate read all data back
		//		{
		//			uint32_t Hibernate_Data[2];
		//			HibernateDataGet(Hibernate_Data, 2);
		//
		//			g_CurrentValvePossition = Hibernate_Data[0];
		//			g_MonoCl = Hibernate_Data[1];
		//		}

		// Configure Hibernate module to exit using WAKE pin
		SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
		HibernateEnableExpClk(SysCtlClockGet());
		HibernateGPIORetentionDisable();	// Must disable when returning from hibernate to regain control of GPIOs
		HibernateWakeSet(HIBERNATE_WAKE_PIN);

		HibernateIntEnable(HIBERNATE_INT_PIN_WAKE);

		HibernateIntClear(HIBERNATE_INT_PIN_WAKE);
	}
}

//**************************************************************************
// Interrupt Handler for WTimer0 timeout, puts device in hibernate mode
// Parameters:  NONE
//**************************************************************************
void HibernateTimeoutIntHandler(void)
{
	TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);

	if(gHibernate_ON)
	{
		DEBUG_PRINT(UARTprintf("Hibernate Timeout... \n");)
		TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
		SysCtlDelay(200);
		HibernateGPIORetentionEnable();

		// Turn off interrupts during hibernation
		TimerDisable(TIMER1_BASE, TIMER_A);
		TimerDisable(WTIMER0_BASE, TIMER_A);

		uint32_t Hibernating = 1;
		EEPROMProgram((uint32_t *) &Hibernating, OFFSET_HIBERNATE_FLAG, 4);

		//	uint32_t Hibernate_Data[2] = {g_CurrentValvePossition, g_MonoCl};

		//	HibernateDataSet(&g_CurrentValvePossition, 1);

		//	UARTprintf("Hibernate valve variables: \n");
		//	UARTprintf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n", g_CurrentValvePossition, g_CurrentValveStep, g_ValveIndex_FW,
		//			g_ValveADCAvg_FW, g_ValveIndex_BW, g_ValveADCAvg_BW, g_ValveDirection, g_ValveHigh);

//		uint32_t HibernateVariables[9] = {0,0,0,0,0,0,0,0,0};
		uint32_t HibernateVariables[8] = {0,0,0,0,0,0,0,0};
		HibernateVariables[0] |= g_CurrentValvePossition;
		HibernateVariables[1] |= g_CurrentValveStep;
		HibernateVariables[2] |= g_ValveIndex_FW;
		HibernateVariables[3] |= g_ValveADCAvg_FW;
		HibernateVariables[4] |= g_ValveIndex_BW;
		HibernateVariables[5] |= g_ValveADCAvg_BW;
		HibernateVariables[6] |= g_ValveDirection;
		HibernateVariables[7] |= g_ValveHigh;
//		HibernateVariables[8] = (gPump_Ratio * 1000);

		HibernateDataSet(HibernateVariables, 8);	// Hibernate module can hold upto 16 32-bit words

		update_Status(STATUS_IDLE, OPERATION_HIBERNATE);

//		// Turn off all LEDs while in hibernation
		SetLED(RED_BUTTON|RED_BUTTON_V|GREEN_BUTTON|GREEN_BUTTON_V|BLUE_BUTTON|BLUE_BUTTON_V|RED_CHARGE|GREEN_CHARGE|YELLOW_CHARGE, 0);	// Turn off all LEDs

		if(gBoard == V6)
		{
#ifdef MCU_ZXR
			GPIOPinWrite(IO_LED_FULL_INTENSITY_BASE, IO_LED_FULL_INTENSITY_PIN, 0x00); // LED full intensity
#else	//	MCU_ZXR
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0x00); // LED full intensity
#endif	//	MCU_ZXR
		}

//		// Put accelerometer in standby mode
//		I2CSend(I2C0_BASE, ACCEL_ADDR, 2, 0x2A, 0x2A);

		if(gBoard >= V6_2)
		{
			AnalogOff();
		}
		else if(gBoard >= V5)
		{
#ifdef MCU_ZXR
			GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, 0x00); // Analog On
#else	//	MCU_ZXR
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00); // Analog On
#endif	//	MCU_ZXR
		}
		else	//
		{
			// Turn off DAC
			// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0, 0x99, 0x80); // Set power down bits in configuration register

			// Turn off ADCs
			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x01); // Enter power-down mode
			SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x01); // Enter power-down mode
			SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x01); // Enter power-down mode
		}

		//	HibernateGPIORetentionEnable();

		DEBUG_PRINT(UARTprintf("Entering Hibernation \n");)
		userDelay(1, 0);

		// Put the device in hibernate mode
		HibernateRequest();
	}
}

#ifdef MEMORY_V3
//**************************************************************************
// Finds and returns the current test number by reading through memory to
// find next open space
// Parameters:	NONE
// Returns:		Test; Test number found
// 1/22/2019: Modified to return max test number found, this will allow
// 	saving new test data over old test data when out of room in memory
//**************************************************************************
uint8_t FindTestNumber(void)
{
	uint8_t *pui8TestByte;
	uint8_t Test = 1;
	uint8_t Max_test = 0;

	pui8TestByte = MemoryRead(PAGE_TEST_INFO, OFFSET_TEST_NUMBER, 1);

	while(!(*pui8TestByte == 0xff) && Test < 125)
	{
		if(*pui8TestByte > Max_test)
			Max_test = *pui8TestByte;
		pui8TestByte = MemoryRead(PAGE_TEST_INFO + PAGES_FOR_TEST * Test, OFFSET_TEST_NUMBER, 1);
		Test++;
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay to give time between consecutive reads
	}

	return Max_test;

//	return (Test - 1);
}

//**************************************************************************
// Finds and returns the current calibration number by reading through memory
// to find next open space
// Parameters:	NONE
// Returns:		Calibration; Calibration number found
//**************************************************************************
uint8_t FindCalNumber(void)
{
	uint8_t *pui8CalByte;
	uint8_t Cal = 1;
	uint8_t Max_cal = 0;

	pui8CalByte = MemoryRead(PAGE_CAL_INFO, OFFSET_CAL_NUMBER, 1);

	while(!(*pui8CalByte == 0xff) && Cal < 46)
	{
		if(*pui8CalByte > Max_cal)
			Max_cal = *pui8CalByte;
		pui8CalByte = MemoryRead(PAGE_CAL_INFO + PAGES_FOR_CAL * Cal, OFFSET_CAL_NUMBER, 1);
		Cal++;
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay to give time between consecutive reads
	}

	return Max_cal;

//	return (Cal - 1);
}
#else
//**************************************************************************
// Finds and returns the current test number by reading through memory to
// find next open space
// Parameters:	NONE
// Returns:		Test; Test number found
// 1/22/2019: Modified to return max test number found, this will allow
// 	saving new test data over old test data when out of room in memory
// 3/26/2020: Changed test to be 16-bit to hold more than 255 tests and
// match specs for memory V4
//**************************************************************************
uint16_t FindTestNumber(void)
{
	uint16_t *pui16TestByte;
	uint16_t Test = 1;
	uint16_t Max_test = 0;

	pui16TestByte = (uint16_t *) MemoryRead(PAGE_TEST, OFFSET_TEST_NUMBER, 2);

	while(!(*pui16TestByte == 0xffff) && Test <= ((512 - PAGE_TEST) / PAGES_FOR_TEST))
	{
		if(*pui16TestByte > Max_test)
			Max_test = *pui16TestByte;
		pui16TestByte = (uint16_t *) MemoryRead(PAGE_TEST + PAGES_FOR_TEST * Test, OFFSET_TEST_NUMBER, 2);
		Test++;
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay to give time between consecutive reads
	}

	return Max_test;
}

//**************************************************************************
// Finds and returns the current calibration number by reading through memory
// to find next open space
// Parameters:	NONE
// Returns:		Calibration; Calibration number found
// 3/26/2020: Changed to 16-bit number to match specs for memory V4
//**************************************************************************
uint16_t FindCalNumber(void)
{
	uint16_t *pui16CalByte;
	uint16_t Cal = 1;
	uint16_t Max_cal = 0;

	pui16CalByte = (uint16_t *) MemoryRead(PAGE_CAL, OFFSET_CAL_NUMBER, 2);

	while(!(*pui16CalByte == 0xffff) && Cal <= ((PAGE_TEST - PAGE_CAL) / PAGES_FOR_CAL))
	{
		if(*pui16CalByte > Max_cal)
			Max_cal = *pui16CalByte;
		pui16CalByte = (uint16_t *) MemoryRead(PAGE_CAL + PAGES_FOR_CAL * Cal, OFFSET_CAL_NUMBER, 2);
		Cal++;
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay to give time between consecutive reads
	}

	return Max_cal;
}
#endif

//**************************************************************************
// Function to build floats from 4-bytes, assumes little endian format
// i.e. first byte is LSB and last byte is MSB
// Parameters:	*pui8bytes; pointer or array to bytes to be parsed; "pui8bytes"
// Returns:		fvalue; float number built inside function
//**************************************************************************
float Build_float(uint8_t *pui8bytes)
{
	float fvalue; // Create variable to store float value in
	uint8_t *pui8ptr;
	pui8ptr = (uint8_t *) &fvalue; // Create pointer and set it to address of fvalue

	// Copy bytes from pui8bytes into fvalue
	*pui8ptr = *pui8bytes;
	*(pui8ptr + 1) = *(pui8bytes + 1);
	*(pui8ptr + 2) = *(pui8bytes + 2);
	*(pui8ptr + 3) = *(pui8bytes + 3);

	return fvalue;
}

//**************************************************************************
// Function to request system status from BT
// Parameters:	NONE
// Returns:		SysStatus;
//**************************************************************************
uint8_t * RequestSystemStatus(void)
{
	int i;
	static uint8_t SysStatus[55];
	uint32_t ui32Rx;
	uint8_t Command_received = 0;
	uint32_t counter = 0;

//	// Disable SSI interrupt because this is a unique SSI transaction
//	IntDisable(INT_SSI0);

	while(Command_received == 0)
	{
		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, REQUEST_SYS_STATUS);
		SSIDataPut(SSI0_BASE, 0xFF);
		SSIDataPut(SSI0_BASE, 0xFF);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
			//			UARTprintf("Pinging BT...\n");
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // Tiva Rqst
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // Tiva Rqst
#else	// MCU_ZXR
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // Tiva Rqst
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // Tiva Rqst
#endif	// MCU_ZXR

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		if(Command_received == 1)
		{
			// Disable SSI interrupt because this is a unique SSI transaction
			IntDisable(INT_SSI0);

#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // Tiva RQST
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // Tiva RQST
#else	// MCU_ZXR
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif	// MCU_ZXR

			for(i = 0; i < 55; i++)
			{
				//		SSIDataGet(SSI0_BASE, &SysStatus[i]);
				SSIDataGet(SSI0_BASE, &ui32Rx);
				SysStatus[i] = ui32Rx & 0xFF;
			}

			SSIIntClear(SSI0_BASE, SSI_RXTO);
			IntEnable(INT_SSI0);

			g_ulSSI0RXTO = 0;

//			UARTprintf("%d/%d/%d%d %d:%d:%d\n", SysStatus[0], SysStatus[1], SysStatus[2], SysStatus[3], SysStatus[4], SysStatus[5], SysStatus[6]);
			//	// If there is no location data, fill with "Roam"
			//	if(SysStatus[35] == 0x00)
			//	{
			//		SysStatus[35] = 'R';//oam                ";
			//		SysStatus[36] = 'o';
			//		SysStatus[37] = 'a';
			//		SysStatus[38] = 'm';
			//		SysStatus[39] = ' ';
			//		SysStatus[40] = '(';
			//		SysStatus[41] = 'N';
			//		SysStatus[42] = 'o';
			//		SysStatus[43] = ' ';
			//		SysStatus[44] = 'L';
			//		SysStatus[45] = 'o';
			//		SysStatus[46] = 'c';
			//		SysStatus[47] = 'a';
			//		SysStatus[48] = 't';
			//		SysStatus[49] = 'i';
			//		SysStatus[50] = 'o';
			//		SysStatus[51] = 'n';
			//		SysStatus[52] = ')';
			//	}

			DEBUG_PRINT(
			if(gDiagnostics == 1)
			{
				UARTprintf("Requesting system status:\n");
				UARTprintf("Date: %d/%d/%d%d\n", SysStatus[0], SysStatus[1], SysStatus[2], SysStatus[3]);
				UARTprintf("Time: %d:%d:%d\n", SysStatus[4], SysStatus[5], SysStatus[6]);
				UARTprintf("UN: ");
				for(i = 0; i < 20; i++)
					UARTprintf("%c", SysStatus[7 + i]);
				int32_t GPS[2];
				memcpy(GPS, SysStatus + 27, 8);
				UARTprintf("\nGPS: %d, %d\n", GPS[0], GPS[1]);
				UARTprintf("LN: ");
				for(i = 0; i < 20; i++)
					UARTprintf("%c", SysStatus[35 + i]);
				UARTprintf("\n");
			}
			)
			return SysStatus;
		}
		else
		{
			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

			DEBUG_PRINT(UARTprintf("Request System Status not working! ");)
			Reset_BT();
			userDelay(1000, 1);
		}
	}

	return SysStatus;	// Will never reach here, put it here to clear compiler warning
}

//**************************************************************************
// Checks if Cartridge is expired (35 days) or has completed max tests
// Parameters:  *pui8SysStatus; System status update from BT returned from
//				RequestSystemStatus() function
// Returns:		NONE; Sets error in global error gui32Error
//**************************************************************************
void CheckCartridge(uint8_t *pui8SysStatus)
{
	// Create time variables to convert time in human readable format to UNIX
	struct tm curTime;
	uint8_t *Sensor_Max;		// Pointer to data for max number of tests, read from memory

	if(g_state == STATE_CALIBRATION)	// If we are running calibration check if we've exceeded max calibrations
	{
		uint8_t Cal = FindCalNumber();	// Find most recently completed calibration

#if defined EXPIRATION_DATE || defined REMAINING_DAYS_MAX_CALS
		uint8_t Max_No_Cals = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_CALS, 1);	// Read max number of tests from memory

		if(Cal > Max_No_Cals)
			gui32Error |= MAX_CALS_REACHED;
#else
		if(Cal >= MAX_CALS)
			gui32Error |= MAX_CALS_REACHED;
#endif
	}
	else if(g_state == STATE_MEASUREMENT)	// If we are running test check if we've exceeded max number of tests
	{
		uint8_t Test = FindTestNumber();		// Find number of most recent test
		Sensor_Max = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_TESTS, 2);	// Read max number of tests from memory

		uint16_t Max_No_Tests = *Sensor_Max | (*(Sensor_Max + 1) << 8); // Assemble data from pointer

		if(Test >= Max_No_Tests)	// Check the next test won't exceed max number of tests
		{
			//		g_error_byte_2 |= 0x01;
			//		return false;		// Fail cartridge if max number of tests exceeded
			gui32Error |= MAX_TESTS_REACHED;
		}
	}
	else	// If idle or any state not test or cal check both test and calibration
	{
		uint8_t Test = FindTestNumber();		// Find number of most recent test
		Sensor_Max = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_TESTS, 2);	// Read max number of tests from memory

		uint16_t Max_No_Tests = *Sensor_Max | (*(Sensor_Max + 1) << 8); // Assemble data from pointer

		if(Test >= Max_No_Tests)	// Check the next test won't exceed max number of tests
		{
			//		g_error_byte_2 |= 0x01;
			//		return false;		// Fail cartridge if max number of tests exceeded
			gui32Error |= MAX_TESTS_REACHED;
		}

		uint8_t Cal = FindCalNumber();	// Find most recently completed calibration
#if defined EXPIRATION_DATE || defined REMAINING_DAYS_MAX_CALS
		uint8_t Max_No_Cals = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_CALS, 1);	// Read max number of tests from memory

		if(Cal > Max_No_Cals)
			gui32Error |= MAX_CALS_REACHED;
#else
		if(Cal >= MAX_CALS)
			gui32Error |= MAX_CALS_REACHED;
#endif
	}

	curTime.tm_mon = *pui8SysStatus - 1;
	curTime.tm_mday = *(pui8SysStatus + 1);
	curTime.tm_year = *(pui8SysStatus + 2) * 100 + *(pui8SysStatus + 3) - 1900;
	curTime.tm_hour = *(pui8SysStatus + 4);
	curTime.tm_min = *(pui8SysStatus + 5);
	curTime.tm_sec = *(pui8SysStatus + 6);

	uint32_t Date = mktime(&curTime);	// Get current time in seconds since 1900

#ifdef EXPIRATION_DATE
	uint8_t *Sensor_Expiration;	// Pointer to data for expiration date, read from memory
	struct tm expTime;

	Sensor_Expiration = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_EXPIRATION_DATE, 4); // Read expiration date from memory

	// Fill in time structure with data read from memory
	expTime.tm_mon = *Sensor_Expiration - 1;	// Month [0,11]
	expTime.tm_mday = *(Sensor_Expiration + 1); // Day [1,31]
	expTime.tm_year = (*(Sensor_Expiration + 2) * 100) + *(Sensor_Expiration + 3) - 1900;	// Years since 1900
	expTime.tm_hour = 0;
	expTime.tm_min = 0;
	expTime.tm_sec = 0;

	uint32_t Expiration_Date = mktime(&expTime);	// Convert expiration from human readable to Epoch time (seconds since 1900)

	if(Date > Expiration_Date) // 35 days is the max life of the cartridge, this is 30 days + shipping
	{
		//		g_error_byte_2 |= 0x01;
		//		return false;
		gui32Error |= CARTRIDGE_EXPIRED;
	}
#else

	uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
	uint8_t Max_Days = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_DAYS, 1);
	uint32_t Cartridge_life;
	if(Max_Days == Max_Days)
	{
		Cartridge_life = 3600 * 24 * Max_Days;	// Standard cartridge lasts for 35 days (30 + 5 for shipping)
	}
	else
	{
		Cartridge_life = 3600 * 24 * 35;	// Standard cartridge lasts for 35 days (30 + 5 for shipping)
		if(Sensor_Config == PH_CL_CART)
			Cartridge_life = 3600 * 24 * 65; // pH Cl cartridge lasts for 65 days (60 + 5 for shipping)
	}

	uint8_t *Sensor_Hydration;	// Pointer to data for hydration date, read from memory
	struct tm hydTime;

	Sensor_Hydration = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_HYDRATION_DATE, 4); // Read hydration date from memory
	// Fill in time structure with data read from memory
	hydTime.tm_mon = *Sensor_Hydration - 1;	// Month [0,11]
	hydTime.tm_mday = *(Sensor_Hydration + 1); // Day [1,31]
	hydTime.tm_year = (*(Sensor_Hydration + 2) * 100) + *(Sensor_Hydration + 3) - 1900;	// Years since 1900
	hydTime.tm_hour = 0;
	hydTime.tm_min = 0;
	hydTime.tm_sec = 0;

	//	UARTprintf("Hydration date: %d/%d/%d \n", (int) hydTime.tm_mon, (int) hydTime.tm_mday, (int) hydTime.tm_year);

	uint32_t Hydration_Date = mktime(&hydTime);	// Convert last cal time from human readable to Epoch time (seconds since 1900)

	int Seconds_passed = Date - Hydration_Date;	// Calculate how many days have passed since hydration

	// 8/14/2023 if Seconds_passed is < 0 then hydration date must be in the future.. in which case allow it to run as the first calibration will set a correct time
	if(Seconds_passed > Cartridge_life && Seconds_passed > 0) // 35 days is the max life of the cartridge, this is 30 days + shipping
	{
		//		g_error_byte_2 |= 0x01;
		//		return false;
		gui32Error |= CARTRIDGE_EXPIRED;
	}
#endif

	//	g_error_byte_2 &= 0xFE;
	//	return ui32Error; // Return true if tests are less than max and 30 days haven't elapsed
}

//**************************************************************************
// Checks if Calibration is current
// Parameters:  *pui8SysStatus; System status update from BT returned from
//				RequestSystemStatus() function
//				Acceptable_Days; Days since calibration for it to be current
// Returns:		true if calibration is current and valid
//				false if calibration is not current nor valid
//**************************************************************************
bool CheckCalibration(uint8_t *pui8SysStatus, uint8_t Acceptable_Days)
{
	uint8_t *Prev_Cal_Date; // Pointer to previous calibration date, read from memory

	// Create time variables to convert time in human readable format to UNIX
	struct tm curTime;
	struct tm calTime;

	// Check if calibration is current
	int Cal = FindCalNumber();	// Find number for last calibration
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Cal: %d \n", Cal);
	)

	if(Cal >= 1)	// Check that at least one calibration has been done before
	{
		// Write calibration data to memory so BT chip can transmit it
#ifdef MEMORY_V3
		uint16_t Cal_page = (PAGE_CAL_INFO + Cal * PAGES_FOR_CAL) - PAGES_FOR_CAL;
		while(Cal_page > (PAGE_TEST_INFO - PAGES_FOR_CAL))
			Cal_page -= (PAGE_TEST_INFO - PAGE_CAL_INFO);

		uint8_t * Cal_Status = MemoryRead(Cal_page + 1, OFFSET_CAL_STATUS, 1);
#else
		uint16_t Cal_page = Find_Cal_page(Cal);

		uint8_t * Cal_Status = MemoryRead(Cal_page, OFFSET_CAL_STATUS, 1);
#endif

		if(*Cal_Status & 1)	// Check that calibration passed
		{
			Prev_Cal_Date = MemoryRead(Cal_page, OFFSET_CAL_DATE, 7); // Read previous cal date

			// Fill in time structure with data read from memory
			calTime.tm_mon = *Prev_Cal_Date - 1;	// Month [0,11]
			calTime.tm_mday = *(Prev_Cal_Date + 1); // Day [1,31]
			calTime.tm_year = (*(Prev_Cal_Date + 2) * 100) + *(Prev_Cal_Date + 3) - 1900;	// Years since 1900
			calTime.tm_hour = *(Prev_Cal_Date + 4);
			calTime.tm_min = *(Prev_Cal_Date + 5);
			calTime.tm_sec = *(Prev_Cal_Date + 6);

			if(gDiagnostics >= 1)
			{
				DEBUG_PRINT(UARTprintf("Cal date: %d/%d/%d \n", (int) calTime.tm_mon, (int) calTime.tm_mday, (int) calTime.tm_year);)
				DEBUG_PRINT(UARTprintf("Cal time: %d:%d:%d \n", (int) calTime.tm_hour, (int) calTime.tm_min, (int) calTime.tm_sec);)
			}


			uint32_t Prev_Date = mktime(&calTime);	// Get date of last cal in seconds since 1900
			//		uint32_t Prev_Date = *Prev_Cal_Date | (*(Prev_Cal_Date + 1) << 8) | (*(Prev_Cal_Date + 2) << 16) | (*(Prev_Cal_Date + 3) << 24); // Assemble previous calibration date from pointer

			curTime.tm_mon = *pui8SysStatus - 1;
			curTime.tm_mday = *(pui8SysStatus + 1);
			curTime.tm_year = *(pui8SysStatus + 2) * 100 + *(pui8SysStatus + 3) - 1900;
			curTime.tm_hour = *(pui8SysStatus + 4);
			curTime.tm_min = *(pui8SysStatus + 5);
			curTime.tm_sec = *(pui8SysStatus + 6);

			if(gDiagnostics >= 1)
			{
				DEBUG_PRINT(UARTprintf("Cur date: %d/%d/%d \n", (int) curTime.tm_mon, (int) curTime.tm_mday, (int) curTime.tm_year);)
		DEBUG_PRINT(UARTprintf("Cur time: %d:%d:%d \n", (int) curTime.tm_hour, (int) curTime.tm_min, (int) curTime.tm_sec);)
			}


			uint32_t Date = mktime(&curTime);	// Get current date in seconds since 1900
			//		uint32_t Date = *pui8SysStatus | (*(pui8SysStatus + 1) << 8) | (*(pui8SysStatus + 2) << 16) | (*(pui8SysStatus + 3) << 24); // Assemble current date from pointer

			uint32_t Seconds_passed = Date - Prev_Date; // Calculate how many days have passed

			if(Seconds_passed < (86400 * (uint32_t) Acceptable_Days)) // Calibrate everyday, new day invalidates calibration
				return true;
		}
	}

	return false; // Fail calibration if never been done or at least a day has passed
}

//**************************************************************************
// Gets the absolute value of a float number and returns it
//**************************************************************************
float abs_val(float number)
{
	if(number < 0)
		number *= -1;

	return number;
}

////**************************************************************************
//// Finds the temperature compensation coefficient for a given temperature
//// Parameters:	T_rinse; Temperature of rinse solution
//// Returns:		Rinse_Table_K_T
////**************************************************************************
//float Rinse_Table_Lookup(float T_rinse)
//{
//	int Rinse_Table_T[11] = {7, 11, 15, 19, 23, 27, 32, 35, 39, 43, 45};
//	float Rinse_Table_K_T[11] = {0.161, 0.132, 0.092, 0.064, 0.027, 0, -0.022, -0.044, -0.060, -0.075, -0.084};
//
//	int i;
//	for(i = 0; i < 11; i++)
//	{
//		if(T_rinse < Rinse_Table_T[i])
//			return Rinse_Table_K_T[i];
//	}
//	return Rinse_Table_K_T[10];
//}

//**************************************************************************
// Initializes on-chip EEPROM
// Parameters:	NONE
// Returns:		NONE
//**************************************************************************
void InitEEPROM(void)
{
	uint32_t ui32EEPROMInit;
	//
	// Enable the EEPROM module.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	//
	// Wait for the EEPROM module to be ready.
	//
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
	{
	}

	//
	// Wait for the EEPROM Initialization to complete
	//
	ui32EEPROMInit = EEPROMInit();

	uint8_t attempts = 0;
	while(ui32EEPROMInit != EEPROM_INIT_OK && attempts < 3)
	{
		SysCtlDelay(SysCtlClockGet()/3);	// Wait 1 second then try to initialize again
		ui32EEPROMInit = EEPROMInit();
		attempts++;
	}

	if(ui32EEPROMInit != EEPROM_INIT_OK && attempts == 3)
	{
		DEBUG_PRINT(UARTprintf("Tiva EEPROM failed to recover from an interrupted write or erase operation\n");)
		gui32Error |= EEPROM_FAIL;
//		update_Error();	// Can't update here, must initialize EEPROM before BT so Tiva can check auto_cal/Monochloramine cycles. Instead have BT update error as soon as it's ready
	}
}

//**************************************************************************
// Calibrate temperature using known resistor value, this function runs
// automatically when a test board is plugged in with the correct data in
// its memory, save calibration to EEPROM of digital board
// Parameters:	NONE
// Returns:		NONE
//**************************************************************************
void CalibrateTemperature(void)
{
	float R_known = 1000;//*((uint16_t *) MemoryRead(PAGE_FACTORY_CAL, 2, 2));

	if(R_known != 0xFFFF)
	{
		DEBUG_PRINT(UARTprintf("Calibrating Temperature at %d Ohms!\n", (int) (R_known));)

		float Resistance;
		if(gABoard < AV6)
		{
			ADCCurrentSet(1);
			SysCtlDelay(SysCtlClockGet());	// Delay # of seconds

			Resistance = ADCReadAvg(TEMP_CH, TEMP_ADC, 5);	// Ohms = V mV / 1 mA

			ADCCurrentSet(0);
		}
		else
		{
			DACVoltageSet(4, 1500, true);
			SysCtlDelay(SysCtlClockGet()/3);

			Resistance = ADCReadAvg(TEMP_CH, TEMP_ADC, 5) - ADCReadAvg(TEMP_CH_2, TEMP_ADC, 5);

			DACVoltageSet(4, 3000, true);
		}

		float Current_Coefficient = R_known / Resistance;
		DEBUG_PRINT(UARTprintf("Coefficient found: %d / 1000\n", (int) (Current_Coefficient * 1000));)

		if(Current_Coefficient <= 1.02 && Current_Coefficient >= 0.98)
		{
			DEBUG_PRINT(UARTprintf("Coefficient is reasonable, saving coefficient to memory!\n");)
			EEPROMProgram((uint32_t *) &Current_Coefficient, OFFSET_TEMP_CAL, 4);
		}
	}
	else
	{
		DEBUG_PRINT(UARTprintf("Temperature resistance not known, not calibrating temperature current coefficient!\n");)
	}
}

//**************************************************************************
// Measures temperature by setting current and calculating resistance then
// converting resistance to temperature according to quadratic relationship
// Parameters:	seconds; time to leave current on before taking measurement
// Returns:		Temp; Calculated temperature of sensor
//**************************************************************************
float MeasureTemperature(int seconds)
{
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		float Resistance, Temp;
		float A, B, C;
		uint8_t *pui8TempCoefficients;
		float Current_Coefficient = 1;
//		EEPROMRead((uint32_t *) &Current_Coefficient, OFFSET_TEMP_CAL_HIGH_PREC, 4);
//
//		if(Current_Coefficient != Current_Coefficient)
//		{
//			EEPROMRead((uint32_t *) &Current_Coefficient, OFFSET_TEMP_CAL, 4);
//			Current_Coefficient *= 1002/1000;
//		}

		EEPROMRead((uint32_t *) &Current_Coefficient, OFFSET_TEMP_CAL, 4);

		if(Current_Coefficient != Current_Coefficient)
			Current_Coefficient = 1;

		if(gABoard < AV6)
		{
			ADCCurrentSet(1);
			SysCtlDelay(1 + SysCtlClockGet()/3 * seconds);	// Delay # of seconds

			Resistance = ADCReadAvg(TEMP_CH, TEMP_ADC, 5);	// Ohms = V mV / 1 mA

			ADCCurrentSet(0);
		}
		else
		{
			DACVoltageSet(4, 1500, true);
			SysCtlDelay(1 + SysCtlClockGet()/3 * seconds);

			Resistance = (ADCReadAvg(TEMP_CH_2, TEMP_ADC, 5) - ADCReadAvg(TEMP_CH, TEMP_ADC, 5)) * Current_Coefficient;

			DACVoltageSet(4, 3000, true);
		}

		pui8TempCoefficients = MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_COEFFICIENT_A, 12);
		A = Build_float(pui8TempCoefficients);
		B = Build_float(pui8TempCoefficients + 4);
		C = Build_float(pui8TempCoefficients + 8);

		if(A == A && B == B && C == C)	// Verify there is calibration data stored in chip
		{
			// Temperature is quadratic relationship T = A * R^2 + B * R + C
			Temp = A * Resistance * Resistance + B * Resistance + C;
		}
		else
			Temp = T_ASSUME;

		//	UARTprintf("Temperature calculated \t %d \t C * 1000 \n", (int) (Temp * 1000));

		// Check make sure we have a valid temperature
		if(Temp == Temp)
		{
			if(Temp > 55 || Temp <= 0)	// TODO: Change these numbers to expand temperature sensor range
				Temp = T_ASSUME;
		}
		else
		{
			Temp = T_ASSUME;
		}

		return Temp;

		//	return Temp;
		//	return T_ASSUME;
	}

	return 25;
}

//*****************************************************************************
//
// Passes control to the bootloader and initiates a remote software update.
//
// This function passes control to the bootloader and initiates an update of
// the main application firmware image via UART0 or USB depending
// upon the specific boot loader binary in use.
//
// \return Never returns.
//
//*****************************************************************************
void JumpToBootLoader(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Going to Boot Loader \n");
	)

    //
    // Disable all processor interrupts.  Instead of disabling them
    // one at a time, a direct write to NVIC is done to disable all
    // peripheral interrupts.
    //
    HWREG(NVIC_DIS0) = 0xffffffff;
    HWREG(NVIC_DIS1) = 0xffffffff;

    //
    // Return control to the boot loader.  This is a call to the SVC
    // handler in the boot loader.
    //
    (*((void (*)(void))(*(uint32_t *)0x2c)))();
}

//**************************************************************************
// Compare function to be used in sorting function to find median, sorts
// values from smallest to largest
// Taken from: https://stackoverflow.com/questions/3886446/problem-trying-to-use-the-c-qsort-function
//**************************************************************************
int cmpfunc (const void * a, const void * b)
{
	float fa = *(const float*) a;
	float fb = *(const float*) b;
	return (fa > fb) - (fa < fb);
}

//**************************************************************************
// Finds the median value of an array, array must have length of 9 or less
// Parameters:	array; given array to find median
// 				size; length of array to sort through
// Returns:		Median; median value of array
//**************************************************************************
float FindMedian(float * array, unsigned int size)
{
	float Sorted_array[9];
	uint8_t i;
	for(i = 0; i < size; i++)
	{
		Sorted_array[i] = array[i];
	}

	qsort(Sorted_array, size, 4, cmpfunc); // array to sort, length of array, # of bytes for each element, compare function

	// If there are an odd number of points then return middle point
	if((size & 1) == 1)
		return Sorted_array[(size/2)];
	else	// If there are an even number of points then return the average the two middle points
		return (Sorted_array[size/2] + Sorted_array[size/2 - 1]) / 2;
}

//**************************************************************************
// Calculates the standard deviation of an array of floats using population
// standard deviation calculation sd = sqrt(sum((x_i - avg)^2)/N)
// Parameters:	array; given array to find standard deviation
// 				size; length of array to sort through
// Returns:		Median; median value of array
//**************************************************************************
float FindStdDev(float * array, uint8_t size)
{
	float Avg = 0;

	uint8_t i;
	for(i = 0; i < size; i++)
	{
		Avg += array[i];
	}
	Avg /= size;

	float Std_Dev = 0;
	for(i = 0; i < size; i++)
	{
		Std_Dev += ((array[i] - Avg)*(array[i] - Avg));
	}
	Std_Dev /= size;

	return powf(Std_Dev,0.5);
}

////**************************************************************************
//// Calculates the standard deviation of an array of floats using population
//// standard deviation calculation sd = sqrt(sum((x_i - avg)^2)/N)
//// Parameters:	EEPROM_addr; Location in EEPROM the first point is stored;
////					assume each subsequent point is stored 36 bytes later
////					in memory, 36 = 4(bytes/float) * 9(ISE sensors)
//// 				size; number of points to use in calculation
//// Returns:		Std_Dev; standard deviation of data
////**************************************************************************
//float FindStdDevISEs(uint32_t EEPROM_addr, uint8_t size)
//{
//	float Avg = 0;
//	float Holder;
//
//	uint8_t i;
//	for(i = 0; i < size; i++)
//	{
//		EEPROMRead((uint32_t *) &Holder, EEPROM_addr + (36 * i), 4);
//		Avg += Holder;
//	}
//	Avg /= size;
//
//	float Std_Dev = 0;
//	for(i = 0; i < size; i++)
//	{
//		EEPROMRead((uint32_t *) &Holder, EEPROM_addr + (36 * i), 4);
//		Std_Dev += ((Holder - Avg)*(Holder - Avg));
//	}
//	Std_Dev /= size;
//
//	return powf(Std_Dev,0.5);
//}

//**************************************************************************
// Initializes the ADC sequence to find the CPU temperature
// Parameters:	NONE
// Returns:		NONE
//**************************************************************************
void InitCPUTemp(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);			// Enable ADC1

	ADCHardwareOversampleConfigure(ADC1_BASE, 64);		// Turns on hardware oversampling
	ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 1);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 2, ADC_CTL_TS);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 3, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);

	ADCSequenceEnable(ADC1_BASE, 1);
	ADCIntClear(ADC1_BASE, 1);
}

//**************************************************************************
// Gets the CPU Temperature by sampling the ADC temperature select channel
// and using the equation given in the TM4C123GH6PM Data Sheet pg 813
// Parameters:	NONE
// Returns:		CPUTemp; Temperature of the CPU
//**************************************************************************
float GetCPUTemp(void)
{
	uint32_t ADC1Value[4];

	DEBUG_PRINT(
	if(gDiagnostics == 1)
		UARTprintf("Reading CPU Temp...\n");
	)

	ADCProcessorTrigger(ADC1_BASE, 1);									// Trigger ADCs to get value from temp sensor
	while(!ADCIntStatus(ADC1_BASE, 1, false)){}							// Wait for conversion to finish
	ADCIntClear(ADC1_BASE, 1);											// Clear interrupt vector
	ADCSequenceDataGet(ADC1_BASE, 1, ADC1Value);						// Get data from ADC
	float ADC1ValueAvg = ((float) (ADC1Value[0] + ADC1Value[1] + ADC1Value[2] + ADC1Value[3]))/4;

//	UARTprintf("ADC Value: %d \n", ADC1Value);

	float CPUTemp = 147.5 - ((75 * 3.3 * ADC1ValueAvg) / 4096);
//	UARTprintf("CPU Temperature: %d C * 1000 \n", (int) (CPUTemp * 1000));

	return CPUTemp;
}

//**************************************************************************
// Checks of there is a cartridge present and updates the BT chip
// 1/3/2019: Modified to work with digital board V6.2
//**************************************************************************
void InitCartridge(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Init Cartridge \n");
	)

	if(gBoard >= V6 && gABoard >= AV5)
	{


#ifdef MCU_ZXR
		GPIODirModeSet(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard), GPIO_DIR_MODE_IN);						// Cartridge present _B
		GPIOPadConfigSet(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
		GPIOPinTypeGPIOInput(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard)); // Cartridge_present_B
		GPIOIntTypeSet(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard), GPIO_BOTH_EDGES);
		GPIOIntEnable(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard));
		GPIOIntClear(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard));	// GPIO_INT_PIN_5 is = to GPIO_PIN_5
		IntEnable(INT_CART_PRES_B);

		IntPrioritySet(INT_CART_PRES_B, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits

		if(GPIOPinRead(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard)) == IO_CART_PRES_B_PIN(gBoard))	// Signal is high, cartridge not connected
#else
		uint8_t Signal_pin;

		if(gBoard < V6_2)
		{
			GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN );						// Cartridge present _B
			GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
			GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0); // Cartridge_present_B
			GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
			GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
			GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
			IntEnable(INT_GPIOB);

			IntPrioritySet(INT_GPIOB, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits

			Signal_pin = GPIO_PIN_0;
		}
		else if(gBoard >= V6_2)
		{
			GPIODirModeSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN );						// Cartridge present _B
			GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
			GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5); // Cartridge_present_B
			GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);
			GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
			GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
			GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
			IntEnable(INT_GPIOB);

			IntPrioritySet(INT_GPIOB, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits

			Signal_pin = GPIO_PIN_5;
		}

		if(GPIOPinRead(GPIO_PORTB_BASE, Signal_pin) == Signal_pin)	// Signal is high, cartridge not connected
#endif
		{
			gCartridge = 0;
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Cartridge missing! \n");
			)
		}
		else
		{
			gCartridge = 1;
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Cartridge present! \n");
			)

			// Check if device has programmed in device information, if it does write to memory for BT
			unsigned char Man_Name[20];
			EEPROMRead((uint32_t *) &Man_Name, OFFEST_MANUFACTURER_NAME, 20);
			if(Man_Name[0] != 0xFF)
			{
	//			UARTprintf("Found dev info\n");
				unsigned char Mod_Num[20];
				unsigned char Serial[20];
				unsigned char Hardware[20];
				unsigned char Firmware[20];

				EEPROMRead((uint32_t *) &Mod_Num, OFFSET_MODEL_NUMBER, 20);
				EEPROMRead((uint32_t *) &Serial, OFFSET_SERIAL_NUMBER, 20);
				EEPROMRead((uint32_t *) &Hardware, OFFSET_HARDWARE_REV, 20);
				EEPROMRead((uint32_t *) &Firmware, OFFSET_FIRMWARE_REV, 20);

				SysCtlDelay(SysCtlClockGet()/3000);	// Wait 1 ms for memory to power up and be ready

				uint8_t Zero = 0;
				// Write Device Information to Memory
				MemoryWrite(PAGE_DEVICE_INFO, 0, 20, Man_Name);
				MemoryWrite(PAGE_DEVICE_INFO, 20, 20, Mod_Num);
				MemoryWrite(PAGE_DEVICE_INFO, 40, 20, Serial);
				MemoryWrite(PAGE_DEVICE_INFO, 60, 20, Hardware);
				MemoryWrite(PAGE_DEVICE_INFO, 80, 20, Firmware);
				MemoryWrite(PAGE_DEVICE_INFO, 100, 1, &Zero);
				// check RSSI
				// Mac address
			}
		}

#ifdef VALVE_STRUCT
		uint8_t * TempValve = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_VALVE_SETUP, 10);
		if(TempValve[0] != 0xFF)	// Check there is something saved in cartridge memory here
		{
			ValveSetup.Air = TempValve[0];
			ValveSetup.Samp = TempValve[1];
			ValveSetup.B1 = TempValve[2];
			ValveSetup.B2 = TempValve[3];
			ValveSetup.C2 = TempValve[4];
			ValveSetup.Clean = TempValve[5];
			ValveSetup.Cal_1 = TempValve[6];
			ValveSetup.Cal_2 = TempValve[7];
			ValveSetup.T1 = TempValve[8];
			ValveSetup.Rinse = TempValve[9];
		}
		else	// Nothing saved, set to default case
		{
			ValveSetup.Air = 1;
			ValveSetup.Samp = 2;
			ValveSetup.B1 = 3;
			ValveSetup.B2 = 4;
			ValveSetup.C2 = 5;
			ValveSetup.Clean = 6;
			ValveSetup.Cal_1 = 7;
			ValveSetup.Cal_2 = 8;
			ValveSetup.T1 = 9;
			ValveSetup.Rinse = 10;
		}
#endif

//		if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// If device is NOT waking from hibernate
//		{
//			update_Cartridge_Status(gCartridge);
//			if(gCartridge == 1)
//			{
//				update_TCNumbers();
////				update_Test(FindTestNumber());
////				update_Cal(FindCalNumber());
//			}
//		}

		// Update cartridge status even when waking from hibernate just in case cartridge was plugged in while in hibernate
		update_Cartridge_Status(gCartridge);
		if(gCartridge == 1)
		{
			update_TCNumbers();
//				update_Test(FindTestNumber());
//				update_Cal(FindCalNumber());
		}
	}
}

//**************************************************************************
// Interrupt handler for cartridge signal, triggers when cartridge is either
// inserted or removed from device. Updates global variable and BT chip
// Parameters: NONE
// 1/3/2019: Modified to work with digital board V6.2
//**************************************************************************
void CartridgeIntHandler(void)
{
#ifdef MCU_ZXR
	GPIOIntClear(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard));
#else
	uint8_t Signal_pin;
	if(gBoard < V6_2)
	{
		Signal_pin = GPIO_PIN_0;
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
	}
	else if(gBoard >= V6_2)
	{
		Signal_pin = GPIO_PIN_5;
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
	}
#endif

	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Cartridge interrupt! \n");
	)

#ifdef MCU_ZXR
	if(GPIOPinRead(IO_CART_PRES_B_BASE, IO_CART_PRES_B_PIN(gBoard)) == IO_CART_PRES_B_PIN(gBoard))	// Signal is high, cartridge not connected
#else
	if(GPIOPinRead(GPIO_PORTB_BASE, Signal_pin) == Signal_pin)	// Signal is high, cartridge not connected
#endif
	{
		gCartridge = 0;
//		GMEMORY = 0;
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Cartridge removed! \n");
		)
	}
	else
	{
		gCartridge = 1;
		g_CurrentValvePossition = NO_OF_VALVE_PORTS + 1;
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Cartridge inserted! \n");
		)

		// Check if device has programmed in device information, if it does write to memory for BT
		unsigned char Man_Name[20];
		EEPROMRead((uint32_t *) &Man_Name, OFFEST_MANUFACTURER_NAME, 20);
		if(Man_Name[0] != 0xFF)
		{
//			UARTprintf("Found dev info\n");
			unsigned char Mod_Num[20];
			unsigned char Serial[20];
			unsigned char Hardware[20];
			unsigned char Firmware[20];

			EEPROMRead((uint32_t *) &Mod_Num, OFFSET_MODEL_NUMBER, 20);
			EEPROMRead((uint32_t *) &Serial, OFFSET_SERIAL_NUMBER, 20);
			EEPROMRead((uint32_t *) &Hardware, OFFSET_HARDWARE_REV, 20);
			EEPROMRead((uint32_t *) &Firmware, OFFSET_FIRMWARE_REV, 20);

			SysCtlDelay(SysCtlClockGet()/3000);	// Wait 1 ms for memory to power up and be ready

			uint8_t Zero = 0;
			// Write Device Information to Memory
			MemoryWrite(PAGE_DEVICE_INFO, 0, 20, Man_Name);
			MemoryWrite(PAGE_DEVICE_INFO, 20, 20, Mod_Num);
			MemoryWrite(PAGE_DEVICE_INFO, 40, 20, Serial);
			MemoryWrite(PAGE_DEVICE_INFO, 60, 20, Hardware);
			MemoryWrite(PAGE_DEVICE_INFO, 80, 20, Firmware);
			MemoryWrite(PAGE_DEVICE_INFO, 100, 1, &Zero);
			// check RSSI
			// Mac address
		}
//		else
//		{
//			UARTprintf("Did not find dev info\n");
//		}

//		if(*MemoryRead(PAGE_FACTORY_CAL, 0, 1) == 1)
//		{
//			if(*MemoryRead(PAGE_FACTORY_CAL, 1, 1) == 2)
//				CalibrateTemperature();
//		}

//		gMemory = *MemoryRead(PAGE_DEVICE_INFO, OFFSET_MEMORY_CONFIG, 1);
	}

	update_Cartridge_Status(gCartridge);
	if(gCartridge == 1)
	{
		update_TCNumbers();
//		update_Test(FindTestNumber());
//		update_Cal(FindCalNumber());

#ifdef VALVE_STRUCT
		uint8_t * TempValve = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_VALVE_SETUP, 10);
		if(TempValve[0] != 0xFF)	// Check there is something saved in cartridge memory here
		{
			ValveSetup.Air = TempValve[0];
			ValveSetup.Samp = TempValve[1];
			ValveSetup.B1 = TempValve[2];
			ValveSetup.B2 = TempValve[3];
			ValveSetup.C2 = TempValve[4];
			ValveSetup.Clean = TempValve[5];
			ValveSetup.Cal_1 = TempValve[6];
			ValveSetup.Cal_2 = TempValve[7];
			ValveSetup.T1 = TempValve[8];
			ValveSetup.Rinse = TempValve[9];
		}
		else	// Nothing saved, set to default case
		{
			ValveSetup.Air = 1;
			ValveSetup.Samp = 2;
			ValveSetup.B1 = 3;
			ValveSetup.B2 = 4;
			ValveSetup.C2 = 5;
			ValveSetup.Clean = 6;
			ValveSetup.Cal_1 = 7;
			ValveSetup.Cal_2 = 8;
			ValveSetup.T1 = 9;
			ValveSetup.Rinse = 10;
		}
#endif
	}
}

//********************************************************************************
// Scans through the whole array to find the largest value
// Parameters:	Array; pointer to beginning of array
//				size; number of elements in array
// Outputs:		Max; largest value found in array
//********************************************************************************
float FindArrayMax(float *Array, int size)
{
	float Max = Array[0];

	int i;
	for(i = 1; i < size; i++)
	{
		if(Array[i] > Max)
			Max = Array[i];
	}

	return Max;
}

//********************************************************************************
// Scans through the whole array to find the smallest value
// Parameters:	Array; pointer to beginning of array
//				size; number of elements in array
// Outputs:		Min; smallest value found in array
//********************************************************************************
float FindArrayMin(float *Array, int size)
{
	float Min = Array[0];

	int i;
	for(i = 1; i < size; i++)
	{
		if(Array[i] < Min)
			Min = Array[i];
	}

	return Min;
}

////*********************************************************************************
//// Reads CO2 sensors and waits (up to Max_time) for them to become stable as
//// defined by samples, period, and max_spread variables
//// Parameters:	samples; number of samples looked at when determining if steady
////					state has been reached
////				period; number of seconds between taking samples
////				max_spread; mV of spread that is acceptable to be considered steady
////				Min_time; Minimum time that must be waited before checking for
////					steady state
////				Max_time; Maximum time to wait for steady state, quit if steady
////					state is not reached before time has expired
////*********************************************************************************
//void WaitCO2Steady(uint8_t samples, float period, float max_spread, uint16_t Min_time, uint16_t Max_time)
//{
//	uint8_t Steady_state = 0;
//	uint32_t cycle = Min_time * (1 / period);
//	uint8_t buffer_pointer = 0;
//	uint8_t spot_1_steady = 0;
//	uint8_t spot_2_steady = 0;
//
//	float buffer_1[10];		// Size of this array must match samples
//	float buffer_2[10];		// Size of this array must match samples
//
//	TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet()/3 * period));	// Set periodic timer to go every every sample period
//	TimerEnable(TIMER1_BASE, TIMER_A);
//
//	float CO2_mV[2];
//	while(Steady_state == 0)
//	{
//		CO2_mV[0] = (2 * ADCReadAvg(CO2_1_CH, CO2_1_ADC, 20) - 3000)/5;
//		CO2_mV[1] = (2 * ADCReadAvg(CO2_2_CH, CO2_2_ADC, 20) - 3000)/5;
//
//		buffer_1[buffer_pointer] = CO2_mV[0];
//		buffer_2[buffer_pointer] = CO2_mV[1];
//
//		if(cycle > ((samples - 1) + Min_time * (1 / period)))
//		{
//			// Check for steady state
//			if(spot_1_steady == 0)
//			{
//				float Range_1 = FindArrayMax(buffer_1, samples) - FindArrayMin(buffer_1, samples);
//				if(Range_1 < max_spread)
//				{
//					spot_1_steady = 1;
//					UARTprintf("CO2 1 reached steady state after %d seconds, at a value of %d mV * 1000 \n", (int) ((float) cycle * period), (int) (buffer_1[buffer_pointer] * 1000));
//				}
//			}
//			if(spot_2_steady == 0)
//			{
//				float Range_2 = FindArrayMax(buffer_2, samples) - FindArrayMin(buffer_2, samples);
//				if(Range_2 < max_spread)
//				{
//					spot_2_steady = 1;
//					UARTprintf("CO2 2 reached steady state after %d seconds, at a value of %d mV * 1000 \n", (int) ((float) cycle * period), (int) (buffer_2[buffer_pointer] * 1000));
//				}
//			}
//			if(spot_1_steady == 1 && spot_2_steady == 1)
//				Steady_state = 1;
//
//		}
//
//		if(cycle == (Max_time/period))
//		{
//			UARTprintf("Max time reached before steady state \n");
//			Steady_state = 1;
//		}
//
//		cycle++;
//		buffer_pointer++;
//		if(buffer_pointer == samples)
//			buffer_pointer = 0;
//
//		while(g_TimerPeriodicInterruptFlag == 0);
//		g_TimerPeriodicInterruptFlag = 0;
//	}
//	TimerDisable(TIMER1_BASE, TIMER_A);
//}

////********************************************************************************
//// Drives conductivity pad and measures voltage on REF_EL_GUARD to determine if
//// there is connection to reference electrode
//// Parameters:	RE_Threshold; threshold voltage to check for to determine if
////				there is connection to RE or not
//// Outputs:		Status; bitwise status for RE and ISEs
////********************************************************************************
//uint16_t CheckConnection(int16_t RE_Threshold)
//{
//	// Set SPI communication to mode 1 for ADC5
//	SSIDisable(SSI1_BASE);
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
//	SSIEnable(SSI1_BASE);
//
//	int16_t Step_voltage = 500;
//
//	// Configure ADC 5 so AINp is AIN0 and AINn is GND
//	// Set FSR to +/- 2.048V; LSB of 62.5 uV
//	// Set to continuous conversion mode
//	// Set data rate to 128 SPS
//	// ADC Mode
//	// Disable pull-up resistor
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x83);
//
//	uint16_t Status = 0;	// 0, 0, 0, 0, 0, 0, RE, CO2_2, CO2_1, NH4, TH_2, TH_1, Ca_2, Ca_1, pH_2, pH_1
//	uint8_t Ch[10] = {PH_1_CH, PH_2_CH, PH_3_CH, CA_1_CH, CA_2_CH, TH_1_CH, TH_2_CH, NH4_CH, CO2_1_CH, CO2_2_CH};
//	uint8_t ADC[10] = {PH_1_ADC, PH_2_ADC, PH_3_ADC, CA_1_ADC, CA_2_ADC, TH_1_ADC, TH_2_ADC, NH4_ADC, CO2_1_ADC, CO2_2_ADC};
////	uint8_t Threshold[9] = {50, 50, 150, 150, 150, 150, 150, 50, 50};
////	uint8_t Threshold[9] = {100, 100, 150, 150, 150, 150, 150, 150, 150};
//	uint16_t ISE_Threshold = 1250;
//	uint16_t pH_Threshold = 1000;
////	uint8_t RE_Threshold = 10;
//	uint32_t RE_ADC_MSB;
//	uint32_t RE_ADC_LSB;
//	int16_t RE_High = 0;
//	int16_t RE_Low = 0;
//
////	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// Connect RE to amp loop
////	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Connect RE to amp loop
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// CE connected
//	IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 1);		// Connect CE to DAC
//
//	// Check RE
//	DACVoltageSet(6, Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_High |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	DACVoltageSet(6, -Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_Low |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	float RE_Voltage = (RE_High - RE_Low) * .0625;	// mV
//
//	// Check ISEs
//	uint8_t i;
//	for(i = 0; i < 10; i++)
//	{
//		DACVoltageSet(6, Step_voltage, 1);
////		userDelay(500, 1);
//
//		float ADC_Voltage_1 = ADCRead(Ch[i], ADC[i]);
//
//		DACVoltageSet(6, -Step_voltage, 1);
////		userDelay(500, 1);
//
//		float ADC_Voltage_2 = ADCRead(Ch[i], ADC[i]);
//
////		if((ADC_Voltage_1 - ADC_Voltage_2) >= Threshold[i])
//
//		if(i < 2)
//		{
//			if((ADC_Voltage_1 - ADC_Voltage_2) >= pH_Threshold)
//				Status |= (int) pow(2, i);
//		}
//		else
//		{
//			if((ADC_Voltage_1 - ADC_Voltage_2) >= ISE_Threshold)
//				Status |= (int) pow(2, i);
//		}
//
//		UARTprintf("%d \t %d \n", i, (int) ((ADC_Voltage_1 - ADC_Voltage_2) * 1000));
//	}
//
//	RE_High = 0;
//	RE_Low = 0;
//	// Check RE
//	DACVoltageSet(6, Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_High |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	DACVoltageSet(6, -Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_Low |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	DACVoltageSet(6, 0, 1);
//
//	SysCtlDelay(SysCtlClockGet()/3 * .05);
//
//	IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 0);		// Disconnect CE from DAC
//
//	// GND RE for ISEs
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating
//
//	// Configure ADC 5 so AINp is AIN0 and AINn is GND
//	// Set FSR to +/- 2.048V; LSB of 62.5 uV
//	// Set it in power-down/single-shot mode when not using
//	// Set data rate to 128 SPS
//	// ADC Mode
//	// Disable pullup resistor
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC5, 0x83);
//
//	if(RE_Voltage < ((RE_High - RE_Low) * .0625))
//		RE_Voltage = (RE_High - RE_Low) * .0625;	// mV
//
//	if(RE_Voltage > RE_Threshold)
//		Status |= (int) pow(2, 9);
//
////	UARTprintf("DOUT \t %d \t %d \n", RE_High, RE_Low);
//	UARTprintf("RE_Voltage \t %d \n", (int) (RE_Voltage * 1000));
//
////	UARTprintf("Sensor Status: \n");
////	UARTprintf("RE: \t %d \n", (Status >> 9) & 1);
////	UARTprintf("pH: \t %d \t %d \n", (Status) & 1, (Status >> 1) & 1);
////	UARTprintf("Ca: \t %d \t %d \n", (Status >> 2) & 1, (Status >> 3) & 1);
////	UARTprintf("TH: \t %d \t %d \n", (Status >> 4) & 1, (Status >> 5) & 1);
////	UARTprintf("NH4: \t %d \t %d \t %d \n", (Status >> 6) & 1, (Status >> 7) & 1, (Status >> 8) & 1);
//
//	return Status;
//}
//
////********************************************************************************
//// Drives conductivity pad and measures voltage on REF_EL_GUARD to determine if
//// there is connection to reference electrode
//// Parameters:	RE_Threshold; threshold voltage to check for to determine if
////				there is connection to RE or not
//// Outputs:		Status; bitwise status for RE and ISEs
////********************************************************************************
//uint8_t CheckRE(int16_t RE_Threshold)
//{
//	// Set SPI communication to mode 1 for ADC5
//	SSIDisable(SSI1_BASE);
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
//	SSIEnable(SSI1_BASE);
//
//	int16_t Step_voltage = 500;
//
//	// Configure ADC 5 so AINp is AIN0 and AINn is GND
//	// Set FSR to +/- 2.048V; LSB of 62.5 uV
//	// Set to continuous conversion mode
//	// Set data rate to 128 SPS
//	// ADC Mode
//	// Disable pull-up resistor
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x83);
//
//	uint8_t Status = 0;
//
//	uint32_t RE_ADC_MSB;
//	uint32_t RE_ADC_LSB;
//	int16_t RE_High = 0;
//	int16_t RE_Low = 0;
//
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// Connect RE to amp loop
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating
//	IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 1);		// Connect CE to DAC
//
//	// Check RE
//	DACVoltageSet(6, Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_High |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	DACVoltageSet(6, -Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_Low |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	float RE_Voltage = (RE_High - RE_Low) * .0625;	// mV
//
//	RE_High = 0;
//	RE_Low = 0;
//
//	// Check RE
//	DACVoltageSet(6, Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_High |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	DACVoltageSet(6, -Step_voltage, 1);
//	SysCtlDelay(SysCtlClockGet()/3 * .0075);
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC4, 0x81);
//	SSIDataGet(SSI1_BASE, &RE_ADC_MSB);
//	SSIDataGet(SSI1_BASE, &RE_ADC_LSB);
//	RE_Low |= (RE_ADC_MSB << 8 | RE_ADC_LSB);
//
//	DACVoltageSet(6, 0, 1);
//
//	SysCtlDelay(SysCtlClockGet()/3 * .05);
//
//	IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 0);		// Disconnect CE from DAC
//
//	// GND RE for ISEs
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating
//
//	// Configure ADC 5 so AINp is AIN0 and AINn is GND
//	// Set FSR to +/- 2.048V; LSB of 62.5 uV
//	// Set it in power-down/single-shot mode when not using
//	// Set data rate to 128 SPS
//	// ADC Mode
//	// Disable pullup resistor
//	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0xC5, 0x83);
//
//	if(RE_Voltage < ((RE_High - RE_Low) * .0625))
//		RE_Voltage = (RE_High - RE_Low) * .0625;	// mV
//
//	if(RE_Voltage > RE_Threshold)
//		Status = 1;
//
//	UARTprintf("RE_Voltage \t %d \n", (int) (RE_Voltage * 1000));
//
//	return Status;
//}
//
////**************************************************************************
//// Repumps solution if ISEs or RE do not pass checks in WatchISEs function
//// Parameters:	Solution; Which solution is being pumped as defined at the
////					top of Helper.h; CAL_PRERINSE
//// Outputs:		NONE
////**************************************************************************
//void Repump_Solution(uint8_t Solution)
//{
//	uint8_t i;
//	uint8_t valve_delay = 2;	// Delay from stopping pumping to turning valve
//
//	// Variables to control pump and valve
//	uint16_t runSteps_Solution = 8000 * gPump_Ratio;	// Number of pump steps for solution between air bubbles
//
//	uint16_t runSteps_plug = 7500 * gPump_Ratio; 		// Number of pump steps to get the sample plug centered on the sensors/reference
//
//	// Variables to control pumping buffer and solution
//	uint16_t Steps_Buffer;
//	uint16_t Steps_Solution_1 = 1000;			// Steps to pump Cal 3 before buffer
//	uint16_t Steps_Solution_2 = (5000 * gPump_Ratio) - 1000;			// Steps to pump Cal 3 after buffer
//
//	uint16_t Steps_air = 4000 * gPump_Ratio;				// Steps to pump air
//	uint16_t diffusion_time = 5;			// Time to wait after oscillating in mixing chamber
//
//	// Variables to control mixing
//	uint16_t mix_cycles = 10;				// Number of forward backward pump cycles
//	uint16_t Steps_cycles = 1000 * gPump_Ratio;			// Steps to pump backwards/forwards
//
//	// Variables to control where in sample plug to pause when measuring
//	uint16_t Steps_before_sample = 4500 * gPump_Ratio;		// Steps to pump to place solution over sensor
//
////	// Variables to control pump and valve with chromtech tubing
////	uint16_t runSteps_Solution = 11500;	// Number of pump steps for solution between air bubbles
////
////	uint16_t runSteps_plug = 11000; 		// Number of pump steps to get the sample plug centered on the sensors/reference
////
////	// Variables to control pumping buffer and solution
////	uint16_t Steps_Buffer;
////	uint16_t Steps_Solution_1 = 2000;			// Steps to pump Cal 3 before buffer
////	uint16_t Steps_Solution_2 = 5000;			// Steps to pump Cal 3 after buffer
////
////	uint16_t Steps_air = 5500;				// Steps to pump air
////	uint16_t diffusion_time = 5;			// Time to wait after oscillating in mixing chamber
////
////	// Variables to control mixing
////	uint16_t mix_cycles = 10;				// Number of forward backward pump cycles
////	uint16_t Steps_cycles = 1500;			// Steps to pump backwards/forwards
////
////	// Variables to control where in sample plug to pause when measuring
////	uint16_t Steps_before_sample = 7000;		// Steps to pump to place solution over sensor
//
//	uint8_t Valve_Solution;
//	uint8_t Valve_Buffer;
//
//	if(Solution == CAL_PRERINSE || Solution == CAL_POSTRINSE || Solution == TEST_PRERINSE || Solution == TEST_POSTRINSE)
//		Valve_Solution = V_RINSE;
//	else if(Solution == CAL_3)
//		Valve_Solution = V_CAL_3;
//	else if(Solution == CAL_3_B1)
//	{
//		Valve_Solution = V_CAL_3;
//		Valve_Buffer = V_B1;
//		Steps_Buffer = 600;		// Steps to pump buffer 1 with Cal 3
//	}
//	else if(Solution == CAL_2)
//		Valve_Solution = V_CAL_2;
//	else if(Solution == CAL_2_B1)
//	{
//		Valve_Solution = V_CAL_2;
//		Valve_Buffer = V_B1;
//		Steps_Buffer = 240;		// Steps to pump buffer 1 with Cal 2
//	}
//	else if(Solution == CAL_1)
//		Valve_Solution = V_CAL_1;
//	else if(Solution == RINSE_B2)
//	{
//		Valve_Solution = V_RINSE;
//		Valve_Buffer = V_B2;
//		Steps_Buffer = 320;		// Steps to pump buffer 2 with Rinse
//	}
//	else if(Solution == TEST_SAMPLE)
//		Valve_Solution = V_SAMP;
//
//	if(Solution == CAL_3_B1 || Solution == CAL_2_B1 || Solution == RINSE_B2)
//	{
//		RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
//		FindPossitionZeroPump();
//
//		// Pump buffer and solution
//		RunValveToPossition_Bidirectional(Valve_Solution, VALVE_STEPS_PER_POSITION);
//		PumpStepperRunStepSlow(FW, Steps_Solution_1);
//		SysCtlDelay(SysCtlClockGet()/3 * valve_delay);
//		RunValveToPossition_Bidirectional(Valve_Buffer, VALVE_STEPS_PER_POSITION);
//		PumpStepperRunStepSlow(FW, Steps_Buffer);
//		SysCtlDelay(SysCtlClockGet()/3 * valve_delay);
//		RunValveToPossition_Bidirectional(Valve_Solution, VALVE_STEPS_PER_POSITION);
//		PumpStepperRunStepSlow(FW, Steps_Solution_2);
//		SysCtlDelay(SysCtlClockGet()/3 * valve_delay);
//		RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
//		PumpStepperRunStepSlow(FW, Steps_air);
//
//		for(i = 0; i < mix_cycles; i++)
//		{
//			PumpStepperRunStepSlow(BW, Steps_cycles);
//			PumpStepperRunStepSlow(FW, Steps_cycles);
//		}
//
//		SysCtlDelay(SysCtlClockGet()/3 * diffusion_time);	// Delay
//
//		PumpStepperRunStepSlow(FW, Steps_before_sample);
//	}
//	else
//	{
//		RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
//		FindPossitionZeroPump();
//
//		RunValveToPossition_Bidirectional(Valve_Solution, VALVE_STEPS_PER_POSITION);
//		PumpStepperRunStepSlow(FW, runSteps_Solution);
//		SysCtlDelay(SysCtlClockGet()/3 * valve_delay);
//		RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
//		PumpStepperRunStepSlow(FW, runSteps_plug);
//	}
//}
//
////**************************************************************************
//// Collects data for ISEs over time and saves it in EEPROM so calculations
//// can be done on whole data set, calculates standard deviation of drift
//// after connecting RE, runs square wave through CE and checks that RE and
//// ISEs follows this voltage
//// Parameters:	ui8Seconds; # of seconds to watch the ISEs settle
////				Solution; Soltuion that is being pumped defined at top of
////					Helper.h; CAL_PRERINSE
////				SensorCal; Calibration variable that stores whether sensor
////					had connection to solutions used for calibration; pointer
////					because the variable is in calibration state scope
//// Outputs:		Address; address in EEPROM where ISE data is saved
////**************************************************************************
//uint8_t WatchISEs(uint8_t ui8Seconds, uint8_t Solution, uint16_t * SensorStatus)
//{
//	uint8_t i;
//	uint8_t ui8Current_time;
//	float pH_mV_time[2], Ca_mV_time[2], TH_mV_time[2], NH4_mV_time[3];//, ORP_time;
//
//	float Shift[9];
//	float StDev[9];
//	uint16_t CE_Drive_Status;
//
//	uint8_t Mix_type = 0;
//	if(Solution == CAL_3_B1 || Solution == CAL_2_B1)
//		Mix_type = 1;
//	else if(Solution == RINSE_B2)
//		Mix_type = 2;
//
//	uint8_t time_RE_on = 2;		// Time to wait before connecting RE
//	uint8_t time_start_measure;	// Time to start measurement for standard deviation
//	if(Mix_type == 0)
//		time_start_measure = time_RE_on + 3;
//	else
//		time_start_measure = ui8Seconds - 4;
//
//	uint16_t Combined_Status;
//
//	uint16_t Solution_Sensors;
//	if(Solution == CAL_PRERINSE)
//		Solution_Sensors = SENSORS_CAL_PRERINSE;
//	else if(Solution == CAL_3)
//		Solution_Sensors = SENSORS_CAL_3;
//	else if(Solution == CAL_3_B1)
//		Solution_Sensors = SENSORS_CAL_3_B1;
//	else if(Solution == CAL_2)
//		Solution_Sensors = SENSORS_CAL_2;
//	else if(Solution == CAL_2_B1)
//		Solution_Sensors = SENSORS_CAL_2_B1;
//	else if(Solution == CAL_1)
//		Solution_Sensors = SENSORS_CAL_1;
//	else if(Solution == CAL_POSTRINSE)
//		Solution_Sensors = SENSORS_CAL_POSTRINSE;
//	else if(Solution == RINSE_B2)
//		Solution_Sensors = SENSORS_RINSE_B2;
//	else if(Solution == TEST_PRERINSE)
//		Solution_Sensors = SENSORS_TEST_PRERINSE;
//	else if(Solution == TEST_SAMPLE)
//		Solution_Sensors = SENSORS_TEST_SAMPLE;
//	else if(Solution == TEST_POSTRINSE)
//		Solution_Sensors = SENSORS_TEST_POSTRINSE;
//
//	uint8_t Success = 0;
//	uint8_t attempt = 1;
//	uint8_t Max_attempts = 1;
//	while(Success == 0 && attempt < (Max_attempts + 1))
//	{
//		// RE and CE floating
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating
//
//		if(attempt > 1)
//			Repump_Solution(Solution);
//
//		if(gABoard >= AV6_1)
//			IO_Ext_Set(IO_EXT2_ADDR, 2, I2C_SENSOR_ON_B, 1);	// Disconnect I2C lines from cartridge while sampling ISEs
//
//		UARTprintf("\t pH 1 \t pH 2 \t Ca 1 \t Ca 2 \t TH 1 \t TH 2 \t NH4 1 \t NH4 2 \t NH4 3 \n"); //\t ORP \n");
//		TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 1); // Set periodic timer to collect a point every second
//		TimerEnable(TIMER1_BASE, TIMER_A);
//
//		if(GND_CE_ON)
//		{
//			DACVoltageSet(6, 0, 1);
//			IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_GND, 1);
//		}
//
//		for(ui8Current_time = 0; ui8Current_time < (ui8Seconds + 1); ui8Current_time++)
//		{
//			if(ui8Current_time == 0)
//				pH_mV_time[0] = (2 * ADCReadAvg(PH_1_CH, PH_1_ADC, 20) - 3000)/4;
//			pH_mV_time[0] = (2 * ADCReadAvg(PH_1_CH, PH_1_ADC, 20) - 3000)/4;
//			pH_mV_time[1] = (2 * ADCReadAvg(PH_2_CH, PH_2_ADC, 20) - 3000)/4;
//			Ca_mV_time[0] = (2 * ADCReadAvg(CA_1_CH, CA_1_ADC, 20) - 3000)/5;
//			Ca_mV_time[1] = (2 * ADCReadAvg(CA_2_CH, CA_2_ADC, 20) - 3000)/5;
//			TH_mV_time[0] = (2 * ADCReadAvg(TH_1_CH, TH_1_ADC, 20) - 3000)/5;
//			TH_mV_time[1] = (2 * ADCReadAvg(TH_2_CH, TH_2_ADC, 20) - 3000)/5;
//			NH4_mV_time[0] = (2 * ADCReadAvg(NH4_CH, NH4_ADC, 20) - 3000)/5;
//			NH4_mV_time[1] = (2 * ADCReadAvg(CO2_1_CH, CO2_1_ADC, 20) - 3000)/5;
//			NH4_mV_time[2] = (2 * ADCReadAvg(CO2_2_CH, CO2_2_ADC, 20) - 3000)/5;
//
//			UARTprintf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n", ui8Current_time, (int) (pH_mV_time[0] * 1000), (int) (pH_mV_time[1] * 1000), (int) (Ca_mV_time[0] * 1000), (int) (Ca_mV_time[1] * 1000), (int) (TH_mV_time[0] * 1000), (int) (TH_mV_time[1] * 1000), (int) (NH4_mV_time[0] * 1000), (int) (NH4_mV_time[1] * 1000), (int) (NH4_mV_time[2] * 1000));
//
//			if(ui8Current_time == time_RE_on)
//			{
//				if(GND_CE_ON)
//					IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_GND, 0);
//
//				// Check connection of RE and ISEs
//				if(CHECK_RE)
//				{
//					if(Solution == CAL_2)
//						CE_Drive_Status = CheckConnection(-10);
//					else
//						CE_Drive_Status = CheckConnection(15);
//				}
//
//				// GND RE for ISEs
//				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
//				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating
//
//				Shift[0] = pH_mV_time[0];
//				Shift[1] = pH_mV_time[1];
//				Shift[2] = Ca_mV_time[0];
//				Shift[3] = Ca_mV_time[1];
//				Shift[4] = TH_mV_time[0];
//				Shift[5] = TH_mV_time[1];
//				Shift[6] = NH4_mV_time[0];
//				Shift[7] = NH4_mV_time[1];
//				Shift[8] = NH4_mV_time[2];
//			}
//
//			// Check the range after connecting RE
//			if(ui8Current_time == (time_RE_on + 3)/*(ui8Seconds - 5)*/)
//			{
//				for(i = 0; i < 2; i++)
//				{
//					Shift[i] = pH_mV_time[i] - Shift[i];
//					Shift[i+2] = Ca_mV_time[i] - Shift[i+2];
//					Shift[i+4] = TH_mV_time[i] - Shift[i+4];
//					Shift[i+6] = NH4_mV_time[i] - Shift[i+6];
//				}
//				Shift[8] = NH4_mV_time[2] - Shift[8];
//			}
//
//			if(ui8Current_time >= time_start_measure)
//			{
//				EEPROMProgram((uint32_t *) &pH_mV_time[0], ((OFFSET_ISE_TIME) + (36 * (ui8Current_time - time_start_measure))), 8);
//				EEPROMProgram((uint32_t *) &Ca_mV_time[0], ((OFFSET_ISE_TIME + 8) + (36 * (ui8Current_time - time_start_measure))), 8);
//				EEPROMProgram((uint32_t *) &TH_mV_time[0], ((OFFSET_ISE_TIME + 16) + (36 * (ui8Current_time - time_start_measure))), 8);
//				EEPROMProgram((uint32_t *) &NH4_mV_time[0], ((OFFSET_ISE_TIME + 24) + (36 * (ui8Current_time - time_start_measure))), 12);
//			}
//
//			while(g_TimerPeriodicInterruptFlag == 0);
//			g_TimerPeriodicInterruptFlag = 0;
//		}
//
//		if(gABoard >= AV6_1)
//			IO_Ext_Set(IO_EXT2_ADDR, 2, I2C_SENSOR_ON_B, 0);	// Reconnect I2C lines to cartridge after sampling ISEs
//
//		UARTprintf("\n");
//
//		float Shift_median = FindMedian(Shift, 9);
//		UARTprintf("Grounding CE \n");
//		UARTprintf("Median shift after connecting RE \t %d \t mV * 1000 \n", (int) (Shift_median * 1000));
//		UARTprintf("pH shift \t %d \t %d \t mV * 1000 \n", (int) (Shift[0] * 1000), (int) (Shift[1] * 1000));
//		UARTprintf("Ca shift \t %d \t %d \t mV * 1000 \n", (int) (Shift[2] * 1000), (int) (Shift[3] * 1000));
//		UARTprintf("TH shift \t %d \t %d \t mV * 1000 \n", (int) (Shift[4] * 1000), (int) (Shift[5] * 1000));
//		UARTprintf("NH4 shift \t %d \t %d \t %d \t mV * 1000 \n", (int) (Shift[6] * 1000), (int) (Shift[7] * 1000), (int) (Shift[8] * 1000));
//		UARTprintf("\n");
//
//		StDev[0] = FindStdDevISEs(OFFSET_ISE_TIME, (ui8Seconds - time_start_measure + 1));
//		StDev[1] = FindStdDevISEs((OFFSET_ISE_TIME + 4), (ui8Seconds - time_start_measure + 1));
//		StDev[2] = FindStdDevISEs((OFFSET_ISE_TIME + 8), (ui8Seconds - time_start_measure + 1));
//		StDev[3] = FindStdDevISEs((OFFSET_ISE_TIME + 12), (ui8Seconds - time_start_measure + 1));
//		StDev[4] = FindStdDevISEs((OFFSET_ISE_TIME + 16), (ui8Seconds - time_start_measure + 1));
//		StDev[5] = FindStdDevISEs((OFFSET_ISE_TIME + 20), (ui8Seconds - time_start_measure + 1));
//		StDev[6] = FindStdDevISEs((OFFSET_ISE_TIME + 24), (ui8Seconds - time_start_measure + 1));
//		StDev[7] = FindStdDevISEs((OFFSET_ISE_TIME + 28), (ui8Seconds - time_start_measure + 1));
//		StDev[8] = FindStdDevISEs((OFFSET_ISE_TIME + 32), (ui8Seconds - time_start_measure + 1));
//
//		UARTprintf("Standard deviation after connecting RE: \n");
//		UARTprintf("pH \t %d \t %d \t mV * 1000 \n", (int) (StDev[0] * 1000), (int) (StDev[1] * 1000));
//		UARTprintf("Ca \t %d \t %d \t mV * 1000 \n", (int) (StDev[2] * 1000), (int) (StDev[3] * 1000));
//		UARTprintf("TH \t %d \t %d \t mV * 1000 \n", (int) (StDev[4] * 1000), (int) (StDev[5] * 1000));
//		UARTprintf("NH4 \t %d \t %d \t %d \t mV * 1000 \n\n", (int) (StDev[6] * 1000), (int) (StDev[7] * 1000), (int) (StDev[8] * 1000));
//		TimerDisable(TIMER1_BASE, TIMER_A);
//		g_TimerPeriodicInterruptFlag = 0;
//
//		// Print out CE driving status and fail all if RE is not detected
//		if(CHECK_RE)
//		{
//			UARTprintf("Sensor Status from driving CE: \n");
//			UARTprintf("RE: \t %d \n", (CE_Drive_Status >> 9) & 1);
//			UARTprintf("pH: \t %d \t %d \n", (CE_Drive_Status) & 1, (CE_Drive_Status >> 1) & 1);
//			UARTprintf("Ca: \t %d \t %d \n", (CE_Drive_Status >> 2) & 1, (CE_Drive_Status >> 3) & 1);
//			UARTprintf("TH: \t %d \t %d \n", (CE_Drive_Status >> 4) & 1, (CE_Drive_Status >> 5) & 1);
//			UARTprintf("NH4: \t %d \t %d \t %d \n", (CE_Drive_Status >> 6) & 1, (CE_Drive_Status >> 7) & 1, (CE_Drive_Status >> 8) & 1);
//
//			// If connection to RE is not detected fail all sensors
//			if(((CE_Drive_Status >> 9) & 1) == 0)
//			{
//				UARTprintf("No connection to RE detected! Failing all sensors! \n\n");
//				CE_Drive_Status = 0;
//			}
//		}
//
//		// Sort through shift and standard deviation data
//		uint16_t Stability_Status = 0; // 0,0,0,0,0,0,0,NH4_3,NH4_2,NH4_1,TH_2,TH_1,Ca_2,Ca_1,pH_2,pH_1
//		uint8_t Shift_count = 0;	// Count up how many sensors passed the shift check
//		if(Shift_median > 50)	// Shift should always be greater than 50 mV, otherwise fail all sensors
//		{
//			for(i = 0; i < 9; i++)
//			{
//				// Check if sensor shift is greater than -10% of median shift AND
//				// EITHER sensor shift is less than 10% of median OR this is a B2 mixture
//				if((((Shift[i] - Shift_median)/Shift_median) >= -0.1) && ((((Shift[i] - Shift_median)/Shift_median) <= 0.1) || Mix_type == 2))
//				{
//					Shift_count++;
//
//					// Standard deviation should be less than 2.5 mV
//					// unless were mixing B2, then must be less than 4 mV
//					if(StDev[i] < 2.5 || (StDev[i] < 4 && Mix_type == 2))
//						Stability_Status |= (int) pow(2, i);
//				}
//			}
//
//			// Fail all sensors if at least 5 did not pass shift check, this is to make sure median is where it should be
//			if(Shift_count < 5)
//			{
//				UARTprintf("At least 5 sensors didn't pass shift check, failing all sensors \n");
//				Stability_Status = 0;
//			}
//		}
//
//		UARTprintf("Sensor Status after connecting RE: \n");
//		UARTprintf("pH: \t %d \t %d \n", (Stability_Status) & 1, (Stability_Status >> 1) & 1);
//		UARTprintf("Ca: \t %d \t %d \n", (Stability_Status >> 2) & 1, (Stability_Status >> 3) & 1);
//		UARTprintf("TH: \t %d \t %d \n", (Stability_Status >> 4) & 1, (Stability_Status >> 5) & 1);
//		UARTprintf("NH4: \t %d \t %d \t %d \n", (Stability_Status >> 6) & 1, (Stability_Status >> 7) & 1, (Stability_Status >> 8) & 1);
//
//		// Combine RE connection and CE driving checks into one
//		Combined_Status = 0 | (Stability_Status & CE_Drive_Status);
//
//		UARTprintf("Sensor Status after connecting RE and driving CE: \n");
//		UARTprintf("1 means sensor passed, 0 means sensor failed \n");
//		UARTprintf("pH 1 \t pH 2 \t Ca 1 \t Ca 2 \t TH 1 \t TH 2 \t NH4 1 \t NH4 2 \t NH4 3 \n");
//		UARTprintf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n\n", (Combined_Status) & 1, (Combined_Status >> 1) & 1, (Combined_Status >> 2) & 1,
//				(Combined_Status >> 3) & 1, (Combined_Status >> 4) & 1, (Combined_Status >> 5) & 1, (Combined_Status >> 6) & 1, (Combined_Status >> 7) & 1, (Combined_Status >> 8) & 1);
//
//		// Check that the sensors see the solutions they are supposed to
//		// Don't repump if failure has happened on previous solution as well
//		if(((Combined_Status & Solution_Sensors & *SensorStatus) == (Solution_Sensors & *SensorStatus)))	// Check that all sensors that need this solution passed checks
//			Success = 1;
//		else
//		{
//			if(attempt < Max_attempts && Max_attempts > 1)
//				UARTprintf("Required sensors did not have connection! Pumping again! \n\n");
//
//			attempt++;
//		}
//	}
//
//	*SensorStatus &= (Combined_Status | ~Solution_Sensors);
//
//	UARTprintf("SensorStatus: \n");
//	UARTprintf("pH 1 \t pH 2 \t Ca 1 \t Ca 2 \t TH 1 \t TH 2 \t NH4 1 \t NH4 2 \t NH4 3 \n");
//	UARTprintf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n", ((*SensorStatus >> 0) & 1), ((*SensorStatus >> 1) & 1),
//			((*SensorStatus >> 2) & 1), ((*SensorStatus >> 3) & 1), ((*SensorStatus >> 4) & 1), ((*SensorStatus >> 5) & 1), ((*SensorStatus >> 6) & 1),
//			((*SensorStatus >> 7) & 1), ((*SensorStatus >> 8) & 1));
//
//	return time_start_measure;
////	return ((OFFSET_ISE_TIME) + (36 * (ui8Seconds - time_start_measure)));
//}

//********************************************************************************
// Calculates r^2 of a line given the x points, y points, and number of points
// Inputs: 	xs; array of all x values
//			ys; array of all y values
//			n; number of points
// Outputs: r^2
//********************************************************************************
float RSQ(float *xs, float *ys, int n)
{
	int i;

	float Sum_xy = 0;
	float Sum_x = 0;
	float Sum_y = 0;
	float Sum_xx = 0;
	float Sum_yy = 0;

	for(i = 0; i < n; i++)
	{
		Sum_xy += (xs[i] * ys[i]);
		Sum_x += xs[i];
		Sum_y += ys[i];
		Sum_xx += pow(xs[i], 2);
		Sum_yy += pow(ys[i], 2);
	}

	float r = (n * Sum_xy - Sum_x* Sum_y)/sqrt((n * Sum_xx - pow(Sum_x, 2)) * (n * Sum_yy - pow(Sum_y, 2)));

	return pow(r, 2);
}

//********************************************************************************
// Calculates best fit slope of a line given the x points, y points, and number of points
// Inputs: 	xs; array of all x values
//			ys; array of all y values
//			n; number of points
// Outputs: Slope
// Created: 1/10/2023
// Created to calculate the best fit slope for the conductivity sensor
// Slope = Sum((x-xa)(y-ya))/Sum((x-xa)(x-xa))
//********************************************************************************
float FindBestFitSlope(float *xs, float *ys, uint8_t n)
{
	float sum_num = 0;	// Create variable to hold the sume of the numerator
	float sum_den = 0;	// Create variable to hold the sum of the denominator
	float x_a = 0;	// X average
	float y_a = 0;	// Y average

	uint8_t i;

	// Need to calculate the average x and y first
	for(i = 0; i < n; i++)	// Sum all the values first
	{
		x_a += xs[i];
		y_a += ys[i];
	}
	x_a /= n;	// Divide by total number of points
	y_a /= n;	// Divide by total number of points

	for(i = 0; i < n; i++)
	{
		sum_num += ((xs[i] - x_a) * (ys[i] - y_a));
		sum_den += ((xs[i] - x_a) * (xs[i] - x_a));
	}

	float slope = sum_num / sum_den;
	return slope;
}

//********************************************************************************
// Sorts an array of floats into ascending order
// Inputs: 	pfArray; float pointer to array
//			size; length of array
// Outputs: Slope
// Created: 1/20/2023
// Created to sort the conductivity array used to calculate the best fit slope
//********************************************************************************
void SortArray(float * pfArray, uint8_t size)
{
	uint8_t i, j, min_index;

	for(i = 0; i < size - 1; i++)
	{
		// Find the minimum element
		min_index = i;
		for(j = i + 1; j < size; j++)
			if(pfArray[j] < pfArray[min_index])
				min_index = j;

		// Swap the smallest value with the current location
		float temp = pfArray[i];
		pfArray[i] = pfArray[min_index];
		pfArray[min_index] = temp;
	}
}

//********************************************************************************
// Connects or disconnects cartridge memory, checks that BT isn't currently using
// cartridge memory, waits until it is finished before disconnecting
// Created: 12/20/2018
// 3/6/2020: Created gui8MemConnected variable and set it so the I2C used by tiva
//   is raised while memory is disconnected
// Inputs: Connect; 1 to connect memory 0 to disconnect memory
//********************************************************************************
void ConnectMemory(uint8_t Connect)
{
	if(gABoard >= AV6_1)
	{
		if(Connect == 0)
		{
#ifdef MCU_ZXR
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
			update_TivaI2C(1);	// Call update function before setting gui8MemConnected because the update function doesn't change state unless mem is connected
			gui8MemConnected = 0;
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
#else
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
			update_TivaI2C(1);	// Call update function before setting gui8MemConnected because the update function doesn't change state unless mem is connected
			gui8MemConnected = 0;
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C

#endif
		}

		//		IO_Ext_Set(IO_EXT2_ADDR, 2, I2C_SENSOR_ON_B, (1 - Connect));	// Connect or disconnect memory, connected low

		// Copied in the IO_Ext_Set function rather than using the function to skip the I2C used by BT at beginning, this function already has it built in and it was creating a infinite loop of entering new IO_Ext_Set functions
		uint8_t ui8PinValues; // Generic variable to use when writing and reading back
		// Adjust register values using bitwise operations so only desired pin is changed
		if((1 - Connect))
			gui8IO_Ext2_Reg2 |= I2C_SENSOR_ON_B; // Set bit to turn on uiPin
		else
			gui8IO_Ext2_Reg2 &= ~I2C_SENSOR_ON_B; // Clear bit to turn off uiPin
		ui8PinValues = gui8IO_Ext2_Reg2;

		uint8_t ui8PinCheck = ~ui8PinValues;

		uint8_t Counter = 0;
		while(ui8PinCheck != ui8PinValues && Counter <= 10)// && ui32Error == 0)
		{
			// Write modified Pin Values
			I2CSend(I2C0_BASE, IO_EXT2_ADDR, 2, 2, ui8PinValues);

			// Read back pins to make sure it was written to correctly
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);

			if(Counter < 5)
				Counter++;
			else if(Counter == 5)
			{
				Counter++;
				DEBUG_PRINT(UARTprintf("Tried writing IO Extender 5 times, not writing! \n");)
				DEBUG_PRINT(UARTprintf("Resetting analog board!\n");)

				update_TivaI2C(0);

				AnalogOff();

				userDelay(1000, 1);

				InitAnalog();

				update_TivaI2C(1);
			}
			else if(Counter < 10)
				Counter++;
			else if(Counter == 10)
			{
				Counter++;

				DEBUG_PRINT(UARTprintf("Tried writing IO Extender 5 more times after reset, still not writing!\n");)

				gui32Error |= IO_EXT2_FAIL;

				update_Error();
			}
		}

		if(Connect == 1)
		{
			gui8MemConnected = 1;
			update_TivaI2C(0);	// Call update function before setting gui8MemConnected because the update function doesn't change state unless mem is connected
		}
	}
}

//********************************************************************************
// Connects or disconnects cartridge memory, checks that BT isn't currently using
// cartridge memory, waits until it is finished before disconnecting
// Created: 12/20/2018
// Inputs:	milliseconds; number of milliseconds to delay, max of 4,294,967,296
//			Abort; 0 or 1, 0 will always delay, 1 will skip if abort error occurred
// Outputs: NONE
//********************************************************************************
void userDelay(uint32_t milliseconds, uint8_t Abort)
{
	if((gui32Error & ABORT_ERRORS) == 0 || Abort == 0)
	{
		uint32_t i;
		for(i = 0; i < milliseconds; i++)
		{
			if((gui32Error & ABORT_ERRORS) != 0 && Abort == 1)
				return;

			SysCtlDelay(SysCtlClockGet()/3000);
		}
	}
}

#ifdef TESTING_MODE
#ifdef MEMORY_V4
//********************************************************************************
// Reads cartridge memory and prints all its data over UART
// Created: 5/8/2019
// 12/7/2020: Modifying to work with full pH Die
// 3/1/2020: Modified to print out a constant number of lines for each section, added
//		sensor temp to test results for pH temp correction
// Inputs:	PRINT_AS_MG_HARDNESS; 0 will print total hardness, 1 will print Mg hardness
// Outputs: NONE
//********************************************************************************
void MemoryDump(uint8_t Print_as_mg_hardness, uint8_t Die_RevD)
{
	SetLED(GREEN_BUTTON, 1);

	UARTprintf("Reading data from memory:\n");
	uint16_t Cal_Number = FindCalNumber();
	uint16_t Test_Number = FindTestNumber();
//			uint8_t Test_Number = 25;

	uint8_t * Sensor_SN;
	Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_SN, 7);
	uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
	uint8_t ui8Size_pH = 3;
	uint8_t ui8Size_Ca = 2;
	uint8_t ui8Size_TH = 2;
	uint8_t ui8Size_NH4 = 3;
	if(Sensor_Config == PH_CL_CART)
	{
		ui8Size_pH = 10;
		ui8Size_Ca = 0;
		ui8Size_TH = 0;
		ui8Size_NH4 = 0;
	}

	UARTprintf("Sensor %c%c%c-%c%c%c%c\n", *(Sensor_SN), *(Sensor_SN + 1), *(Sensor_SN + 2), *(Sensor_SN + 3), *(Sensor_SN + 4), *(Sensor_SN + 5), *(Sensor_SN + 6));
	if(Sensor_Config == PH_CL_CART)
		UARTprintf("pH only die\n");
	else
		UARTprintf("Standard die\n");
	UARTprintf("Found %d Calibrations\n", Cal_Number);
	UARTprintf("Found %d Tests\n", Test_Number);
	UARTprintf("\n");

	// Check if these are over number that can fit in memory, if they are only read ones in memory, later ones are written at beginning over older data
	if(Cal_Number > 45)
		Cal_Number = 45;
	if(Test_Number > 124)
		Test_Number = 124;

	//
	// Collect solution data before calculations
	//
	float pH_EEP_Rinse, Ca_EEP_Rinse, TH_EEP_Rinse, NH4_EEP_Rinse, Cond_EEP_Rinse;
	float pH_EEP_Cal_2, Ca_EEP_Cal_2, TH_EEP_Cal_2, NH4_EEP_Cal_2, Cond_EEP_Cal_2;
	float pH_EEP_Cal_1, Ca_EEP_Cal_1, TH_EEP_Cal_1, NH4_EEP_Cal_1, Cond_EEP_Cal_1;
	float pH_EEP_Clean, NH4_EEP_Clean;

	// Rinse
	pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
	Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
	TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_TH, 4));
	NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_NH4, 4));
	Cond_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND, 4));

	// Cal 2
	pH_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_PH, 4));
	Ca_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_CA, 4));
	TH_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_TH, 4));
	NH4_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_NH4, 4));
	Cond_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_COND, 4));

	// Cal 1
	pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
	Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
	TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));
	NH4_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_NH4, 4));
	Cond_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_COND, 4));

	// Clean
	pH_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
	NH4_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_NH4, 4));

	float HCl_N = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_T1_HCL_N, 4));

	UARTprintf("Solution Values:\n");
	UARTprintf("Solution\tpH\tpCa\tp(Ca+Mg)\tpNH4\tCond\n");
	UARTprintf("Rinse\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\n", (int) (pH_EEP_Rinse * 1000), (int) (Ca_EEP_Rinse * 1000), (int) (TH_EEP_Rinse * 1000), (int) (NH4_EEP_Rinse * 1000), (int) (Cond_EEP_Rinse * 1000));
	UARTprintf("Cal 1\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\n", (int) (pH_EEP_Cal_1 * 1000), (int) (Ca_EEP_Cal_1 * 1000), (int) (TH_EEP_Cal_1 * 1000), (int) (NH4_EEP_Cal_1 * 1000), (int) (Cond_EEP_Cal_1 * 1000));
	UARTprintf("Cal 2\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\n", (int) (pH_EEP_Cal_2 * 1000), (int) (Ca_EEP_Cal_2 * 1000), (int) (TH_EEP_Cal_2 * 1000), (int) (NH4_EEP_Cal_2 * 1000), (int) (Cond_EEP_Cal_2 * 1000));
	UARTprintf("\n");

	UARTprintf("Calibration Slopes: \n");
	if(Sensor_Config != PH_CL_CART)
		UARTprintf("\tDate\tpH 1\tpH 2\tpH 3\tTH 1\tTH 2\tNH4 1\tNH4 2\tNH4 3\tCa 1\tCa 2\tCond Low\tCond Mid\tCond High\tChosen\tRepump\tError\n");
	else
		UARTprintf("\tDate\tpH 1\tpH 2\tpH 3\tpH 4\tpH 5\tpH 6\tpH 7\tpH 8\tpH 9\tpH 10\tCond Low\tCond Mid\tCond High\tChosen\tRepump\tError\n");

	uint16_t k;
	for(k = 1; k < (Cal_Number + 1); k++)
	{
		uint16_t Cal_page = ((PAGE_CAL + k * PAGES_FOR_CAL) - PAGES_FOR_CAL);
		uint16_t Cal = *((uint16_t *) MemoryRead(Cal_page, OFFSET_CAL_NUMBER, 2));

		UARTprintf("%d\t", Cal);
		uint8_t *Date = MemoryRead(Cal_page, OFFSET_CAL_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		float pH_EEP_Slope[3];
		float Ca_EEP_Slope[2];
		float NH4_EEP_Slope[3], TH_EEP_Slope[2];
		pH_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_PH_1_SLOPE, 4));
		pH_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_PH_2_SLOPE, 4));
		pH_EEP_Slope[2] = Build_float(MemoryRead(Cal_page, OFFSET_PH_3_SLOPE, 4));
		Ca_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_SLOPE, 4));
		Ca_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_CA_2_SLOPE, 4));
		TH_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_SLOPE, 4));
		TH_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_TH_2_SLOPE, 4));
		NH4_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_SLOPE, 4));
		NH4_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_2_SLOPE, 4));
		NH4_EEP_Slope[2] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_3_SLOPE, 4));
		float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
		float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
		float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));

		// Read chosen sensors from on-chip memory, in actual device will read from cartridge memory as well
		uint8_t ChosenSensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
		uint8_t PS_Chosen_pH = ChosenSensors & 0x03;
		uint8_t PS_Chosen_Ca = (ChosenSensors >> 2) & 0x01;
		uint8_t PS_Chosen_TH = (ChosenSensors >> 3) & 0x01;
		uint8_t PS_Chosen_NH4 = (ChosenSensors >> 4) & 0x03;
//		uint8_t PS_Chosen_Alk = (ChosenSensors >> 6) & 0x03;

		if(PS_Chosen_pH > 2)
			PS_Chosen_pH = 0;
		if(PS_Chosen_Ca > 1)
			PS_Chosen_Ca = 0;
		if(PS_Chosen_TH > 1)
			PS_Chosen_TH = 0;
		if(PS_Chosen_NH4 > 2)
			PS_Chosen_NH4 = 0;

		if(Sensor_Config == PH_CL_CART)
			PS_Chosen_pH = ChosenSensors;

		uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
		uint32_t Error = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_ERROR, 4));

		UARTprintf("=%d/1000\t", (int) (pH_EEP_Slope[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (pH_EEP_Slope[1] * 1000));
		if(Die_RevD)
			UARTprintf("=%d/1000\t", (int) (pH_EEP_Slope[2] * 1000));
		UARTprintf("=%d/1000\t", (int) (TH_EEP_Slope[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (TH_EEP_Slope[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_EEP_Slope[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_EEP_Slope[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_EEP_Slope[2] * 1000));
		UARTprintf("=%d/1000\t", (int) (Ca_EEP_Slope[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (Ca_EEP_Slope[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (CalConductivitySlopeLow * 1000));
		UARTprintf("=%d/1000\t", (int) (CalConductivitySlopeMid * 1000));
		UARTprintf("=%d/1000\t", (int) (CalConductivitySlopeHigh * 1000));
		UARTprintf("%d\t", PS_Chosen_pH);
		if(ui8Size_TH > 0)
			UARTprintf("%d\t", PS_Chosen_TH);
		if(ui8Size_NH4 > 0)
			UARTprintf("%d\t", PS_Chosen_NH4);
		if(ui8Size_Ca > 0)
			UARTprintf("%d\t", PS_Chosen_Ca);
		if((Cal_Status >> 19) & 1)
			UARTprintf("Rinse\t");
		else if((Cal_Status >> 20) & 1)
			UARTprintf("Cal 1\t");
		else if((Cal_Status >> 21) & 1)
			UARTprintf("Cal 2\t");
		else
			UARTprintf("None\t");
		UARTprintf("0x%x\n", Error);

		userDelay(50, 0);
	}
	for(k = (Cal_Number + 1); k < 46; k++)
		UARTprintf("\n");
	UARTprintf("\n");

	UARTprintf("Calibration Raw Data: \n");
	if(Sensor_Config != PH_CL_CART)
		UARTprintf("\tDate\tpH 1 Rinse\tpH 2 Rinse\tpH 3 Rinse\tTH 1 Rinse\tTH 2 Rinse\tNH4 1 Rinse\tNH4 2 Rinse\tNH4 3 Rinse\tCa 1 Rinse\tCa 2 Rinse\tCond Mid 2\tCond High 1\tTemp\tpH 1 Cal 1\tpH 2 Cal 1\tpH 3 Cal 1\tTH 1 Cal 1\tTH 2 Cal 1\tNH4 1 Cal 1\tNH4 2 Cal 1\tNH4 3 Cal 1\tCa 1 Cal 1\tCa 2 Cal 1\tCond High 2\tTemp\tpH 1 Cal 2\tpH 2 Cal 2\tpH 3 Cal 2\tTH 1 Cal 2\tTH 2 Cal 2\tNH4 1 Cal 2\tNH4 2 Cal 2\tNH4 3 Cal 2\tCa 1 Cal 2\tCa 2 Cal 2\tCond Low 2\tCond Mid 1\tTemp\tpH 1 Post\tpH 2 Post\tpH 3 Post\tTH 1 Post\tTH 2 Post\tNH4 1 Post\tNH4 2 Post\tNH4 3 Post\tCa 1 Post\tCa 2 Post\tTemp\tpH Rinse\tpH Cal 1\tpH Cal 2\tpTH Rinse\tpTH Cal 1\tpTH Cal 2\tpNH4 Rinse\tpNH4 Cal 1\tpNH4 Clean\tpCa Rinse\tpCa Cal 1\tpCa Cal 2\n");
	else
		UARTprintf("\tDate\tpH 1 Rinse\tpH 2 Rinse\tpH 3 Rinse\tpH 4 Rinse\tpH 5 Rinse\tpH 6 Rinse\tpH 7 Rinse\tpH 8 Rinse\tpH 9 Rinse\tpH 10 Rinse\tCond Mid 2\tCond High 1\tTemp\tpH 1 Cal 1\tpH 2 Cal 1\tpH 3 Cal 1\tpH 4 Cal 1\tpH 5 Cal 1\tpH 6 Cal 1\tpH 7 Cal 1\tpH 8 Cal 1\tpH 9 Cal 1\tpH 10 Cal 1\tCond High 2\tTemp\tpH 1 Cal 2\tpH 2 Cal 2\tpH 3 Cal 2\tpH 4 Cal 2\tpH 5 Cal 2\tpH 6 Cal 2\tpH 7 Cal 2\tpH 8 Cal 2\tpH 9 Cal 2\tpH 10 Cal 2\tCond Low 2\tCond Mid 1\tTemp\tpH 1 Post\tpH 2 Post\tpH 3 Post\tpH 4 Post\tpH 5 Post\tpH 6 Post\tpH 7 Post\tpH 8 Post\tpH 9 Post\tpH 10 Post\tTemp\tpH Clean\tpH Cal 1\tpH Cal 2\n");

//	float pH_linear_mV[3], Ca_linear_mV[2], TH_linear_mV[2], NH4_linear_mV[3];
	for(k = 1; k < (Cal_Number + 1); k++)
	{
		uint16_t Cal_page = ((PAGE_CAL + k * PAGES_FOR_CAL) - PAGES_FOR_CAL);
		uint16_t Cal = *((uint16_t *) MemoryRead(Cal_page, OFFSET_CAL_NUMBER, 2));

		UARTprintf("%d\t", Cal);
		uint8_t *Date = MemoryRead(Cal_page, OFFSET_CAL_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		// Pull ISE slope and intercept information to calculate raw data
//		float pH_EEP_Slope[3];
//		float Ca_EEP_Slope[2];
//		float NH4_EEP_Slope[3], TH_EEP_Slope[2];
		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
		float ISE_Slope[10];
		float *pH_EEP_Slope = &ISE_Slope[0];
		float *TH_EEP_Slope = &ISE_Slope[3];
		float *NH4_EEP_Slope = &ISE_Slope[5];
		float *Ca_EEP_Slope = &ISE_Slope[8];
		pH_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_PH_1_SLOPE, 4));
		pH_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_PH_2_SLOPE, 4));
		pH_EEP_Slope[2] = Build_float(MemoryRead(Cal_page, OFFSET_PH_3_SLOPE, 4));
		Ca_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_SLOPE, 4));
		Ca_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_CA_2_SLOPE, 4));
		TH_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_SLOPE, 4));
		TH_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_TH_2_SLOPE, 4));
		NH4_EEP_Slope[0] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_SLOPE, 4));
		NH4_EEP_Slope[1] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_2_SLOPE, 4));
		NH4_EEP_Slope[2] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_3_SLOPE, 4));
//		float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
//		float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
//		float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));

//		float pH_EEP_Int[3];
//		float Ca_EEP_Int[2];
//		float NH4_EEP_Int[3], TH_EEP_Int[2];
		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
		float ISE_Int[10];
		float *pH_EEP_Int = &ISE_Int[0];
		float *TH_EEP_Int = &ISE_Int[3];
		float *NH4_EEP_Int = &ISE_Int[5];
		float *Ca_EEP_Int = &ISE_Int[8];
		pH_EEP_Int[0] = Build_float(MemoryRead(Cal_page, OFFSET_PH_1_INT, 4));
		pH_EEP_Int[1] = Build_float(MemoryRead(Cal_page, OFFSET_PH_2_INT, 4));
		pH_EEP_Int[2] = Build_float(MemoryRead(Cal_page, OFFSET_PH_3_INT, 4));
		TH_EEP_Int[0] = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_INT, 4));
		TH_EEP_Int[1] = Build_float(MemoryRead(Cal_page, OFFSET_TH_2_INT, 4));
		NH4_EEP_Int[0] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_INT, 4));
		NH4_EEP_Int[1] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_2_INT, 4));
		NH4_EEP_Int[2] = Build_float(MemoryRead(Cal_page, OFFSET_NH4_3_INT, 4));
		Ca_EEP_Int[0] = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_INT, 4));
		Ca_EEP_Int[1] = Build_float(MemoryRead(Cal_page, OFFSET_CA_2_INT, 4));

		// Pull conductivity slopes and intercepts to calculate raw data
		float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
		float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
		float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));
		float CalConductivityIntLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_INT, 4));
		float CalConductivityIntMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_INT, 4));
		float CalConductivityIntHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_INT, 4));
//		float Cond_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND, 4));
//		float Cond_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_COND, 4));
//		float Cond_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_COND, 4));

//		float pH[3], Ca[2], TH[2], NH4[3];
		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
		float ISE_mV[10];
		float *pH = &ISE_mV[0];
		float *TH = &ISE_mV[3];
		float *NH4 = &ISE_mV[5];
		float *Ca = &ISE_mV[8];
		pH[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_PH_1_RINSE, 4));
		pH[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_PH_2_RINSE, 4));
		pH[2] = Build_float(MemoryRead(Cal_page, OFFSET_CR_PH_3_RINSE, 4));
		TH[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_TH_1_RINSE, 4));
		TH[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_TH_2_RINSE, 4));
		NH4[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_NH4_1_RINSE, 4));
		NH4[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_NH4_2_RINSE, 4));
		NH4[2] = Build_float(MemoryRead(Cal_page, OFFSET_CR_NH4_3_RINSE, 4));
		Ca[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CA_1_RINSE, 4));
		Ca[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CA_2_RINSE, 4));

		float Cond_High, Cond_Low, Temp;
		Temp = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));
		Cond_Low = 1000000000 / (((Cond_EEP_Rinse * (1 + Sols->Rinse_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntMid) * 1000 / CalConductivitySlopeMid);
		Cond_High = 1000000000 / (((Cond_EEP_Rinse * (1 + Sols->Rinse_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntHigh) * 1000 / CalConductivitySlopeHigh);

		uint8_t i;
		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_High * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp * 1000));

		float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

//		float pTH_Rinse;
//		float log_K_Ca_Mg = Build_float(MemoryRead(Test_page, OFFSET_TEST_LOG_K, 4));
//		// Check if values in memory are p-values or concentration
//		if(Ca_EEP_Rinse < 10)	// Values are p-values
//		{
//			float Mg_100Ca_Rinse = -log10(pow(10, -TH_EEP_Rinse) - pow(10, -Ca_EEP_Rinse));
//			pTH_Rinse = -log10(pow(10, -Mg_100Ca_Rinse) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Rinse));
//		}
//		else	// Values are concentration
//			pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_RS, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_RS, IS_RINSE));
//
//		float pNH4_Rinse;
//		if(Ca_EEP_Rinse < 10)	// Values are p-values
//			pNH4_Rinse = NH4_EEP_Rinse;
//		else	// Values are concentration
//			pNH4_Rinse = -log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_RS, IS_RINSE));

		float pH_TCor_Rinse = pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
		float pH_TCor_Cal_1 = pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
		float pH_TCor_Cal_2 = pH_EEP_Cal_2 + K_T_pH_Cal_2 * (T_Cal - 25);
		float pH_TCor_Clean = pH_EEP_Clean + (K_T_pH_Clean_Sq * (pow(T_Cal, 2) - pow(25, 2)) + K_T_pH_Clean_Ln * (T_Cal - 25));	// Temperature corrected pH for Rinse

		float pCa_Rinse, pCa_Cal_1, pCa_Cal_2;
		float pTH_Rinse, pTH_Cal_1, pTH_Cal_2;
		float pNH4_Rinse, pNH4_Cal_1, pNH4_Clean;
		float log_K_Ca_Mg = Build_float(MemoryRead(Cal_page, OFFSET_CAL_LOG_K, 4));
		if(Ca_EEP_Cal_1 < 10)	// Values are p-values
		{
			pCa_Rinse = Ca_EEP_Rinse;
			pCa_Cal_1 = Ca_EEP_Cal_1;
			pCa_Cal_2 = Ca_EEP_Cal_2;

			float Mg_100Ca_Cal_1 = -log10(pow(10, -TH_EEP_Cal_1) - pow(10, -Ca_EEP_Cal_1));
			float Mg_100Ca_Cal_2 = -log10(pow(10, -TH_EEP_Cal_2) - pow(10, -Ca_EEP_Cal_2));

			pTH_Cal_1 = -log10(pow(10, -Mg_100Ca_Cal_1) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Cal_1));
			pTH_Cal_2 = -log10(pow(10, -Mg_100Ca_Cal_2) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Cal_2));

			pNH4_Rinse = NH4_EEP_Rinse;
			pNH4_Cal_1 = NH4_EEP_Cal_1;
			pNH4_Clean = NH4_EEP_Clean;


		}
		else	// Values are concentration
		{
			pCa_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE));
			pCa_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1));
			pCa_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2));

			pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_Cal, IS_RINSE));
			pTH_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_1 - Ca_EEP_Cal_1) / 100090 * Lambda_Mg(T_Cal, IS_CAL_1));
			pTH_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_2 - Ca_EEP_Cal_2) / 100090 * Lambda_Mg(T_Cal, IS_CAL_2));


			pNH4_Rinse = -log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_RINSE));
			pNH4_Cal_1 = -log10(NH4_EEP_Cal_1 / 14000 * pow(10, -pH_TCor_Cal_1) / (pow(10, -pH_TCor_Cal_1) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CAL_1) + 0.010928 * Lambda_Na(T_Cal, IS_CAL_1) * pow(10, LOG_K_NA_NH4));
			pNH4_Clean = -log10(NH4_EEP_Clean / 14000 * pow(10, -pH_TCor_Clean) / (pow(10, -pH_TCor_Clean) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CLEAN));
		}

		for(i = 0; i < ui8Size_pH; i++)
			pH[i] = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (Temp - 25)) + pH_EEP_Int[i];
		for(i = 0; i < ui8Size_TH; i++)
			TH[i] = TH_EEP_Slope[i] * pTH_Cal_1 + TH_EEP_Int[i];
		for(i = 0; i < ui8Size_NH4; i++)
			NH4[i] = NH4_EEP_Slope[i] * pNH4_Cal_1 + NH4_EEP_Int[i];
		for(i = 0; i < ui8Size_Ca; i++)
			Ca[i] = Ca_EEP_Slope[i] * pCa_Cal_1 + Ca_EEP_Int[i];

		Cond_High = 1000000000 / (((Cond_EEP_Cal_1 * (1 + Sols->Cal_1_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntHigh) * 1000 / CalConductivitySlopeHigh);

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_High * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp * 1000));

		for(i = 0; i < ui8Size_pH; i++)
			pH[i] = pH_EEP_Slope[i] * (pH_EEP_Cal_2 + K_T_pH_Cal_2 * (Temp - 25)) + pH_EEP_Int[i];
		for(i = 0; i < ui8Size_TH; i++)
			TH[i] = TH_EEP_Slope[i] * pTH_Cal_2 + TH_EEP_Int[i];
		for(i = 0; i < ui8Size_NH4; i++)
			NH4[i] = NH4_EEP_Slope[i] * pNH4_Clean + NH4_EEP_Int[i];
		for(i = 0; i < ui8Size_Ca; i++)
			Ca[i] = Ca_EEP_Slope[i] * pCa_Cal_2 + Ca_EEP_Int[i];

		Cond_Low = 1000000000 / (((Cond_EEP_Cal_2 * (1 + Sols->Cal_2_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntLow) * 1000 / CalConductivitySlopeLow);
		Cond_High = 1000000000 / (((Cond_EEP_Cal_2 * (1 + Sols->Cal_2_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntMid) * 1000 / CalConductivitySlopeMid);
//		Temp = Build_float(MemoryRead(Cal_page + 2, OFFSET_CR_TEMP_CAL_2, 4));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_High * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp * 1000));

		pH[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_PH_1_POSTRINSE, 4));
		pH[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_PH_2_POSTRINSE, 4));
		pH[2] = Build_float(MemoryRead(Cal_page, OFFSET_CR_PH_3_POSTRINSE, 4));
		TH[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_TH_1_POSTRINSE, 4));
		TH[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_TH_2_POSTRINSE, 4));
		NH4[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_NH4_1_POSTRINSE, 4));
		NH4[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_NH4_2_POSTRINSE, 4));
		NH4[2] = Build_float(MemoryRead(Cal_page, OFFSET_CR_NH4_3_POSTRINSE, 4));
		Ca[0] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CA_1_POSTRINSE, 4));
		Ca[1] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CA_2_POSTRINSE, 4));

//		Temp = Build_float(MemoryRead(Cal_page, OFFSET_CR_T_POSTRINSE, 4));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp * 1000));

		// Print out p-values calculated for this calibration
		if(Sensor_Config != PH_CL_CART)
			UARTprintf("=%d/1000\t", (int) (pH_TCor_Rinse * 1000));
		else
			UARTprintf("=%d/1000\t", (int) (pH_TCor_Clean * 1000));
		UARTprintf("=%d/1000\t", (int) (pH_TCor_Cal_1 * 1000));
		UARTprintf("=%d/1000\t", (int) (pH_TCor_Cal_2 * 1000));

		if(Sensor_Config != PH_CL_CART)
		{
			UARTprintf("=%d/1000\t", (int) (pTH_Rinse * 1000));
			UARTprintf("=%d/1000\t", (int) (pTH_Cal_1 * 1000));
			UARTprintf("=%d/1000\t", (int) (pTH_Cal_2 * 1000));
			UARTprintf("=%d/1000\t", (int) (pNH4_Rinse * 1000));
			UARTprintf("=%d/1000\t", (int) (pNH4_Cal_1 * 1000));
			UARTprintf("=%d/1000\t", (int) (pNH4_Clean * 1000));
			UARTprintf("=%d/1000\t", (int) (pCa_Rinse * 1000));
			UARTprintf("=%d/1000\t", (int) (pCa_Cal_1 * 1000));
			UARTprintf("=%d/1000\t", (int) (pCa_Cal_2 * 1000));
		}

		UARTprintf("\n");
		userDelay(50, 0);
	}
	for(k = (Cal_Number + 1); k < 46; k++)
		UARTprintf("\n");
	UARTprintf("\n");

	UARTprintf("Test Results: \n");
	if(Sensor_Config != PH_CL_CART)
		UARTprintf("\tDate\tpH 1\t pH 2\tpH 3\tTH 1\tTH 2\tNH4 1\t NH4 2\tNH4 3\tNH4 1 T1\tNH4 2 T1\tNH4 3 T1\tCa 1\tCa 2\tFCl\tSteps B1\tTCl\tSteps B2\tAlk 1\tMethod 1\tSlope 1\tAlk 2\tMethod 2\tSlope 2\tAlk 3\tMethod 3\tSlope 3\tConductivity\tORP\tTherm Temp\tSensor Temp\tCal pH\tCal TH\tCal NH4\tCal Ca\tCal Alk\tTest pH\tTest TH\tTest NH4\tTest Ca\tTest Alk\tError\n");
	else
		UARTprintf("\tDate\tpH 1\t pH 2\tpH 3\tpH 4\tpH 5\tpH 6\tpH 7\tpH 8\tpH 9\tpH 10\tFCl\tSteps B1\tTCl\tSteps B2\tConductivity\tORP\tTherm Temp\tSensor Temp\tCal Chosen pH\tTest Chosen pH\tError\n");
//	if(Die_RevD)
//	{
//		if(Print_as_mg_hardness)
//			UARTprintf("\tDate\tpH 1\t pH 2\tpH 3\tMg 1\tMg 2\tNH4 1\t NH4 2\tNH4 3\tNH4 1 T1\tNH4 2 T1\t NH4 3 T1\tCa 1\tCa 2\tFCl\tSteps B1\tTCl\tSteps B2\tAlk 1\tMethod 1\tSlope 1\tAlk 2\tMethod 2\tSlope 2\tAlk 3\tMethod 3\tSlope 3\tConductivity\tORP\tCal pH\tCal TH\tCal NH4\tCal Ca\tCal Alk\tTest pH\tTest TH\tTest NH4\tTest Ca\tTest Alk\tError\n");
//		else
//			UARTprintf("\tDate\tpH 1\t pH 2\tpH 3\tTH 1\tTH 2\tNH4 1\t NH4 2\tNH4 3\tNH4 1 T1\tNH4 2 T1\tNH4 3 T1\tCa 1\tCa 2\tFCl\tSteps B1\tTCl\tSteps B2\tAlk 1\tMethod 1\tSlope 1\tAlk 2\tMethod 2\tSlope 2\tAlk 3\tMethod 3\tSlope 3\tConductivity\tORP\tCal pH\tCal TH\tCal NH4\tCal Ca\tCal Alk\tTest pH\tTest TH\tTest NH4\tTest Ca\tTest Alk\tError\n");
////			UARTprintf("\tDate\tpH 1\t pH 2\tpH 3\tCa 1\tCa 2\tTH 1\tTH 2\tNH4 1\t NH4 2\tNH4 3\tFCl\tFCl 2\tSteps B1\tTCl\tTCl 2\tSteps B2\tAlk 1\tMethod 1\tAlk 2\tMethod 2\tAlk 3\tMethod 3\tConductivity\tORP\t\t\t\tError\n");
//	}
//	else
//	{
//		if(Print_as_mg_hardness)
//			UARTprintf("\tDate\tpH 1\t pH 2\tMg 1\tMg 2\tNH4 1\t NH4 2\tNH4 3\tCa 1\tCa 2\tFCl\tSteps B1\tTCl\tSteps B2\tAlk 1\tAlk 2\tConductivity\tORP\t\t\t\tError\n");
//		else
//			UARTprintf("\tDate\tpH 1\t pH 2\tTH 1\tTH 2\tNH4 1\t NH4 2\tNH4 3\tCa 1\tCa 2\tFCl\tSteps B1\tTCl\tSteps B2\tAlk 1\tAlk 2\tConductivity\tORP\t\t\t\tError\n");
//	}

	for(k = 1; k < (Test_Number + 1); k++)
	{
		uint16_t Test_page = ((PAGE_TEST + k * PAGES_FOR_TEST) - PAGES_FOR_TEST);
		uint16_t Test = *((uint16_t *) MemoryRead(Test_page, OFFSET_TEST_NUMBER, 2));

		UARTprintf("%d\t", Test);
		uint8_t *Date = MemoryRead(Test_page, OFFSET_TEST_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		uint8_t Test_Cal_Number = *(MemoryRead(Test_page, OFFSET_TEST_CAL, 1));
		uint16_t Cal_page = Find_Cal_page(Test_Cal_Number);
		uint8_t i;
		uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
		uint8_t Last_cal_passed[10];
		memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
		for(i = 0; i < 10; i++)
			if(Last_cal_passed[i] == 0xFF)
				Last_cal_passed[i] = Test_Cal_Number;

		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
//		float pH_EEP_Slope[3];
//		float Ca_EEP_Slope[2];
//		float NH4_EEP_Slope[3], TH_EEP_Slope[2];
		float ISE_EEP_Slope[10];
		float *pH_EEP_Slope = &ISE_EEP_Slope[0];
		float *TH_EEP_Slope = &ISE_EEP_Slope[3];
		float *NH4_EEP_Slope = &ISE_EEP_Slope[5];
		float *Ca_EEP_Slope = &ISE_EEP_Slope[8];
		pH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_SLOPE, 4));
		pH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_SLOPE, 4));
		pH_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_SLOPE, 4));
		TH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[3]), OFFSET_TH_1_SLOPE, 4));
		TH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[4]), OFFSET_TH_2_SLOPE, 4));
		NH4_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[5]), OFFSET_NH4_1_SLOPE, 4));
		NH4_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[6]), OFFSET_NH4_2_SLOPE, 4));
		NH4_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[7]), OFFSET_NH4_3_SLOPE, 4));
		Ca_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[8]), OFFSET_CA_1_SLOPE, 4));
		Ca_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[9]), OFFSET_CA_2_SLOPE, 4));

		// Collect temperatures
		float T_Rinse = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_RINSE, 4));
		float T_Samp = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));
		float T_RS = (T_Rinse + T_Samp) / 2;
		float T_EEP_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

		// Collect sample mV data
		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
//		float pH_E_Samp[3], Ca_E_Samp[2], TH_E_Samp[2], NH4_E_Samp[3];
		float ISE_E_Samp[10];
		float *pH_E_Samp = &ISE_E_Samp[0];
		float *TH_E_Samp = &ISE_E_Samp[3];
		float *NH4_E_Samp = &ISE_E_Samp[5];
		float *Ca_E_Samp = &ISE_E_Samp[8];
		pH_E_Samp[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_1_SAMP, 4));
		pH_E_Samp[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_2_SAMP, 4));
		pH_E_Samp[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_3_SAMP, 4));
		TH_E_Samp[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_1_SAMP, 4));
		TH_E_Samp[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_2_SAMP, 4));
		NH4_E_Samp[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_1_SAMP, 4));
		NH4_E_Samp[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_2_SAMP, 4));
		NH4_E_Samp[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_3_SAMP, 4));
		Ca_E_Samp[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_1_SAMP, 4));
		Ca_E_Samp[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_2_SAMP, 4));

		// Collect rinse mV data
//		float pH_E_Rinse[3], Ca_E_Rinse[2], TH_E_Rinse[2], NH4_E_Rinse[3];
		float ISE_E_Rinse[10];
		float *pH_E_Rinse = &ISE_E_Rinse[0];
		float *TH_E_Rinse = &ISE_E_Rinse[3];
		float *NH4_E_Rinse = &ISE_E_Rinse[5];
		float *Ca_E_Rinse = &ISE_E_Rinse[8];
		pH_E_Rinse[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_1_RINSE, 4));
		pH_E_Rinse[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_2_RINSE, 4));
		pH_E_Rinse[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_3_RINSE, 4));
		Ca_E_Rinse[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_1_RINSE, 4));
		Ca_E_Rinse[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_2_RINSE, 4));
		TH_E_Rinse[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_1_RINSE, 4));
		TH_E_Rinse[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_2_RINSE, 4));
		NH4_E_Rinse[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_1_RINSE, 4));
		NH4_E_Rinse[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_2_RINSE, 4));
		NH4_E_Rinse[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_3_RINSE, 4));

		// Read conductivity
		float Conductivity_TCorrected = Build_float(MemoryRead(Test_page, OFFSET_TEST_COND, 4));
		//
		// Conductivity Temperature Correction
		//
		// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
		float Conductivity = Conductivity_TCorrected * (1 + COND_TCOMP_SAMP*(T_Samp - 25));

		uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
		uint8_t pH_Cal_Status[3] = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
//		uint8_t Ca_Cal_Status[2] = {(Cal_Status >> 3) & 1, (Cal_Status >> 4) & 1};
//		uint8_t TH_Cal_Status[2] = {(Cal_Status >> 5) & 1, (Cal_Status >> 6) & 1};
//		uint8_t NH4_Cal_Status[3] = {(Cal_Status >> 7) & 1, (Cal_Status >> 8) & 1, (Cal_Status >> 9) & 1};

		uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1)); // Alk, Alk, NH4, NH4, TH, Ca, pH, pH
		uint8_t PS_Chosen_pH = Chosen_Sensors & 0x03;
		uint8_t PS_Chosen_Ca = (Chosen_Sensors >> 2) & 0x01;
		uint8_t PS_Chosen_TH = (Chosen_Sensors >> 3) & 0x01;
		uint8_t PS_Chosen_NH4 = (Chosen_Sensors >> 4) & 0x03;
		uint8_t PS_Chosen_Alk = PS_Chosen_pH;

		if(PS_Chosen_pH > 2)
			PS_Chosen_pH = 0;
		if(PS_Chosen_Ca > 1)
			PS_Chosen_Ca = 0;
		if(PS_Chosen_TH > 1)
			PS_Chosen_TH = 0;
		if(PS_Chosen_NH4 > 2)
			PS_Chosen_NH4 = 0;
		if(PS_Chosen_Alk > 2)
			PS_Chosen_Alk = 0;

		float ISE_Reading[10];

		float T_Therm = Build_float(MemoryRead(Test_page, OFFSET_TEST_T_THERM, 4));
		float T_Sensor = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));

		// pH
//		float K_T_Rinse = Rinse_Table_Lookup(T_Rinse);	// Temperature coefficient of Rinse
		float pH_TCor_Rinse = pH_EEP_Rinse + K_T_pH_Rinse * (T_RS - 25);	// Temperature corrected pH for Rinse
		if(Sensor_Config == PH_CL_CART)
			pH_TCor_Rinse = pH_EEP_Clean + (K_T_pH_Clean_Sq * (pow(T_RS, 2) - pow(25, 2)) + K_T_pH_Clean_Ln * (T_RS - 25));	// Temperature corrected pH for Rinse

//		float pH_Slope_SampT[3], pH_Samp[3]; //pH_E_Samp_TCor[3];//, pH_Samp[2];
		float *pH_Samp = &ISE_Reading[0];
//		float pH_Samp_RS[10];

		for(i = 0; i < ui8Size_pH; i++)
		{
			float pH_Slope_SampT = pH_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
			pH_Samp[i] = pH_TCor_Rinse + ((pH_E_Samp[i] - pH_E_Rinse[i]) / pH_Slope_SampT); // pH of sample
//			pH_Samp[i] = pH_Samp_RS[i] + K_T_pH_Samp * (T_Therm - T_RS);
		}

		uint8_t T_Chosen_pH;
		if(Sensor_Config != PH_CL_CART)
			T_Chosen_pH = Choose_pH_Sensor(Test_Cal_Number, pH_Samp, pH_E_Rinse, T_Rinse);
		else
			T_Chosen_pH = Choose_pH_Sensor_pHDie(Test_Cal_Number, pH_Samp);

		//
		// Ca Measurement
		//
		float IS;
		if(Conductivity > 62)
			IS = 0.000016 * Conductivity;
		else
			IS = 0.00001 * Conductivity;

//		float Ca_Slope_SampT[2], Ca_Samp[2];
//		float /*Ca_Hardness[2],*/ Ca_M_activity[2], Ca_M_conc[2];//, Ca_ppm[2];
		float Ca_M_activity, Ca_M_conc;
		float *Ca_Hardness = &ISE_Reading[8];

		float pCa_Rinse;
		// Check if values in the memory are p-values or concentrations
		if(Ca_EEP_Rinse < 10)	// Values are p-values
		{
			pCa_Rinse = Ca_EEP_Rinse;
		}
		else	// Values are concentrations
			pCa_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_RS, IS_RINSE));

		for(i = 0; i < ui8Size_Ca; i++)
		{
			float Ca_Slope_RST = Ca_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
			// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
			float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i]) / Ca_Slope_RST); //pCa

			Ca_M_activity = pow(10, -Ca_Samp);
//			Ca_ppm[i] = Ca_M_activity[i] / Lambda_Ca(T_RS, IS) * 40078;	// [Ca2+]ppm Ca
			Ca_Hardness[i] = Ca_M_activity / Lambda_Ca(T_RS, IS) * 100086.9;	// [Ca Hardness] ppm CaCO3
		}

		uint8_t T_Chosen_Ca = Choose_Ca_Sensor(Test_Cal_Number, Ca_Hardness, Ca_E_Rinse);

		// Recalculate Ca M activity and Ca M conc with the chosen sensor, if the chosen sensor wasn't the last one in the loop, this will be used in TH calculation
		if((T_Chosen_Ca + 1) != ui8Size_Ca)
		{
			float Ca_Slope_RST = Ca_EEP_Slope[T_Chosen_Ca] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
			float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - Ca_E_Rinse[T_Chosen_Ca]) / Ca_Slope_RST); //pCa
			Ca_M_activity = pow(10, -Ca_Samp);
			Ca_M_conc = Ca_M_activity / Lambda_Ca(T_RS, IS);
		}

		//
		// Magnesium and Total Hardness Measurement (log K 0.15)
		//
		float pTH_Rinse;
		float log_K_Ca_Mg = Build_float(MemoryRead(Test_page, OFFSET_TEST_LOG_K, 4));
		// Check if values in memory are p-values or concentration
		if(Ca_EEP_Rinse < 10)	// Values are p-values
		{
			float Mg_100Ca_Rinse = -log10(pow(10, -TH_EEP_Rinse) - pow(10, -Ca_EEP_Rinse));
			pTH_Rinse = -log10(pow(10, -Mg_100Ca_Rinse) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Rinse));
		}
		else	// Values are concentration
			pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_RS, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_RS, IS_RINSE));

		float *TH_corr = &ISE_Reading[3];
		for(i = 0; i < ui8Size_TH; i++)
		{
			float TH_Slope_RST = TH_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
			float Mg_perCa_Samp = pTH_Rinse + ((TH_E_Samp[i] - TH_E_Rinse[i]) / TH_Slope_RST);
			float Mg_perCa = pow(10, -Mg_perCa_Samp);	// a[Mg+X%Ca]
			float Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
			float Mg_M_activity = Mg_100perCa - Ca_M_activity;
			float Mg_M_conc = Mg_M_activity / Lambda_Mg(T_RS, IS);
			if(Mg_100perCa > Ca_M_activity)
				TH_corr[i] = (Ca_M_conc + Mg_M_conc) * 100086.9; // [TH]
			else
				TH_corr[i] = Ca_Hardness[T_Chosen_Ca];
		}

		uint8_t T_Chosen_TH = Choose_TH_Sensor(Test_Cal_Number, TH_corr, TH_E_Rinse);

		//
		// NH4 Measurement
		//
		float NH4_Lambda;
		if(Conductivity > 62)
			NH4_Lambda = pow(10, -((0.512 * sqrt(IS)) / (1.0 + 0.984 * sqrt(IS))));
		else
			NH4_Lambda = 1;

		// NH4
//		float NH4_NH3_N_Free[3];//, NH4_NH3_N_Total[3];
//		float *NH4_NH3_N_Free = &ISE_Reading[5];
		float *NH4_NH3_N_Total = &ISE_Reading[5];

		float NH4_Alpha = pow(10, -pH_Samp[T_Chosen_pH]) / (pow(10, -pH_Samp[T_Chosen_pH]) + pow(10, -9.25));
//		float NH4_Samp[3];
//		float NH4_Slope_SampT[3];//, NH4_E_Samp_TCor[3];

		float pNH4_Rinse;
		if(Ca_EEP_Rinse < 10)	// Values are p-values
			pNH4_Rinse = NH4_EEP_Rinse;
		else	// Values are concentration
			pNH4_Rinse = -log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_RS, IS_RINSE));

		for(i = 0; i < ui8Size_NH4; i++)
		{
//			float NH4_Slope_SampT = NH4_EEP_Slope[i] * (T_Samp + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
//			float NH4_Samp = NH4_EEP_Rinse + ((NH4_E_Samp[i] - NH4_E_Rinse[i]) / NH4_Slope_SampT);	// pNH4
//			float NH4_NH3_N_Free = pow(10, -NH4_Samp) * 14000.0 / NH4_Lambda - NH4K_Interference; // Free Ammonia
//			NH4_NH3_N_Total[i] = NH4_NH3_N_Free / NH4_Alpha; // Total ammonia not including monochloramine
			float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
			float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

			float Activity_NH4_K_Na = pow(10, -NH4_Samp);

			// TODO: Enter potassium and sodium interference values
			float K_interference = 1; // ppm
			float Na_interference = 0; // ppm
			float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS);
			float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS);
			float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
			float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;

			float NH4_NH3_N_Free = Activity_NH4 / Lambda_NH4(T_RS, IS) * 14000;
			NH4_NH3_N_Total[i] = NH4_NH3_N_Free / NH4_Alpha;

		}

		uint8_t T_Chosen_NH4 = Choose_NH4_Sensor(Test_Cal_Number, NH4_NH3_N_Total, NH4_E_Rinse);
//		uint8_t T_Chosen_NH4 = Choose_NH4_Sensor(Test_Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse);

		// Alkalinity
		float Alk_Samp[3] = {-1, -1, -1};
		float Alk_Slope[3] = {0, 0, 0};
		uint8_t method[3] = {0,0,0};
		float NH4_NH3_N_Total_T1[3];
		float NH4_NH3_N_Free_T1[3];
		uint8_t T_Chosen_Alk;
		if(Sensor_Config != PH_CL_CART)	// Check that it is not the pH Cl cartridge, will have to adjust this when adding other configurations
		{
			float pH_Samp_T1[6] = {0,0,0,0,0,0};	// Collect 2 pH readings for each sensor {pH 1 Mix 1, pH 2 Mix 1, pH 3 Mix 1, pH 1 Mix 2, pH 2 Mix 2, pH 3 Mix 2};
			pH_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_1, 4));
			pH_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_1, 4));
			pH_Samp_T1[2] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_1, 4));
			pH_Samp_T1[3] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_2, 4));
			pH_Samp_T1[4] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_2, 4));
			pH_Samp_T1[5] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_2, 4));

			uint16_t Steps_T1[2];	// First mixing, Second mixing
			Steps_T1[0] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_1, 2));
			Steps_T1[1] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_2, 2));

			float T_Samp_T1[2];
			T_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_SAMP_T1_1, 4));
			T_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_SAMP_T1_2, 4));

			float Conductivity_T1 = Build_float(MemoryRead(Test_page, OFFSET_TEST_COND_T1, 4));

			float Steps_Sample[2] = {0,0};	// First mixing, Second mixing
			for(i = 0; i < 2; i++)	// Calculate steps of sample pumped for each mixing
			{
				Steps_Sample[i] = 4320;
				if(Steps_T1[i] > 600)
					Steps_Sample[i] = 4370;
			}

			float Steps_T1_Endpoint[3] = {0,0,0};	// Sensor 1, Sensor 2, Sensor 3

			for(i = 0; i < 3; i++)	// Iterate through each sensor
			{
				// First check if we found the endpoint on either mix, then pick the mix that was closest to 4.5
				if(((pH_Samp_T1[i] >= 4.1 && pH_Samp_T1[i] <= 4.9) || (pH_Samp_T1[i + 3] >= 4.1 && pH_Samp_T1[i + 3] <= 4.9)) && pH_Cal_Status[i])
				{
					if(abs_val(pH_Samp_T1[i] - 4.5) < abs_val(pH_Samp_T1[i + 3] - 4.5))
						Steps_T1_Endpoint[i] = Steps_T1[0];
					else
						Steps_T1_Endpoint[i] = Steps_T1[1];
					method[i] = 2;
				}
				if(Steps_T1_Endpoint[i] == 0 && pH_Cal_Status[i])	// Calculate endpoint if pH sensor passed calibration and hasn't already found endpoint
				{
					if(pH_Samp_T1[i] > 4.9 && pH_Samp_T1[i + 3] > 4.9)	// Both pH points landed above 4.9, endpoint
					{
						float M = (((float) Steps_T1[1] * pow(10, pH_Samp_T1[i + 3])) - ((float) Steps_T1[0] * pow(10, pH_Samp_T1[i])))/((float) Steps_T1[1] - (float) Steps_T1[0]);
						float B = ((float) Steps_T1[0] * pow(10, pH_Samp_T1[i])) - (M * (float) Steps_T1[0]);
						Steps_T1_Endpoint[i] = -B/M;
						method[i] = 1;

						Alk_Slope[i] = log10(-1.0/M);
					}
					else if(pH_Samp_T1[i] < 4.1 && pH_Samp_T1[i + 3] < 4.1 && pH_Samp_T1[i + 3] != 0)	// Both pH points landed below 4.1 endpoint, and there is saved data
					{
						float M = ((((float) Steps_Sample[1] + (float) Steps_T1[1]) * pow(10, -pH_Samp_T1[i + 3])) - (((float) Steps_Sample[0] + (float) Steps_T1[0]) * pow(10, -pH_Samp_T1[i]))) / ((float) Steps_T1[1] - (float) Steps_T1[0]);
						float B = (((float) Steps_Sample[0] + (float) Steps_T1[0]) * pow(10, -pH_Samp_T1[i])) - (M * (float) Steps_T1[0]);
						Steps_T1_Endpoint[i] = -B/M;
						method[i] = 3;

						Alk_Slope[i] = M;
					}
					else if(pH_Samp_T1[i + 3] != 0)	// Check that we have both pH points, if we do then it used interpolation method, if not that means didn't get mixing correct
					{
						//					UARTprintf("pH Values for sensor %u were above and below endpoint\n", i + 1);
						Steps_T1_Endpoint[i] = ((4.5 - pH_Samp_T1[i + 3]) * ((float) Steps_T1[0] - (float) Steps_T1[1]))/(pH_Samp_T1[i] - pH_Samp_T1[i + 3]) + (float) Steps_T1[1];
						//					UARTprintf("Interpolated steps for endpoint\n\n");
						method[i] = 4;
					}
				}
				if(Steps_T1_Endpoint[i] != 0)
				{
					float Steps_Samp_Endpoint = (Steps_Sample[0] + Steps_Sample[1]) / 2;

					Alk_Samp[i] = 50044.0 * HCl_N * Steps_T1_Endpoint[i] / (Steps_Samp_Endpoint);
				}
			}

			T_Chosen_Alk = Choose_Alk_Sensor(Test_Cal_Number, Alk_Samp, pH_E_Rinse, T_Rinse, method, Alk_Slope);

			float NH4_E_Samp_T1[3];
			NH4_E_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_1_T1, 4));
			NH4_E_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_2_T1, 4));
			NH4_E_Samp_T1[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_3_T1, 4));

			//
			// NH4 Measurement T1
			//
			float NH4_Samp_T1[3];
			float NH4_Slope_SampT_T1[3];//, NH4_E_Samp_T1_TCor[3];
			for(i = 0; i < 3; i++)
			{
				NH4_Slope_SampT_T1[i] = NH4_EEP_Slope[i] * (T_Samp_T1[1] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
				NH4_Samp_T1[i] = NH4_EEP_Rinse + ((NH4_E_Samp_T1[i] - NH4_E_Rinse[i]) / NH4_Slope_SampT_T1[i]);	// pNH4
			}

			float IS_T1;
			if(Conductivity_T1 > 62)
				IS_T1 = 0.000016 * Conductivity_T1;
			else
				IS_T1 = 0.00001 * Conductivity_T1;

			if(Conductivity_T1 > 62)
				NH4_Lambda = pow(10, -((0.512 * sqrt(IS_T1)) / (1.0 + 0.984 * sqrt(IS_T1))));
			else
				NH4_Lambda = 1;

			// NH4
			//				float /*NH4_NH3_N_Free[3],*/ NH4_Alpha;//, NH4_NH3_N_Total[3];
			NH4_Alpha = pow(10, -pH_Samp_T1[T_Chosen_pH]) / (pow(10, -pH_Samp_T1[T_Chosen_pH]) + pow(10, -9.25));

			for(i = 0; i < 3; i++)
			{
				NH4_NH3_N_Free_T1[i] = ((pow(10, -NH4_Samp_T1[i]) * 14000.0 / NH4_Lambda));
				NH4_NH3_N_Total_T1[i] = NH4_NH3_N_Free_T1[i] / NH4_Alpha; // Total ammonia not including monochloramine
			}

			if(pH_Samp[T_Chosen_pH] >= 8.5)
				T_Chosen_NH4 = Choose_NH4_Sensor(Test_Cal_Number, NH4_NH3_N_Total_T1, NH4_E_Rinse);
		}

		float FCl = Build_float(MemoryRead(Test_page, OFFSET_TEST_FREE_CL, 4));
		float TCl = Build_float(MemoryRead(Test_page, OFFSET_TEST_TOTAL_CL, 4));
		float FCl_2 = Build_float(MemoryRead(Test_page, 112, 4));
		float TCl_2 = Build_float(MemoryRead(Test_page, 116, 4));

		uint32_t Error = *((uint32_t *) MemoryRead(Test_page, OFFSET_TEST_ERROR, 4));

		float ORP = Build_float(MemoryRead(Test_page, OFFSET_TEST_ORP, 4));

//		MemoryWrite(Test_page, OFFSET_TEST_NITRITE_BACK_NA, 4, (uint8_t *) &Nitrite_Samp_nA);
//		MemoryWrite(Test_page, OFFSET_TEST_NITRITE_NA, 4, (uint8_t *) &Nitrite_SampNitrite_nA);
		float Nitrite_Samp = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE, 4));

//		UARTprintf("=%d/1000\t", (int) (pH_Samp[0] * 1000));
//		UARTprintf("=%d/1000\t", (int) (pH_Samp[1] * 1000));
//		if(Die_RevD)
//			UARTprintf("=%d/1000\t", (int) (pH_Samp[2] * 1000));
//		UARTprintf("=%d/1000\t", (int) (Ca_Hardness[0] * 1000));
//		UARTprintf("=%d/1000\t", (int) (Ca_Hardness[1] * 1000));
//		if(Print_as_mg_hardness)
//		{
//			UARTprintf("=%d/1000\t", (int) ((TH_corr[0] - Ca_Hardness[T_Chosen_Ca]) * 1000));
//			UARTprintf("=%d/1000\t", (int) ((TH_corr[1] - Ca_Hardness[T_Chosen_Ca]) * 1000));
//		}
//		else
//		{
//			UARTprintf("=%d/1000\t", (int) (TH_corr[0] * 1000));
//			UARTprintf("=%d/1000\t", (int) (TH_corr[1] * 1000));
//		}
//
//		UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Total[0] * 1000));
//		UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Total[1] * 1000));
//		UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Total[2] * 1000));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_Reading[i] * 1000));

		if(Sensor_Config != PH_CL_CART)
		{
			UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Total_T1[0] * 1000));
			UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Total_T1[1] * 1000));
			UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Total_T1[2] * 1000));
		}
		UARTprintf("=%d/1000\t", (int) (FCl * 1000));
//		UARTprintf("=%d/1000\t", (int) (FCl_2 * 1000));
		UARTprintf("=480\t");
		UARTprintf("=%d/1000\t", (int) (TCl * 1000));
//		UARTprintf("=%d/1000\t", (int) (TCl_2 * 1000));
		UARTprintf("=480\t");
		if(Sensor_Config != PH_CL_CART)
		{
			UARTprintf("=%d/1000\t", (int) (Alk_Samp[0] * 1000));
			UARTprintf("%u\t", method[0]);
			UARTprintf("=%d/1000\t", (int) (Alk_Slope[0] * 1000));
			UARTprintf("=%d/1000\t", (int) (Alk_Samp[1] * 1000));
			UARTprintf("%u\t", method[1]);
			UARTprintf("=%d/1000\t", (int) (Alk_Slope[1] * 1000));
			if(Die_RevD)
			{
				UARTprintf("=%d/1000\t", (int) (Alk_Samp[2] * 1000));
				UARTprintf("%u\t", method[2]);
				UARTprintf("=%d/1000\t", (int) (Alk_Slope[2] * 1000));
			}
		}
		UARTprintf("=%d/1000\t", (int) (Conductivity_TCorrected * 1000));
		UARTprintf("=%d/1000\t", (int) (ORP * 1000));
		UARTprintf("=%d/1000\t", (int) (T_Therm * 1000));
		UARTprintf("=%d/1000\t", (int) (T_Sensor * 1000));
		if(Sensor_Config == PH_CL_CART)
			UARTprintf("=%d/1000\t", (int) (Nitrite_Samp * 1000));
//		UARTprintf("=%d\t", ORP_2);
		UARTprintf("%d\t", PS_Chosen_pH);
		if(Sensor_Config != PH_CL_CART)
		{
			UARTprintf("%d\t", PS_Chosen_TH);
			UARTprintf("%d\t", PS_Chosen_NH4);
			UARTprintf("%d\t", PS_Chosen_Ca);
			UARTprintf("%d\t", PS_Chosen_Alk);
		}
		UARTprintf("%d\t", T_Chosen_pH);
		if(Sensor_Config != PH_CL_CART)
		{
			UARTprintf("%d\t", T_Chosen_TH);
			UARTprintf("%d\t", T_Chosen_NH4);
			UARTprintf("%d\t", T_Chosen_Ca);
			UARTprintf("%d\t", T_Chosen_Alk);
		}

		UARTprintf("0x%x\n", Error);

		userDelay(50, 0);
	}
	for(k = (Test_Number + 1); k < 125; k++)
		UARTprintf("\n");
	UARTprintf("\n");

	UARTprintf("Raw Test Data: \n");
	if(Sensor_Config != PH_CL_CART)
		UARTprintf("\tDate\tpH 1 Rinse\tpH 2 Rinse\tpH 3 Rinse\tTH 1 Rinse\tTH 2 Rinse\tNH4 1 Rinse\t NH4 2 Rinse\tNH4 3 Rinse\tCa 1 Rinse\tCa 2 Rinse\tT Rinse\tpH 1 Samp\tpH 2 Samp\tpH 3 Samp\tTH 1 Samp\tTH 2 Samp\tNH4 1 Samp\t NH4 2 Samp\tNH4 3 Samp\tCa 1 Samp\tCa 2 Samp\tCond Samp\tFCl nA\tTCl nA\tT Samp\tDevice Serial\n");
	else
		UARTprintf("\tDate\tpH 1 Rinse\tpH 2 Rinse\tpH 3 Rinse\tpH 4 Rinse\tpH 5 Rinse\tpH 6 Rinse\tpH 7 Rinse\tpH 8 Rinse\tpH 9 Rinse\tpH 10 Rinse\tT Rinse\tpH 1 Samp\tpH 2 Samp\tpH 3 Samp\tpH 4 Samp\tpH 5 Samp\tpH 6 Samp\tpH 7 Samp\tpH 8 Samp\tpH 9 Samp\tpH 10 Samp\tCond Samp\tFCl nA\tTCl nA\tT Samp\tNitrite nA\tNitrite Addition nA\tDevice Serial\n");
//	if(Die_RevD)
//		UARTprintf("\tDate\tpH 1 Rinse\tpH 2 Rinse\tpH 3 Rinse\tTH 1 Rinse\tTH 2 Rinse\tNH4 1 Rinse\t NH4 2 Rinse\tNH4 3 Rinse\tCa 1 Rinse\tCa 2 Rinse\tT Rinse\tpH 1 Samp\tpH 2 Samp\tpH 3 Samp\tTH 1 Samp\tTH 2 Samp\tNH4 1 Samp\t NH4 2 Samp\tNH4 3 Samp\tCa 1 Samp\tCa 2 Samp\tCond Samp\tFCl nA\tTCl nA\tT Samp\n");
////		UARTprintf("\tDate\tpH 1 Rinse\tpH 2 Rinse\tpH 3 Rinse\tCa 1 Rinse\tCa 2 Rinse\tTH 1 Rinse\tTH 2 Rinse\tNH4 1 Rinse\t NH4 2 Rinse\tNH4 3 Rinse\tT Rinse\tpH 1 Samp\tpH 2 Samp\tpH 3 Samp\tCa 1 Samp\tCa 2 Samp\tTH 1 Samp\tTH 2 Samp\tNH4 1 Samp\t NH4 2 Samp\tNH4 3 Samp\tCond Samp\tFCl nA\tFCl nA 2\tTCl nA\tTCl nA 2\tT Samp\n");
//	else
//		UARTprintf("\tDate\tpH 1 Rinse\tpH 2 Rinse\tTH 1 Rinse\tTH 2 Rinse\tNH4 1 Rinse\t NH4 2 Rinse\tNH4 3 Rinse\tCa 1 Rinse\tCa 2 Rinse\tT Rinse\tpH 1 Samp\tpH 2 Samp\tTH 1 Samp\tTH 2 Samp\tNH4 1 Samp\t NH4 2 Samp\tNH4 3 Samp\tCa 1 Samp\tCa 2 Samp\tCond Samp\tFCl nA\tTCl nA\tT Samp\n");
	for(k = 1; k < (Test_Number + 1); k++)
	{
		uint16_t Test_page = ((PAGE_TEST + k * PAGES_FOR_TEST) - PAGES_FOR_TEST);
		uint8_t Test = *(MemoryRead(Test_page, OFFSET_TEST_NUMBER, 1));

		uint8_t Test_Cal_Number = *(MemoryRead(Test_page, OFFSET_TEST_CAL, 1));
		uint16_t Cal_page = (PAGE_CAL + Test_Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
		while(Cal_page > (PAGE_TEST))
			Cal_page -= (PAGE_TEST - PAGE_CAL);

		UARTprintf("%d\t", Test);
		uint8_t *Date = MemoryRead(Test_page, OFFSET_TEST_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		float pH_R[3], Ca_R[2], TH_R[2], NH4_R[3];
		pH_R[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_1_RINSE, 4));
		pH_R[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_2_RINSE, 4));
		pH_R[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_3_RINSE, 4));
		Ca_R[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_1_RINSE, 4));
		Ca_R[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_2_RINSE, 4));
		TH_R[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_1_RINSE, 4));
		TH_R[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_2_RINSE, 4));
		NH4_R[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_1_RINSE, 4));
		NH4_R[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_2_RINSE, 4));
		NH4_R[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_3_RINSE, 4));

		float pH_S[3], Ca_S[2], TH_S[2], NH4_S[3];
		pH_S[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_1_SAMP, 4));
		pH_S[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_2_SAMP, 4));
		pH_S[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_PH_3_SAMP, 4));
		Ca_S[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_1_SAMP, 4));
		Ca_S[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_CA_2_SAMP, 4));
		TH_S[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_1_SAMP, 4));
		TH_S[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_TH_2_SAMP, 4));
		NH4_S[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_1_SAMP, 4));
		NH4_S[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_2_SAMP, 4));
		NH4_S[2] = Build_float(MemoryRead(Test_page, OFFSET_RAW_NH4_3_SAMP, 4));


		// Read conductivity
		float Conductivity_TCorrected = Build_float(MemoryRead(Test_page, OFFSET_TEST_COND, 4));
		//
		// Conductivity Temperature Correction
		//
		// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
		float T_Samp = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));
		float Conductivity = Conductivity_TCorrected * (1 + COND_TCOMP_SAMP*(T_Samp - 25));
		float ConductivityReading;
		if(Conductivity < Cond_EEP_Cal_2)
		{
			float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
			float CalConductivityIntLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_INT, 4));
			ConductivityReading = (CalConductivitySlopeLow * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntLow * 1000));
		}
		else if(Conductivity < Cond_EEP_Rinse)
		{
			float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
			float CalConductivityIntMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_INT, 4));
			ConductivityReading = (CalConductivitySlopeMid * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntMid * 1000));
		}
		else
		{
			float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));
			float CalConductivityIntHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_INT, 4));
			ConductivityReading = (CalConductivitySlopeHigh * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntHigh * 1000));
		}

//		float Cond_S = Build_float(MemoryRead(Test_page, OFFSET_RAW_COND, 4));

		float Temp_R, Temp_S;

		Temp_R = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_RINSE, 4));
		Temp_S = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));

		float FCl_nA = Build_float(MemoryRead(Test_page, OFFSET_RAW_CL_FCL, 4));
		float TCl_nA = Build_float(MemoryRead(Test_page, OFFSET_RAW_CL_TCL, 4));
		float FCl_nA_2 = Build_float(MemoryRead(Test_page, 120, 4));
		float TCl_nA_2 = Build_float(MemoryRead(Test_page, 124, 4));

		float Nitrite_nA = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE_BACK_NA, 4));
		float NitriteAdd_nA = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE_NA, 4));

		UARTprintf("=%d/1000\t", (int) (pH_R[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (pH_R[1] * 1000));
		if(Die_RevD)
			UARTprintf("=%d/1000\t", (int) (pH_R[2] * 1000));
		UARTprintf("=%d/1000\t", (int) (TH_R[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (TH_R[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_R[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_R[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_R[2] * 1000));
		UARTprintf("=%d/1000\t", (int) (Ca_R[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (Ca_R[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp_R * 1000));

		UARTprintf("=%d/1000\t", (int) (pH_S[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (pH_S[1] * 1000));
		if(Die_RevD)
			UARTprintf("=%d/1000\t", (int) (pH_S[2] * 1000));
		UARTprintf("=%d/1000\t", (int) (TH_S[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (TH_S[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_S[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_S[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (NH4_S[2] * 1000));
		UARTprintf("=%d/1000\t", (int) (Ca_S[0] * 1000));
		UARTprintf("=%d/1000\t", (int) (Ca_S[1] * 1000));
		UARTprintf("=%d/1000\t", (int) (ConductivityReading * 1000));
		UARTprintf("=%d/1000\t", (int) (FCl_nA * 1000));
//		UARTprintf("=%d/1000\t", (int) (FCl_nA_2 * 1000));
		UARTprintf("=%d/1000\t", (int) (TCl_nA * 1000));
//		UARTprintf("=%d/1000\t", (int) (TCl_nA_2 * 1000));
		UARTprintf("=%d/1000", (int) (Temp_S * 1000));
		if(Sensor_Config == PH_CL_CART)
		{
			UARTprintf("\t=%d/1000\t", (int) (Nitrite_nA * 1000));
			UARTprintf("=%d/1000", (int) (NitriteAdd_nA * 1000));
		}

		uint8_t * Device_Serial;
		Device_Serial = MemoryRead(Test_page, OFFSET_TEST_DEVICE_SERIAL, 7);
		UARTprintf("\t%c%c%c-%c%c%c%c", Device_Serial[0], Device_Serial[1], Device_Serial[2], Device_Serial[3], Device_Serial[4], Device_Serial[5], Device_Serial[6]);

		UARTprintf("\n");

		userDelay(50, 0);
	}
	UARTprintf("\n");

//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
//	UARTprintf("pH of Cal 1: %d\n\n", (int) (pH_EEP_Cal_1 * 1000));

	// Track number of tests each sensor has performed in Sensor Usage characteristic
	uint16_t No_of_cals = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2));
	if(No_of_cals == 0xFFFF)
		No_of_cals = 0;

	UARTprintf("%d calibrations performed on this sensor! \n", No_of_cals);

	// Track number of tests each sensor has performed in Sensor Usage characteristic
	uint16_t No_of_tests = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_TESTS, 2));
	if(No_of_tests == 0xFFFF)
		No_of_tests = 0;

	UARTprintf("Tests performed on this sensor \t %d \n", No_of_tests);

	SetLED(GREEN_BUTTON, 0);
}
#else
//********************************************************************************
// Reads cartridge memory and prints all its data over UART
// Created: 5/8/2019
// 12/7/2020: Modifying to work with full pH Die
// 3/1/2020: Modified to print out a constant number of lines for each section, added
//		sensor temp to test results for pH temp correction
// Inputs:	PRINT_AS_MG_HARDNESS; 0 will print total hardness, 1 will print Mg hardness
// Outputs: NONE
//********************************************************************************
void MemoryDump(uint8_t Print_as_mg_hardness, uint8_t Die_RevD)
{
	SetLED(GREEN_BUTTON, 1);

	UARTprintf("Reading data from memory:\n");
	uint16_t Cal_Number = FindCalNumber();
	uint16_t Test_Number = FindTestNumber();
//			uint8_t Test_Number = 25;

	if(1)
	{
		uint8_t * Sensor_SN;
		Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_CARTRIDGE_SN, 7);
		UARTprintf("Cartridge %c%c%c-%c%c%c%c\n", *(Sensor_SN), *(Sensor_SN + 1), *(Sensor_SN + 2), *(Sensor_SN + 3), *(Sensor_SN + 4), *(Sensor_SN + 5), *(Sensor_SN + 6));

		Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_SN, 7);
		UARTprintf("Sensor %c%c%c-%c%c%c%c\n", *(Sensor_SN), *(Sensor_SN + 1), *(Sensor_SN + 2), *(Sensor_SN + 3), *(Sensor_SN + 4), *(Sensor_SN + 5), *(Sensor_SN + 6));

		Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_HYDRATION_DATE, 4);
		UARTprintf("Hydration Date %d/%d/%d%d\n", *(Sensor_SN), *(Sensor_SN + 1), *(Sensor_SN + 2), *(Sensor_SN + 3));

		Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_EXPIRATION_DATE, 4);
		UARTprintf("Expiration Date %d/%d/%d%d\n", *(Sensor_SN), *(Sensor_SN + 1), *(Sensor_SN + 2), *(Sensor_SN + 3));

		Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_DAYS, 1);
		UARTprintf("Max Days %d\n", *(Sensor_SN));

		Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_CALS, 1);
		UARTprintf("Max Cals %d\n", *(Sensor_SN));

		Sensor_SN = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_TESTS, 2);
		UARTprintf("Max Tests %d\n", *(Sensor_SN));
	}

	struct ISEConfig ISEs;
	FillISEStruct(&ISEs);
	UARTprintf("Config: %d\n", ISEs.Config);

	switch (ISEs.Config){
	case PH_CL_CART:
	{
		UARTprintf("pH Cr only die\n");
		break;
	}
	case H2_CART:
	{
		UARTprintf("Product die\n");
		break;
	}
	case PH_H2_CART:
	{
		UARTprintf("Full pH die split half H2 and half Cr\n");
		break;
	}
	case H2_ONLY:
	{
		UARTprintf("Full H2 die\n");
		break;
	}
	case CA_ONLY:
	{
		UARTprintf("Full Ca die\n");
		break;
	}
	case TH_ONLY:
	{
		UARTprintf("Full TH die\n");
		break;
	}
	case NH4_ONLY:
	{
		UARTprintf("Full NH4 die\n");
		break;
	}
	case CR_CA_SWAP:
	{
		UARTprintf("Product die with chromo and calcium swapped\n");
		break;
	}
	case ACROSS:
	{
		UARTprintf("Product die with each type across from itself, Ca, NH4, TH, Cr, H2\n");
		break;
	}
	default:
	{
		UARTprintf("Original die\n");
		break;
	}
	}

	UARTprintf("Found %d Calibrations\n", Cal_Number);
	UARTprintf("Found %d Tests\n", Test_Number);
	UARTprintf("\n");

//	// Check if these are over number that can fit in memory, if they are only read ones in memory, later ones are written at beginning over older data
//	if(Cal_Number > 45)
//		Cal_Number = 45;
//	if(Test_Number > 124)
//		Test_Number = 124;

		// Check if these are over number that can fit in memory, if they are only read ones in memory, later ones are written at beginning over older data
		if(Cal_Number > (PAGE_TEST - PAGE_CAL) / PAGES_FOR_CAL)
			Cal_Number = (PAGE_TEST - PAGE_CAL) / PAGES_FOR_CAL;
		if(Test_Number > (512 - PAGE_TEST) / PAGES_FOR_TEST)
			Test_Number = (512 - PAGE_TEST) / PAGES_FOR_TEST;

#ifdef SOLUTION_IN_STRUCT
		struct SolutionVals *Sols = FillSolutionStruct();

//		struct SolutionVals Sols;
//		FillSolutionStruct(&Sols);
#else
	//
	// Collect solution data before calculations
	//
	float pH_EEP_Rinse, Ca_EEP_Rinse, TH_EEP_Rinse, NH4_EEP_Rinse, Cond_EEP_Rinse;
	float pH_EEP_Cal_2, Ca_EEP_Cal_2, TH_EEP_Cal_2, NH4_EEP_Cal_2, Cond_EEP_Cal_2;
	float pH_EEP_Cal_1, Ca_EEP_Cal_1, TH_EEP_Cal_1, NH4_EEP_Cal_1, Cond_EEP_Cal_1;
	float pH_EEP_Clean, NH4_EEP_Clean;

	// Rinse
	pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
	Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
	TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_TH, 4));
	NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_NH4, 4));
	Cond_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND, 4));

	// Cal 2
	pH_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_PH, 4));
	Ca_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_CA, 4));
	TH_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_TH, 4));
	NH4_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_NH4, 4));
	Cond_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_COND, 4));

	// Cal 1
	pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
	Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
	TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));
	NH4_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_NH4, 4));
	Cond_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_COND, 4));

	// Clean
	pH_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
	NH4_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_NH4, 4));

	// Pull K T and IS values from memory
	float IS_RINSE, IS_CLEAN, IS_CAL_1, IS_CAL_2;
	float K_T_pH_Rinse, K_T_pH_Cal_1, K_T_pH_Cal_2, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln;
	IS_RINSE = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_RINSE, 4));
	if(IS_RINSE == IS_RINSE)
	{
		IS_CLEAN = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CLEAN, 4));
		IS_CAL_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CAL_1, 4));
		IS_CAL_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CAL_2, 4));

		K_T_pH_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_RINSE, 4));
		K_T_pH_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CAL_1, 4));
		K_T_pH_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CAL_2, 4));
		K_T_pH_Clean_Sq = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CLEAN_SQ, 4));
		K_T_pH_Clean_Ln = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CLEAN_LN, 4));
	}
	else
	{
		IS_RINSE = 0.0115;
		IS_CLEAN = 0.0187;
		IS_CAL_1 = 0.0321;
		IS_CAL_2 = 0.00335;

		K_T_pH_Rinse = -0.0129;
		K_T_pH_Cal_1 = -0.0025;
		K_T_pH_Cal_2 = -0.0243;
		K_T_pH_Clean_Sq = .00007;
		K_T_pH_Clean_Ln = -.0071;
	}

	float HCl_N = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_T1_HCL_N, 4));
#endif

	if(1)
	{
		float Cl_TCl_Slope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_SLOPE, 4));
		float Cl_TCl_Int = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_INT, 4));
		float Cl_FCl_Int = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_INT, 4));
		float Cl_FCl_Slope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_SLOPE, 4));
		float Cl_TCl_Slope_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_SLOPE_HIGH, 4));
		float Cl_TCl_Int_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_INT_HIGH, 4));
		float Cl_FCl_Int_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_INT_HIGH, 4));
		float Cl_FCl_Slope_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_SLOPE_HIGH, 4));
		float Therm = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_THERM_CORRECTION, 4));
		float Cond = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE, 4));

		float Ts[3], Rs[3];

		// Check if all data required has been gathered
		Ts[0] = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_LOW, 4));
		Ts[1] = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_MID, 4));
		Ts[2] = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_HIGH, 4));

		Rs[0] = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_RESISTANCE_LOW, 4));
		Rs[1] = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_RESISTANCE_MID, 4));
		Rs[2] = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_RESISTANCE_HIGH, 4));

		float Min_Temp = Build_float(MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MIN_CART_TEMP, 4));
		float Max_Temp = Build_float(MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MAX_CART_TEMP, 4));
		uint8_t * DateTime = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MIN_TEMP_DATE, 7);


		UARTprintf("Solution Values:\t\t\t\t\t\t\t\t\t\t\tTemp Cal:\t\t\tCl Cal:\t\t\t\tTemps Experienced:\t\t\t\tTherm Correction:\tCond Factory Slope:\n");
		UARTprintf("Solution\tpH\tpCa\tp(Ca+Mg)\tpNH4\tCond\tIS\tKT Sq\tKT Ln\tCond TComp\t\tTemperature\tResistance\t\t\tSlope\tIntercept\t\t\tTemp\tDate\n");
#ifdef SOLUTION_IN_STRUCT
		UARTprintf("Rinse\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/100000\t\t=%d/100000\t=%d/100000\t\t=%d/1000\t=%d/1000\t\tFCl Low\t=%d/1000\t=%d/1000\t\tMin\t=%d/1000\t%d/%d/%d%d %d:%d\t\t=%d/1000\t=%d/1000\n", (int) (Sols->pH_EEP_Rinse * 1000), (int) (Sols->Ca_EEP_Rinse * 1000), (int) (Sols->TH_EEP_Rinse * 1000), (int) (Sols->NH4_EEP_Rinse * 1000), (int) (Sols->Cond_EEP_Rinse * 1000), (int) (Sols->IS_RINSE * 100000), (int) (Sols->K_T_pH_Rinse * 100000), (int) (Sols->Rinse_Cond_TComp * 100000), (int) (Ts[0] * 1000), (int) (Rs[0] * 1000), (int) (Cl_FCl_Slope * 1000), (int) (Cl_FCl_Int * 1000), (int) (Min_Temp * 1000), DateTime[0], DateTime[1], DateTime[2], DateTime[3], DateTime[4], DateTime[5], (int) (Therm * 1000), (int) (Cond * 1000));
		DateTime = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MAX_TEMP_DATE, 7);
		UARTprintf("Cal 1\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/100000\t\t=%d/100000\t=%d/100000\t\t=%d/1000\t=%d/1000\t\tFCl High\t=%d/1000\t=%d/1000\t\tMax\t=%d/1000\t%d/%d/%d%d %d:%d\t\t\t=%d/1000\n", (int) (Sols->pH_EEP_Cal_1 * 1000), (int) (Sols->Ca_EEP_Cal_1 * 1000), (int) (Sols->TH_EEP_Cal_1 * 1000), (int) (Sols->NH4_EEP_Cal_1 * 1000), (int) (Sols->Cond_EEP_Cal_1 * 1000), (int) (Sols->IS_CAL_1 * 100000), (int) (Sols->K_T_pH_Cal_1 * 100000), (int) (Sols->Cal_1_Cond_TComp * 100000), (int) (Ts[1] * 1000), (int) (Rs[1] * 1000), (int) (Cl_FCl_Slope_High * 1000), (int) (Cl_FCl_Int_High * 1000), (int) (Max_Temp * 1000), DateTime[0], DateTime[1], DateTime[2], DateTime[3], DateTime[4], DateTime[5], (int) ((Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_COND_READ_LOW_POINT, 4)) * 1000000)*1000));
		UARTprintf("Cal 2\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/100000\t\t=%d/100000\t=%d/100000\t\t=%d/1000\t=%d/1000\t\tTCl Low\t=%d/1000\t=%d/1000\n", (int) (Sols->pH_EEP_Cal_2 * 1000), (int) (Sols->Ca_EEP_Cal_2 * 1000), (int) (Sols->TH_EEP_Cal_2 * 1000), (int) (Sols->NH4_EEP_Cal_2 * 1000), (int) (Sols->Cond_EEP_Cal_2 * 1000), (int) (Sols->IS_CAL_2 * 100000), (int) (Sols->K_T_pH_Cal_2 * 100000), (int) (Sols->Cal_2_Cond_TComp * 100000), (int) (Ts[2] * 1000), (int) (Rs[2] * 1000), (int) (Cl_TCl_Slope * 1000), (int) (Cl_TCl_Int * 1000));
		UARTprintf("Clean\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/100000\t=%d/100000\t=%d/100000\t=%d/100000\t\t\t\t\tTCl High\t=%d/1000\t=%d/1000\n", (int) (Sols->pH_EEP_Clean * 1000), (int) (Sols->Ca_EEP_Clean * 1000), (int) (Sols->TH_EEP_Clean * 1000), (int) (Sols->NH4_EEP_Clean * 1000), (int) (Sols->Cond_EEP_Clean * 1000), (int) (Sols->IS_CLEAN * 100000), (int) (Sols->K_T_pH_Clean_Sq * 100000), (int) (Sols->K_T_pH_Clean_Ln * 100000), (int) (Sols->Clean_Cond_TComp * 100000), (int) (Cl_TCl_Slope_High * 1000), (int) (Cl_TCl_Int_High * 1000));
		UARTprintf("T1 HCl N\t=%d/10000\n", (int) (Sols->HCl_N * 10000));

#else
		UARTprintf("Rinse\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t\t=%d/1000\t=%d/1000\t\tFCl Low\t=%d/1000\t=%d/1000\n", (int) (pH_EEP_Rinse * 1000), (int) (Ca_EEP_Rinse * 1000), (int) (TH_EEP_Rinse * 1000), (int) (NH4_EEP_Rinse * 1000), (int) (Cond_EEP_Rinse * 1000), (int) (Ts[0] * 1000), (int) (Rs[0] * 1000), (int) (Cl_FCl_Slope * 1000), (int) (Cl_FCl_Int * 1000));
		UARTprintf("Cal 1\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t\t=%d/1000\t=%d/1000\t\tFCl High\t=%d/1000\t=%d/1000\n", (int) (pH_EEP_Cal_1 * 1000), (int) (Ca_EEP_Cal_1 * 1000), (int) (TH_EEP_Cal_1 * 1000), (int) (NH4_EEP_Cal_1 * 1000), (int) (Cond_EEP_Cal_1 * 1000), (int) (Ts[1] * 1000), (int) (Rs[1] * 1000), (int) (Cl_FCl_Slope_High * 1000), (int) (Cl_FCl_Int_High * 1000));
		UARTprintf("Cal 2\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t=%d/1000\t\t=%d/1000\t=%d/1000\t\tTCl Low\t=%d/1000\t=%d/1000\n", (int) (pH_EEP_Cal_2 * 1000), (int) (Ca_EEP_Cal_2 * 1000), (int) (TH_EEP_Cal_2 * 1000), (int) (NH4_EEP_Cal_2 * 1000), (int) (Cond_EEP_Cal_2 * 1000), (int) (Ts[2] * 1000), (int) (Rs[2] * 1000), (int) (Cl_TCl_Slope * 1000), (int) (Cl_TCl_Int * 1000));
		UARTprintf("Clean\t=%d/1000\t\t\t=%d/1000\t\t\t\t\t\tTCl High\t=%d/1000\t=%d/1000\n", (int) (pH_EEP_Clean * 1000), (int) (NH4_EEP_Clean * 1000), (int) (Cl_TCl_Slope_High * 1000), (int) (Cl_TCl_Int_High * 1000));
		UARTprintf("T1 HCl N\t=%d/10000\n", (int) (HCl_N * 10000));
#endif
	}

	UARTprintf("\n");

	UARTprintf("Calibration Slopes: \n");
	UARTprintf("Cal\tDate");
	uint8_t i;
	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d", i + 1);
#ifdef TH_ITERATED_MATH
	if(ISEs.TH.size > 0)
	{
		for(i = 0; i < 2; i++)
			UARTprintf("\tMg %d", i + 1);
	}
#endif
	UARTprintf("\tCond Low\tCond Mid\tCond High");
	if(ISEs.pH_H2.size > 0)
		UARTprintf("\tChosen pH H2");
	if(ISEs.pH_Cr.size > 0)
		UARTprintf("\tChosen pH Cr");
	if(ISEs.TH.size > 0)
		UARTprintf("\tChosen TH");
	if(ISEs.NH4.size > 0)
		UARTprintf("\tChosen NH4");
	if(ISEs.Ca.size > 0)
		UARTprintf("\tChosen Ca");
	UARTprintf("\tRepump\tError");

#ifdef LINEAR_PH_CORR
	if(ISEs.Ca.size > 0)
		UARTprintf("\tCa 1 pH Slope\tCa 2 pH Slope");
	if(ISEs.TH.size > 0)
		UARTprintf("\tMg 1 pH Slope\tMg 2 pH Slope");
#endif

#ifdef NH4_PH_CORR
//	if(ISEs.NH4.size > 0)
//		UARTprintf("\tNH4 1 pH Slope\tNH4 2 pH Slope");
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d pH Slope", i + 1);
#endif

	UARTprintf("\n");
//	if(Sensor_Config != PH_CL_CART)
//		UARTprintf("\tDate\tpH 1\tpH 2\tpH 3\tTH 1\tTH 2\tNH4 1\tNH4 2\tNH4 3\tCa 1\tCa 2\tCond Low\tCond Mid\tCond High\tChosen\tRepump\tError\n");
//	else
//		UARTprintf("\tDate\tpH 1\tpH 2\tpH 3\tpH 4\tpH 5\tpH 6\tpH 7\tpH 8\tpH 9\tpH 10\tCond Low\tCond Mid\tCond High\tChosen\tRepump\tError\n");

	uint16_t k;
	for(k = 1; k < (Cal_Number + 1); k++)
	{
		uint16_t Cal_page = ((PAGE_CAL + k * PAGES_FOR_CAL) - PAGES_FOR_CAL);
		uint16_t Cal = *((uint16_t *) MemoryRead(Cal_page, OFFSET_CAL_NUMBER, 2));

		UARTprintf("%d\t", Cal);
		uint8_t *Date = MemoryRead(Cal_page, OFFSET_CAL_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		// Create array for all the slopes with pointers for every sensor type
		float ISE_EEP_Slope[10];
//		float *pH_H2_EEP_Slope = &ISE_EEP_Slope[ISEs.pH_H2.index];
//		float *pH_Cr_EEP_Slope = &ISE_EEP_Slope[ISEs.pH_Cr.index];
//		float *TH_EEP_Slope = &ISE_EEP_Slope[ISEs.TH.index];
//		float *NH4_EEP_Slope = &ISE_EEP_Slope[ISEs.NH4.index];
//		float *Ca_EEP_Slope = &ISE_EEP_Slope[ISEs.Ca.index];

		// Fill in slope array
		for(i = 0; i < 10; i++)
			ISE_EEP_Slope[i] = Build_float(MemoryRead(Cal_page, OFFSET_ISE_1_SLOPE + (i * 4), 4));
		float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
		float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
		float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));

		// Read chosen sensors from cartridge memory
		// Conditional for size <= 1 at beginning because you can't log2(0)
		// log2(size - 1) + 1; subtract the 1 because chosen sensor max is size - 1, then I get number of bits possible for chosen sensor
		// (1 << number of bits ) - 1 gives my mask for that number of bits
		uint8_t ChosenSensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
		uint8_t PS_Chosen_pH_H2 = ISEs.pH_H2.size <= 1 ? 0 : (ChosenSensors >> ISEs.pH_H2.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_H2.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_pH_Cr = ISEs.pH_Cr.size <= 1 ? 0 : (ChosenSensors >> ISEs.pH_Cr.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_Cr.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_TH = ISEs.TH.size <= 1 ? 0 : (ChosenSensors >> ISEs.TH.StorBit) & ((1 << ((uint8_t) log2(ISEs.TH.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_NH4 = ISEs.NH4.size <= 1 ? 0 : (ChosenSensors >> ISEs.NH4.StorBit) & ((1 << ((uint8_t) log2(ISEs.NH4.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_Ca = ISEs.Ca.size <= 1 ? 0 : (ChosenSensors >> ISEs.Ca.StorBit) & ((1 << ((uint8_t) log2(ISEs.Ca.size - 1) + 1)) - 1);

		uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
		uint32_t Error = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_ERROR, 4));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_EEP_Slope[i] * 1000));

#ifdef TH_ITERATED_MATH
		if(ISEs.TH.size > 0)
		{
			for(i = 0; i < 2; i++)
				UARTprintf("=%d/1000\t", (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_1_SLOPE + (i * 4), 4)) * 1000));
		}
#endif

		UARTprintf("=%d/1000\t", (int) (CalConductivitySlopeLow * 1000));
		UARTprintf("=%d/1000\t", (int) (CalConductivitySlopeMid * 1000));
		UARTprintf("=%d/1000\t", (int) (CalConductivitySlopeHigh * 1000));
		if(ISEs.pH_H2.size > 0)
			UARTprintf("%d\t", PS_Chosen_pH_H2);
		if(ISEs.pH_Cr.size > 0)
			UARTprintf("%d\t", PS_Chosen_pH_Cr);
		if(ISEs.TH.size > 0)
			UARTprintf("%d\t", PS_Chosen_TH);
		if(ISEs.NH4.size > 0)
			UARTprintf("%d\t", PS_Chosen_NH4);
		if(ISEs.Ca.size > 0)
			UARTprintf("%d\t", PS_Chosen_Ca);
		if((Cal_Status >> 19) & 1)
			UARTprintf("Rinse\t");
		else if((Cal_Status >> 20) & 1)
			UARTprintf("Cal 1\t");
		else if((Cal_Status >> 21) & 1)
			UARTprintf("Cal 2\t");
		else
			UARTprintf("None\t");
		UARTprintf("0x%x ", Error);
		PrintErrors(Error, 0, STATE_CALIBRATION);
#ifdef LINEAR_PH_CORR
		if(ISEs.Ca.size > 0)
			UARTprintf("\t=%d/1000\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K, 4)) * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_2_LOG_K, 4)) * 1000));
		if(ISEs.TH.size > 0)
			UARTprintf("\t=%d/1000\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_1_PH_SLOPE, 4)) * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_2_PH_SLOPE, 4)) * 1000));
#endif

#ifdef NH4_PH_CORR
//		if(ISEs.NH4.size > 0)
//			UARTprintf("\t=%d/1000\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_LOG_K, 4)) * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_NH4_2_LOG_K, 4)) * 1000));

		for(i = 0; i < ISEs.NH4.size; i++)
		{
			if(i < 2)	// Only have two spots specific to NH4 in memory currently...
				UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_LOG_K + ((i) * 4), 4)) * 1000));
			else if(i < 4)
				if(ISEs.Ca.size == 0)	// For disinfection cartridge I'm co-opting Ca interference memory locations
					UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + ((i - 2) * 4), 4)) * 1000));
		}


#endif
		UARTprintf("\n");

		userDelay(50, 0);
	}
	for(k = (Cal_Number + 1); k < ((PAGE_TEST - PAGE_CAL) / PAGES_FOR_CAL + 1); k++)
		UARTprintf("\n");
	UARTprintf("\n");


	UARTprintf("Calibration Raw Data: \n");
	UARTprintf("Cal\tDate");
	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d Rinse", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d Rinse", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d Rinse", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d Rinse", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d Rinse", i + 1);
	UARTprintf("\tTemp");

	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d Cal 1", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d Cal 1", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d Cal 1", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d Cal 1", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d Cal 1", i + 1);
	UARTprintf("\tTemp");

	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d Cal 2", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d Cal 2", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d Cal 2", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d Cal 2", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d Cal 2", i + 1);
	UARTprintf("\tTemp");

	if(ISEs.Config != PH_CL_CART)
	{
		for(i = 0; i < ISEs.pH_H2.size; i++)
			UARTprintf("\tpH H2 %d Clean", i + 1);
		for(i = 0; i < ISEs.pH_Cr.size; i++)
			UARTprintf("\tpH Cr %d Clean", i + 1);
		for(i = 0; i < ISEs.TH.size; i++)
			UARTprintf("\tTH %d Clean", i + 1);
		for(i = 0; i < ISEs.NH4.size; i++)
			UARTprintf("\tNH4 %d Clean", i + 1);
		for(i = 0; i < ISEs.Ca.size; i++)
			UARTprintf("\tCa %d Clean", i + 1);
	}

	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d Post", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d Post", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d Post", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d Post", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d Post", i + 1);
	UARTprintf("\tTemp");
	UARTprintf("\tCond Low\tCond Mid 1\tCond Mid 2\tCond High 1\tCond High 2");

	if(ISEs.pH_H2.size > 0 || ISEs.pH_Cr.size > 0)
		UARTprintf("\tpH Rinse\tpH Cal 1\tpH Cal 2\tpH Clean");
	if(ISEs.TH.size > 0)
		UARTprintf("\tpTH Rinse\tpTH Cal 1\tpTH Cal 2\tpTH Clean");
	if(ISEs.NH4.size > 0)
		UARTprintf("\tpNH4 Rinse\tpNH4 Cal 1\tpNH4 Cal 2\tpNH4 Clean");
	if(ISEs.Ca.size > 0)
		UARTprintf("\tpCa Rinse\tpCa Cal 1\tpCa Cal 2\tpCa Clean");
#ifdef TH_ITERATED_MATH
	if(ISEs.TH.size > 0)
		UARTprintf("\tpMg Rinse\tpMg Cal 1\tpMg Cal 2\tMg 1 Log k\tMg 2 Log k");
#endif

#ifndef MEMORY_V5
	UARTprintf("\tCC starting nA\tCC ending nA\tCC Time ms\tOR A1 SmV\tOR A2 Start mV\tOR A3 Start mV\tOR A4 Start mV\tOR A5 Start mV\tOR A1 End mV\tOR A2 End mV\tOR A3 End mV\tOR A4 End mV\tOR A5 End mV\tOR Time ms");
#endif

#ifdef PH_LOG_K
	UARTprintf("\tCa 1 pH Log K\tCa 2 pH Log K\tMg 1 pH Log K\tMg 2 pH Log K");
#endif

	UARTprintf("\tRoam SN");
	UARTprintf("\tH2 Slope Per\tpH Slope Per\tCa Slope Per\tTH Slope Per\tNH4 Slope Per\tCond Slope Per");

	UARTprintf("\tCond Rinse Mid Raw\tCond Rinse High Raw\tCond Clean Mid Raw\tCond Clean High Raw");

	UARTprintf("\n");

//	float pH_linear_mV[3], Ca_linear_mV[2], TH_linear_mV[2], NH4_linear_mV[3];
	for(k = 1; k < (Cal_Number + 1); k++)
	{
		uint16_t Cal_page = ((PAGE_CAL + k * PAGES_FOR_CAL) - PAGES_FOR_CAL);
		uint16_t Cal = *((uint16_t *) MemoryRead(Cal_page, OFFSET_CAL_NUMBER, 2));

		UARTprintf("%d\t", Cal);
		uint8_t *Date = MemoryRead(Cal_page, OFFSET_CAL_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		// Pull ISE slope and intercept information to calculate raw data
//		float pH_EEP_Slope[3];
//		float Ca_EEP_Slope[2];
//		float NH4_EEP_Slope[3], TH_EEP_Slope[2];
		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
		float ISE_Slope[10];
		float *pH_H2_EEP_Slope = &ISE_Slope[ISEs.pH_H2.index];
		float *pH_Cr_EEP_Slope = &ISE_Slope[ISEs.pH_Cr.index];
		float *TH_EEP_Slope = &ISE_Slope[ISEs.TH.index];
		float *NH4_EEP_Slope = &ISE_Slope[ISEs.NH4.index];
		float *Ca_EEP_Slope = &ISE_Slope[ISEs.Ca.index];
		for(i = 0; i < 10; i++)
			ISE_Slope[i] = Build_float(MemoryRead(Cal_page, OFFSET_ISE_1_SLOPE + (i * 4), 4));

		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
		float ISE_Int[10];
		float *pH_H2_EEP_Int = &ISE_Int[ISEs.pH_H2.index];
		float *pH_Cr_EEP_Int = &ISE_Int[ISEs.pH_Cr.index];
		float *TH_EEP_Int = &ISE_Int[ISEs.TH.index];
		float *NH4_EEP_Int = &ISE_Int[ISEs.NH4.index];
		float *Ca_EEP_Int = &ISE_Int[ISEs.Ca.index];
		for(i = 0; i < 10; i++)
			ISE_Int[i] = Build_float(MemoryRead(Cal_page, OFFSET_ISE_1_INT + (i * 4), 4));

		// Pull conductivity slopes and intercepts to calculate raw data
		float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
		float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
		float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));
		float CalConductivityIntLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_INT, 4));
		float CalConductivityIntMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_INT, 4));
		float CalConductivityIntHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_INT, 4));
//		float Cond_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND, 4));
//		float Cond_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_COND, 4));
//		float Cond_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_COND, 4));

//		float pH[3], Ca[2], TH[2], NH4[3];
		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
		float ISE_mV[10];
		float *pH_H2 = &ISE_mV[ISEs.pH_H2.index];
		float *pH_Cr = &ISE_mV[ISEs.pH_Cr.index];
		float *TH = &ISE_mV[ISEs.TH.index];
		float *NH4 = &ISE_mV[ISEs.NH4.index];
		float *Ca = &ISE_mV[ISEs.Ca.index];

		for(i = 0; i < 10; i++)
			ISE_mV[i] = Build_float(MemoryRead(Cal_page, OFFSET_CR_ISE_1_RINSE + (i * 4), 4));


		float Temp = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

		uint8_t i;
		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp * 1000));

		float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

#ifdef SOLUTION_IN_STRUCT
		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse);
		float pH_TCor_Cal_1 = Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1);
		float pH_TCor_Cal_2 = Calc_pH_TCor(Sols->pH_EEP_Cal_2, T_Cal, 25, 0, Sols->K_T_pH_Cal_2);
		float pH_TCor_Clean = Calc_pH_TCor(Sols->pH_EEP_Clean, T_Cal, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln);
#else
		float pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Rinse, T_Cal, 25, 0, K_T_pH_Rinse);
		float pH_TCor_Cal_1 = Calc_pH_TCor(pH_EEP_Cal_1, T_Cal, 25, 0, K_T_pH_Cal_1);
		float pH_TCor_Cal_2 = Calc_pH_TCor(pH_EEP_Cal_2, T_Cal, 25, 0, K_T_pH_Cal_2);
		float pH_TCor_Clean = Calc_pH_TCor(pH_EEP_Clean, T_Cal, 25, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln);
#endif
//		float pH_TCor_Rinse = pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
//		float pH_TCor_Cal_1 = pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
//		float pH_TCor_Cal_2 = pH_EEP_Cal_2 + K_T_pH_Cal_2 * (T_Cal - 25);
//		float pH_TCor_Clean = pH_EEP_Clean + (K_T_pH_Clean_Sq * (pow(T_Cal, 2) - pow(25, 2)) + K_T_pH_Clean_Ln * (T_Cal - 25));	// Temperature corrected pH for Rinse

		float pCa_Rinse, pCa_Cal_1, pCa_Cal_2, pCa_Clean;
		float pTH_Rinse, pTH_Cal_1, pTH_Cal_2, pTH_Clean;
		float pNH4_Rinse, pNH4_Cal_1, pNH4_Cal_2, pNH4_Clean;
		float log_K_Ca_Mg = Build_float(MemoryRead(Cal_page, OFFSET_CAL_LOG_K, 4));

#ifdef SOLUTION_IN_STRUCT
//		if(Sols->Ca_EEP_Cal_1 < 10)	// Values are p-values
//		{
//			pCa_Rinse = Sols->Ca_EEP_Rinse;
//			pCa_Cal_1 = Sols->Ca_EEP_Cal_1;
//			pCa_Cal_2 = Sols->Ca_EEP_Cal_2;
//
//			float Mg_100Ca_Cal_1 = -log10(pow(10, -Sols->TH_EEP_Cal_1) - pow(10, -Sols->Ca_EEP_Cal_1));
//			float Mg_100Ca_Cal_2 = -log10(pow(10, -Sols->TH_EEP_Cal_2) - pow(10, -Sols->Ca_EEP_Cal_2));
//
//			pTH_Cal_1 = -log10(pow(10, -Mg_100Ca_Cal_1) + pow(10, log_K_Ca_Mg) * pow(10, -Sols->Ca_EEP_Cal_1));
//			pTH_Cal_2 = -log10(pow(10, -Mg_100Ca_Cal_2) + pow(10, log_K_Ca_Mg) * pow(10, -Sols->Ca_EEP_Cal_2));
//
//			pNH4_Rinse = Sols->NH4_EEP_Rinse;
//			pNH4_Cal_1 = Sols->NH4_EEP_Cal_1;
//			pNH4_Clean = Sols->NH4_EEP_Clean;
//		}
//		else	// Values are concentration
//		{
			pCa_Rinse = Calc_pCa(Sols->Ca_EEP_Rinse, T_Cal, Sols->IS_RINSE);
			pCa_Cal_1 = Calc_pCa(Sols->Ca_EEP_Cal_1, T_Cal, Sols->IS_CAL_1);
			pCa_Cal_2 = Calc_pCa(Sols->Ca_EEP_Cal_2, T_Cal, Sols->IS_CAL_2);
			pCa_Clean = Calc_pCa(Sols->Ca_EEP_Clean, T_Cal, Sols->IS_CLEAN);

			pTH_Rinse = Calc_pTH(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, -5, T_Cal, Sols->IS_RINSE);
			pTH_Cal_1 = Calc_pTH(Sols->Ca_EEP_Cal_1, Sols->TH_EEP_Cal_1, -5, T_Cal, Sols->IS_CAL_1);
			pTH_Cal_2 = Calc_pTH(Sols->Ca_EEP_Cal_2, Sols->TH_EEP_Cal_2, -5, T_Cal, Sols->IS_CAL_2);
			pTH_Clean = Calc_pTH(Sols->Ca_EEP_Clean, Sols->TH_EEP_Clean, -5, T_Cal, Sols->IS_CLEAN);

			pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_Cal, Sols->IS_RINSE);
			pNH4_Cal_1 = Calc_pNH4(Sols->NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, Sols->IS_CAL_1);
			pNH4_Cal_2 = Calc_pNH4(Sols->NH4_EEP_Cal_2, pH_TCor_Cal_2, 0, T_Cal, Sols->IS_CAL_2);
			pNH4_Clean = Calc_pNH4(Sols->NH4_EEP_Clean, pH_TCor_Clean, 0, T_Cal, Sols->IS_CLEAN);
//			pCa_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE));
//			pCa_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1));
//			pCa_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2));
//
//			pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_Cal, IS_RINSE));
//			pTH_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_1 - Ca_EEP_Cal_1) / 100090 * Lambda_Mg(T_Cal, IS_CAL_1));
//			pTH_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_2 - Ca_EEP_Cal_2) / 100090 * Lambda_Mg(T_Cal, IS_CAL_2));
//
//			pNH4_Rinse = -log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_RINSE));
//			pNH4_Cal_1 = -log10(NH4_EEP_Cal_1 / 14000 * pow(10, -pH_TCor_Cal_1) / (pow(10, -pH_TCor_Cal_1) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CAL_1) + 0.010928 * Lambda_Na(T_Cal, IS_CAL_1) * pow(10, LOG_K_NA_NH4));
//			pNH4_Clean = -log10(NH4_EEP_Clean / 14000 * pow(10, -pH_TCor_Clean) / (pow(10, -pH_TCor_Clean) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CLEAN));
//		}

		//
		// Collect Cal 1 raw data
		//
		for(i = 0; i < 10; i++)
			ISE_mV[i] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CAL_1_MV + (i * 4), 4));

		if(ISE_mV[0] != ISE_mV[0])
		{
			for(i = 0; i < ISEs.pH_H2.size; i++)
				pH_H2[i] = pH_H2_EEP_Slope[i] * (Sols->pH_EEP_Cal_1 + Sols->K_T_pH_Cal_1 * (Temp - 25)) + pH_H2_EEP_Int[i];
			for(i = 0; i < ISEs.pH_Cr.size; i++)
				pH_Cr[i] = pH_Cr_EEP_Slope[i] * (Sols->pH_EEP_Cal_1 + Sols->K_T_pH_Cal_1 * (Temp - 25)) + pH_Cr_EEP_Int[i];
		}

#else
//		if(Ca_EEP_Cal_1 < 10)	// Values are p-values
//		{
//			pCa_Rinse = Ca_EEP_Rinse;
//			pCa_Cal_1 = Ca_EEP_Cal_1;
//			pCa_Cal_2 = Ca_EEP_Cal_2;
//
//			float Mg_100Ca_Cal_1 = -log10(pow(10, -TH_EEP_Cal_1) - pow(10, -Ca_EEP_Cal_1));
//			float Mg_100Ca_Cal_2 = -log10(pow(10, -TH_EEP_Cal_2) - pow(10, -Ca_EEP_Cal_2));
//
//			pTH_Cal_1 = -log10(pow(10, -Mg_100Ca_Cal_1) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Cal_1));
//			pTH_Cal_2 = -log10(pow(10, -Mg_100Ca_Cal_2) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Cal_2));
//
//			pNH4_Rinse = NH4_EEP_Rinse;
//			pNH4_Cal_1 = NH4_EEP_Cal_1;
//			pNH4_Clean = NH4_EEP_Clean;
//		}
//		else	// Values are concentration
//		{
			pCa_Rinse = Calc_pCa(Ca_EEP_Rinse, T_Cal, IS_RINSE);
			pCa_Cal_1 = Calc_pCa(Ca_EEP_Cal_1, T_Cal, IS_CAL_1);
			pCa_Cal_2 = Calc_pCa(Ca_EEP_Cal_2, T_Cal, IS_CAL_2);

			pTH_Rinse = Calc_pTH(Ca_EEP_Rinse, TH_EEP_Rinse, log_K_Ca_Mg, T_Cal, IS_RINSE);
			pTH_Cal_1 = Calc_pTH(Ca_EEP_Cal_1, TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, IS_CAL_1);
			pTH_Cal_2 = Calc_pTH(Ca_EEP_Cal_2, TH_EEP_Cal_2, log_K_Ca_Mg, T_Cal, IS_CAL_2);

			pNH4_Rinse = Calc_pNH4(NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_Cal, IS_RINSE);
			pNH4_Cal_1 = Calc_pNH4(NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, IS_CAL_1);
			pNH4_Clean = Calc_pNH4(NH4_EEP_Clean, pH_TCor_Clean, 0, T_Cal, IS_CLEAN);
//			pCa_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE));
//			pCa_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1));
//			pCa_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2));
//
//			pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_Cal, IS_RINSE));
//			pTH_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_1 - Ca_EEP_Cal_1) / 100090 * Lambda_Mg(T_Cal, IS_CAL_1));
//			pTH_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_2 - Ca_EEP_Cal_2) / 100090 * Lambda_Mg(T_Cal, IS_CAL_2));
//
//			pNH4_Rinse = -log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_RINSE));
//			pNH4_Cal_1 = -log10(NH4_EEP_Cal_1 / 14000 * pow(10, -pH_TCor_Cal_1) / (pow(10, -pH_TCor_Cal_1) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CAL_1) + 0.010928 * Lambda_Na(T_Cal, IS_CAL_1) * pow(10, LOG_K_NA_NH4));
//			pNH4_Clean = -log10(NH4_EEP_Clean / 14000 * pow(10, -pH_TCor_Clean) / (pow(10, -pH_TCor_Clean) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CLEAN));
//		}

		for(i = 0; i < 10; i++)
			ISE_mV[i] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CAL_1_MV + ((ISEs.Location[i] - 1) * 4), 4));

		if(ISE_mV[0] != ISE_mV[0])
		{
			for(i = 0; i < ISEs.pH_H2.size; i++)
				pH_H2[i] = pH_H2_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (Temp - 25)) + pH_H2_EEP_Int[i];
			for(i = 0; i < ISEs.pH_Cr.size; i++)
				pH_Cr[i] = pH_Cr_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (Temp - 25)) + pH_Cr_EEP_Int[i];
		}

#endif

		// Check if there wasn't raw data saved on memory, if not calculate raw values
		if(ISE_mV[ISEs.TH.index] != ISE_mV[ISEs.TH.index])
		{
#ifndef CALIBRATE_TH_R_2
			for(i = 0; i < ISEs.TH.size; i++)
				TH[i] = TH_EEP_Slope[i] * pTH_Cal_1 + TH_EEP_Int[i];
#else
			if(ISEs.TH.size > 0)
			{
				TH[0] = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_MV_CAL_1, 4));
				TH[1] = Build_float(MemoryRead(Cal_page, OFFSET_MG_2_MV_CAL_1, 4));
			}
#endif

			for(i = 0; i < ISEs.NH4.size; i++)
				NH4[i] = NH4_EEP_Slope[i] * pNH4_Cal_1 + NH4_EEP_Int[i];
			for(i = 0; i < ISEs.Ca.size; i++)
				Ca[i] = Ca_EEP_Slope[i] * pCa_Cal_1 + Ca_EEP_Int[i];
		}


		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));

		UARTprintf("=%d/1000\t", (int) (Temp * 1000));

		//
		// Collect Cal 2 raw data
		//
		for(i = 0; i < 10; i++)
			ISE_mV[i] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CAL_2_MV + (i * 4), 4));

		if(ISE_mV[0] != ISE_mV[0])
		{
#ifdef CALIBRATE_H2_IN_CLEAN
			for(i = 0; i < ISEs.pH_H2.size; i++)
				pH_H2[i] = pH_H2_EEP_Slope[i] * pH_TCor_Clean + pH_H2_EEP_Int[i];
#else
			for(i = 0; i < ISEs.pH_H2.size; i++)
				pH_H2[i] = 0; //pH_H2_EEP_Slope[i] * (pH_EEP_Cal_2 + K_T_pH_Cal_2 * (Temp - 25)) + pH_H2_EEP_Int[i];
#endif
			for(i = 0; i < ISEs.pH_Cr.size; i++)
				pH_Cr[i] = pH_Cr_EEP_Slope[i] * pH_TCor_Cal_2 + pH_Cr_EEP_Int[i];
			for(i = 0; i < ISEs.TH.size; i++)
				TH[i] = TH_EEP_Slope[i] * pTH_Cal_2 + TH_EEP_Int[i];
			for(i = 0; i < ISEs.NH4.size; i++)
				NH4[i] = NH4_EEP_Slope[i] * pNH4_Clean + NH4_EEP_Int[i];

#ifndef CALIBRATE_CA_1_R
			for(i = 0; i < ISEs.Ca.size; i++)
				Ca[i] = Ca_EEP_Slope[i] * pCa_Cal_2 + Ca_EEP_Int[i];
#else
			if(ISEs.TH.size > 0)
			{
				Ca[0] = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_MV_CAL_2, 4));
				if(Ca[0] == Ca[0])	// Check that there is data saved here
					Ca[1] = Build_float(MemoryRead(Cal_page, OFFSET_CA_2_MV_CAL_2, 4));
				else	// If there isn't data saved it's probably because the slope was calculated between Cal 1 and Cal 2 so can calculate Cal 2 mV based off slope
					for(i = 0; i < ISEs.Ca.size; i++)
						Ca[i] = Ca_EEP_Slope[i] * pCa_Cal_2 + Ca_EEP_Int[i];
			}
#endif
		}

//		Temp = Build_float(MemoryRead(Cal_page + 2, OFFSET_CR_TEMP_CAL_2, 4));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));

		UARTprintf("=%d/1000\t", (int) (Temp * 1000));

		//
		// Collect and print Clean mV
		//
		if(ISEs.Config != PH_CL_CART)
		{
			for(i = 0; i < 10; i++)
			{
				ISE_mV[i] = Build_float(MemoryRead(Cal_page, OFFSET_CR_CLEAN_MV + (i * 4), 4));
				UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));
			}
		}

		//
		// Collect and print postrinse mV
		//
		for(i = 0; i < 10; i++)
			ISE_mV[i] = Build_float(MemoryRead(Cal_page, OFFSET_CR_ISE_1_POST + (i * 4), 4));

//		Temp = Build_float(MemoryRead(Cal_page, OFFSET_CR_T_POSTRINSE, 4));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_mV[i] * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp * 1000));


#ifdef CURRENT_ADJUSTED_COND
		// Put the 3 calibrants used in the array twice each, order doesn't matter here because the array will be sorted from smallest to largest
		// Really only need 5 points with the highest conductivity calibrant in the array once, but to make it universal have an extra spot and the last spot will be ignored after sorting
		float CalConds[3] = {Sols->Cond_EEP_Clean*(1 + Sols->Clean_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))};
		SortArray(CalConds, 3);

		float Cond_High, Cond_Low;

		// Conductivity Low range
		Cond_Low = (CalConductivitySlopeLow * CalConds[0] / 1000000 + CalConductivityIntLow) * 1000000;
		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));

		// Conductivity Mid range
		Cond_Low = (CalConductivitySlopeMid * CalConds[0] / 1000000 + CalConductivityIntMid) * 1000000;
		Cond_High = (CalConductivitySlopeMid * CalConds[1] / 1000000 + CalConductivityIntMid) * 1000000;

		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_High * 1000));

		// Conductivity High range
		Cond_Low = (CalConductivitySlopeHigh * CalConds[1] / 1000000 + CalConductivityIntHigh) * 1000000;
		Cond_High = (CalConductivitySlopeHigh * CalConds[2] / 1000000 + CalConductivityIntHigh) * 1000000;

		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_High * 1000));

#else
		float Cond_High, Cond_Low;

		// Conductivity Low range
		if(Sols->pH_EEP_Cal_2 < 9.2)	// This is Cal 3
			Cond_Low = 1000000000 / (((Sols->Cond_EEP_Cal_1 * (1 + Sols->Cal_1_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntLow) * 1000 / CalConductivitySlopeLow);
		else
			Cond_Low = 1000000000 / (((Sols->Cond_EEP_Cal_2 * (1 + Sols->Cal_2_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntLow) * 1000 / CalConductivitySlopeLow);

		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));

		// Conductivity Mid range
		if(Sols->pH_EEP_Cal_2 < 9.2)	// This is Cal 3
			Cond_Low = 1000000000 / (((Sols->Cond_EEP_Cal_1 * (1 + Sols->Cal_1_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntMid) * 1000 / CalConductivitySlopeMid);
		else
			Cond_Low = 1000000000 / (((Sols->Cond_EEP_Cal_2 * (1 + Sols->Cal_2_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntMid) * 1000 / CalConductivitySlopeMid);

		if(ISEs.Config == PH_CL_CART)
			Cond_High = 1000000000 / (((Sols->Cond_EEP_Rinse * (1 + Sols->Clean_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntMid) * 1000 / CalConductivitySlopeMid);
		else
		{
			if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
				Cond_High = 1000000000 / (((Sols->Cond_EEP_Clean * (1 + Sols->Clean_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntMid) * 1000 / CalConductivitySlopeMid);
			else
				Cond_High = 1000000000 / (((Sols->Cond_EEP_Rinse * (1 + Sols->Rinse_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntMid) * 1000 / CalConductivitySlopeMid);
		}

		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_High * 1000));

		// Conductivity High range
		if(ISEs.Config == PH_CL_CART)
			Cond_Low = 1000000000 / (((Sols->Cond_EEP_Rinse * (1 + Sols->Clean_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntHigh) * 1000 / CalConductivitySlopeHigh);
		else
		{
			if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
				Cond_Low = 1000000000 / (((Sols->Cond_EEP_Clean * (1 + Sols->Clean_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntHigh) * 1000 / CalConductivitySlopeHigh);
			else
				Cond_Low = 1000000000 / (((Sols->Cond_EEP_Rinse * (1 + Sols->Rinse_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntHigh) * 1000 / CalConductivitySlopeHigh);
		}

		if(Sols->pH_EEP_Cal_2 < 9.2)	// This is Cal 3
			Cond_High = 1000000000 / (((Sols->Cond_EEP_Cal_2 * (1 + Sols->Cal_2_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntHigh) * 1000 / CalConductivitySlopeHigh);
		else
			Cond_High = 1000000000 / (((Sols->Cond_EEP_Cal_1 * (1 + Sols->Cal_1_Cond_TComp * (Temp - 25)) * 1000) - CalConductivityIntHigh) * 1000 / CalConductivitySlopeHigh);

		UARTprintf("=%d/1000\t", (int) (Cond_Low * 1000));
		UARTprintf("=%d/1000\t", (int) (Cond_High * 1000));
#endif


		// Print out p-values calculated for this calibration
		if(ISEs.pH_H2.size > 0 || ISEs.pH_Cr.size > 0)
		{
			if(ISEs.Config != PH_CL_CART)
				UARTprintf("=%d/1000\t", (int) (pH_TCor_Rinse * 1000));
			else
				UARTprintf("=%d/1000\t", (int) (pH_TCor_Clean * 1000));
			UARTprintf("=%d/1000\t", (int) (pH_TCor_Cal_1 * 1000));
			UARTprintf("=%d/1000\t", (int) (pH_TCor_Cal_2 * 1000));
			UARTprintf("=%d/1000\t", (int) (pH_TCor_Clean * 1000));
		}
		if(ISEs.TH.size > 0)
		{
			UARTprintf("=%d/1000\t", (int) (pTH_Rinse * 1000));
			UARTprintf("=%d/1000\t", (int) (pTH_Cal_1 * 1000));
			UARTprintf("=%d/1000\t", (int) (pTH_Cal_2 * 1000));
			UARTprintf("=%d/1000\t", (int) (pTH_Clean * 1000));
		}
		if(ISEs.NH4.size > 0)
		{
			UARTprintf("=%d/1000\t", (int) (pNH4_Rinse * 1000));
			UARTprintf("=%d/1000\t", (int) (pNH4_Cal_1 * 1000));
			UARTprintf("=%d/1000\t", (int) (pNH4_Cal_2 * 1000));
			UARTprintf("=%d/1000\t", (int) (pNH4_Clean * 1000));
		}
		if(ISEs.Ca.size > 0)
		{
			UARTprintf("=%d/1000\t", (int) (pCa_Rinse * 1000));
			UARTprintf("=%d/1000\t", (int) (pCa_Cal_1 * 1000));
			UARTprintf("=%d/1000\t", (int) (pCa_Cal_2 * 1000));
			UARTprintf("=%d/1000\t", (int) (pCa_Clean * 1000));
		}

#ifdef TH_ITERATED_MATH
		if(ISEs.TH.size > 0)
		{
#ifdef SOLUTION_IN_STRUCT
			UARTprintf("=%d/1000\t", (int) (Calc_pMg(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, T_Cal, Sols->IS_RINSE) * 1000));
			UARTprintf("=%d/1000\t", (int) (Calc_pMg(Sols->Ca_EEP_Cal_1, Sols->TH_EEP_Cal_1, T_Cal, Sols->IS_CAL_1) * 1000));
			UARTprintf("=%d/1000\t", (int) (Calc_pMg(Sols->Ca_EEP_Cal_2, Sols->TH_EEP_Cal_2, T_Cal, Sols->IS_CAL_2) * 1000));
#else
			UARTprintf("=%d/1000\t", (int) (Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_Cal, IS_RINSE) * 1000));
			UARTprintf("=%d/1000\t", (int) (Calc_pMg(Ca_EEP_Cal_1, TH_EEP_Cal_1, T_Cal, IS_CAL_1) * 1000));
			UARTprintf("=%d/1000\t", (int) (Calc_pMg(Ca_EEP_Cal_2, TH_EEP_Cal_2, T_Cal, IS_CAL_2) * 1000));
#endif
//			float Log_k_Nick[2];
//			Log_k_Nick[0] = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_LOG_K, 4));
//			Log_k_Nick[1] = Build_float(MemoryRead(Cal_page, OFFSET_MG_2_LOG_K, 4));
//			for(i = 0; i < 2; i++)
//			{
//				if(Log_k_Nick[i] != Log_k_Nick[i])
//				{
//					float TH_mV_Rinse = Build_float(MemoryRead(Cal_page, OFFSET_CR_ISE_1_RINSE + ((i + ISEs.TH.index) * 4), 4));
//					TH[i] = TH_EEP_Slope[i] * pTH_Cal_1 + TH_EEP_Int[i];
//					Log_k_Nick[i] = 0.0005 * pow((TH[i] - TH_mV_Rinse), 3) - 0.0158 * pow((TH[i] - TH_mV_Rinse), 2) + 0.2415 * (TH[i] - TH_mV_Rinse) - 1.5766;
//					MemoryWrite(Cal_page, OFFSET_MG_1_LOG_K + (i * 4), 4, (uint8_t *) &Log_k_Nick[i]);
//				}
//			}

			UARTprintf("=%d/1000\t", (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_1_LOG_K, 4)) * 1000));
			UARTprintf("=%d/1000\t", (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_2_LOG_K, 4)) * 1000));
		}
#endif

#ifndef MEMORY_V5
		// Read from memory and print out the cleaning values
		UARTprintf("%d\t", (int) Build_float(MemoryRead(Cal_page, OFFSET_CLEAN_CATH_START, 4)));
		UARTprintf("%d\t", (int) Build_float(MemoryRead(Cal_page, OFFSET_CLEAN_CATH_FINAL, 4)));
		UARTprintf("%d\t", *MemoryRead(Cal_page, OFFSET_CLEAN_CATH_TIME, 1));

		uint16_t *ArrayVoltages = (uint16_t *) MemoryRead(Cal_page, OFFSET_CLEAN_ARRAY_1_START, 20);
		for(i = 0; i < 10; i++)
			UARTprintf("%d\t", ArrayVoltages[i]);
		UARTprintf("%d\t", *MemoryRead(Cal_page, OFFSET_CLEAN_REBUILD_TIME, 1));
#endif

#ifdef PH_LOG_K
		UARTprintf("%d\t", (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K, 4)) * 1000));
		UARTprintf("%d\t", (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_2_LOG_K, 4)) * 1000));
		UARTprintf("%d\t", (int) (Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K, 4)) * 1000));
		UARTprintf("%d\t", (int) (Build_float(MemoryRead(Cal_page, OFFSET_TH_2_LOG_K, 4)) * 1000));
#endif

		uint8_t * Device_Serial;
		Device_Serial = MemoryRead(Cal_page, OFFSET_CAL_DEVICE_SERIAL, 7);
		UARTprintf("%c%c%c-%c%c%c%c", Device_Serial[0], Device_Serial[1], Device_Serial[2], Device_Serial[3], Device_Serial[4], Device_Serial[5], Device_Serial[6]);

		uint16_t * Slope_per = (uint16_t *) MemoryRead(Cal_page, OFFSET_ALK_SLOPE_PER, 12);
		UARTprintf("\t=%d/100\t=%d/100\t=%d/100\t=%d/100\t=%d/100\t=%d/100", Slope_per[0], Slope_per[1], Slope_per[2], Slope_per[3], Slope_per[4], Slope_per[5]);

		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_RINSE_MID_RAW, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_RINSE_HIGH_RAW, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_CLEAN_MID_RAW, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Cal_page, OFFSET_CLEAN_HIGH_RAW, 4)) * 1000));

		UARTprintf("\n");
		userDelay(50, 0);
	}
	for(k = (Cal_Number + 1); k < ((PAGE_TEST - PAGE_CAL) / PAGES_FOR_CAL + 1); k++)
		UARTprintf("\n");
	UARTprintf("\n");

	UARTprintf("Test Results:\n");
	UARTprintf("Test\tDate");
	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d", i + 1);

#ifdef TH_ITERATED_MATH
	if(ISEs.TH.size > 0)
	{
		for(i = 0; i < 2; i++)
			UARTprintf("\tTH (Nick's Math) %d", i + 1);
	}
#endif

	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d T Corrected", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d T Corrected", i + 1);

	UARTprintf("\tFCl\tTCl");

	if(ISEs.RunAlk)
	{
		for(i = 0; i < ISEs.pH_H2.size; i++)
			UARTprintf("\tAlk (pH H2 %d)\tMethod\tSlope", i + 1);
		for(i = 0; i < ISEs.pH_Cr.size; i++)
			UARTprintf("\tAlk (pH Cr %d)\tMethod\tSlope", i + 1);
	}

	UARTprintf("\tConductivity\tORP\tTherm Temp\tSensor Temp");

	if(ISEs.pH_Cr.size > 0)
		UARTprintf("\tCal Chosen pH Cr");
	if(ISEs.TH.size > 0)
		UARTprintf("\tCal Chosen TH");
	if(ISEs.NH4.size > 0)
		UARTprintf("\tCal Chosen NH4");
	if(ISEs.Ca.size > 0)
		UARTprintf("\tCal Chosen Ca");
	if(ISEs.RunAlk)
		UARTprintf("\tCal Chosen Alk");

	if(ISEs.pH_Cr.size > 0)
		UARTprintf("\tTest Chosen pH Cr");
	if(ISEs.TH.size > 0)
		UARTprintf("\tTest Chosen TH Fixed Log K");
	if(ISEs.TH.size > 0)
		UARTprintf("\tTest Chosen TH Ratio Ramp");
	if(ISEs.NH4.size > 0)
		UARTprintf("\tTest Chosen NH4");
	if(ISEs.Ca.size > 0)
		UARTprintf("\tTest Chosen Ca");
	if(ISEs.RunAlk)
		UARTprintf("\tTest Chosen Alk");
	UARTprintf("\tError");

//#ifdef TH_ITERATED_MATH
//	UARTprintf("\tNick's Log K");
//#endif

	UARTprintf("\n");

	for(k = 1; k < (Test_Number + 1); k++)
	{
		uint16_t Test_page = Find_Test_page(k);//((PAGE_TEST + k * PAGES_FOR_TEST) - PAGES_FOR_TEST);
		uint16_t Test = *((uint16_t *) MemoryRead(Test_page, OFFSET_TEST_NUMBER, 2));

		UARTprintf("%d\t", Test);
		uint8_t *Date = MemoryRead(Test_page, OFFSET_TEST_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		uint8_t Test_Cal_Number = *(MemoryRead(Test_page, OFFSET_TEST_CAL, 1));
		uint16_t Cal_page = Find_Cal_page(Test_Cal_Number);
		uint8_t i;
		uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
		uint8_t Last_cal_passed[10];
		memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
		for(i = 0; i < 10; i++)
			if(Last_cal_passed[i] == 0xFF || Last_cal_passed[i] == 0)
				Last_cal_passed[i] = Test_Cal_Number;

		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
//		float pH_EEP_Slope[3];
//		float Ca_EEP_Slope[2];
//		float NH4_EEP_Slope[3], TH_EEP_Slope[2];
		float ISE_EEP_Slope[10];
		float *pH_H2_EEP_Slope = &ISE_EEP_Slope[ISEs.pH_H2.index];
		float *pH_Cr_EEP_Slope = &ISE_EEP_Slope[ISEs.pH_Cr.index];
		float *TH_EEP_Slope = &ISE_EEP_Slope[ISEs.TH.index];
		float *NH4_EEP_Slope = &ISE_EEP_Slope[ISEs.NH4.index];
		float *Ca_EEP_Slope = &ISE_EEP_Slope[ISEs.Ca.index];
		for(i = 0; i < 10; i++)
			ISE_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i]), OFFSET_ISE_1_SLOPE + (i * 4), 4));

		// Collect temperatures
		float T_Rinse = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_RINSE, 4));
		float T_Samp = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));
		float T_RS = (T_Rinse + T_Samp) / 2;
		float T_EEP_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

		// Collect sample mV data
		// Create a single array then point each sensor type to its place in the array, this is so multiple functions don't need to be created for running normal die or all pH die
//		float pH_E_Samp[3], Ca_E_Samp[2], TH_E_Samp[2], NH4_E_Samp[3];
		float ISE_E_Samp[10];
		float *pH_H2_E_Samp = &ISE_E_Samp[ISEs.pH_H2.index];
		float *pH_Cr_E_Samp = &ISE_E_Samp[ISEs.pH_Cr.index];
		float *TH_E_Samp = &ISE_E_Samp[ISEs.TH.index];
		float *NH4_E_Samp = &ISE_E_Samp[ISEs.NH4.index];
		float *Ca_E_Samp = &ISE_E_Samp[ISEs.Ca.index];
		for(i = 0; i < 10; i++)
			ISE_E_Samp[i] = Build_float(MemoryRead(Test_page, OFFSET_RAW_ISE_1_SAMP + (i * 4), 4));

		// Collect rinse mV data
//		float pH_E_Rinse[3], Ca_E_Rinse[2], TH_E_Rinse[2], NH4_E_Rinse[3];
		float ISE_E_Rinse[10];
		float *pH_H2_E_Rinse = &ISE_E_Rinse[ISEs.pH_H2.index];
		float *pH_Cr_E_Rinse = &ISE_E_Rinse[ISEs.pH_Cr.index];
		float *TH_E_Rinse = &ISE_E_Rinse[ISEs.TH.index];
		float *NH4_E_Rinse = &ISE_E_Rinse[ISEs.NH4.index];
		float *Ca_E_Rinse = &ISE_E_Rinse[ISEs.Ca.index];

		for(i = 0; i < 10; i++)
			ISE_E_Rinse[i] = Build_float(MemoryRead(Test_page, OFFSET_RAW_ISE_1_RINSE + (i * 4), 4));

		// Read conductivity
		//
		// Conductivity Temperature Correction
		//
		// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
		float Conductivity_TCorrected = Build_float(MemoryRead(Test_page, OFFSET_TEST_COND, 4));
		float Conductivity = Conductivity_TCorrected * (1 + COND_TCOMP_SAMP*(T_Samp - 25));
		if(Conductivity_TCorrected != Conductivity_TCorrected)	// Conductivity wasn't saved, generate based off the raw data
		{
			float Conductivity_Raw = Build_float(MemoryRead(Test_page, OFFSET_RAW_COND, 4));
			uint8_t Cond_Region = *MemoryRead(Test_page, OFFSET_RAW_COND_REG, 4);

			uint16_t Cal_Number = Test_Cal_Number;

			// Write calibration data to memory so BT chip can transmit it
			uint16_t Cond_Cal_page = Find_Cal_page(Cal_Number);
		//	uint16_t Cal_page = (PAGE_CAL + Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
		//	while(Cal_page > (PAGE_TEST - PAGES_FOR_CAL))
		//		Cal_page -= (PAGE_TEST - PAGE_CAL);

			float CalConductivitySlopeLow = Build_float(MemoryRead(Cond_Cal_page, OFFSET_COND_R1_SLOPE, 4));
			if(CalConductivitySlopeLow == 0)	// Slope would be 0 if calibration was reset partway through, for customers this would prevent tests from running but for our testing I will have it use the last calibration with data in it
			{
				while(Cal_Number > 1 && CalConductivitySlopeLow == 0)
				{
					Cal_Number--;
					Cond_Cal_page = Find_Cal_page(Cal_Number);
					CalConductivitySlopeLow = Build_float(MemoryRead(Cond_Cal_page, OFFSET_COND_R1_SLOPE, 4));
				}
			}
			float CalConductivitySlopeMid = Build_float(MemoryRead(Cond_Cal_page, OFFSET_COND_R2_SLOPE, 4));
			float CalConductivitySlopeHigh = Build_float(MemoryRead(Cond_Cal_page, OFFSET_COND_R3_SLOPE, 4));
			float CalConductivityKLow = Build_float(MemoryRead(Cond_Cal_page, OFFSET_COND_R1_INT, 4));
			float CalConductivityKMid = Build_float(MemoryRead(Cond_Cal_page, OFFSET_COND_R2_INT, 4));
			float CalConductivityKHigh = Build_float(MemoryRead(Cond_Cal_page, OFFSET_COND_R3_INT, 4));

			if(CalConductivitySlopeLow < 1)
			{
				if(Cond_Region == 1)
				{
					float I_Low;
					EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
					if(I_Low != I_Low)
						I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B

					if(Conductivity_Raw < 1000)
						Conductivity = Conductivity_Raw - (CalConductivityKLow * 1000000) / CalConductivitySlopeLow;
					else
						Conductivity = (I_Low / Conductivity_Raw - CalConductivityKLow) * 1000000 / CalConductivitySlopeLow;
				}
				else if(Cond_Region == 2)
				{
					float I_Mid;
					EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
					if(I_Mid != I_Mid)
						I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B

					if(Conductivity_Raw < 1000)
						Conductivity = Conductivity_Raw - (CalConductivityKMid * 1000000) / CalConductivitySlopeMid;
					else
						Conductivity = (I_Mid / Conductivity_Raw - CalConductivityKMid) * 1000000 / CalConductivitySlopeMid;
				}
				else
				{
					float I_High;
					EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);
					if(I_High != I_High)
						I_High = 43.57 * .812;	// Average from circuits before ARV1_0B

					if(Conductivity_Raw < 1000)
						Conductivity = Conductivity_Raw - (CalConductivityKHigh * 1000000) / CalConductivitySlopeHigh;
					else
						Conductivity = (I_High / Conductivity_Raw - CalConductivityKHigh) * 1000000 / CalConductivitySlopeHigh;
				}
			}
			else
			{
				if(Cond_Region == 1)
					Conductivity = (((CalConductivitySlopeLow * 1000000000 / Conductivity_Raw) + (CalConductivityKLow * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
				else if(Cond_Region == 2)
					Conductivity = (((CalConductivitySlopeMid * 1000000000 / Conductivity_Raw) + (CalConductivityKMid * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
				else
					Conductivity = (((CalConductivitySlopeHigh * 1000000000 / Conductivity_Raw) + (CalConductivityKHigh * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
			}


			Conductivity_TCorrected = Conductivity / (1 + COND_TCOMP_SAMP*(T_Samp - 25));
		}



		uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
		uint8_t ISE_Cal_Status[10];
		for(i = 0; i < 10; i++)
			ISE_Cal_Status[i] = (Cal_Status >> (i + 1)) & 1;
//		uint8_t pH_Cal_Status[3] = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
//		uint8_t Ca_Cal_Status[2] = {(Cal_Status >> 3) & 1, (Cal_Status >> 4) & 1};
//		uint8_t TH_Cal_Status[2] = {(Cal_Status >> 5) & 1, (Cal_Status >> 6) & 1};
//		uint8_t NH4_Cal_Status[3] = {(Cal_Status >> 7) & 1, (Cal_Status >> 8) & 1, (Cal_Status >> 9) & 1};

		// Read calibration chosen sensors from cartridge memory
		// Conditional for size <= 1 at beginning because you can't log2(0)
		// log2(size - 1) + 1; subtract the 1 because chosen sensor max is size - 1, then I get number of bits possible for chosen sensor
		// (1 << number of bits ) - 1 gives my mask for that number of bits
		uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1)); // Alk, Alk, NH4, NH4, TH, Ca, pH, pH
		uint8_t PS_Chosen_Alk = ISEs.pH_H2.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.pH_H2.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_H2.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_pH_Cr = ISEs.pH_Cr.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.pH_Cr.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_Cr.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_TH = ISEs.TH.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.TH.StorBit) & ((1 << ((uint8_t) log2(ISEs.TH.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_NH4 = ISEs.NH4.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.NH4.StorBit) & ((1 << ((uint8_t) log2(ISEs.NH4.size - 1) + 1)) - 1);
		uint8_t PS_Chosen_Ca = ISEs.Ca.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.Ca.StorBit) & ((1 << ((uint8_t) log2(ISEs.Ca.size - 1) + 1)) - 1);

		float ISE_Reading[10];

		float T_Therm = Build_float(MemoryRead(Test_page, OFFSET_TEST_T_THERM, 4));
		float T_Sensor = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));

		// pH
//		float K_T_Rinse = Rinse_Table_Lookup(T_Rinse);	// Temperature coefficient of Rinse
#ifdef SOLUTION_IN_STRUCT
#ifdef CAL_2_RINSE
		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Cal_2, T_RS, 25, 0, Sols->K_T_pH_Cal_2);
#else
		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_RS, 25, 0, Sols->K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_RS - 25);	// Temperature corrected pH for Rinse
		if(ISEs.Config == PH_CL_CART)
			pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Clean, T_RS, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln); //pH_EEP_Clean + (K_T_pH_Clean_Sq * (pow(T_RS, 2) - pow(25, 2)) + K_T_pH_Clean_Ln * (T_RS - 25));	// Temperature corrected pH for Rinse

#endif
//		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_RS, 25, 0, Sols->K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_RS - 25);	// Temperature corrected pH for Rinse
//		if(ISEs.Config == PH_CL_CART)
//			pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Clean, T_RS, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln); //pH_EEP_Clean + (K_T_pH_Clean_Sq * (pow(T_RS, 2) - pow(25, 2)) + K_T_pH_Clean_Ln * (T_RS - 25));	// Temperature corrected pH for Rinse

#else
		float pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Rinse, T_RS, 25, 0, K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_RS - 25);	// Temperature corrected pH for Rinse
		if(ISEs.Config == PH_CL_CART)
			pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Clean, T_RS, 25, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln); //pH_EEP_Clean + (K_T_pH_Clean_Sq * (pow(T_RS, 2) - pow(25, 2)) + K_T_pH_Clean_Ln * (T_RS - 25));	// Temperature corrected pH for Rinse

#endif

		// Calculate un-T-corrected pH here, will recalculate for T-corrected when printing
		float *pH_H2_Samp = &ISE_Reading[ISEs.pH_H2.index];

		for(i = 0; i < ISEs.pH_H2.size; i++)
		{
			float pH_Slope_SampT = pH_H2_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
			pH_H2_Samp[i] = pH_TCor_Rinse + ((pH_H2_E_Samp[i] - pH_H2_E_Rinse[i]) / pH_Slope_SampT); // pH of sample
//			pH_Samp[i] = pH_Samp_RS[i] + K_T_pH_Samp * (T_Therm - T_RS);
		}

		// Calculate un-T-corrected pH here, will recalculate for T-corrected when printing
		float *pH_Cr_Samp = &ISE_Reading[ISEs.pH_Cr.index];

		for(i = 0; i < ISEs.pH_Cr.size; i++)
		{
			float pH_Slope_SampT = pH_Cr_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
			pH_Cr_Samp[i] = pH_TCor_Rinse + ((pH_Cr_E_Samp[i] - pH_Cr_E_Rinse[i]) / pH_Slope_SampT); // pH of sample
//			pH_Samp[i] = pH_Samp_RS[i] + K_T_pH_Samp * (T_Therm - T_RS);
		}

		uint8_t T_Chosen_pH;
		if(ISEs.Config != PH_CL_CART && ISEs.Config != PH_H2_CART)
		{
#ifdef SOLUTION_IN_STRUCT
			T_Chosen_pH = Choose_pH_Sensor(Test_Cal_Number, pH_Cr_Samp, pH_Cr_E_Rinse, T_Rinse, ISEs, Sols);
#else
			T_Chosen_pH = Choose_pH_Sensor(Test_Cal_Number, pH_Cr_Samp, pH_Cr_E_Rinse, T_Rinse, ISEs);
#endif
		}
		else
			T_Chosen_pH = Choose_pH_Sensor_pHDie(Test_Cal_Number, ISE_Reading);

		//
		// Ca Measurement
		//
		float IS;
		if(Conductivity > 62)
			IS = 0.000016 * Conductivity;
		else
			IS = 0.00001 * Conductivity;

//		float Ca_Slope_SampT[2], Ca_Samp[2];
//		float /*Ca_Hardness[2],*/ Ca_M_activity[2], Ca_M_conc[2];//, Ca_ppm[2];
		float Ca_M_activity, Ca_M_conc;
		float *Ca_Hardness = &ISE_Reading[ISEs.Ca.index];

		float pCa_Rinse;
		// Check if values in the memory are p-values or concentrations
#ifdef SOLUTION_IN_STRUCT
//		if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
//		{
//			pCa_Rinse = Sols->Ca_EEP_Rinse;
//		}
//		else	// Values are concentrations
			pCa_Rinse = Calc_pCa(Sols->Ca_EEP_Rinse, T_RS, Sols->IS_RINSE);
#else
//		if(Ca_EEP_Rinse < 10)	// Values are p-values
//		{
//			pCa_Rinse = Ca_EEP_Rinse;
//		}
//		else	// Values are concentrations
			pCa_Rinse = Calc_pCa(Ca_EEP_Rinse, T_RS, IS_RINSE);
#endif

		for(i = 0; i < ISEs.Ca.size; i++)
		{
			float Ca_Slope_RST = Ca_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope

#ifdef PH_LOG_K
			float log_K_Ca_pH = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (4 * i), 4));
			if(log_K_Ca_pH == log_K_Ca_pH)
			{
				float E_pH = (2 * Ca_Slope_RST * log10((sqrt(pow(10, -pCa_Rinse)) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(pow(10, -pCa_Rinse))));

				//			float E_pH = -0.72 * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
				//			float mV_Cal_1 = Build_float(MemoryRead(Cal_page, OFFSET_CR_CAL_1_MV + ((ISEs.Ca.index + i) * 4), 4));
				//			float mV_Rinse = Build_float(MemoryRead(Cal_page, OFFSET_CR_ISE_1_RINSE + ((ISEs.Ca.index + i) * 4), 4));
				//			float E_pH = (mV_Cal_1 - mV_Rinse) / (Sols->pH_EEP_Cal_1 - Sols->pH_EEP_Rinse) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);

				// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
				float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i] - E_pH) / Ca_Slope_RST); //pCa
				//			Ca_M_activity = pow(10, -Ca_Samp);

				float Ca_pH_activity = pow(10, -Ca_Samp) + ((1.0 - pow(10, log_K_Ca_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
				Ca_M_activity = Ca_pH_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
			}
			else
			{
				// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
				float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i]) / Ca_Slope_RST); //pCa
				Ca_M_activity = pow(10, -Ca_Samp);
			}

			float log_K_Ca_pH = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (4 * i), 4));
			if(log_K_Ca_pH == log_K_Ca_pH)
			{
				float E_pH_R = (2 * Ca_Slope_RST * log10((sqrt(pow(10, -pCa_Rinse)) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(pow(10, -pCa_Rinse))));

				// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
				float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - (Ca_E_Rinse[i] + E_pH_R)) / Ca_Slope_RST); //pCa
				Ca_M_activity = pow(10, -Ca_Samp);

				uint8_t j;
				for(j = 0; j < 10; j++)
				{
					float E_pH_S = (2 * Ca_Slope_RST * log10((sqrt(Ca_M_activity) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_Cr_Samp[T_Chosen_pH]))) / sqrt(Ca_M_activity)));
					Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] + E_pH_S - (Ca_E_Rinse[i] + E_pH_R)) / Ca_Slope_RST); //pCa
					Ca_M_activity = pow(10, -Ca_Samp);
				}

//				float Ca_pH_activity = pow(10, -Ca_Samp) + ((1.0 - pow(10, log_K_Ca_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
//				Ca_M_activity = Ca_pH_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
			}
			else
			{
				// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
				float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i]) / Ca_Slope_RST); //pCa
				Ca_M_activity = pow(10, -Ca_Samp);
			}
#elif defined LINEAR_PH_CORR
//			float E_pH = 0;
//			if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
//			{
//				float mV_Cal_1 = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_CR_CAL_1_MV + ((ISEs.Ca.index + i) * 4), 4));
//				float mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_CR_ISE_1_RINSE + ((ISEs.Ca.index + i) * 4), 4));
//				float T_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_T_CAL, 4));
//				E_pH = (mV_Cal_1 - mV_Rinse) / (Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1) - Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse)) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//			}
			float Ca_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_CA_1_LOG_K + (i * 4), 4));
			if(Ca_mpH != Ca_mpH)
				Ca_mpH = -1;
//			float E_pH = Ca_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
			float E_pH;
			if(pH_Cr_Samp[T_Chosen_pH] < 8)
				E_pH = Ca_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
			else
				E_pH = Ca_mpH * (8 - pH_TCor_Rinse);

			// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
			float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i] - E_pH) / Ca_Slope_RST); //pCa
			Ca_M_activity = pow(10, -Ca_Samp);
#else
			// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
			float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i]) / Ca_Slope_RST); //pCa
			Ca_M_activity = pow(10, -Ca_Samp);
#endif	// PH_LOG_K
//			Ca_ppm[i] = Ca_M_activity[i] / Lambda_Ca(T_RS, IS) * 40078;	// [Ca2+]ppm Ca
			Ca_M_conc = Ca_M_activity / Lambda_Ca(T_RS, IS);
			Ca_Hardness[i] = Ca_M_conc * 100086.9;	// [Ca Hardness] ppm CaCO3
		}

//		uint8_t T_Chosen_Ca = Choose_Ca_Sensor(Test_Cal_Number, Ca_Hardness, Ca_E_Rinse, ISEs);
#ifdef SOLUTION_IN_STRUCT
		uint8_t T_Chosen_Ca = Choose_Ca_Sensor(Test_Cal_Number, Ca_Hardness, Ca_E_Rinse, ISEs, Sols);
#else
		uint8_t T_Chosen_Ca = Choose_Ca_Sensor(Test_Cal_Number, Ca_Hardness, Ca_E_Rinse, ISEs);
#endif

		// Recalculate Ca M activity and Ca M conc with the chosen sensor, if the chosen sensor wasn't the last one in the loop, this will be used in TH calculation
		if((T_Chosen_Ca + 1) != ISEs.Ca.size)
		{
			Ca_M_conc = Ca_Hardness[T_Chosen_Ca] / 100086.9;
			Ca_M_activity = Ca_M_conc * Lambda_Ca(T_RS, IS);
//			float Ca_Slope_RST = Ca_EEP_Slope[T_Chosen_Ca] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
//#ifdef PH_LOG_K
//			float log_K_Ca_pH = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (4 * T_Chosen_Ca), 4));
//			if(log_K_Ca_pH == log_K_Ca_pH)
//			{
//				float E_pH_R = (2 * Ca_Slope_RST * log10((sqrt(pow(10, -pCa_Rinse)) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(pow(10, -pCa_Rinse))));
//
//				// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
//				float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - (Ca_E_Rinse[T_Chosen_Ca] + E_pH_R)) / Ca_Slope_RST); //pCa
//				Ca_M_activity = pow(10, -Ca_Samp);
//
//				uint8_t j;
//				for(j = 0; j < 10; j++)
//				{
//					float E_pH_S = (2 * Ca_Slope_RST * log10((sqrt(Ca_M_activity) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_Cr_Samp[T_Chosen_pH]))) / sqrt(Ca_M_activity)));
//					Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] + E_pH_S - (Ca_E_Rinse[T_Chosen_Ca] + E_pH_R)) / Ca_Slope_RST); //pCa
//					Ca_M_activity = pow(10, -Ca_Samp);
//				}
//
//				//				float Ca_pH_activity = pow(10, -Ca_Samp) + ((1.0 - pow(10, log_K_Ca_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
//				//				Ca_M_activity = Ca_pH_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
//			}
//			else
//			{
//				// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
//				float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - Ca_E_Rinse[T_Chosen_Ca]) / Ca_Slope_RST); //pCa
//				Ca_M_activity = pow(10, -Ca_Samp);
//			}
//#elif defined LINEAR_PH_CORR
////			float E_pH = 0;
////			if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
////			{
////				float mV_Cal_1 = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CR_CAL_1_MV + ((ISEs.Ca.index + T_Chosen_Ca) * 4), 4));
////				float mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CR_ISE_1_RINSE + ((ISEs.Ca.index + T_Chosen_Ca) * 4), 4));
////				float T_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_T_CAL, 4));
////				E_pH = (mV_Cal_1 - mV_Rinse) / (Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1) - Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse)) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
////			}
//			float Ca_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CA_1_LOG_K + (T_Chosen_Ca * 4), 4));
//			if(Ca_mpH != Ca_mpH)
//				Ca_mpH = -1;
////			float E_pH = Ca_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//			float E_pH;
//			if(pH_Cr_Samp[T_Chosen_pH] < 8)
//				E_pH = Ca_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//			else
//				E_pH = Ca_mpH * (8 - pH_TCor_Rinse);
//
//			// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
//			float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - Ca_E_Rinse[T_Chosen_Ca] - E_pH) / Ca_Slope_RST); //pCa
//			Ca_M_activity = pow(10, -Ca_Samp);
//#else
//			float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - Ca_E_Rinse[T_Chosen_Ca]) / Ca_Slope_RST); //pCa
//			Ca_M_activity = pow(10, -Ca_Samp);
//#endif	// PH_LOG_K
//			Ca_M_conc = Ca_M_activity / Lambda_Ca(T_RS, IS);
		}

		//
		// Magnesium and Total Hardness Measurement (log K -0.5)
		//
		float pTH_Rinse;
		float log_K_Ca_Mg = Build_float(MemoryRead(Test_page, OFFSET_TEST_LOG_K, 4));
		// Check if values in memory are p-values or concentration
#ifdef SOLUTION_IN_STRUCT
//		if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
//		{
//			float Mg_100Ca_Rinse = -log10(pow(10, -Sols->TH_EEP_Rinse) - pow(10, -Sols->Ca_EEP_Rinse));
//			pTH_Rinse = -log10(pow(10, -Mg_100Ca_Rinse) + pow(10, log_K_Ca_Mg) * pow(10, -Sols->Ca_EEP_Rinse));
//		}
//		else	// Values are concentration

		pTH_Rinse = Calc_pTH(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, -5, T_RS, Sols->IS_RINSE);

#else	// SOLUTION_IN_STRUCT
//		if(Ca_EEP_Rinse < 10)	// Values are p-values
//		{
//			float Mg_100Ca_Rinse = -log10(pow(10, -TH_EEP_Rinse) - pow(10, -Ca_EEP_Rinse));
//			pTH_Rinse = -log10(pow(10, -Mg_100Ca_Rinse) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Rinse));
//		}
//		else	// Values are concentration
			pTH_Rinse = Calc_pTH(Ca_EEP_Rinse, TH_EEP_Rinse, log_K_Ca_Mg, T_RS, IS_RINSE);
#endif	// SOLUTION_IN_STRUCT

		float *TH_corr = &ISE_Reading[ISEs.TH.index];
		for(i = 0; i < ISEs.TH.size; i++)
		{
#ifdef PH_LOG_K
			float log_K_TH_pH = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K + (4 * i), 4));

			if(log_K_TH_pH == log_K_TH_pH)
				pTH_Rinse = Calc_pTHpH(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, -5, T_RS, Sols->IS_RINSE, pH_Cr_Samp[T_Chosen_pH], log_K_TH_pH);

			float TH_Slope_RST = TH_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
			float Mg_perCa_Samp = pTH_Rinse + ((TH_E_Samp[i] - TH_E_Rinse[i]) / TH_Slope_RST);
			float Mg_perCa = pow(10, -Mg_perCa_Samp);	// a[Mg+X%Ca]

			float Mg_100perCa, Mg_M_activity;
			if(log_K_TH_pH == log_K_TH_pH)
			{
				Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity) + ((1.0 - pow(10, log_K_TH_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
				Mg_M_activity = Mg_100perCa - Ca_M_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
			}
			else
			{
				Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
				Mg_M_activity = Mg_100perCa - Ca_M_activity;
			}

#elif defined LINEAR_PH_CORR
			float TH_Slope_RST = TH_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope

//			float E_pH = 0;
//			if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
//			{
//				float mV_Cal_1 = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_CR_CAL_1_MV + ((ISEs.TH.index + i) * 4), 4));
//				float mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_CR_ISE_1_RINSE + ((ISEs.TH.index + i) * 4), 4));
//				float T_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_T_CAL, 4));
//				E_pH = (mV_Cal_1 - mV_Rinse) / (Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1) - Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse)) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//			}
			float TH_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_PH_SLOPE + (i * 4), 4));
			if(TH_mpH == 0 || TH_mpH != TH_mpH)
				TH_mpH = -2;

			float E_pH = TH_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);

			float Mg_perCa_Samp = pTH_Rinse + ((TH_E_Samp[i] - TH_E_Rinse[i] - E_pH) / TH_Slope_RST);
			float Mg_perCa = pow(10, -Mg_perCa_Samp);	// a[Mg+X%Ca]

			float Mg_100perCa, Mg_M_activity;

			Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
			Mg_M_activity = Mg_100perCa - Ca_M_activity;

#else
			float TH_Slope_RST = TH_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
			float Mg_perCa_Samp = pTH_Rinse + ((TH_E_Samp[i] - TH_E_Rinse[i]) / TH_Slope_RST);
			float Mg_perCa = pow(10, -Mg_perCa_Samp);	// a[Mg+X%Ca]

			float Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
			float Mg_M_activity = Mg_100perCa - Ca_M_activity;
#endif

			float Mg_M_conc = Mg_M_activity / Lambda_Mg(T_RS, IS);
			if(Mg_100perCa > Ca_M_activity)
				TH_corr[i] = (Ca_M_conc + Mg_M_conc) * 100086.9; // [TH]
			else
				TH_corr[i] = Ca_Hardness[T_Chosen_Ca];
		}

//		uint8_t T_Chosen_TH = Choose_TH_Sensor(Test_Cal_Number, TH_corr, TH_E_Rinse, ISEs);
#ifdef SOLUTION_IN_STRUCT
		uint8_t T_Chosen_TH = Choose_TH_Sensor(Test_Cal_Number, TH_corr, TH_E_Rinse, ISEs, Sols);
#else
		uint8_t T_Chosen_TH = Choose_TH_Sensor(Test_Cal_Number, TH_corr, TH_E_Rinse, ISEs);
#endif

#ifdef TH_ITERATED_MATH
#ifdef PH_LOG_K
		// Calculate assuming Mg sensor (Nick's math)
		float TH_iterated[2];
//		float log_K_Ca_Mg_Nick = -0.2;
		if(ISEs.TH.size > 0)
		{
#ifdef SOLUTION_IN_STRUCT
			float pMg_Rinse = Calc_pMg(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, T_RS, Sols->IS_RINSE);
#else
			float pMg_Rinse = Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_RS, IS_RINSE);
#endif


			// Calculate the activity of both Mg and Ca in rinse
			float a_Mg_R = pow(10, -pMg_Rinse);
			float a_Ca_R = pow(10, -pCa_Rinse);

			for(i = 0; i < 2; i++)
			{
//				float log_K_Ca_Mg_Nick = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_LOG_K + (i * 4), 4));
//				float Mg_EEP_Slope = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_SLOPE + (i * 4), 4));
				float log_K_Ca_Mg_Nick = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_LOG_K + (i * 4), 4));
				float log_K_TH_pH = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K + (4 * i), 4));
				float Mg_EEP_Slope = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_SLOPE + (i * 4), 4));
				float Mg_Slope_RST = Mg_EEP_Slope * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
//				float E_Ca_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick) * sqrt(a_Ca_R)) / sqrt(a_Mg_R)));
//				float E_pH_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_TH_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(a_Mg_R)));

				float Mg_activity_fit;
				if(log_K_TH_pH == log_K_TH_pH)
				{

					float E_Total_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick) * sqrt(a_Ca_R)) / sqrt(a_Mg_R)))
							+ (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_TH_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(a_Mg_R)));

					float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Total_R)) / Mg_Slope_RST);
					Mg_activity_fit = pow(10, -pMg_Samp);

					uint8_t j;
					for(j = 0; j < 10; j++)
					{
						//					float E_Ca = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick) * sqrt(Ca_M_activity)) / sqrt(Mg_activity_fit)));
						//					float E_pH = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_TH_pH) * sqrt(pH_Cr_Samp[T_Chosen_pH])) / sqrt(Mg_activity_fit)));
						float E_Total = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick) * sqrt(Ca_M_activity)) / sqrt(Mg_activity_fit)))
								+ (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_TH_pH) * sqrt(pow(10, -pH_Cr_Samp[T_Chosen_pH]))) / sqrt(Mg_activity_fit)));
						pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] + E_Total - (TH_E_Rinse[i] + E_Total_R)) / Mg_Slope_RST);
						Mg_activity_fit = pow(10, -pMg_Samp);
					}
				}
				else
				{
					float E_Ca_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick) * sqrt(a_Ca_R)) / sqrt(a_Mg_R)));

					float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
					Mg_activity_fit = pow(10, -pMg_Samp);

					uint8_t j;
					for(j = 0; j < 10; j++)
					{
						float E_Ca = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick) * sqrt(Ca_M_activity)) / sqrt(Mg_activity_fit)));
						pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] + E_Ca - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
						Mg_activity_fit = pow(10, -pMg_Samp);
					}
				}


				TH_iterated[i] = (Ca_M_activity / Lambda_Ca(T_RS, IS) + Mg_activity_fit / Lambda_Mg(T_RS, IS)) * 100086.9;
			}
		}
#else	// PH_LOG_K
		// Calculate assuming Mg sensor (Nick's math)
		float TH_iterated[2];
		uint8_t T_Chosen_TH_RR = 0;
		// Calculate by ramping ratio/log K until theoretical signal matches observed signal
		if(ISEs.TH.size > 0)
		{
			for(i = 0; i < 2; i++)
			{
				float TH_Slope_RST = TH_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
				float TH_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_PH_SLOPE + (i * 4), 4));
				if(TH_mpH != TH_mpH)
					TH_mpH = -2;
				float E_pH = TH_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);

				float E_Search = TH_E_Samp[i] - TH_E_Rinse[i] - E_pH;
				float E_Theory = E_Search - 1;
				float step_size = .1;
				float ratio = -step_size;
				float K = 1, Mg_Calc, a_Mg, a_Mg_xCa, pTH;
				while(E_Search > E_Theory && step_size >= .001 && K > 0)
				{
					ratio += step_size;

//					Log_K = log10(-0.7936 * ratio + 0.6671);
//					if(Log_K != Log_K)	// Negative K value
//						Log_K = -5;
					K = -0.7936 * ratio + 0.6671;
					if(K < 0)
						K = 0;

					Mg_Calc = ratio * Ca_Hardness[T_Chosen_Ca];
					a_Mg = Mg_Calc * Lambda_Mg(T_RS, IS) / 100086.9;
					a_Mg_xCa = a_Mg + K * Ca_M_activity;
					pTH = -log10(a_Mg_xCa);
					E_Theory = (pTH - pTH_Rinse) * TH_Slope_RST;

					if(E_Search <= E_Theory && step_size >= .001)
					{
						ratio -= step_size;
						step_size /= 10;
					}
				}

				if(K == 0)	// Ratio was high enough just calling K = 0
				{
					pTH = pTH_Rinse + (E_Search / TH_Slope_RST);
					a_Mg_xCa = pow(10, -pTH);
					a_Mg = a_Mg_xCa - K * Ca_M_activity;
					TH_iterated[i] = (Ca_M_activity / Lambda_Ca(T_RS, IS) + a_Mg / Lambda_Mg(T_RS, IS)) * 100086.9;
				}
				else
					TH_iterated[i] = Mg_Calc + Ca_Hardness[T_Chosen_Ca];
			}

			T_Chosen_TH_RR = Choose_TH_Sensor(Test_Cal_Number, TH_iterated, TH_E_Rinse, ISEs, Sols);
		}

//		float log_K_Ca_Mg_Nick = -0.2;
//		if(ISEs.TH.size > 0)
//		{
//#ifdef SOLUTION_IN_STRUCT
//			float pMg_Rinse = Calc_pMg(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, T_RS, Sols->IS_RINSE);
//#else
//			float pMg_Rinse = Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_RS, IS_RINSE);
//#endif
//
//			// Calculate the activity of both Mg and Ca in rinse
//			float a_Mg_R = pow(10, -pMg_Rinse);
//			float a_Ca_R = pow(10, -pCa_Rinse);
//
//			for(i = 0; i < 2; i++)
//			{
////				float log_K_Ca_Mg_Nick = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_LOG_K + (i * 4), 4));
////				float Mg_EEP_Slope = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_SLOPE + (i * 4), 4));
//				float log_K_Ca_Mg_Nick = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_LOG_K + (i * 4), 4));
//				float Mg_EEP_Slope = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_SLOPE + (i * 4), 4));
//				float Mg_Slope_RST = Mg_EEP_Slope * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
//				float E_Ca_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick) * sqrt(a_Ca_R)) / sqrt(a_Mg_R)));
//
//				float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
//				float Mg_activity_fit = pow(10, -pMg_Samp);
//
//				uint8_t j;
//				for(j = 0; j < 10; j++)
//				{
//					float E_Ca = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick) * sqrt(Ca_M_activity)) / sqrt(Mg_activity_fit)));
//					pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] + E_Ca - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
//					Mg_activity_fit = pow(10, -pMg_Samp);
//				}
//
//				TH_iterated[i] = (Ca_M_activity / Lambda_Ca(T_RS, IS) + Mg_activity_fit / Lambda_Mg(T_RS, IS)) * 100086.9;
//			}
//		}



//		// Calculate assuming Mg sensor (Nick's math)
//		float log_K_Ca_Mg_Nick = -0.2;
//		if(1)
//		{
////			float Ca_H = Ca_Hardness[T_Chosen_Ca];
////			float Ca_A = Ca_M_activity;
////			if(k == 11)
////			{
////				Ca_H = 55.939;
////				Ca_A = Ca_H / 100086.9 * Lambda_Ca(T_RS, IS);
////			}
////			else if(k == 15)
////			{
////				Ca_H = 57.937;
////				Ca_A = Ca_H / 100086.9 * Lambda_Ca(T_RS, IS);
////			}
////			else if(k == 16)
////			{
////				Ca_H = 51.944;
////				Ca_A = Ca_H / 100086.9 * Lambda_Ca(T_RS, IS);
////			}
//
//
//
//
//
//			float pMg_Rinse = Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_RS, IS_RINSE);
//
//			// Calculate the activity of both Mg and Ca in rinse
//			float a_Mg_R = pow(10, -pMg_Rinse);
//			float a_Ca_R = pow(10, -pCa_Rinse);
//			float Mg_Hardness;
//
//			float Mg_Slope_RST[2];
//			Mg_Slope_RST[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_SLOPE, 4)) * (T_RS + 273) / (T_EEP_Cal + 273);
//			Mg_Slope_RST[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_2_SLOPE, 4)) * (T_RS + 273) / (T_EEP_Cal + 273);
//
//			uint8_t iter;
//			for(iter = 0; iter < 10; iter++)
//			{
//				for(i = 0; i < ISEs.TH.size; i++)
//				{
//					float E_Ca_R = 2 * Mg_Slope_RST[i] * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick) * sqrt(a_Ca_R)) / sqrt(a_Mg_R));
//
//					float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST[i]);
//					float Mg_activity_fit = pow(10, -pMg_Samp);
//
//					uint8_t j;
//					for(j = 0; j < 10; j++)
//					{
//						float E_Ca = 2 * Mg_Slope_RST[i] * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick) * Ca_M_activity) / sqrt(Mg_activity_fit));
//						pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - E_Ca - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST[i]);
//						Mg_activity_fit = pow(10, -pMg_Samp);
//					}
//
//					TH_iterated[i] = (Ca_M_activity / Lambda_Ca(T_RS, IS) + Mg_activity_fit / Lambda_Mg(T_RS, IS)) * 100086.9;
//				}
//
//
//				T_Chosen_TH = Choose_TH_Sensor(Cal_Number, TH_iterated, TH_E_Rinse, ISEs);
//
//				Mg_Hardness = TH_iterated[T_Chosen_TH] - Ca_Hardness[T_Chosen_Ca];
//				float CaH_MgH_Ratio = Ca_Hardness[T_Chosen_Ca] / Mg_Hardness;
//
//				if(iter < 9)	// Don't recalculate on the last go through, want to save the log K in the memory
//					log_K_Ca_Mg_Nick = log10(0.1681 * CaH_MgH_Ratio + .0092);
//			}
//
////			uint8_t iter;
////			for(iter = 0; iter < 10; iter++)
////			{
////				for(i = 0; i < ISEs.TH.size; i++)
////				{
////					float E_Ca_R = 2 * Mg_Slope_RST[i] * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick) * sqrt(a_Ca_R)) / sqrt(a_Mg_R));
////
////					float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST[i]);
////					float Mg_activity_fit = pow(10, -pMg_Samp);
////
////					uint8_t j;
////					for(j = 0; j < 10; j++)
////					{
////						float E_Ca = 2 * Mg_Slope_RST[i] * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick) * Ca_A) / sqrt(Mg_activity_fit));
////						pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - E_Ca - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST[i]);
////						Mg_activity_fit = pow(10, -pMg_Samp);
////					}
////
////					TH_iterated[i] = (Ca_A / Lambda_Ca(T_RS, IS) + Mg_activity_fit / Lambda_Mg(T_RS, IS)) * 100086.9;
////				}
////
////
////				T_Chosen_TH = Choose_TH_Sensor(Cal_Number, TH_iterated, TH_E_Rinse, ISEs);
////
////				Mg_Hardness = TH_iterated[T_Chosen_TH] - Ca_H;
////				float CaH_MgH_Ratio = Ca_H / Mg_Hardness;
////
//////				Mg_Hardness = TH_iterated[T_Chosen_TH] - Ca_Hardness[T_Chosen_Ca];
//////				float CaH_MgH_Ratio = Ca_Hardness[T_Chosen_Ca] / Mg_Hardness;
////
////				if(iter < 9)	// Don't recalculate on the last go through, want to save the log K in the memory
////					log_K_Ca_Mg_Nick = log10(0.1681 * CaH_MgH_Ratio + .0092);
////			}
//		}
#endif	// PH_LOG_K

#endif

		//
		// NH4 Measurement
		//

		// NH4
		float *NH4_NH3_N_Free = &ISE_Reading[ISEs.NH4.index];

		float NH4_Alpha = pow(10, -pH_Cr_Samp[T_Chosen_pH]) / (pow(10, -pH_Cr_Samp[T_Chosen_pH]) + pow(10, -(0.09018 + 2729.92/T_RS)));
//		float NH4_Samp[3];
//		float NH4_Slope_SampT[3];//, NH4_E_Samp_TCor[3];

		float pNH4_Rinse;
#ifdef SOLUTION_IN_STRUCT
//		if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
//			pNH4_Rinse = Sols->NH4_EEP_Rinse;
//		else	// Values are concentration
			pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);
#else
//		if(Ca_EEP_Rinse < 10)	// Values are p-values
//			pNH4_Rinse = NH4_EEP_Rinse;
//		else	// Values are concentration
			pNH4_Rinse = Calc_pNH4(NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, IS_RINSE);
#endif


		for(i = 0; i < ISEs.NH4.size; i++)
		{
//			float NH4_Slope_SampT = NH4_EEP_Slope[i] * (T_Samp + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
//			float NH4_Samp = NH4_EEP_Rinse + ((NH4_E_Samp[i] - NH4_E_Rinse[i]) / NH4_Slope_SampT);	// pNH4
//			float NH4_NH3_N_Free = pow(10, -NH4_Samp) * 14000.0 / NH4_Lambda - NH4K_Interference; // Free Ammonia
//			NH4_NH3_N_Total[i] = NH4_NH3_N_Free / NH4_Alpha; // Total ammonia not including monochloramine
			float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
			float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

			float Activity_NH4_K_Na = pow(10, -NH4_Samp);

			// TODO: Enter potassium and sodium interference values
			float K_interference = 1.5; // ppm
			float Na_interference = 15; // ppm
			float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS);
			float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS);

//#ifdef PH_LOG_K
//			float log_K_NH4_pH = Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_LOG_K + (4 * i), 4));
//			float Activity_NH4;
//			if(log_K_NH4_pH == log_K_NH4_pH)
//			{
//				float Activity_Total = Activity_NH4_K_Na + ((1.0 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1.0 - pow(10, LOG_K_NA_NH4)) * Activity_Na) + ((1.0 - pow(10, log_K_NH4_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
//				Activity_NH4 = Activity_Total - Activity_K - Activity_Na - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
//			}
//			else
//			{
//				float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
//				Activity_NH4 = Activity_Total - Activity_K - Activity_Na;
//			}
//
//#else
			float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
			float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;
//#endif // PH_LOG_K

			float NH4_Ammonium = Activity_NH4 / Lambda_NH4(T_RS, IS) * 14000;
			NH4_NH3_N_Free[i] = NH4_Ammonium / NH4_Alpha;
		}

//		uint8_t T_Chosen_NH4 = Choose_NH4_Sensor(Test_Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse, ISEs);
//		uint8_t T_Chosen_NH4 = Choose_NH4_Sensor(Test_Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse);
#ifdef SOLUTION_IN_STRUCT
		uint8_t T_Chosen_NH4 = Choose_NH4_Sensor(Test_Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse, ISEs, Sols);
#else
		uint8_t T_Chosen_NH4 = Choose_NH4_Sensor(Test_Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse, ISEs);
#endif

		float FCl = Build_float(MemoryRead(Test_page, OFFSET_TEST_FREE_CL, 4));
		float TCl = Build_float(MemoryRead(Test_page, OFFSET_TEST_TOTAL_CL, 4));

		uint32_t Error = *((uint32_t *) MemoryRead(Test_page, OFFSET_TEST_ERROR, 4));

		float ORP = Build_float(MemoryRead(Test_page, OFFSET_TEST_ORP, 4));

//		MemoryWrite(Test_page, OFFSET_TEST_NITRITE_BACK_NA, 4, (uint8_t *) &Nitrite_Samp_nA);
//		MemoryWrite(Test_page, OFFSET_TEST_NITRITE_NA, 4, (uint8_t *) &Nitrite_SampNitrite_nA);
//		float Nitrite_Samp = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE, 4));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_Reading[i] * 1000));

#ifdef TH_ITERATED_MATH
		for(i = 0; i < ISEs.TH.size; i++)
			UARTprintf("=%d/1000\t", (int) (TH_iterated[i] * 1000));
#endif

		// Temperature correct the pH values then print them off
		if(T_Therm > 2 && T_Therm < 50)
		{
			for(i = 0; i < ISEs.pH_H2.size; i++)
				pH_H2_Samp[i] = Calc_pH_TCor(pH_H2_Samp[i], T_Therm, T_RS, K_T_pH_Samp_Sq, K_T_pH_Samp_Ln);//pH_H2_Samp[i] + (K_T_pH_Samp_Sq * (pow(T_Therm, 2) - pow(T_RS, 2)) + K_T_pH_Samp_Ln * (T_Therm - T_RS));//+ K_T_pH_Samp * (T_Therm - T_RS);
			for(i = 0; i < ISEs.pH_Cr.size; i++)
				pH_Cr_Samp[i] = Calc_pH_TCor(pH_Cr_Samp[i], T_Therm, T_RS, K_T_pH_Samp_Sq, K_T_pH_Samp_Ln);//pH_Cr_Samp[i] + (K_T_pH_Samp_Sq * (pow(T_Therm, 2) - pow(T_RS, 2)) + K_T_pH_Samp_Ln * (T_Therm - T_RS));//+ K_T_pH_Samp * (T_Therm - T_RS);
		}

		for(i = 0; i < ISEs.pH_H2.size; i++)
			UARTprintf("=%d/1000\t", (int) (pH_H2_Samp[i] * 1000));
		for(i = 0; i < ISEs.pH_Cr.size; i++)
			UARTprintf("=%d/1000\t", (int) (pH_Cr_Samp[i] * 1000));

		// Check that FCl exists in memory... This is in case a Cl error occurred and I didn't save the result so the app wouldn't report it
		if(FCl != FCl)
		{
			float Cl_FCl_Int = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_INT, 4));
			float Cl_FCl_Slope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_SLOPE, 4));
			float Cl_FCl_Int_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_INT_HIGH, 4));
			float Cl_FCl_Slope_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_SLOPE_HIGH, 4));
			float Cl_FCl_Midpoint = Cl_FCl_Slope * ((Cl_FCl_Int_High - Cl_FCl_Int) / (Cl_FCl_Slope - Cl_FCl_Slope_High)) + Cl_FCl_Int;

			float FCl_nA = Build_float(MemoryRead(Test_page, OFFSET_RAW_CL_FCL, 4));
			if(FCl_nA > Cl_FCl_Midpoint)
				FCl = ((FCl_nA - Cl_FCl_Int) / Cl_FCl_Slope);//*((Steps_Sample_B1 + (float) Steps_B1) / Steps_Sample_B1); // ppm Cl2
			else
				FCl = ((FCl_nA - Cl_FCl_Int_High) / Cl_FCl_Slope_High);//*((Steps_Sample_B1 + (float) Steps_B1) / Steps_Sample_B1); // ppm Cl2

		}

		// Check that TCl exists in memory... This is in case a Cl error occurred and I didn't save the result so the app wouldn't report it
		if(TCl != TCl)
		{
			float Cl_TCl_Slope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_SLOPE, 4));
			float Cl_TCl_Int = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_INT, 4));
			float Cl_TCl_Slope_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_SLOPE_HIGH, 4));
			float Cl_TCl_Int_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_INT_HIGH, 4));
			float Cl_TCl_Midpoint = Cl_TCl_Slope * ((Cl_TCl_Int_High - Cl_TCl_Int) / (Cl_TCl_Slope - Cl_TCl_Slope_High)) + Cl_TCl_Int;

			float TCl_nA = Build_float(MemoryRead(Test_page, OFFSET_RAW_CL_TCL, 4));

			if(TCl_nA > Cl_TCl_Midpoint)
				TCl = ((TCl_nA - Cl_TCl_Int) / Cl_TCl_Slope);//*((Steps_Sample_B2 + (float) Steps_B2 + (float) Steps_C2) / Steps_Sample_B2); // ppm Cl2
			else
				TCl = ((TCl_nA - Cl_TCl_Int_High) / Cl_TCl_Slope_High);//*((Steps_Sample_B2 + (float) Steps_B2 + (float) Steps_C2) / Steps_Sample_B2); // ppm Cl2
		}

		UARTprintf("=%d/1000\t", (int) (FCl * 1000));
		UARTprintf("=%d/1000\t", (int) (TCl * 1000));

		// Alkalinity
		uint8_t T_Chosen_Alk;
		if(ISEs.RunAlk)	// Check that it is not the pH Cl cartridge, will have to adjust this when adding other configurations
		{
			float Alk_Samp[10] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
			float Alk_Slope[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			uint8_t method[10] = {0,0,0,0,0,0,0,0,0,0};
			float pH_Samp_T1[2] = {0,0};	// Collect 2 pH readings for each sensor {pH 1 Mix 1, pH 2 Mix 1, pH 3 Mix 1, pH 1 Mix 2, pH 2 Mix 2, pH 3 Mix 2};

			uint8_t Alk_index = 0;
			for(Alk_index = 0; Alk_index < (ISEs.pH_H2.size + ISEs.pH_Cr.size); Alk_index++)
			{
				if(Alk_index == 0)
				{
					pH_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_1, 4));
					pH_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_2, 4));
				}
				if(Alk_index == 1)
				{
					pH_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_1, 4));
					pH_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_2, 4));
				}
				if(Alk_index == 2)
				{
					pH_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_1, 4));
					pH_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_2, 4));
				}
				if(Alk_index > 2)
				{
					pH_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_ISE_4_T1_1 + ((Alk_index - 3) * 4), 4));
					pH_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_ISE_4_T1_2 + ((Alk_index - 3) * 4), 4));
				}

				if(pH_Samp_T1[0] == pH_Samp_T1[0])	// Check that I have a pH saved in the memory, if I don't alkalinity wasn't ran
				{
					//				// Subtract Steps T1 dead from here because these variables are only used in calculations
					//				uint16_t Steps_T1[2];	// First mixing, Second mixing
					//				Steps_T1[0] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_1, 2)) - Steps_T1_dead;
					//				Steps_T1[1] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_2, 2)) - Steps_T1_dead;

					// Subtract Steps T1 dead from here because these variables are only used in calculations
					uint16_t Steps_T1_mem[2];	// First mixing, Second mixing
					Steps_T1_mem[0] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_1, 2));
					Steps_T1_mem[1] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_2, 2));
					float Steps_T1_dead = 30;
					float Steps_Sample = 7000;	// First mixing, Second mixing
					float Pump_Ratio = 0.61;	// TODO: Adjust pump ratio for alkalinity here
					float Steps_Samp_Endpoint = Steps_Sample * Pump_Ratio;

					if(Steps_T1_mem[0] > 1000)	// This is actually the volume of T1 in nL rather than steps, so convert other steps values to volumes
					{
						Steps_T1_dead = 30 * 16.8 / 610 * 1000;	// nL
						Steps_Samp_Endpoint = 7 * 16.8 * 1000;	// nL
					}

					// Once we've figured out the
					float Steps_T1[2];
					Steps_T1[0] = Steps_T1_mem[0] - Steps_T1_dead;
					Steps_T1[1] = Steps_T1_mem[1] - Steps_T1_dead;

					//				float T_Samp_T1[2];
					//				T_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_SAMP_T1_1, 4));
					//				T_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_SAMP_T1_2, 4));

					float Steps_T1_Endpoint = 0;

					// First check if we found the endpoint on either mix, then pick the mix that was closest to 4.5
					if(((pH_Samp_T1[0] >= 4.5 && pH_Samp_T1[0] <= 4.9) || (pH_Samp_T1[1] >= 4.5 && pH_Samp_T1[1] <= 4.9)) && ISE_Cal_Status[Alk_index + ISEs.pH_H2.index])
					{
						if(abs_val(pH_Samp_T1[0] - 4.5) < abs_val(pH_Samp_T1[1] - 4.5))
							Steps_T1_Endpoint = Steps_T1[0];
						else
							Steps_T1_Endpoint = Steps_T1[1];
						method[Alk_index] = 2;
					}

					if(Steps_T1_Endpoint == 0 && ISE_Cal_Status[Alk_index + ISEs.pH_H2.index])	// Calculate endpoint if pH sensor passed calibration and hasn't already found endpoint
					{
						if(pH_Samp_T1[0] < 4.5 && pH_Samp_T1[1] < 4.5 && pH_Samp_T1[1] != 0)	// Both pH points landed below 4.5 endpoint, and there is saved data
						{
							Alk_Slope[Alk_index] = ((((float) Steps_Samp_Endpoint + (float) Steps_T1[1]) * pow(10, -pH_Samp_T1[1])) - (((float) Steps_Samp_Endpoint + (float) Steps_T1[0]) * pow(10, -pH_Samp_T1[0]))) / ((float) Steps_T1[1] - (float) Steps_T1[0]);
							float B = (((float) Steps_Samp_Endpoint + (float) Steps_T1[0]) * pow(10, -pH_Samp_T1[0])) - (Alk_Slope[Alk_index] * (float) Steps_T1[0]);
							Steps_T1_Endpoint = -B/Alk_Slope[Alk_index];
							method[Alk_index] = 3;
						}
					}

					float Alk_Saved = Build_float(MemoryRead(Test_page, OFFSET_TEST_ALKALINITY, 4));
					if(Steps_T1_Endpoint == 0 && Alk_Saved != -1 && pH_Samp_T1[0] <= 4.9)	// First two methods didn't find endpoint but during test did find one, meaning calculated based on steps being close enough
					{
						float Volume_Sample = 16.8 * 7 * 1000;	// First mixing, Second mixing
						float Volume_Temp = (((float) (Steps_T1[0]) * (Sols->HCl_N - pow(10, -pH_Samp_T1[0])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_Samp_T1[0]))) / (Sols->HCl_N - pow(10, -4.5)));
						Steps_T1_Endpoint = Volume_Temp;

						//					Alk_Samp[Alk_index] = Alk_Saved;
						method[Alk_index] = 5;
					}

					if(Steps_T1_Endpoint != 0)
					{
#ifdef SOLUTION_IN_STRUCT
						Alk_Samp[Alk_index] = 50044.0 * Sols->HCl_N * Steps_T1_Endpoint / Steps_Samp_Endpoint;
#else
						Alk_Samp[Alk_index] = 50044.0 * HCl_N * Steps_T1_Endpoint / Steps_Samp_Endpoint;
#endif
					}

					if(ISEs.pH_H2.size > 0)
					{
						//					T_Chosen_Alk = Choose_Alk_Sensor(Test_Cal_Number, Alk_Samp, pH_H2_E_Rinse, T_Rinse, method, Alk_Slope, ISEs);
#ifdef SOLUTION_IN_STRUCT
						T_Chosen_Alk = Choose_Alk_Sensor(Test_Cal_Number, Alk_Samp, pH_H2_E_Rinse, T_Rinse, method, Alk_Slope, ISEs, Sols);
#else
						T_Chosen_Alk = Choose_Alk_Sensor(Test_Cal_Number, Alk_Samp, pH_H2_E_Rinse, T_Rinse, method, Alk_Slope, ISEs);
#endif
					}
					else
					{
						//					T_Chosen_Alk = Choose_Alk_Sensor(Test_Cal_Number, Alk_Samp, pH_Cr_E_Rinse, T_Rinse, method, Alk_Slope, ISEs);
#ifdef SOLUTION_IN_STRUCT
						T_Chosen_Alk = Choose_Alk_Sensor(Test_Cal_Number, Alk_Samp, pH_Cr_E_Rinse, T_Rinse, method, Alk_Slope, ISEs, Sols);
#else
						T_Chosen_Alk = Choose_Alk_Sensor(Test_Cal_Number, Alk_Samp, pH_Cr_E_Rinse, T_Rinse, method, Alk_Slope, ISEs);
#endif
					}

					UARTprintf("=%d/1000\t", (int) (Alk_Samp[Alk_index] * 1000));
					if(method[Alk_index] == 2)
						UARTprintf("Endpoint\t");
					else if(method[Alk_index] == 3)
					{
						if(Steps_T1[0] < Steps_T1[1])
							UARTprintf("Add T1 Overshoot\t");
						else
							UARTprintf("Sub T1 Overshoot\t");
					}
					else if(method[Alk_index] == 5)
						UARTprintf("Assumed Slope\t");
					else
						UARTprintf("Failed\t");
					UARTprintf("=%d/1000\t", (int) (Alk_Slope[Alk_index] * 1000));
				}
				else
				{
					UARTprintf("#N/A\t#N/A\t#N/A\t");
				}
			}
		}

		UARTprintf("=%d/1000\t", (int) (Conductivity_TCorrected * 1000));
		UARTprintf("=%d/1000\t", (int) (ORP * 1000));
		UARTprintf("=%d/1000\t", (int) (T_Therm * 1000));
		UARTprintf("=%d/1000\t", (int) (T_Sensor * 1000));

		if(ISEs.pH_Cr.size > 0)
			UARTprintf("%d\t", PS_Chosen_pH_Cr);
		if(ISEs.TH.size > 0)
			UARTprintf("%d\t", PS_Chosen_TH);
		if(ISEs.NH4.size > 0)
			UARTprintf("%d\t", PS_Chosen_NH4);
		if(ISEs.Ca.size > 0)
			UARTprintf("%d\t", PS_Chosen_Ca);
		if(ISEs.RunAlk)
			UARTprintf("%d\t", PS_Chosen_Alk);

		if(ISEs.pH_Cr.size > 0)
			UARTprintf("%d\t", T_Chosen_pH);
		if(ISEs.TH.size > 0)
			UARTprintf("%d\t", T_Chosen_TH);
		if(ISEs.TH.size > 0)
			UARTprintf("%d\t", T_Chosen_TH_RR);
		if(ISEs.NH4.size > 0)
			UARTprintf("%d\t", T_Chosen_NH4);
		if(ISEs.Ca.size > 0)
			UARTprintf("%d\t", T_Chosen_Ca);
		if(ISEs.RunAlk)
			UARTprintf("%d\t", T_Chosen_Alk);


		UARTprintf("0x%x ", Error);
		PrintErrors(Error, 0, STATE_MEASUREMENT);

//#ifdef TH_ITERATED_MATH
//		UARTprintf("\t=%d/1000", (int) (log_K_Ca_Mg_Nick * 1000));
//#endif

		UARTprintf("\n");

		userDelay(50, 0);
	}
	for(k = (Test_Number + 1); k < ((512 - PAGE_TEST) / PAGES_FOR_TEST + 1); k++)
		UARTprintf("\n");
	UARTprintf("\n");

	UARTprintf("Raw Test Data: \n");
	UARTprintf("Test\tDate");
	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d Rinse", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d Rinse", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d Rinse", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d Rinse", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d Rinse", i + 1);
	UARTprintf("\tTemp");

	for(i = 0; i < ISEs.pH_H2.size; i++)
		UARTprintf("\tpH H2 %d Samp", i + 1);
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		UARTprintf("\tpH Cr %d Samp", i + 1);
	for(i = 0; i < ISEs.TH.size; i++)
		UARTprintf("\tTH %d Samp", i + 1);
	for(i = 0; i < ISEs.NH4.size; i++)
		UARTprintf("\tNH4 %d Samp", i + 1);
	for(i = 0; i < ISEs.Ca.size; i++)
		UARTprintf("\tCa %d Samp", i + 1);

	UARTprintf("\tCond Samp\tFCl nA\tTCl nA\tT Samp\tDevice Serial");

	if(ISEs.Config == PH_CL_CART)
		UARTprintf("\tNitrite Blank 1 nA (clean)\tNitrite Samp nA\tNitrite Blank 2 nA (clean)\tNitrite Samp+Add nA");

	UARTprintf("\tpH 1 B1\tpH 2 B1\tpH 3 B1\tpH 4 B1\tpH 5 B1\tpH 6 B1\tpH 7 B1\tpH 8 B1\tpH 9 B1\tpH 10 B1");
	UARTprintf("\tpH 1 B2\tpH 2 B2\tpH 3 B2\tpH 4 B2\tpH 5 B2\tpH 6 B2\tpH 7 B2\tpH 8 B2\tpH 9 B2\tpH 10 B2");

	UARTprintf("\tRep pH\tRep Sensor Temp\tRep Cond\tRep Alk\tRep Ca\tRep Mg\tRep Mg Hard\tRep TH\tRep Ca Hard\tRep Diss CO2\tRep Ammonium\tRep Free Ammonia\tRep FCl\tRep TCl\tRep Total Ammonia\tRep ORP\tRep Cl/NH4 ratio\tRep Samp Temp\tRep Nitrite\tRep MCl\tRep Nitrification Potential");

#ifndef MEMORY_V5
	UARTprintf("\tCC starting nA\tCC ending nA\tCC Time ms\tOR A1 SmV\tOR A2 Start mV\tOR A3 Start mV\tOR A4 Start mV\tOR A5 Start mV\tOR A1 End mV\tOR A2 End mV\tOR A3 End mV\tOR A4 End mV\tOR A5 End mV\tRebuild Time ms");
#endif

	UARTprintf("\tSteps T1 Mix 1\tSteps T1 Mix 2");

#ifndef MEMORY_V5
	UARTprintf("\tStart Therm Temp\tFinal Therm Temp");
#endif

#ifdef TH_ITERATED_MATH
	UARTprintf("\tMg 1 Log k\tMg 2 Log k");
#endif

	UARTprintf("\tTCor Clean Conductivity");
	UARTprintf("\tB1 Mix Conductivity");
	UARTprintf("\tB2 Mix Conductivity");
	UARTprintf("\tB1 Prime Conductivity");
	UARTprintf("\tC2 Prime Conductivity");
	UARTprintf("\tB1 Mix Final Temp");
	UARTprintf("\tB2 Mix Final Temp");
	UARTprintf("\tB1 Mix Start Temp");
	UARTprintf("\tB2 Mix Start Temp");
	UARTprintf("\tB1 Mix Therm Temp");
	UARTprintf("\tB2 Mix Therm Temp");
	UARTprintf("\tUser ID");
	UARTprintf("\tLocation ID");
	UARTprintf("\n");

	for(k = 1; k < (Test_Number + 1); k++)
	{
		uint16_t Test_page = ((PAGE_TEST + k * PAGES_FOR_TEST) - PAGES_FOR_TEST);
		uint8_t Test = *(MemoryRead(Test_page, OFFSET_TEST_NUMBER, 1));

		uint8_t Test_Cal_Number = *(MemoryRead(Test_page, OFFSET_TEST_CAL, 1));
		uint16_t Cal_page = (PAGE_CAL + Test_Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
		while(Cal_page > (PAGE_TEST))
			Cal_page -= (PAGE_TEST - PAGE_CAL);

		UARTprintf("%d\t", Test);
		uint8_t *Date = MemoryRead(Test_page, OFFSET_TEST_DATE, 7);
		UARTprintf("%d/%d/%d%d %d:%d:%d\t", *(Date + 0), *(Date + 1), *(Date + 2), *(Date + 3), *(Date + 4), *(Date + 5), *(Date + 6));

		float ISE_Rinse[10], ISE_Samp[10];
		for(i = 0; i < 10; i++)
		{
			ISE_Rinse[i] = Build_float(MemoryRead(Test_page, OFFSET_RAW_ISE_1_RINSE + (i * 4), 4));
			ISE_Samp[i] = Build_float(MemoryRead(Test_page, OFFSET_RAW_ISE_1_SAMP + (i * 4), 4));
		}

//		// Read conductivity
//		float Conductivity_TCorrected = Build_float(MemoryRead(Test_page, OFFSET_TEST_COND, 4));
//		//
//		// Conductivity Temperature Correction
//		//
//		// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
//		float T_Samp = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));
//		float Conductivity = Conductivity_TCorrected * (1 + COND_TCOMP_SAMP*(T_Samp - 25));
//		float ConductivityReading;
//#ifdef SOLUTION_IN_STRUCT
//		if(Conductivity < Sols->Cond_EEP_Cal_2)
//		{
//			float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
//			float CalConductivityIntLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_INT, 4));
//			ConductivityReading = (CalConductivitySlopeLow * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntLow * 1000));
//		}
//		else if(Conductivity < Sols->Cond_EEP_Rinse)
//		{
//			float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
//			float CalConductivityIntMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_INT, 4));
//			ConductivityReading = (CalConductivitySlopeMid * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntMid * 1000));
//		}
//		else
//		{
//			float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));
//			float CalConductivityIntHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_INT, 4));
//			ConductivityReading = (CalConductivitySlopeHigh * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntHigh * 1000));
//		}
//#else
//		if(Conductivity < Cond_EEP_Cal_2)
//		{
//			float CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
//			float CalConductivityIntLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_INT, 4));
//			ConductivityReading = (CalConductivitySlopeLow * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntLow * 1000));
//		}
//		else if(Conductivity < Cond_EEP_Rinse)
//		{
//			float CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
//			float CalConductivityIntMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_INT, 4));
//			ConductivityReading = (CalConductivitySlopeMid * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntMid * 1000));
//		}
//		else
//		{
//			float CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));
//			float CalConductivityIntHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_INT, 4));
//			ConductivityReading = (CalConductivitySlopeHigh * 1000000000) / ((Conductivity * 1000000) - (CalConductivityIntHigh * 1000));
//		}
//#endif

		float ConductivityReading = Build_float(MemoryRead(Test_page, OFFSET_RAW_COND, 4));

		float Temp_R, Temp_S;

		Temp_R = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_RINSE, 4));
		Temp_S = Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4));

		float FCl_nA = Build_float(MemoryRead(Test_page, OFFSET_RAW_CL_FCL, 4));
		float TCl_nA = Build_float(MemoryRead(Test_page, OFFSET_RAW_CL_TCL, 4));
//		float FCl_nA_2 = Build_float(MemoryRead(Test_page, 120, 4));
//		float TCl_nA_2 = Build_float(MemoryRead(Test_page, 124, 4));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_Rinse[i] * 1000));
		UARTprintf("=%d/1000\t", (int) (Temp_R * 1000));

		for(i = 0; i < 10; i++)
			UARTprintf("=%d/1000\t", (int) (ISE_Samp[i] * 1000));

		UARTprintf("=%d/1000\t", (int) (ConductivityReading * 1000));
		UARTprintf("=%d/1000\t", (int) (FCl_nA * 1000));
		UARTprintf("=%d/1000\t", (int) (TCl_nA * 1000));
		UARTprintf("=%d/1000", (int) (Temp_S * 1000));

		uint8_t * Device_Serial;
		Device_Serial = MemoryRead(Test_page, OFFSET_TEST_DEVICE_SERIAL, 7);
		UARTprintf("\t%c%c%c-%c%c%c%c", Device_Serial[0], Device_Serial[1], Device_Serial[2], Device_Serial[3], Device_Serial[4], Device_Serial[5], Device_Serial[6]);

		if(ISEs.Config == PH_CL_CART)
		{
			float Nitrite_Blank_1 = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE_BLANK_1_NA, 4));
			float Nitrite_Blank_2 = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE_BLANK_2_NA, 4));

			float Nitrite_Samp = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE_BACK_NA, 4));
			float Nitrite_SampAdd = Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE_NA, 4));

			UARTprintf("\t=%d/1000", (int) (Nitrite_Blank_1 * 1000));
			UARTprintf("\t=%d/1000", (int) (Nitrite_Samp * 1000));
			UARTprintf("\t=%d/1000", (int) (Nitrite_Blank_2 * 1000));
			UARTprintf("\t=%d/1000", (int) (Nitrite_SampAdd * 1000));
		}

		// Print out pH values for B1 mix, or T1 Mix 1
		if(1)
		{
			float B1_Mix[10] = {0,0,0,0,0,0,0,0,0,0};
			B1_Mix[0] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_1, 4));
			B1_Mix[1] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_1, 4));
			B1_Mix[2] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_1, 4));

			// Write the rest of the values to the memory
			for(i = 0; i < 7; i++)
			{
				B1_Mix[3 + i] = Build_float(MemoryRead(Test_page, OFFSET_ISE_4_T1_1 + (i * 4), 4));
			}

			for(i = 0; i < 10; i++)
				UARTprintf("\t=%d/1000", (int) (B1_Mix[i] * 1000));
		}

		// Print out pH values for B2 mix, or T1 Mix 2
		if(1)
		{
			float B2_Mix[10] = {0,0,0,0,0,0,0,0,0,0};
			B2_Mix[0] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_2, 4));
			B2_Mix[1] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_2, 4));
			B2_Mix[2] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_2, 4));

			// Write the rest of the values to the memory
			for(i = 0; i < 7; i++)
			{
				B2_Mix[3 + i] = Build_float(MemoryRead(Test_page, OFFSET_ISE_4_T1_2 + (i * 4), 4));
			}

			for(i = 0; i < 10; i++)
				UARTprintf("\t=%d/1000", (int) (B2_Mix[i] * 1000));
		}

		// Print out all the reported values
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_PH, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_TEMP, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_COND, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_ALKALINITY, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_CALCIUM, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_MAGNESIUM, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_MAG_HARDNESS, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_TOTAL_HARDNESS, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_CAL_HARDNESS, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_CO2, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_FREE_NH4, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_TOTAL_NH4, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_FREE_CL, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_TOTAL_CL, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_TOTAL_NH4_MONO, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_ORP, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_CL_NH4_RATIO, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_T_THERM, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_NITRITE, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_MONO, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_BFR, 4)) * 1000));

#ifndef MEMORY_V5
		// Read from memory and print out the cleaning values
		UARTprintf("\t%d", (int) Build_float(MemoryRead(Test_page, OFFSET_CLEAN_CATH_START, 4)));
		UARTprintf("\t%d", (int) Build_float(MemoryRead(Test_page, OFFSET_CLEAN_CATH_FINAL, 4)));
		UARTprintf("\t%d", *MemoryRead(Test_page, OFFSET_CLEAN_CATH_TIME, 1));

		uint16_t *ArrayVoltages = (uint16_t *) MemoryRead(Test_page, OFFSET_CLEAN_ARRAY_1_START, 20);
		for(i = 0; i < 10; i++)
			UARTprintf("\t%d", ArrayVoltages[i]);
		UARTprintf("\t%d", *MemoryRead(Test_page, OFFSET_CLEAN_REBUILD_TIME, 1));
#endif

		// Leave these steps T1 unmodified because I want to print out the steps the pump actually turned
		uint16_t Steps_T1[2];	// First mixing, Second mixing
		Steps_T1[0] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_1, 2));
		Steps_T1[1] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_2, 2));

		UARTprintf("\t%d\t%d", Steps_T1[0], Steps_T1[1]);

#ifndef MEMORY_V5
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_START_THERM, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_FINAL_THERM, 4)) * 1000));
#endif

#ifdef TH_ITERATED_MATH
		uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
		uint8_t Last_cal_passed[10];
		memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
		for(i = 0; i < 10; i++)
			if(Last_cal_passed[i] == 0xFF || Last_cal_passed[i] == 0)
				Last_cal_passed[i] = Test_Cal_Number;

		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.TH.index]), OFFSET_MG_1_LOG_K, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1 + ISEs.TH.index]), OFFSET_MG_2_LOG_K, 4)) * 1000));
#endif

		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_TEST_CLEAN_COND_TCOR, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B1_MIX_COND, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B2_MIX_COND, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B1_PRIME_COND, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_C2_PRIME_COND, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B1_MIX_TEMP, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B2_MIX_TEMP, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B1_MIX_START_TEMP, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B2_MIX_START_TEMP, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B1_THERM_TEMP, 4)) * 1000));
		UARTprintf("\t=%d/1000", (int) (Build_float(MemoryRead(Test_page, OFFSET_B2_THERM_TEMP, 4)) * 1000));

		UARTprintf("\t=%d", *((uint32_t *) MemoryRead(Test_page, OFFSET_TEST_USER_NAME, 4)));
		UARTprintf("\t=%d", *((uint32_t *) MemoryRead(Test_page, OFFSET_TEST_LOCATION, 4)));

		UARTprintf("\n");

		userDelay(200, 0);
	}
	UARTprintf("\n");

//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
//	UARTprintf("pH of Cal 1: %d\n\n", (int) (pH_EEP_Cal_1 * 1000));

	// Track number of tests each sensor has performed in Sensor Usage characteristic
	uint16_t No_of_cals = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2));
	if(No_of_cals == 0xFFFF)
		No_of_cals = 0;

	UARTprintf("%d calibrations performed on this sensor! \n", No_of_cals);

	// Track number of tests each sensor has performed in Sensor Usage characteristic
	uint16_t No_of_tests = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_TESTS, 2));
	if(No_of_tests == 0xFFFF)
		No_of_tests = 0;

	UARTprintf("Tests performed on this sensor \t %d \n", No_of_tests);

	SetLED(GREEN_BUTTON, 0);
}
#endif
#endif	// ifdef TESTING_MODE

////********************************************************************************
//// Prints out over UART what the errors are in a given error code
//// Created: 1/23/2020
////	6/3/2021: Created option to not create new lines between errors
//// Inputs:	ui32ErrorCode; the error code to print, will usually be gui32Error
//// Outputs: NONE
////********************************************************************************
//void PrintErrors(uint32_t ui32ErrorCode)
//{
//	if(ui32ErrorCode != 0)
//	{
////		UARTprintf("Errors:");
//		if((ui32ErrorCode & MAX_TESTS_REACHED) != 0)
//			UARTprintf("Error! Max tests exceeded!\n");
//		if((ui32ErrorCode & MAX_CALS_REACHED) != 0)
//			UARTprintf("Error! Max calibrations exceeded!\n");
//		if((ui32ErrorCode & CARTRIDGE_EXPIRED) != 0)
//			UARTprintf("Error! Cartridge expired, hydrated over 35 days ago!\n");
//		if((ui32ErrorCode & BATTERY_TOO_LOW) != 0)
//			UARTprintf("Error! Battery too low, valve and pump reliability questionable!\n");
//		if((ui32ErrorCode & I2C_FAILED) != 0)
//			UARTprintf("Error! I2C communication failed, I2CMasterErr() Tivaware function returned error!\n");
//		if((ui32ErrorCode & IO_EXT1_FAIL) != 0)
//			UARTprintf("Error! IO Extender 1 failed to write and read back correctly!\n");
//		if((ui32ErrorCode & IO_EXT2_FAIL) != 0)
//			UARTprintf("Error! IO Extender 2 failed to write and read back correctly!\n");
//		if((ui32ErrorCode & DAC_FAIL) != 0)
//			UARTprintf("Error! DAC failed to write and read back correctly!\n");
//		if((ui32ErrorCode & ADC1_FAIL) != 0)
//			UARTprintf("Error! ADC 1 failed to write and read back channel scan register!\n");
//		if((ui32ErrorCode & ADC2_FAIL) != 0)
//			UARTprintf("Error! ADC 2 failed to write and read back channel scan register!\n");
//		if((ui32ErrorCode & ADC3_FAIL) != 0)
//			UARTprintf("Error! Conductivity ADC never returned anything more than 0s!\n");
//		if((ui32ErrorCode & ADC4_FAIL) != 0)
//			UARTprintf("Error! ADC 4 failed to write and read back channel scan register!\n");
//		if((ui32ErrorCode & WAVE_GEN_FAIL) != 0)
//			UARTprintf("Error! Waveform generator not creating 1 kHz signal!\n");
//		if((ui32ErrorCode & BT_FAILED) != 0)
//			UARTprintf("Error! Bluetooth chip didn't respond to request, had to reset!\n");
//		if((ui32ErrorCode & ROAM_RESET) != 0)
//			UARTprintf("Error! Roam was reset partway through and didn't finish run!\n");
//		if((ui32ErrorCode & CL_CLEANING_OUT_OF_RANGE) != 0)
//			UARTprintf("Error! Amperometic cleaning either returned currents with wrong sign or rebuild voltages were more than 100 mV apart!\n");
////		if((ui32ErrorCode & FCL_MIX_OUT_OF_RANGE) != 0)
////			UARTprintf("Error! Free chlorine mixing never found correct pH range!\n");
////		if((ui32ErrorCode & TCL_MIX_OUT_OF_RANGE) != 0)
////			UARTprintf("Error! Total chlorine mixing never found correct pH range!\n");
//		if((ui32ErrorCode & CAL_FAILED_PH_H2_SLOPES) != 0)
//			UARTprintf("Error! pH hydrogen 2 slopes failed!\n");
//		if((ui32ErrorCode & CAL_FAILED_PH_CR_SLOPES) != 0)
//			UARTprintf("Error! If this is a calibration pH chronoionophore slopes failed, if this is a test thermistor read incorrectly!\n");
//		if((ui32ErrorCode & CAL_FAILED_CA_SLOPES) != 0)
//			UARTprintf("Error! If this is a calibration Ca slopes failed, if this is a test FCl didn't mix to correct conductivity!\n");
//		if((ui32ErrorCode & CAL_FAILED_TH_SLOPES) != 0)
//			UARTprintf("Error! If this is a calibration TH slopes failed, if this is a test TCl didn't mix to correct conductivity!\n");
//		if((ui32ErrorCode & CAL_FAILED_NH4_SLOPES) != 0)
//			UARTprintf("Error! NH4 slopes failed calibration!\n");
//		if((ui32ErrorCode & CAL_FAILED_COND_SLOPES) != 0)
//			UARTprintf("Error! Conductivity slopes failed calibration!\n");
//		if((ui32ErrorCode & MEMORY_FAILED) != 0)
//			UARTprintf("Error! Cartridge memory communication failure, I2CMasterErr() returned error!\n");
//		if((ui32ErrorCode & USER_CANCELLED) != 0)
//			UARTprintf("Error! User aborted run!\n");
//		if((ui32ErrorCode & ALK_MIX_OUT_OF_RANGE) != 0)
//			UARTprintf("Error! Alkalinity mixed more than twice!\n");
//		if((ui32ErrorCode & LED_EXTENDER_FAIL) != 0)
//			UARTprintf("Error! LED extender not writing and reading back correctly or output is not moving!\n");
//		if((ui32ErrorCode & ADC5_FAIL) != 0)
//			UARTprintf("Error! ADC 5 checking conductivity frequency and turbidity failed!\n");
//		if((ui32ErrorCode & EEPROM_FAIL) != 0)
//			UARTprintf("Error! Tiva EEPROM failed to recover from an interrupted write or erase operation!\n");
//		if((ui32ErrorCode & BATTERY_FAIL) != 0)
//			UARTprintf("Error! Battery monitor chip failed to communicate correctly!\n");
//		if((ui32ErrorCode & CAL_REPUMP_FAIL) != 0)
//			UARTprintf("Calibration had to repump a solution!\n");
//		if((ui32ErrorCode & VALVE_DRIFT_FAIL) != 0)
//			UARTprintf("Error! Valve drifted farther than acceptable during run!\n");
//		if((ui32ErrorCode & PUMP_DRIFT_FAIL) != 0)
//			UARTprintf("Error! Pump drifted farther than acceptable during run!\n");
//	}
//}

//********************************************************************************
// Prints out over UART what the errors are in a given error code
// Created: 1/23/2020
//	6/3/2021: Created option to not create new lines between errors
// Inputs:	ui32ErrorCode; the error code to print, will usually be gui32Error
// Outputs: NONE
//********************************************************************************
void PrintErrors(uint32_t ui32ErrorCode, uint8_t new_lines, uint8_t state)
{
	DEBUG_PRINT(
	if(ui32ErrorCode != 0)
	{
		UARTprintf("Errors:");
		if(new_lines)
			UARTprintf("\n");
		else
			UARTprintf("; ");

		if((ui32ErrorCode & MAX_TESTS_REACHED) != 0)
		{
			if(state == STATE_MEASUREMENT)
				UARTprintf("Max tests exceeded!");
			else if(state == STATE_CALIBRATION)
				UARTprintf("Max calibrations exceeded!");
			else
				UARTprintf("Max tests or calibrations exceeded!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}
//		if((ui32ErrorCode & MAX_CALS_REACHED) != 0 && state == STATE_CALIBRATION)
//		{
//			UARTprintf("Max calibrations exceeded!");
//			if(new_lines)
//				UARTprintf("\n");
//			else
//				UARTprintf("; ");
//		}

		if((ui32ErrorCode & CARTRIDGE_EXPIRED) != 0)
		{
			UARTprintf("Cartridge expired!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & BATTERY_TOO_LOW) != 0)
		{
			UARTprintf("Battery too low, valve and pump reliability questionable!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & I2C_FAILED) != 0)
		{
			UARTprintf("I2C communication failed, I2CMasterErr() Tivaware function returned error!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & IO_EXT1_FAIL) != 0)
		{
			UARTprintf("IO Extender 1 failed to write and read back correctly!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & IO_EXT2_FAIL) != 0)
		{
			UARTprintf("IO Extender 2 failed to write and read back correctly!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & DAC_FAIL) != 0)
		{
			UARTprintf("DAC failed to write and read back correctly!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & ADC1_FAIL) != 0)
		{
			UARTprintf("ADC 1 failed to write and read back channel scan register!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & ADC2_FAIL) != 0)
		{
			UARTprintf("ADC 2 failed to write and read back channel scan register!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & ADC3_FAIL) != 0)
		{
			UARTprintf("Conductivity ADC never returned anything more than 0s!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & ADC4_FAIL) != 0)
		{
			UARTprintf("ADC 4 failed to write and read back channel scan register!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & WAVE_GEN_FAIL) != 0)
		{
			UARTprintf("Waveform generator not creating 1 kHz signal!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & BT_FAILED) != 0)
		{
			UARTprintf("Bluetooth chip didn't respond to request, had to reset!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & ROAM_RESET) != 0)
		{
			UARTprintf("Roam was reset partway through and didn't finish run!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & CL_CLEANING_OUT_OF_RANGE) != 0)
		{
			UARTprintf("Amperometic cleaning either returned currents with wrong sign or rebuild voltages were more than 100 mV apart!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

//		if((ui32ErrorCode & FCL_MIX_OUT_OF_RANGE) != 0)
//			UARTprintf("Error! Free chlorine mixing never found correct pH range!\n");
//		if((ui32ErrorCode & TCL_MIX_OUT_OF_RANGE) != 0)
//			UARTprintf("Error! Total chlorine mixing never found correct pH range!\n");
		if((ui32ErrorCode & CAL_FAILED_PH_H2_SLOPES) != 0)
		{
			if(state == STATE_CALIBRATION)
				UARTprintf("pH H2 slopes failed!");
			else if(state == STATE_MEASUREMENT)
				UARTprintf("T1 Prime conductivity check failed!");
			else
				UARTprintf("If this is a calibration pH H2 slopes failed, if this is a test T1 Prime check failed!");

			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & CAL_FAILED_PH_CR_SLOPES) != 0)
		{
			if(state == STATE_CALIBRATION)
				UARTprintf("pH chronoionophore slopes failed!");
			else if(state == STATE_MEASUREMENT)
				UARTprintf("Thermistor read incorrectly!");
			else
				UARTprintf("If this is a calibration pH Cr slopes failed, if this is a test thermistor read incorrectly!");

			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & CAL_FAILED_CA_SLOPES) != 0)
		{
			if(state == STATE_CALIBRATION)
				UARTprintf("Ca slopes failed!");
			else if(state == STATE_MEASUREMENT)
				UARTprintf("FCl didn't mix to correct conductivity!");
			else
				UARTprintf("If this is a calibration Ca slopes failed, if this is a test FCl didn't mix to correct conductivity!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & CAL_FAILED_TH_SLOPES) != 0)
		{
			if(state == STATE_CALIBRATION)
				UARTprintf("TH slopes failed!");
			else if(state == STATE_MEASUREMENT)
				UARTprintf("TCl didn't mix to correct conductivity!");
			else
				UARTprintf("If this is a calibration TH slopes failed, if this is a test TCl didn't mix to correct conductivity!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & CAL_FAILED_NH4_SLOPES) != 0)
		{
			if(state == STATE_CALIBRATION)
				UARTprintf("NH4 slopes failed calibration!");
			else if(state == STATE_MEASUREMENT)
				UARTprintf("B1 Prime didn't find correct conductivity on first try!");
			else
				UARTprintf("NH4 slopes failed calibration, or B1 prime didn't find correct conductivity!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & CAL_FAILED_COND_SLOPES) != 0)
		{
			if(state == STATE_CALIBRATION)
				UARTprintf("Conductivity slopes failed calibration!");
			else if(state == STATE_MEASUREMENT)
				UARTprintf("C2 Prime didn't find correct conductivity on first try!");
			else
				UARTprintf("Conductivity slopes failed calibration, or C2 prime didn't find correct conductivity!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & MEMORY_FAILED) != 0)
		{
			UARTprintf("Cartridge memory communication failure, I2CMasterErr() returned error!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & USER_CANCELLED) != 0)
		{
			UARTprintf("User aborted run!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & ALK_MIX_OUT_OF_RANGE) != 0)
		{
			UARTprintf("Alkalinity mixed more than twice!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & LED_EXTENDER_FAIL) != 0)
		{
			UARTprintf("LED extender not writing and reading back correctly or output is not moving!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & ADC5_FAIL) != 0)
		{
			UARTprintf("ADC 5 checking conductivity frequency and turbidity failed!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & EEPROM_FAIL) != 0)
		{
			UARTprintf("Tiva EEPROM failed to recover from an interrupted write or erase operation!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & BATTERY_FAIL) != 0)
		{
			UARTprintf("Battery monitor chip failed to communicate correctly!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & CAL_REPUMP_FAIL) != 0)
		{
			UARTprintf("Calibration had to repump a solution!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & VALVE_DRIFT_FAIL) != 0)
		{
			UARTprintf("Valve drifted farther than acceptable during run!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}

		if((ui32ErrorCode & PUMP_DRIFT_FAIL) != 0)
		{
			UARTprintf("Pump drifted farther than acceptable during run!");
			if(new_lines)
				UARTprintf("\n");
			else
				UARTprintf("; ");
		}
	}
	)	// DEBUG_PRINT
}

//********************************************************************************
// Calculates what page in memory the calibration data is on given cal number
// Created: 10/5/2020
// Inputs:	Cal_Number, the calibration number to find page for
// Outputs: Cal_page, the page in memory the calibration data is on
//********************************************************************************
uint16_t Find_Cal_page(uint16_t Cal_Number)
{
	uint16_t Cal_page = (PAGE_CAL + Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
	while(Cal_page > (PAGE_TEST - PAGES_FOR_CAL))
		Cal_page -= (PAGE_TEST - PAGE_CAL);

	return Cal_page;
}

//********************************************************************************
// Calculates what page in memory the test data is on given test number
// Created: 8/26/2021
// Inputs:	Test_Number, the calibration number to find page for
// Outputs: Test_page, the page in memory the test data is on
//********************************************************************************
uint16_t Find_Test_page(uint16_t Test_Number)
{
	uint16_t Test_page = ((PAGE_TEST + Test_Number * PAGES_FOR_TEST) - PAGES_FOR_TEST);
	while(Test_page > (512 - PAGES_FOR_TEST))
		Test_page -= (512 - PAGE_TEST);

	return Test_page;
}

////********************************************************************************
//// Pick which pH sensor to report, put in function so I can update in one place
//// and it updates for all version of code
//// Created: 11/24/2020
//// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
////			pH_Samp; pointer to array holding the calculated pH values for each sensor
////			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
////			T_Rinse; Temperature of rinse during this test
//// Outputs: T_Chosen_pH; the chosen sensor
////********************************************************************************
//uint8_t Choose_pH_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse)
//{
//	uint8_t i;
//	uint8_t T_Chosen_pH = 0;
//
//	uint16_t Cal_page = Find_Cal_page(Cal_Number);
//	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
//	uint8_t pH_Cal_Status[3] = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
//
//	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
//	uint8_t PS_Chosen_pH = Chosen_Sensors & 0x03;
//
//	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
//	uint8_t Last_cal_passed[10];
//	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
//	for(i = 0; i < 10; i++)
//		if(Last_cal_passed[i] == 0xFF)
//			Last_cal_passed[i] = Cal_Number;
//
//	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
//
//	float pH_EEP_Slope[3];
//	pH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_SLOPE, 4));
//	pH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_SLOPE, 4));
//	pH_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_SLOPE, 4));
//
//	float pH_EEP_Int[3];
//	pH_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_INT, 4));
//	pH_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_INT, 4));
//	pH_EEP_Int[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_INT, 4));
//
//	float T_EEP_Cal[3];
//	T_EEP_Cal[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_T_CAL, 4));
//	T_EEP_Cal[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_T_CAL, 4));
//	T_EEP_Cal[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_T_CAL, 4));
//
//	// Set up limits to check against
//	float pH_Max = 11;
//	float pH_Min = 5;
//	float pH_linear_split = 10;	// mV to compare test prerinse to cal prerinse
//	float pH_linear_mV[3];
//	for(i = 0; i < 3; i++)
//	{
//		float pH_Cal_1_mV = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
//		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * ((pH_EEP_Rinse + K_T_pH_Rinse * (T_Rinse - 25)) - (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)));
//	}
//
//	// Choose pH sensor
//	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
//	{
//		float within = 0.05; // pH units
//		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
//		{
//			float max_reading = pH_Samp[0];
//			float min_reading = pH_Samp[0];
//			for(i = 1; i < 3; i++)
//			{
//				if(pH_Samp[i] > max_reading)
//					max_reading = pH_Samp[i];
//				if(pH_Samp[i] < min_reading)
//					min_reading = pH_Samp[i];
//			}
//			if(abs_val(max_reading - min_reading) < within) // All three points read close to each other, go with chosen from calibration
//				T_Chosen_pH = PS_Chosen_pH;
//
//			if(abs_val(max_reading - min_reading) < within) // All three points read close to each other, go with chosen from calibration
//				T_Chosen_pH = PS_Chosen_pH;
//			else	// The spread between the three points is greater than 0.1 pH units, check rinse mV
//			{
//				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
//				{
//					if(abs_val(pH_E_Rinse[i] - pH_linear_mV[i]) > pH_linear_split)
//						pH_Cal_Status[i] = 0;
//					if(pH_Samp[i] > 11 || pH_Samp[i] < 5)
//						pH_Cal_Status[i] = 0;
//				}
//
//				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
//				{
//					float Differences[3];
//					Differences[0] = abs_val(pH_Samp[0] - pH_Samp[1]);
//					Differences[1] = abs_val(pH_Samp[0] - pH_Samp[2]);
//					Differences[2] = abs_val(pH_Samp[1] - pH_Samp[2]);
//
//					uint8_t Min_diff = 0;
//					for(i = 1; i < 3; i++)
//						if(Differences[Min_diff] > Differences[i])
//							Min_diff = i;
//
//					if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
//					{
//						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
//							T_Chosen_pH = 0;
//						else
//							T_Chosen_pH = 1;
//					}
//					else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
//					{
//						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
//							T_Chosen_pH = 0;
//						else
//							T_Chosen_pH = 2;
//					}
//					else	// Smallest split is between sensors 2 and 3
//					{
//						if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
//							T_Chosen_pH = 1;
//						else
//							T_Chosen_pH = 2;
//					}
//				}
//				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
//				{
//					if(pH_Cal_Status[0] && pH_Cal_Status[1]) // Sensors 1 and 2 are the ones that passed
//					{
//						if(abs_val(pH_Samp[0] - pH_Samp[1]) < within)	// pH sensors are close to each other
//						{
//							if(PS_Chosen_pH == 0 || PS_Chosen_pH == 1)	// If the calibration chosen sensor was one that passed go with that sensor
//								T_Chosen_pH = PS_Chosen_pH;
//							else // Calibration chosen sensor failed rinse check
//							{
//								if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
//									T_Chosen_pH = 0;
//								else
//									T_Chosen_pH = 1;
//							}
//						}
//						else	// pH sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
//						{
//							if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
//								T_Chosen_pH = 0;
//							else
//								T_Chosen_pH = 1;
//						}
//					}
//					else if(pH_Cal_Status[0] && pH_Cal_Status[2])	// Sensors 1 and 3 are the ones that passed
//					{
//						if(abs_val(pH_Samp[0] - pH_Samp[2]) < within)	// pH sensors are close to each other
//						{
//							if(PS_Chosen_pH == 0 || PS_Chosen_pH == 2)	// If the calibration chosen sensor was one that passed go with that sensor
//								T_Chosen_pH = PS_Chosen_pH;
//							else // Calibration chosen sensor failed rinse check
//							{
//								if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
//									T_Chosen_pH = 0;
//								else
//									T_Chosen_pH = 2;
//							}
//						}
//						else	// pH sensors did not read close to each other, both rinse checks passed
//						{
//							if((pH_Samp[0] > pH_Max || pH_Samp[0] < pH_Min) && (pH_Samp[2] <= pH_Max && pH_Samp[2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
//								T_Chosen_pH = 1;
//							else if((pH_Samp[0] <= pH_Max && pH_Samp[0] >= pH_Min) && (pH_Samp[2] > pH_Max || pH_Samp[2] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
//								T_Chosen_pH = 0;
//							else	// both pH sensors read within device range then just go with sensor chosen during calibration
//							{
//								if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
//									T_Chosen_pH = 0;
//								else
//									T_Chosen_pH = 2;
//							}
//						}
//					}
//					else	// Sensors 2 and 3 are the ones that passed
//					{
//						if(abs_val(pH_Samp[1] - pH_Samp[2]) < within)	// pH sensors are close to each other
//						{
//							if(PS_Chosen_pH == 1 || PS_Chosen_pH == 2)	// If the calibration chosen sensor was one that passed go with that sensor
//								T_Chosen_pH = PS_Chosen_pH;
//							else // Calibration chosen sensor failed rinse check
//							{
//								if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
//									T_Chosen_pH = 1;
//								else
//									T_Chosen_pH = 2;
//							}
//						}
//						else	// pH sensors did not read close to each other, both rinse checks passed
//						{
//							if((pH_Samp[1] > pH_Max || pH_Samp[1] < pH_Min) && (pH_Samp[2] <= pH_Max && pH_Samp[2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
//								T_Chosen_pH = 2;
//							else if((pH_Samp[1] <= pH_Max && pH_Samp[1] >= pH_Min) && (pH_Samp[2] > pH_Max || pH_Samp[2] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
//								T_Chosen_pH = 1;
//							else	// both pH sensors read within device range then just go with sensor chosen during calibration
//							{
//								if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
//									T_Chosen_pH = 1;
//								else
//									T_Chosen_pH = 2;
//							}
//						}
//					}
//				}
//				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
//				{
//					if(pH_Cal_Status[0] == 1)
//						T_Chosen_pH = 0;
//					else if(pH_Cal_Status[1] == 1)
//						T_Chosen_pH = 1;
//					else
//						T_Chosen_pH = 2;
//				}
//				else	// All sensors failed the rinse mV check
//				{
//					T_Chosen_pH = PS_Chosen_pH;
//				}
//			}
//		}
//		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
//		{
//			if(pH_Cal_Status[0] && pH_Cal_Status[1]) // Sensors 1 and 2 are the ones that passed
//			{
//				if(abs_val(pH_Samp[0] - pH_Samp[1]) < within)	// pH sensors are close to each other
//				{
//					T_Chosen_pH = PS_Chosen_pH;
//				}
//				else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
//				{
//					if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) > pH_linear_split && abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
//						T_Chosen_pH = 1;
//					else if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < pH_linear_split && abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
//						T_Chosen_pH = 0;
//					else	// If both sensors read within 20 mV of their theoretical rinse value
//					{
//						if((pH_Samp[0] > pH_Max || pH_Samp[0] < pH_Min) && (pH_Samp[1] <= pH_Max && pH_Samp[1] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
//							T_Chosen_pH = 1;
//						else if((pH_Samp[0] <= pH_Max && pH_Samp[0] >= pH_Min) && (pH_Samp[1] > pH_Max || pH_Samp[1] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
//							T_Chosen_pH = 0;
//						else	// both pH sensors read within device range then just go with sensor chosen during calibration
//							T_Chosen_pH = PS_Chosen_pH;
//					}
//				}
//			}
//			else if(pH_Cal_Status[0] && pH_Cal_Status[2])	// Sensors 1 and 3 are the ones that passed
//			{
//				if(abs_val(pH_Samp[0] - pH_Samp[2]) < within)	// pH sensors are close to each other
//				{
//					T_Chosen_pH = PS_Chosen_pH;
//				}
//				else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
//				{
//					if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) > pH_linear_split && abs_val(pH_E_Rinse[2] - pH_linear_mV[2]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
//						T_Chosen_pH = 2;
//					else if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < pH_linear_split && abs_val(pH_E_Rinse[2] - pH_linear_mV[2]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
//						T_Chosen_pH = 0;
//					else	// If both sensors read within 20 mV of their theoretical rinse value
//					{
//						if((pH_Samp[0] >= pH_Max || pH_Samp[0] <= pH_Min) && (pH_Samp[2] <= pH_Max && pH_Samp[2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
//							T_Chosen_pH = 2;
//						else if((pH_Samp[0] <= pH_Max && pH_Samp[0] >= pH_Min) && (pH_Samp[2] >= pH_Max || pH_Samp[2] <= pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
//							T_Chosen_pH = 0;
//						else	// both pH sensors read within device range then just go with sensor chosen during calibration
//							T_Chosen_pH = PS_Chosen_pH;
//					}
//				}
//			}
//			else	// Sensors 2 and 3 are the ones that passed
//			{
//				if(abs_val(pH_Samp[1] - pH_Samp[2]) < within)	// pH sensors are close to each other
//				{
//					T_Chosen_pH = PS_Chosen_pH;
//				}
//				else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
//				{
//					if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) > pH_linear_split && abs_val(pH_E_Rinse[2] - pH_linear_mV[2]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
//						T_Chosen_pH = 2;
//					else if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < pH_linear_split && abs_val(pH_E_Rinse[2] - pH_linear_mV[2]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
//						T_Chosen_pH = 1;
//					else	// If both sensors read within 20 mV of their theoretical rinse value
//					{
//						if((pH_Samp[1] >= pH_Max || pH_Samp[1] <= pH_Min) && (pH_Samp[2] <= pH_Max && pH_Samp[2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
//							T_Chosen_pH = 2;
//						else if((pH_Samp[1] <= pH_Max && pH_Samp[1] >= pH_Min) && (pH_Samp[2] >= pH_Max || pH_Samp[2] <= pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
//							T_Chosen_pH = 1;
//						else	// both pH sensors read within device range then just go with sensor chosen during calibration
//							T_Chosen_pH = PS_Chosen_pH;
//					}
//				}
//			}
//		}
//	}
//	else	// If only one or no sensors passed
//	{
//		if(pH_Cal_Status[1] == 1)
//			T_Chosen_pH = 1;
//		else if(pH_Cal_Status[2] == 1)
//			T_Chosen_pH = 2;
//	}
//
//	return T_Chosen_pH;
//}

#ifdef MEMORY_V4
//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[3] = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_pH = Chosen_Sensors & 0x03;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));

	float pH_EEP_Slope[3];
	pH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_SLOPE, 4));
	pH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_SLOPE, 4));
	pH_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_SLOPE, 4));

	float pH_EEP_Int[3];
	pH_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_INT, 4));
	pH_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_INT, 4));
	pH_EEP_Int[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_INT, 4));

	float T_EEP_Cal[3];
	T_EEP_Cal[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_T_CAL, 4));
	T_EEP_Cal[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_T_CAL, 4));
	T_EEP_Cal[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_T_CAL, 4));

	// Set up limits to check against
	float pH_Max = 11;
	float pH_Min = 5;
	float pH_linear_split = 10;	// mV to compare test prerinse to cal prerinse
	float pH_linear_mV[3];
	for(i = 0; i < 3; i++)
	{
		float pH_Cal_1_mV = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * ((pH_EEP_Rinse + K_T_pH_Rinse * (T_Rinse - 25)) - (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)));
	}

	// Choose pH sensor
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		float within = 0.05; // pH units
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find min and max reading to see how far apart all three are
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(pH_Samp[i] > pH_Samp[max_reading])
					max_reading = i;
				if(pH_Samp[i] < pH_Samp[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(pH_Samp[max_reading] - pH_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_pH = PS_Chosen_pH;
			else if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within || abs_val(pH_Samp[mid_reading] - pH_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_pH || max_reading == PS_Chosen_pH)
					T_Chosen_pH = PS_Chosen_pH;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_E_Rinse[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_E_Rinse[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_pH = min_reading;
					else
						T_Chosen_pH = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.1 pH units, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_E_Rinse[i] - pH_linear_mV[i]) > pH_linear_split)
						pH_Cal_Status[i] = 0;
					if(pH_Samp[i] > pH_Max || pH_Samp[i] < pH_Min)
						pH_Cal_Status[i] = 0;
				}

				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					float Differences[3];
					Differences[0] = abs_val(pH_Samp[0] - pH_Samp[1]);
					Differences[1] = abs_val(pH_Samp[0] - pH_Samp[2]);
					Differences[2] = abs_val(pH_Samp[1] - pH_Samp[2]);

					uint8_t Min_diff = 0;
					for(i = 1; i < 3; i++)
						if(Differences[Min_diff] > Differences[i])
							Min_diff = i;

					if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
					else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 2;
					}
					else	// Smallest split is between sensors 2 and 3
					{
						if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 1;
						else
							T_Chosen_pH = 2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed rinse mV check
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
					{
						if(PS_Chosen_pH == spot_1 || PS_Chosen_pH == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_pH = PS_Chosen_pH;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_pH = spot_1;
							else
								T_Chosen_pH = spot_2;
						}
					}
					else	// pH sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(pH_Cal_Status[0] == 1)
						T_Chosen_pH = 0;
					else if(pH_Cal_Status[1] == 1)
						T_Chosen_pH = 1;
					else
						T_Chosen_pH = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
			{
				T_Chosen_pH = PS_Chosen_pH;
			}
			else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) > pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_pH = spot_2;
				else if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_pH = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((pH_Samp[spot_1] > pH_Max || pH_Samp[spot_1] < pH_Min) && (pH_Samp[spot_2] <= pH_Max && pH_Samp[spot_2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_pH = spot_2;
					else if((pH_Samp[spot_1] <= pH_Max && pH_Samp[spot_1] >= pH_Min) && (pH_Samp[spot_2] > pH_Max || pH_Samp[spot_2] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_pH = spot_1;
					else	// both pH sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_pH = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_pH = 2;
	}

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// 1/15/2021: Set up to work for full pH die,
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_Sensor_pHDie(uint16_t Cal_Number, float * pH_Samp)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
//	uint8_t ui8Size_pH = 3;
//	if(Sensor_Config == PH_CL_CART)
//		ui8Size_pH = 10;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[10] = {0,0,0,0,0,0,0,0,0,0};// = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
	pH_Cal_Status[0] = (Cal_Status >> 1) & 1;
	pH_Cal_Status[1] = (Cal_Status >> 2) & 1;
	pH_Cal_Status[2] = (Cal_Status >> 28) & 1;
	if(Sensor_Config == PH_CL_CART)
	{
		pH_Cal_Status[3] = (Cal_Status >> 5) & 1;
		pH_Cal_Status[4] = (Cal_Status >> 6) & 1;
		pH_Cal_Status[5] = (Cal_Status >> 7) & 1;
		pH_Cal_Status[6] = (Cal_Status >> 8) & 1;
		pH_Cal_Status[7] = (Cal_Status >> 9) & 1;
		pH_Cal_Status[8] = (Cal_Status >> 3) & 1;
		pH_Cal_Status[9] = (Cal_Status >> 4) & 1;
	}

//	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
//	uint8_t PS_Chosen_pH = Chosen_Sensors & 0x03;
//	if(Sensor_Config == PH_CL_CART)
//		PS_Chosen_pH = Chosen_Sensors;

//	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
//	uint8_t Last_cal_passed[10];
//	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
//	for(i = 0; i < 10; i++)
//		if(Last_cal_passed[i] == 0xFF)
//			Last_cal_passed[i] = Cal_Number;
//
//	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
//
//	float pH_EEP_Slope[10];
//	pH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_SLOPE, 4));
//	pH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_SLOPE, 4));
//	pH_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_SLOPE, 4));
//	if(Sensor_Config == PH_CL_CART)
//	{
//		pH_EEP_Slope[3] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[3]), OFFSET_PH_4_SLOPE, 4));
//		pH_EEP_Slope[4] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[4]), OFFSET_PH_5_SLOPE, 4));
//		pH_EEP_Slope[5] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[5]), OFFSET_PH_6_SLOPE, 4));
//		pH_EEP_Slope[6] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[6]), OFFSET_PH_7_SLOPE, 4));
//		pH_EEP_Slope[7] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[7]), OFFSET_PH_8_SLOPE, 4));
//		pH_EEP_Slope[8] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[8]), OFFSET_PH_9_SLOPE, 4));
//		pH_EEP_Slope[9] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[9]), OFFSET_PH_10_SLOPE, 4));
//	}
//
//	float pH_EEP_Int[10];
//	pH_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_INT, 4));
//	pH_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_INT, 4));
//	pH_EEP_Int[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_INT, 4));
//	if(Sensor_Config == PH_CL_CART)
//	{
//		pH_EEP_Int[3] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[3]), OFFSET_PH_4_INT, 4));
//		pH_EEP_Int[4] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[4]), OFFSET_PH_5_INT, 4));
//		pH_EEP_Int[5] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[5]), OFFSET_PH_6_INT, 4));
//		pH_EEP_Int[6] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[6]), OFFSET_PH_7_INT, 4));
//		pH_EEP_Int[7] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[7]), OFFSET_PH_8_INT, 4));
//		pH_EEP_Int[8] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[8]), OFFSET_PH_9_INT, 4));
//		pH_EEP_Int[9] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[9]), OFFSET_PH_10_INT, 4));
//	}

	// Set up limits to check against
//	float pH_Max = 11;
//	float pH_Min = 5;
//	float pH_linear_split = 10;	// mV to compare test prerinse to cal prerinse
//	float pH_linear_mV[10];
//	for(i = 0; i < ui8Size_pH; i++)
//	{
//		float T_EEP_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i]), OFFSET_T_CAL, 4));
//		float pH_Cal_1_mV = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
//		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * ((pH_EEP_Rinse + K_T_pH_Rinse * (T_Rinse - 25)) - (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)));
//	}

	// Choose pH sensor
	float within = 0.01; // pH units

	// Choose a sensor by trying to find groupings of readings, pick the largest grouping and go with the median

	// Count how many close sensors each sensor has and record the max
	uint8_t CloseSensors[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t MaxClose = 0;
	uint8_t j;
	for(i = 0; i < 10; i++)
	{
		if(pH_Cal_Status[i])
			for(j = 0; j < 10; j++)
				if(abs_val(pH_Samp[i] - pH_Samp[j]) <= within && pH_Cal_Status[j])
					CloseSensors[i]++;

		if(CloseSensors[i] > MaxClose)
			MaxClose = CloseSensors[i];
	}

	// After finding max, find out how many sensors matched this max
	uint8_t MaxClose_Sensors[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t TotalCloseSensors = 0;
	for(i = 0; i < 10; i++)
		if(CloseSensors[i] == MaxClose)
		{
			MaxClose_Sensors[i] = 1;	// Set flag so each sensor that had max sensors close is set to 1 and all others are 0
			TotalCloseSensors++;
		}

	// Create array with index values sorted from lowest to highest pH
	uint8_t SortedSensors[10] = {0,0,0,0,0,0,0,0,0,0};
//	uint8_t CurrentIndex = 0;
	for(i = 0; i < TotalCloseSensors; i++)
	{
		uint8_t MinIndex = 0;

		for(j = 0; j < 10; j++)
		{
			if(MaxClose_Sensors[j] == 1)
			{
				MinIndex = j;
				break;
			}
		}

		for(j = 0; j < 10; j++)
		{
			if(MaxClose_Sensors[j] == 1 && pH_Samp[j] < pH_Samp[MinIndex])
				MinIndex = j;
		}

		MaxClose_Sensors[MinIndex] = 0;
		SortedSensors[i] = MinIndex;
	}

	// Set the chosen sensor as the median sensor from the tight group
	if(TotalCloseSensors % 2 == 0)	// If an even number, divide by 2 to find the index to return
		T_Chosen_pH = SortedSensors[TotalCloseSensors / 2 - 1];
	else
		T_Chosen_pH = SortedSensors[TotalCloseSensors / 2];

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which Ca sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			Ca_Hardness; pointer to array holding the calculated Calcium Hardness values for each sensor
//			Ca_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_Ca; the chosen sensor
//********************************************************************************
uint8_t Choose_Ca_Sensor(uint16_t Cal_Number, float * Ca_Hardness, float * Ca_R)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t Ca_Cal_Status[2] = {(Cal_Status >> 3) & 1, (Cal_Status >> 4) & 1};

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_Ca = (Chosen_Sensors >> 2) & 0x01;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));

	float Ca_EEP_Slope[2];
	Ca_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[3]), OFFSET_CA_1_SLOPE, 4));
	Ca_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[4]), OFFSET_CA_2_SLOPE, 4));

	float Ca_EEP_Int[2];
	Ca_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[3]), OFFSET_CA_1_INT, 4));
	Ca_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[4]), OFFSET_CA_2_INT, 4));

	// Set up bounds to check against
	float Ca_Hardness_Max = 1250;
	float Ca_Hardness_Min = 2;
	float Ca_linear_mV_Split = 5;
	float within = 10; // ppm Ca Hardness

	float Ca_linear_mV[2];
	for(i = 0; i < 2; i++)
	{
		float Ca_Cal_1_mV = Ca_EEP_Slope[i] * Ca_EEP_Cal_1 + Ca_EEP_Int[i];
		Ca_linear_mV[i] = Ca_Cal_1_mV + Ca_EEP_Slope[i] * (Ca_EEP_Rinse - Ca_EEP_Cal_1);
	}

	// Choose Ca sensor
	uint8_t T_Chosen_Ca = 0;
	if((Ca_Cal_Status[0] + Ca_Cal_Status[1]) > 1)	// Check that both sensors passed
	{
		if(abs_val(Ca_Hardness[0] - Ca_Hardness[1]) < within)	// Ca sensors are close to each other
		{
			T_Chosen_Ca = PS_Chosen_Ca;
		}
		else	// Ca sensors did not read close to each other, check if rinse value is close to what it should be
		{
			if(abs_val(Ca_R[0] - Ca_linear_mV[0]) > Ca_linear_mV_Split && abs_val(Ca_R[1] - Ca_linear_mV[1]) < Ca_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
				T_Chosen_Ca = 1;
			else if(abs_val(Ca_R[0] - Ca_linear_mV[0]) < Ca_linear_mV_Split && abs_val(Ca_R[1] - Ca_linear_mV[1]) > Ca_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
				T_Chosen_Ca = 0;
			else	// If both sensors read within 5 mV of their theoretical rinse value or they both read further away
			{
				if((Ca_Hardness[0] > Ca_Hardness_Max || Ca_Hardness[0] < Ca_Hardness_Min) && (Ca_Hardness[1] <= Ca_Hardness_Max && Ca_Hardness[1] >= Ca_Hardness_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
					T_Chosen_Ca = 1;
				else if((Ca_Hardness[0] <= Ca_Hardness_Max && Ca_Hardness[0] >= Ca_Hardness_Min) && (Ca_Hardness[1] > Ca_Hardness_Max || Ca_Hardness[1] < Ca_Hardness_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
					T_Chosen_Ca = 0;
				else	// both pH sensors read within device range then just go with sensor chosen during calibration
					T_Chosen_Ca = PS_Chosen_Ca;
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(Ca_Cal_Status[1] == 1)
			T_Chosen_Ca = 1;
	}

	return T_Chosen_Ca;
}

//********************************************************************************
// Pick which TH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			TH_corr; pointer to array holding the calculated Total Hardness values for each sensor
//			TH_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_TH; the chosen sensor
//********************************************************************************
uint8_t Choose_TH_Sensor(uint16_t Cal_Number, float * TH_corr, float * TH_R)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t TH_Cal_Status[2] = {(Cal_Status >> 5) & 1, (Cal_Status >> 6) & 1};

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_TH = (Chosen_Sensors >> 3) & 0x01;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
	float TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));
	float TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_TH, 4));

	float TH_EEP_Slope[2];
	TH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[5]), OFFSET_TH_1_SLOPE, 4));
	TH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[6]), OFFSET_TH_2_SLOPE, 4));

	float TH_EEP_Int[2];
	TH_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[5]), OFFSET_TH_1_INT, 4));
	TH_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[6]), OFFSET_TH_2_INT, 4));

	// Set up Bounds to check against
	float TH_Max = 2000;
	float TH_Min = 5;
	float TH_linear_mV_Split = 5;	// Acceptable difference between cal prerinse mV and test prerinse mV
	float within = 10; // ppm Total Hardness

	float TH_linear_mV[2];
	for(i = 0; i < 2; i++)
	{
		float log_K_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[5 + i]), OFFSET_CAL_LOG_K, 4));
		float TH_Cal_1_mV = TH_EEP_Slope[0] * (-log10(pow(10, -(-log10(pow(10, -TH_EEP_Cal_1) - pow(10, -Ca_EEP_Cal_1)))) + pow(10, log_K_Cal) * pow(10, -Ca_EEP_Cal_1))) + TH_EEP_Int[0];
		TH_linear_mV[i] = TH_Cal_1_mV + TH_EEP_Slope[i] * (TH_EEP_Rinse - TH_EEP_Cal_1);
	}

	// Choose TH sensor
	uint8_t T_Chosen_TH = 0;
	if((TH_Cal_Status[0] + TH_Cal_Status[1]) > 1)	// Check that both sensors passed
	{
		if(abs_val(TH_corr[0] - TH_corr[1]) < within)	// TH sensors are close to each other
		{
			T_Chosen_TH = PS_Chosen_TH;
		}
		else	// TH sensors did not read close to each other, check if rinse value is close to what it should be
		{
			if(abs_val(TH_R[0] - TH_linear_mV[0]) > TH_linear_mV_Split && abs_val(TH_R[1] - TH_linear_mV[1]) < TH_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
				T_Chosen_TH = 1;
			else if(abs_val(TH_R[0] - TH_linear_mV[0]) < TH_linear_mV_Split && abs_val(TH_R[1] - TH_linear_mV[1]) > TH_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
				T_Chosen_TH = 0;
			else	// If both sensors read within 5 mV of their theoretical rinse value or they both read further away
			{
				if((TH_corr[0] > TH_Max || TH_corr[0] < TH_Min) && (TH_corr[1] <= TH_Max && TH_corr[1] >= TH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
					T_Chosen_TH = 1;
				else if((TH_corr[0] <= TH_Max && TH_corr[0] >= TH_Min) && (TH_corr[1] > TH_Max || TH_corr[1] < TH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
					T_Chosen_TH = 0;
				else	// both pH sensors read within device range then just go with sensor chosen during calibration
					T_Chosen_TH = PS_Chosen_TH;
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(TH_Cal_Status[1] == 1)
			T_Chosen_TH = 1;
	}

	return T_Chosen_TH;
}

//********************************************************************************
// Pick which TH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			NH4_NH3_N_Total; pointer to array holding the calculated NH4 values for each sensor
//			NH4_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_NH4; the chosen sensor
//********************************************************************************
uint8_t Choose_NH4_Sensor(uint16_t Cal_Number, float * NH4_NH3_N_Total, float * NH4_R)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t NH4_Cal_Status[3] = {(Cal_Status >> 7) & 1, (Cal_Status >> 8) & 1, (Cal_Status >> 9) & 1};

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_NH4 = (Chosen_Sensors >> 4) & 0x03;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_NH4, 4));
	float NH4_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_NH4, 4));

	float NH4_EEP_Slope[3];
	NH4_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[7]), OFFSET_NH4_1_SLOPE, 4));
	NH4_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[8]), OFFSET_NH4_2_SLOPE, 4));
	NH4_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[9]), OFFSET_NH4_3_SLOPE, 4));

	float NH4_EEP_Int[3];
	NH4_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[7]), OFFSET_NH4_1_INT, 4));
	NH4_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[8]), OFFSET_NH4_2_INT, 4));
	NH4_EEP_Int[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[9]), OFFSET_NH4_3_INT, 4));

	// Set up bounds to check against
	float NH4_Max = 5;
	float NH4_Min = 0;
	float NH4_linear_mV_Split = 20;
	float within = 0.2; // NH4 ppm

	float NH4_linear_mV[3];
	for(i = 0; i < 3; i++)
	{
		float NH4_Cal_1_mV = NH4_EEP_Slope[i] * NH4_EEP_Cal_1 + NH4_EEP_Int[i];
		NH4_linear_mV[i] = NH4_Cal_1_mV + NH4_EEP_Slope[i] * (NH4_EEP_Rinse - NH4_EEP_Cal_1);
	}

	// Choose NH4 sensor
	uint8_t T_Chosen_NH4 = 0;
	if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(NH4_NH3_N_Total[i] > NH4_NH3_N_Total[max_reading])
					max_reading = i;
				if(NH4_NH3_N_Total[i] < NH4_NH3_N_Total[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_NH4 = PS_Chosen_NH4;
			else if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading]) < within || abs_val(NH4_NH3_N_Total[mid_reading] - NH4_NH3_N_Total[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_NH4 || max_reading == PS_Chosen_NH4)
					T_Chosen_NH4 = PS_Chosen_NH4;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(NH4_R[min_reading] - NH4_linear_mV[min_reading]) < abs_val(NH4_R[max_reading] - NH4_linear_mV[max_reading]))
						T_Chosen_NH4 = min_reading;
					else
						T_Chosen_NH4 = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.2 NH4 ppm, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV, or reading outside our range
				{
					if(abs_val(NH4_R[i] - NH4_linear_mV[i]) > NH4_linear_mV_Split)
						NH4_Cal_Status[i] = 0;
					if(NH4_NH3_N_Total[i] > NH4_Max || NH4_NH3_N_Total[i] < NH4_Min)
						NH4_Cal_Status[i] = 0;
				}

				if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					if(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading] < NH4_NH3_N_Total[mid_reading] - NH4_NH3_N_Total[min_reading])
					{
						if(abs_val(NH4_R[max_reading] - NH4_linear_mV[max_reading]) < abs_val(NH4_R[mid_reading] - NH4_linear_mV[mid_reading]))
							T_Chosen_NH4 = max_reading;
						else
							T_Chosen_NH4 = mid_reading;
					}
					else
					{
						if(abs_val(NH4_R[min_reading] - NH4_linear_mV[min_reading]) < abs_val(NH4_R[mid_reading] - NH4_linear_mV[mid_reading]))
							T_Chosen_NH4 = min_reading;
						else
							T_Chosen_NH4 = mid_reading;
					}
				}
				else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed calibration
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(NH4_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(NH4_NH3_N_Total[spot_1] - NH4_NH3_N_Total[spot_2]) < within)	// NH4 sensors are close to each other
					{
						if(PS_Chosen_NH4 == spot_1 || PS_Chosen_NH4 == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_NH4 = PS_Chosen_NH4;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]))
								T_Chosen_NH4 = spot_1;
							else
								T_Chosen_NH4 = spot_2;
						}
					}
					else	// NH4 sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]))
							T_Chosen_NH4 = spot_1;
						else
							T_Chosen_NH4 = spot_2;
					}

				}
				else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(NH4_Cal_Status[0] == 1)
						T_Chosen_NH4 = 0;
					else if(NH4_Cal_Status[1] == 1)
						T_Chosen_NH4 = 1;
					else
						T_Chosen_NH4 = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_NH4 = PS_Chosen_NH4;
				}
			}
		}
		else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(NH4_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(NH4_NH3_N_Total[spot_1] - NH4_NH3_N_Total[spot_2]) < within)	// NH4 sensors are close to each other
			{
				T_Chosen_NH4 = PS_Chosen_NH4;
			}
			else	// NH4 sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) > NH4_linear_mV_Split && abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]) < NH4_linear_mV_Split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_NH4 = spot_2;
				else if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < NH4_linear_mV_Split && abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]) > NH4_linear_mV_Split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_NH4 = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((NH4_NH3_N_Total[spot_1] > NH4_Max || NH4_NH3_N_Total[spot_1] < NH4_Min) && (NH4_NH3_N_Total[spot_2] <= NH4_Max && NH4_NH3_N_Total[spot_2] >= NH4_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_NH4 = spot_2;
					else if((NH4_NH3_N_Total[spot_1] <= NH4_Max && NH4_NH3_N_Total[spot_1] >= NH4_Min) && (NH4_NH3_N_Total[spot_2] > NH4_Max || NH4_NH3_N_Total[spot_2] < NH4_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_NH4 = spot_1;
					else	// both NH4 sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_NH4 = PS_Chosen_NH4;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(NH4_Cal_Status[1] == 1)
			T_Chosen_NH4 = 1;
		else if(NH4_Cal_Status[2] == 1)
			T_Chosen_NH4 = 2;
	}

	return T_Chosen_NH4;
}

////********************************************************************************
//// Pick which Alk sensor to report, put in function so I can update in one place
//// and it updates for all version of code
//// Created: 11/24/2020
//// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
////			Alk_Samp; pointer to array holding the calculated Alkalinity values for each sensor
////			pH_R; pointer to array holding the Rinse mV for each sensor
////			T_Rinse; Temperature of rinse during this test
////			method; pointer to array holding which method was used to calculate Alkalinity
////			Alk_Slope; pointer to array holding the calculated alkalinity slope for each sensor
//// Outputs: T_Chosen_NH4; the chosen sensor
////********************************************************************************
//uint8_t Choose_Alk_Sensor(uint16_t Cal_Number, float * Alk_Samp, float * pH_R, float T_Rinse, uint8_t * method, float * Alk_Slope)
//{
//	uint8_t i;
//	uint16_t Cal_page = Find_Cal_page(Cal_Number);
//	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
//	uint8_t pH_Cal_Status[3] = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
//
//	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
//	uint8_t PS_Chosen_pH = Chosen_Sensors & 0x03;
//	uint8_t PS_Chosen_Alk = PS_Chosen_pH;
//
//	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
//	uint8_t Last_cal_passed[10];
//	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
//	for(i = 0; i < 10; i++)
//		if(Last_cal_passed[i] == 0xFF)
//			Last_cal_passed[i] = Cal_Number;
//
//	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
//	float HCl_N = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_T1_HCL_N, 4));
//
//	float pH_EEP_Slope[3];
//	pH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_SLOPE, 4));
//	pH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_SLOPE, 4));
//	pH_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_SLOPE, 4));
//
//	float pH_EEP_Int[3];
//	pH_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_INT, 4));
//	pH_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_INT, 4));
//	pH_EEP_Int[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_INT, 4));
//
//	float T_EEP_Cal[3];
//	T_EEP_Cal[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_T_CAL, 4));
//	T_EEP_Cal[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_T_CAL, 4));
//	T_EEP_Cal[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_T_CAL, 4));
//
//	// Set up bounds to check against
//	float Alk_Max = 1000;
//	float Alk_Min = 10;
//	float pH_linear_mV_Split = 5;
//	float within = 10; // Alk ppm
//
//	float pH_linear_mV[3];
//	for(i = 0; i < 3; i++)
//	{
//		float pH_Cal_1_mV = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
//		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * ((pH_EEP_Rinse + K_T_pH_Rinse * (T_Rinse - 25)) - (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)));
//	}
//
//	// Choose Alk sensor
//	// Start by excluding any sensors that didn't find a reading
//	for(i = 0; i < 3; i++)
//		if(Alk_Samp[i] == -1)
//		{
//			pH_Cal_Status[i] = 0;
//
//		}
//
//
//	uint8_t T_Chosen_Alk = 0;
//	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
//	{
//		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
//		{
//			float max_reading = Alk_Samp[0];
//			float min_reading = Alk_Samp[0];
//			for(i = 1; i < 3; i++)
//			{
//				if(Alk_Samp[i] > max_reading)
//					max_reading = Alk_Samp[i];
//				if(Alk_Samp[i] < min_reading)
//					min_reading = Alk_Samp[i];
//			}
//			if(abs_val(max_reading - min_reading) < within) // All three points read close to each other, go with chosen from calibration
//				T_Chosen_Alk = PS_Chosen_Alk;
//			else	// The spread between the three points is greater than 10 Alk ppm, check rinse mV
//			{
//				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
//				{
//					if(abs_val(pH_R[i] - pH_linear_mV[i]) > pH_linear_mV_Split)
//						pH_Cal_Status[i] = 0;
//					if(Alk_Samp[i] > Alk_Max || Alk_Samp[i] < Alk_Min)
//						pH_Cal_Status[i] = 0;
//				}
//
//				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
//				{
//					// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
//					if(method[0] == 2 || method[1] == 2 || method[2] == 2)	// First check if any found the endpoint
//					{
//						if(method[0] == 2 && method[1] == 2 && method[2] == 2)	// All three found the endpoint
//						{
//							float Differences[3];
//							Differences[0] = abs_val(Alk_Samp[0] - Alk_Samp[1]);
//							Differences[1] = abs_val(Alk_Samp[0] - Alk_Samp[2]);
//							Differences[2] = abs_val(Alk_Samp[1] - Alk_Samp[2]);
//
//							uint8_t Min_diff = 0;
//							for(i = 1; i < 3; i++)
//								if(Differences[Min_diff] > Differences[i])
//									Min_diff = i;
//
//							if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
//							{
//								if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[1] - pH_linear_mV[1]))
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 1;
//							}
//							else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
//							{
//								if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 2;
//							}
//							else	// Smallest split is between sensors 2 and 3
//							{
//								if(abs_val(pH_R[1] - pH_linear_mV[1]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//									T_Chosen_Alk = 1;
//								else
//									T_Chosen_Alk = 2;
//							}
//						}
//						else if(method[0] == 2 && method[1] == 2)	// Sensors 1 and 2 found the endpoint
//						{
//							if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[1] - pH_linear_mV[1]))
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 1;
//						}
//						else if(method[0] == 2 && method[2] == 2)	// Sensors 1 and 3 found the endpoint
//						{
//							if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 2;
//						}
//						else if(method[1] == 2 && method[2] == 2)	// Sensors 2 and 3 found the endpoint
//						{
//							if(abs_val(pH_R[1] - pH_linear_mV[1]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//								T_Chosen_Alk = 1;
//							else
//								T_Chosen_Alk = 2;
//						}
//						else if(method[0] == 2)	// Only sensor 1 found the endpoint
//							T_Chosen_Alk = 0;
//						else if(method[1] == 2)	// Only sensor 2 found the endpoint
//							T_Chosen_Alk = 1;
//						else if(method[2] == 2)	// Only sensor 3 found the endpoint
//							T_Chosen_Alk = 2;
//					}
//					else if(method[0] == 3 || method[1] == 3 || method[2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
//					{
//						if(method[0] == 3 && method[1] == 3 && method[2] == 3)	// All three used method with two points below endpoint
//						{
//							float Differences[3];	// Choose sensor based on which sensor's slope was closest to HCl_N
//							Differences[0] = abs_val(Alk_Slope[0] - HCl_N);
//							Differences[1] = abs_val(Alk_Slope[1] - HCl_N);
//							Differences[2] = abs_val(Alk_Slope[2] - HCl_N);
//
//							uint8_t Min_diff = 0;
//							for(i = 1; i < 3; i++)
//								if(Differences[Min_diff] > Differences[i])
//									Min_diff = i;
//
//							if(Min_diff == 0)	// Slope is closest on sensor 1
//								T_Chosen_Alk = 0;
//							else if(Min_diff == 1)	// Slope is closest on sensor 2
//								T_Chosen_Alk = 1;
//							else	// Slope is closest on sensor 3
//								T_Chosen_Alk = 2;
//						}
//						else if(method[0] == 3 && method[1] == 3)	// Sensors 1 and 2 used method with two points below endpoint
//						{
//							float Differences[2];	// Choose sensor based on which sensor's slope was closest to HCl_N
//							Differences[0] = abs_val(Alk_Slope[0] - HCl_N);
//							Differences[1] = abs_val(Alk_Slope[1] - HCl_N);
//
//							if(Differences[0] < Differences[1])
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 1;
//						}
//						else if(method[0] == 3 && method[2] == 3)	// Sensors 1 and 3 used method with two points below endpoint
//						{
//							float Differences[2];	// Choose sensor based on which sensor's slope was closest to HCl_N
//							Differences[0] = abs_val(Alk_Slope[0] - HCl_N);
//							Differences[1] = abs_val(Alk_Slope[2] - HCl_N);
//
//							if(Differences[0] < Differences[1])
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 2;
//						}
//						else if(method[1] == 3 && method[2] == 3)	// Sensors 2 and 3 used method with two points below endpoint
//						{
//							float Differences[2];	// Choose sensor based on which sensor's slope was closest to HCl_N
//							Differences[0] = abs_val(Alk_Slope[1] - HCl_N);
//							Differences[1] = abs_val(Alk_Slope[2] - HCl_N);
//
//							if(Differences[0] < Differences[1])
//								T_Chosen_Alk = 1;
//							else
//								T_Chosen_Alk = 2;
//						}
//						else if(method[0] == 3)	// Only sensor 1 used method with two points below endpoint
//							T_Chosen_Alk = 0;
//						else if(method[1] == 3)	// Only sensor 2 used method with two points below endpoint
//							T_Chosen_Alk = 1;
//						else if(method[2] == 3)	// Only sensor 3 used method with two points below endpoint
//							T_Chosen_Alk = 2;
//					}
//					else if(method[0] == 1 || method[1] == 1 || method[2] == 1)	// Then check if any sensor used method with two points above endpoint
//					{
//						if(method[0] == 1 && method[1] == 1 && method[2] == 1)	// All three used method with two points above endpoint
//						{
//							float Differences[3];	// Choose sensor based on which sensor's slope was closest to 6.3
//							Differences[0] = abs_val(-Alk_Slope[0] - 6.3);
//							Differences[1] = abs_val(-Alk_Slope[1] - 6.3);
//							Differences[2] = abs_val(-Alk_Slope[2] - 6.3);
//
//							uint8_t Min_diff = 0;
//							for(i = 1; i < 3; i++)
//								if(Differences[Min_diff] > Differences[i])
//									Min_diff = i;
//
//							if(Min_diff == 0)	// Slope is closest on sensor 1
//								T_Chosen_Alk = 0;
//							else if(Min_diff == 1)	// Slope is closest on sensor 2
//								T_Chosen_Alk = 1;
//							else	// Slope is closest on sensor 3
//								T_Chosen_Alk = 2;
//						}
//						else if(method[0] == 1 && method[1] == 1)	// Sensors 1 and 2 used method with two points above endpoint
//						{
//							float Differences[2];	// Choose sensor based on which sensor's slope was closest to 6.3
//							Differences[0] = abs_val(-Alk_Slope[0] - 6.3);
//							Differences[1] = abs_val(-Alk_Slope[1] - 6.3);
//
//							if(Differences[0] < Differences[1])
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 1;
//						}
//						else if(method[0] == 1 && method[2] == 1)	// Sensors 1 and 3 used method with two points above endpoint
//						{
//							float Differences[2];	// Choose sensor based on which sensor's slope was closest to 6.3
//							Differences[0] = abs_val(-Alk_Slope[0] - 6.3);
//							Differences[1] = abs_val(-Alk_Slope[2] - 6.3);
//
//							if(Differences[0] < Differences[1])
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 2;
//						}
//						else if(method[1] == 1 && method[2] == 1)	// Sensors 2 and 3 used method with two points above endpoint
//						{
//							float Differences[2];	// Choose sensor based on which sensor's slope was closest to 6.3
//							Differences[0] = abs_val(-Alk_Slope[1] - 6.3);
//							Differences[1] = abs_val(-Alk_Slope[2] - 6.3);
//
//							if(Differences[0] < Differences[1])
//								T_Chosen_Alk = 1;
//							else
//								T_Chosen_Alk = 2;
//						}
//						else if(method[0] == 1)	// Only sensor 1 used method with two points above endpoint
//							T_Chosen_Alk = 0;
//						else if(method[1] == 1)	// Only sensor 2 used method with two points above endpoint
//							T_Chosen_Alk = 1;
//						else if(method[2] == 1)	// Only sensor 3 used method with two points above endpoint
//							T_Chosen_Alk = 2;
//					}
//					else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
//					{
//						float Differences[3];
//						Differences[0] = abs_val(Alk_Samp[0] - Alk_Samp[1]);
//						Differences[1] = abs_val(Alk_Samp[0] - Alk_Samp[2]);
//						Differences[2] = abs_val(Alk_Samp[1] - Alk_Samp[2]);
//
//						uint8_t Min_diff = 0;
//						for(i = 1; i < 3; i++)
//							if(Differences[Min_diff] > Differences[i])
//								Min_diff = i;
//
//						if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
//						{
//							if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[1] - pH_linear_mV[1]))
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 1;
//						}
//						else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
//						{
//							if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 2;
//						}
//						else	// Smallest split is between sensors 2 and 3
//						{
//							if(abs_val(pH_R[1] - pH_linear_mV[1]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//								T_Chosen_Alk = 1;
//							else
//								T_Chosen_Alk = 2;
//						}
//					}
//				}
//				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
//				{
//					if(pH_Cal_Status[0] && pH_Cal_Status[1]) // Sensors 1 and 2 are the ones that passed
//					{
//						// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
//						if(method[0] == 2 || method[1] == 2)	// First check if any found the endpoint
//						{
//							if(method[0] == 2 && method[1] == 2)	// Sensors 1 and 2 found the endpoint
//							{
//								if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[1] - pH_linear_mV[1]))
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 1;
//							}
//							else if(method[0] == 2)	// Only sensor 1 found the endpoint
//								T_Chosen_Alk = 0;
//							else if(method[1] == 2)	// Only sensor 2 found the endpoint
//								T_Chosen_Alk = 1;
//						}
//						else if(method[0] == 3 || method[1] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
//						{
//							if(method[0] == 3 && method[1] == 3)	// Sensors 1 and 2 used method with two points below endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to HCl_N
//								Differences[0] = abs_val(Alk_Slope[0] - HCl_N);
//								Differences[1] = abs_val(Alk_Slope[1] - HCl_N);
//
//								if(Differences[0] < Differences[1])
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 1;
//							}
//							else if(method[0] == 3)	// Only sensor 1 used method with two points below endpoint
//								T_Chosen_Alk = 0;
//							else if(method[1] == 3)	// Only sensor 2 used method with two points below endpoint
//								T_Chosen_Alk = 1;
//						}
//						else if(method[0] == 1 || method[1] == 1)	// Then check if any sensor used method with two points above endpoint
//						{
//							if(method[0] == 1 && method[1] == 1)	// Sensors 1 and 2 used method with two points above endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to 6.3
//								Differences[0] = abs_val(-Alk_Slope[0] - 6.3);
//								Differences[1] = abs_val(-Alk_Slope[1] - 6.3);
//
//								if(Differences[0] < Differences[1])
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 1;
//							}
//							else if(method[0] == 1)	// Only sensor 1 used method with two points above endpoint
//								T_Chosen_Alk = 0;
//							else if(method[1] == 1)	// Only sensor 2 used method with two points above endpoint
//								T_Chosen_Alk = 1;
//						}
//						else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
//						{
//							if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[1] - pH_linear_mV[1]))
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 1;
//						}
//					}
//					else if(pH_Cal_Status[0] && pH_Cal_Status[2])	// Sensors 1 and 3 are the ones that passed
//					{
//						// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
//						if(method[0] == 2 || method[2] == 2)	// First check if any found the endpoint
//						{
//							if(method[0] == 2 && method[2] == 2)	// Sensors 1 and 2 found the endpoint
//							{
//								if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 2;
//							}
//							else if(method[0] == 2)	// Only sensor 1 found the endpoint
//								T_Chosen_Alk = 0;
//							else if(method[2] == 2)	// Only sensor 2 found the endpoint
//								T_Chosen_Alk = 2;
//						}
//						else if(method[0] == 3 || method[2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
//						{
//							if(method[0] == 3 && method[2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to HCl_N
//								Differences[0] = abs_val(Alk_Slope[0] - HCl_N);
//								Differences[1] = abs_val(Alk_Slope[2] - HCl_N);
//
//								if(Differences[0] < Differences[1])
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 2;
//							}
//							else if(method[0] == 3)	// Only sensor 1 used method with two points below endpoint
//								T_Chosen_Alk = 0;
//							else if(method[2] == 3)	// Only sensor 2 used method with two points below endpoint
//								T_Chosen_Alk = 2;
//						}
//						else if(method[0] == 1 || method[2] == 1)	// Then check if any sensor used method with two points above endpoint
//						{
//							if(method[0] == 1 && method[2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to 6.3
//								Differences[0] = abs_val(-Alk_Slope[0] - 6.3);
//								Differences[1] = abs_val(-Alk_Slope[2] - 6.3);
//
//								if(Differences[0] < Differences[1])
//									T_Chosen_Alk = 0;
//								else
//									T_Chosen_Alk = 2;
//							}
//							else if(method[0] == 1)	// Only sensor 1 used method with two points above endpoint
//								T_Chosen_Alk = 0;
//							else if(method[2] == 1)	// Only sensor 2 used method with two points above endpoint
//								T_Chosen_Alk = 2;
//						}
//						else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
//						{
//							if(abs_val(pH_R[0] - pH_linear_mV[0]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//								T_Chosen_Alk = 0;
//							else
//								T_Chosen_Alk = 2;
//						}
//					}
//					else	// Sensors 2 and 3 are the ones that passed
//					{
//						// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
//						if(method[1] == 2 || method[2] == 2)	// First check if any found the endpoint
//						{
//							if(method[1] == 2 && method[2] == 2)	// Sensors 1 and 2 found the endpoint
//							{
//								if(abs_val(pH_R[1] - pH_linear_mV[1]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//									T_Chosen_Alk = 1;
//								else
//									T_Chosen_Alk = 2;
//							}
//							else if(method[1] == 2)	// Only sensor 1 found the endpoint
//								T_Chosen_Alk = 1;
//							else if(method[2] == 2)	// Only sensor 2 found the endpoint
//								T_Chosen_Alk = 2;
//						}
//						else if(method[1] == 3 || method[2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
//						{
//							if(method[1] == 3 && method[2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to HCl_N
//								Differences[0] = abs_val(Alk_Slope[1] - HCl_N);
//								Differences[1] = abs_val(Alk_Slope[2] - HCl_N);
//
//								if(Differences[0] < Differences[1])
//									T_Chosen_Alk = 1;
//								else
//									T_Chosen_Alk = 2;
//							}
//							else if(method[1] == 3)	// Only sensor 1 used method with two points below endpoint
//								T_Chosen_Alk = 1;
//							else if(method[2] == 3)	// Only sensor 2 used method with two points below endpoint
//								T_Chosen_Alk = 2;
//						}
//						else if(method[1] == 1 || method[2] == 1)	// Then check if any sensor used method with two points above endpoint
//						{
//							if(method[1] == 1 && method[2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to 6.3
//								Differences[0] = abs_val(-Alk_Slope[1] - 6.3);
//								Differences[1] = abs_val(-Alk_Slope[2] - 6.3);
//
//								if(Differences[0] < Differences[1])
//									T_Chosen_Alk = 1;
//								else
//									T_Chosen_Alk = 2;
//							}
//							else if(method[1] == 1)	// Only sensor 1 used method with two points above endpoint
//								T_Chosen_Alk = 1;
//							else if(method[2] == 1)	// Only sensor 2 used method with two points above endpoint
//								T_Chosen_Alk = 2;
//						}
//						else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
//						{
//							if(abs_val(pH_R[1] - pH_linear_mV[1]) < abs_val(pH_R[2] - pH_linear_mV[2]))
//								T_Chosen_Alk = 1;
//							else
//								T_Chosen_Alk = 2;
//						}
//					}
//				}
//				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
//				{
//					if(pH_Cal_Status[0] == 1)
//						T_Chosen_Alk = 0;
//					else if(pH_Cal_Status[1] == 1)
//						T_Chosen_Alk = 1;
//					else
//						T_Chosen_Alk = 2;
//				}
//				else	// All sensors failed the rinse mV check
//				{
//					T_Chosen_Alk = PS_Chosen_Alk;
//				}
//			}
//		}
//		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
//		{
//			// Pick out the two spots that passed calibration
//			uint8_t spot_1 = 0xFF;
//			uint8_t spot_2 = 0xFF;
//			for(i = 0; i < 3; i++)
//				if(pH_Cal_Status[i] == 1)
//					if(spot_1 == 0xFF)
//						spot_1 = i;
//					else
//						spot_2 = i;
//
//			if(abs_val(Alk_Samp[spot_1] - Alk_Samp[spot_2]) < within)	// Alk sensors are close to each other
//			{
//				T_Chosen_Alk = PS_Chosen_Alk;
//			}
//			else	// Alk sensors did not read close to each other, check if rinse value is close to what it should be
//			{
//				if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) > pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) < pH_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
//					T_Chosen_Alk = spot_2;
//				else if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) > pH_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
//					T_Chosen_Alk = spot_1;
//				else	// If both sensors read within 20 mV of their theoretical rinse value
//				{
//					if((Alk_Samp[spot_1] > Alk_Max || Alk_Samp[spot_1] < Alk_Min) && (Alk_Samp[spot_2] <= Alk_Max && Alk_Samp[spot_2] >= Alk_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
//						T_Chosen_Alk = spot_2;
//					else if((Alk_Samp[spot_1] <= Alk_Max && Alk_Samp[spot_1] >= Alk_Min) && (Alk_Samp[spot_2] > Alk_Max || Alk_Samp[spot_2] < Alk_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
//						T_Chosen_Alk = spot_1;
//					else	// both Alk sensors read within device range then choose based off method and slope
//					{
//						// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
//						if(method[spot_1] == 2 || method[spot_2] == 2)	// First check if any found the endpoint
//						{
//							if(method[spot_1] == 2 && method[spot_2] == 2)	// Sensors 1 and 2 found the endpoint
//							{
//								if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
//									T_Chosen_Alk = spot_1;
//								else
//									T_Chosen_Alk = spot_2;
//							}
//							else if(method[spot_1] == 2)	// Only sensor 1 found the endpoint
//								T_Chosen_Alk = spot_1;
//							else if(method[spot_2] == 2)	// Only sensor 2 found the endpoint
//								T_Chosen_Alk = spot_2;
//						}
//						else if(method[spot_1] == 3 || method[spot_2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
//						{
//							if(method[spot_1] == 3 && method[spot_2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to HCl_N
//								Differences[0] = abs_val(Alk_Slope[spot_1] - HCl_N);
//								Differences[1] = abs_val(Alk_Slope[spot_2] - HCl_N);
//
//								if(Differences[spot_1] < Differences[spot_2])
//									T_Chosen_Alk = spot_1;
//								else
//									T_Chosen_Alk = spot_2;
//							}
//							else if(method[spot_1] == 3)	// Only sensor 1 used method with two points below endpoint
//								T_Chosen_Alk = spot_1;
//							else if(method[spot_2] == 3)	// Only sensor 2 used method with two points below endpoint
//								T_Chosen_Alk = spot_2;
//						}
//						else if(method[spot_1] == 1 || method[spot_2] == 1)	// Then check if any sensor used method with two points above endpoint
//						{
//							if(method[spot_1] == 1 && method[spot_2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
//							{
//								float Differences[2];	// Choose sensor based on which sensor's slope was closest to 6.3
//								Differences[0] = abs_val(-Alk_Slope[spot_1] - 6.3);
//								Differences[1] = abs_val(-Alk_Slope[spot_2] - 6.3);
//
//								if(Differences[spot_1] < Differences[spot_2])
//									T_Chosen_Alk = spot_1;
//								else
//									T_Chosen_Alk = spot_2;
//							}
//							else if(method[spot_1] == 1)	// Only sensor 1 used method with two points above endpoint
//								T_Chosen_Alk = spot_1;
//							else if(method[spot_2] == 1)	// Only sensor 2 used method with two points above endpoint
//								T_Chosen_Alk = spot_2;
//						}
//						else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
//						{
//							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
//								T_Chosen_Alk = spot_1;
//							else
//								T_Chosen_Alk = spot_2;
//						}
//					}
//				}
//			}
//		}
//	}
//	else	// If only one or no sensors passed
//	{
//		if(pH_Cal_Status[1] == 1)
//			T_Chosen_Alk = 1;
//		else if(pH_Cal_Status[2] == 1)
//			T_Chosen_Alk = 2;
//	}
//
//	return T_Chosen_Alk;
//}

//********************************************************************************
// Pick which Alk sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			Alk_Samp; pointer to array holding the calculated Alkalinity values for each sensor
//			pH_R; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
//			method; pointer to array holding which method was used to calculate Alkalinity
//			Alk_Slope; pointer to array holding the calculated alkalinity slope for each sensor
// Outputs: T_Chosen_NH4; the chosen sensor
//********************************************************************************
uint8_t Choose_Alk_Sensor(uint16_t Cal_Number, float * Alk_Samp, float * pH_R, float T_Rinse, uint8_t * method, float * Alk_Slope)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[3] = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_pH = Chosen_Sensors & 0x03;
	uint8_t PS_Chosen_Alk = PS_Chosen_pH;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
	float HCl_N = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_T1_HCL_N, 4));

	float pH_EEP_Slope[3];
	pH_EEP_Slope[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_SLOPE, 4));
	pH_EEP_Slope[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_SLOPE, 4));
	pH_EEP_Slope[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_SLOPE, 4));

	float pH_EEP_Int[3];
	pH_EEP_Int[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_PH_1_INT, 4));
	pH_EEP_Int[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_PH_2_INT, 4));
	pH_EEP_Int[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_PH_3_INT, 4));

	float T_EEP_Cal[3];
	T_EEP_Cal[0] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[0]), OFFSET_T_CAL, 4));
	T_EEP_Cal[1] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[1]), OFFSET_T_CAL, 4));
	T_EEP_Cal[2] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[2]), OFFSET_T_CAL, 4));

	// Set up bounds to check against
	float Alk_Max = 1000;
	float Alk_Min = 10;
	float pH_linear_mV_Split = 10;
	float within = 10; // Alk ppm

	float pH_linear_mV[3];
	for(i = 0; i < 3; i++)
	{
		float pH_Cal_1_mV = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * ((pH_EEP_Rinse + K_T_pH_Rinse * (T_Rinse - 25)) - (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)));
	}

	// Choose Alk sensor
	// Start by excluding any sensors that didn't find a reading
	for(i = 0; i < 3; i++)
		if(Alk_Samp[i] == -1)
			pH_Cal_Status[i] = 0;
	// Next check that I didn't exclude the calibration chosen sensor
	if(Alk_Samp[PS_Chosen_Alk] == -1)
	{
		uint8_t passed_sensors = pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2];

		if(passed_sensors == 2)
		{
			// Pick out the two spots that found the alkalinity
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(method[i] == 2)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(pH_R[spot_1] - pH_linear_mV[spot_1] < pH_R[spot_2] - pH_linear_mV[spot_2])
				PS_Chosen_Alk = spot_1;
			else
				PS_Chosen_Alk = spot_2;
		}
		else if(passed_sensors == 1)
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					PS_Chosen_Alk = i;
	}

	uint8_t T_Chosen_Alk = 0;
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find Min and Max readings index
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(Alk_Samp[i] > max_reading)
					max_reading = i;
				if(Alk_Samp[i] < min_reading)
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(Alk_Samp[max_reading] - Alk_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_Alk = PS_Chosen_Alk;
			else if(abs_val(Alk_Samp[max_reading] - Alk_Samp[mid_reading]) < within || abs_val(Alk_Samp[mid_reading] - Alk_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(Alk_Samp[max_reading] - Alk_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_Alk || max_reading == PS_Chosen_Alk)
					T_Chosen_Alk = PS_Chosen_Alk;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_R[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_R[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_Alk = min_reading;
					else
						T_Chosen_Alk = max_reading;
				}
			}
			else	// The spread between the three points is greater than 10 Alk ppm, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_R[i] - pH_linear_mV[i]) > pH_linear_mV_Split)
						pH_Cal_Status[i] = 0;
					if(Alk_Samp[i] > Alk_Max || Alk_Samp[i] < Alk_Min)
						pH_Cal_Status[i] = 0;
				}

				// All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3)
				{
					// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
					if(method[0] == 2 || method[1] == 2 || method[2] == 2)	// First check if any found the endpoint
					{
						// Count up how many sensors found endpoint
						uint8_t sensors_at_endpoint = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 2)
								sensors_at_endpoint++;

						if(sensors_at_endpoint == 3)	// All three sensors found the endpoint
						{
							// Pick out of the two sensors that were closest together, then choose the one thats rinse was closest to its linear mV value
							if(Alk_Samp[max_reading] - Alk_Samp[mid_reading] < Alk_Samp[mid_reading] - Alk_Samp[min_reading])	// Smallest split is between mid and max reading
							{
								if(abs_val(pH_R[max_reading] - pH_linear_mV[max_reading]) < abs_val(pH_R[mid_reading] - pH_linear_mV[mid_reading]))
									T_Chosen_Alk = max_reading;
								else
									T_Chosen_Alk = mid_reading;
							}
							else
							{
								if(abs_val(pH_R[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_R[mid_reading] - pH_linear_mV[mid_reading]))
									T_Chosen_Alk = min_reading;
								else
									T_Chosen_Alk = mid_reading;
							}
						}
						else if(sensors_at_endpoint == 2)	// Only 2 sensors found the endpoint
						{
							// Pick out the two spots that found the endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 2)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// Only 1 sensor found the endpoint
						{
							for(i = 0; i < 3; i++)	// Iterate through sensors to find the one that found the endpoint
								if(method[i] == 2)
									T_Chosen_Alk = i;
						}
					}
					// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
					else if(method[0] == 3 || method[1] == 3 || method[2] == 3)
					{
						// Count up how many sensors used method with two points below endpoint
						uint8_t sensors_using_method = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 3)
								sensors_using_method++;

						if(sensors_using_method == 3)	// All three used method with two points below endpoint
						{
							for(i = 0; i < 3; i++)	// Iterate through each sensor
								if(abs_val(Alk_Slope[i] - HCl_N) < abs_val(Alk_Slope[T_Chosen_Alk] - HCl_N))	// If current slope is closer to HCl_N value then chosen sensor, set current sensor as chosen
									T_Chosen_Alk = i;
						}
						else if(sensors_using_method == 2)	// 2 sensors used method with two points below endpoint
						{
							// Pick out the two spots that used method with two points below endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 3)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(Alk_Slope[spot_1] - HCl_N) < abs_val(Alk_Slope[spot_2] - HCl_N))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// Only 1 sensor used method with two points below endpoint
						{
							for(i = 0; i < 3; i++)
								if(method[i] == 3)
									T_Chosen_Alk = i;
						}
					}
					// Then check if any sensor used method with two points above endpoint
					else if(method[0] == 1 || method[1] == 1 || method[2] == 1)
					{
						// Count up how many sensors used method with two points above endpoint
						uint8_t sensors_using_method = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 1)
								sensors_using_method++;

						if(sensors_using_method == 3)	// All three sensors used method with two points above endpoint
						{
							for(i = 0; i < 3; i++)
								if(abs_val(-Alk_Slope[i] - 6.3) < abs_val(-Alk_Slope[T_Chosen_Alk] - 6.3))
									T_Chosen_Alk = i;
						}
						else if(sensors_using_method == 2)	// 2 sensors used method with two points above endpoint
						{
							// Pick out the two spots that used method with two points above endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 1)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// 1 sensor1 used method with two points above endpoint
						{
							for(i = 0; i < 3; i++)
								if(method[i] == 1)
									T_Chosen_Alk = i;
						}
					}
					// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
					else
					{
						for(i = 0; i < 3; i++)
							if(method[i] == 4)	// Make sure were only changing to sensors that used the interpolation method and not a sensor that didnt find mixing range
								if(abs_val(pH_R[i] - pH_linear_mV[i]) < abs_val(pH_R[T_Chosen_Alk] - pH_linear_mV[T_Chosen_Alk]))
									T_Chosen_Alk = i;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that used method with two points above endpoint
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
					if(method[spot_1] == 2 || method[spot_2] == 2)	// First check if any found the endpoint
					{
						if(method[spot_1] == 2 && method[spot_2] == 2)	// Both sensors found the endpoint
						{
							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 2)	// Only spot 1 found the endpoint
							T_Chosen_Alk = spot_1;
						else	// Only spot 2 found the endpoint
							T_Chosen_Alk = spot_2;
					}
					else if(method[spot_1] == 3 || method[spot_2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
					{
						if(method[spot_1] == 3 && method[spot_2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
						{
							if(abs_val(Alk_Slope[spot_1] - HCl_N) < abs_val(Alk_Slope[spot_2] - HCl_N))	// Choose the sensors whose slope is closest to HCl_N value
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 3)	// Only sensor 1 used method with two points below endpoint
							T_Chosen_Alk = spot_1;
						else	// Only sensor 2 used method with two points below endpoint
							T_Chosen_Alk = spot_2;
					}
					else if(method[spot_1] == 1 || method[spot_2] == 1)	// Then check if any sensor used method with two points above endpoint
					{
						if(method[spot_1] == 1 && method[spot_2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
						{
							if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 1)	// Only sensor 1 used method with two points above endpoint
							T_Chosen_Alk = spot_1;
						else	// Only sensor 2 used method with two points above endpoint
							T_Chosen_Alk = spot_2;
					}
					else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
					{
						if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
							T_Chosen_Alk = spot_1;
						else
							T_Chosen_Alk = spot_2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							T_Chosen_Alk = i;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_Alk = PS_Chosen_Alk;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(Alk_Samp[spot_1] - Alk_Samp[spot_2]) < within)	// Alk sensors are close to each other
			{
				T_Chosen_Alk = PS_Chosen_Alk;
			}
			else	// Alk sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) > pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) < pH_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
					T_Chosen_Alk = spot_2;
				else if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) > pH_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
					T_Chosen_Alk = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((Alk_Samp[spot_1] > Alk_Max || Alk_Samp[spot_1] < Alk_Min) && (Alk_Samp[spot_2] <= Alk_Max && Alk_Samp[spot_2] >= Alk_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_Alk = spot_2;
					else if((Alk_Samp[spot_1] <= Alk_Max && Alk_Samp[spot_1] >= Alk_Min) && (Alk_Samp[spot_2] > Alk_Max || Alk_Samp[spot_2] < Alk_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_Alk = spot_1;
					else	// both Alk sensors read within device range then choose based off method and slope
					{
						// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
						if(method[spot_1] == 2 || method[spot_2] == 2)	// First check if any found the endpoint
						{
							if(method[spot_1] == 2 && method[spot_2] == 2)	// Sensors 1 and 2 found the endpoint
							{
								if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 2)	// Only spot 1 found the endpoint
								T_Chosen_Alk = spot_1;
							else	// Only spot 2 found the endpoint
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 3 || method[spot_2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
						{
							if(method[spot_1] == 3 && method[spot_2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
							{
								if(abs_val(Alk_Slope[spot_1] - HCl_N) < abs_val(Alk_Slope[spot_2] - HCl_N))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 3)	// Only spot 1 used method with two points below endpoint
								T_Chosen_Alk = spot_1;
							else	// Only spot 2 used method with two points below endpoint
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 1 || method[spot_2] == 1)	// Then check if any sensor used method with two points above endpoint
						{
							if(method[spot_1] == 1 && method[spot_2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
							{
								if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 1)	// Only sensor 1 used method with two points above endpoint
								T_Chosen_Alk = spot_1;
							else	// Only sensor 2 used method with two points above endpoint
								T_Chosen_Alk = spot_2;
						}
						else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
						{
							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
					}
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_Alk = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_Alk = 2;
	}

	return T_Chosen_Alk;
}
#else
#ifdef SOLUTION_IN_STRUCT
//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs, struct SolutionVals *Sols)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[3] = {0,0,0};
	for(i = 0; i < (ISEs.pH_Cr.size > 3 ? 3 : ISEs.pH_Cr.size); i++)
		pH_Cal_Status[i] = (Cal_Status >> (ISEs.pH_Cr.index + 1 + i)) & 1;

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_pH = ISEs.pH_Cr.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.pH_Cr.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_Cr.size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

//	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));

	float pH_EEP_Slope[3] = {0,0,0};
	float pH_EEP_Int[3] = {0,0,0};
//	float T_EEP_Cal[3];
	for(i = 0; i < (ISEs.pH_Cr.size > 3 ? 3 : ISEs.pH_Cr.size); i++)
	{
		pH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_Cr.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.pH_Cr.index) * 4), 4));
		pH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_Cr.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.pH_Cr.index) * 4), 4));
//		T_EEP_Cal[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_Cr.index + i]), OFFSET_T_CAL, 4));
	}

	// Set up limits to check against
	float pH_Max = 11;
	float pH_Min = 5;
	float pH_linear_split = 10;	// mV to compare test prerinse to cal prerinse
	float pH_linear_mV[3];
//	for(i = 0; i < ISEs.pH_Cr.size; i++)
//	{
//		float pH_Cal_1_mV = pH_EEP_Slope[i] * (Sols->pH_EEP_Cal_1 + Sols->K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
//		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Rinse, 25, 0, Sols->K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
//		float pH_TCor_Cal_1 = Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_EEP_Cal[i], 25, 0, Sols->K_T_pH_Cal_1);//pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
//		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
//	}

	for(i = 0; i < (ISEs.pH_Cr.size > 3 ? 3 : ISEs.pH_Cr.size); i++)
	{
		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Rinse, 25, 0, Sols->K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
		pH_linear_mV[i] = pH_EEP_Slope[i] * pH_TCor_Rinse + pH_EEP_Int[i];
	}

	// Choose pH sensor
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		float within = 0.05; // pH units
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find min and max reading to see how far apart all three are
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(pH_Samp[i] > pH_Samp[max_reading])
					max_reading = i;
				if(pH_Samp[i] < pH_Samp[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(pH_Samp[max_reading] - pH_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_pH = PS_Chosen_pH;
			else if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within || abs_val(pH_Samp[mid_reading] - pH_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_pH || max_reading == PS_Chosen_pH)
					T_Chosen_pH = PS_Chosen_pH;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_E_Rinse[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_E_Rinse[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_pH = min_reading;
					else
						T_Chosen_pH = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.1 pH units, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_E_Rinse[i] - pH_linear_mV[i]) > pH_linear_split)
						pH_Cal_Status[i] = 0;
					if(pH_Samp[i] > pH_Max || pH_Samp[i] < pH_Min)
						pH_Cal_Status[i] = 0;
				}

				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					float Differences[3];
					Differences[0] = abs_val(pH_Samp[0] - pH_Samp[1]);
					Differences[1] = abs_val(pH_Samp[0] - pH_Samp[2]);
					Differences[2] = abs_val(pH_Samp[1] - pH_Samp[2]);

					uint8_t Min_diff = 0;
					for(i = 1; i < 3; i++)
						if(Differences[Min_diff] > Differences[i])
							Min_diff = i;

					if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
					else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 2;
					}
					else	// Smallest split is between sensors 2 and 3
					{
						if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 1;
						else
							T_Chosen_pH = 2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed rinse mV check
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
					{
						if(PS_Chosen_pH == spot_1 || PS_Chosen_pH == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_pH = PS_Chosen_pH;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_pH = spot_1;
							else
								T_Chosen_pH = spot_2;
						}
					}
					else	// pH sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(pH_Cal_Status[0] == 1)
						T_Chosen_pH = 0;
					else if(pH_Cal_Status[1] == 1)
						T_Chosen_pH = 1;
					else
						T_Chosen_pH = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
			{
				T_Chosen_pH = PS_Chosen_pH;
			}
			else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) > pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_pH = spot_2;
				else if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_pH = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((pH_Samp[spot_1] > pH_Max || pH_Samp[spot_1] < pH_Min) && (pH_Samp[spot_2] <= pH_Max && pH_Samp[spot_2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_pH = spot_2;
					else if((pH_Samp[spot_1] <= pH_Max && pH_Samp[spot_1] >= pH_Min) && (pH_Samp[spot_2] > pH_Max || pH_Samp[spot_2] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_pH = spot_1;
					else	// both pH sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_pH = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_pH = 2;
	}

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_H2_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs, struct SolutionVals *Sols)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[3] = {0,0,0};
	for(i = 0; i < ISEs.pH_H2.size; i++)
		pH_Cal_Status[i] = (Cal_Status >> (ISEs.pH_H2.index + 1 + i)) & 1;

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_pH = ISEs.pH_H2.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.pH_H2.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_H2.size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

//	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));

	float pH_EEP_Slope[3] = {0,0,0};
	float pH_EEP_Int[3] = {0,0,0};
//	float T_EEP_Cal[3];
	for(i = 0; i < ISEs.pH_H2.size; i++)
	{
		pH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_H2.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.pH_H2.index) * 4), 4));
		pH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_H2.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.pH_H2.index) * 4), 4));
//		T_EEP_Cal[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_H2.index + i]), OFFSET_T_CAL, 4));
	}

	// Set up limits to check against
	float pH_Max = 11;
	float pH_Min = 2;
	float pH_linear_split = 10;	// mV to compare test prerinse to cal prerinse
	float pH_linear_mV[3];
//	for(i = 0; i < ISEs.pH_H2.size; i++)
//	{
//		float pH_Cal_1_mV = pH_EEP_Slope[i] * (Sols->pH_EEP_Cal_1 + Sols->K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
//		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Rinse, 25, 0, Sols->K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
//		float pH_TCor_Cal_1 = Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_EEP_Cal[i], 25, 0, Sols->K_T_pH_Cal_1);//pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
//		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
//	}

	for(i = 0; i < ISEs.pH_H2.size; i++)
	{
		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Rinse, 25, 0, Sols->K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
		pH_linear_mV[i] = pH_EEP_Slope[i] * pH_TCor_Rinse + pH_EEP_Int[i];
	}

	// Choose pH sensor
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		float within = 0.05; // pH units
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find min and max reading to see how far apart all three are
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(pH_Samp[i] > pH_Samp[max_reading])
					max_reading = i;
				if(pH_Samp[i] < pH_Samp[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(pH_Samp[max_reading] - pH_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_pH = PS_Chosen_pH;
			else if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within || abs_val(pH_Samp[mid_reading] - pH_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_pH || max_reading == PS_Chosen_pH)
					T_Chosen_pH = PS_Chosen_pH;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_E_Rinse[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_E_Rinse[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_pH = min_reading;
					else
						T_Chosen_pH = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.1 pH units, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_E_Rinse[i] - pH_linear_mV[i]) > pH_linear_split)
						pH_Cal_Status[i] = 0;
					if(pH_Samp[i] > pH_Max || pH_Samp[i] < pH_Min)
						pH_Cal_Status[i] = 0;
				}

				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					float Differences[3];
					Differences[0] = abs_val(pH_Samp[0] - pH_Samp[1]);
					Differences[1] = abs_val(pH_Samp[0] - pH_Samp[2]);
					Differences[2] = abs_val(pH_Samp[1] - pH_Samp[2]);

					uint8_t Min_diff = 0;
					for(i = 1; i < 3; i++)
						if(Differences[Min_diff] > Differences[i])
							Min_diff = i;

					if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
					else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 2;
					}
					else	// Smallest split is between sensors 2 and 3
					{
						if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 1;
						else
							T_Chosen_pH = 2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed rinse mV check
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
					{
						if(PS_Chosen_pH == spot_1 || PS_Chosen_pH == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_pH = PS_Chosen_pH;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_pH = spot_1;
							else
								T_Chosen_pH = spot_2;
						}
					}
					else	// pH sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(pH_Cal_Status[0] == 1)
						T_Chosen_pH = 0;
					else if(pH_Cal_Status[1] == 1)
						T_Chosen_pH = 1;
					else
						T_Chosen_pH = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
			{
				T_Chosen_pH = PS_Chosen_pH;
			}
			else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) > pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_pH = spot_2;
				else if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_pH = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((pH_Samp[spot_1] > pH_Max || pH_Samp[spot_1] < pH_Min) && (pH_Samp[spot_2] <= pH_Max && pH_Samp[spot_2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_pH = spot_2;
					else if((pH_Samp[spot_1] <= pH_Max && pH_Samp[spot_1] >= pH_Min) && (pH_Samp[spot_2] > pH_Max || pH_Samp[spot_2] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_pH = spot_1;
					else	// both pH sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_pH = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_pH = 2;
	}

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// 1/15/2021: Set up to work for full pH die,
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_Sensor_pHDie(uint16_t Cal_Number, float * pH_Samp)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
//	uint8_t ui8Size_pH = 3;
//	if(Sensor_Config == PH_CL_CART)
//		ui8Size_pH = 10;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[10] = {0,0,0,0,0,0,0,0,0,0};// = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
	pH_Cal_Status[0] = (Cal_Status >> 1) & 1;
	pH_Cal_Status[1] = (Cal_Status >> 2) & 1;
	pH_Cal_Status[2] = (Cal_Status >> 28) & 1;
	if(Sensor_Config == PH_CL_CART)
	{
		pH_Cal_Status[3] = (Cal_Status >> 5) & 1;
		pH_Cal_Status[4] = (Cal_Status >> 6) & 1;
		pH_Cal_Status[5] = (Cal_Status >> 7) & 1;
		pH_Cal_Status[6] = (Cal_Status >> 8) & 1;
		pH_Cal_Status[7] = (Cal_Status >> 9) & 1;
		pH_Cal_Status[8] = (Cal_Status >> 3) & 1;
		pH_Cal_Status[9] = (Cal_Status >> 4) & 1;
	}

	// Choose pH sensor
	float within = 0.01; // pH units

	// Choose a sensor by trying to find groupings of readings, pick the largest grouping and go with the median

	// Count how many close sensors each sensor has and record the max
	uint8_t CloseSensors[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t MaxClose = 0;
	uint8_t j;
	for(i = 0; i < 10; i++)
	{
		if(pH_Cal_Status[i])
			for(j = 0; j < 10; j++)
				if(abs_val(pH_Samp[i] - pH_Samp[j]) <= within && pH_Cal_Status[j])
					CloseSensors[i]++;

		if(CloseSensors[i] > MaxClose)
			MaxClose = CloseSensors[i];
	}

	// After finding max, find out how many sensors matched this max
	uint8_t MaxClose_Sensors[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t TotalCloseSensors = 0;
	for(i = 0; i < 10; i++)
		if(CloseSensors[i] == MaxClose)
		{
			MaxClose_Sensors[i] = 1;	// Set flag so each sensor that had max sensors close is set to 1 and all others are 0
			TotalCloseSensors++;
		}

	// Create array with index values sorted from lowest to highest pH
	uint8_t SortedSensors[10] = {0,0,0,0,0,0,0,0,0,0};
//	uint8_t CurrentIndex = 0;
	for(i = 0; i < TotalCloseSensors; i++)
	{
		uint8_t MinIndex = 0;

		for(j = 0; j < 10; j++)
		{
			if(MaxClose_Sensors[j] == 1)
			{
				MinIndex = j;
				break;
			}
		}

		for(j = 0; j < 10; j++)
		{
			if(MaxClose_Sensors[j] == 1 && pH_Samp[j] < pH_Samp[MinIndex])
				MinIndex = j;
		}

		MaxClose_Sensors[MinIndex] = 0;
		SortedSensors[i] = MinIndex;
	}

	// Set the chosen sensor as the median sensor from the tight group
	if(TotalCloseSensors % 2 == 0)	// If an even number, divide by 2 and subtract 1 to find the index to return
		T_Chosen_pH = SortedSensors[TotalCloseSensors / 2 - 1];
	else
		T_Chosen_pH = SortedSensors[TotalCloseSensors / 2];

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which Ca sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			Ca_Hardness; pointer to array holding the calculated Calcium Hardness values for each sensor
//			Ca_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_Ca; the chosen sensor
//********************************************************************************
uint8_t Choose_Ca_Sensor(uint16_t Cal_Number, float * Ca_Hardness, float * Ca_R, struct ISEConfig ISEs, struct SolutionVals *Sols)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t Ca_Cal_Status[2] = {(Cal_Status >> (ISEs.Ca.index + 1)) & 1, (Cal_Status >> (ISEs.Ca.index + 2)) & 1};

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_Ca = (Chosen_Sensors >> ISEs.Ca.StorBit) & 0x01;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

//	float Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
//	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
//	float TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));

	// Check if values in the memory are p-values or concentrations
	float pCa_Rinse;
//	float pCa_Cal_1;
//	if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
//	{
//		pCa_Rinse = Sols->Ca_EEP_Rinse;
//		pCa_Cal_1 = Sols->Ca_EEP_Cal_1;
//	}
//	else	// Values are concentrations
//	{
		pCa_Rinse = Calc_pCa(Sols->Ca_EEP_Rinse, T_Cal, Sols->IS_RINSE);
//		pCa_Cal_1 = Calc_pCa(Ca_EEP_Cal_1, T_Cal, IS_CAL_1);
//		pCa_Cal_1 = Calc_pCa(Sols->Ca_EEP_Cal_1, T_Cal, Sols->IS_CAL_1);
//		pCa_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE));
//		pCa_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1));
//	}

	float Ca_EEP_Slope[2];
	float Ca_EEP_Int[2];
	for(i = 0; i < ISEs.Ca.size; i++)
	{
		Ca_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_ISE_1_SLOPE + ((i + ISEs.Ca.index) * 4), 4));
		Ca_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4));
	}

	// Set up bounds to check against
	float Ca_Hardness_Max = 1250;
	float Ca_Hardness_Min = 2;
	float Ca_linear_mV_Split = 5;
	float within = 10; // ppm Ca Hardness

	float Ca_linear_mV[2];
//	for(i = 0; i < 2; i++)
//	{
//		float Ca_Cal_1_mV = Ca_EEP_Slope[i] * pCa_Cal_1 + Ca_EEP_Int[i];
//		Ca_linear_mV[i] = Ca_Cal_1_mV + Ca_EEP_Slope[i] * (pCa_Rinse - pCa_Cal_1);
//	}

	for(i = 0; i < 2; i++)
	{
//		float Ca_Cal_1_mV = Ca_EEP_Slope[i] * pCa_Cal_1 + Ca_EEP_Int[i];
		Ca_linear_mV[i] = Ca_EEP_Slope[i] * pCa_Rinse + Ca_EEP_Int[i];
	}

	// Choose Ca sensor
	uint8_t T_Chosen_Ca = 0;
	if((Ca_Cal_Status[0] + Ca_Cal_Status[1]) > 1)	// Check that both sensors passed
	{
		if(abs_val(Ca_Hardness[0] - Ca_Hardness[1]) < within)	// Ca sensors are close to each other
		{
			T_Chosen_Ca = PS_Chosen_Ca;
		}
		else	// Ca sensors did not read close to each other, check if rinse value is close to what it should be
		{
			if(abs_val(Ca_R[0] - Ca_linear_mV[0]) > Ca_linear_mV_Split && abs_val(Ca_R[1] - Ca_linear_mV[1]) < Ca_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
				T_Chosen_Ca = 1;
			else if(abs_val(Ca_R[0] - Ca_linear_mV[0]) < Ca_linear_mV_Split && abs_val(Ca_R[1] - Ca_linear_mV[1]) > Ca_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
				T_Chosen_Ca = 0;
			else	// If both sensors read within 5 mV of their theoretical rinse value or they both read further away
			{
				if((Ca_Hardness[0] > Ca_Hardness_Max || Ca_Hardness[0] < Ca_Hardness_Min) && (Ca_Hardness[1] <= Ca_Hardness_Max && Ca_Hardness[1] >= Ca_Hardness_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
					T_Chosen_Ca = 1;
				else if((Ca_Hardness[0] <= Ca_Hardness_Max && Ca_Hardness[0] >= Ca_Hardness_Min) && (Ca_Hardness[1] > Ca_Hardness_Max || Ca_Hardness[1] < Ca_Hardness_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
					T_Chosen_Ca = 0;
				else	// both pH sensors read within device range then just go with sensor chosen during calibration
					T_Chosen_Ca = PS_Chosen_Ca;
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(Ca_Cal_Status[1] == 1)
			T_Chosen_Ca = 1;
	}

	return T_Chosen_Ca;
}

//********************************************************************************
// Pick which TH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			TH_corr; pointer to array holding the calculated Total Hardness values for each sensor
//			TH_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_TH; the chosen sensor
//********************************************************************************
uint8_t Choose_TH_Sensor(uint16_t Cal_Number, float * TH_corr, float * TH_R, struct ISEConfig ISEs, struct SolutionVals *Sols)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t TH_Cal_Status[2] = {(Cal_Status >> ISEs.TH.index + 1) & 1, (Cal_Status >> ISEs.TH.index + 2) & 1};

	float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_TH = (Chosen_Sensors >> ISEs.TH.StorBit) & 0x01;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

//	float Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
//	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
//	float TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));
//	float TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_TH, 4));

	float pTH_Rinse;//, pTH_Cal_1;
//	float log_K_Ca_Mg = Build_float(MemoryRead(Cal_page, OFFSET_CAL_LOG_K, 4));
//	if(Sols->Ca_EEP_Cal_1 < 10)	// Values are p-values
//	{
//		float Mg_100Ca_Cal_1 = -log10(pow(10, -Sols->TH_EEP_Cal_1) - pow(10, -Sols->Ca_EEP_Cal_1));
//		float Mg_100Ca_Rinse = -log10(pow(10, -Sols->TH_EEP_Rinse) - pow(10, -Sols->Ca_EEP_Rinse));
//
//		pTH_Cal_1 = -log10(pow(10, -Mg_100Ca_Cal_1) + pow(10, log_K_Ca_Mg) * pow(10, -Sols->Ca_EEP_Cal_1));
//		pTH_Rinse = -log10(pow(10, -Mg_100Ca_Rinse) + pow(10, log_K_Ca_Mg) * pow(10, -Sols->Ca_EEP_Rinse));
//	}
//	else	// Values are concentration
//	{
		pTH_Rinse = Calc_pTH(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, -5, T_Cal, Sols->IS_RINSE);
//		pTH_Cal_1 = Calc_pTH(Ca_EEP_Cal_1, TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, IS_CAL_1);
//		pTH_Cal_1 = Calc_pTH(Sols->Ca_EEP_Cal_1, Sols->TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, Sols->IS_CAL_1);
//		pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_Cal, IS_RINSE));
//		pTH_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_1 - Ca_EEP_Cal_1) / 100090 * Lambda_Mg(T_Cal, IS_CAL_1));
//	}

	float TH_EEP_Slope[2];
	float TH_EEP_Int[2];
	for(i = 0; i < ISEs.TH.size; i++)
	{
		TH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.TH.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.TH.index) * 4), 4));
		TH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.TH.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.TH.index) * 4), 4));
	}

	// Set up Bounds to check against
	float TH_Max = 2000;
	float TH_Min = 5;
	float TH_linear_mV_Split = 5;	// Acceptable difference between cal prerinse mV and test prerinse mV
	float within = 10; // ppm Total Hardness

	float TH_linear_mV[2];
//	for(i = 0; i < 2; i++)
//	{
//		float log_K_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.TH.index + i]), OFFSET_CAL_LOG_K, 4));
//		float TH_Cal_1_mV = TH_EEP_Slope[0] * pTH_Cal_1 + TH_EEP_Int[0];
//		TH_linear_mV[i] = TH_Cal_1_mV + TH_EEP_Slope[i] * (pTH_Rinse - pTH_Cal_1);
//	}
	for(i = 0; i < 2; i++)
	{
		TH_linear_mV[i] = TH_EEP_Slope[i] * pTH_Rinse + TH_EEP_Int[0];
	}

	// Choose TH sensor
	uint8_t T_Chosen_TH = 0;
	if((TH_Cal_Status[0] + TH_Cal_Status[1]) > 1)	// Check that both sensors passed
	{
		if(abs_val(TH_corr[0] - TH_corr[1]) < within)	// TH sensors are close to each other
		{
			T_Chosen_TH = PS_Chosen_TH;
		}
		else	// TH sensors did not read close to each other, check if rinse value is close to what it should be
		{
			if(abs_val(TH_R[0] - TH_linear_mV[0]) > TH_linear_mV_Split && abs_val(TH_R[1] - TH_linear_mV[1]) < TH_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
				T_Chosen_TH = 1;
			else if(abs_val(TH_R[0] - TH_linear_mV[0]) < TH_linear_mV_Split && abs_val(TH_R[1] - TH_linear_mV[1]) > TH_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
				T_Chosen_TH = 0;
			else	// If both sensors read within 5 mV of their theoretical rinse value or they both read further away
			{
				if((TH_corr[0] > TH_Max || TH_corr[0] < TH_Min) && (TH_corr[1] <= TH_Max && TH_corr[1] >= TH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
					T_Chosen_TH = 1;
				else if((TH_corr[0] <= TH_Max && TH_corr[0] >= TH_Min) && (TH_corr[1] > TH_Max || TH_corr[1] < TH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
					T_Chosen_TH = 0;
				else	// both pH sensors read within device range then just go with sensor chosen during calibration
					T_Chosen_TH = PS_Chosen_TH;
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(TH_Cal_Status[1] == 1)
			T_Chosen_TH = 1;
	}

	return T_Chosen_TH;
}

//********************************************************************************
// Pick which TH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			NH4_NH3_N_Total; pointer to array holding the calculated NH4 values for each sensor
//			NH4_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_NH4; the chosen sensor
//********************************************************************************
uint8_t Choose_NH4_Sensor(uint16_t Cal_Number, float * NH4_NH3_N_Total, float * NH4_R, struct ISEConfig ISEs, struct SolutionVals *Sols)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t NH4_Cal_Status[3] = {0,0,0};
	for(i = 0; i < (ISEs.NH4.size > 3 ? 3 : ISEs.NH4.size); i++)
		NH4_Cal_Status[i] = (Cal_Status >> (ISEs.NH4.index + 1 + i)) & 1;

	float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_NH4 = ISEs.NH4.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.NH4.StorBit) & ((1 << ((uint8_t) log2(ISEs.NH4.size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

//	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
//	float NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_NH4, 4));
//	float NH4_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_NH4, 4));
//	float TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));

	float pNH4_Rinse;//, pNH4_Cal_1;
//	if(Sols->Ca_EEP_Cal_1 < 10)	// Values are p-values
//	{
//		pNH4_Rinse = Sols->NH4_EEP_Rinse;
//		pNH4_Cal_1 = Sols->NH4_EEP_Cal_1;
//	}
//	else	// Values are concentration
//	{
		float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
		float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
		float pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
		float pH_TCor_Cal_1 = Calc_pH_TCor(pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1);//pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
		pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_Cal, Sols->IS_RINSE);//-log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_RINSE));
//		pNH4_Cal_1 = Calc_pNH4(NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, IS_CAL_1);//-log10(NH4_EEP_Cal_1 / 14000 * pow(10, -pH_TCor_Cal_1) / (pow(10, -pH_TCor_Cal_1) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CAL_1) + 0.010928 * Lambda_Na(T_Cal, IS_CAL_1) * pow(10, LOG_K_NA_NH4));
//		pNH4_Cal_1 = Calc_pNH4(Sols->NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, Sols->IS_CAL_1);
//	}

	float NH4_EEP_Slope[3] = {0,0,0};
	float NH4_EEP_Int[3] = {0,0,0};
	for(i = 0; i < (ISEs.NH4.size > 3 ? 3 : ISEs.NH4.size); i++)
	{
		NH4_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.NH4.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.NH4.index) * 4), 4));
		NH4_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.NH4.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4));
	}

	// Set up bounds to check against
	float NH4_Max = 5;
	float NH4_Min = -0.15;	// Negative value becasue if we assume too much sodium or potassium it can push this value negative when it's close to 0
	float NH4_linear_mV_Split = 20;
	float within = 0.1; // NH4 ppm

	float NH4_linear_mV[3];
//	for(i = 0; i < ISEs.NH4.size; i++)
//	{
//		float NH4_Cal_1_mV = NH4_EEP_Slope[i] * pNH4_Cal_1 + NH4_EEP_Int[i];
//		NH4_linear_mV[i] = NH4_Cal_1_mV + NH4_EEP_Slope[i] * (pNH4_Rinse - pNH4_Cal_1);
//	}
	for(i = 0; i < (ISEs.NH4.size > 3 ? 3 : ISEs.NH4.size); i++)
	{
		NH4_linear_mV[i] = NH4_EEP_Slope[i] * pNH4_Rinse + NH4_EEP_Int[i];
	}

	// Choose NH4 sensor
	uint8_t T_Chosen_NH4 = 0;
	if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(NH4_NH3_N_Total[i] > NH4_NH3_N_Total[max_reading])
					max_reading = i;
				if(NH4_NH3_N_Total[i] < NH4_NH3_N_Total[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_NH4 = PS_Chosen_NH4;
			else if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading]) < within || abs_val(NH4_NH3_N_Total[mid_reading] - NH4_NH3_N_Total[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_NH4 || max_reading == PS_Chosen_NH4)
					T_Chosen_NH4 = PS_Chosen_NH4;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(NH4_R[min_reading] - NH4_linear_mV[min_reading]) < abs_val(NH4_R[max_reading] - NH4_linear_mV[max_reading]))
						T_Chosen_NH4 = min_reading;
					else
						T_Chosen_NH4 = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.2 NH4 ppm, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV, or reading outside our range
				{
					if(abs_val(NH4_R[i] - NH4_linear_mV[i]) > NH4_linear_mV_Split)
						NH4_Cal_Status[i] = 0;
					if(NH4_NH3_N_Total[i] > NH4_Max || NH4_NH3_N_Total[i] < NH4_Min)
						NH4_Cal_Status[i] = 0;
				}

				if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					if(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading] < NH4_NH3_N_Total[mid_reading] - NH4_NH3_N_Total[min_reading])
					{
						if(abs_val(NH4_R[max_reading] - NH4_linear_mV[max_reading]) < abs_val(NH4_R[mid_reading] - NH4_linear_mV[mid_reading]))
							T_Chosen_NH4 = max_reading;
						else
							T_Chosen_NH4 = mid_reading;
					}
					else
					{
						if(abs_val(NH4_R[min_reading] - NH4_linear_mV[min_reading]) < abs_val(NH4_R[mid_reading] - NH4_linear_mV[mid_reading]))
							T_Chosen_NH4 = min_reading;
						else
							T_Chosen_NH4 = mid_reading;
					}
				}
				else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed calibration
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(NH4_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(NH4_NH3_N_Total[spot_1] - NH4_NH3_N_Total[spot_2]) < within)	// NH4 sensors are close to each other
					{
						if(PS_Chosen_NH4 == spot_1 || PS_Chosen_NH4 == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_NH4 = PS_Chosen_NH4;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]))
								T_Chosen_NH4 = spot_1;
							else
								T_Chosen_NH4 = spot_2;
						}
					}
					else	// NH4 sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]))
							T_Chosen_NH4 = spot_1;
						else
							T_Chosen_NH4 = spot_2;
					}

				}
				else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(NH4_Cal_Status[0] == 1)
						T_Chosen_NH4 = 0;
					else if(NH4_Cal_Status[1] == 1)
						T_Chosen_NH4 = 1;
					else
						T_Chosen_NH4 = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_NH4 = PS_Chosen_NH4;
				}
			}
		}
		else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(NH4_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(NH4_NH3_N_Total[spot_1] - NH4_NH3_N_Total[spot_2]) < within)	// NH4 sensors are close to each other
			{
				T_Chosen_NH4 = PS_Chosen_NH4;
			}
			else	// NH4 sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) > NH4_linear_mV_Split && abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]) < NH4_linear_mV_Split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_NH4 = spot_2;
				else if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < NH4_linear_mV_Split && abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]) > NH4_linear_mV_Split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_NH4 = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((NH4_NH3_N_Total[spot_1] > NH4_Max || NH4_NH3_N_Total[spot_1] < NH4_Min) && (NH4_NH3_N_Total[spot_2] <= NH4_Max && NH4_NH3_N_Total[spot_2] >= NH4_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_NH4 = spot_2;
					else if((NH4_NH3_N_Total[spot_1] <= NH4_Max && NH4_NH3_N_Total[spot_1] >= NH4_Min) && (NH4_NH3_N_Total[spot_2] > NH4_Max || NH4_NH3_N_Total[spot_2] < NH4_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_NH4 = spot_1;
					else	// both NH4 sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_NH4 = PS_Chosen_NH4;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(NH4_Cal_Status[1] == 1)
			T_Chosen_NH4 = 1;
		else if(NH4_Cal_Status[2] == 1)
			T_Chosen_NH4 = 2;
	}

	return T_Chosen_NH4;
}

//********************************************************************************
// Pick which Alk sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			Alk_Samp; pointer to array holding the calculated Alkalinity values for each sensor
//			pH_R; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
//			method; pointer to array holding which method was used to calculate Alkalinity
//			Alk_Slope; pointer to array holding the calculated alkalinity slope for each sensor
// Outputs: T_Chosen_NH4; the chosen sensor
//********************************************************************************
uint8_t Choose_Alk_Sensor(uint16_t Cal_Number, float * Alk_Samp, float * pH_R, float T_Rinse, uint8_t * method, float * Alk_Slope, struct ISEConfig ISEs, struct SolutionVals *Sols)
{
	uint8_t i;

	// Determine whether this die has H2 to use
	uint8_t Alk_index = ISEs.pH_H2.index;
	uint8_t Alk_size = ISEs.pH_H2.size;
	uint8_t Alk_StorBit = ISEs.pH_H2.StorBit;
	if(Alk_size == 0)
	{
		Alk_index = ISEs.pH_Cr.index;
		Alk_size = ISEs.pH_Cr.size;
		Alk_StorBit = ISEs.pH_Cr.StorBit;
	}

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[10] = {0,0,0,0,0,0,0,0,0,0};
	for(i = 0; i < Alk_size; i++)
		pH_Cal_Status[i] = (Cal_Status >> (Alk_index + 1 + i)) & 1;

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_Alk = Alk_size <= 1 ? 0 : (Chosen_Sensors >> Alk_StorBit) & ((1 << ((uint8_t) log2(Alk_size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

//	float pH_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
//	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
//	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
//	float HCl_N = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_T1_HCL_N, 4));

	float pH_EEP_Slope[10];
	float pH_EEP_Int[10];
//	float T_EEP_Cal[10];
	for(i = 0; i < Alk_size; i++)
	{
		pH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[Alk_index + i]), OFFSET_ISE_1_SLOPE + ((i + Alk_index) * 4), 4));
		pH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[Alk_index + i]), OFFSET_ISE_1_INT + ((i + Alk_index) * 4), 4));
//		T_EEP_Cal[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[Alk_index + i]), OFFSET_T_CAL, 4));
	}

	// Set up bounds to check against
	float Alk_Max = 1000;
	float Alk_Min = 10;
	float pH_linear_mV_Split = 10;
	float within = 10; // Alk ppm

	float pH_linear_mV[10];
//	for(i = 0; i < Alk_size; i++)
//	{
//		float pH_TCor_Cal_1 = Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_EEP_Cal[i], 25, 0, Sols->K_T_pH_Cal_1);
//		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Rinse, 25, 0, Sols->K_T_pH_Rinse);
//		float pH_TCor_Clean = Calc_pH_TCor(Sols->pH_EEP_Clean, T_Rinse, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln);
//		float pH_Cal_1_mV = pH_EEP_Slope[i] * pH_TCor_Cal_1 + pH_EEP_Int[i];
//#ifdef CALIBRATE_H2_IN_CLEAN
//		if(i >= ISEs.pH_H2.index && i < (ISEs.pH_H2.index + ISEs.pH_H2.size))
//			pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Clean - pH_TCor_Cal_1);
//		else
//			pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
//#else
//		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
//#endif
//	}
	for(i = 0; i < Alk_size; i++)
	{
//		float pH_TCor_Cal_1 = Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_EEP_Cal[i], 25, 0, Sols->K_T_pH_Cal_1);
		float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Rinse, 25, 0, Sols->K_T_pH_Rinse);
		float pH_TCor_Clean = Calc_pH_TCor(Sols->pH_EEP_Clean, T_Rinse, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln);
#ifdef CALIBRATE_H2_IN_CLEAN
		if(i >= ISEs.pH_H2.index && i < (ISEs.pH_H2.index + ISEs.pH_H2.size))
			pH_linear_mV[i] = pH_EEP_Slope[i] * pH_TCor_Clean + pH_EEP_Int[i];
		else
			pH_linear_mV[i] = pH_EEP_Slope[i] * pH_TCor_Rinse + pH_EEP_Int[i];
#else
		pH_linear_mV[i] = pH_EEP_Slope[i] * pH_TCor_Rinse + pH_EEP_Int[i];
#endif
	}

	// Choose Alk sensor
	// Start by excluding any sensors that didn't find a reading
	for(i = 0; i < 3; i++)
		if(Alk_Samp[i] != Alk_Samp[i])
			pH_Cal_Status[i] = 0;

	// Next check that I didn't exclude the calibration chosen sensor
	if(Alk_Samp[PS_Chosen_Alk] != Alk_Samp[PS_Chosen_Alk])
	{
		uint8_t passed_sensors = pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2];

		if(passed_sensors == 2)
		{
			// Pick out the two spots that found the alkalinity
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(method[i] == 2)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(pH_R[spot_1] - pH_linear_mV[spot_1] < pH_R[spot_2] - pH_linear_mV[spot_2])
				PS_Chosen_Alk = spot_1;
			else
				PS_Chosen_Alk = spot_2;
		}
		else if(passed_sensors == 1)
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					PS_Chosen_Alk = i;
	}

	uint8_t T_Chosen_Alk = 0;
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find Min and Max readings index
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(Alk_Samp[i] > max_reading)
					max_reading = i;
				if(Alk_Samp[i] < min_reading)
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(Alk_Samp[max_reading] - Alk_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_Alk = PS_Chosen_Alk;
			else if(abs_val(Alk_Samp[max_reading] - Alk_Samp[mid_reading]) < within || abs_val(Alk_Samp[mid_reading] - Alk_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(Alk_Samp[max_reading] - Alk_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_Alk || max_reading == PS_Chosen_Alk)
					T_Chosen_Alk = PS_Chosen_Alk;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_R[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_R[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_Alk = min_reading;
					else
						T_Chosen_Alk = max_reading;
				}
			}
			else	// The spread between the three points is greater than 10 Alk ppm, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_R[i] - pH_linear_mV[i]) > pH_linear_mV_Split)
						pH_Cal_Status[i] = 0;
					if(Alk_Samp[i] > Alk_Max || Alk_Samp[i] < Alk_Min)
						pH_Cal_Status[i] = 0;
				}

				// All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3)
				{
					// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
					if(method[0] == 2 || method[1] == 2 || method[2] == 2)	// First check if any found the endpoint
					{
						// Count up how many sensors found endpoint
						uint8_t sensors_at_endpoint = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 2)
								sensors_at_endpoint++;

						if(sensors_at_endpoint == 3)	// All three sensors found the endpoint
						{
							// Pick out of the two sensors that were closest together, then choose the one thats rinse was closest to its linear mV value
							if(Alk_Samp[max_reading] - Alk_Samp[mid_reading] < Alk_Samp[mid_reading] - Alk_Samp[min_reading])	// Smallest split is between mid and max reading
							{
								if(abs_val(pH_R[max_reading] - pH_linear_mV[max_reading]) < abs_val(pH_R[mid_reading] - pH_linear_mV[mid_reading]))
									T_Chosen_Alk = max_reading;
								else
									T_Chosen_Alk = mid_reading;
							}
							else
							{
								if(abs_val(pH_R[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_R[mid_reading] - pH_linear_mV[mid_reading]))
									T_Chosen_Alk = min_reading;
								else
									T_Chosen_Alk = mid_reading;
							}
						}
						else if(sensors_at_endpoint == 2)	// Only 2 sensors found the endpoint
						{
							// Pick out the two spots that found the endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 2)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// Only 1 sensor found the endpoint
						{
							for(i = 0; i < 3; i++)	// Iterate through sensors to find the one that found the endpoint
								if(method[i] == 2)
									T_Chosen_Alk = i;
						}
					}
					// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
					else if(method[0] == 3 || method[1] == 3 || method[2] == 3)
					{
						// Count up how many sensors used method with two points below endpoint
						uint8_t sensors_using_method = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 3)
								sensors_using_method++;

						if(sensors_using_method == 3)	// All three used method with two points below endpoint
						{
							for(i = 0; i < 3; i++)	// Iterate through each sensor
								if(abs_val(Alk_Slope[i] - Sols->HCl_N) < abs_val(Alk_Slope[T_Chosen_Alk] - Sols->HCl_N))	// If current slope is closer to HCl_N value then chosen sensor, set current sensor as chosen
									T_Chosen_Alk = i;
						}
						else if(sensors_using_method == 2)	// 2 sensors used method with two points below endpoint
						{
							// Pick out the two spots that used method with two points below endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 3)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(Alk_Slope[spot_1] - Sols->HCl_N) < abs_val(Alk_Slope[spot_2] - Sols->HCl_N))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// Only 1 sensor used method with two points below endpoint
						{
							for(i = 0; i < 3; i++)
								if(method[i] == 3)
									T_Chosen_Alk = i;
						}
					}
					// Then check if any sensor used method with two points above endpoint
					else if(method[0] == 1 || method[1] == 1 || method[2] == 1)
					{
						// Count up how many sensors used method with two points above endpoint
						uint8_t sensors_using_method = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 1)
								sensors_using_method++;

						if(sensors_using_method == 3)	// All three sensors used method with two points above endpoint
						{
							for(i = 0; i < 3; i++)
								if(abs_val(-Alk_Slope[i] - 6.3) < abs_val(-Alk_Slope[T_Chosen_Alk] - 6.3))
									T_Chosen_Alk = i;
						}
						else if(sensors_using_method == 2)	// 2 sensors used method with two points above endpoint
						{
							// Pick out the two spots that used method with two points above endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 1)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// 1 sensor1 used method with two points above endpoint
						{
							for(i = 0; i < 3; i++)
								if(method[i] == 1)
									T_Chosen_Alk = i;
						}
					}
					// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
					else
					{
						for(i = 0; i < 3; i++)
							if(method[i] == 4)	// Make sure were only changing to sensors that used the interpolation method and not a sensor that didnt find mixing range
								if(abs_val(pH_R[i] - pH_linear_mV[i]) < abs_val(pH_R[T_Chosen_Alk] - pH_linear_mV[T_Chosen_Alk]))
									T_Chosen_Alk = i;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that used method with two points above endpoint
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
					if(method[spot_1] == 2 || method[spot_2] == 2)	// First check if any found the endpoint
					{
						if(method[spot_1] == 2 && method[spot_2] == 2)	// Both sensors found the endpoint
						{
							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 2)	// Only spot 1 found the endpoint
							T_Chosen_Alk = spot_1;
						else	// Only spot 2 found the endpoint
							T_Chosen_Alk = spot_2;
					}
					else if(method[spot_1] == 3 || method[spot_2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
					{
						if(method[spot_1] == 3 && method[spot_2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
						{
							if(abs_val(Alk_Slope[spot_1] - Sols->HCl_N) < abs_val(Alk_Slope[spot_2] - Sols->HCl_N))	// Choose the sensors whose slope is closest to HCl_N value
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 3)	// Only sensor 1 used method with two points below endpoint
							T_Chosen_Alk = spot_1;
						else	// Only sensor 2 used method with two points below endpoint
							T_Chosen_Alk = spot_2;
					}
					else if(method[spot_1] == 1 || method[spot_2] == 1)	// Then check if any sensor used method with two points above endpoint
					{
						if(method[spot_1] == 1 && method[spot_2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
						{
							if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 1)	// Only sensor 1 used method with two points above endpoint
							T_Chosen_Alk = spot_1;
						else	// Only sensor 2 used method with two points above endpoint
							T_Chosen_Alk = spot_2;
					}
					else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
					{
						if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
							T_Chosen_Alk = spot_1;
						else
							T_Chosen_Alk = spot_2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							T_Chosen_Alk = i;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_Alk = PS_Chosen_Alk;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(Alk_Samp[spot_1] - Alk_Samp[spot_2]) < within)	// Alk sensors are close to each other
			{
				T_Chosen_Alk = PS_Chosen_Alk;
			}
			else	// Alk sensors did not read close to each other, check if rinse value is close to what it should be
			{
//				if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) > pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) < pH_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
//					T_Chosen_Alk = spot_2;
//				else if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) > pH_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
//					T_Chosen_Alk = spot_1;
//				else	// If both sensors read within 20 mV of their theoretical rinse value
//				{
					if((Alk_Samp[spot_1] > Alk_Max || Alk_Samp[spot_1] < Alk_Min) && (Alk_Samp[spot_2] <= Alk_Max && Alk_Samp[spot_2] >= Alk_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_Alk = spot_2;
					else if((Alk_Samp[spot_1] <= Alk_Max && Alk_Samp[spot_1] >= Alk_Min) && (Alk_Samp[spot_2] > Alk_Max || Alk_Samp[spot_2] < Alk_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_Alk = spot_1;
					else	// both Alk sensors read within device range then choose based off method and slope
					{
						// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
						if(method[spot_1] == 2 || method[spot_2] == 2)	// First check if any found the endpoint
						{
							if(method[spot_1] == 2 && method[spot_2] == 2)	// Sensors 1 and 2 found the endpoint
							{
								if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 2)	// Only spot 1 found the endpoint
								T_Chosen_Alk = spot_1;
							else	// Only spot 2 found the endpoint
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 3 || method[spot_2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
						{
							if(method[spot_1] == 3 && method[spot_2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
							{
								if(abs_val(Alk_Slope[spot_1] - Sols->HCl_N) < abs_val(Alk_Slope[spot_2] - Sols->HCl_N))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 3)	// Only spot 1 used method with two points below endpoint
								T_Chosen_Alk = spot_1;
							else	// Only spot 2 used method with two points below endpoint
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 1 || method[spot_2] == 1)	// Then check if any sensor used method with two points above endpoint
						{
							if(method[spot_1] == 1 && method[spot_2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
							{
								if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 1)	// Only sensor 1 used method with two points above endpoint
								T_Chosen_Alk = spot_1;
							else	// Only sensor 2 used method with two points above endpoint
								T_Chosen_Alk = spot_2;
						}
						else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
						{
							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
					}
//				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_Alk = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_Alk = 2;
	}

	return T_Chosen_Alk;
}
#else
//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[3] = {0,0,0};
	for(i = 0; i < ISEs.pH_Cr.size; i++)
		pH_Cal_Status[i] = (Cal_Status >> (ISEs.pH_Cr.index + 1 + i)) & 1;

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_pH = ISEs.pH_Cr.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.pH_Cr.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_Cr.size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));

	float pH_EEP_Slope[3] = {0,0,0};
	float pH_EEP_Int[3] = {0,0,0};
	float T_EEP_Cal[3];
	for(i = 0; i < ISEs.pH_Cr.size; i++)
	{
		pH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_Cr.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.pH_Cr.index) * 4), 4));
		pH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_Cr.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.pH_Cr.index) * 4), 4));
		T_EEP_Cal[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_Cr.index + i]), OFFSET_T_CAL, 4));
	}

	// Set up limits to check against
	float pH_Max = 11;
	float pH_Min = 5;
	float pH_linear_split = 10;	// mV to compare test prerinse to cal prerinse
	float pH_linear_mV[3];
	for(i = 0; i < ISEs.pH_Cr.size; i++)
	{
		float pH_Cal_1_mV = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
		float pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Rinse, T_Rinse, 25, 0, K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
		float pH_TCor_Cal_1 = Calc_pH_TCor(pH_EEP_Cal_1, T_EEP_Cal[i], 25, 0, K_T_pH_Cal_1);//pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
	}

	// Choose pH sensor
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		float within = 0.05; // pH units
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find min and max reading to see how far apart all three are
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(pH_Samp[i] > pH_Samp[max_reading])
					max_reading = i;
				if(pH_Samp[i] < pH_Samp[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(pH_Samp[max_reading] - pH_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_pH = PS_Chosen_pH;
			else if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within || abs_val(pH_Samp[mid_reading] - pH_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_pH || max_reading == PS_Chosen_pH)
					T_Chosen_pH = PS_Chosen_pH;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_E_Rinse[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_E_Rinse[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_pH = min_reading;
					else
						T_Chosen_pH = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.1 pH units, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_E_Rinse[i] - pH_linear_mV[i]) > pH_linear_split)
						pH_Cal_Status[i] = 0;
					if(pH_Samp[i] > pH_Max || pH_Samp[i] < pH_Min)
						pH_Cal_Status[i] = 0;
				}

				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					float Differences[3];
					Differences[0] = abs_val(pH_Samp[0] - pH_Samp[1]);
					Differences[1] = abs_val(pH_Samp[0] - pH_Samp[2]);
					Differences[2] = abs_val(pH_Samp[1] - pH_Samp[2]);

					uint8_t Min_diff = 0;
					for(i = 1; i < 3; i++)
						if(Differences[Min_diff] > Differences[i])
							Min_diff = i;

					if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
					else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 2;
					}
					else	// Smallest split is between sensors 2 and 3
					{
						if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 1;
						else
							T_Chosen_pH = 2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed rinse mV check
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
					{
						if(PS_Chosen_pH == spot_1 || PS_Chosen_pH == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_pH = PS_Chosen_pH;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_pH = spot_1;
							else
								T_Chosen_pH = spot_2;
						}
					}
					else	// pH sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(pH_Cal_Status[0] == 1)
						T_Chosen_pH = 0;
					else if(pH_Cal_Status[1] == 1)
						T_Chosen_pH = 1;
					else
						T_Chosen_pH = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
			{
				T_Chosen_pH = PS_Chosen_pH;
			}
			else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) > pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_pH = spot_2;
				else if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_pH = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((pH_Samp[spot_1] > pH_Max || pH_Samp[spot_1] < pH_Min) && (pH_Samp[spot_2] <= pH_Max && pH_Samp[spot_2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_pH = spot_2;
					else if((pH_Samp[spot_1] <= pH_Max && pH_Samp[spot_1] >= pH_Min) && (pH_Samp[spot_2] > pH_Max || pH_Samp[spot_2] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_pH = spot_1;
					else	// both pH sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_pH = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_pH = 2;
	}

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_H2_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[3] = {0,0,0};
	for(i = 0; i < ISEs.pH_H2.size; i++)
		pH_Cal_Status[i] = (Cal_Status >> (ISEs.pH_H2.index + 1 + i)) & 1;

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_pH = ISEs.pH_H2.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.pH_H2.StorBit) & ((1 << ((uint8_t) log2(ISEs.pH_H2.size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));

	float pH_EEP_Slope[3] = {0,0,0};
	float pH_EEP_Int[3] = {0,0,0};
	float T_EEP_Cal[3];
	for(i = 0; i < ISEs.pH_H2.size; i++)
	{
		pH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_H2.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.pH_H2.index) * 4), 4));
		pH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_H2.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.pH_H2.index) * 4), 4));
		T_EEP_Cal[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.pH_H2.index + i]), OFFSET_T_CAL, 4));
	}

	// Set up limits to check against
	float pH_Max = 11;
	float pH_Min = 2;
	float pH_linear_split = 10;	// mV to compare test prerinse to cal prerinse
	float pH_linear_mV[3];
	for(i = 0; i < ISEs.pH_H2.size; i++)
	{
		float pH_Cal_1_mV = pH_EEP_Slope[i] * (pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_EEP_Cal[i] - 25)) + pH_EEP_Int[i];
		float pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Rinse, T_Rinse, 25, 0, K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
		float pH_TCor_Cal_1 = Calc_pH_TCor(pH_EEP_Cal_1, T_EEP_Cal[i], 25, 0, K_T_pH_Cal_1);//pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
	}

	// Choose pH sensor
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		float within = 0.05; // pH units
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find min and max reading to see how far apart all three are
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(pH_Samp[i] > pH_Samp[max_reading])
					max_reading = i;
				if(pH_Samp[i] < pH_Samp[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(pH_Samp[max_reading] - pH_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_pH = PS_Chosen_pH;
			else if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within || abs_val(pH_Samp[mid_reading] - pH_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(pH_Samp[max_reading] - pH_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_pH || max_reading == PS_Chosen_pH)
					T_Chosen_pH = PS_Chosen_pH;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_E_Rinse[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_E_Rinse[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_pH = min_reading;
					else
						T_Chosen_pH = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.1 pH units, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_E_Rinse[i] - pH_linear_mV[i]) > pH_linear_split)
						pH_Cal_Status[i] = 0;
					if(pH_Samp[i] > pH_Max || pH_Samp[i] < pH_Min)
						pH_Cal_Status[i] = 0;
				}

				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					float Differences[3];
					Differences[0] = abs_val(pH_Samp[0] - pH_Samp[1]);
					Differences[1] = abs_val(pH_Samp[0] - pH_Samp[2]);
					Differences[2] = abs_val(pH_Samp[1] - pH_Samp[2]);

					uint8_t Min_diff = 0;
					for(i = 1; i < 3; i++)
						if(Differences[Min_diff] > Differences[i])
							Min_diff = i;

					if(Min_diff == 0)	// Smallest split is between sensors 1 and 2
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
					else if(Min_diff == 1)	// Smallest split is between sensors 1 and 3
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 2;
					}
					else	// Smallest split is between sensors 2 and 3
					{
						if(abs_val(pH_E_Rinse[1] - pH_linear_mV[1]) < abs_val(pH_E_Rinse[2] - pH_linear_mV[2]))
							T_Chosen_pH = 1;
						else
							T_Chosen_pH = 2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed rinse mV check
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
					{
						if(PS_Chosen_pH == spot_1 || PS_Chosen_pH == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_pH = PS_Chosen_pH;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_pH = spot_1;
							else
								T_Chosen_pH = spot_2;
						}
					}
					else	// pH sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(pH_E_Rinse[0] - pH_linear_mV[0]) < abs_val(pH_E_Rinse[1] - pH_linear_mV[1]))
							T_Chosen_pH = 0;
						else
							T_Chosen_pH = 1;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(pH_Cal_Status[0] == 1)
						T_Chosen_pH = 0;
					else if(pH_Cal_Status[1] == 1)
						T_Chosen_pH = 1;
					else
						T_Chosen_pH = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(pH_Samp[spot_1] - pH_Samp[spot_2]) < within)	// pH sensors are close to each other
			{
				T_Chosen_pH = PS_Chosen_pH;
			}
			else	// pH sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) > pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) < pH_linear_split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_pH = spot_2;
				else if(abs_val(pH_E_Rinse[spot_1] - pH_linear_mV[spot_1]) < pH_linear_split && abs_val(pH_E_Rinse[spot_2] - pH_linear_mV[spot_2]) > pH_linear_split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_pH = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((pH_Samp[spot_1] > pH_Max || pH_Samp[spot_1] < pH_Min) && (pH_Samp[spot_2] <= pH_Max && pH_Samp[spot_2] >= pH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_pH = spot_2;
					else if((pH_Samp[spot_1] <= pH_Max && pH_Samp[spot_1] >= pH_Min) && (pH_Samp[spot_2] > pH_Max || pH_Samp[spot_2] < pH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_pH = spot_1;
					else	// both pH sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_pH = PS_Chosen_pH;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_pH = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_pH = 2;
	}

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which pH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// 1/15/2021: Set up to work for full pH die,
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			pH_Samp; pointer to array holding the calculated pH values for each sensor
//			pH_E_Rinse; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
// Outputs: T_Chosen_pH; the chosen sensor
//********************************************************************************
uint8_t Choose_pH_Sensor_pHDie(uint16_t Cal_Number, float * pH_Samp)
{
	uint8_t i;
	uint8_t T_Chosen_pH = 0;

	uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
//	uint8_t ui8Size_pH = 3;
//	if(Sensor_Config == PH_CL_CART)
//		ui8Size_pH = 10;

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[10] = {0,0,0,0,0,0,0,0,0,0};// = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
	pH_Cal_Status[0] = (Cal_Status >> 1) & 1;
	pH_Cal_Status[1] = (Cal_Status >> 2) & 1;
	pH_Cal_Status[2] = (Cal_Status >> 28) & 1;
	if(Sensor_Config == PH_CL_CART)
	{
		pH_Cal_Status[3] = (Cal_Status >> 5) & 1;
		pH_Cal_Status[4] = (Cal_Status >> 6) & 1;
		pH_Cal_Status[5] = (Cal_Status >> 7) & 1;
		pH_Cal_Status[6] = (Cal_Status >> 8) & 1;
		pH_Cal_Status[7] = (Cal_Status >> 9) & 1;
		pH_Cal_Status[8] = (Cal_Status >> 3) & 1;
		pH_Cal_Status[9] = (Cal_Status >> 4) & 1;
	}

	// Choose pH sensor
	float within = 0.01; // pH units

	// Choose a sensor by trying to find groupings of readings, pick the largest grouping and go with the median

	// Count how many close sensors each sensor has and record the max
	uint8_t CloseSensors[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t MaxClose = 0;
	uint8_t j;
	for(i = 0; i < 10; i++)
	{
		if(pH_Cal_Status[i])
			for(j = 0; j < 10; j++)
				if(abs_val(pH_Samp[i] - pH_Samp[j]) <= within && pH_Cal_Status[j])
					CloseSensors[i]++;

		if(CloseSensors[i] > MaxClose)
			MaxClose = CloseSensors[i];
	}

	// After finding max, find out how many sensors matched this max
	uint8_t MaxClose_Sensors[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t TotalCloseSensors = 0;
	for(i = 0; i < 10; i++)
		if(CloseSensors[i] == MaxClose)
		{
			MaxClose_Sensors[i] = 1;	// Set flag so each sensor that had max sensors close is set to 1 and all others are 0
			TotalCloseSensors++;
		}

	// Create array with index values sorted from lowest to highest pH
	uint8_t SortedSensors[10] = {0,0,0,0,0,0,0,0,0,0};
//	uint8_t CurrentIndex = 0;
	for(i = 0; i < TotalCloseSensors; i++)
	{
		uint8_t MinIndex = 0;

		for(j = 0; j < 10; j++)
		{
			if(MaxClose_Sensors[j] == 1)
			{
				MinIndex = j;
				break;
			}
		}

		for(j = 0; j < 10; j++)
		{
			if(MaxClose_Sensors[j] == 1 && pH_Samp[j] < pH_Samp[MinIndex])
				MinIndex = j;
		}

		MaxClose_Sensors[MinIndex] = 0;
		SortedSensors[i] = MinIndex;
	}

	// Set the chosen sensor as the median sensor from the tight group
	if(TotalCloseSensors % 2 == 0)	// If an even number, divide by 2 and subtract 1 to find the index to return
		T_Chosen_pH = SortedSensors[TotalCloseSensors / 2 - 1];
	else
		T_Chosen_pH = SortedSensors[TotalCloseSensors / 2];

	return T_Chosen_pH;
}

//********************************************************************************
// Pick which Ca sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			Ca_Hardness; pointer to array holding the calculated Calcium Hardness values for each sensor
//			Ca_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_Ca; the chosen sensor
//********************************************************************************
uint8_t Choose_Ca_Sensor(uint16_t Cal_Number, float * Ca_Hardness, float * Ca_R, struct ISEConfig ISEs)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t Ca_Cal_Status[2] = {(Cal_Status >> (ISEs.Ca.index + 1)) & 1, (Cal_Status >> (ISEs.Ca.index + 2)) & 1};

	float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_Ca = (Chosen_Sensors >> ISEs.Ca.StorBit) & 0x01;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
	float TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));

	// Check if values in the memory are p-values or concentrations
	float pCa_Rinse;
	float pCa_Cal_1;
	if(Ca_EEP_Rinse < 10)	// Values are p-values
	{
		pCa_Rinse = Ca_EEP_Rinse;
		pCa_Cal_1 = Ca_EEP_Cal_1;
	}
	else	// Values are concentrations
	{
		pCa_Rinse = Calc_pCa(Ca_EEP_Rinse, T_Cal, IS_RINSE);
//		pCa_Cal_1 = Calc_pCa(Ca_EEP_Cal_1, T_Cal, IS_CAL_1);
		if(TH_EEP_Cal_1 > 400)
			pCa_Cal_1 = Calc_pCa(Ca_EEP_Cal_1, T_Cal, IS_CAL_1);
		else
			pCa_Cal_1 = Calc_pCa(Ca_EEP_Cal_1, T_Cal, IS_CAL_1_MG_ADJ);
//		pCa_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE));
//		pCa_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1));
	}

	float Ca_EEP_Slope[2];
	float Ca_EEP_Int[2];
	for(i = 0; i < ISEs.Ca.size; i++)
	{
		Ca_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_ISE_1_SLOPE + ((i + ISEs.Ca.index) * 4), 4));
		Ca_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4));
	}

	// Set up bounds to check against
	float Ca_Hardness_Max = 1250;
	float Ca_Hardness_Min = 2;
	float Ca_linear_mV_Split = 5;
	float within = 10; // ppm Ca Hardness

	float Ca_linear_mV[2];
	for(i = 0; i < 2; i++)
	{
		float Ca_Cal_1_mV = Ca_EEP_Slope[i] * pCa_Cal_1 + Ca_EEP_Int[i];
		Ca_linear_mV[i] = Ca_Cal_1_mV + Ca_EEP_Slope[i] * (pCa_Rinse - pCa_Cal_1);
	}

	// Choose Ca sensor
	uint8_t T_Chosen_Ca = 0;
	if((Ca_Cal_Status[0] + Ca_Cal_Status[1]) > 1)	// Check that both sensors passed
	{
		if(abs_val(Ca_Hardness[0] - Ca_Hardness[1]) < within)	// Ca sensors are close to each other
		{
			T_Chosen_Ca = PS_Chosen_Ca;
		}
		else	// Ca sensors did not read close to each other, check if rinse value is close to what it should be
		{
			if(abs_val(Ca_R[0] - Ca_linear_mV[0]) > Ca_linear_mV_Split && abs_val(Ca_R[1] - Ca_linear_mV[1]) < Ca_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
				T_Chosen_Ca = 1;
			else if(abs_val(Ca_R[0] - Ca_linear_mV[0]) < Ca_linear_mV_Split && abs_val(Ca_R[1] - Ca_linear_mV[1]) > Ca_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
				T_Chosen_Ca = 0;
			else	// If both sensors read within 5 mV of their theoretical rinse value or they both read further away
			{
				if((Ca_Hardness[0] > Ca_Hardness_Max || Ca_Hardness[0] < Ca_Hardness_Min) && (Ca_Hardness[1] <= Ca_Hardness_Max && Ca_Hardness[1] >= Ca_Hardness_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
					T_Chosen_Ca = 1;
				else if((Ca_Hardness[0] <= Ca_Hardness_Max && Ca_Hardness[0] >= Ca_Hardness_Min) && (Ca_Hardness[1] > Ca_Hardness_Max || Ca_Hardness[1] < Ca_Hardness_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
					T_Chosen_Ca = 0;
				else	// both pH sensors read within device range then just go with sensor chosen during calibration
					T_Chosen_Ca = PS_Chosen_Ca;
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(Ca_Cal_Status[1] == 1)
			T_Chosen_Ca = 1;
	}

	return T_Chosen_Ca;
}

//********************************************************************************
// Pick which TH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			TH_corr; pointer to array holding the calculated Total Hardness values for each sensor
//			TH_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_TH; the chosen sensor
//********************************************************************************
uint8_t Choose_TH_Sensor(uint16_t Cal_Number, float * TH_corr, float * TH_R, struct ISEConfig ISEs)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t TH_Cal_Status[2] = {(Cal_Status >> ISEs.TH.index + 1) & 1, (Cal_Status >> ISEs.TH.index + 2) & 1};

	float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_TH = (Chosen_Sensors >> ISEs.TH.StorBit) & 0x01;

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
	float TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));
	float TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_TH, 4));

	float pTH_Rinse, pTH_Cal_1;
	float log_K_Ca_Mg = Build_float(MemoryRead(Cal_page, OFFSET_CAL_LOG_K, 4));
	if(Ca_EEP_Cal_1 < 10)	// Values are p-values
	{
		float Mg_100Ca_Cal_1 = -log10(pow(10, -TH_EEP_Cal_1) - pow(10, -Ca_EEP_Cal_1));
		float Mg_100Ca_Rinse = -log10(pow(10, -TH_EEP_Rinse) - pow(10, -Ca_EEP_Rinse));

		pTH_Cal_1 = -log10(pow(10, -Mg_100Ca_Cal_1) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Cal_1));
		pTH_Rinse = -log10(pow(10, -Mg_100Ca_Rinse) + pow(10, log_K_Ca_Mg) * pow(10, -Ca_EEP_Rinse));
	}
	else	// Values are concentration
	{
		pTH_Rinse = Calc_pTH(Ca_EEP_Rinse, TH_EEP_Rinse, log_K_Ca_Mg, T_Cal, IS_RINSE);
//		pTH_Cal_1 = Calc_pTH(Ca_EEP_Cal_1, TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, IS_CAL_1);
		if(TH_EEP_Cal_1 > 400)
			pTH_Cal_1 = Calc_pTH(Ca_EEP_Cal_1, TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, IS_CAL_1);
		else
			pTH_Cal_1 = Calc_pTH(Ca_EEP_Cal_1, TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, IS_CAL_1_MG_ADJ);
//		pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_Cal, IS_RINSE));
//		pTH_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_1 - Ca_EEP_Cal_1) / 100090 * Lambda_Mg(T_Cal, IS_CAL_1));
	}

	float TH_EEP_Slope[2];
	float TH_EEP_Int[2];
	for(i = 0; i < ISEs.TH.size; i++)
	{
		TH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.TH.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.TH.index) * 4), 4));
		TH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.TH.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.TH.index) * 4), 4));
	}

	// Set up Bounds to check against
	float TH_Max = 2000;
	float TH_Min = 5;
	float TH_linear_mV_Split = 5;	// Acceptable difference between cal prerinse mV and test prerinse mV
	float within = 10; // ppm Total Hardness

	float TH_linear_mV[2];
	for(i = 0; i < 2; i++)
	{
		float log_K_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.TH.index + i]), OFFSET_CAL_LOG_K, 4));
		float TH_Cal_1_mV = TH_EEP_Slope[0] * pTH_Cal_1 + TH_EEP_Int[0];
		TH_linear_mV[i] = TH_Cal_1_mV + TH_EEP_Slope[i] * (pTH_Rinse - pTH_Cal_1);
	}

	// Choose TH sensor
	uint8_t T_Chosen_TH = 0;
	if((TH_Cal_Status[0] + TH_Cal_Status[1]) > 1)	// Check that both sensors passed
	{
		if(abs_val(TH_corr[0] - TH_corr[1]) < within)	// TH sensors are close to each other
		{
			T_Chosen_TH = PS_Chosen_TH;
		}
		else	// TH sensors did not read close to each other, check if rinse value is close to what it should be
		{
			if(abs_val(TH_R[0] - TH_linear_mV[0]) > TH_linear_mV_Split && abs_val(TH_R[1] - TH_linear_mV[1]) < TH_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
				T_Chosen_TH = 1;
			else if(abs_val(TH_R[0] - TH_linear_mV[0]) < TH_linear_mV_Split && abs_val(TH_R[1] - TH_linear_mV[1]) > TH_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
				T_Chosen_TH = 0;
			else	// If both sensors read within 5 mV of their theoretical rinse value or they both read further away
			{
				if((TH_corr[0] > TH_Max || TH_corr[0] < TH_Min) && (TH_corr[1] <= TH_Max && TH_corr[1] >= TH_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
					T_Chosen_TH = 1;
				else if((TH_corr[0] <= TH_Max && TH_corr[0] >= TH_Min) && (TH_corr[1] > TH_Max || TH_corr[1] < TH_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
					T_Chosen_TH = 0;
				else	// both pH sensors read within device range then just go with sensor chosen during calibration
					T_Chosen_TH = PS_Chosen_TH;
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(TH_Cal_Status[1] == 1)
			T_Chosen_TH = 1;
	}

	return T_Chosen_TH;
}

//********************************************************************************
// Pick which TH sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			NH4_NH3_N_Total; pointer to array holding the calculated NH4 values for each sensor
//			NH4_R; pointer to array holding the Rinse mV for each sensor
// Outputs: T_Chosen_NH4; the chosen sensor
//********************************************************************************
uint8_t Choose_NH4_Sensor(uint16_t Cal_Number, float * NH4_NH3_N_Total, float * NH4_R, struct ISEConfig ISEs)
{
	uint8_t i;
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t NH4_Cal_Status[3] = {0,0,0};
	for(i = 0; i < ISEs.NH4.size; i++)
		NH4_Cal_Status[i] = (Cal_Status >> (ISEs.NH4.index + 1 + i)) & 1;

	float T_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_NH4 = ISEs.NH4.size <= 1 ? 0 : (Chosen_Sensors >> ISEs.NH4.StorBit) & ((1 << ((uint8_t) log2(ISEs.NH4.size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
	float NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_NH4, 4));
	float NH4_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_NH4, 4));
	float TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));

	float pNH4_Rinse, pNH4_Cal_1;
	if(Ca_EEP_Cal_1 < 10)	// Values are p-values
	{
		pNH4_Rinse = NH4_EEP_Rinse;
		pNH4_Cal_1 = NH4_EEP_Cal_1;
	}
	else	// Values are concentration
	{
		float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
		float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
		float pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Rinse, T_Cal, 25, 0, K_T_pH_Rinse);//pH_EEP_Rinse + K_T_pH_Rinse * (T_Cal - 25);	// Temperature corrected pH for Rinse
		float pH_TCor_Cal_1 = Calc_pH_TCor(pH_EEP_Cal_1, T_Cal, 25, 0, K_T_pH_Cal_1);//pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
		pNH4_Rinse = Calc_pNH4(NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_Cal, IS_RINSE);//-log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_RINSE));
//		pNH4_Cal_1 = Calc_pNH4(NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, IS_CAL_1);//-log10(NH4_EEP_Cal_1 / 14000 * pow(10, -pH_TCor_Cal_1) / (pow(10, -pH_TCor_Cal_1) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CAL_1) + 0.010928 * Lambda_Na(T_Cal, IS_CAL_1) * pow(10, LOG_K_NA_NH4));
		if(TH_EEP_Cal_1 > 400)
			pNH4_Cal_1 = Calc_pNH4(NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, IS_CAL_1);
		else
			pNH4_Cal_1 = Calc_pNH4(NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, IS_CAL_1_MG_ADJ);
	}

	float NH4_EEP_Slope[3] = {0,0,0};
	float NH4_EEP_Int[3] = {0,0,0};
	for(i = 0; i < ISEs.NH4.size; i++)
	{
		NH4_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.NH4.index + i]), OFFSET_ISE_1_SLOPE + ((i + ISEs.NH4.index) * 4), 4));
		NH4_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[ISEs.NH4.index + i]), OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4));
	}

	// Set up bounds to check against
	float NH4_Max = 5;
	float NH4_Min = -0.15;	// Negative value becasue if we assume too much sodium or potassium it can push this value negative when it's close to 0
	float NH4_linear_mV_Split = 20;
	float within = 0.1; // NH4 ppm

	float NH4_linear_mV[3];
	for(i = 0; i < ISEs.NH4.size; i++)
	{
		float NH4_Cal_1_mV = NH4_EEP_Slope[i] * pNH4_Cal_1 + NH4_EEP_Int[i];
		NH4_linear_mV[i] = NH4_Cal_1_mV + NH4_EEP_Slope[i] * (pNH4_Rinse - pNH4_Cal_1);
	}

	// Choose NH4 sensor
	uint8_t T_Chosen_NH4 = 0;
	if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(NH4_NH3_N_Total[i] > NH4_NH3_N_Total[max_reading])
					max_reading = i;
				if(NH4_NH3_N_Total[i] < NH4_NH3_N_Total[min_reading])
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_NH4 = PS_Chosen_NH4;
			else if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading]) < within || abs_val(NH4_NH3_N_Total[mid_reading] - NH4_NH3_N_Total[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_NH4 || max_reading == PS_Chosen_NH4)
					T_Chosen_NH4 = PS_Chosen_NH4;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(NH4_R[min_reading] - NH4_linear_mV[min_reading]) < abs_val(NH4_R[max_reading] - NH4_linear_mV[max_reading]))
						T_Chosen_NH4 = min_reading;
					else
						T_Chosen_NH4 = max_reading;
				}
			}
			else	// The spread between the three points is greater than 0.2 NH4 ppm, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV, or reading outside our range
				{
					if(abs_val(NH4_R[i] - NH4_linear_mV[i]) > NH4_linear_mV_Split)
						NH4_Cal_Status[i] = 0;
					if(NH4_NH3_N_Total[i] > NH4_Max || NH4_NH3_N_Total[i] < NH4_Min)
						NH4_Cal_Status[i] = 0;
				}

				if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 3) // All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				{
					if(NH4_NH3_N_Total[max_reading] - NH4_NH3_N_Total[mid_reading] < NH4_NH3_N_Total[mid_reading] - NH4_NH3_N_Total[min_reading])
					{
						if(abs_val(NH4_R[max_reading] - NH4_linear_mV[max_reading]) < abs_val(NH4_R[mid_reading] - NH4_linear_mV[mid_reading]))
							T_Chosen_NH4 = max_reading;
						else
							T_Chosen_NH4 = mid_reading;
					}
					else
					{
						if(abs_val(NH4_R[min_reading] - NH4_linear_mV[min_reading]) < abs_val(NH4_R[mid_reading] - NH4_linear_mV[mid_reading]))
							T_Chosen_NH4 = min_reading;
						else
							T_Chosen_NH4 = mid_reading;
					}
				}
				else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that passed calibration
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(NH4_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					if(abs_val(NH4_NH3_N_Total[spot_1] - NH4_NH3_N_Total[spot_2]) < within)	// NH4 sensors are close to each other
					{
						if(PS_Chosen_NH4 == spot_1 || PS_Chosen_NH4 == spot_2)	// If the calibration chosen sensor was one that passed go with that sensor
							T_Chosen_NH4 = PS_Chosen_NH4;
						else // Calibration chosen sensor failed rinse check
						{
							if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]))
								T_Chosen_NH4 = spot_1;
							else
								T_Chosen_NH4 = spot_2;
						}
					}
					else	// NH4 sensors did not read close to each other, both rinse checks passed, both are within readable range // TODO: Here is where I would add stability
					{
						if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]))
							T_Chosen_NH4 = spot_1;
						else
							T_Chosen_NH4 = spot_2;
					}

				}
				else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					if(NH4_Cal_Status[0] == 1)
						T_Chosen_NH4 = 0;
					else if(NH4_Cal_Status[1] == 1)
						T_Chosen_NH4 = 1;
					else
						T_Chosen_NH4 = 2;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_NH4 = PS_Chosen_NH4;
				}
			}
		}
		else if((NH4_Cal_Status[0] + NH4_Cal_Status[1] + NH4_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(NH4_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(NH4_NH3_N_Total[spot_1] - NH4_NH3_N_Total[spot_2]) < within)	// NH4 sensors are close to each other
			{
				T_Chosen_NH4 = PS_Chosen_NH4;
			}
			else	// NH4 sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) > NH4_linear_mV_Split && abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]) < NH4_linear_mV_Split) // If sensor 1 read more than 20 mV off theoretical rinse, but sensor 2 was less than 20 mV use sensor 2
					T_Chosen_NH4 = spot_2;
				else if(abs_val(NH4_R[spot_1] - NH4_linear_mV[spot_1]) < NH4_linear_mV_Split && abs_val(NH4_R[spot_2] - NH4_linear_mV[spot_2]) > NH4_linear_mV_Split) // If sensor 2 read more than 20 mV off theoretical rinse, but sensor 1 was less than 20 mV use sensor 1
					T_Chosen_NH4 = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((NH4_NH3_N_Total[spot_1] > NH4_Max || NH4_NH3_N_Total[spot_1] < NH4_Min) && (NH4_NH3_N_Total[spot_2] <= NH4_Max && NH4_NH3_N_Total[spot_2] >= NH4_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_NH4 = spot_2;
					else if((NH4_NH3_N_Total[spot_1] <= NH4_Max && NH4_NH3_N_Total[spot_1] >= NH4_Min) && (NH4_NH3_N_Total[spot_2] > NH4_Max || NH4_NH3_N_Total[spot_2] < NH4_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_NH4 = spot_1;
					else	// both NH4 sensors read within device range then just go with sensor chosen during calibration
						T_Chosen_NH4 = PS_Chosen_NH4;
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(NH4_Cal_Status[1] == 1)
			T_Chosen_NH4 = 1;
		else if(NH4_Cal_Status[2] == 1)
			T_Chosen_NH4 = 2;
	}

	return T_Chosen_NH4;
}

//********************************************************************************
// Pick which Alk sensor to report, put in function so I can update in one place
// and it updates for all version of code
// Created: 11/24/2020
// Inputs:	Cal_Number; Calibration used for this test, input rather than find so this can be used in MemoryDump()
//			Alk_Samp; pointer to array holding the calculated Alkalinity values for each sensor
//			pH_R; pointer to array holding the Rinse mV for each sensor
//			T_Rinse; Temperature of rinse during this test
//			method; pointer to array holding which method was used to calculate Alkalinity
//			Alk_Slope; pointer to array holding the calculated alkalinity slope for each sensor
// Outputs: T_Chosen_NH4; the chosen sensor
//********************************************************************************
uint8_t Choose_Alk_Sensor(uint16_t Cal_Number, float * Alk_Samp, float * pH_R, float T_Rinse, uint8_t * method, float * Alk_Slope, struct ISEConfig ISEs)
{
	uint8_t i;

	// Determine whether this die has H2 to use
	uint8_t Alk_index = ISEs.pH_H2.index;
	uint8_t Alk_size = ISEs.pH_H2.size;
	uint8_t Alk_StorBit = ISEs.pH_H2.StorBit;
	if(Alk_size == 0)
	{
		Alk_index = ISEs.pH_Cr.index;
		Alk_size = ISEs.pH_Cr.size;
		Alk_StorBit = ISEs.pH_Cr.StorBit;
	}

	uint16_t Cal_page = Find_Cal_page(Cal_Number);
	uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
	uint8_t pH_Cal_Status[10] = {0,0,0,0,0,0,0,0,0,0};
	for(i = 0; i < Alk_size; i++)
		pH_Cal_Status[i] = (Cal_Status >> (Alk_index + 1 + i)) & 1;

	uint8_t Chosen_Sensors = *(MemoryRead(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1));
	uint8_t PS_Chosen_Alk = Alk_size <= 1 ? 0 : (Chosen_Sensors >> Alk_StorBit) & ((1 << ((uint8_t) log2(Alk_size - 1) + 1)) - 1);

	uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
	uint8_t Last_cal_passed[10];
	memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
	for(i = 0; i < 10; i++)
		if(Last_cal_passed[i] == 0xFF)
			Last_cal_passed[i] = Cal_Number;

	float pH_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
	float pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
	float pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
	float HCl_N = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_T1_HCL_N, 4));

	float pH_EEP_Slope[10];
	float pH_EEP_Int[10];
	float T_EEP_Cal[10];
	for(i = 0; i < Alk_size; i++)
	{
		pH_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[Alk_index + i]), OFFSET_ISE_1_SLOPE + ((i + Alk_index) * 4), 4));
		pH_EEP_Int[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[Alk_index + i]), OFFSET_ISE_1_INT + ((i + Alk_index) * 4), 4));
		T_EEP_Cal[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[Alk_index + i]), OFFSET_T_CAL, 4));
	}

	// Set up bounds to check against
	float Alk_Max = 1000;
	float Alk_Min = 10;
	float pH_linear_mV_Split = 10;
	float within = 10; // Alk ppm

	float pH_linear_mV[10];
	for(i = 0; i < Alk_size; i++)
	{
		float pH_TCor_Cal_1 = Calc_pH_TCor(pH_EEP_Cal_1, T_EEP_Cal[i], 25, 0, K_T_pH_Cal_1);
		float pH_TCor_Rinse = Calc_pH_TCor(pH_EEP_Rinse, T_Rinse, 25, 0, K_T_pH_Rinse);
		float pH_TCor_Clean = Calc_pH_TCor(pH_EEP_Clean, T_Rinse, 25, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln);
		float pH_Cal_1_mV = pH_EEP_Slope[i] * pH_TCor_Cal_1 + pH_EEP_Int[i];
#ifdef CALIBRATE_H2_IN_CLEAN
		if(i >= ISEs.pH_H2.index && i < (ISEs.pH_H2.index + ISEs.pH_H2.size))
			pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Clean - pH_TCor_Cal_1);
		else
			pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
#else
		pH_linear_mV[i] = pH_Cal_1_mV + pH_EEP_Slope[i] * (pH_TCor_Rinse - pH_TCor_Cal_1);
#endif
	}

	// Choose Alk sensor
	// Start by excluding any sensors that didn't find a reading
	for(i = 0; i < 3; i++)
		if(Alk_Samp[i] == -1)
			pH_Cal_Status[i] = 0;
	// Next check that I didn't exclude the calibration chosen sensor
	if(Alk_Samp[PS_Chosen_Alk] == -1)
	{
		uint8_t passed_sensors = pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2];

		if(passed_sensors == 2)
		{
			// Pick out the two spots that found the alkalinity
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(method[i] == 2)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(pH_R[spot_1] - pH_linear_mV[spot_1] < pH_R[spot_2] - pH_linear_mV[spot_2])
				PS_Chosen_Alk = spot_1;
			else
				PS_Chosen_Alk = spot_2;
		}
		else if(passed_sensors == 1)
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					PS_Chosen_Alk = i;
	}

	uint8_t T_Chosen_Alk = 0;
	if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) > 1)	// Check that multiple sensors passed
	{
		if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3) // All three sensors passed calibration
		{
			// Find Min and Max readings index
			uint8_t max_reading = 0;
			uint8_t min_reading = 0;
			for(i = 1; i < 3; i++)
			{
				if(Alk_Samp[i] > max_reading)
					max_reading = i;
				if(Alk_Samp[i] < min_reading)
					min_reading = i;
			}

			// Find mid reading
			uint8_t mid_reading = 0;
			for(i = 0; i < 3; i++)
				if(i != max_reading && i != min_reading)
					mid_reading = i;

			if(abs_val(Alk_Samp[max_reading] - Alk_Samp[min_reading]) < within) // All three points read close to each other, go with chosen from calibration
				T_Chosen_Alk = PS_Chosen_Alk;
			else if(abs_val(Alk_Samp[max_reading] - Alk_Samp[mid_reading]) < within || abs_val(Alk_Samp[mid_reading] - Alk_Samp[min_reading]) < within)	// Two sensors are close to each other but the third isn't
			{
				// Figure out which two are close to each other and set those indicies in min and max variables
				if(abs_val(Alk_Samp[max_reading] - Alk_Samp[mid_reading]) < within)
					min_reading = mid_reading;
				else
					max_reading = mid_reading;

				// If one of the two close sensors was the chosen sensor from calibration go with that sensor
				if(min_reading == PS_Chosen_Alk || max_reading == PS_Chosen_Alk)
					T_Chosen_Alk = PS_Chosen_Alk;
				else // Chosen sensor from calibration is not one of the two sensors close to each other
				{
					// Shouldn't matter too much as these sensors are close to each other, pick the one with less drift in prerinse
					if(abs_val(pH_R[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_R[max_reading] - pH_linear_mV[max_reading]))
						T_Chosen_Alk = min_reading;
					else
						T_Chosen_Alk = max_reading;
				}
			}
			else	// The spread between the three points is greater than 10 Alk ppm, check rinse mV
			{
				for(i = 0; i < 3; i++)	// Try to remove any sensors based on rinse mV
				{
					if(abs_val(pH_R[i] - pH_linear_mV[i]) > pH_linear_mV_Split)
						pH_Cal_Status[i] = 0;
					if(Alk_Samp[i] > Alk_Max || Alk_Samp[i] < Alk_Min)
						pH_Cal_Status[i] = 0;
				}

				// All three sensors were within 20 mV of their theoretical rinse value and within reading range of our device
				if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 3)
				{
					// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
					if(method[0] == 2 || method[1] == 2 || method[2] == 2)	// First check if any found the endpoint
					{
						// Count up how many sensors found endpoint
						uint8_t sensors_at_endpoint = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 2)
								sensors_at_endpoint++;

						if(sensors_at_endpoint == 3)	// All three sensors found the endpoint
						{
							// Pick out of the two sensors that were closest together, then choose the one thats rinse was closest to its linear mV value
							if(Alk_Samp[max_reading] - Alk_Samp[mid_reading] < Alk_Samp[mid_reading] - Alk_Samp[min_reading])	// Smallest split is between mid and max reading
							{
								if(abs_val(pH_R[max_reading] - pH_linear_mV[max_reading]) < abs_val(pH_R[mid_reading] - pH_linear_mV[mid_reading]))
									T_Chosen_Alk = max_reading;
								else
									T_Chosen_Alk = mid_reading;
							}
							else
							{
								if(abs_val(pH_R[min_reading] - pH_linear_mV[min_reading]) < abs_val(pH_R[mid_reading] - pH_linear_mV[mid_reading]))
									T_Chosen_Alk = min_reading;
								else
									T_Chosen_Alk = mid_reading;
							}
						}
						else if(sensors_at_endpoint == 2)	// Only 2 sensors found the endpoint
						{
							// Pick out the two spots that found the endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 2)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// Only 1 sensor found the endpoint
						{
							for(i = 0; i < 3; i++)	// Iterate through sensors to find the one that found the endpoint
								if(method[i] == 2)
									T_Chosen_Alk = i;
						}
					}
					// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
					else if(method[0] == 3 || method[1] == 3 || method[2] == 3)
					{
						// Count up how many sensors used method with two points below endpoint
						uint8_t sensors_using_method = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 3)
								sensors_using_method++;

						if(sensors_using_method == 3)	// All three used method with two points below endpoint
						{
							for(i = 0; i < 3; i++)	// Iterate through each sensor
								if(abs_val(Alk_Slope[i] - HCl_N) < abs_val(Alk_Slope[T_Chosen_Alk] - HCl_N))	// If current slope is closer to HCl_N value then chosen sensor, set current sensor as chosen
									T_Chosen_Alk = i;
						}
						else if(sensors_using_method == 2)	// 2 sensors used method with two points below endpoint
						{
							// Pick out the two spots that used method with two points below endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 3)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(Alk_Slope[spot_1] - HCl_N) < abs_val(Alk_Slope[spot_2] - HCl_N))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// Only 1 sensor used method with two points below endpoint
						{
							for(i = 0; i < 3; i++)
								if(method[i] == 3)
									T_Chosen_Alk = i;
						}
					}
					// Then check if any sensor used method with two points above endpoint
					else if(method[0] == 1 || method[1] == 1 || method[2] == 1)
					{
						// Count up how many sensors used method with two points above endpoint
						uint8_t sensors_using_method = 0;
						for(i = 0; i < 3; i++)
							if(method[i] == 1)
								sensors_using_method++;

						if(sensors_using_method == 3)	// All three sensors used method with two points above endpoint
						{
							for(i = 0; i < 3; i++)
								if(abs_val(-Alk_Slope[i] - 6.3) < abs_val(-Alk_Slope[T_Chosen_Alk] - 6.3))
									T_Chosen_Alk = i;
						}
						else if(sensors_using_method == 2)	// 2 sensors used method with two points above endpoint
						{
							// Pick out the two spots that used method with two points above endpoint
							uint8_t spot_1 = 0xFF;
							uint8_t spot_2 = 0xFF;
							for(i = 0; i < 3; i++)
								if(method[i] == 1)
									if(spot_1 == 0xFF)
										spot_1 = i;
									else
										spot_2 = i;

							if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else	// 1 sensor1 used method with two points above endpoint
						{
							for(i = 0; i < 3; i++)
								if(method[i] == 1)
									T_Chosen_Alk = i;
						}
					}
					// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
					else
					{
						for(i = 0; i < 3; i++)
							if(method[i] == 4)	// Make sure were only changing to sensors that used the interpolation method and not a sensor that didnt find mixing range
								if(abs_val(pH_R[i] - pH_linear_mV[i]) < abs_val(pH_R[T_Chosen_Alk] - pH_linear_mV[T_Chosen_Alk]))
									T_Chosen_Alk = i;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only 2 sensors were within 20 mV of their theoretical rinse value
				{
					// Pick out the two spots that used method with two points above endpoint
					uint8_t spot_1 = 0xFF;
					uint8_t spot_2 = 0xFF;
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							if(spot_1 == 0xFF)
								spot_1 = i;
							else
								spot_2 = i;

					// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
					if(method[spot_1] == 2 || method[spot_2] == 2)	// First check if any found the endpoint
					{
						if(method[spot_1] == 2 && method[spot_2] == 2)	// Both sensors found the endpoint
						{
							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 2)	// Only spot 1 found the endpoint
							T_Chosen_Alk = spot_1;
						else	// Only spot 2 found the endpoint
							T_Chosen_Alk = spot_2;
					}
					else if(method[spot_1] == 3 || method[spot_2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
					{
						if(method[spot_1] == 3 && method[spot_2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
						{
							if(abs_val(Alk_Slope[spot_1] - HCl_N) < abs_val(Alk_Slope[spot_2] - HCl_N))	// Choose the sensors whose slope is closest to HCl_N value
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 3)	// Only sensor 1 used method with two points below endpoint
							T_Chosen_Alk = spot_1;
						else	// Only sensor 2 used method with two points below endpoint
							T_Chosen_Alk = spot_2;
					}
					else if(method[spot_1] == 1 || method[spot_2] == 1)	// Then check if any sensor used method with two points above endpoint
					{
						if(method[spot_1] == 1 && method[spot_2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
						{
							if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 1)	// Only sensor 1 used method with two points above endpoint
							T_Chosen_Alk = spot_1;
						else	// Only sensor 2 used method with two points above endpoint
							T_Chosen_Alk = spot_2;
					}
					else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
					{
						if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
							T_Chosen_Alk = spot_1;
						else
							T_Chosen_Alk = spot_2;
					}
				}
				else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 1) // Only 1 sensor was within 20 mV of their theoretical rinse value
				{
					for(i = 0; i < 3; i++)
						if(pH_Cal_Status[i] == 1)
							T_Chosen_Alk = i;
				}
				else	// All sensors failed the rinse mV check
				{
					T_Chosen_Alk = PS_Chosen_Alk;
				}
			}
		}
		else if((pH_Cal_Status[0] + pH_Cal_Status[1] + pH_Cal_Status[2]) == 2) // Only two sensors passed calibration
		{
			// Pick out the two spots that passed calibration
			uint8_t spot_1 = 0xFF;
			uint8_t spot_2 = 0xFF;
			for(i = 0; i < 3; i++)
				if(pH_Cal_Status[i] == 1)
					if(spot_1 == 0xFF)
						spot_1 = i;
					else
						spot_2 = i;

			if(abs_val(Alk_Samp[spot_1] - Alk_Samp[spot_2]) < within)	// Alk sensors are close to each other
			{
				T_Chosen_Alk = PS_Chosen_Alk;
			}
			else	// Alk sensors did not read close to each other, check if rinse value is close to what it should be
			{
				if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) > pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) < pH_linear_mV_Split) // If sensor 1 read more than 5 mV off theoretical rinse, but sensor 2 was less than 5 mV use sensor 2
					T_Chosen_Alk = spot_2;
				else if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < pH_linear_mV_Split && abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]) > pH_linear_mV_Split) // If sensor 2 read more than 5 mV off theoretical rinse, but sensor 1 was less than 5 mV use sensor 1
					T_Chosen_Alk = spot_1;
				else	// If both sensors read within 20 mV of their theoretical rinse value
				{
					if((Alk_Samp[spot_1] > Alk_Max || Alk_Samp[spot_1] < Alk_Min) && (Alk_Samp[spot_2] <= Alk_Max && Alk_Samp[spot_2] >= Alk_Min)) // if sensor 1 read outside our reading range but sensor 2 read inside pick sensor 2
						T_Chosen_Alk = spot_2;
					else if((Alk_Samp[spot_1] <= Alk_Max && Alk_Samp[spot_1] >= Alk_Min) && (Alk_Samp[spot_2] > Alk_Max || Alk_Samp[spot_2] < Alk_Min)) // if sensor 2 read outside our reading range but sensor 1 read inside pick sensor 1
						T_Chosen_Alk = spot_1;
					else	// both Alk sensors read within device range then choose based off method and slope
					{
						// Other sensors chosen based off linearity, for alkalinity choose based off method and slope
						if(method[spot_1] == 2 || method[spot_2] == 2)	// First check if any found the endpoint
						{
							if(method[spot_1] == 2 && method[spot_2] == 2)	// Sensors 1 and 2 found the endpoint
							{
								if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 2)	// Only spot 1 found the endpoint
								T_Chosen_Alk = spot_1;
							else	// Only spot 2 found the endpoint
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 3 || method[spot_2] == 3)	// Then check if any sensor used method with two points below endpoint (this is method we are shooting for)
						{
							if(method[spot_1] == 3 && method[spot_2] == 3)	// Sensors 1 and 2 used method with two points below endpoint
							{
								if(abs_val(Alk_Slope[spot_1] - HCl_N) < abs_val(Alk_Slope[spot_2] - HCl_N))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 3)	// Only spot 1 used method with two points below endpoint
								T_Chosen_Alk = spot_1;
							else	// Only spot 2 used method with two points below endpoint
								T_Chosen_Alk = spot_2;
						}
						else if(method[spot_1] == 1 || method[spot_2] == 1)	// Then check if any sensor used method with two points above endpoint
						{
							if(method[spot_1] == 1 && method[spot_2] == 1)	// Sensors 1 and 2 used method with two points above endpoint
							{
								if(abs_val(-Alk_Slope[spot_1] - 6.3) < abs_val(-Alk_Slope[spot_2] - 6.3))
									T_Chosen_Alk = spot_1;
								else
									T_Chosen_Alk = spot_2;
							}
							else if(method[spot_1] == 1)	// Only sensor 1 used method with two points above endpoint
								T_Chosen_Alk = spot_1;
							else	// Only sensor 2 used method with two points above endpoint
								T_Chosen_Alk = spot_2;
						}
						else	// All sensors used interpolation method or mixing never found it's range and all three alkalinities = -1
						{
							if(abs_val(pH_R[spot_1] - pH_linear_mV[spot_1]) < abs_val(pH_R[spot_2] - pH_linear_mV[spot_2]))
								T_Chosen_Alk = spot_1;
							else
								T_Chosen_Alk = spot_2;
						}
					}
				}
			}
		}
	}
	else	// If only one or no sensors passed
	{
		if(pH_Cal_Status[1] == 1)
			T_Chosen_Alk = 1;
		else if(pH_Cal_Status[2] == 1)
			T_Chosen_Alk = 2;
	}

	return T_Chosen_Alk;
}
#endif	// SOLUTION_IN_STRUCT
#endif

//********************************************************************************
// Measure cartridge temperature and record the minimum and maximum temperature
// the cartridge experienced in its lifetime
// Created: 12/18/2020
// Inputs:	NONE
// Outputs: NONE; saves data to cartridge memory
//********************************************************************************
void RecordTemp(void)
{
	// Can only measure and save if there is a cartridge plugged into box
	if(gCartridge == 1)
	{
#ifdef MCU_ZXR
		int32_t analog_on = GPIOPinRead(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN); // Analog On
#else
		int32_t analog_on = GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_6); // Analog On
#endif

		if(analog_on == 0 && gBoard >= V6_4)
			InitAnalog();

		float Temp = MeasureTemperature(1);

		if(analog_on == 0 && gBoard >= V6_4)
			AnalogOff();

//		PrintTime();
//		float Temp = GetCPUTemp();

		float Min_Temp = Build_float(MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MIN_CART_TEMP, 4));
		float Max_Temp = Build_float(MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MAX_CART_TEMP, 4));

		// Check if measure temperature is less than recorded minimum temperature, also check that there is a real number saved, if there isn't then save the current number
		if(Temp < Min_Temp || Min_Temp != Min_Temp)
		{
//			pui8SysStatus = RequestSystemStatus(); // Get time and user information at beginning of calibration
			uint8_t *DateTime = GetTime();
			MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_MIN_CART_TEMP, 4, (uint8_t *) &Temp);
			MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_MIN_TEMP_DATE, 4, DateTime);
			MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_MIN_TEMP_TIME, 2, (DateTime + 4));
		}

		// Check if measure temperature is less than recorded minimum temperature, also check that there is a real number saved, if there isn't then save the current number
		if(Temp > Max_Temp || Max_Temp != Max_Temp)
		{
//			pui8SysStatus = RequestSystemStatus(); // Get time and user information at beginning of calibration
			uint8_t *DateTime = GetTime();
			MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_MAX_CART_TEMP, 4, (uint8_t *) &Temp);
			MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_MAX_TEMP_DATE, 4, DateTime);
			MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_MAX_TEMP_TIME, 2, (DateTime + 4));
		}

		if((Temp < Min_Temp || Min_Temp != Min_Temp) || (Temp > Max_Temp || Max_Temp != Max_Temp))
			update_CartTemp();
	}
}

////********************************************************************************
//// Check the most recent test and cal to see if the sensor was pumped through
//// recently
//// Created: 3/8/2020
//// Inputs:	NONE
//// Outputs: 0 or 1; 1 if sensor hasn't been run recently
////********************************************************************************
//uint8_t CheckSensorSat(void)
//{
//	if(gCartridge == 1)
//	{
//		// Find the unix time for the last test
//		uint16_t TestNumber = FindTestNumber();
//		uint16_t Test_page = Find_Test_page(TestNumber);
//
//		uint8_t * pui8TestTime = MemoryRead(Test_page, OFFSET_TEST_DATE, 7);
//
//		// Fill in time structure with data read from memory
//		struct tm TestTime;
//		TestTime.tm_mon = *pui8TestTime - 1;	// Month [0,11]
//		TestTime.tm_mday = *(pui8TestTime + 1); // Day [1,31]
//		TestTime.tm_year = (*(pui8TestTime + 2) * 100) + *(pui8TestTime + 3) - 1900;	// Years since 1900
//		TestTime.tm_hour = *(pui8TestTime + 4);
//		TestTime.tm_min = *(pui8TestTime + 5);
//		TestTime.tm_sec = *(pui8TestTime + 6);
//
//		uint32_t Test_Date = mktime(&TestTime);	// Convert last cal time from human readable to Epoch time (seconds since 1900)
//
//		// Find the unix time for the last cal
//		uint16_t CalNumber = FindCalNumber();
//		uint16_t Cal_page = Find_Cal_page(CalNumber);
//		uint8_t * pui8CalTime = MemoryRead(Cal_page, OFFSET_CAL_DATE, 7);
//		struct tm CalTime;
//		CalTime.tm_mon = *pui8CalTime - 1;	// Month [0,11]
//		CalTime.tm_mday = *(pui8CalTime + 1); // Day [1,31]
//		CalTime.tm_year = (*(pui8CalTime + 2) * 100) + *(pui8CalTime + 3) - 1900;	// Years since 1900
//		CalTime.tm_hour = *(pui8CalTime + 4);
//		CalTime.tm_min = *(pui8CalTime + 5);
//		CalTime.tm_sec = *(pui8CalTime + 6);
//
//		uint32_t Cal_Date = mktime(&CalTime);
//
//		// Find the current unix time
//		uint8_t * pui8CurTime = GetTime();
//		struct tm CurTime;
//		CurTime.tm_mon = *pui8CurTime - 1;
//		CurTime.tm_mday = *(pui8CurTime + 1);
//		CurTime.tm_year = *(pui8CurTime + 2) * 100 + *(pui8CurTime + 3) - 1900;
//		CurTime.tm_hour = *(pui8CurTime + 4);
//		CurTime.tm_min = *(pui8CurTime + 5);
//		CurTime.tm_sec = *(pui8CurTime + 6);
//
//		uint32_t Cur_Date = mktime(&CurTime);	// Get current time in seconds since 1900
//
//		uint32_t TimeLimit = 60 * 60;	// Set time limit for 1 hour
//
//		// Check if both the last test and last cal was before the time limit and return 1 if they were
//		if((Cur_Date - Test_Date) > TimeLimit && (Cur_Date - Cal_Date) > TimeLimit)
//			return 1;
//	}
//
//	return 0;
//}

//********************************************************************************
// Calculates the calcium lambda value given a temperature and ionic strength,
// put in fuction because of how often it is reused
// Created: 3/19/2021
// Inputs:	Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated Lambda
//********************************************************************************
float Lambda_Ca(float Temp, float IS)
{
	return pow(10, (-0.51016 * 298 / (273 + Temp) * 4 * sqrt(IS)) / (1 + sqrt(298 / (273 + Temp)) * 600/305 * sqrt(IS)));
}

//********************************************************************************
// Calculates the magnesium lambda value given a temperature and ionic strength,
// put in fuction because of how often it is reused
// Created: 3/19/2021
// Inputs:	Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated Lambda
//********************************************************************************
float Lambda_Mg(float Temp, float IS)
{
	return pow(10, (-0.51016 * 298 / (273 + Temp) * 4 * sqrt(IS)) / (1 + sqrt(298 / (273 + Temp)) * 800/305 * sqrt(IS)));
}

//********************************************************************************
// Calculates the ammonium lambda value given a temperature and ionic strength,
// put in fuction because of how often it is reused
// Created: 3/19/2021
// Inputs:	Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated Lambda
//********************************************************************************
float Lambda_NH4(float Temp, float IS)
{
	return pow(10, (-0.51016 * 298 / (273 + Temp) * sqrt(IS)) / (1 + sqrt(298 / (273 + Temp)) * 250/305 * sqrt(IS)));
}

//********************************************************************************
// Calculates the sodium lambda value given a temperature and ionic strength,
// put in fuction because of how often it is reused
// Created: 3/19/2020
// Inputs:	Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated Lambda
//********************************************************************************
float Lambda_Na(float Temp, float IS)
{
	return pow(10, (-0.51016 * 298 / (273 + Temp) * sqrt(IS)) / (1 + sqrt(298 / (273 + Temp)) * 450/305 * sqrt(IS)));
}

//********************************************************************************
// Calculates the potassium lambda value given a temperature and ionic strength,
// put in fuction because of how often it is reused
// Created: 3/19/2021
// Inputs:	Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated Lambda
//********************************************************************************
float Lambda_K(float Temp, float IS)
{
	return pow(10, (-0.51016 * 298 / (273 + Temp) * sqrt(IS)) / (1 + sqrt(298 / (273 + Temp)) * 300/305 * sqrt(IS)));
}

//********************************************************************************
// Calculates the temperature correction for pH, done in a function so I only have
// to update the math in one place
// Created: 4/19/2021
// Inputs:	pH_EEP, pH of solution, for cals coming from EEP, but can be used for sample too
//			Temp_Cor; Temp we are correcting to
//			Temp_Meas; Temperature readings are taken at, 25 for standards, T_Sensor for samples
//			K_T_pH_Sq; Squared term for correction, 0 if using linear correction
//			K_T_pH_Ln; Linear term for correction
// Outputs: Calculated pH at Temp_Cor
//********************************************************************************
float Calc_pH_TCor(float pH, float Temp_Cor, float Temp_Meas, float K_T_pH_Sq, float K_T_pH_Ln)
{
	float pH_TCor = pH + (K_T_pH_Sq * (pow(Temp_Cor, 2) - pow(Temp_Meas, 2)) + K_T_pH_Ln * (Temp_Cor - Temp_Meas));	// Temperature corrected pH for Rinse;
	return pH_TCor;
//	float pH_TCor_Rinse = pH_EEP + K_T_pH * (T_Cal - 25);	// Temperature corrected pH for Rinse
//	float pH_TCor_Cal_1 = pH_EEP_Cal_1 + K_T_pH_Cal_1 * (T_Cal - 25);
//	float pH_TCor_Cal_2 = pH_EEP_Cal_2 + K_T_pH_Cal_2 * (T_Cal - 25);
//	float pH_TCor_Clean = pH_EEP_Clean + (K_T_pH_Clean_Sq * (pow(T_Cal, 2) - pow(25, 2)) + K_T_pH_Clean_Ln * (T_Cal - 25));	// Temperature corrected pH for Rinse
}

//********************************************************************************
// Calculates the p-value for calcium put in function because this math is reused
// many times.
// Created: 4/19/2021
// Inputs:	Ca_EEP_Conc, Concentration of calcium in solution
//			Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated p-value
//********************************************************************************
float Calc_pCa(float Ca_EEP_Conc, float Temp, float IS)
{
	float pCa = -log10(Ca_EEP_Conc / 100090 * Lambda_Ca(Temp, IS));
	return pCa;
//	pCa_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE));
//	pCa_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1));
//	pCa_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2));
}

#ifdef PH_LOG_K
//********************************************************************************
// Calculates the p-value for total hardness put in function because this math is reused
// many times.
// Created: 4/19/2021
// Inputs:	Ca_EEP_Conc, Concentration of calcium in solution
//			TH_EEP_Conc, Concentration of total hardness in solution
//			Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated p-value
//********************************************************************************
float Calc_pTHpH(float Ca_EEP_Conc, float TH_EEP_Conc, float Log_K_Ca_Mg, float Temp, float IS, float pH, float Log_K_pH_TH)
{
	float pTH = -log10(Ca_EEP_Conc / 100090 * Lambda_Ca(Temp, IS) * pow(10, Log_K_Ca_Mg) + (TH_EEP_Conc - Ca_EEP_Conc) / 100090 * Lambda_Mg(Temp, IS) + pow(10, -pH) * pow(10, Log_K_pH_TH));
	return pTH;
	//	pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_Cal, IS_RINSE));
	//	pTH_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_1 - Ca_EEP_Cal_1) / 100090 * Lambda_Mg(T_Cal, IS_CAL_1));
	//	pTH_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_2 - Ca_EEP_Cal_2) / 100090 * Lambda_Mg(T_Cal, IS_CAL_2));
}
#endif	// PH_LOG_K

//********************************************************************************
// Calculates the p-value for total hardness put in function because this math is reused
// many times.
// Created: 4/19/2021
// Inputs:	Ca_EEP_Conc, Concentration of calcium in solution
//			TH_EEP_Conc, Concentration of total hardness in solution
//			Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated p-value
//********************************************************************************
float Calc_pTH(float Ca_EEP_Conc, float TH_EEP_Conc, float Log_K_Ca_Mg, float Temp, float IS)
{
	float pTH = -log10(Ca_EEP_Conc / 100090 * Lambda_Ca(Temp, IS) * pow(10, Log_K_Ca_Mg) + (TH_EEP_Conc - Ca_EEP_Conc) / 100090 * Lambda_Mg(Temp, IS));
	return pTH;
	//	pTH_Rinse = -log10(Ca_EEP_Rinse / 100090 * Lambda_Ca(T_Cal, IS_RINSE) * pow(10, log_K_Ca_Mg) + (TH_EEP_Rinse - Ca_EEP_Rinse) / 100090 * Lambda_Mg(T_Cal, IS_RINSE));
	//	pTH_Cal_1 = -log10(Ca_EEP_Cal_1 / 100090 * Lambda_Ca(T_Cal, IS_CAL_1) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_1 - Ca_EEP_Cal_1) / 100090 * Lambda_Mg(T_Cal, IS_CAL_1));
	//	pTH_Cal_2 = -log10(Ca_EEP_Cal_2 / 100090 * Lambda_Ca(T_Cal, IS_CAL_2) * pow(10, log_K_Ca_Mg) + (TH_EEP_Cal_2 - Ca_EEP_Cal_2) / 100090 * Lambda_Mg(T_Cal, IS_CAL_2));
}

//********************************************************************************
// Calculates the p-value for Mg hardness, put in function because this math is reused
// many times.
// Created: 8/27/2021
// Inputs:	Ca_EEP_Conc, Concentration of calcium in solution
//			TH_EEP_Conc, Concentration of total hardness in solution
//			Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated p-value
//********************************************************************************
float Calc_pMg(float Ca_EEP_Conc, float TH_EEP_Conc, float Temp, float IS)
{
	float pMg = -log10((TH_EEP_Conc - Ca_EEP_Conc) / 100090 * Lambda_Mg(Temp, IS));
	return pMg;
}

//********************************************************************************
// Calculates the p-value for ammonia put in function because this math is reused
// many times.
// Created: 4/19/2021
// Inputs:	NH4_EEP_Conc, Concentration of ammonia in solution
//			pH_TCor, pH of solution at temperature the ammonia reading is taken at
//			Temp; Temperature readings are taken at
//			IS; Ionic strength of solution
// Outputs: Calculated p-value
//********************************************************************************
float Calc_pNH4(float NH4_EEP_Conc, float pH_TCor, float SM_Na, float Temp, float IS)
{
	float pNH4 = -log10(NH4_EEP_Conc / 14000 * pow(10, -pH_TCor) / (pow(10, -pH_TCor) + pow(10, -9.25)) * Lambda_NH4(Temp, IS) + SM_Na * Lambda_Na(Temp, IS) * pow(10, LOG_K_NA_NH4));
	return pNH4;
	//	pNH4_Rinse = -log10(NH4_EEP_Rinse / 14000 * pow(10, -pH_TCor_Rinse) / (pow(10, -pH_TCor_Rinse) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_RINSE));
	//	pNH4_Cal_1 = -log10(NH4_EEP_Cal_1 / 14000 * pow(10, -pH_TCor_Cal_1) / (pow(10, -pH_TCor_Cal_1) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CAL_1) + 0.010928 * Lambda_Na(T_Cal, IS_CAL_1) * pow(10, LOG_K_NA_NH4));
	//	pNH4_Clean = -log10(NH4_EEP_Clean / 14000 * pow(10, -pH_TCor_Clean) / (pow(10, -pH_TCor_Clean) + pow(10, -9.25)) * Lambda_NH4(T_Cal, IS_CLEAN));
}


//********************************************************************************
// Collects the ISE mVs for all 10 spots and saves them in an array given a
// pointer. Made function to work with all ISE configurations based on the ISEConfig
// structure data
// Created: 4/12/2021
// Inputs:	ISE_mV_destination; pointer to array that I want mV saved into
//			SpotsToSave; bitwise mark which spots I want saved into the array pointed at,
//				1 will save, 0 will skip; 0xFFFF will save all
//			time_to_wait; how long to read ISEs for
//			PRINT_ISE_TIME_DATA; boolean switch to print ISE data every second
//			Sensor_Config; Tell function which configuration this is so it can
//				print and order data correctly
// Outputs: NONE
//********************************************************************************
void CollectISEmV(float * ISE_mV_destination, uint16_t SpotsToSave, uint16_t time_to_wait, uint8_t PRINT_ISE_TIME_DATA, struct ISEConfig *ISEConfig)
{
	uint16_t cycle = 0;
	uint8_t i;

	if((gui32Error & ABORT_ERRORS) == 0)
	{
		// GND RE for ISEs
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating

		ConnectMemory(0);

		//			SysCtlDelay(SysCtlClockGet()/3 * ISE_WAIT);

		DEBUG_PRINT(
		if(PRINT_ISE_TIME_DATA)
		{
//					UARTprintf("\tpH 1\tpH 2\tpH 3\tTH 1\tTH 2\tNH4 1\tNH4 2\tNH4 3\tCa 1\tCa 2\n");
			UARTprintf("Time");
			for(i = 0; i < ISEConfig->pH_H2.size; i++)
				UARTprintf("\tpH (H2) %d", i + 1);
			for(i = 0; i < ISEConfig->pH_Cr.size; i++)
				UARTprintf("\tpH (Cr) %d", i + 1);
			for(i = 0; i < ISEConfig->TH.size; i++)
				UARTprintf("\tTH %d", i + 1);
			for(i = 0; i < ISEConfig->NH4.size; i++)
				UARTprintf("\tNH4 %d", i + 1);
			for(i = 0; i < ISEConfig->Ca.size; i++)
				UARTprintf("\tCa %d", i + 1);
			UARTprintf("\n");
		}
		)

		TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 1); // Set periodic timer
		TimerEnable(TIMER1_BASE, TIMER_A);

		// Collect all the readings in a temporary buffer
		float Temp_ISE_Buffer[10];
		for(i = 0; i < 5; i++)
			Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i + 1, ADC1_CS_B, 10) - 3000)/5;
		for(i = 5; i < 10; i++)
			Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i - 4, ADC2_CS_B, 10) - 3000)/5;

		if(gABoard == ARV1_0B)
			Temp_ISE_Buffer[5] *= 5.0/2.0;	// Gain on this version was set to 2 instead of 5 on accident, will fix it on the next version of board

		// Sort the readings into an array that is organized by sensor type, pH->TH->NH4->Ca
		for(i = 0; i < 10; i++)
			if(((SpotsToSave >> i) & 1) && ISE_mV_destination != NULL)	// Only save data if flag for spot is marked in SpotsToSave variable
				ISE_mV_destination[i] = Temp_ISE_Buffer[ISEConfig->Location[i] - 1];

		DEBUG_PRINT(
		if(PRINT_ISE_TIME_DATA)
		{
			UARTprintf("%d", 0);
			for(i = 0; i < 10; i++)
				UARTprintf("\t%d", (int) (Temp_ISE_Buffer[ISEConfig->Location[i] - 1] * 1000));
			UARTprintf("\n");
		}
		)

		while(time_to_wait > 0)
		{
			if((gui32Error & ABORT_ERRORS) != 0)
				break;

			//					while(g_TimerPeriodicInterruptFlag == 0);
			while(g_TimerPeriodicInterruptFlag == 0)
			{
				// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
				// it will prevent the BT from reading incorrect data into the app
#ifdef MCU_ZXR
				if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
					ConnectMemory(1);
#else
				if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
					ConnectMemory(1);
#endif

			}
			g_TimerPeriodicInterruptFlag = 0;
			cycle++;

			// Collect all the readings in a temporary buffer
			for(i = 0; i < 5; i++)
				Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i + 1, ADC1_CS_B, 10) - 3000)/5;
			for(i = 5; i < 10; i++)
				Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i - 4, ADC2_CS_B, 10) - 3000)/5;

			if(gABoard == ARV1_0B)
				Temp_ISE_Buffer[5] *= 5.0/2.0;	// Gain on this version was set to 2 instead of 5 on accident, will fix it on the next version of board

			// Sort the readings into an array that is organized by sensor type, pH->TH->NH4->Ca
			for(i = 0; i < 10; i++)
				if(((SpotsToSave >> i) & 1) && ISE_mV_destination != NULL)	// Only save data if flag for spot is marked in SpotsToSave variable
					ISE_mV_destination[i] = Temp_ISE_Buffer[ISEConfig->Location[i] - 1];

			DEBUG_PRINT(
			if(PRINT_ISE_TIME_DATA)
			{
				UARTprintf("%d", cycle);
				for(i = 0; i < 10; i++)
					UARTprintf("\t%d", (int) (Temp_ISE_Buffer[ISEConfig->Location[i] - 1] * 1000));
				UARTprintf("\n");
			}
			)

			time_to_wait--;
		}
		TimerDisable(TIMER1_BASE, TIMER_A);
		g_TimerPeriodicInterruptFlag = 0;
	}

	ConnectMemory(1);
}

//**************************************************************************
// Pumps and measures ISEs for a certain amount of time, then saves ISE mV
// into the provided buffer
// Created: 7/1/2021
// Parameters:  Direction: FW or BW
//				EndDelay; Delay to end pump at to adjust pump speed
//				ISE_mV_destination, buffer to save ISE mV into
//				SpotsToSave; Bit-mask to determine which ISEs to save into buffer
//				Time; time to pump for while reading ISEs
//				PRINT_ISE_TIME_DATA; flag whether or not to print ISE data
//				ISEConfig; ISE structure to identify which type of sensor is being used
//**************************************************************************
void CollectISEmV_WhilePumping(uint8_t Direction, uint32_t EndDelay, float * ISE_mV_destination, uint16_t SpotsToSave, uint16_t Time, uint8_t PRINT_ISE_TIME_DATA, struct ISEConfig *ISEConfig){

	uint16_t cycle = 0;
	uint16_t i;
	uint32_t StepsTravelled = 0;

	if((gui32Error & ABORT_ERRORS) == 0)
	{
		// GND RE for ISEs
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating

		ConnectMemory(0);

		//			SysCtlDelay(SysCtlClockGet()/3 * ISE_WAIT);

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

		DEBUG_PRINT(
		if(PRINT_ISE_TIME_DATA)
		{
//					UARTprintf("\tpH 1\tpH 2\tpH 3\tTH 1\tTH 2\tNH4 1\tNH4 2\tNH4 3\tCa 1\tCa 2\n");
			UARTprintf("Time");
			for(i = 0; i < ISEConfig->pH_H2.size; i++)
				UARTprintf("\tpH (H2) %d", i + 1);
			for(i = 0; i < ISEConfig->pH_Cr.size; i++)
				UARTprintf("\tpH (Cr) %d", i + 1);
			for(i = 0; i < ISEConfig->TH.size; i++)
				UARTprintf("\tTH %d", i + 1);
			for(i = 0; i < ISEConfig->NH4.size; i++)
				UARTprintf("\tNH4 %d", i + 1);
			for(i = 0; i < ISEConfig->Ca.size; i++)
				UARTprintf("\tCa %d", i + 1);
			UARTprintf("\n");
		}
		)

#ifdef MCU_ZXR
		GPIOPinWrite(IO_PUMP_SLEEP_BASE, IO_PUMP_SLEEP_PIN, IO_PUMP_SLEEP_PIN);					// PUMP_SLEEP   = HIGH (turn ON)

		if(Direction == FW)
			GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, IO_PUMP_DIR_PIN);				// DIR  = HIGH
		else
			GPIOPinWrite(IO_PUMP_DIR_BASE, IO_PUMP_DIR_PIN, ~IO_PUMP_DIR_PIN);				// DIR  = LOW
#else	// MCU_ZXR
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)

		if(Direction == FW)
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
		else
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
#endif	// MCU_ZXR

		SysCtlDelay(16000);

		TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 1); // Set periodic timer
		TimerEnable(TIMER1_BASE, TIMER_A);
		float Temp_ISE_Buffer[10];

		while(Time > 0)
		{
			if((gui32Error & ABORT_ERRORS) != 0)
				break;

			//					while(g_TimerPeriodicInterruptFlag == 0);
			while(g_TimerPeriodicInterruptFlag == 0)
			{
				// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
				// it will prevent the BT from reading incorrect data into the app
#ifdef MCU_ZXR
				if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
					ConnectMemory(1);

				uint32_t Steps_accelerated = 0;
				GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, IO_PUMP_STEP_PIN);				// STEP = HIGH
				SysCtlDelay(Delay);									// wait a bit
				GPIOPinWrite(IO_PUMP_STEP_BASE, IO_PUMP_STEP_PIN, ~IO_PUMP_STEP_PIN);				// STEP = LOW
				SysCtlDelay(Delay);
#else
				if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
					ConnectMemory(1);

				uint32_t Steps_accelerated = 0;
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
				SysCtlDelay(Delay);									// wait a bit
				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
				SysCtlDelay(Delay);
#endif
							// wait a bit
				StepsTravelled++;
				Steps_accelerated++;

				if(Steps_accelerated < (Accel_Steps))
				{
					Delay -= DelayDecrease;
					if (Delay < EndDelay)
						Delay = EndDelay;
				}
//				else if(StepsToGo <= (Accel_Steps + 1))
//				{
//					Delay += DelayDecrease;
//				}
				//			Delay = Delay - DelayDecrease;
				//			if (Delay < EndDelay)
				//				Delay = EndDelay;

				if((gui32Error & ABORT_ERRORS) != 0)
					break;

			}
			g_TimerPeriodicInterruptFlag = 0;
			cycle++;

			// Decelerate pump
			for(i = 0; i < Accel_Steps; i++)
			{
				Delay += DelayDecrease;

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
				StepsTravelled++;
			}

			// Collect all the readings in a temporary buffer
//			userDelay(10, 1);	// Delay to try to get rid of noise, realized "noise" is the dead spot in the pump
			for(i = 0; i < 5; i++)
				Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i + 1, ADC1_CS_B, 10) - 3000)/5;
			for(i = 5; i < 10; i++)
				Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i - 4, ADC2_CS_B, 10) - 3000)/5;

			// Sort the readings into an array that is organized by sensor type, pH->TH->NH4->Ca
			for(i = 0; i < 10; i++)
				if(((SpotsToSave >> i) & 1) && ISE_mV_destination != NULL)	// Only save data if flag for spot is marked in SpotsToSave variable
					ISE_mV_destination[i] = Temp_ISE_Buffer[ISEConfig->Location[i] - 1];

			DEBUG_PRINT(
			if(PRINT_ISE_TIME_DATA)
			{
				UARTprintf("%d", cycle);
				for(i = 0; i < 10; i++)
					UARTprintf("\t%d", (int) (Temp_ISE_Buffer[ISEConfig->Location[i] - 1] * 1000));
				UARTprintf("\n");
			}
			)

			Time--;
		}
		TimerDisable(TIMER1_BASE, TIMER_A);
		g_TimerPeriodicInterruptFlag = 0;
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
			g_PumpStepsTravelled += StepsTravelled;
		else
			g_PumpStepsTravelled -= StepsTravelled;
	}
	DEBUG_PRINT(UARTprintf("Travelled %d steps while measuring!\n", StepsTravelled);)

	ConnectMemory(1);
}

//#ifndef MEMORY_V4
////**************************************************************************
//// Pumps while reading pH sensors until they read within a certain pH range,
//// limit is set by max number of steps, then reads all ISEs
//// Parameters:  Direction: FW or BW
////				EndDelay; Delay to end pump at to adjust pump speed
////				ISE_mV_destination, buffer to save ISE mV into
////				SpotsToSave; Bit-mask to determine which ISEs to save into buffer
////				Max_Steps; maximum number of steps to pump before giving up on looking for pH
////				Min_pH; Minimum pH to look for
////				Max_pH; Maximum pH to look for
////				PRINT_ISE_TIME_DATA; flag whether or not to print ISE data
////				ISEConfig; ISE structure to identify which type of sensor is being used
////				Cal_Number; Current calibration for looking up data
////				pH_TCor_Rinse; pH value for the temperature corrected rinse value, used to calculate sample pH
////				ISE_E_Rinse; mVs for all ISEs in rinse, used to calculate sample pH
//// Returns: StepsTravelled; Steps made before finding pH
////**************************************************************************
//int32_t PumpUntilpH(uint8_t Direction, uint32_t EndDelay, float * ISE_mV_destination, uint16_t SpotsToSave, uint32_t Max_Steps, float Min_pH, float Max_pH, uint8_t PRINT_ISE_TIME_DATA, struct ISEConfig *ISEConfig, uint16_t Cal_Number, float pH_TCor_Rinse, float * ISE_E_Rinse){
//
//	uint16_t cycle = 0;
//	uint16_t i;
//	uint32_t StepsTravelled = 0;
//
//	if((gui32Error & ABORT_ERRORS) == 0)
//	{
//		ConnectMemory(1);
//		float Temp = MeasureTemperature(1);
//		float ISE_EEP_Slope[10] = {0,0,0,0,0,0,0,0,0,0};
//		uint16_t Cal_page = Find_Cal_page(Cal_Number);
//
//		uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
//		uint8_t Last_cal_passed[10];
//		memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
//		for(i = 0; i < 10; i++)
//			if(Last_cal_passed[i] == 0xFF)
//				Last_cal_passed[i] = Cal_Number;
//
//		for(i = 0; i < 10; i++)
//			ISE_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i]), OFFSET_ISE_1_SLOPE + (i * 4), 4));
//
//		float T_EEP_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));
//
//		ConnectMemory(0);
//
//		// GND RE for ISEs
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating
//
//		// Fastest pump turns is with delay of 3000, needs (8000 - 3000)/300 = 16.66 steps
//		int Delay = 8000;			// starting delay
//		//		int EndDelay = 6000;		// 3000
//		if(EndDelay > Delay)
//			Delay = EndDelay;
//
//		int DelayDecrease = 20; //100; //300;
//
//		// Number of steps that it will be accelerating and decelerating
//		uint32_t Accel_Steps = (Delay - EndDelay) / DelayDecrease;
////		if(NumberOfSteps < (2 * (Accel_Steps + 1)))	// Adding 1 because using integers gives chance there's a decimal thats lost, want to overshoot delay then correct
////			Accel_Steps = NumberOfSteps / 2;	// Accelerate for half the steps, decelerate the other half
//
//		if(PRINT_ISE_TIME_DATA)
//		{
////					UARTprintf("\tpH 1\tpH 2\tpH 3\tTH 1\tTH 2\tNH4 1\tNH4 2\tNH4 3\tCa 1\tCa 2\n");
//			UARTprintf("Time");
//			for(i = 0; i < ISEConfig->pH_H2.size; i++)
//				UARTprintf("\tpH (H2) %d", i + 1);
//			for(i = 0; i < ISEConfig->pH_Cr.size; i++)
//				UARTprintf("\tpH (Cr) %d", i + 1);
//			for(i = 0; i < ISEConfig->TH.size; i++)
//				UARTprintf("\tTH %d", i + 1);
//			for(i = 0; i < ISEConfig->NH4.size; i++)
//				UARTprintf("\tNH4 %d", i + 1);
//			for(i = 0; i < ISEConfig->Ca.size; i++)
//				UARTprintf("\tCa %d", i + 1);
//			UARTprintf("\n");
//		}
//
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);					// PUMP_SLEEP   = HIGH (turn ON)
//
//		if(Direction == FW)
//			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);				// DIR  = HIGH
//		else
//			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, ~GPIO_PIN_5);				// DIR  = LOW
//		SysCtlDelay(16000);
//
//		TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 1); // Set periodic timer
//		TimerEnable(TIMER1_BASE, TIMER_A);
//		float Temp_ISE_Buffer[10];
//		float pH_Samp_T1[10] = {0,0,0,0,0,0,0,0,0,0};
//		uint8_t T_Chosen_pH = 0;
//
//		while(StepsTravelled < Max_Steps && (pH_Samp_T1[T_Chosen_pH] < Min_pH || pH_Samp_T1[T_Chosen_pH] > Max_pH))
//		{
//			if((gui32Error & ABORT_ERRORS) != 0)
//				break;
//
//			//					while(g_TimerPeriodicInterruptFlag == 0);
//			while(g_TimerPeriodicInterruptFlag == 0)
//			{
//				// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
//				// it will prevent the BT from reading incorrect data into the app
//				if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
//					ConnectMemory(1);
//
//				uint32_t Steps_accelerated = 0;
//				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
//				SysCtlDelay(Delay);									// wait a bit
//				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
//				SysCtlDelay(Delay);									// wait a bit
//				StepsTravelled++;
//				Steps_accelerated++;
//
//				if(Steps_accelerated < (Accel_Steps))
//				{
//					Delay -= DelayDecrease;
//					if (Delay < EndDelay)
//						Delay = EndDelay;
//				}
//
//				if((gui32Error & ABORT_ERRORS) != 0)
//					break;
//			}
//			g_TimerPeriodicInterruptFlag = 0;
//			cycle++;
//
//			// Decelerate pump
//			for(i = 0; i < Accel_Steps; i++)
//			{
//				Delay += DelayDecrease;
//
//				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);				// STEP = HIGH
//				SysCtlDelay(Delay);									// wait a bit
//				GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~GPIO_PIN_4);				// STEP = LOW
//				SysCtlDelay(Delay);									// wait a bit
//				StepsTravelled++;
//			}
//
//			// Collect all the readings in a temporary buffer
////			userDelay(10, 1);	// Delay to try to get rid of noise, realized "noise" is the dead spot in the pump
//			for(i = 0; i < 5; i++)
//				Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i + 1, ADC1_CS_B, 10) - 3000)/5;
//			for(i = 5; i < 10; i++)
//				Temp_ISE_Buffer[i] = (2 * ADCReadAvg(i - 4, ADC2_CS_B, 10) - 3000)/5;
//
//			// Sort the readings into an array that is organized by sensor type, pH->TH->NH4->Ca
//			for(i = 0; i < 10; i++)
//				if(((SpotsToSave >> i) & 1) && ISE_mV_destination != NULL)	// Only save data if flag for spot is marked in SpotsToSave variable
//					ISE_mV_destination[i] = Temp_ISE_Buffer[ISEConfig->Location[i] - 1];
//
//			// Calculate pH so we know when to stop
//			float pH_H2_Samp_T1[2];
//			for(i = 0; i < ISEConfig->pH_H2.size; i++)
//			{
//				float pH_H2_Slope_Samp_T1T = ISE_EEP_Slope[ISEConfig->pH_H2.index + i] * (Temp + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
//				pH_H2_Samp_T1[i] = pH_TCor_Rinse + ((ISE_mV_destination[ISEConfig->pH_H2.index + i] - ISE_E_Rinse[ISEConfig->pH_H2.index + i]) / pH_H2_Slope_Samp_T1T); // pH of sample
//			}
//
//			for(i = 0; i < ISEConfig->pH_Cr.size; i++)
//			{
//				float pH_Cr_Slope_Samp_T1T = ISE_EEP_Slope[ISEConfig->pH_Cr.index + i] * (Temp + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
//				pH_Samp_T1[i] = pH_TCor_Rinse + ((ISE_mV_destination[ISEConfig->pH_Cr.index + i] - ISE_E_Rinse[ISEConfig->pH_Cr.index + i]) / pH_Cr_Slope_Samp_T1T); // pH of sample
//			}
//
//			ConnectMemory(1);
//#ifdef SOLUTION_IN_STRUCT
//			T_Chosen_pH = Choose_pH_Sensor(Cal_Number, pH_Samp_T1, &ISE_E_Rinse[ISEConfig->pH_Cr.index], Temp, *ISEConfig);
//#else
//			T_Chosen_pH = Choose_pH_Sensor(Cal_Number, pH_Samp_T1, &ISE_E_Rinse[ISEConfig->pH_Cr.index], Temp, *ISEConfig);
//#endif
//			ConnectMemory(0);
//
//			if(PRINT_ISE_TIME_DATA)
//			{
//				UARTprintf("%d", cycle);
//				for(i = 0; i < 10; i++)
//					UARTprintf("\t%d", (int) (Temp_ISE_Buffer[ISEConfig->Location[i] - 1] * 1000));
//				UARTprintf("\tpH H2:");
//				for(i = 0; i < ISEConfig->pH_Cr.size; i++)
//					UARTprintf("\t%d", (int) (pH_H2_Samp_T1[i] * 1000));
//				UARTprintf("\tpH Cr:");
//				for(i = 0; i < ISEConfig->pH_Cr.size; i++)
//					UARTprintf("\t%d", (int) (pH_Samp_T1[i] * 1000));
//				UARTprintf("\n");
//			}
//
////			UARTprintf("pH of mixed T1:\n");
////			UARTprintf("pH H2:");
////			for(i = 0; i < ISEs.pH_H2.size; i++)
////				UARTprintf("\t%d", (int) (pH_H2_Samp_T1[i + (mixing_index * 10)] * 1000));
////			UARTprintf("\n");
////			UARTprintf("pH Cr:");
////			for(i = 0; i < ISEConfig->pH_Cr.size; i++)
////				UARTprintf("\t%d", (int) (pH_Cr_Samp_T1[i + (mixing_index * 10)] * 1000));
////			UARTprintf("\n");
//		}
//		TimerDisable(TIMER1_BASE, TIMER_A);
//		g_TimerPeriodicInterruptFlag = 0;
//	}
//
//	if((gui32Error & ABORT_ERRORS) != 0)
//		SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us before turning on pump sleep
//
//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, ~GPIO_PIN_4);					// PUMP_SLEEP   = OFF (turn ON)
//
//	if(g_PumpStepsTravelled != 0xFFFFFFFF)	// Don't adjust this if pump home position hasn't been found yet, assuming I never pump -1 steps...
//	{
//		if(Direction == FW)
//			g_PumpStepsTravelled += StepsTravelled;
//		else
//			g_PumpStepsTravelled -= StepsTravelled;
//	}
//	UARTprintf("Travelled %d steps while measuring!\n", StepsTravelled);
//
//	ConnectMemory(1);
//
//	return StepsTravelled;
//}
//#endif

//********************************************************************************
// Fills in the data for the ISEConfig structure pointed at based on the sensor
// configuration given, put into this function so I only have to update in one spot
// as we change ISE configurations
// Created: 4/12/2021
// Inputs:	*ISEConfig; pointer to the ISE configuration structure to be filled in
// Outputs: NONE
//********************************************************************************
void FillISEStruct(struct ISEConfig *ISEConfig)
{
	ConnectMemory(1);

	ISEConfig->Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);

	// Almost all configs have these settings so initilize here, only ones that don't are the all of one type die
	ISEConfig->CalDelay = 1;
	ISEConfig->RunAlk = 1;

	ISEConfig->pH_H2.size = 2;
	ISEConfig->pH_Cr.size = 2;
	ISEConfig->TH.size = 2;
	ISEConfig->NH4.size = 2;
	ISEConfig->Ca.size = 2;

	ISEConfig->pH_H2.index = 0;
	ISEConfig->pH_Cr.index = 2;
	ISEConfig->TH.index = 4;
	ISEConfig->NH4.index = 6;
	ISEConfig->Ca.index = 8;

	// With only two sensors per type only need 1 bit per type
	ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
	ISEConfig->pH_Cr.StorBit = 1;
	ISEConfig->TH.StorBit = 2;
	ISEConfig->NH4.StorBit = 3;
	ISEConfig->Ca.StorBit = 4;

	switch (ISEConfig->Config) {
	case ORIGINAL_CART:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 1;
		ISEConfig->RunAlk = 1;

		ISEConfig->pH_H2.size = 0;
		ISEConfig->pH_Cr.size = 3;
		ISEConfig->TH.size = 2;
		ISEConfig->NH4.size = 3;
		ISEConfig->Ca.size = 2;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 0;
		ISEConfig->TH.index = 3;
		ISEConfig->NH4.index = 5;
		ISEConfig->Ca.index = 8;

		// Sensor types with 3 spots need 2 bits, all others need 1 bit
		ISEConfig->pH_H2.StorBit = 6;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 0;
		ISEConfig->TH.StorBit = 2;
		ISEConfig->NH4.StorBit = 3;
		ISEConfig->Ca.StorBit = 5;

		break;
	}
	case PH_CL_CART:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 2;
		ISEConfig->RunAlk = 0;

		// Fill in the number of each type of sensor
		ISEConfig->pH_H2.size = 0;
		ISEConfig->pH_Cr.size = 10;
		ISEConfig->TH.size = 0;
		ISEConfig->NH4.size = 0;
		ISEConfig->Ca.size = 0;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 0;
		ISEConfig->TH.index = 0;
		ISEConfig->NH4.index = 0;
		ISEConfig->Ca.index = 0;

		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 0;
		ISEConfig->TH.StorBit = 0;
		ISEConfig->NH4.StorBit = 0;
		ISEConfig->Ca.StorBit = 0;
		break;
	}
	case H2_CART:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 8, 6, 4, 9, 7, 5, 3, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
//		ISEConfig->CalDelay = 1;
//		ISEConfig->RunAlk = 1;
//
//		ISEConfig->pH_H2.size = 2;
//		ISEConfig->pH_Cr.size = 2;
//		ISEConfig->TH.size = 2;
//		ISEConfig->NH4.size = 2;
//		ISEConfig->Ca.size = 2;
//
//		ISEConfig->pH_H2.index = 0;
//		ISEConfig->pH_Cr.index = 2;
//		ISEConfig->TH.index = 4;
//		ISEConfig->NH4.index = 6;
//		ISEConfig->Ca.index = 8;
//
//		// With only two sensors per type only need 1 bit per type
//		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
//		ISEConfig->pH_Cr.StorBit = 1;
//		ISEConfig->TH.StorBit = 2;
//		ISEConfig->NH4.StorBit = 3;
//		ISEConfig->Ca.StorBit = 4;

		break;
	}
	case PH_H2_CART:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 1;
		ISEConfig->RunAlk = 1;

		ISEConfig->pH_H2.size = 5;
		ISEConfig->pH_Cr.size = 5;
		ISEConfig->TH.size = 0;
		ISEConfig->NH4.size = 0;
		ISEConfig->Ca.size = 0;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 5;
		ISEConfig->TH.index = 0;
		ISEConfig->NH4.index = 0;
		ISEConfig->Ca.index = 0;

		// 5 sensor per type I need 3 bits each
		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 3;
		ISEConfig->TH.StorBit = 0;
		ISEConfig->NH4.StorBit = 0;
		ISEConfig->Ca.StorBit = 0;

		break;
	}
	case H2_ONLY:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 1;
		ISEConfig->RunAlk = 1;

		// Fill in the number of each type of sensor
		ISEConfig->pH_H2.size = 10;
		ISEConfig->pH_Cr.size = 0;
		ISEConfig->TH.size = 0;
		ISEConfig->NH4.size = 0;
		ISEConfig->Ca.size = 0;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 0;
		ISEConfig->TH.index = 0;
		ISEConfig->NH4.index = 0;
		ISEConfig->Ca.index = 0;

		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 0;
		ISEConfig->TH.StorBit = 0;
		ISEConfig->NH4.StorBit = 0;
		ISEConfig->Ca.StorBit = 0;
		break;
	}
	case CA_ONLY:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 1;
		ISEConfig->RunAlk = 0;

		// Fill in the number of each type of sensor
		ISEConfig->pH_H2.size = 0;
		ISEConfig->pH_Cr.size = 0;
		ISEConfig->TH.size = 0;
		ISEConfig->NH4.size = 0;
		ISEConfig->Ca.size = 10;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 0;
		ISEConfig->TH.index = 0;
		ISEConfig->NH4.index = 0;
		ISEConfig->Ca.index = 0;

		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 0;
		ISEConfig->TH.StorBit = 0;
		ISEConfig->NH4.StorBit = 0;
		ISEConfig->Ca.StorBit = 0;
		break;
	}
	case TH_ONLY:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 1;
		ISEConfig->RunAlk = 0;

		// Fill in the number of each type of sensor
		ISEConfig->pH_H2.size = 0;
		ISEConfig->pH_Cr.size = 0;
		ISEConfig->TH.size = 10;
		ISEConfig->NH4.size = 0;
		ISEConfig->Ca.size = 0;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 0;
		ISEConfig->TH.index = 0;
		ISEConfig->NH4.index = 0;
		ISEConfig->Ca.index = 0;

		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 0;
		ISEConfig->TH.StorBit = 0;
		ISEConfig->NH4.StorBit = 0;
		ISEConfig->Ca.StorBit = 0;
		break;
	}
	case NH4_ONLY:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 1;
		ISEConfig->RunAlk = 0;

		// Fill in the number of each type of sensor
		ISEConfig->pH_H2.size = 0;
		ISEConfig->pH_Cr.size = 0;
		ISEConfig->TH.size = 0;
		ISEConfig->NH4.size = 10;
		ISEConfig->Ca.size = 0;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 0;
		ISEConfig->TH.index = 0;
		ISEConfig->NH4.index = 0;
		ISEConfig->Ca.index = 0;

		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 0;
		ISEConfig->TH.StorBit = 0;
		ISEConfig->NH4.StorBit = 0;
		ISEConfig->Ca.StorBit = 0;
		break;
	}
	case CR_CA_SWAP:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 8, 2, 1, 9, 7, 5, 3, 6, 4};
		memcpy(ISEConfig->Location, TempLocation, 10);
//		ISEConfig->CalDelay = 1;
//		ISEConfig->RunAlk = 1;
//
//		ISEConfig->pH_H2.size = 2;
//		ISEConfig->pH_Cr.size = 2;
//		ISEConfig->TH.size = 2;
//		ISEConfig->NH4.size = 2;
//		ISEConfig->Ca.size = 2;
//
//		ISEConfig->pH_H2.index = 0;
//		ISEConfig->pH_Cr.index = 2;
//		ISEConfig->TH.index = 4;
//		ISEConfig->NH4.index = 6;
//		ISEConfig->Ca.index = 8;
//
//		// With only two sensors per type only need 1 bit per type
//		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
//		ISEConfig->pH_Cr.StorBit = 1;
//		ISEConfig->TH.StorBit = 2;
//		ISEConfig->NH4.StorBit = 3;
//		ISEConfig->Ca.StorBit = 4;

		break;
	}
	case ACROSS:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 6, 5, 3, 4, 2, 1};
		memcpy(ISEConfig->Location, TempLocation, 10);
//		ISEConfig->CalDelay = 1;
//		ISEConfig->RunAlk = 1;
//
//		ISEConfig->pH_H2.size = 2;
//		ISEConfig->pH_Cr.size = 2;
//		ISEConfig->TH.size = 2;
//		ISEConfig->NH4.size = 2;
//		ISEConfig->Ca.size = 2;
//
//		ISEConfig->pH_H2.index = 0;
//		ISEConfig->pH_Cr.index = 2;
//		ISEConfig->TH.index = 4;
//		ISEConfig->NH4.index = 6;
//		ISEConfig->Ca.index = 8;
//
//		// With only two sensors per type only need 1 bit per type
//		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
//		ISEConfig->pH_Cr.StorBit = 1;
//		ISEConfig->TH.StorBit = 2;
//		ISEConfig->NH4.StorBit = 3;
//		ISEConfig->Ca.StorBit = 4;

		break;
	}
	case ACROSS_V2:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {2, 1, 3, 4, 10, 9, 7, 8, 6, 5};
		memcpy(ISEConfig->Location, TempLocation, 10);
//		ISEConfig->CalDelay = 1;
//		ISEConfig->RunAlk = 1;
//
//		ISEConfig->pH_H2.size = 2;
//		ISEConfig->pH_Cr.size = 2;
//		ISEConfig->TH.size = 2;
//		ISEConfig->NH4.size = 2;
//		ISEConfig->Ca.size = 2;
//
//		ISEConfig->pH_H2.index = 0;
//		ISEConfig->pH_Cr.index = 2;
//		ISEConfig->TH.index = 4;
//		ISEConfig->NH4.index = 6;
//		ISEConfig->Ca.index = 8;
//
//		// With only two sensors per type only need 1 bit per type
//		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
//		ISEConfig->pH_Cr.StorBit = 1;
//		ISEConfig->TH.StorBit = 2;
//		ISEConfig->NH4.StorBit = 3;
//		ISEConfig->Ca.StorBit = 4;

		break;
	}
	case ACROSS_V3:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {3, 1, 4, 2, 10, 9, 7, 8, 6, 5};
		memcpy(ISEConfig->Location, TempLocation, 10);
//		ISEConfig->CalDelay = 1;
//		ISEConfig->RunAlk = 1;
//
//		ISEConfig->pH_H2.size = 2;
//		ISEConfig->pH_Cr.size = 2;
//		ISEConfig->TH.size = 2;
//		ISEConfig->NH4.size = 2;
//		ISEConfig->Ca.size = 2;
//
//		ISEConfig->pH_H2.index = 0;
//		ISEConfig->pH_Cr.index = 2;
//		ISEConfig->TH.index = 4;
//		ISEConfig->NH4.index = 6;
//		ISEConfig->Ca.index = 8;
//
//		// With only two sensors per type only need 1 bit per type
//		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
//		ISEConfig->pH_Cr.StorBit = 1;
//		ISEConfig->TH.StorBit = 2;
//		ISEConfig->NH4.StorBit = 3;
//		ISEConfig->Ca.StorBit = 4;

		break;
	}
	case ACROSS_V4:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 7, 8, 4, 2, 3, 1, 6, 5};
		memcpy(ISEConfig->Location, TempLocation, 10);
//		ISEConfig->CalDelay = 1;
//		ISEConfig->RunAlk = 1;
//
//		ISEConfig->pH_H2.size = 2;
//		ISEConfig->pH_Cr.size = 2;
//		ISEConfig->TH.size = 2;
//		ISEConfig->NH4.size = 2;
//		ISEConfig->Ca.size = 2;
//
//		ISEConfig->pH_H2.index = 0;
//		ISEConfig->pH_Cr.index = 2;
//		ISEConfig->TH.index = 4;
//		ISEConfig->NH4.index = 6;
//		ISEConfig->Ca.index = 8;
//
//		// With only two sensors per type only need 1 bit per type
//		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
//		ISEConfig->pH_Cr.StorBit = 1;
//		ISEConfig->TH.StorBit = 2;
//		ISEConfig->NH4.StorBit = 3;
//		ISEConfig->Ca.StorBit = 4;

		break;
	}
	case ACROSS_V5:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {6, 5, 7, 8, 4, 2, 3, 1, 10, 9};
		memcpy(ISEConfig->Location, TempLocation, 10);

		break;
	}
	case ACROSS_V6:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {3, 1, 7, 8, 4, 2, 6, 5, 10, 9};
		memcpy(ISEConfig->Location, TempLocation, 10);

		break;
	}
	case ACROSS_V7:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 2, 1, 6, 5, 7, 8, 3, 4};
		memcpy(ISEConfig->Location, TempLocation, 10);

		break;
	}
	case CONFIG_V8:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 8, 6, 4, 2, 1, 9, 7, 5, 3};
		memcpy(ISEConfig->Location, TempLocation, 10);

		break;
	}
	case CONFIG_V9:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {7, 8, 3, 4, 2, 1, 10, 9, 6, 5};
		memcpy(ISEConfig->Location, TempLocation, 10);

		break;
	}
	case CONFIG_V10:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {7, 8, 3, 4, 2, 1, 6, 5, 10, 9};
		memcpy(ISEConfig->Location, TempLocation, 10);

		break;
	}
	case ACROSS_V7_BACKWARDS:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {2, 1, 10, 9, 6, 5, 3, 4, 7, 8};
		memcpy(ISEConfig->Location, TempLocation, 10);

		break;
	}
	case DISINFECTION_CART:
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {6, 5, 3, 4, 2, 1, 10, 9, 7, 8};
		memcpy(ISEConfig->Location, TempLocation, 10);
		ISEConfig->CalDelay = 1;
		ISEConfig->RunAlk = 0;

		ISEConfig->pH_H2.size = 0;
		ISEConfig->pH_Cr.size = 6;
		ISEConfig->TH.size = 0;
		ISEConfig->NH4.size = 4;
		ISEConfig->Ca.size = 0;

		ISEConfig->pH_H2.index = 0;
		ISEConfig->pH_Cr.index = 0;
		ISEConfig->TH.index = 0;
		ISEConfig->NH4.index = 6;
		ISEConfig->Ca.index = 0;

		// Sensor types with 3 spots need 2 bits, all others need 1 bit
		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
		ISEConfig->pH_Cr.StorBit = 0;
		ISEConfig->TH.StorBit = 0;
		ISEConfig->NH4.StorBit = 4;
		ISEConfig->Ca.StorBit = 0;

		break;
	}
	default:	// Make the default case the AcrossV7 cartridge as this is what is going to market with
	{
		// Create map between hardware ISE # and firmware ISE #
		uint8_t TempLocation[10] = {10, 9, 2, 1, 6, 5, 7, 8, 3, 4};
		memcpy(ISEConfig->Location, TempLocation, 10);
//		ISEConfig->CalDelay = 1;
//		ISEConfig->RunAlk = 1;
//
//		ISEConfig->pH_H2.size = 2;
//		ISEConfig->pH_Cr.size = 2;
//		ISEConfig->TH.size = 2;
//		ISEConfig->NH4.size = 2;
//		ISEConfig->Ca.size = 2;
//
//		ISEConfig->pH_H2.index = 0;
//		ISEConfig->pH_Cr.index = 2;
//		ISEConfig->TH.index = 4;
//		ISEConfig->NH4.index = 6;
//		ISEConfig->Ca.index = 8;
//
//		// With only two sensors per type only need 1 bit per type
//		ISEConfig->pH_H2.StorBit = 0;	// Doubles as alkalinity store bit, therefore must be different from pH Cr
//		ISEConfig->pH_Cr.StorBit = 1;
//		ISEConfig->TH.StorBit = 2;
//		ISEConfig->NH4.StorBit = 3;
//		ISEConfig->Ca.StorBit = 4;

		break;
	}
	}	// Switch Sensor_Config
}

#ifdef SOLUTION_IN_STRUCT
//********************************************************************************
// Statically creates and fills in the data for a solution structure based on what's saved
// in the cartridge memory. If something isn't saved it will fill in with default
// values for the calibrants
// Size of this structure is 104 bytes, ran into an overflow problem with MemoryDump function
// Removed the structure from the stack by defining it here statically, since this will
// be used quite often it's ok for it to live forever.
// Should keep all assignments for this structure inside this function so I don't
// accidentally change solution values that will be used later..
// Probably should have just increased stack size...
// Created: 2/15/2022
// Inputs:	*Sols; pointer to the solution structure to be filled in
// Outputs: NONE
//********************************************************************************
struct SolutionVals* FillSolutionStruct(void)
{
	ConnectMemory(1);

	uint8_t * pui8Sols = MemoryRead(PAGE_SOLUTIONS, 0, 120);
	static struct SolutionVals Sols;

	Sols.pH_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_PH));
	Sols.Ca_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_CA));
	Sols.TH_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_TH));
	Sols.NH4_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_NH4));
	Sols.Cond_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_COND));

	Sols.pH_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_PH));
	Sols.Ca_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_CA));
	Sols.TH_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_TH));
	Sols.NH4_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_NH4));
	Sols.Cond_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_COND));

	Sols.pH_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_PH));
	Sols.Ca_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_CA));
	Sols.TH_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_TH));
	Sols.NH4_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_NH4));
	Sols.Cond_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_COND));

	Sols.pH_EEP_Clean = *((float *) (pui8Sols + OFFSET_CLEAN_PH));
	Sols.Ca_EEP_Clean = *((float *) (pui8Sols + OFFSET_CLEAN_CA));
	Sols.TH_EEP_Clean = *((float *) (pui8Sols + OFFSET_CLEAN_TH));
	Sols.NH4_EEP_Clean = *((float *) (pui8Sols + OFFSET_CLEAN_NH4));
	Sols.Cond_EEP_Clean = *((float *) (pui8Sols + OFFSET_CLEAN_COND));

	Sols.HCl_N = *((float *) (pui8Sols + OFFSET_T1_HCL_N));

	if(Sols.pH_EEP_Rinse != Sols.pH_EEP_Rinse)
	{
		Sols.pH_EEP_Rinse = 7.52;
		Sols.Ca_EEP_Rinse = 150;
		Sols.TH_EEP_Rinse = 300;
		Sols.NH4_EEP_Rinse = .77;
		Sols.Cond_EEP_Rinse = 931;

		Sols.pH_EEP_Cal_2 = 6.01;
		Sols.Ca_EEP_Cal_2 = 300;
		Sols.TH_EEP_Cal_2 = 600;
		Sols.NH4_EEP_Cal_2 = 2.7;
		Sols.Cond_EEP_Cal_2 = 2260;

		Sols.pH_EEP_Cal_1 = 4.31;
		Sols.Ca_EEP_Cal_1 = 0;
		Sols.TH_EEP_Cal_1 = 0;
		Sols.NH4_EEP_Cal_1 = 0.2;
		Sols.Cond_EEP_Cal_1 = 349;

		Sols.pH_EEP_Clean = 8.98;
		Sols.Ca_EEP_Clean = 10;
		Sols.TH_EEP_Clean = 20;
		Sols.NH4_EEP_Clean = 0;
		Sols.Cond_EEP_Clean = 1022;

		Sols.HCl_N = 0.0566;
	}

	Sols.IS_RINSE = *((float *) (pui8Sols + OFFSET_IS_RINSE));
	Sols.IS_CLEAN = *((float *) (pui8Sols + OFFSET_IS_CLEAN));
	Sols.IS_CAL_1 = *((float *) (pui8Sols + OFFSET_IS_CAL_1));
	Sols.IS_CAL_2 = *((float *) (pui8Sols + OFFSET_IS_CAL_2));

	if(Sols.IS_RINSE != Sols.IS_RINSE)
	{
		Sols.IS_RINSE = 0.0115;
		Sols.IS_CLEAN = 0.009;	// pH 7.5 Clean: 0.0187
		Sols.IS_CAL_1 = 0.0321;
		Sols.IS_CAL_2 = 0.00335;
	}

	Sols.K_T_pH_Rinse = *((float *) (pui8Sols + OFFSET_KT_RINSE));
	Sols.K_T_pH_Cal_1 = *((float *) (pui8Sols + OFFSET_KT_CAL_1));
	Sols.K_T_pH_Cal_2 = *((float *) (pui8Sols + OFFSET_KT_CAL_2));
	Sols.K_T_pH_Clean_Sq = *((float *) (pui8Sols + OFFSET_KT_CLEAN_SQ));
	Sols.K_T_pH_Clean_Ln = *((float *) (pui8Sols + OFFSET_KT_CLEAN_LN));

	if(Sols.K_T_pH_Rinse != Sols.K_T_pH_Rinse)
	{
		Sols.K_T_pH_Rinse = -0.0129;
		Sols.K_T_pH_Cal_1 = -0.0025;
		Sols.K_T_pH_Cal_2 = -0.0243;
		Sols.K_T_pH_Clean_Sq = .00007;
		Sols.K_T_pH_Clean_Ln = -.0071;
	}

	// Read the conductivity TComps from memory separately because these values are past 128 bytes
	pui8Sols = MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND_TCOMP, 16);
	Sols.Rinse_Cond_TComp = *((float *) (pui8Sols + (OFFSET_RINSE_COND_TCOMP - 128)));
	Sols.Cal_1_Cond_TComp = *((float *) (pui8Sols + (OFFSET_CAL_1_COND_TCOMP - 128)));
	Sols.Cal_2_Cond_TComp = *((float *) (pui8Sols + (OFFSET_CAL_2_COND_TCOMP - 128)));
	Sols.Clean_Cond_TComp = *((float *) (pui8Sols + (OFFSET_CLEAN_COND_TCOMP - 128)));

	if(Sols.Rinse_Cond_TComp != Sols.Rinse_Cond_TComp)
	{
		Sols.Rinse_Cond_TComp = COND_TCOMP_RINSE;
		Sols.Cal_1_Cond_TComp = COND_TCOMP_CAL_1;
		Sols.Cal_2_Cond_TComp = COND_TCOMP_CAL_2;
		Sols.Clean_Cond_TComp = COND_TCOMP_CLEAN;
	}

	return &Sols;
}


////********************************************************************************
//// Fills in the data for the solution structure pointed at based on what's saved
//// in the cartridge memory. If something isn't saved it will fill in with default
//// values for the calibrants
//// Created: 2/15/2022
//// Inputs:	*Sols; pointer to the solution structure to be filled in
//// Outputs: NONE
////********************************************************************************
//void FillSolutionStruct(struct SolutionVals *Sols)
//{
//	ConnectMemory(1);
//
//	uint8_t * pui8Sols = MemoryRead(PAGE_SOLUTIONS, 0, 120);
//
//	Sols->pH_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_PH));
//	Sols->Ca_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_CA));
//	Sols->TH_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_TH));
//	Sols->NH4_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_NH4));
//	Sols->Cond_EEP_Rinse = *((float *) (pui8Sols + OFFSET_RINSE_COND));
//
//	Sols->pH_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_PH));
//	Sols->Ca_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_CA));
//	Sols->TH_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_TH));
//	Sols->Cond_EEP_Cal_2 = *((float *) (pui8Sols + OFFSET_CAL_2_COND));
//
//	Sols->pH_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_PH));
//	Sols->Ca_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_CA));
//	Sols->TH_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_TH));
//	Sols->NH4_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_NH4));
//	Sols->Cond_EEP_Cal_1 = *((float *) (pui8Sols + OFFSET_CAL_1_COND));
//
//	Sols->pH_EEP_Clean = *((float *) (pui8Sols + OFFSET_CLEAN_PH));
//	Sols->NH4_EEP_Clean = *((float *) (pui8Sols + OFFSET_CLEAN_NH4));
//
//	Sols->HCl_N = *((float *) (pui8Sols + OFFSET_T1_HCL_N));
//
//	if(Sols->pH_EEP_Rinse != Sols->pH_EEP_Rinse)
//	{
//		Sols->pH_EEP_Rinse = 7.54;
//		Sols->Ca_EEP_Rinse = 50;
//		Sols->TH_EEP_Rinse = 100;
//		Sols->NH4_EEP_Rinse = .77;
//		Sols->Cond_EEP_Rinse = 939;
//
//		Sols->pH_EEP_Cal_2 = 9.32;
//		Sols->Ca_EEP_Cal_2 = 10;
//		Sols->TH_EEP_Cal_2 = 20;
//		Sols->Cond_EEP_Cal_2 = 279;
//
//		Sols->pH_EEP_Cal_1 = 4.24;
//		Sols->Ca_EEP_Cal_1 = 300;
//		Sols->TH_EEP_Cal_1 = 365;
//		Sols->NH4_EEP_Cal_1 = 2.7;
//		Sols->Cond_EEP_Cal_1 = 2070;
//
//		Sols->pH_EEP_Clean = 6;
//		Sols->NH4_EEP_Clean = .33;
//
//		Sols->HCl_N = 0.0566;
//	}
//
//	Sols->IS_RINSE = *((float *) (pui8Sols + OFFSET_IS_RINSE));
//	Sols->IS_CLEAN = *((float *) (pui8Sols + OFFSET_IS_CLEAN));
//	Sols->IS_CAL_1 = *((float *) (pui8Sols + OFFSET_IS_CAL_1));
//	Sols->IS_CAL_2 = *((float *) (pui8Sols + OFFSET_IS_CAL_2));
//
//	if(Sols->IS_RINSE != Sols->IS_RINSE)
//	{
//		Sols->IS_RINSE = 0.0115;
//		Sols->IS_CLEAN = 0.009;	// pH 7.5 Clean: 0.0187
//		Sols->IS_CAL_1 = 0.0321;
//		Sols->IS_CAL_2 = 0.00335;
//	}
//
//	Sols->K_T_pH_Rinse = *((float *) (pui8Sols + OFFSET_KT_RINSE));
//	Sols->K_T_pH_Cal_1 = *((float *) (pui8Sols + OFFSET_KT_CAL_1));
//	Sols->K_T_pH_Cal_2 = *((float *) (pui8Sols + OFFSET_KT_CAL_2));
//	Sols->K_T_pH_Clean_Sq = *((float *) (pui8Sols + OFFSET_KT_CLEAN_SQ));
//	Sols->K_T_pH_Clean_Ln = *((float *) (pui8Sols + OFFSET_KT_CLEAN_LN));
//
//	if(Sols->K_T_pH_Rinse != Sols->K_T_pH_Rinse)
//	{
//		Sols->K_T_pH_Rinse = -0.0129;
//		Sols->K_T_pH_Cal_1 = -0.0025;
//		Sols->K_T_pH_Cal_2 = -0.0243;
//		Sols->K_T_pH_Clean_Sq = .00007;
//		Sols->K_T_pH_Clean_Ln = -.0071;
//	}
//}
#endif

//********************************************************************************
// Calculates and returns the mV drift of the reference electrode, if its 3M it uses
// empirically derived model to calculate, if saturated it calculates based on temperature
// and returns the difference compared to 25C, assuming a
// Created: 7/14/2021
// Inputs:	Saturated_KCl; 0 if 3M, 1 if saturated
//			Temperature; Temperature in C
// Outputs: Ref_Drift; mV lower than 25 mV reference
//********************************************************************************
int8_t Calculate_Ref_Drift(uint8_t Saturated_KCl, float Temperature)
{
	int8_t Ref_drift = 0;
	if(Saturated_KCl)
	{
		Ref_drift = -1.01 * (Temperature - 25);	// 25 - T because
		DEBUG_PRINT(UARTprintf("Reference difference calculated to be: %i\n", Ref_drift);)
	}
	else
	{
#define REF_PUMP_COEFFICIENT		0.01378
#define REF_DAYS_COEFFICIENT		0.4708

		// Starting in MemoryV4 will be saving completed tests and cals in the cartridge info page of memory
		uint16_t Completed_Tests = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_TESTS, 2));
		if(Completed_Tests == 0xFFFF)
			Completed_Tests = 0;
		uint16_t Completed_Cals = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2));
		if(Completed_Cals == 0xFFFF)
			Completed_Cals = 0;

		float Ref_drift_pump = ((Completed_Tests * 7) + (Completed_Cals * 4) + 10) * REF_PUMP_COEFFICIENT;
//		UARTprintf("Test Number: %d\n", Test_Number);
//		UARTprintf("Cal Number: %d\n", Cal_Number);
		DEBUG_PRINT(UARTprintf("Completed Tests: %d\n", Completed_Tests);)
		DEBUG_PRINT(UARTprintf("Completed Cals: %d\n", Completed_Cals);)

		uint8_t * Time = GetTime();

		// Fill in time structure with data read from memory
		struct tm curTime;
		curTime.tm_mon = Time[0] - 1;
		curTime.tm_mday = Time[1];
		curTime.tm_year = (Time[2] * 100) + Time[3] - 1900;
		curTime.tm_hour = Time[4];
		curTime.tm_min = Time[5];
		curTime.tm_sec = Time[6];
		time_t CurrentTime = mktime(&curTime);	// Convert last cal time from human readable to Epoch time (seconds since 1900)

		uint8_t * hydDate = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_HYDRATION_DATE, 4);
		struct tm hydTime;
		hydTime.tm_mon = *hydDate - 1;
		hydTime.tm_mday = *(hydDate + 1);
		hydTime.tm_year = (*(hydDate + 2) * 100) + *(hydDate + 3) - 1900;
		hydTime.tm_hour = 0;
		hydTime.tm_min = 0;
		hydTime.tm_sec = 0;
		time_t HydrationTime = mktime(&hydTime);	// Convert last cal time from human readable to Epoch time (seconds since 1900)
		float Days_since_hydration = ((CurrentTime - HydrationTime) / 86400);
		if(Days_since_hydration > 40)
		{
			DEBUG_PRINT(UARTprintf("Days since hydration is: %d, setting to 40 days\n", (int) Days_since_hydration);)
			Days_since_hydration = 40;
		}
		else if(Days_since_hydration < 0)
		{
			DEBUG_PRINT(UARTprintf("Days since hydration is: %d, setting to 0 days\n", (int) Days_since_hydration);)
			Days_since_hydration = 0;
		}
		else
		{
			DEBUG_PRINT(UARTprintf("Days since hydration is: %d\n", (int) Days_since_hydration);)
		}
		float Ref_drift_days = Days_since_hydration * REF_DAYS_COEFFICIENT;

		Ref_drift = (uint8_t) (Ref_drift_pump + Ref_drift_days + 0.5);	// Add 0.5 so it rounds when casting, ie (3.2 rounds to 3, 3.2+.5=3.7 trucates to 3 when casted, 3.6+.5=4.1 truncates to 4 when casted)
		DEBUG_PRINT(UARTprintf("Reference drift estimated to be: %d mV\n", Ref_drift);)
	}

	return Ref_drift;
}
