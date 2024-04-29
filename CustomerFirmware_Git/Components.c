//*****************************************************************************
//
// Components.c - Functions used in e-SENS firmware to control physical components
// Includes functions controlling: IO Extenders, DAC, ADC, and Memory
//
// 11/13/2018: Added global variables for IO Extenders so I do not have to read
//	current state of extenders every write cycle, also allows return to current
//	state in case of reset
//
// Author: Jason Castillo
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ssi.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/eeprom.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "Bluetooth.h"
#include "Communication.h"
#include "Components.h"
#include "inc/tm4c123gh6pm.h"
#include "main.h"
#include "helper.h"
#include "Amperometrics.h"
#ifdef MCU_ZXR
#include "PinMap.h"
#endif

// Initial states of IO Extenders defined as global so these variables can be updated
// and stored in memory rather than having to read from IO extender every write cycle
uint8_t gui8IO_Ext1_Reg2 = 0xDF;	// 0xFF to raise DAC_LDAC and run in synchronous update mode
uint8_t gui8IO_Ext1_Reg3 = 0x01;
uint8_t gui8IO_Ext2_Reg2 = 0x90;
uint8_t gui8IO_Ext2_Reg3 = 0x80;
uint16_t gLED_State = 0x0000;

int16_t gi16_DAC[7] = {0, 3000, 3000, 3000, 3000, 0, 0};

//**************************************************************************
// Function to set pin values on IO Extender
// Parameters:	IO_EXT_ADDR; slave address of IO Extender; IO_EXT1_ADDR || IO_EXT2_ADDR
//				ui8Register; Register address that uiPin is on; 2 || 3
//					REF_EL.. and WORK_EL.. pins are register 3 all others are register 2
//				uiPin; Pin to be controlled; Use defined pins at top of Components.h
//				bState; Set pin high or low; true = high; false = low;
//
//	11/13/2018: Modified to use global variables rather than reading
//**************************************************************************
void IO_Ext_Set(unsigned int IO_EXT_ADDR, uint8_t ui8Register, unsigned int uiPin, bool bState)
{
	if(gBoard > V1)
	{
		uint32_t ms_waited = 0;
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
#endif
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("IO Ext Set");
			)

			// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
			// it will prevent the BT from reading incorrect data into the app, TODO: Redesign app to wait for data rather than write read move on
			if(gui8MemConnected == 0)
				ConnectMemory(1);

			userDelay(1, 1);
			ms_waited++;
			if(ms_waited == 10000)
			{
				DEBUG_PRINT(UARTprintf("Waiting for BT to finish with I2C, now waited 10 seconds for IO Ext Set\n");)
				ms_waited = 0;
			}
		}
		update_TivaI2C(1);
		ms_waited = 0;
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
#endif
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("IO Ext Set");
			)

			userDelay(1, 1);
			ms_waited++;
			if(ms_waited == 10000)
			{
				DEBUG_PRINT(UARTprintf("Told BT that Tiva needs I2C, now waited 10 seconds for IO Ext Set\n");)
				ms_waited = 0;
			}
		}
	}

	uint8_t ui8PinValues; // Generic variable to use when writing and reading back
	if(IO_EXT_ADDR == IO_EXT1_ADDR)
	{
		if(ui8Register == 2)
		{
			// Adjust register values using bitwise operations so only desired pin is changed
			if(bState)
				gui8IO_Ext1_Reg2 |= uiPin; // Set bit to turn on uiPin
			else
				gui8IO_Ext1_Reg2 &= ~uiPin; // Clear bit to turn off uiPin
			ui8PinValues = gui8IO_Ext1_Reg2;
		}
		else if(ui8Register == 3)
		{
			// Adjust register values using bitwise operations so only desired pin is changed
			if(bState)
				gui8IO_Ext1_Reg3 |= uiPin; // Set bit to turn on uiPin
			else
				gui8IO_Ext1_Reg3 &= ~uiPin; // Clear bit to turn off uiPin
			ui8PinValues = gui8IO_Ext1_Reg3;
		}
		else
		{
			DEBUG_PRINT(UARTprintf("Writing to incorrect IO Extender register!\n");)
		}
	}
	else if(IO_EXT_ADDR == IO_EXT2_ADDR)
	{
		if(ui8Register == 2)
		{
			// Adjust register values using bitwise operations so only desired pin is changed
			if(bState)
				gui8IO_Ext2_Reg2 |= uiPin; // Set bit to turn on uiPin
			else
				gui8IO_Ext2_Reg2 &= ~uiPin; // Clear bit to turn off uiPin
			ui8PinValues = gui8IO_Ext2_Reg2;
		}
		else if(ui8Register == 3)
		{
			// Adjust register values using bitwise operations so only desired pin is changed
			if(bState)
				gui8IO_Ext2_Reg3 |= uiPin; // Set bit to turn on uiPin
			else
				gui8IO_Ext2_Reg3 &= ~uiPin; // Clear bit to turn off uiPin
			ui8PinValues = gui8IO_Ext2_Reg3;
		}
		else
		{
			DEBUG_PRINT(UARTprintf("Writing to incorrect IO Extender register!\n");)
		}
	}
	else
	{
		DEBUG_PRINT(UARTprintf("Trying to write to IO Extender that doesn't exist!\n");)
	}

	uint8_t ui8PinCheck = ~ui8PinValues;

	uint8_t Counter = 0;
	while(ui8PinCheck != ui8PinValues && Counter <= 10)// && ui32Error == 0)
	{
		// Write modified Pin Values
		I2CSend(I2C0_BASE, IO_EXT_ADDR, 2, ui8Register, ui8PinValues);

		// Read back pins to make sure it was written to correctly
		ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT_ADDR, ui8Register, 1);

		if(Counter < 5)
			Counter++;
		else if(Counter == 5)
		{
			Counter++;
			DEBUG_PRINT(
			UARTprintf("Tried writing IO Extender 5 times, not writing! \n");
			UARTprintf("Resetting analog board!\n");
			)

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

			if(IO_EXT_ADDR == IO_EXT1_ADDR)
				gui32Error |= IO_EXT1_FAIL;
			else if(IO_EXT_ADDR == IO_EXT2_ADDR)
				gui32Error |= IO_EXT2_FAIL;

			update_Error();
		}
	}

	update_TivaI2C(0);
//	userDelay(1, 1);
}

//**************************************************************************
// Initializes both IO extenders on I2C0 bus
// Parameters:	NONE
//
// 11/13/2018: Modified to use the initial values defined at top
// 3/25/2019: Combined all writes and checks into one loop, added ability
//			  to reset analog board and try again if things aren't working
//**************************************************************************
void InitIO_Ext(void)
{
	if(gBoard > V1)
	{
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Initializing IO Ext.. \n");
		)

#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Init IO Extenders \n");
			)
		}
		update_TivaI2C(1);
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Init IO Extenders \n");
			)
		}
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Init IO Extenders \n");
			)
		}
		update_TivaI2C(1);
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Init IO Extenders \n");
			)
		}
#endif
	}

	uint8_t Counter = 0;

	//	uint8_t Register_Values[2] = {0x00, 0x00};
	//	uint16_t Register_Check;
	uint16_t IO1_Setup = 0x0000;
	uint16_t IO2_Setup;
	if(gABoard >= AV6_6)
	{
#ifdef CARTRIDGE_VERSION
		IO2_Setup = 0x0000;	// On the heavy metals cartridge prototype all the pins are used as outputs
#else
		IO2_Setup = 0x0040;
#endif
	}
	else if(gABoard >= AV6_2)
	{
		IO2_Setup = 0x00E0;
//		I2CSend(I2C0_BASE, IO_EXT1_ADDR, 3, 6, 0x00, 0x00); // Set all pins on IO_Ext1 as output
//		I2CSend(I2C0_BASE, IO_EXT2_ADDR, 3, 6, 0xE0, 0x00); // Set used pins on IO_Ext2 ports 0 and 1 as output, leave rest as input
	}
	else if(gABoard == AV6_1)
	{
		IO2_Setup = 0xF0F0;
//		I2CSend(I2C0_BASE, IO_EXT1_ADDR, 3, 6, 0x00, 0x00); // Set all pins on IO_Ext1 as output
//		I2CSend(I2C0_BASE, IO_EXT2_ADDR, 3, 6, 0xF0, 0xF0); // Set used pins on IO_Ext2 ports 0 and 1 as output, leave rest as input
	}
	else if(gABoard == AV6)
	{
		IO2_Setup = 0xF0F8;
//		I2CSend(I2C0_BASE, IO_EXT1_ADDR, 3, 6, 0x00, 0x00); // Set all pins on IO_Ext1 as output
//		I2CSend(I2C0_BASE, IO_EXT2_ADDR, 3, 6, 0xF8, 0xF0); // Set used pins on IO_Ext2 ports 0 and 1 as output, leave rest as input
	}
	else
	{
		IO2_Setup = 0xF0FE;
//		I2CSend(I2C0_BASE, IO_EXT1_ADDR, 3, 6, 0x00, 0x00); // Set all pins on IO_Ext1 as output
//		I2CSend(I2C0_BASE, IO_EXT2_ADDR, 3, 6, 0xFE, 0xF0); // Set used pins on IO_Ext2 ports 0 and 1 as output, leave rest as input
	}

	// Setup IO Extender 1 pins
	uint16_t ui16IO1_Setup = ~IO1_Setup;
	uint16_t ui16IO2_Setup = ~IO2_Setup;

	uint8_t ui8PinValues12 = gui8IO_Ext1_Reg2;
	uint8_t ui8PinValues13 = gui8IO_Ext1_Reg3;
	uint8_t ui8PinValues22 = gui8IO_Ext2_Reg2;
	uint8_t ui8PinValues23 = gui8IO_Ext2_Reg3;
	uint8_t ui8PinCheck12 = ~ui8PinValues12;
	uint8_t ui8PinCheck13 = ~ui8PinValues13;
	uint8_t ui8PinCheck22 = ~ui8PinValues22;
	uint8_t ui8PinCheck23 = ~ui8PinValues23;

	while((ui16IO1_Setup != IO1_Setup || ui16IO2_Setup != IO2_Setup || ui8PinCheck12 != ui8PinValues12 || ui8PinCheck13 != ui8PinValues13 || ui8PinCheck22 != ui8PinValues22 || ui8PinCheck23 != ui8PinValues23) && Counter <= 10)
	{
		I2CSend(I2C0_BASE, IO_EXT1_ADDR, 3, 6, IO1_Setup & 0xFF, (IO1_Setup >> 8) & 0xFF); // Set all pins on IO_Ext1 as output
		I2CSend(I2C0_BASE, IO_EXT2_ADDR, 3, 6, IO2_Setup & 0xFF, (IO2_Setup >> 8) & 0xFF); // Set used pins on IO_Ext2 ports 0 and 1 as output, leave rest as input

		ui16IO1_Setup = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 6, 2);
		ui16IO2_Setup = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 6, 2);

		// Write IO extender 1 register 2
		I2CSend(I2C0_BASE, IO_EXT1_ADDR, 2, 2, ui8PinValues12);
		ui8PinCheck12 = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 2, 1);	// Read back pins to make sure it was written to correctly

		// Write IO extender 1 register 3
		I2CSend(I2C0_BASE, IO_EXT1_ADDR, 2, 3, ui8PinValues13);
		ui8PinCheck13 = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);	// Read back pins to make sure it was written to correctly

		// Write IO extender 2 register 2
		I2CSend(I2C0_BASE, IO_EXT2_ADDR, 2, 2, ui8PinValues22);
		ui8PinCheck22 = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);	// Read back pins to make sure it was written to correctly

		// Write IO extender 2 register 3
		I2CSend(I2C0_BASE, IO_EXT2_ADDR, 2, 3, ui8PinValues23);
		ui8PinCheck23 = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);	// Read back pins to make sure it was written to correctly

		if(Counter < 5)
			Counter++;
		else if(Counter == 5)
		{
			Counter++;
			DEBUG_PRINT(
			UARTprintf("Tried writing IO Extender 5 times, not writing! \n");
			UARTprintf("Resetting analog board!\n");
			)

			update_TivaI2C(0);

			// Turn off analog board
			AnalogOff();

			userDelay(1000, 1);

			// Set pin low, turn on analog board, then set high
#ifdef MCU_ZXR
			GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
			GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On
#else
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
#endif

			SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component

			if(gBoard == V6_2 || gBoard == V6_3)
				InitLED_Ext();

			// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
#ifdef MCU_ZXR
			if(gABoard >= AV6_1)
				GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN); // Conductivity ADC CNV Pin
			else
				GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin
#else
			if(gABoard >= AV6_1)
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
			else
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
#endif


			update_TivaI2C(1);
		}
		else if(Counter < 10)
			Counter++;
		else if(Counter == 10)
		{
			Counter++;

			DEBUG_PRINT(UARTprintf("Tried writing IO Extender 5 more times after reset, still not writing!\n");)

			if(ui8PinCheck12 != ui8PinValues12 || ui8PinCheck13 != ui8PinValues13)
				gui32Error |= IO_EXT1_FAIL;
			if(ui8PinCheck22 != ui8PinValues22 || ui8PinCheck23 != ui8PinValues23)
				gui32Error |= IO_EXT2_FAIL;

			update_Error();
		}
	}

//		// Setup IO Extender 1 pins
//		ui8PinValues = gui8IO_Ext1_Reg3;
//		ui8PinCheck = ~ui8PinValues;
//		Counter = 0;
//		while(ui8PinCheck != ui8PinValues)
//		{
//			// Write modified Pin Values
//			I2CSend(I2C0_BASE, IO_EXT1_ADDR, 2, 3, ui8PinValues);
//
//			// Read back pins to make sure it was written to correctly
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);
//
//			if(Counter < 5)
//				Counter++;
//			else if(Counter == 5)
//			{
//				Counter++;
//				UARTprintf("Tried writing IO Extender 5 times, not writing! \n");
//				UARTprintf("Resetting analog board!\n");
//
//				// Turn off analog board
//				AnalogOff();
//
//				SysCtlDelay(SysCtlClockGet()/3000 * 5);	// Wait for analog to completely power-off
//
//				InitLED_Ext();
//
//				// Set pin low, turn on analog board, then set high
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//
//				SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
//				// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
//				if(gABoard >= AV6_1)
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//				else
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
//			}
//			else if(Counter < 10)
//				Counter++;
//			else if(Counter == 10)
//			{
//				Counter++;
//
//				UARTprintf("Tried writing IO Extender 5 more times after reset, still not writing!\n");
//
//				gui32Error |= IO_EXT1_FAIL;
//				update_Error();
//
//				Errors++;
//				break;
//			}
//		}
//
//		// Setup IO Extender 2 pins
//		ui8PinValues = gui8IO_Ext2_Reg2;
//		ui8PinCheck = ~ui8PinValues;
//		Counter = 0;
//		while(ui8PinCheck != ui8PinValues)
//		{
//			// Write modified Pin Values
//			I2CSend(I2C0_BASE, IO_EXT2_ADDR, 2, 2, ui8PinValues);
//
//			// Read back pins to make sure it was written to correctly
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);
//
//			if(Counter < 5)
//				Counter++;
//			else if(Counter == 5)
//			{
//				Counter++;
//				UARTprintf("Tried writing IO Extender 5 times, not writing! \n");
//				UARTprintf("Resetting analog board!\n");
//
//				// Turn off analog board
//				AnalogOff();
//
//				SysCtlDelay(SysCtlClockGet()/3000 * 5);	// Wait for analog to completely poweroff
//
//				InitLED_Ext();
//
//				// Set pin low, turn on analog board, then set high
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//
//				SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
//				// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
//				if(gABoard >= AV6_1)
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//				else
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
//			}
//			else if(Counter < 10)
//				Counter++;
//			else if(Counter == 10)
//			{
//				Counter++;
//
//				UARTprintf("Tried writing IO Extender 5 more times after reset, still not writing!\n");
//
//				gui32Error |= IO_EXT2_FAIL;
//				update_Error();
//
//				Errors++;
//				break;
//			}
//		}
//
//		// Setup IO Extender 2 pins
//		ui8PinValues = gui8IO_Ext2_Reg3;
//		ui8PinCheck = ~ui8PinValues;
//		Counter = 0;
//		while(ui8PinCheck != ui8PinValues)
//		{
//			// Write modified Pin Values
//			I2CSend(I2C0_BASE, IO_EXT2_ADDR, 2, 3, ui8PinValues);
//
//			// Read back pins to make sure it was written to correctly
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
//
//			if(Counter < 5)
//				Counter++;
//			else if(Counter == 5)
//			{
//				Counter++;
//				UARTprintf("Tried writing IO Extender 5 times, not writing! \n");
//				UARTprintf("Resetting analog board!\n");
//
//				// Turn off analog board
//				AnalogOff();
//
//				SysCtlDelay(SysCtlClockGet()/3000 * 5);	// Wait for analog to completely poweroff
//
//				InitLED_Ext();
//
//				// Set pin low, turn on analog board, then set high
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//
//				SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
//				// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
//				if(gABoard >= AV6_1)
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//				else
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
//			}
//			else if(Counter < 10)
//				Counter++;
//			else if(Counter == 10)
//			{
//				Counter++;
//
//				UARTprintf("Tried writing IO Extender 5 more times after reset, still not writing!\n");
//
//				gui32Error |= IO_EXT2_FAIL;
//				update_Error();
//
//				Errors++;
//				break;
//			}
//		}

	update_TivaI2C(0);
}

//**************************************************************************
// Initializes pins for DAC and resets device
// Parameters:	NONE
//**************************************************************************
void InitDAC(void)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	uint8_t Setup_bits_a = 0x81;	// D15-D8 Set read mode and Gain-A, Gain-A bit is set, output span = 4*REF_A = 4* 1.5 = 6 V output span
	uint8_t Setup_bits_b = 0x80;	// D7-D0  Set Gain-B, Gain-B bit is set, output span = 4*REF_B = 4* 1.5 = 6 V output span

	uint32_t ui32DAC_rx[3] = {0, ~Setup_bits_a, ~Setup_bits_b};

	uint8_t counter = 0;

	while((ui32DAC_rx[1] != Setup_bits_a || ui32DAC_rx[2] != Setup_bits_b) && counter <= 15)
	{
		SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0, 0xA0, 0); // Reset DAC

		SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0, Setup_bits_a, Setup_bits_b); // Set Gain bits	(register 0)

		// Read back gain bits to determine if they wrote correctly
		SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0x80, Setup_bits_a, Setup_bits_b); // Read Gain bits (register 0 with r/w bit set) send which register to read
		SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0, Setup_bits_a, Setup_bits_b | 0x20); // Clock out signal with NOP bit so DAC can return data


		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
	//	SysCtlDelay(2000);
		SSIDataGet(SSI1_BASE, &ui32DAC_rx[0]);
		SSIDataGet(SSI1_BASE, &ui32DAC_rx[1]);
		SSIDataGet(SSI1_BASE, &ui32DAC_rx[2]);

	//	UARTprintf("DAC gain read back: %d, %d, %d\n", ui32DAC_rx[0], ui32DAC_rx[1], ui32DAC_rx[2]);

		if(counter < 5)
			counter++;
		else if(counter == 5)
		{
			if(ui32DAC_rx[1] != Setup_bits_a || ui32DAC_rx[2] != Setup_bits_b)
			{
				DEBUG_PRINT(
				UARTprintf("Tried initializing DAC 5 times, DAC not returning correct data! \n");
				UARTprintf("Resetting analog board!\n");
				)

				counter++;

				AnalogOff();

				userDelay(1000, 1);

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
#endif


				InitIO_Ext();		// Sets up IO Extenders and sets initial values

				if(gBoard == V6_2 || gBoard == V6_3)
					InitLED_Ext();
				else
					SetLED(0, 1);

				// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
				SSIDisable(SSI1_BASE);
				SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
				SSIEnable(SSI1_BASE);

				SysCtlDelay(2000);
			}

		}
		else if(counter < 15)
			counter++;
		else if(counter == 15)
		{
			DEBUG_PRINT(UARTprintf("Tried writing DAC 15 times, DAC didn't program voltage correctly! \n");)
			counter++;

			gui32Error |= DAC_FAIL;
			update_Error();

			DEBUG_PRINT(
			uint8_t ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 2, 1);
			UARTprintf("IO Extender 1 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg2);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);
			UARTprintf("IO Extender 1 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg3);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);
			UARTprintf("IO Extender 2 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg2);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
			UARTprintf("IO Extender 2 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg3);
#ifdef MCU_ZXR
			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN));
#else
			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1));
#endif
			)
		}
	}

	if(gABoard >= AV6)
	{
		DACVoltageSet(0, gi16_DAC[0], false);
		DACVoltageSet(1, gi16_DAC[1], false);
		DACVoltageSet(2, gi16_DAC[2], true);
		DACVoltageSet(3, gi16_DAC[3], false);
		DACVoltageSet(4, gi16_DAC[4], true);
		DACVoltageSet(5, gi16_DAC[5], false);
		DACVoltageSet(6, gi16_DAC[6], true);

	}
}

//**************************************************************************
// Set DAC_port to output voltage V_out
// Parameters:	DAC_port; Port to apply voltage to; [0:7]; 7 not connected
//				V_out; Desired output voltage in mV [-3000, 3000]
//				update; true to have DAC output voltage
//						false to update register without outputing voltage
//**************************************************************************
void DACVoltageSet(uint8_t DAC_port, float V_out, bool update)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	gi16_DAC[DAC_port] = V_out;

	int16_t INPUT_CODE; // Value to write to DAC to set desired V_out

	uint8_t DAC_address = 8 + DAC_port; // DAC0 starts at address 8, each DAC is incremented 1 after

	if(V_out < -3000)
		V_out = -3000;
	if(V_out > 3000)
		V_out = 3000;

	float INPUT_Float;
	if(gABoard >= ARV1_0B)
	{
		INPUT_Float = (V_out * 8192) / 3000.0; // signed 14-bit number spans -3 to +3 Volts, add .5 for rounding when converting to int16_t
		if(INPUT_Float > 8191)	// 1 less than max number because it's signed
			INPUT_Float = 8191;
	}
	else
	{
		INPUT_Float = (V_out * 32768) / 3000.0 + 0.5; // signed 16-bit number spans -3 to +3 Volts, add .5 for rounding when converting to int16_t
		if(INPUT_Float > 32767)	// 1 less than max number because it's signed
			INPUT_Float = 32767;
	}

	INPUT_CODE = INPUT_Float; // Convert float to signed 16-bit format
	if(gABoard >= ARV1_0B)
		INPUT_CODE = INPUT_CODE << 2;	// On the 14-bit DAC part the 2 LSB are not read


	uint8_t bits07 = INPUT_CODE & 0xff; // Get LSB of INPUT_CODE
	uint8_t bits815 = (INPUT_CODE >> 8); // Get MSB of INPUT_CODE
	uint32_t ui32DAC_rx[3] = {0, ~bits815, ~bits07};
	uint8_t counter = 0;

//	UARTprintf("Bits 07: %d\n", bits07);
//	UARTprintf("Bits 815: %d\n", bits815);
	while((ui32DAC_rx[1] != bits815 || ui32DAC_rx[2] != bits07) && counter <= 15)
	{
		// Write INPUT_CODE to DAC
		SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, DAC_address, bits815, bits07);

		// Read back from DAC register to determine if it was written correctly
		SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, DAC_address | 0x80, bits815, bits07);	// Read back from register just written by setting r/w bit
		SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0, 0, 0x20);	// Clock out signal to receive data with NOP bit set

		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
		//	SysCtlDelay(2000);
		SSIDataGet(SSI1_BASE, &ui32DAC_rx[0]); // First byte received is junk, data overwritten on next line
		SSIDataGet(SSI1_BASE, &ui32DAC_rx[1]); // Save MSB
		SSIDataGet(SSI1_BASE, &ui32DAC_rx[2]); // Save LSB

		//	UARTprintf("DAC writing: %d, %d\n", bits815, bits07);
		//	UARTprintf("DAC V read back: %d, %d\n", ui32DAC_rx[1], ui32DAC_rx[2]);

		if(counter < 10)
			counter++;
		else if(counter == 10)
		{
			if(ui32DAC_rx[1] != bits815 || ui32DAC_rx[2] != bits07)
			{
				DEBUG_PRINT(
				UARTprintf("Tried writing DAC 10 times, DAC didn't program voltage correctly! \n");
				UARTprintf("Resetting analog board!\n");
				)
				counter++;

				AnalogOff();

				userDelay(1000, 1);

				SysCtlDelay(2000);

				InitAnalog();

				// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
				SSIDisable(SSI1_BASE);
				SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
				SSIEnable(SSI1_BASE);

				SysCtlDelay(2000);
			}

		}
		else if(counter < 15)
			counter++;
		else if(counter == 15)
		{
			DEBUG_PRINT(UARTprintf("Tried writing DAC 15 times, DAC didn't program voltage correctly! \n");)
			counter++;

			gui32Error |= DAC_FAIL;
			update_Error();

			DEBUG_PRINT(
			uint8_t ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 2, 1);
			UARTprintf("IO Extender 1 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg2);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);
			UARTprintf("IO Extender 1 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg3);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);
			UARTprintf("IO Extender 2 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg2);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
			UARTprintf("IO Extender 2 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg3);
#ifdef MCU_ZXR
			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN));
#else
			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1));
#endif
			)
		}
	}

//#ifndef HEAVY_METALS	// 2/28/2023: Leaving the DAC in asynchronous mode, it slows it down to have to change the LDAC and for heavy metals we won't be able to do that
//	if(update && gABoard >= ARV1_0B)
//	{
//		// LDAC Pin must be toggled to update the DAC Output
//		IO_Ext_Set(IO_EXT1_ADDR, 2, DAC1_LDAC_B, 0); // Set DAC LDAC Pin low
//		IO_Ext_Set(IO_EXT1_ADDR, 2, DAC1_LDAC_B, 1); // Set DAC LDAC Pin high
//	}
//#endif
}

//**************************************************************************
// Initialize ADC component LMP90080
// Sets up channels to compare against Ain7 = V_ref = 1.5V
// Parameters:	NONE
//**************************************************************************
void InitADC(void)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

//	// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
//	if(gABoard >= AV6_1)
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//	else
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin

//	if(gABoard >= AV6_6)
//	{
//		// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
//		SSIDisable(SSI1_BASE);
//		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
//		SSIEnable(SSI1_BASE);
//
//		uint32_t InMux_Rx = 0, DataRate_Rx = 0;
//		uint8_t counter = 0;
//
//		while(InMux_Rx != 0x4C || DataRate_Rx != 0x1E)
//		{
//			// Write register 010r rrrr
//			// Read register 001r rrrr
//
//			// Input Multiplexer register 0x02, write = 0x42, read = 0x22
//			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x42, 0x00, 0x4C);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)
//			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x22, 0x00, 0x00);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)
//
//
//			SSIDataGet(SSI1_BASE, &InMux_Rx);
//			SSIDataGet(SSI1_BASE, &InMux_Rx);
//			SSIDataGet(SSI1_BASE, &InMux_Rx);
//
//			//		UARTprintf("Input mux = %x\n", InMux_Rx);
//
//			// Data rate register 0x04, write = 0x44, read = 0x24
//			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x44, 0x00, 0x1E);	// Set low latency filter 0x10, Set 2000 SPS data rate 0x0C, 4000 SPS data rate 0x0E
//			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x24, 0x00, 0x00);	// Read back data rate register
//
//
//			SSIDataGet(SSI1_BASE, &DataRate_Rx);
//			SSIDataGet(SSI1_BASE, &DataRate_Rx);
//			SSIDataGet(SSI1_BASE, &DataRate_Rx);
//
//			//		UARTprintf("Data rate = %x\n", DataRate_Rx);
//
//			if(counter < 5)
//				counter++;
//			else if(counter == 5)
//			{
//				counter++;
//
//				UARTprintf("Tried writing ADC 5 registers 5 times, not writing!\n");
//				UARTprintf("InMux Rx = %x, should be 0x4c\n");
//				UARTprintf("Data rate Rx = %x, should be 0x1c\n");
//				UARTprintf("Resetting analog board!\n");
//
//				AnalogOff();
//
//				userDelay(1000, 1);
//
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation
//
//				// Set pin low, turn on analog board, then set high
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//
//				SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
//				// Need to set conductivity ADC pin high after powering on analog board so it sees rising edge
//				if(gABoard >= AV6_1)
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2); // Conductivity ADC CNV Pin
//				else
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin
//
//				// Toggle Reset Pin
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0); // Reset is active low, send pulse at startup to reset IO extenders and DAC
//				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Set high for normal operation
//				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1ms
//
//				if(gBoard == V6_2 || gBoard == V6_3)
//				{
//					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//				}
//
//				InitIO_Ext();		// Sets up IO Extenders and sets initial values
//
//				if(gBoard == V6_2 || gBoard == V6_3)
//					InitLED_Ext();
//				else
//					SetLED(0, 1);
//
//				InitDAC();			// Resets device and writes configuration register
//
//				// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
//				SSIDisable(SSI1_BASE);
//				SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
//				SSIEnable(SSI1_BASE);
//			}
//			else if(counter < 10)
//				counter++;
//			else if(counter == 10)
//			{
//				counter++;
//				gui32Error |= ADC5_FAIL;
//				update_Error();
//			}
//		}
//
//
//	}
//	else if(gABoard >= AV6_2)
//	{
//		// Set SPI communication to mode 1 for ADC5
//		SSIDisable(SSI1_BASE);
//		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
//		SSIEnable(SSI1_BASE);
//
//		// Configure ADC 5 so AINp is AIN0 and AINn is GND
//		// Set FSR to +/- 2.048V; LSB of 62.5 uV
//		// Set it in power-down/single-shot mode when not using
//		// Set data rate to 128 SPS
//		// ADC Mode
//		// Disable pullup resistor
//		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0x45, 0x83);
//	}


	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	uint8_t Errors = 1;
	uint8_t counter = 0;
	while(Errors != 0 && counter <= 10)
	{
//		UARTprintf("Initializing ADCs...\n");
		Errors = 0;

		SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, 0x10, 0x00, 0x00, 0xC3); // Send Reset Command
		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x00, 0x00, 0xC3); // Send Reset Command
		SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x00, 0x00, 0xC3); // Send Reset Command

		// Write the channel scan register to set which channel to read
		// 0x10: Device address
		// 0x01: Upper address byte for CH_SCAN register
		// 0x0F: Write 1 byte to register with lower address byte 0xF
		// 0x00: Sets single channel continuous scan
		SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, 0x10, 0x01, 0x0F, 0x00); // Set channel scan register
		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x01, 0x0F, 0x00); // Set channel scan register
		SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x01, 0x0F, 0x00); // Set channel scan register

		// Read back Channel scan registers to determine if they wrote correctly
		uint32_t ui32ADC1_rx, ui32ADC2_rx, ui32ADC4_rx;
		uint8_t i;
		SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, 0x10, 0x01, 0x8F, 0x00); // Set channel scan register
		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
		for(i = 0; i < 4; i++)
			SSIDataGet(SSI1_BASE, &ui32ADC1_rx); // First byte received is junk, data overwritten on next line
		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x01, 0x8F, 0x00); // Set channel scan register
		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
		for(i = 0; i < 4; i++)
			SSIDataGet(SSI1_BASE, &ui32ADC2_rx); // First byte received is junk, data overwritten on next line
		SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x01, 0x8F, 0x00); // Set channel scan register
		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
		for(i = 0; i < 4; i++)
			SSIDataGet(SSI1_BASE, &ui32ADC4_rx); // First byte received is junk, data overwritten on next line

		if(counter < 5)
			counter++;
		else if(counter == 5)
		{
			counter++;

			DEBUG_PRINT(
			UARTprintf("Tried writing ADC channel scan register 5 times, not writing!\n");
			UARTprintf("Resetting analog board!\n");
			)

			AnalogOff();

			userDelay(1000, 1);

#ifdef MCU_ZXR
			GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

			// Set pin low, turn on analog board, then set high
			GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
			GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

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

			if(gBoard == V6_2 || gBoard == V6_3)
			{
				// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
				GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
				GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
			}
#else
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

			// Set pin low, turn on analog board, then set high
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

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

			if(gBoard == V6_2 || gBoard == V6_3)
			{
				// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
			}
#endif

			InitIO_Ext();		// Sets up IO Extenders and sets initial values

			if(gBoard == V6_2 || gBoard == V6_3)
				InitLED_Ext();
			else
				SetLED(0, 1);

			InitDAC();			// Resets device and writes configuration register

			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);
		}
		else if(counter < 10)
			counter++;
		else if(counter == 10)
		{
			counter++;
			if(ui32ADC1_rx != 0)
			{
				DEBUG_PRINT(UARTprintf("ADC 1 didn't program channel scan register correctly! \n");)

				Errors++;
				gui32Error |= ADC1_FAIL;
				update_Error();
			}
			if(ui32ADC2_rx != 0)
			{
				DEBUG_PRINT(UARTprintf("ADC 2 didn't program channel scan register correctly! \n");)

				Errors++;
				gui32Error |= ADC2_FAIL;
				update_Error();
			}
			if(ui32ADC4_rx != 0)
			{
				DEBUG_PRINT(UARTprintf("ADC 4 didn't program channel scan register correctly! \n");)

				Errors++;
				gui32Error |= ADC4_FAIL;
				update_Error();
			}
		}

		if(ui32ADC1_rx != 0)
		{
			Errors++;
		}
		if(ui32ADC2_rx != 0)
		{
			Errors++;
		}
		if(ui32ADC4_rx != 0)
		{
			Errors++;
		}

	}

	// Configure ADC1 and ADC2 so all channels compare to 1.5V ref on A_in6
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, 0x10, 0x02, 0x00, 0x06); // Send first transaction with URA and set up CH0: Temp VINP = VIN0, VINN = VIN6
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x02, 0x0E); // CH1: ISE 1: VINP = VIN1; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x04, 0x16); // CH2: ISE 2: VINP = VIN2; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x06, 0x1E); // CH3: ISE 3: VINP = VIN3; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x08, 0x26); // CH4: ISE 4: VINP = VIN4; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x0A, 0x2E); // CH5: ISE 5: VINP = VIN5; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x0C, 0x3E); // CH6: Temp 2: VINP = VIN7; VINN = VIN6 = 1.5V

	// Configure ADC1 to include unity gain buffer on all channels
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, 0x10, 0x02, 0x01, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x03, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x05, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x07, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x09, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x0B, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 2, 0x0D, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path

	// Configure channels on ADC 2 to all compare to 1.5V ref
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x02, 0x00, 0x06); // Send first transaction with URA and set up CH0: ORP VINP = VIN0, VINN = VIN6
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x02, 0x0E); // CH1: ISE 6: VINP = VIN1; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x04, 0x16); // CH2: ISE 7: VINP = VIN2; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x06, 0x1E); // CH3: ISE 8: VINP = VIN3; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x08, 0x26); // CH4: ISE 9: VINP = VIN4; VINN = VIN6 = 1.5V
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x0A, 0x2E); // CH5: ISE 10: VINP = VIN5; VINN = VIN6 = 1.5V
	if(gABoard >= AV6)
		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x0C, 0x3E); // CH6: ORP I Read: VINP = VIN7; VINN = VIN6 = 1.5V

	// Configure ADC2 to include unity gain buffer on all channels
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x02, 0x01, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x03, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x05, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x07, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x09, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x0B, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	if(gABoard >= AV6)
		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 2, 0x0D, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path

	// Configure ADC4 so all channels compare to 1.5V ref on A_in7
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x02, 0x00, 0x07); // Send first transaction with URA and set up CH0: All Amps VINP = VIN0, VINN = VIN7
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x02, 0x0F); // CH1: Amp 1: VINP = VIN1; VINN = VIN7 = 1.5V
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x04, 0x17); // CH2: Amp 2: VINP = VIN2; VINN = VIN7 = 1.5V
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x06, 0x1F); // CH3: Amp 3: VINP = VIN3; VINN = VIN7 = 1.5V
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x08, 0x27); // CH4: Amp 4: VINP = VIN4; VINN = VIN7 = 1.5V
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x0A, 0x2F); // CH5: Amp 5: VINP = VIN5; VINN = VIN7 = 1.5V
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x0C, 0x37); // CH6: Amp 6: VINP = VIN6; VINN = VIN7 = 1.5V

//	// Configure ADC4 so all channels compare to 1.5V ref on A_in6
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x02, 0x00, 0x06); // Send first transaction with URA and set up CH0: Temp VINP = VIN0, VINN = VIN6
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x02, 0x0E); // CH1: ISE 1: VINP = VIN1; VINN = VIN6 = 1.5V
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x04, 0x16); // CH2: ISE 2: VINP = VIN2; VINN = VIN6 = 1.5V
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x06, 0x1E); // CH3: ISE 3: VINP = VIN3; VINN = VIN6 = 1.5V
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x08, 0x26); // CH4: ISE 4: VINP = VIN4; VINN = VIN6 = 1.5V
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x0A, 0x2E); // CH5: ISE 5: VINP = VIN5; VINN = VIN6 = 1.5V
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x0C, 0x3E); // CH6: Temp 2: VINP = VIN7; VINN = VIN6 = 1.5V

	// Configure ADC4 to include unity gain buffer on all channels
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x02, 0x01, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path

//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x81, 0x00); // Read back register
//	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
//	uint8_t i;
//	uint32_t ui32ADC1_rx;
//	for(i = 0; i < 2; i++)
//		SSIDataGet(SSI1_BASE, &ui32ADC1_rx); // First byte received is junk, data overwritten on next line
//	UARTprintf("ADC 4 Register Value: 0x%x\n", ui32ADC1_rx);

	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x03, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x05, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x07, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x09, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x0B, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 2, 0x0D, 0x71); // ODR = 214.65 SPS include unity gain buffer in signal path

//	// Write the channel scan register to set which channel to read
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x01, 0x0F, 0x00); // Set channel scan register to scan multiple channels once // continuously scan all channels B1

	// Set controlled streaming
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x00, 0x03, 0x82); // Turn on controlled streaming with range = 2

	// Set all ADCs into stand-by mode to reduce power consumption
	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
}

//**************************************************************************
// Configures ADC 5, AV6_6 (ADS114S06) and up use this for turbidity and checking
// conductivity frequency, AV6_2-5 used it for checking for bubbles
// disconnecting RE
// Parameters:	NONE
// Outputs:		NONE
//**************************************************************************
void InitTurbidityADC(void)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	if(gABoard >= AV6_6)
	{
		// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
		SSIDisable(SSI1_BASE);
		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
		SSIEnable(SSI1_BASE);

		uint32_t InMux_Rx = 0, DataRate_Rx = 0;
		uint8_t counter = 0;

		while((InMux_Rx != 0x4C || DataRate_Rx != 0x1E) && counter <= 10)
		{
			// Write register 010r rrrr
			// Read register 001r rrrr

			// Input Multiplexer register 0x02, write = 0x42, read = 0x22
			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x42, 0x00, 0x4C);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)
			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x22, 0x00, 0x00);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)

			SSIDataGet(SSI1_BASE, &InMux_Rx);
			SSIDataGet(SSI1_BASE, &InMux_Rx);
			SSIDataGet(SSI1_BASE, &InMux_Rx);

			//		UARTprintf("Input mux = %x\n", InMux_Rx);

			// Data rate register 0x04, write = 0x44, read = 0x24
			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x44, 0x00, 0x1E);	// Set low latency filter 0x10, Set 2000 SPS data rate 0x0C, 4000 SPS data rate 0x0E
			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x24, 0x00, 0x00);	// Read back data rate register

			SSIDataGet(SSI1_BASE, &DataRate_Rx);
			SSIDataGet(SSI1_BASE, &DataRate_Rx);
			SSIDataGet(SSI1_BASE, &DataRate_Rx);

			//		UARTprintf("Data rate = %x\n", DataRate_Rx);

			if(counter < 5)
				counter++;
			else if(counter == 5 && (InMux_Rx != 0x4C || DataRate_Rx != 0x1E))
			{
				counter++;

				DEBUG_PRINT(
				UARTprintf("Tried writing ADC 5 registers 5 times, not writing!\n");
				UARTprintf("InMux Rx = %x, should be 0x4c\n");
				UARTprintf("Data rate Rx = %x, should be 0x1c\n");
				UARTprintf("Resetting analog board!\n");
				)

				AnalogOff();

				userDelay(1000, 1);

#ifdef MCU_ZXR
				GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

				// Set pin low, turn on analog board, then set high
				GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
				GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

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

				if(gBoard == V6_2 || gBoard == V6_3)
				{
					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
					GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
				}
#else
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

				// Set pin low, turn on analog board, then set high
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

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

				if(gBoard == V6_2 || gBoard == V6_3)
				{
					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
				}
#endif


				InitIO_Ext();		// Sets up IO Extenders and sets initial values

				if(gBoard == V6_2 || gBoard == V6_3)
					InitLED_Ext();
				else
					SetLED(0, 1);

				InitDAC();			// Resets device and writes configuration register

				// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
				SSIDisable(SSI1_BASE);
				SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
				SSIEnable(SSI1_BASE);
			}
			else if(counter < 10)
				counter++;
			else if(counter == 10 && (InMux_Rx != 0x4C || DataRate_Rx != 0x1E))
			{
				DEBUG_PRINT(UARTprintf("ADC5 failed to initialize input mux or data rate!\nUpdating error and moving on!\n");)
				counter++;
				gui32Error |= ADC5_FAIL;
				update_Error();
			}
		}

		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x04);	// Put ADC into power-down mode until it is used

	}
	else if(gABoard >= AV6_2)
	{
		// Set SPI communication to mode 1 for ADC5
		SSIDisable(SSI1_BASE);
		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
		SSIEnable(SSI1_BASE);

		// Configure ADC 5 so AINp is AIN0 and AINn is GND
		// Set FSR to +/- 2.048V; LSB of 62.5 uV
		// Set it in power-down/single-shot mode when not using
		// Set data rate to 128 SPS
		// ADC Mode
		// Disable pullup resistor
		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 2, 0x45, 0x83);
	}
}

////**************************************************************************
//// Function to read in voltage on specific channel
//// Parameters:	Channel; [0,6] 	0: All amperometrics
////								1: Amp 1
////								2: Amp 2... etc
////				ADC_CS_PIN; ADC1_CS_B, ADC2_CS_B, or ADC4_CS_B
//// Outputs:		fVoltage; Voltage coming into specified channel in mV
////**************************************************************************
//float ADCRead(uint8_t ui8Channel, int ADC_CS_PIN)
//{
//	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//
//	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//	SSIDisable(SSI1_BASE);
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//	SSIEnable(SSI1_BASE);
//
//	SysCtlDelay(2000);
//
//	uint8_t ADC_Address = 0x10; // Instruction 1, must be sent whenever URA Changes
//	uint8_t URA = 0x01; // Upper Register Address; Channel Scan Register and ADC_DOUT register have same URA
//	uint8_t LRA_ch_scan_w = 0x0F; // Lower Register Address; write 1 byte to channel scan register
//	uint8_t LRA_ch_scan_r = 0x8F; // Read 1 byte from channel scan register
//	uint8_t LRA_ADC_DOUT = 0xAA; // Read 2 bytes from ADC_DOUT registers
//
//	uint8_t ui8Channel_Tx = (ui8Channel | ((ui8Channel) << 3)); // Set first and last channel to scan
//
//	uint32_t ui32ADC_MSB; // Most Significant byte of ADC_DOUT
//	uint32_t ui32ADC_LSB; // Least Signficant byte of ADC_DOUT
//	int16_t ADC_DOUT = 0; // signed 16 bit data assembled from two ADC_DOUT registers
//	uint32_t ui32Channel_Rx = ui8Channel_Tx + 1;
//
//	// Random data points seem really far off, as if the channel wasn't changed successfully and wrong channel was being read
//	// Created loop to verify the channel register was updated correctly; JC 2/9/2018
//	while(ui8Channel_Tx != ui32Channel_Rx)
//	{
//		// Set ADC we are using into active mode
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x00); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//		// Write the channel scan register to set which channel to read
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_w, ui8Channel_Tx); // Set channel scan register
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
//		SysCtlDelay(30000); // Delay so channel scan register can update
//
//		// Must read channel scan register after setting it
//		// Assume this is to update values as register is buffered
//		// Don't actually know why this is required
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_r, 0x00); // Read Channel Scan Register
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//	}
//
//	// Read from ADC_DOUT registers
//	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 3, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers
//	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//	SysCtlDelay(2000);
//	SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // First byte received is junk, data overwritten on next line
//	SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // Save MSB
//	SSIDataGet(SSI1_BASE, &ui32ADC_LSB); // Save LSB
//
//	// Set ADC back into stand-by mode
//	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//	ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
//	ADC_DOUT |= ui32ADC_LSB; // Set LSB
//
////	float VREFP = 3; // Positive reference voltage
////	float VREFN = 0; // Negative reference voltage
////	float GAIN = 1; // by default
////	float VINN = 1.5; // Compare all channels to incoming 1.5 reference voltage
////	float fVoltage = (ADC_DOUT * (VREFP - VREFN) / pow(2,15))/GAIN + VINN;
//	float fVoltage = (ADC_DOUT * (1.5) / 32768.0) + 1.5; // Convert ADC_DOUT data into voltage
//
//	return (fVoltage * 1000);
//}

//**************************************************************************
// Function to read in voltage on specific channel
// Samples at ~200 SPS, max of 20 samples
// Returns average voltage
// Parameters:	Channel; [0,6] 	0: All amperometrics
//								1: Amp 1
//								2: Amp 2... etc
//				ADC_CS_PIN; ADC1_CS_B, ADC2_CS_B, or ADC4_CS_B
//				samples; Number of samples to take; 20
// Outputs:		V_avg; Average voltage, mV
//**************************************************************************
float ADCReadAvg(uint8_t ui8Channel, int ADC_CS_PIN, int samples)
{
	float fVoltage;
	int total_samples = samples;

	uint8_t ADC_Address = 0x10; // Instruction 1, must be sent whenever URA Changes
	uint8_t URA = 0x01; // Upper Register Address; Channel Scan Register and ADC_DOUT register have same URA
	uint8_t LRA_ch_scan_w = 0x0F; // Lower Register Address; write 1 byte to channel scan register
	uint8_t LRA_ch_scan_r = 0x8F; // Read 1 byte from channel scan register
	uint8_t LRA_ADC_DOUT = 0xAA; // Read 2 bytes from ADC_DOUT registers

	uint8_t ui8Channel_Tx = (ui8Channel | ((ui8Channel) << 3)); // Set first and last channel to scan

	uint32_t ui32ADC_MSB; // Most Significant byte of ADC_DOUT
	uint32_t ui32ADC_LSB; // Least Signficant byte of ADC_DOUT
	int16_t ADC_DOUT = 0; // signed 16 bit data assembled from two ADC_DOUT registers
	uint32_t ui32Channel_Rx = ~ui8Channel_Tx;

	uint8_t counter = 0;

	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	SysCtlDelay(2000);

	// Random data points seem really far off, as if the channel wasn't changed successfully and wrong channel was being read
	// Created loop to verify the channel register was updated correctly
	while(ui8Channel_Tx != ui32Channel_Rx && counter <= 15)
	{
		// Set ADC we are using into active mode
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x00); // Register 0x08, 0x3 stand-by mode, 0x0 active mode

		// Write the channel scan register to set which channel to read
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_w, ui8Channel_Tx); // Set channel scan register
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
		SysCtlDelay(30000); // Delay so channel scan register can update

//		UARTprintf("Reading back Channel \n");

		// Must read channel scan register after setting it
		// Assume this is to update values as register is buffered
		// Don't actually know why this is required
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_r, 0x00); // Read Channel Scan Register
		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);

		if(counter < 5)
			counter++;
		else if(counter == 5)
		{
			DEBUG_PRINT(UARTprintf("Tried writing ADC channel 5 times, reinitializing all ADCs!\n");)

			InitADC();
			counter++;
		}
		else if(counter < 10)
			counter++;
		else if(counter == 10)
		{
			DEBUG_PRINT(
			UARTprintf("Tried writing ADC channel 10 times, failed everytime! \n");
			UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);
			UARTprintf("Resetting analog board!\n");
			)
			counter++;

			AnalogOff();

			userDelay(1000, 1);

			SysCtlDelay(2000);

			InitAnalog();

			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SysCtlDelay(2000);
		}
		else if(counter < 15)
			counter++;
		else if(counter == 15)
		{
			counter++;
			DEBUG_PRINT(UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);)

			if(ADC_CS_PIN == ADC1_CS_B)
			{
				DEBUG_PRINT(UARTprintf("ADC 1 didn't program channel scan register correctly! \n");)

				gui32Error |= ADC1_FAIL;
				update_Error();
			}
			if(ADC_CS_PIN == ADC2_CS_B)
			{
				DEBUG_PRINT(UARTprintf("ADC 2 didn't program channel scan register correctly! \n");)

				gui32Error |= ADC2_FAIL;
				update_Error();
			}
			if(ADC_CS_PIN == ADC4_CS_B)
			{
				DEBUG_PRINT(UARTprintf("ADC 4 didn't program channel scan register correctly! \n");)

				gui32Error |= ADC4_FAIL;
				update_Error();
			}

			DEBUG_PRINT(
			uint8_t ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 2, 1);
			UARTprintf("IO Extender 1 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg2);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);
			UARTprintf("IO Extender 1 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg3);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);
			UARTprintf("IO Extender 2 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg2);
			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
			UARTprintf("IO Extender 2 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg3);
#ifdef MCU_ZXR
			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN));
#else
			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1));
#endif
			)
		}
	}

	float V_sum = 0;	// Sum of all voltages read

	// Read from ADC_DOUT registers one time before before using the data for averaging
	// Ondrej found this chip often gave incorrect value for first sample so throw away this first point
	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 3, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
	SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // First byte received is junk, data overwritten on next line
	SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // Save MSB
	SSIDataGet(SSI1_BASE, &ui32ADC_LSB); // Save LSB

	SysCtlDelay(SysCtlClockGet()/3 * .005);

	while(samples > 0)
	{
		// Read from ADC_DOUT registers
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 3, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers
		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // First byte received is junk, data overwritten on next line
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // Save MSB
		SSIDataGet(SSI1_BASE, &ui32ADC_LSB); // Save LSB

		ADC_DOUT = 0;
		ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
		ADC_DOUT |= ui32ADC_LSB; // Set LSB

		fVoltage = (ADC_DOUT * (1500) / 32768.0) + 1500; // Convert ADC_DOUT data into voltage
		V_sum += fVoltage;
//		UARTprintf("Voltage: \t %d \t mV \n", (int) fVoltage);

		samples--;

//		UARTprintf("%d\t", (int) (fVoltage * 1000));

		if(samples > 0)
			SysCtlDelay(SysCtlClockGet()/3 * .005);
	}
//	UARTprintf("\n");

	// Set ADC back into stand-by mode
	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode

	float V_avg = V_sum / total_samples;
//	UARTprintf("Average: \t %d \t mV \n", (int) V_avg);

	return V_avg;
}

////**************************************************************************
//// Function to read in voltage on specific channel, continuously while printing
//// out to UART, used to check for noise
//// Samples at ~200 SPS, max of 20 samples
//// Parameters:	Channel; [0,6] 	0: All amperometrics
////								1: Amp 1
////								2: Amp 2... etc
////				ADC_CS_PIN; ADC1_CS_B, ADC2_CS_B, or ADC4_CS_B
////				time; how long to stream channel for
//// Outputs:		last voltage read
////**************************************************************************
//float StreamADCChannel(uint8_t ui8Channel, int ADC_CS_PIN, int time)
//{
//	float fVoltage;
//
//	uint8_t ADC_Address = 0x10; // Instruction 1, must be sent whenever URA Changes
//	uint8_t URA = 0x01; // Upper Register Address; Channel Scan Register and ADC_DOUT register have same URA
//	uint8_t LRA_ch_scan_w = 0x0F; // Lower Register Address; write 1 byte to channel scan register
//	uint8_t LRA_ch_scan_r = 0x8F; // Read 1 byte from channel scan register
//	uint8_t LRA_ADC_DOUT = 0xAA; // Read 2 bytes from ADC_DOUT registers
//
//	uint8_t ui8Channel_Tx = (ui8Channel | ((ui8Channel) << 3)); // Set first and last channel to scan
//
//	uint32_t ui32ADC_MSB; // Most Significant byte of ADC_DOUT
//	uint32_t ui32ADC_LSB; // Least Signficant byte of ADC_DOUT
//	int16_t ADC_DOUT = 0; // signed 16 bit data assembled from two ADC_DOUT registers
//	uint32_t ui32Channel_Rx = ~ui8Channel_Tx;
//
//	uint8_t counter = 0;
//
//	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//
//	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//	SSIDisable(SSI1_BASE);
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//	SSIEnable(SSI1_BASE);
//
//	SysCtlDelay(2000);
//
//	// Random data points seem really far off, as if the channel wasn't changed successfully and wrong channel was being read
//	// Created loop to verify the channel register was updated correctly
//	while(ui8Channel_Tx != ui32Channel_Rx && counter <= 15)
//	{
//		// Set ADC we are using into active mode
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x00); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//		// Write the channel scan register to set which channel to read
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_w, ui8Channel_Tx); // Set channel scan register
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
//		SysCtlDelay(30000); // Delay so channel scan register can update
//
////		UARTprintf("Reading back Channel \n");
//
//		// Must read channel scan register after setting it
//		// Assume this is to update values as register is buffered
//		// Don't actually know why this is required
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_r, 0x00); // Read Channel Scan Register
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//
//		if(counter < 5)
//			counter++;
//		else if(counter == 5)
//		{
//			DEBUG_PRINT(UARTprintf("Tried writing ADC channel 5 times, reinitializing all ADCs!\n");)
//
//			InitADC();
//			counter++;
//		}
//		else if(counter < 10)
//			counter++;
//		else if(counter == 10)
//		{
//			DEBUG_PRINT(
//			UARTprintf("Tried writing ADC channel 10 times, failed everytime! \n");
//			UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);
//			UARTprintf("Resetting analog board!\n");
//			)
//			counter++;
//
//			AnalogOff();
//
//			userDelay(1000, 1);
//
//			SysCtlDelay(2000);
//
//			InitAnalog();
//
//			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//			SSIDisable(SSI1_BASE);
//			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//			SSIEnable(SSI1_BASE);
//
//			SysCtlDelay(2000);
//		}
//		else if(counter < 15)
//			counter++;
//		else if(counter == 15)
//		{
//			counter++;
//			DEBUG_PRINT(UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);)
//
//			if(ADC_CS_PIN == ADC1_CS_B)
//			{
//				DEBUG_PRINT(UARTprintf("ADC 1 didn't program channel scan register correctly! \n");)
//
//				gui32Error |= ADC1_FAIL;
//				update_Error();
//			}
//			if(ADC_CS_PIN == ADC2_CS_B)
//			{
//				DEBUG_PRINT(UARTprintf("ADC 2 didn't program channel scan register correctly! \n");)
//
//				gui32Error |= ADC2_FAIL;
//				update_Error();
//			}
//			if(ADC_CS_PIN == ADC4_CS_B)
//			{
//				DEBUG_PRINT(UARTprintf("ADC 4 didn't program channel scan register correctly! \n");)
//
//				gui32Error |= ADC4_FAIL;
//				update_Error();
//			}
//
//			DEBUG_PRINT(
//			uint8_t ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 2, 1);
//			UARTprintf("IO Extender 1 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg2);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);
//			UARTprintf("IO Extender 1 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg3);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);
//			UARTprintf("IO Extender 2 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg2);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
//			UARTprintf("IO Extender 2 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg3);
//#ifdef MCU_ZXR
//			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN));
//#else
//			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1));
//#endif
//			)
//		}
//	}
//
////	float V_sum = 0;	// Sum of all voltages read
//
//	// Read from ADC_DOUT registers one time before before using the data for averaging
//	// Ondrej found this chip often gave incorrect value for first sample so throw away this first point
//	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 3, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers
//	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//	SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // First byte received is junk, data overwritten on next line
//	SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // Save MSB
//	SSIDataGet(SSI1_BASE, &ui32ADC_LSB); // Save LSB
//
//	SysCtlDelay(SysCtlClockGet()/3 * .005);
//
//	g_TimerInterruptFlag = 0;
//	TimerLoadSet(TIMER0_BASE, TIMER_A, (time * SysCtlClockGet()));		// Timer set
//	TimerEnable(TIMER0_BASE, TIMER_A);
//
//	DEBUG_PRINT(UARTprintf("Voltage at ISE (not ADC) (uV):\n");)
//	uint32_t ui32ADC_Done = 0xFF;
//	uint8_t LRA_ADC_DOUT_check = 0xE8;	// Start reading at ADC Data Available register to check if the data coming out is new
//	while(g_TimerInterruptFlag == 0)
//	{
//		// Read from ADC_DOUT registers
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 5, LRA_ADC_DOUT_check, 0x00, 0x00, 0, 0); // Read 2 bytes from ADC_DOUT registers
//		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // First byte received is junk, data overwritten on next line
//		SSIDataGet(SSI1_BASE, &ui32ADC_Done); // ADC Data Availabe register
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // Senosr Diagnostic Flags register
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // Save MSB
//		SSIDataGet(SSI1_BASE, &ui32ADC_LSB); // Save LSB
//
//		if(ui32ADC_Done != 0xFF)
//		{
//			ADC_DOUT = 0;
//			ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
//			ADC_DOUT |= ui32ADC_LSB; // Set LSB
//
//			fVoltage = (ADC_DOUT * (1500) / 32768.0) + 1500; // Convert ADC_DOUT data into voltage
//			//		V_sum += fVoltage;
//			DEBUG_PRINT(UARTprintf("%d\n", (int) ((2 * fVoltage - 3000)/5 * 1000));)
//		}
//
////		samples--;
////
//////		UARTprintf("%d\t", (int) (fVoltage * 1000));
////
////		if(samples > 0)
////			SysCtlDelay(SysCtlClockGet()/3 * .005);
//	}
////	UARTprintf("\n");
//
//	// Set ADC back into stand-by mode
//	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
////	float V_avg = V_sum / total_samples;
////	UARTprintf("Average: \t %d \t mV \n", (int) V_avg);
//
//	return fVoltage;
//}

//**************************************************************************
// Function to control current source for temperature sensor outputs 1000 uA
// Parameters:	state; true - on
//					   false - off
//**************************************************************************
void ADCCurrentSet(bool state)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);



	uint8_t ADC_Address = 0x10;		// First byte to send is ADC address
	uint8_t URA = 0x01;				// Seconds byte is upper register address
	uint8_t LRA = 0x02;				// Third byte is lower register address and num of bytes
//	uint8_t Data = 0x0A * state;	// 0x0A Sets 1000 uA Fourth byte is byte to write into register
	uint8_t Data = 0x01 * state;	// 0x01 sets 100 uA Fourth byte is byte to write into register

	// Set ADC we are using into active mode

	if(state == 1)
		SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, ADC_Address, 0x00, 0x08, 0x00); // Register 0x08, 0x03 stand-by mode, 0x00 active mode

	SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, ADC_Address, URA, LRA, Data); // Write to ADC_AUXCN register

	if(state == 0)
		SPISend(SSI1_BASE, 1, ADC1_CS_B, 0, 4, ADC_Address, 0x00, 0x08, 0x00); // Register 0x08, 0x03 stand-by mode, 0x00 active mode
}

//**************************************************************************
// Function to write to the EEPROM; CAT24C512
// Parameters:	page; Page to be written to; 0-511
//				byte; Byte on page to start writing on; 0-127
//				num_of_bytes; number of bytes to write to page
//				*pui8Data; pointer to data to be written; input the address
//**************************************************************************
void MemoryWrite(unsigned int page, unsigned int byte, unsigned int num_of_bytes, uint8_t *pui8DataTx)
{
	unsigned int i;
	uint32_t Error = I2C_MASTER_ERR_NONE;

	if(gBoard > V1)
	{
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("Memory Write \n");
			)
		}
		update_TivaI2C(1);
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("Memory Write \n");
			)
		}
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
		{
			if(gDiagnostics >= 2)
				UARTprintf("Memory Write \n");
		}
		update_TivaI2C(1);
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
		{
			if(gDiagnostics >= 2)
				UARTprintf("Memory Write \n");
		}
#endif
	}

	DEBUG_PRINT(
	if(gBoard > V1 && gDiagnostics >= 1)
		UARTprintf("Writing to memory on I2C \n");
	)

	// Memory requires 3 bytes to begin commnuication 1st byte is address, second byte is
	// 8 msbs of page, third byte is lsb of page then 7 bits for starting byte
	if(GMEMORY == MEMORY_1M)
	{
		while(byte > 255)
		{
			page++;
			byte -= 256;
		}

		I2CMasterSlaveAddrSet(I2C0_BASE, (MEM_ADDR | ((page >> 8) & 1)), false); // Set Address and R/W bit
		I2CMasterDataPut(I2C0_BASE, page); // Shift page bits to send 8 mSbs
	}
	else
	{
		while(byte > 127)
		{
			page++;
			byte -= 128;
		}

		byte |= ((page & 1) << 7); // Set lSb of page as mSb of byte

		I2CMasterSlaveAddrSet(I2C0_BASE, MEM_ADDR, false); // Set Address and R/W bit
		I2CMasterDataPut(I2C0_BASE, (page >> 1)); // Shift page bits to send 8 mSbs
	}

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Start transfer
	while(I2CMasterBusy(I2C0_BASE));

	I2CMasterDataPut(I2C0_BASE, byte); // Send byte, second byte of address
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	while(I2CMasterBusy(I2C0_BASE));

	for(i = 0; i < (num_of_bytes - 1); i++) // Loop through pointer
	{
		I2CMasterDataPut(I2C0_BASE, *(pui8DataTx + i));
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
		while(I2CMasterBusy(I2C0_BASE));
	}

	I2CMasterDataPut(I2C0_BASE, *(pui8DataTx + num_of_bytes - 1));
	Error = I2CMasterErr(I2C0_BASE);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE));

//	if(Error & I2C_MASTER_ERR_ADDR_ACK)
//		UARTprintf("Adrress Ack\n");
//	if(Error & I2C_MASTER_ERR_DATA_ACK)
//		UARTprintf("Data Ack\n");
//	if(Error & I2C_MASTER_ERR_ARB_LOST)
//		UARTprintf("Arb Lost\n");

	SysCtlDelay(.005 * SysCtlClockGet()/3); // Delay write cycle time; 5 ms;

	update_TivaI2C(0);	// Tell BT I2C is free before going into memory read

	// Check for I2C error, if none occurred read back data to verify all was written correctly
	if(Error == I2C_MASTER_ERR_NONE)
	{
		// Read back what was just written to determine if it worked
		uint8_t * pui8DataRx;
		if(GMEMORY == MEMORY_1M)
			pui8DataRx = MemoryRead(page, byte, num_of_bytes);	// & byte to remove the bit added from page
		else
			pui8DataRx = MemoryRead(page, (byte & 0x7F), num_of_bytes);	// & byte to remove the bit added from page

		for(i = 0; i < num_of_bytes; i++)
		{
			if(*(pui8DataRx + i) != *(pui8DataTx + i))
			{
				gui32Error |= MEMORY_FAILED;
//				UARTprintf("Write Memory, read back didn't match!\n");
				DEBUG_PRINT(
				if(gDiagnostics >= 1)
					UARTprintf("Write Memory, read back didn't match!\n");
				)

				update_Error();
			}
		}
	}
	else	// I2C error occurred (address or data wasn't ack'd by slave)
	{
		gui32Error |= MEMORY_FAILED;
//		UARTprintf("Write Memory I2C failure!\n");

		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Writing to memory failed!\n");
		)

		update_Error();
	}

}

//**************************************************************************
// Function to read from the EEPROM; CAT24C512
// Parameters:	page; Page to be read from; 0-511
//				byte; Byte on page to start reading from; 0-127
//				num_of_bytes; number of bytes to read from page
// Returns:		pui8DataRx; pointer to array holding all data
//**************************************************************************
uint8_t * MemoryRead(unsigned int page, unsigned int byte, unsigned int num_of_bytes)
{
	// Array must be called as static to prevent changes in memory when exiting function
	static uint8_t pui8DataRx[128]; // Create array large enough to hold any transfer
	unsigned int i;
	uint32_t Error = I2C_MASTER_ERR_NONE;

	if(gBoard > V1)
	{
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("Memory Read \n");
			)
		}
		update_TivaI2C(1);
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("Memory Read \n");
			)
		}
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
		{
			if(gDiagnostics >= 2)
				UARTprintf("Memory Read \n");
		}
		update_TivaI2C(1);
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
		{
			if(gDiagnostics >= 2)
				UARTprintf("Memory Read \n");
		}
#endif

	}

//	if(gBoard > V1 && gDiagnostics >= 1)
//		UARTprintf("Reading Memory on I2C \n");

	// Memory requires 3 bytes to begin commnuication 1st byte is address, second byte is
	// 8 msbs of page, third byte is lsb of page then 7 bits for starting byte
	if(GMEMORY == MEMORY_1M)
	{
		while(byte > 255)
		{
			page++;
			byte -= 256;
		}

		I2CMasterSlaveAddrSet(I2C0_BASE, (MEM_ADDR | ((page >> 8) & 1)), false); // Set address with R/W bit
		I2CMasterDataPut(I2C0_BASE, page); // Load data to be sent with 8 mSb of page
	}
	else
	{
		while(byte > 127)
		{
			page++;
			byte -= 128;
		}

		byte |= ((page & 1) << 7); // Set lSb of page as mSb of byte

		I2CMasterSlaveAddrSet(I2C0_BASE, MEM_ADDR, false); // Set address with R/W bit
		I2CMasterDataPut(I2C0_BASE, (page >> 1)); // Load data to be sent with 8 mSb of page
	}

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send slave address and page
    while(I2CMasterBusy(I2C0_BASE));

	I2CMasterDataPut(I2C0_BASE, byte); // Load data to be sent with byte
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Send byte
	while(I2CMasterBusy(I2C0_BASE));

    //specify that we are going to read from slave device
	if(GMEMORY == MEMORY_1M)
		I2CMasterSlaveAddrSet(I2C0_BASE, (MEM_ADDR | ((page >> 8) & 1)), true);
	else
		I2CMasterSlaveAddrSet(I2C0_BASE, MEM_ADDR, true);

    if(num_of_bytes == 1)
    {
    	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    	while(I2CMasterBusy(I2C0_BASE));
    	pui8DataRx[0] = I2CMasterDataGet(I2C0_BASE);
    	Error = I2CMasterErr(I2C0_BASE);
    }
    else
    {
    	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    	while(I2CMasterBusy(I2C0_BASE));
    	pui8DataRx[0] = I2CMasterDataGet(I2C0_BASE);

    	for(i = 1; i < num_of_bytes - 1; i++)
    	{
    		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    		while(I2CMasterBusy(I2C0_BASE));
    		pui8DataRx[i] = I2CMasterDataGet(I2C0_BASE);
    	}


    	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    	while(I2CMasterBusy(I2C0_BASE));
    	pui8DataRx[num_of_bytes - 1] = I2CMasterDataGet(I2C0_BASE);
    	Error = I2CMasterErr(I2C0_BASE);
    }

//    if(Error & I2C_MASTER_ERR_ADDR_ACK)
//    	UARTprintf("Adrress Ack\n");
//    if(Error & I2C_MASTER_ERR_DATA_ACK)
//    	UARTprintf("Data Ack\n");
//    if(Error & I2C_MASTER_ERR_ARB_LOST)
//    	UARTprintf("Arb Lost\n");

    update_TivaI2C(0);

	if(Error != I2C_MASTER_ERR_NONE)
	{
		gui32Error |= MEMORY_FAILED;
//		UARTprintf("Read Memory I2C failure!\n");

		update_Error();
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Reading from memory failed!\n");
		)
	}

    return pui8DataRx;
}

//**************************************************************************
// Sets up battery IC to send alert signal when battery goes below 20% or
// above 95%, alert signal is active low also set initial charging LED states
// Part: MAX17201
// Parameters:	NONE
//**************************************************************************
void InitBattery(void)
{
	if(gBoard > V1)
	{
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
#endif
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("Init Battery \n");
			)
			SysCtlDelay(SysCtlClockGet()/300);
		}
	}

	DEBUG_PRINT(
	if(gBoard > V1 && gDiagnostics >= 1)
		UARTprintf("Initializing battery IC over I2C \n");
	)

	if(gBoard > V1)
	{
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
		update_TivaI2C(1);
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
		update_TivaI2C(1);
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
#endif

		uint16_t reg_val = 0, config_val = 0;
		uint8_t attempts = 0;
		while((reg_val != 0x5F14 || config_val != 0x0004) && attempts < 3)
		{
			uint8_t reg_address = 0x03;
			I2CSend(I2C0_BASE, BAT_SLAVE_ADDR, 3, reg_address, 20, 95);	// Program alert pin to interrupt if SOC goes below 20 or above 95 0x14, 0x5F

			uint8_t config_reg = 0x1D;
			I2CSend(I2C0_BASE, BAT_SLAVE_ADDR, 3, config_reg, 0x04, 0x00);	// Enable alerts and set active low

			// Read back from battery chip to verify registers wrote correctly
			reg_val = I2CReceive(I2C0_BASE, BAT_SLAVE_ADDR, reg_address, 2);
			config_val = I2CReceive(I2C0_BASE, BAT_SLAVE_ADDR, config_reg, 2);

			attempts++;
		}

		if((reg_val != 0x5F14 || config_val != 0x0004) && attempts == 3)
		{
			DEBUG_PRINT(UARTprintf("Battery gauge failed 3 times to initialize correctly!\n");)
			gui32Error |= BATTERY_FAIL;
			update_Error();
		}

		update_TivaI2C(0);

		update_Battery(gPumping);

#ifdef MCU_ZXR
		// Setup and enable interrupts for battery pins
		GPIOPinTypeGPIOInput(IO_BAT_ALERT_BASE, IO_BAT_ALERT_PIN); // Battery alert
		GPIOIntTypeSet(IO_BAT_ALERT_BASE, IO_BAT_ALERT_PIN, GPIO_BOTH_EDGES);
		GPIOIntEnable(IO_BAT_ALERT_BASE, IO_BAT_ALERT_PIN);

		GPIOIntClear(IO_BAT_CHARGING_BASE, IO_BAT_CHARGING_PIN);
		GPIOIntClear(IO_BAT_ALERT_BASE, IO_BAT_ALERT_PIN);

		if(gBoard >= V6_3)
		{
			GPIOPinTypeGPIOInput(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN); // Power good _B
			GPIOIntTypeSet(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN, GPIO_BOTH_EDGES);
			GPIOIntEnable(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN);
			GPIOIntClear(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN);
		}
		else
		{
			GPIOPinTypeGPIOInput(IO_BAT_CHARGING_BASE, IO_BAT_CHARGING_PIN); // Battery charging
			GPIOIntTypeSet(IO_BAT_CHARGING_BASE, IO_BAT_CHARGING_PIN, GPIO_BOTH_EDGES);
			GPIOIntEnable(IO_BAT_CHARGING_BASE, IO_BAT_CHARGING_PIN);
		}

		IntPrioritySet(INT_BATTERY_BASE, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits
		IntEnable(INT_BATTERY_BASE);
#else
		// Setup and enable interrupts for battery pins
		GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1); // Battery alert
		GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_BOTH_EDGES);
		GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_1);

		GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_0);
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_1);

		if(gBoard >= V6_3)
		{
			GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4); // Power good _B
			GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_BOTH_EDGES);
			GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_4);
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_4);
		}
		else
		{
			GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0); // Battery charging
			GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
			GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_0);
		}

		IntPrioritySet(INT_GPIOE, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits
		IntEnable(INT_GPIOE);
#endif
	}
}

//**************************************************************************
// Function to read from Battery registers
// Parameters:	Register; register address of value wanted
// Registers:	REP_CAP_REG;	Reported capacity register, mAh, Capacities have to be divided by R_sense (0.010 Ohms)
//				REP_SOC_REG;	State of charge register, Percentage
//				TTE_REG;		Estimated time to empty register, Time (min)
//				TTF_REG;		Estimated time to full register, Time (min)
//				AGE_REG;		Calculated % value of present cell capacity compared to expected capacity
//				CYC_REG;		Total cound of the number of charge/dischard cycles, % of full cycle, full range of 0 to 10485 cycles with a 16% LSb
//**************************************************************************
unsigned int BatteryRead(uint8_t ui8Register)
{
	DEBUG_PRINT(
	if(gBoard > V1 && gDiagnostics >= 1)
		UARTprintf("Reading battery over I2C \n");
	)
	while(I2CMasterBusy(I2C0_BASE));

#ifdef MCU_ZXR
	if(gBoard > V1)
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("Battery Read \n");
			)
		}

	update_TivaI2C(1);
	while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
#else
	if(gBoard > V1)
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
		{
			if(gDiagnostics >= 2)
				UARTprintf("Battery Read \n");
		}

	update_TivaI2C(1);
	while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
#endif

	unsigned int reg_val = I2CReceive(I2C0_BASE, BAT_SLAVE_ADDR, ui8Register, 2);

	update_TivaI2C(0);

//	unsigned int ulerrorstate = I2CMasterErr(I2C1_BASE);

	if(ui8Register == REP_CAP_REG) // Capacity registers
		reg_val *= .5; // mAh
	else if(ui8Register == REP_SOC_REG) // Percentage registers
		reg_val /= 256; // Percent
	else if(ui8Register == TTE_REG || ui8Register == TTF_REG) // Time registers
		reg_val *= 5.625/60; // Minutes
	else if(ui8Register == AGE_REG)	// Age register
		reg_val /= 256; // Percent
	else if(ui8Register == CYC_REG)	// Cycles register
		reg_val /= 6.25; // Percent with LSb of 16%, by dividing by 6.25 I get full cycle count (truncating partial cycle)

	return reg_val;
}

#ifdef CONST_COND_FREQ
//**************************************************************************
// Initializes waveform generator to 1 kHz or 100 Hz, leaves reset bit set
// Parameters:	Check_freq; 1 to check that frequency set correctly, 0 to skip
//**************************************************************************
void InitWaveGen(uint8_t Check_freq)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	uint8_t Check = 0;
	uint8_t Counter = 0;

	// Set low current range
	// 10.7 uApp R = 309k + 499k = 808k
	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

	while(Check == 0 && Counter <= 10)
	{
		//0x21 0x00: Write to control register to turn on reset bit
		//0x54 0xF9: Write to frequency LSB register: (0x54 0xF9 for 100 Hz)(0x51 0xB7 for 1 kHz)
		//0x40 0x00: Write to frequency MSB register: (0x40 0x00 for 100 Hz)(0x40 0x03 for 1 kHz)
		//0xC0 0x00: Write to phase register, 0 for no offset
		//0x20 0x00: Write to control register to turn off reset bit: taken care of in WaveGetSet() function
//		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, 0x54, 0xF9, 0x40, 0x00, 0xC0, 0x00);	// 100 Hz
//		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, 0x51, 0xB7, 0x40, 0x03, 0xC0, 0x00);	// 1 kHz
//		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, 0x58, 0x93, 0x40, 0x10, 0xC0, 0x00);	// 5 kHz

		uint32_t Freq_Reg = ((float) COND_FREQ / 5000000.0 * 268435456.0);
		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, (Freq_Reg >> 8 & 0x3F) | 0x40, (Freq_Reg & 0xFF), (Freq_Reg >> 22 & 0x3F) | 0x40, (Freq_Reg >> 14 & 0xFF), 0xC0, 0x00);	// 5 kHz

		if(Check_freq == 0)
			break;

		if(gABoard >= AV6_6)
		{
			WaveGenSet(1);

			Check = CheckCond();

			WaveGenSet(0);

			if(Counter < 3)
				Counter++;
			else if(Counter == 3)
			{
				DEBUG_PRINT(
				UARTprintf("Tried initializing waveform generator 5 times, not showing %d Hz!\n", COND_FREQ);
				UARTprintf("Resetting analog board!\n");
				)

				AnalogOff();

				userDelay(1000, 1);

#ifdef MCU_ZXR
				GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

				// Set pin low, turn on analog board, then set high
				GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
				GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

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

				if(gBoard == V6_2 || gBoard == V6_3)
				{
					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
					GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
				}
#else
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

				// Set pin low, turn on analog board, then set high
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

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

				if(gBoard == V6_2 || gBoard == V6_3)
				{
					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
				}
#endif
				InitIO_Ext();		// Sets up IO Extenders and sets initial values

				if(gBoard == V6_2 || gBoard == V6_3)
					InitLED_Ext();
				else
					SetLED(0, 1);

				InitDAC();			// Resets device and writes configuration register
				InitTurbidityADC();
				InitADC();			// Resets devices and configures all channels

				// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
				SSIDisable(SSI1_BASE);
				SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, SPI_CLOCK, 8);
				SSIEnable(SSI1_BASE);

				Counter++;
			}
			else if(Counter < 10)
				Counter++;
			else if(Counter == 10)
			{
				DEBUG_PRINT(
				UARTprintf("Tried initializing waveform generator 10 times, not showing %d Hz!\n", COND_FREQ);
				UARTprintf("Setting error and leaving initialization!\n");
				)
				gui32Error |= WAVE_GEN_FAIL;
				update_Error();

				Counter++;
			}
		}
		else
			Check = 1;
	}

	// Set SPI communication polarity back to idle low after talking to waveform generator
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);


//	if(gCartridge > 0)
//	{
//		// Measure conductivity
//		// Set RE and CE floating and close RE/CE loop for conductivity
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//		// Set high current range
//		// 45 uApp R = 180k
//		IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//		IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);
//
//		float CondReading_1 = ConductivityMovingAvg();
//
//		WaveGenSet(1);
//
//		float CondReading_2 = ConductivityMovingAvg();
//
//		WaveGenSet(0);	// Turn off waveform generator when switching ranges
//
////		UARTprintf("Cond: %d, %d\n", (int) CondReading_1, (int) CondReading_2);
//		//
//		if(CondReading_2 * .9 <= CondReading_1)
//		{
//			UARTprintf("Waveform gen failed check!\n");
//
//			gui32Error |= WAVE_GEN_FAIL;
//			update_Error();
//		}
//	}

}
#else
//**************************************************************************
// Initializes waveform generator to 1 kHz or 100 Hz, leaves reset bit set
// Parameters:	Check_freq; 1 to check that frequency set correctly, 0 to skip
//**************************************************************************
void InitWaveGen(uint8_t Check_freq, uint16_t ui16freq)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	uint8_t Check = 0;
	uint8_t Counter = 0;

	// Set low current range
	// 10.7 uApp R = 309k + 499k = 808k
	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

	while(Check == 0 && Counter <= 10)
	{
		//0x21 0x00: Write to control register to turn on reset bit
		//0x54 0xF9: Write to frequency LSB register: (0x54 0xF9 for 100 Hz)(0x51 0xB7 for 1 kHz)
		//0x40 0x00: Write to frequency MSB register: (0x40 0x00 for 100 Hz)(0x40 0x03 for 1 kHz)
		//0xC0 0x00: Write to phase register, 0 for no offset
		//0x20 0x00: Write to control register to turn off reset bit: taken care of in WaveGetSet() function
//		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, 0x54, 0xF9, 0x40, 0x00, 0xC0, 0x00);	// 100 Hz
//		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, 0x51, 0xB7, 0x40, 0x03, 0xC0, 0x00);	// 1 kHz
//		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, 0x58, 0x93, 0x40, 0x10, 0xC0, 0x00);	// 5 kHz

		uint32_t Freq_Reg = ((float) ui16freq / 5000000.0 * 268435456.0);
		SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 8, 0x21, 0x00, (Freq_Reg >> 8 & 0x3F) | 0x40, (Freq_Reg & 0xFF), (Freq_Reg >> 22 & 0x3F) | 0x40, (Freq_Reg >> 14 & 0xFF), 0xC0, 0x00);	// 5 kHz

		if(Check_freq == 0)
			break;

		if(gABoard >= AV6_6)
		{
			WaveGenSet(1);

			Check = CheckCond(COND_FREQ);

			WaveGenSet(0);

			if(Counter < 3)
				Counter++;
			else if(Counter == 3)
			{
				DEBUG_PRINT(
				UARTprintf("Tried initializing waveform generator 5 times, not showing %d Hz!\n", ui16freq);
				UARTprintf("Resetting analog board!\n");
				)

				AnalogOff();

				userDelay(1000, 1);

#ifdef MCU_ZXR
				GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

				// Set pin low, turn on analog board, then set high
				GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
				GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

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

				if(gBoard == V6_2 || gBoard == V6_3)
				{
					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
					GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
				}
#else
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

				// Set pin low, turn on analog board, then set high
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

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

				if(gBoard == V6_2 || gBoard == V6_3)
				{
					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
				}
#endif
				InitIO_Ext();		// Sets up IO Extenders and sets initial values

				if(gBoard == V6_2 || gBoard == V6_3)
					InitLED_Ext();
				else
					SetLED(0, 1);

				InitDAC();			// Resets device and writes configuration register
				InitTurbidityADC();
				InitADC();			// Resets devices and configures all channels

				// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
				SSIDisable(SSI1_BASE);
				SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, SPI_CLOCK, 8);
				SSIEnable(SSI1_BASE);

				Counter++;
			}
			else if(Counter < 10)
				Counter++;
			else if(Counter == 10)
			{
				DEBUG_PRINT(
				UARTprintf("Tried initializing waveform generator 10 times, not showing %d Hz!\n", ui16freq);
				UARTprintf("Setting error and leaving initialization!\n");
				)
				gui32Error |= WAVE_GEN_FAIL;
				update_Error();

				Counter++;
			}
		}
		else
			Check = 1;
	}

	// Set SPI communication polarity back to idle low after talking to waveform generator
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);


//	if(gCartridge > 0)
//	{
//		// Measure conductivity
//		// Set RE and CE floating and close RE/CE loop for conductivity
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//		// Set high current range
//		// 45 uApp R = 180k
//		IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//		IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);
//
//		float CondReading_1 = ConductivityMovingAvg();
//
//		WaveGenSet(1);
//
//		float CondReading_2 = ConductivityMovingAvg();
//
//		WaveGenSet(0);	// Turn off waveform generator when switching ranges
//
////		UARTprintf("Cond: %d, %d\n", (int) CondReading_1, (int) CondReading_2);
//		//
//		if(CondReading_2 * .9 <= CondReading_1)
//		{
//			UARTprintf("Waveform gen failed check!\n");
//
//			gui32Error |= WAVE_GEN_FAIL;
//			update_Error();
//		}
//	}

}
#endif


//**************************************************************************
// Turn on or off the waveform generator by controlling clock source and
// setting/clearing the reset bit
// No error checking in this function because the MeasureConductivity
// function verifies wave gen starts and frequency is correct
// Parameters:	state: 1 - ON
//					   0 - OFF
//**************************************************************************
void WaveGenSet(bool state)
{
	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	SPISend(SSI1_BASE, 1, WAVE_GEN_CS_B, 0, 2, (0x21 - state), 0x00);	// Set/clear reset bit

	// Set SPI communication polarity back to idle low after talking to waveform generator
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	if(state == 1)
	{
		IO_Ext_Set(IO_EXT1_ADDR, 3, OSC_EN, state);							// Clock source for wave gen
		IO_Ext_Set(IO_EXT1_ADDR, 3, COND_SW, state);						// Input for switch connecting circuit
		if(gABoard >= AV6_2)
			IO_Ext_Set(IO_EXT2_ADDR, 3, COND_SHORT_SW, 0);					// Disconnect feedback switch over conductivity op-amp
	}
	else
	{
		if(gABoard >= AV6_2)
			IO_Ext_Set(IO_EXT2_ADDR, 3, COND_SHORT_SW, 1);					// Reconnect feedback switch over conductivity op-amp
		SysCtlDelay(SysCtlClockGet()/3000 * 1000);
		IO_Ext_Set(IO_EXT1_ADDR, 3, COND_SW, state);						// Input for switch connecting circuit
		IO_Ext_Set(IO_EXT1_ADDR, 3, OSC_EN, state);							// Clock source for wave gen
	}
}

//**************************************************************************
// Interrupt handler for battery alert and battery charging pins, reads the
// battery state of charge as a percentage and checks the charging status
// Can be triggered either by pins or by periodic timer 2 used during idle
// state
// Updates BT chip on percentage and charging status
// Parameters: NONE
//**************************************************************************
void BatteryIntHandler(void)
{
#ifdef MCU_ZXR
	GPIOIntClear(IO_BAT_ALERT_BASE, IO_BAT_CHARGING_PIN | IO_BAT_ALERT_PIN);

	if(gBoard >= V6_3)
		GPIOIntClear(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN);
#else
	GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);

	if(gBoard >= V6_3)
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_4);
#endif


	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

//	g_TimerTemperatureInterruptFlag = 1;	// Piggy backing off battery interrupt timer to record device temperature as well, record temperature function will execute in the idle state

//	UARTprintf("BatteryIntHandler\n");
	update_Battery(gPumping);
}

//**************************************************************************
// Makes the buzzer beep twice in quick succession
// Parameters:  uiCycles, adjusts how long buzzer beeps for, 400
// 12/27/2022: Added #ifdef to handle change in MCU for Roam Digital V1
//**************************************************************************
void BuzzerSound(unsigned int uiCycles)
{
	unsigned int i;
	unsigned long freq = 4000; // Sets the frequency for buzzer signal
	unsigned long delay_cycles = (SysCtlClockGet()/3)/freq/2; // # of cycles to delay to produce required frequency

#ifdef MCU_ZXR
	unsigned int GPIO_Base = IO_BUZZER_BASE(gBoard);

	for(i = 0; i < uiCycles; i++)
	{
		GPIOPinWrite(GPIO_Base, IO_BUZZER_PIN, IO_BUZZER_PIN);
		SysCtlDelay(delay_cycles);
		GPIOPinWrite(GPIO_Base, IO_BUZZER_PIN, 0x00);
		SysCtlDelay(delay_cycles);
	}

	SysCtlDelay(800000);

	for(i = 0; i < uiCycles; i++) // Second for loop makes it beep twice
	{
		GPIOPinWrite(GPIO_Base, IO_BUZZER_PIN, IO_BUZZER_PIN);
		SysCtlDelay(delay_cycles);
		GPIOPinWrite(GPIO_Base, IO_BUZZER_PIN, 0x00);
		SysCtlDelay(delay_cycles);
	}
#else
	for(i = 0; i < uiCycles; i++)
	{
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);
		SysCtlDelay(delay_cycles);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x00);
		SysCtlDelay(delay_cycles);
	}

	SysCtlDelay(800000);

	for(i = 0; i < uiCycles; i++) // Second for loop makes it beep twice
	{
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);
		SysCtlDelay(delay_cycles);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0x00);
		SysCtlDelay(delay_cycles);
	}
#endif
}

////**************************************************************************
//// Initialize accelerometer to watch for increases in x,y g's over .252 g's
//// and to watch for z g's to drop below .882 g's, both of these events will
//// trigger interrupts, x,y on ACC_INT1 pin and z on ACC_INT2 pin
//// Parameters:	NONE
//// Returns:		NONE
////**************************************************************************
//void InitAccel(void)
//{
//	// Setup and enable interrupts for accelerometer
//	// Interrupt signals are active low, this can be changed by writing 0x02 into register 0x2C
//	// GPIO_LOW_LEVEL will continue firing until device is returned to flat
//	// GPIO_FALLING_EDGE will fire only once when device is tilted away from flat
//	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4); // Acc Int 1
//	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
//	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_4);
//	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_5); // Acc Int 2
//	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
//	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_5);
//	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_4);
//	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_5);
//	IntEnable(INT_GPIOD);
//	IntPrioritySet(INT_GPIOD, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits
//
//	// Send reset command to device
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 2, 0x2B, 0x40);
//
//	SysCtlDelay(SysCtlClockGet()/30000 * 5);	// Delay 500 us after resetting device so I2C is ready for operation
//
//	// Set up Freefall/Motion configuration register to watch for motion on z axis
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 2, 0x15, 0x20);			// Set freefall mode so Z axis has to drop below .882 g's to trigger
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 3, 0x17, 0x8E, 0x03);	// Set motion threshold to .882 g's and debounce to 240 ms
//
//	// Set up TRANSIENT_CFG register
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 2, 0x1D, 0x07);			// Watch for acceleration greater than threshold on x and y axes
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 3, 0x1F, 0x84, 0x03);	// Set motion threshold to .252 g's and debounce to 240 ms
//
//	// Set up active mode to run in Low Power Mode
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 2, 0x2B, 0x03);
//
//	// Enable freefall/motion interrupt and route to INT 1 pin, enable transient interrupt and leave routed to INT 2 pin
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 3, 0x2D, 0x24, 0x04);
//
//	// Set F_READ bit in CTRL_REG1 so only 8-bit data is read and 4-bit LSB registers are skipped over
//	// Set data rate to be 12.5 Hz
//	I2CSend(I2C0_BASE, ACCEL_ADDR, 2, 0x2A, 0x2B);
//}
//
////**************************************************************************
//// Function to read accelerometer data over I2C
//// Parameters:	NONE
//// Returns:		Accel_Data; float pointer to array holding x, y, and z
////				acceleration data in g's, if device is flat and upright
////				z data will return ~1
////**************************************************************************
//float * AccelRead(void)
//{
//	// Array must be called as static to prevent changes in memory when exiting function
//	int8_t pi8DataRx[3]; // Create array large enough to hold any transfer
//	static float Accel_Data[3];
//	uint8_t i;
//
//	if(gDiagnostics == 1)
//		UARTprintf("Reading Accelerometer \n");
//
//	if(gBoard > V1)
//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
//		{
//			if(gDiagnostics >= 2)
//				UARTprintf("Waiting for BT to finish with I2C in AccelRead \n");
//		}
//
//	uint8_t ui8Data_ready = 0;	// Flag to wait for new data to be available
//	while(ui8Data_ready != 0x07)	// Wait until new data becomes availabe for all axis
//	{
//	    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false); // Set address with R/W bit
//
//	    I2CMasterDataPut(I2C0_BASE, 0x00); // Read Status register to determine if new data is available
//	    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send slave address and register
//	    while(I2CMasterBusy(I2C0_BASE));
//
//	    //specify that we are going to read from slave device
//	    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, true);
//
//	    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//	    while(I2CMasterBusy(I2C0_BASE));
//
//	    ui8Data_ready = I2CMasterDataGet(I2C0_BASE) & 0x07;
//	}
//
//    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false); // Set address with R/W bit
//
//    I2CMasterDataPut(I2C0_BASE, 0x01); // Start at OUT_X_MSB register, continue for 3 registers for x,y,z data
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send slave address and register
//    while(I2CMasterBusy(I2C0_BASE));
//
//    //specify that we are going to read from slave device
//    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, true);
//
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
//    while(I2CMasterBusy(I2C0_BASE));
//    pi8DataRx[0] = I2CMasterDataGet(I2C0_BASE);
//
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//    while(I2CMasterBusy(I2C0_BASE));
//    pi8DataRx[1] = I2CMasterDataGet(I2C0_BASE);
//
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
//    while(I2CMasterBusy(I2C0_BASE));
//    pi8DataRx[2] = I2CMasterDataGet(I2C0_BASE);
//
//    for(i = 0; i < 3; i++)
//    {
//    	Accel_Data[i] = pi8DataRx[i] * .0156;	// LSB of .0156 g at 8-bit +/- 2g range
////    	UARTprintf("%d \t %d \n", (i + 1), (int) (pi8DataRx[i] * 1000));
//    }
//
//    return Accel_Data;
//}
//
////**************************************************************************
//// Interrupt handler for accelerometer, still need to decide what to do in
//// case of interrupts...
//// Parameters:	NONE
//// Returns:		NONE
////**************************************************************************
//void AccelIntHandler(void)
//{
//	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_4);
//	GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_5);
//
////	if(gDiagnostics == 1)
////		UARTprintf("Accelerometer interrupt triggered! \n");
////
////	float * AccelData = AccelRead();
////
////	UARTprintf("X: %d \t Y: %d \t Z: %d \n", (int) (*(AccelData) * 1000), (int) (*(AccelData + 1) * 1000), (int) (*(AccelData + 2) * 1000));
//}

//**************************************************************************
// Function to set up IO Extender used for LEDs created on digital V6.2
// Part MCP23S17
// Created 1/4/2019
// 7/1/2019: Updated for digital board V6.4
// 10/7/2020: Modified to work with I2C version
// Parameters:	NONE
// Returns:		NONE
//**************************************************************************
void InitLED_Ext(void)
{
	if(gBoard >= V6_2)
	{
		if(gBoard >= V7_3)
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Initializing LED IO Extender!\n");
			)

#ifdef MCU_ZXR
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
			update_TivaI2C(1);
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
#else
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
			update_TivaI2C(1);
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
#endif

	    	uint8_t LED_IO_EXT_ADDR = 0x76;			// I2C part address
	    	if(gBoard >= V7_3E)
	    		LED_IO_EXT_ADDR = 0x77;			// I2C part address
	    	uint8_t LED_IO_EXT_CONFIG = 0x06;	// Command byte to write the configuration registers

			// Initialize opposite of what is being set so while loop is entered
			uint16_t Config = 0xFFFF;
			uint8_t Counter = 0;

			while(Config != 0 && Counter <= 10)
			{
				// Write to IO Extender, V7.3 part requires config register to be written with 0 for output pins
				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 3, LED_IO_EXT_CONFIG, 0, 0);

				// V7.3 part requires command byte to read register, then 2 bytes of data
				Config = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, LED_IO_EXT_CONFIG, 2);

//				UARTprintf("GPIO A: %u\n", GPIO_A);
//				UARTprintf("GPIO B: %u\n", GPIO_B);

				if(Counter < 5)
					Counter++;
				else if(Config != 0 && Counter == 5)
				{
					Counter++;
					DEBUG_PRINT(UARTprintf("Config: 0x%x\n", Config);)

					userDelay(1, 1);
				}
				else if(Counter < 10)
					Counter++;
				else if(Config != 0 && Counter == 10)
				{
					Counter++;
					DEBUG_PRINT(UARTprintf("LED IO Extender still not initializing!\n");)

//					AnalogOff();
//
//					userDelay(1000, 1);
//
//					InitAnalog();

					SysCtlDelay(2000);

					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
				}
//				else if(Counter < 15)
//					Counter++;
//				else if(Config != 0 && Counter == 15)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not initialize correctly!\n");
//				}
			}

			update_TivaI2C(0);

			SetLED(0, 1);
		}
		else if(gBoard >= V7_2)
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Initializing LED IO Extender! Nothing actually required!\n");
			)

			uint8_t LED_IO_EXT_ADDR = 0x20;

			// Write to IO Extender, new part only needs 3 bytes, address, first data byte, second data byte (doesn't have an address register)
			I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 2, 0, 0);
		}
		else if(gBoard >= V6_4)
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Initializing LED IO Extender!\n");
			)

			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			uint8_t Ext_Addr_w = 0x40;	// writing address
			uint8_t Ext_Addr_r = 0x41;	// reading address

#ifdef MCU_ZXR
			// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
			GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3

			// Reset LED extender, then read it's power on reset values to make sure communication is working
			GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, 0x00); // LED Ext RST _B
			userDelay(1, 0);	// Wait a little before raising reset line
			GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
#else
			// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3

			// Reset LED extender, then read it's power on reset values to make sure communication is working
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
			userDelay(1, 0);	// Wait a little before raising reset line
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
#endif
//			SysCtlDelay(2000);	// Even though specs say no time is required wait after raising reset signal

			// Initialize to 1 so while loop is entered
			uint32_t Dir_A = 1;
			uint32_t Dir_B = 1;
			uint8_t Counter = 0;

			while((Dir_A != 0xFF || Dir_B != 0xFF) && Counter <= 10)
			{
				// Read address: 0x41
				// GPIO direction register: 0x00
				// Set all IO pins as output: 0x00
				// Set all IO pins as output: 0x00
#ifdef MCU_ZXR
				SPISend(SSI1_BASE, IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
#else
				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
#endif


				SSIDataGet(SSI1_BASE, &Dir_A);
				SSIDataGet(SSI1_BASE, &Dir_A);
				SSIDataGet(SSI1_BASE, &Dir_A);
				SSIDataGet(SSI1_BASE, &Dir_B);

				if(Counter < 5)
					Counter++;
				else if((Dir_A != 0xFF || Dir_B != 0xFF) && Counter == 5)
				{
					Counter++;
					DEBUG_PRINT(UARTprintf("LED IO extender didn't reset properly or didn't read back correctly!\n");)

#ifdef MCU_ZXR
					// If LED extender didn't work after 5 attempts, reset LED extender and try again
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, 0x00); // LED Ext RST _B
					userDelay(1, 0);	// Wait a little before raising reset line
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
#else
					// If LED extender didn't work after 5 attempts, reset LED extender and try again
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
					userDelay(1, 0);	// Wait a little before raising reset line
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
#endif

				}
				else if(Counter < 10)
					Counter++;
				else if((Dir_A != 0 || Dir_B != 0) && Counter == 10)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					DEBUG_PRINT(
					UARTprintf("LED IO Extender did not reset correctly!\n");
					UARTprintf("Updating error and moving on!\n");
					)
				}

			}

			while((Dir_A != 0 || Dir_B != 0) && Counter <= 10)
			{
				// Write address: 0x40
				// GPIO direction register: 0x00
				// Set all IO pins as output: 0x00
				// Set all IO pins as output: 0x00
#ifdef MCU_ZXR
				SPISend(SSI1_BASE, IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, 0, 4, Ext_Addr_w, 0x00, 0x00, 0x00);	// Write to IO direction registers to set all pins as output

				SysCtlDelay(5);	// Wait a little before reading back register values

				// Read address: 0x41
				// GPIO direction register: 0x00
				// Set all IO pins as output: 0x00
				// Set all IO pins as output: 0x00
				SPISend(SSI1_BASE, IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
#else
				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_w, 0x00, 0x00, 0x00);	// Write to IO direction registers to set all pins as output

				SysCtlDelay(5);	// Wait a little before reading back register values

				// Read address: 0x41
				// GPIO direction register: 0x00
				// Set all IO pins as output: 0x00
				// Set all IO pins as output: 0x00
				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
#endif


				SSIDataGet(SSI1_BASE, &Dir_A);
				SSIDataGet(SSI1_BASE, &Dir_A);
				SSIDataGet(SSI1_BASE, &Dir_A);
				SSIDataGet(SSI1_BASE, &Dir_B);

//				UARTprintf("Dir A: %u\n", Dir_A);
//				UARTprintf("Dir B: %u\n", Dir_B);

				if(Counter < 5)
					Counter++;
				else if((Dir_A != 0 || Dir_B != 0) && Counter == 5)
				{
					Counter++;
					DEBUG_PRINT(UARTprintf("LED Extender not initializing correctly, resetting extender!\n");)

#ifdef MCU_ZXR
					// If LED extender didn't work after 5 attempts, reset LED extender and try again
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, 0x00); // LED Ext RST _B
					userDelay(1, 0);	// Wait a little before raising reset line
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
#else
					// If LED extender didn't work after 5 attempts, reset LED extender and try again
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
					userDelay(1, 0);	// Wait a little before raising reset line
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
#endif
				}
				else if(Counter < 10)
					Counter++;
				else if((Dir_A != 0 || Dir_B != 0) && Counter == 10)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					DEBUG_PRINT(
					UARTprintf("LED IO Extender did not initialize correctly!\n");
					UARTprintf("Updating error and moving on!\n");
					)
				}
			}
//
//			if(gLED_State != 0)	// If we are re-initializing extender, this happens after resetting the analog board
//				SetLED(0, 1);	// Setting LED 0 will not change gLED_State but will rewrite current state to LED extender
		}
	}
}

#ifdef MCU_ZXR
//**************************************************************************
// Function to control state of LEDs
// Created 1/3/2019: To control LEDs on digital V6.2 and previous boards
// Parameters:	LED; Which LED to activate; defined at top of components.h
//					Multiple pins can be or'd together for same state
//				state; whether to turn on or off; 0=off, 1=on
// Returns:		NONE
//**************************************************************************
void SetLED(uint16_t LED, uint8_t state)
{
	if(gBoard >= V6_2)	// Added oscillator circuit and split horizontal/vertical LEDs
	{
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Setting LED\n");
		)

	    if(state == 0)
	    {
	    	gLED_State &= ~LED;
	    	// Mechanism to turn ofF blinking even if I only turn off solid switch
	    	if(((LED_BLINK >> 1) & gLED_State) == 0)	// Shift to line up all possible blink bits with power bits and see if both are active for any
	    		gLED_State &= ~LED_BLINK;	// If there are no blink and power bits both set, clear all blink bits
	    }
	    else
	    	gLED_State |= LED;

	    if(gBoard >= RV1_0)
	    {
		    // If any of the LEDs are blinking turn on the oscillator
		    if((LED_BLINK & gLED_State) != 0)
		    {
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B
				SysCtlDelay(SysCtlClockGet()/3000000);
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, IO_LED_OSC_RST_B_PIN); // LED Osc RST _B
		    }
		    else
		    	GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B

		    // LEDs are now on GPIO pins but they are on two IO banks and organized the same way as the extender was so the same masks apply
		    // By writing the to pin 0xFF all pins will be written, since gLED_State holds the state for all pins at once it is unnecessary to specify which pin is written
		    GPIOPinWrite(LED_BUT_R_H_SW_BASE, 0xFF, (gLED_State));
		    GPIOPinWrite(LED_BUT_B_V_SW_BASE, 0xFF, (gLED_State >> 8));
	    }
	    else if(gBoard >= V7_3)
	    {
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
			update_TivaI2C(1);
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C

		    // If any of the LEDs are blinking turn on the oscillator
		    if((LED_BLINK & gLED_State) != 0)
		    {
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B
				SysCtlDelay(SysCtlClockGet()/3000000);
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, IO_LED_OSC_RST_B_PIN); // LED Osc RST _B
		    }
		    else
		    	GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B

	    	uint8_t LED_IO_EXT_ADDR = 0x76;			// I2C part address
	    	if(gBoard >= V7_3E)
	    		LED_IO_EXT_ADDR = 0x77;			// I2C part address
	    	uint8_t LED_IO_EXT_INPUT_PORT = 0x00;	// Command byte to write the output port registers
	    	uint8_t LED_IO_EXT_OUTPUT_PORT = 0x02;	// Command byte to write the output port registers

			// Initialize opposite of what is being set so while loop is entered
			uint32_t GPIO_A = ~(gLED_State & 0xFF);
			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
			uint8_t Counter = 0;

			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 10)
			{
				// Write to IO Extender, new part needs 4 bytes, address, first data byte, second data byte (doesn't have an address register)
				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 3, LED_IO_EXT_OUTPUT_PORT, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));

				// V7.3 part requires command byte to read register, then 2 bytes of data
				uint16_t GPIO_reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, LED_IO_EXT_INPUT_PORT, 2);
				GPIO_A = GPIO_reg & 0xFF;
				GPIO_B = (GPIO_reg >> 8) & 0xFF;

//				UARTprintf("GPIO A: %u\n", GPIO_A);
//				UARTprintf("GPIO B: %u\n", GPIO_B);


				if(Counter < 5)
				{
#ifdef TESTING_MODE
					DEBUG_PRINT(
					if(Counter == 0 && (GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)))
					{
						UARTprintf("First attempt to set LED failed!\n");
						UARTprintf("gLED: %u\n", gLED_State);
						UARTprintf("GPIO A: %u\n", GPIO_A);
						UARTprintf("GPIO B: %u\n", GPIO_B);
					}
					)
#endif
					Counter++;
				}
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
				{
					Counter++;
					DEBUG_PRINT(
					UARTprintf("gLED: %u\n", gLED_State);
					UARTprintf("GPIO A: %u\n", GPIO_A);
					UARTprintf("GPIO B: %u\n", GPIO_B);
					)

					userDelay(1, 1);
				}
				else if(Counter < 10)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
				{
					Counter++;
					DEBUG_PRINT(UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");)

					AnalogOff();

					userDelay(1000, 1);

					InitAnalog();

					InitLED_Ext();

					SysCtlDelay(2000);
				}
				else if(Counter < 15)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					DEBUG_PRINT(UARTprintf("LED IO Extender did not set correctly!\n");)
				}
			}

			update_TivaI2C(0);
	    }
	    else if(gBoard >= V7_2)
	    {
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C
			update_TivaI2C(1);
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish using I2C

		    // If any of the LEDs are blinking turn on the oscillator
		    if((LED_BLINK & gLED_State) != 0)
		    {
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B
				SysCtlDelay(SysCtlClockGet()/3000000);
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, IO_LED_OSC_RST_B_PIN); // LED Osc RST _B
		    }
		    else
		    	GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B


	    	uint8_t LED_IO_EXT_ADDR = 0x20;

			// Initialize opposite of what is being set so while loop is entered
			uint32_t GPIO_A = ~(gLED_State & 0xFF);
			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
			uint8_t Counter = 0;

			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 10)
			{
				// Write to IO Extender, new part only needs 3 bytes, address, first data byte, second data byte (doesn't have an address register)
				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 2, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));

				// Don't need a register, set up I2CReceive function to not write a register if 0xFF is entered
				uint16_t GPIO_reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, 0xFF, 2);
				GPIO_A = GPIO_reg & 0xFF;
				GPIO_B = (GPIO_reg >> 8) & 0xFF;

//				UARTprintf("GPIO A: %u\n", GPIO_A);
//				UARTprintf("GPIO B: %u\n", GPIO_B);

				if(Counter < 5)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
				{
					Counter++;
					DEBUG_PRINT(
					UARTprintf("gLED: %u\n", gLED_State);
					UARTprintf("GPIO A: %u\n", GPIO_A);
					UARTprintf("GPIO B: %u\n", GPIO_B);
					)

					userDelay(1, 1);
				}
				else if(Counter < 10)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
				{
					Counter++;
					DEBUG_PRINT(UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");)

					AnalogOff();

					userDelay(1000, 1);

					InitAnalog();

					InitLED_Ext();

					SysCtlDelay(2000);
				}
				else if(Counter < 15)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					DEBUG_PRINT(UARTprintf("LED IO Extender did not set correctly!\n");)
				}
			}

			update_TivaI2C(0);
	    }
	    else if(gBoard >= V6_4)
	    {
		    // If any of the LEDs are blinking turn on the oscillator
		    if((LED_BLINK & gLED_State) != 0)
		    {
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B
				SysCtlDelay(SysCtlClockGet()/3000000);
				GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, IO_LED_OSC_RST_B_PIN); // LED Osc RST _B
		    }
		    else
		    	GPIOPinWrite(IO_LED_OSC_RST_B_BASE, IO_LED_OSC_RST_B_PIN, 0); // LED Osc RST _B

			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			uint8_t Ext_Addr_w = 0x40;	// writing address
			uint8_t Ext_Addr_r = 0x41;	// reading address

			// Initialize to 1 so while loop is entered
			uint32_t GPIO_A = ~(gLED_State & 0xFF);
			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
			uint8_t Counter = 0;

			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 15)
			{
				// Write address: 0x40
				// GPIO port register: 0x12
				// Set pins according to gLED_State
				// Set pins according to gLED_State
				SPISend(SSI1_BASE, IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, 0, 4, Ext_Addr_w, 0x12, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));	// Write to IO direction registers to set LEDs

				// Read address: 0x41
				// GPIO port register: 0x12
				// Set all IO pins as output: 0x00
				// Set all IO pins as output: 0x00
				SPISend(SSI1_BASE, IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, 0, 4, Ext_Addr_r, 0x12, 0x00, 0x00);	// Read from IO direction registers

//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B

				SSIDataGet(SSI1_BASE, &GPIO_A);
				SSIDataGet(SSI1_BASE, &GPIO_A);
				SSIDataGet(SSI1_BASE, &GPIO_A);
				SSIDataGet(SSI1_BASE, &GPIO_B);

//				UARTprintf("GPIO A: %u\n", GPIO_A);
//				UARTprintf("GPIO B: %u\n", GPIO_B);

				if(Counter < 4)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 4)
				{
					Counter++;

					DEBUG_PRINT(UARTprintf("LED not working! Reading from Output latch register!\n");)

					// Read address: 0x41
					// GPIO Output latch register: 0x14
					// Set all IO pins as output: 0x00
					// Set all IO pins as output: 0x00
					SPISend(SSI1_BASE, IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, 0, 4, Ext_Addr_r, 0x14, 0x00, 0x00);	// Read from IO Output latch registers

					SSIDataGet(SSI1_BASE, &GPIO_A);
					SSIDataGet(SSI1_BASE, &GPIO_A);
					SSIDataGet(SSI1_BASE, &GPIO_A);
					SSIDataGet(SSI1_BASE, &GPIO_B);

//					UARTprintf("gLED: %u\n", gLED_State);
//					UARTprintf("Output A: %u\n", GPIO_A);
//					UARTprintf("Output B: %u\n", GPIO_B);

					if((GPIO_A == (gLED_State & 0xFF) && GPIO_B == ((gLED_State >> 8) & 0xFF)))
					{
						DEBUG_PRINT(UARTprintf("Output latch reading back correctly, communication working, output not moving!\n");)

						gui32Error |= LED_EXTENDER_FAIL;
						update_Error();
					}
				}
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
				{
					Counter++;
					DEBUG_PRINT(
					UARTprintf("gLED: %u\n", gLED_State);
					UARTprintf("GPIO A: %u\n", GPIO_A);
					UARTprintf("GPIO B: %u\n", GPIO_B);
					)

					// If LED extender didn't work after 5 attempts, reset LED extender and try again
					DEBUG_PRINT(UARTprintf("Resetting and retrying LED IO extender!\n");)
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, 0x00); // LED Ext RST _B
					userDelay(10, 0);	// Wait a little before raising reset line
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B

					InitLED_Ext();
				}
				else if(Counter < 10)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
				{
					Counter++;
					DEBUG_PRINT(UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");)

					AnalogOff();

					userDelay(1000, 1);

					InitAnalog();

					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, 0x00); // LED Ext RST _B
					userDelay(10, 0);	// Wait a little before raising reset line
					GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B

					InitLED_Ext();

					// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
					SSIDisable(SSI1_BASE);
					SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
					SSIEnable(SSI1_BASE);

					SysCtlDelay(2000);
				}
				else if(Counter < 15)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					DEBUG_PRINT(UARTprintf("LED IO Extender did not set correctly!\n");)
				}
			}
	    }
	}
	else	// Before V6_2, On or Off, hadn't added the oscillator
	{
		if(state == 2)
			state = 1;
		if((LED & RED_BUTTON) != 0)
			GPIOPinWrite(LED_BUT_RED_BASE, LED_BUT_RED_PIN, (LED_BUT_RED_PIN * state));
		if((LED & GREEN_BUTTON) != 0)
			GPIOPinWrite(LED_BUT_GREEN_BASE, LED_BUT_GREEN_PIN, (LED_BUT_GREEN_PIN * state));
		if((LED & BLUE_BUTTON) != 0)
			GPIOPinWrite(LED_BUT_BLUE_BASE, LED_BUT_BLUE_PIN, (LED_BUT_BLUE_PIN * state));

		if((LED & RED_CHARGE) != 0)
			GPIOPinWrite(LED_CHG_RED_BASE, LED_CHG_RED_PIN, (LED_CHG_RED_PIN * state)); // Red charging LED
		if((LED & GREEN_CHARGE) != 0)
			GPIOPinWrite(LED_CHG_GREEN_BASE, LED_CHG_GREEN_PIN, (LED_CHG_GREEN_PIN * state)); // Green charging LED
		if((LED & YELLOW_CHARGE) != 0)
			GPIOPinWrite(LED_CHG_YELLOW_BASE, LED_CHG_YELLOW_PIN, (LED_CHG_YELLOW_PIN * state)); // Orange charging LED

	    if(state == 0)
	    	gLED_State &= ~LED;
	    else
	    	gLED_State |= LED;
	}
}
#else
//**************************************************************************
// Function to control state of LEDs
// Created 1/3/2019: To control LEDs on digital V6.2 and previous boards
// Parameters:	LED; Which LED to activate; defined at top of components.h
//					Multiple pins can be or'd together for same state
//				state; whether to turn on or off; 0=off, 1=on
// Returns:		NONE
//**************************************************************************
void SetLED(uint16_t LED, uint8_t state)
{
	if(gBoard < V6_2)
	{
		if(state == 2)
			state = 1;
		if((LED & RED_BUTTON) != 0)
			GPIOPinWrite(LED_BUT_RED_BASE, LED_BUT_RED_PIN, (LED_BUT_RED_PIN * state));
		if((LED & GREEN_BUTTON) != 0)
			GPIOPinWrite(LED_BUT_GREEN_BASE, LED_BUT_GREEN_PIN, (LED_BUT_GREEN_PIN * state));
		if((LED & BLUE_BUTTON) != 0)
			GPIOPinWrite(LED_BUT_BLUE_BASE, LED_BUT_BLUE_PIN, (LED_BUT_BLUE_PIN * state));

		if((LED & RED_CHARGE) != 0)
			GPIOPinWrite(LED_CHG_RED_BASE, LED_CHG_RED_PIN, (LED_CHG_RED_PIN * state)); // Red charging LED
		if((LED & GREEN_CHARGE) != 0)
			GPIOPinWrite(LED_CHG_GREEN_BASE, LED_CHG_GREEN_PIN, (LED_CHG_GREEN_PIN * state)); // Green charging LED
		if((LED & YELLOW_CHARGE) != 0)
			GPIOPinWrite(LED_CHG_YELLOW_BASE, LED_CHG_YELLOW_PIN, (LED_CHG_YELLOW_PIN * state)); // Orange charging LED

	    if(state == 0)
	    	gLED_State &= ~LED;
	    else
	    	gLED_State |= LED;
	}
	else if(gBoard >= V6_2)
	{
		if(gDiagnostics >= 1)
			UARTprintf("Setting LED\n");

	    if(state == 0)
	    {
	    	gLED_State &= ~LED;
	    	// Mechanism to turn ofF blinking even if I only turn off solid switch
	    	if(((LED_BLINK >> 1) & gLED_State) == 0)	// Shift to lines up all possible blink bits with power bits and see if both are active for any
	    		gLED_State &= ~LED_BLINK;	// If there are no blink and power bits both set, clear all blink bits
	    }
	    else
	    	gLED_State |= LED;

	    if(gBoard >= V7_3)
	    {
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
			update_TivaI2C(1);
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C

		    // If any of the LEDs are blinking turn on the oscillator
		    if((LED_BLINK & gLED_State) != 0)
		    {
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
				SysCtlDelay(SysCtlClockGet()/3000000);
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
		    }
		    else
		    	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B

	    	uint8_t LED_IO_EXT_ADDR = 0x76;			// I2C part address
	    	if(gBoard >= V7_3E)
	    		LED_IO_EXT_ADDR = 0x77;			// I2C part address
	    	uint8_t LED_IO_EXT_INPUT_PORT = 0x00;	// Command byte to write the output port registers
	    	uint8_t LED_IO_EXT_OUTPUT_PORT = 0x02;	// Command byte to write the output port registers

			// Initialize opposite of what is being set so while loop is entered
			uint32_t GPIO_A = ~(gLED_State & 0xFF);
			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
			uint8_t Counter = 0;

			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 10)
			{
				// Write to IO Extender, new part needs 4 bytes, address, first data byte, second data byte (doesn't have an address register)
				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 3, LED_IO_EXT_OUTPUT_PORT, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));

				// V7.3 part requires command byte to read register, then 2 bytes of data
				uint16_t GPIO_reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, LED_IO_EXT_INPUT_PORT, 2);
				GPIO_A = GPIO_reg & 0xFF;
				GPIO_B = (GPIO_reg >> 8) & 0xFF;

//				UARTprintf("GPIO A: %u\n", GPIO_A);
//				UARTprintf("GPIO B: %u\n", GPIO_B);


				if(Counter < 5)
				{
#ifdef TESTING_MODE
					if(Counter == 0 && (GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)))
					{
						UARTprintf("First attempt to set LED failed!\n");
						UARTprintf("gLED: %u\n", gLED_State);
						UARTprintf("GPIO A: %u\n", GPIO_A);
						UARTprintf("GPIO B: %u\n", GPIO_B);
					}
#endif
					Counter++;
				}
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
				{
					Counter++;
					UARTprintf("gLED: %u\n", gLED_State);
					UARTprintf("GPIO A: %u\n", GPIO_A);
					UARTprintf("GPIO B: %u\n", GPIO_B);

					userDelay(1, 1);
				}
				else if(Counter < 10)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
				{
					Counter++;
					UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");

					AnalogOff();

					userDelay(1000, 1);

					InitAnalog();

					InitLED_Ext();

					SysCtlDelay(2000);
				}
				else if(Counter < 15)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					UARTprintf("LED IO Extender did not set correctly!\n");
				}
			}

			update_TivaI2C(0);
	    }
	    else if(gBoard >= V7_2)
	    {
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
			update_TivaI2C(1);
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C

		    // If any of the LEDs are blinking turn on the oscillator
		    if((LED_BLINK & gLED_State) != 0)
		    {
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
				SysCtlDelay(SysCtlClockGet()/3000000);
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
		    }
		    else
		    	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B


	    	uint8_t LED_IO_EXT_ADDR = 0x20;

			// Initialize opposite of what is being set so while loop is entered
			uint32_t GPIO_A = ~(gLED_State & 0xFF);
			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
			uint8_t Counter = 0;

			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 10)
			{
				// Write to IO Extender, new part only needs 3 bytes, address, first data byte, second data byte (doesn't have an address register)
				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 2, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));

				// Don't need a register, set up I2CReceive function to not write a register if 0xFF is entered
				uint16_t GPIO_reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, 0xFF, 2);
				GPIO_A = GPIO_reg & 0xFF;
				GPIO_B = (GPIO_reg >> 8) & 0xFF;

//				UARTprintf("GPIO A: %u\n", GPIO_A);
//				UARTprintf("GPIO B: %u\n", GPIO_B);

				if(Counter < 5)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
				{
					Counter++;
					UARTprintf("gLED: %u\n", gLED_State);
					UARTprintf("GPIO A: %u\n", GPIO_A);
					UARTprintf("GPIO B: %u\n", GPIO_B);

					userDelay(1, 1);
				}
				else if(Counter < 10)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
				{
					Counter++;
					UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");

					AnalogOff();

					userDelay(1000, 1);

					InitAnalog();

					InitLED_Ext();

					SysCtlDelay(2000);
				}
				else if(Counter < 15)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					UARTprintf("LED IO Extender did not set correctly!\n");
				}
			}

			update_TivaI2C(0);
	    }
	    else if(gBoard >= V6_4)
	    {
		    // If any of the LEDs are blinking turn on the oscillator
		    if((LED_BLINK & gLED_State) != 0)
		    {
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
				SysCtlDelay(SysCtlClockGet()/3000000);
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
		    }
		    else
		    	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B

			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			uint8_t Ext_Addr_w = 0x40;	// writing address
			uint8_t Ext_Addr_r = 0x41;	// reading address

			// Initialize to 1 so while loop is entered
			uint32_t GPIO_A = ~(gLED_State & 0xFF);
			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
			uint8_t Counter = 0;

			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 15)
			{
				// Write address: 0x40
				// GPIO port register: 0x12
				// Set pins according to gLED_State
				// Set pins according to gLED_State
				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_w, 0x12, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));	// Write to IO direction registers to set LEDs

				// Read address: 0x41
				// GPIO port register: 0x12
				// Set all IO pins as output: 0x00
				// Set all IO pins as output: 0x00
				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x12, 0x00, 0x00);	// Read from IO direction registers

//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B

				SSIDataGet(SSI1_BASE, &GPIO_A);
				SSIDataGet(SSI1_BASE, &GPIO_A);
				SSIDataGet(SSI1_BASE, &GPIO_A);
				SSIDataGet(SSI1_BASE, &GPIO_B);

//				UARTprintf("GPIO A: %u\n", GPIO_A);
//				UARTprintf("GPIO B: %u\n", GPIO_B);

				if(Counter < 4)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 4)
				{
					Counter++;

					UARTprintf("LED not working! Reading from Output latch register!\n");

					// Read address: 0x41
					// GPIO Output latch register: 0x14
					// Set all IO pins as output: 0x00
					// Set all IO pins as output: 0x00
					SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x14, 0x00, 0x00);	// Read from IO Output latch registers

					SSIDataGet(SSI1_BASE, &GPIO_A);
					SSIDataGet(SSI1_BASE, &GPIO_A);
					SSIDataGet(SSI1_BASE, &GPIO_A);
					SSIDataGet(SSI1_BASE, &GPIO_B);

//					UARTprintf("gLED: %u\n", gLED_State);
//					UARTprintf("Output A: %u\n", GPIO_A);
//					UARTprintf("Output B: %u\n", GPIO_B);

					if((GPIO_A == (gLED_State & 0xFF) && GPIO_B == ((gLED_State >> 8) & 0xFF)))
					{
						UARTprintf("Output latch reading back correctly, communication working, output not moving!\n");

						gui32Error |= LED_EXTENDER_FAIL;
						update_Error();
					}
				}
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
				{
					Counter++;
					UARTprintf("gLED: %u\n", gLED_State);
					UARTprintf("GPIO A: %u\n", GPIO_A);
					UARTprintf("GPIO B: %u\n", GPIO_B);

					// If LED extender didn't work after 5 attempts, reset LED extender and try again
					UARTprintf("Resetting and retrying LED IO extender!\n");
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
					userDelay(10, 0);	// Wait a little before raising reset line
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B

					InitLED_Ext();
				}
				else if(Counter < 10)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
				{
					Counter++;
					UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");

					AnalogOff();

					userDelay(1000, 1);

					InitAnalog();

					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
					userDelay(10, 0);	// Wait a little before raising reset line
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B

					InitLED_Ext();

					// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
					SSIDisable(SSI1_BASE);
					SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
					SSIEnable(SSI1_BASE);

					SysCtlDelay(2000);
				}
				else if(Counter < 15)
					Counter++;
				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
				{
					Counter++;
					gui32Error |= LED_EXTENDER_FAIL;
					update_Error();
					UARTprintf("LED IO Extender did not set correctly!\n");
				}
			}
	    }
	}
}
#endif

//#ifdef I2C_LED_IOEXT
////**************************************************************************
//// Function to set up IO Extender used for LEDs created on digital V6.2
//// Part MCP23S17
//// Created 1/4/2019
//// 7/1/2019: Updated for digital board V6.4
//// 10/7/2020: Modified to work with I2C version
//// Parameters:	NONE
//// Returns:		NONE
////**************************************************************************
//void InitLED_Ext(void)
//{
//	if(gBoard >= V6_2)
//	{
//		if(gBoard >= V7_2)
//		{
//			if(gDiagnostics >= 1)
//				UARTprintf("Initializing LED IO Extender! Nothing actually required!\n");
//
//			uint8_t LED_IO_EXT_ADDR = 0x20;
//
//			// Write to IO Extender, new part only needs 3 bytes, address, first data byte, second data byte (doesn't have an address register)
//			I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 2, 0, 0);
//		}
//		else if(gBoard >= V6_4)
//		{
//			if(gDiagnostics >= 1)
//				UARTprintf("Initializing LED IO Extender!\n");
//
////			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
////			SSIDisable(SSI1_BASE);
////			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
////			SSIEnable(SSI1_BASE);
//
//			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
//			update_TivaI2C(1);
//			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
//
////			uint8_t Ext_Addr_w = 0x40;	// writing address
////			uint8_t Ext_Addr_r = 0x41;	// reading address
//
//			// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
//
//			// Reset LED extender, then read it's power on reset values to make sure communication is working
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//			userDelay(1, 0);	// Wait a little before raising reset line
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//
////			SysCtlDelay(2000);	// Even though specs say no time is required wait after raising reset signal
//
//			// Initialize to 1 so while loop is entered
//			uint32_t Dir_A = 1;
//			uint32_t Dir_B = 1;
//			uint8_t Counter = 0;
//
//			uint8_t LED_IO_EXT_ADDR = 0x20;
//
//			while((Dir_A != 0xFF || Dir_B != 0xFF) && Counter <= 10)
//			{
//				// Read address: 0x41
//				// GPIO direction register: 0x00
//				// Set all IO pins as output: 0x00
//				// Set all IO pins as output: 0x00
////				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
//
////				SSIDataGet(SSI1_BASE, &Dir_A);
////				SSIDataGet(SSI1_BASE, &Dir_A);
////				SSIDataGet(SSI1_BASE, &Dir_A);
////				SSIDataGet(SSI1_BASE, &Dir_B);
//
//				// 0x00 is GPIO direction register
//				uint16_t Dir_Reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, 0x00, 2);
//				Dir_A = Dir_Reg & 0xFF;
//				Dir_B = (Dir_Reg >> 8) & 0xFF;
//
//				if(Counter < 5)
//					Counter++;
//				else if((Dir_A != 0xFF || Dir_B != 0xFF) && Counter == 5)
//				{
//					Counter++;
//					UARTprintf("LED IO extender didn't reset properly or didn't read back correctly!\n");
//
//					// If LED extender didn't work after 5 attempts, reset LED extender and try again
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(1, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//				}
//				else if(Counter < 10)
//					Counter++;
//				else if((Dir_A != 0 || Dir_B != 0) && Counter == 10)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not reset correctly!\n");
//					UARTprintf("Updating error and moving on!\n");
//				}
//
//			}
//
//			while((Dir_A != 0 || Dir_B != 0) && Counter <= 10)
//			{
//				// Write address: 0x40
//				// GPIO direction register: 0x00
//				// Set all IO pins as output: 0x00
//				// Set all IO pins as output: 0x00
////				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_w, 0x00, 0x00, 0x00);	// Write to IO direction registers to set all pins as output
//				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 3, 0x00, 0x00, 0x00);
//
//				SysCtlDelay(5);	// Wait a little before reading back register values
//
//				// Read address: 0x41
//				// GPIO direction register: 0x00
//				// Set all IO pins as output: 0x00
//				// Set all IO pins as output: 0x00
////				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
////
////				SSIDataGet(SSI1_BASE, &Dir_A);
////				SSIDataGet(SSI1_BASE, &Dir_A);
////				SSIDataGet(SSI1_BASE, &Dir_A);
////				SSIDataGet(SSI1_BASE, &Dir_B);
//
//				uint16_t Dir_Reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, 0x00, 2);
//				Dir_A = Dir_Reg & 0xFF;
//				Dir_B = (Dir_Reg >> 8) & 0xFF;
//
////				UARTprintf("Dir A: %u\n", Dir_A);
////				UARTprintf("Dir B: %u\n", Dir_B);
//
//				if(Counter < 5)
//					Counter++;
//				else if((Dir_A != 0 || Dir_B != 0) && Counter == 5)
//				{
//					Counter++;
//					UARTprintf("LED Extender not initializing correctly, resetting extender!\n");
//
//					// If LED extender didn't work after 5 attempts, reset LED extender and try again
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(1, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//				}
//				else if(Counter < 10)
//					Counter++;
//				else if((Dir_A != 0 || Dir_B != 0) && Counter == 10)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not initialize correctly!\n");
//					UARTprintf("Updating error and moving on!\n");
//				}
//			}
//
//			update_TivaI2C(1);
////
////			if(gLED_State != 0)	// If we are re-initializing extender, this happens after resetting the analog board
////				SetLED(0, 1);	// Setting LED 0 will not change gLED_State but will rewrite current state to LED extender
//		}
//	}
//}
//
////**************************************************************************
//// Function to control state of LEDs
//// Created 1/3/2019: To control LEDs on digital V6.2 and previous boards
//// Parameters:	LED; Which LED to activate; defined at top of components.h
////					Multiple pins can be or'd together for same state
////				state; whether to turn on or off; 0=off, 1=on
//// Returns:		NONE
////**************************************************************************
//void SetLED(uint16_t LED, uint8_t state)
//{
//	if(gBoard < V6_2)
//	{
//		if(state == 2)
//			state = 1;
//		if((LED & RED_BUTTON) != 0)
//			GPIOPinWrite(LED_BUT_RED_BASE, LED_BUT_RED_PIN, (LED_BUT_RED_PIN * state));
//		if((LED & GREEN_BUTTON) != 0)
//			GPIOPinWrite(LED_BUT_GREEN_BASE, LED_BUT_GREEN_PIN, (LED_BUT_GREEN_PIN * state));
//		if((LED & BLUE_BUTTON) != 0)
//			GPIOPinWrite(LED_BUT_BLUE_BASE, LED_BUT_BLUE_PIN, (LED_BUT_BLUE_PIN * state));
//
//		if((LED & RED_CHARGE) != 0)
//			GPIOPinWrite(LED_CHG_RED_BASE, LED_CHG_RED_PIN, (LED_CHG_RED_PIN * state)); // Red charging LED
//		if((LED & GREEN_CHARGE) != 0)
//			GPIOPinWrite(LED_CHG_GREEN_BASE, LED_CHG_GREEN_PIN, (LED_CHG_GREEN_PIN * state)); // Green charging LED
//		if((LED & YELLOW_CHARGE) != 0)
//			GPIOPinWrite(LED_CHG_YELLOW_BASE, LED_CHG_YELLOW_PIN, (LED_CHG_YELLOW_PIN * state)); // Orange charging LED
//
//	    if(state == 0)
//	    	gLED_State &= ~LED;
//	    else
//	    	gLED_State |= LED;
//
//	}
//	else if(gBoard >= V6_2)
//	{
//		if(gDiagnostics >= 1)
//			UARTprintf("Setting LED\n");
//
//	    if(state == 0)
//	    {
//	    	gLED_State &= ~LED;
//	    	// Mechanism to turn ofF blinking even if I only turn off solid switch
//	    	if(((LED_BLINK >> 1) & gLED_State) == 0)	// Shift to lines up all possible blink bits with power bits and see if both are active for any
//	    		gLED_State &= ~LED_BLINK;	// If there are no blink and power bits both set, clear all blink bits
//	    }
//	    else
//	    	gLED_State |= LED;
//
//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
//		update_TivaI2C(1);
//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish using I2C
//
//	    // If any of the LEDs are blinking turn on the oscillator
//	    if((LED_BLINK & gLED_State) != 0)
//	    {
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
//			SysCtlDelay(SysCtlClockGet()/3000000);
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
//	    }
//	    else
//	    	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
//
//	    if(gBoard >= V7_2)
//	    {
//	    	uint8_t LED_IO_EXT_ADDR = 0x20;
//
//			// Initialize opposite of what is being set so while loop is entered
//			uint32_t GPIO_A = ~(gLED_State & 0xFF);
//			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
//			uint8_t Counter = 0;
//
//			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 10)
//			{
//				// Write to IO Extender, new part only needs 3 bytes, address, first data byte, second data byte (doesn't have an address register)
//				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 2, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));
//
//				// Don't need a register, set up I2CReceive function to not write a register if 0xFF is entered
//				uint16_t GPIO_reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, 0xFF, 2);
//				GPIO_A = GPIO_reg & 0xFF;
//				GPIO_B = (GPIO_reg >> 8) & 0xFF;
//
////				UARTprintf("GPIO A: %u\n", GPIO_A);
////				UARTprintf("GPIO B: %u\n", GPIO_B);
//
//				if(Counter < 5)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
//				{
//					Counter++;
//					UARTprintf("gLED: %u\n", gLED_State);
//					UARTprintf("GPIO A: %u\n", GPIO_A);
//					UARTprintf("GPIO B: %u\n", GPIO_B);
//
//					userDelay(1, 1);
//				}
//				else if(Counter < 10)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
//				{
//					Counter++;
//					UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");
//
//					AnalogOff();
//
//					userDelay(1000, 1);
//
//					InitAnalog();
//
//					InitLED_Ext();
//
//					SysCtlDelay(2000);
//				}
//				else if(Counter < 15)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not set correctly!\n");
//				}
//			}
//	    }
//	    else if(gBoard >= V6_4)
//	    {
////			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
////			SSIDisable(SSI1_BASE);
////			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
////			SSIEnable(SSI1_BASE);
//
////			uint8_t Ext_Addr_w = 0x40;	// writing address
////			uint8_t Ext_Addr_r = 0x41;	// reading address
//	    	uint8_t LED_IO_EXT_ADDR = 0x20;
//
//			// Initialize to 1 so while loop is entered
//			uint32_t GPIO_A = ~(gLED_State & 0xFF);
//			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
//			uint8_t Counter = 0;
//
//			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 15)
//			{
//				// Write address: 0x40
//				// GPIO port register: 0x12
//				// Set pins according to gLED_State
//				// Set pins according to gLED_State
////				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_w, 0x12, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));	// Write to IO direction registers to set LEDs
//				I2CSend(I2C0_BASE, LED_IO_EXT_ADDR, 3, 0x12, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));
//
////				// Read address: 0x41
////				// GPIO port register: 0x12
////				// Set all IO pins as output: 0x00
////				// Set all IO pins as output: 0x00
////				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x12, 0x00, 0x00);	// Read from IO direction registers
////
//////				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B
////
////				SSIDataGet(SSI1_BASE, &GPIO_A);
////				SSIDataGet(SSI1_BASE, &GPIO_A);
////				SSIDataGet(SSI1_BASE, &GPIO_A);
////				SSIDataGet(SSI1_BASE, &GPIO_B);
//				uint16_t GPIO_reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, 0x12, 2);
//				GPIO_A = GPIO_reg & 0xFF;
//				GPIO_B = (GPIO_reg >> 8) & 0xFF;
//
//
////				UARTprintf("GPIO A: %u\n", GPIO_A);
////				UARTprintf("GPIO B: %u\n", GPIO_B);
//
//				if(Counter < 4)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 4)
//				{
//					Counter++;
//
//					UARTprintf("LED not working! Reading from Output latch register!\n");
//
////					// Read address: 0x41
////					// GPIO Output latch register: 0x14
////					// Set all IO pins as output: 0x00
////					// Set all IO pins as output: 0x00
////					SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x14, 0x00, 0x00);	// Read from IO Output latch registers
////
////					SSIDataGet(SSI1_BASE, &GPIO_A);
////					SSIDataGet(SSI1_BASE, &GPIO_A);
////					SSIDataGet(SSI1_BASE, &GPIO_A);
////					SSIDataGet(SSI1_BASE, &GPIO_B);
//
//					uint16_t OUT_reg = I2CReceive(I2C0_BASE, LED_IO_EXT_ADDR, 0x14, 2);
//					GPIO_A = OUT_reg & 0xFF;
//					GPIO_B = (OUT_reg >> 8) & 0xFF;
//
////					UARTprintf("gLED: %u\n", gLED_State);
////					UARTprintf("Output A: %u\n", GPIO_A);
////					UARTprintf("Output B: %u\n", GPIO_B);
//
//					if((GPIO_A == (gLED_State & 0xFF) && GPIO_B == ((gLED_State >> 8) & 0xFF)))
//					{
//						UARTprintf("Output latch reading back correctly, communication working, output not moving!\n");
//
//						gui32Error |= LED_EXTENDER_FAIL;
//						update_Error();
//					}
//				}
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
//				{
//					Counter++;
//					UARTprintf("gLED: %u\n", gLED_State);
//					UARTprintf("GPIO A: %u\n", GPIO_A);
//					UARTprintf("GPIO B: %u\n", GPIO_B);
//
//					// If LED extender didn't work after 5 attempts, reset LED extender and try again
//					UARTprintf("Resetting and retrying LED IO extender!\n");
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(10, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//
//					InitLED_Ext();
//				}
//				else if(Counter < 10)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
//				{
//					Counter++;
//					UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");
//
//					AnalogOff();
//
//					userDelay(1000, 1);
//
//					InitAnalog();
//
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(10, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//
//					InitLED_Ext();
//
////					// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
////					SSIDisable(SSI1_BASE);
////					SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
////					SSIEnable(SSI1_BASE);
//
//					SysCtlDelay(2000);
//				}
//				else if(Counter < 15)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not set correctly!\n");
//				}
//			}
//	    }
//
//	    update_TivaI2C(0);
//
//	}
//}
//
//#else
//
////**************************************************************************
//// Function to set up IO Extender used for LEDs created on digital V6.2
//// Part MCP23S17
//// Created 1/4/2019
//// 7/1/2019: Updated for digital board V6.4
//// Parameters:	NONE
//// Returns:		NONE
////**************************************************************************
//void InitLED_Ext(void)
//{
//	if(gBoard >= V6_2)
//	{
//		if(gBoard >= V6_4)
//		{
//			if(gDiagnostics >= 1)
//				UARTprintf("Initializing LED IO Extender!\n");
//
//			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//			SSIDisable(SSI1_BASE);
//			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//			SSIEnable(SSI1_BASE);
//
//			uint8_t Ext_Addr_w = 0x40;	// writing address
//			uint8_t Ext_Addr_r = 0x41;	// reading address
//
//			// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
//
//			// Reset LED extender, then read it's power on reset values to make sure communication is working
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//			userDelay(1, 0);	// Wait a little before raising reset line
//			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//
////			SysCtlDelay(2000);	// Even though specs say no time is required wait after raising reset signal
//
//			// Initialize to 1 so while loop is entered
//			uint32_t Dir_A = 1;
//			uint32_t Dir_B = 1;
//			uint8_t Counter = 0;
//
//			while((Dir_A != 0xFF || Dir_B != 0xFF) && Counter <= 10)
//			{
//				// Read address: 0x41
//				// GPIO direction register: 0x00
//				// Set all IO pins as output: 0x00
//				// Set all IO pins as output: 0x00
//				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
//
//				SSIDataGet(SSI1_BASE, &Dir_A);
//				SSIDataGet(SSI1_BASE, &Dir_A);
//				SSIDataGet(SSI1_BASE, &Dir_A);
//				SSIDataGet(SSI1_BASE, &Dir_B);
//
//				if(Counter < 5)
//					Counter++;
//				else if((Dir_A != 0xFF || Dir_B != 0xFF) && Counter == 5)
//				{
//					Counter++;
//					UARTprintf("LED IO extender didn't reset properly or didn't read back correctly!\n");
//
//					// If LED extender didn't work after 5 attempts, reset LED extender and try again
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(1, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//				}
//				else if(Counter < 10)
//					Counter++;
//				else if((Dir_A != 0 || Dir_B != 0) && Counter == 10)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not reset correctly!\n");
//					UARTprintf("Updating error and moving on!\n");
//				}
//
//			}
//
//			while((Dir_A != 0 || Dir_B != 0) && Counter <= 10)
//			{
//				// Write address: 0x40
//				// GPIO direction register: 0x00
//				// Set all IO pins as output: 0x00
//				// Set all IO pins as output: 0x00
//				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_w, 0x00, 0x00, 0x00);	// Write to IO direction registers to set all pins as output
//
//				SysCtlDelay(5);	// Wait a little before reading back register values
//
//				// Read address: 0x41
//				// GPIO direction register: 0x00
//				// Set all IO pins as output: 0x00
//				// Set all IO pins as output: 0x00
//				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x00, 0x00, 0x00);	// Read from IO direction registers
//
//				SSIDataGet(SSI1_BASE, &Dir_A);
//				SSIDataGet(SSI1_BASE, &Dir_A);
//				SSIDataGet(SSI1_BASE, &Dir_A);
//				SSIDataGet(SSI1_BASE, &Dir_B);
//
////				UARTprintf("Dir A: %u\n", Dir_A);
////				UARTprintf("Dir B: %u\n", Dir_B);
//
//				if(Counter < 5)
//					Counter++;
//				else if((Dir_A != 0 || Dir_B != 0) && Counter == 5)
//				{
//					Counter++;
//					UARTprintf("LED Extender not initializing correctly, resetting extender!\n");
//
//					// If LED extender didn't work after 5 attempts, reset LED extender and try again
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(1, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//				}
//				else if(Counter < 10)
//					Counter++;
//				else if((Dir_A != 0 || Dir_B != 0) && Counter == 10)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not initialize correctly!\n");
//					UARTprintf("Updating error and moving on!\n");
//				}
//			}
////
////			if(gLED_State != 0)	// If we are re-initializing extender, this happens after resetting the analog board
////				SetLED(0, 1);	// Setting LED 0 will not change gLED_State but will rewrite current state to LED extender
//		}
//		else
//		{
//			if(gDiagnostics >= 1)
//				UARTprintf("Initializing LED IO Extender through BT\n");
//
//			int counter = 0;
//			uint8_t Command_received = 0;
//
//			while(Command_received == 0)
//			{
//				g_ulSSI0RXTO = 0;
//
//				// Empty SSI FIFO
//				while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
//				{
//				}
//
//				SSIDataPut(SSI0_BASE, INIT_LED_EXT);
//				SSIDataPut(SSI0_BASE, 0x00);
//				SSIDataPut(SSI0_BASE, 0x00);
//
//				uint8_t attempts = 0;
//				while(Command_received == 0 && attempts < 3)
//				{
//					counter = 0;
//					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
//					SysCtlDelay(2000);
//					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
//
//					while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
//					{
//						SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
//						counter++;
//					}
//					if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
//					{
//						Command_received = 1;
//					}
//
//					attempts++;
//				}
//
//				g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag
//
//				if(Command_received == 1)
//				{
//					//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On
//
//					// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//
//					GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN );	// Set SSI transmit pin as input so we can listen to BT LED without driving
//					GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00); // LED Ext CS _B
//
//					// Disable SSI interrupt because this is a unique SSI transaction
//					IntDisable(INT_SSI0);
//
//					// Toggle pin again to let BT know CS pin is selected
//					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
//					SysCtlDelay(2000);
//					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
//
//					uint32_t junk;
//
//					uint8_t i;
//					// Wait for 4 bytes of data to know all has been transmitted
//					for(i = 0; i < 4; i++)
//					{
//						SSIDataGet(SSI0_BASE, &junk);
//					}
//
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B
//
//					GPIOPinConfigure(GPIO_PA5_SSI0TX);
//					GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
//
//					SSIIntClear(SSI0_BASE, SSI_RXTO);
//					IntEnable(INT_SSI0);
//
//					//	    gLED_State = 0x0000;	// Turn all off by default
//
//					//	    SysCtlDelay(SysCtlClockGet()/3000 * 7);	// Wait 7 ms after powering up analog board, this is ADCs startup time which is slowest component
//
//					if(gLED_State != 0)	// If we are re-initializing extender, this happens after resetting the analog board
//						SetLED(0, 1);	// Setting LED 0 will not change gLED_State but will rewrite current state to LED extender
//				}
//				else
//				{
//					UARTprintf("Init LED not working! ");
//					Reset_BT();
//				}
//			}
//		}
//
//
//	}
//}
//
////**************************************************************************
//// Function to control state of LEDs
//// Created 1/3/2019: To control LEDs on digital V6.2 and previous boards
//// Parameters:	LED; Which LED to activate; defined at top of components.h
////					Multiple pins can be or'd together for same state
////				state; whether to turn on or off; 0=off, 1=on
//// Returns:		NONE
////**************************************************************************
//void SetLED(uint16_t LED, uint8_t state)
//{
//	if(gBoard < V6_2)
//	{
//		if(state == 2)
//			state = 1;
//		if((LED & RED_BUTTON) != 0)
//			GPIOPinWrite(LED_BUT_RED_BASE, LED_BUT_RED_PIN, (LED_BUT_RED_PIN * state));
//		if((LED & GREEN_BUTTON) != 0)
//			GPIOPinWrite(LED_BUT_GREEN_BASE, LED_BUT_GREEN_PIN, (LED_BUT_GREEN_PIN * state));
//		if((LED & BLUE_BUTTON) != 0)
//			GPIOPinWrite(LED_BUT_BLUE_BASE, LED_BUT_BLUE_PIN, (LED_BUT_BLUE_PIN * state));
//
//		if((LED & RED_CHARGE) != 0)
//			GPIOPinWrite(LED_CHG_RED_BASE, LED_CHG_RED_PIN, (LED_CHG_RED_PIN * state)); // Red charging LED
//		if((LED & GREEN_CHARGE) != 0)
//			GPIOPinWrite(LED_CHG_GREEN_BASE, LED_CHG_GREEN_PIN, (LED_CHG_GREEN_PIN * state)); // Green charging LED
//		if((LED & YELLOW_CHARGE) != 0)
//			GPIOPinWrite(LED_CHG_YELLOW_BASE, LED_CHG_YELLOW_PIN, (LED_CHG_YELLOW_PIN * state)); // Orange charging LED
//
//	    if(state == 0)
//	    	gLED_State &= ~LED;
//	    else
//	    	gLED_State |= LED;
//
//	}
//	else if(gBoard >= V6_2)
//	{
//		if(gDiagnostics >= 1)
//			UARTprintf("Setting LED\n");
//
//	    if(state == 0)
//	    {
//	    	gLED_State &= ~LED;
//	    	// Mechanism to turn ofF blinking even if I only turn off solid switch
//	    	if(((LED_BLINK >> 1) & gLED_State) == 0)	// Shift to lines up all possible blink bits with power bits and see if both are active for any
//	    		gLED_State &= ~LED_BLINK;	// If there are no blink and power bits both set, clear all blink bits
//	    }
//	    else
//	    	gLED_State |= LED;
//
//	    // If any of the LEDs are blinking turn on the oscillator
//	    if((LED_BLINK & gLED_State) != 0)
//	    {
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
//			SysCtlDelay(SysCtlClockGet()/3000000);
//			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5); // LED Osc RST _B
//	    }
//
//	    if(gBoard >= V6_4)
//	    {
//			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//			SSIDisable(SSI1_BASE);
//			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//			SSIEnable(SSI1_BASE);
//
//			uint8_t Ext_Addr_w = 0x40;	// writing address
//			uint8_t Ext_Addr_r = 0x41;	// reading address
//
//			// Initialize to 1 so while loop is entered
//			uint32_t GPIO_A = ~(gLED_State & 0xFF);
//			uint32_t GPIO_B = ~((gLED_State >> 8) & 0xFF);
//			uint8_t Counter = 0;
//
//			while((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter <= 15)
//			{
//				// Write address: 0x40
//				// GPIO port register: 0x12
//				// Set pins according to gLED_State
//				// Set pins according to gLED_State
//				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_w, 0x12, (gLED_State & 0xFF), ((gLED_State >> 8) & 0xFF));	// Write to IO direction registers to set LEDs
//
//				// Read address: 0x41
//				// GPIO port register: 0x12
//				// Set all IO pins as output: 0x00
//				// Set all IO pins as output: 0x00
//				SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x12, 0x00, 0x00);	// Read from IO direction registers
//
////				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B
//
//				SSIDataGet(SSI1_BASE, &GPIO_A);
//				SSIDataGet(SSI1_BASE, &GPIO_A);
//				SSIDataGet(SSI1_BASE, &GPIO_A);
//				SSIDataGet(SSI1_BASE, &GPIO_B);
//
////				UARTprintf("GPIO A: %u\n", GPIO_A);
////				UARTprintf("GPIO B: %u\n", GPIO_B);
//
//				if(Counter < 4)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 4)
//				{
//					Counter++;
//
//					UARTprintf("LED not working! Reading from Output latch register!\n");
//
//					// Read address: 0x41
//					// GPIO Output latch register: 0x14
//					// Set all IO pins as output: 0x00
//					// Set all IO pins as output: 0x00
//					SPISend(SSI1_BASE, GPIO_PORTD_BASE, GPIO_PIN_1, 0, 4, Ext_Addr_r, 0x14, 0x00, 0x00);	// Read from IO Output latch registers
//
//					SSIDataGet(SSI1_BASE, &GPIO_A);
//					SSIDataGet(SSI1_BASE, &GPIO_A);
//					SSIDataGet(SSI1_BASE, &GPIO_A);
//					SSIDataGet(SSI1_BASE, &GPIO_B);
//
////					UARTprintf("gLED: %u\n", gLED_State);
////					UARTprintf("Output A: %u\n", GPIO_A);
////					UARTprintf("Output B: %u\n", GPIO_B);
//
//					if((GPIO_A == (gLED_State & 0xFF) && GPIO_B == ((gLED_State >> 8) & 0xFF)))
//					{
//						UARTprintf("Output latch reading back correctly, communication working, output not moving!\n");
//
//						gui32Error |= LED_EXTENDER_FAIL;
//						update_Error();
//					}
//				}
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 5)
//				{
//					Counter++;
//					UARTprintf("gLED: %u\n", gLED_State);
//					UARTprintf("GPIO A: %u\n", GPIO_A);
//					UARTprintf("GPIO B: %u\n", GPIO_B);
//
//					// If LED extender didn't work after 5 attempts, reset LED extender and try again
//					UARTprintf("Resetting and retrying LED IO extender!\n");
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(10, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//
//					InitLED_Ext();
//				}
//				else if(Counter < 10)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 10)
//				{
//					Counter++;
//					UARTprintf("LED IO Extender still not working, resetting analog board and IO extender!\n");
//
//					AnalogOff();
//
//					userDelay(1000, 1);
//
//					InitAnalog();
//
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0x00); // LED Ext RST _B
//					userDelay(10, 0);	// Wait a little before raising reset line
//					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
//
//					InitLED_Ext();
//
//					// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//					SSIDisable(SSI1_BASE);
//					SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//					SSIEnable(SSI1_BASE);
//
//					SysCtlDelay(2000);
//				}
//				else if(Counter < 15)
//					Counter++;
//				else if((GPIO_A != (gLED_State & 0xFF) || GPIO_B != ((gLED_State >> 8) & 0xFF)) && Counter == 15)
//				{
//					Counter++;
//					gui32Error |= LED_EXTENDER_FAIL;
//					update_Error();
//					UARTprintf("LED IO Extender did not set correctly!\n");
//				}
//			}
//	    }
//	    else
//	    {
//	    	int counter = 0;
//	    	uint8_t Command_received = 0;
//
//	    	while(Command_received == 0)
//	    	{
//	    		g_ulSSI0RXTO = 0;
//
//	    		// Empty SSI FIFO
//	    		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
//	    		{
//	    		}
//
//	    		SSIDataPut(SSI0_BASE, SET_LED);
//	    		SSIDataPut(SSI0_BASE, (gLED_State & 0xFF));
//	    		SSIDataPut(SSI0_BASE, ((gLED_State >> 8) & 0xFF));
//
//	    		uint8_t attempts = 0;
//	    		while(Command_received == 0 && attempts < 3)
//	    		{
//	    			counter = 0;
//	    			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
//	    			SysCtlDelay(2000);
//	    			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
//
//	    			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
//	    			{
//	    				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
//	    				counter++;
//	    			}
//	    			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
//	    			{
//	    				Command_received = 1;
//	    			}
//
//	    			attempts++;
//	    		}
//
//	    		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag
//
//	    		if(Command_received == 1)
//	    		{
//	    			GPIODirModeSet( GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN );	// Set SSI transmit pin as input so we can listen to BT LED without driving
//	    			//		GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	    			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0x00); // LED Ext CS _B
//
//	    			// Disable SSI interrupt because this is a unique SSI transaction
//	    			IntDisable(INT_SSI0);
//
//	    			// Toggle pin again to let BT know CS pin is selected
//	    			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
//	    			SysCtlDelay(2000);
//	    			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
//
//	    			uint8_t i;
//	    			uint32_t junk;
//	    			// Wait for 4 bytes of data to know all has been transmitted
//	    			for(i = 0; i < 4; i++)
//	    			{
//	    				SSIDataGet(SSI0_BASE, &junk);
//	    				//			UARTprintf("%d\n", junk);
//	    			}
//	    			//		uint32_t LED_Rx[2];
//	    			//		uint16_t LED_State_Rx = ~gLED_State;
//	    			//		SSIDataGet(SSI0_BASE, &LED_Rx[0]);
//	    			//		SSIDataGet(SSI0_BASE, &LED_Rx[1]);
//	    			//		UARTprintf("%d\n", LED_Rx[0]);
//	    			//		UARTprintf("%d\n", LED_Rx[1]);
//
//	    			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B
//
//	    			GPIOPinConfigure(GPIO_PA5_SSI0TX);
//	    			GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
//
//	    			SSIIntClear(SSI0_BASE, SSI_RXTO);
//	    			IntEnable(INT_SSI0);
//
//	    			// If none of the LEDs are blinking turn off the oscillator
//	    			if((LED_BLINK & gLED_State) == 0)
//	    				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0); // LED Osc RST _B
//
//	    			if(gDiagnostics >= 1)
//	    			{
//	    				//	    	LED_State_Rx = 0 | (LED_Rx[0] & 0xFF) | ((LED_Rx[1] << 8) & 0xFF);
//	    				UARTprintf("Wrote LED state: %d\n", gLED_State);
//	    				//	    	UARTprintf("Read LED state: %d\n", LED_State_Rx);
//	    			}
//	    		}
//	    		else
//	    		{
//	    			UARTprintf("Set LED not working! ");
//	    			Reset_BT();
//	    		}
//	    	}
//	    }
//
////		if(analog_on == 0)
////		{
////			AnalogOff();
////		}
//
//	}
//}
//
//#endif


////**************************************************************************
//// Uses ADC 5 to verify the waveform generator is running at 1 kHz
//// Created 7/2/2019
//// Parameters:	NONE
//// Returns:		1 if waveform generator is working, 0 if not seeing 1 kHz signal
////**************************************************************************
//uint8_t CheckCond(void)
//{
//	if(gABoard >= AV6_6)
//	{
//		if(gDiagnostics >= 1)
//			UARTprintf("Checking waveform generator!\n");
//
//		uint8_t attempts = 0;
//		uint16_t Non_Zero_counter = 0;	// Count up how many times ADC returns something besides 0 to make sure its working
//		uint16_t frequency;
//
//		while(attempts < 3 && Non_Zero_counter < 3000)
//		{
//			attempts++;
//			uint32_t bits07 = 0, bits815 = 0;
//			int16_t Data = 0;
//
//			// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
//			SSIDisable(SSI1_BASE);
//			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
//			SSIEnable(SSI1_BASE);
//
//			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x02);	// Send wakeup command to ADC
//
//			while(SSIDataGetNonBlocking(SSI1_BASE, &bits07)); // Clear FIFO
//
//			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 0);	// Set CS_B low while sending/receiving data from ADC5
//			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 1);
//
//			SysCtlDelay(SysCtlClockGet()/3000);	// Wait 2 ms after raising start pin
//
//			g_TimerPeriodicInterruptFlag = 0;
//			TimerLoadSet(TIMER1_BASE, TIMER_A, 6250); // Set periodic timer
//			TimerEnable(TIMER1_BASE, TIMER_A);
//
//			uint16_t i;
//			float Voltage[2];
//			//		uint32_t Cross_zero = 0;
//			uint16_t increasing_count = 0;
//			for(i = 0; i < 4001; i++)
//			{
//				//					while(g_TimerPeriodicInterruptFlag == 0);
//				while(g_TimerPeriodicInterruptFlag == 0)
//				{
//					// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
//					// it will prevent the BT from reading incorrect data into the app, TODO: Redesign app to wait for data rather than write read move on
//					if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
//						ConnectMemory(1);
//				}
//				g_TimerPeriodicInterruptFlag = 0;
//
//				SSIDataPut(SSI1_BASE, 0x12); // Send read data command
//				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//				SSIDataGet(SSI1_BASE, &bits07);
//
//				SSIDataPut(SSI1_BASE, 0x00); // Send read data command
//				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//				SSIDataPut(SSI1_BASE, 0x00); // Send read data command
//				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//
//				SSIDataGet(SSI1_BASE, &bits815);
//				SSIDataGet(SSI1_BASE, &bits07);
//
//				Data = bits07 | (bits815 << 8);
//				if(Data != 0)
//					Non_Zero_counter++;
//				Voltage[i & 1] = ((Data * 2.0 * 3000.0 / 65536.0) - 1500) * 2;
//
//				// Check that data crossed zero
//				//			if(i > 0)
//				//				if((Voltage[0] * Voltage[1]) < 0)	// If sign changed this will be negative
//				//					Cross_zero++;	// Count number of times it crosses zero to calculate frequency
//
//				// Check that voltage is increasing, should happen twice every cycle, switched to this because AC divider wasn't centered on 1.5V like voltage shifter was
//				if(i & 1)
//				{
//					if(Voltage[1] > Voltage[0])
//						increasing_count++;
//				}
//				else
//				{
//					if(Voltage[0] > Voltage[1])
//						increasing_count++;
//				}
//
//				//			SysCtlDelay(SysCtlClockGet()/6000);
//
//				//			UARTprintf("%d\n", (int) Voltage[i%2]);
//			}
//
//			TimerDisable(TIMER1_BASE, TIMER_A);
//			g_TimerPeriodicInterruptFlag = 0;
//
//			//		uint32_t frequency = Cross_zero / 2;
//			//		if(gDiagnostics >= 1)
//			//			UARTprintf("Frequency = %d\n", frequency);
//
//			frequency = increasing_count / 2;
//
//			UARTprintf("Frequency = %d\n", frequency);
//
//			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 0);
//			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 1);	// Set CS_B high after completion
//
//			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x04);	// Send power-down command to ADC
//
//			if(Non_Zero_counter < 3000 && attempts < 3)
//			{
//				UARTprintf("Conductivity frequency check ADC returned 0s!\n");
//				UARTprintf("Resetting Analog Board!\n");
//
//				AnalogOff();
//
//				SysCtlDelay(SysCtlClockGet()/3);	// Wait to give analog board time to power-down before turning back on
//
//				InitAnalog();
//			}
//		}
//
//		if(Non_Zero_counter < 3000 && attempts == 3)
//		{
//			gui32Error |= ADC5_FAIL;
//			update_Error();
//			UARTprintf("Conductivity frequency check ADC always read 0's, updating error!\n");
//		}
//
////		WaveGenSet(0);	// Turn off waveform generator
//
//		if(frequency >= 990 && frequency <= 1010)
//			return 1;
//
//		return 0;
//	}
//	else
//		userDelay(1000, 1);
//
//	return 1;
//}

#ifdef CONST_COND_FREQ
//**************************************************************************
// Uses ADC 5 to verify the waveform generator is running at 1 kHz
// ADS114S06
// Created 7/2/2019
// 11/2/2020; Modified to setup channels on ADC, thermistor using another channel
// 12/14/2023: Changed the alorithm to count the number of times the signal changes
// direction and use that to calculate frequency. Works for anything 2 kHz or less
// added a condition that should make multiples with remainders less than .5 work... at least 5 kHz works
// Parameters:	NONE
// Returns:		1 if waveform generator is working, 0 if not seeing 1 kHz signal
//**************************************************************************
uint8_t CheckCond(void)
{
	if(gABoard >= AV6_6)
	{
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Checking waveform generator!\n");
		)

		uint8_t attempts = 0;
		uint16_t Non_Zero_counter = 0;	// Count up how many times ADC returns something besides 0 to make sure its working
		uint16_t frequency;

		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

		// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
		SSIDisable(SSI1_BASE);
		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
		SSIEnable(SSI1_BASE);

		while(attempts < 3 && Non_Zero_counter < 3000)
		{
			attempts++;
			uint32_t bits07 = 0, bits815 = 0;
			int16_t Data = 0;

			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

			// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x02);	// Send wakeup command to ADC

			//
			// After waking ADC setup input channel to COND_AC_DRV, and set data rate
			//
			uint32_t InMux_Rx = 0, DataRate_Rx = 0;
			uint8_t counter = 0;

			while((InMux_Rx != 0x4C || DataRate_Rx != 0x1E) && counter <= 10)
			{
				// Write register 010r rrrr
				// Read register 001r rrrr

				// Input Multiplexer register 0x02, write = 0x42, read = 0x22
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x42, 0x00, 0x4C);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x22, 0x00, 0x00);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)

				SSIDataGet(SSI1_BASE, &InMux_Rx);
				SSIDataGet(SSI1_BASE, &InMux_Rx);
				SSIDataGet(SSI1_BASE, &InMux_Rx);

				//		UARTprintf("Input mux = %x\n", InMux_Rx);

				// Data rate register 0x04, write = 0x44, read = 0x24
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x44, 0x00, 0x1E);	// Set low latency filter 0x10, Set 2000 SPS data rate 0x0C, 4000 SPS data rate 0x0E
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x24, 0x00, 0x00);	// Read back data rate register

				SSIDataGet(SSI1_BASE, &DataRate_Rx);
				SSIDataGet(SSI1_BASE, &DataRate_Rx);
				SSIDataGet(SSI1_BASE, &DataRate_Rx);

				//		UARTprintf("Data rate = %x\n", DataRate_Rx);

				if(counter < 5)
					counter++;
				else if(counter == 5 && (InMux_Rx != 0x4C || DataRate_Rx != 0x1E))
				{
					counter++;

					DEBUG_PRINT(
					UARTprintf("Tried writing ADC 5 registers 5 times, not writing!\n");
					UARTprintf("InMux Rx = %x, should be 0x4c\n");
					UARTprintf("Data rate Rx = %x, should be 0x1c\n");
					UARTprintf("Resetting analog board!\n");
					)

					AnalogOff();

					userDelay(1000, 1);

#ifdef MCU_ZXR
					GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

					// Set pin low, turn on analog board, then set high
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
					GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

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

					if(gBoard == V6_2 || gBoard == V6_3)
					{
						// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
						GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
						GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
					}
#else
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

					// Set pin low, turn on analog board, then set high
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

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

					if(gBoard == V6_2 || gBoard == V6_3)
					{
						// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
						GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
						GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
					}
#endif

					InitIO_Ext();		// Sets up IO Extenders and sets initial values

					if(gBoard == V6_2 || gBoard == V6_3)
						InitLED_Ext();
					else
						SetLED(0, 1);

					InitDAC();			// Resets device and writes configuration register

					// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
					SSIDisable(SSI1_BASE);
					SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
					SSIEnable(SSI1_BASE);
				}
				else if(counter < 10)
					counter++;
				else if(counter == 10 && (InMux_Rx != 0x4C || DataRate_Rx != 0x1E))
				{
					DEBUG_PRINT(UARTprintf("ADC5 failed to initialize input mux or data rate!\nUpdating error and moving on!\n");)
					counter++;
					gui32Error |= ADC5_FAIL;
					update_Error();
				}
			}

			//
			// Once ADC is setup, do the actual measurement
			//
			while(SSIDataGetNonBlocking(SSI1_BASE, &bits07)); // Clear FIFO

			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 0);	// Set CS_B low while sending/receiving data from ADC5
			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 1);

			SysCtlDelay(SysCtlClockGet()/3000);	// Wait 2 ms after raising start pin

			g_TimerPeriodicInterruptFlag = 0;
			TimerLoadSet(TIMER1_BASE, TIMER_A, 6250); // Set periodic timer
			TimerEnable(TIMER1_BASE, TIMER_A);

			uint16_t i;
			float Voltage[2];
//			uint32_t Cross_zero = 0;
			uint16_t count = 0;
			uint8_t Direction = 0; // Flag to indicate the direction we are moving
			for(i = 0; i < 4001; i++)
			{
				//					while(g_TimerPeriodicInterruptFlag == 0);
				while(g_TimerPeriodicInterruptFlag == 0)
				{
					// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
					// it will prevent the BT from reading incorrect data into the app, TODO: Redesign app to wait for data rather than write read move on
#ifdef MCU_ZXR
					if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
						ConnectMemory(1);
#else
					if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
						ConnectMemory(1);
#endif
				}
				g_TimerPeriodicInterruptFlag = 0;

				SSIDataPut(SSI1_BASE, 0x12); // Send read data command
				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
				SSIDataGet(SSI1_BASE, &bits07);

				SSIDataPut(SSI1_BASE, 0x00); // Send read data command
				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
				SSIDataPut(SSI1_BASE, 0x00); // Send read data command
				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

				SSIDataGet(SSI1_BASE, &bits815);
				SSIDataGet(SSI1_BASE, &bits07);

				Data = bits07 | (bits815 << 8);
				if(Data != 0)
					Non_Zero_counter++;
				Voltage[i & 1] = ((Data * 2.0 * 3000.0 / 65536.0) - 1500) * 2;

//				// Check that data crossed zero
//				if(i > 0)
//					if((Voltage[0] * Voltage[1]) < 0)	// If sign changed this will be negative
//						Cross_zero++;	// Count number of times it crosses zero to calculate frequency

//				// Check that voltage is increasing, should happen twice every cycle (, switched to this because AC divider wasn't centered on 1.5V like voltage shifter was
//				if(i & 1)
//				{
//					if(Voltage[1] > Voltage[0])
//						count++;
//				}
//				else
//				{
//					if(Voltage[0] > Voltage[1])
//						count++;
//				}

				// 12/14/2023: Changing to counting direction changes... the previous method was specific to 1kHz sample...
				// count will now be the number of times it changes direction... this should happen twice per cycle
				if(Voltage[i & 1] - Voltage[1 - (i & 1)] >= 0)	// Current reading - previous reading, > 0 (increasing)
				{
					if(Direction != 1)
					{
						Direction = 1;
						count++;	//
					}
				}
				else	// Decreasing
				{
					if(Direction != 0)
					{
						Direction = 0;
						count++;	//
					}
				}

				//			SysCtlDelay(SysCtlClockGet()/6000);

				//			UARTprintf("%d\n", (int) Voltage[i%2]);
			}

			TimerDisable(TIMER1_BASE, TIMER_A);
			g_TimerPeriodicInterruptFlag = 0;

//			frequency = Cross_zero / 2;
//			if(gDiagnostics >= 1)
//				UARTprintf("Frequency = %d\n", frequency);

			frequency = count / 2;	// Dividing by 2 because the event I'm looking for happens twice per cycle

			frequency += 4000 * (int) (COND_FREQ / 4000);

			DEBUG_PRINT(UARTprintf("Frequency = %d\n", frequency);)

			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 0);
			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 1);	// Set CS_B high after completion

			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x04);	// Send power-down command to ADC

			if(Non_Zero_counter < 3000 && attempts < 3)
			{
				DEBUG_PRINT(
				UARTprintf("Conductivity frequency check ADC returned 0s!\n");
				UARTprintf("Resetting Analog Board!\n");
				)

				AnalogOff();

				SysCtlDelay(SysCtlClockGet()/3);	// Wait to give analog board time to power-down before turning back on

				InitAnalog();
			}
		}

		if(Non_Zero_counter < 3000 && attempts == 3)
		{
			gui32Error |= ADC5_FAIL;
			update_Error();
			DEBUG_PRINT(UARTprintf("Conductivity frequency check ADC always read 0's, updating error!\n");)
		}

//		WaveGenSet(0);	// Turn off waveform generator

//		if(frequency >= 990 && frequency <= 1010)
//			return 1;

		if(frequency >= (COND_FREQ * .99) && frequency <= (COND_FREQ * 1.01))
			return 1;

		return 0;
	}
	else
		userDelay(1000, 1);

	return 1;
}
#else
//**************************************************************************
// Uses ADC 5 to verify the waveform generator is running at specified frequency
// ADS114S06
// Created 7/2/2019
// 11/2/2020; Modified to setup channels on ADC, thermistor using another channel
// 12/14/2023: Changed the alorithm to count the number of times the signal changes
// 2/7/2024: Giving expected frequency as input
// direction and use that to calculate frequency. Works for anything 2 kHz or less
// added a condition that should make multiples with remainders less than .5 work... at least 5 kHz works
// Parameters:	NONE
// Returns:		1 if waveform generator is working, 0 if not seeing 1 kHz signal
//**************************************************************************
uint8_t CheckCond(uint16_t ui16freq)
{
	if(gABoard >= AV6_6)
	{
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Checking waveform generator!\n");
		)

		uint8_t attempts = 0;
		uint16_t Non_Zero_counter = 0;	// Count up how many times ADC returns something besides 0 to make sure its working
		uint16_t frequency;

		while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

		// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
		SSIDisable(SSI1_BASE);
		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
		SSIEnable(SSI1_BASE);

		while(attempts < 3 && Non_Zero_counter < 3000)
		{
			attempts++;
			uint32_t bits07 = 0, bits815 = 0;
			int16_t Data = 0;

			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

			// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x02);	// Send wakeup command to ADC

			//
			// After waking ADC setup input channel to COND_AC_DRV, and set data rate
			//
			uint32_t InMux_Rx = 0, DataRate_Rx = 0;
			uint8_t counter = 0;

			while((InMux_Rx != 0x4C || DataRate_Rx != 0x1E) && counter <= 10)
			{
				// Write register 010r rrrr
				// Read register 001r rrrr

				// Input Multiplexer register 0x02, write = 0x42, read = 0x22
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x42, 0x00, 0x4C);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x22, 0x00, 0x00);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)

				SSIDataGet(SSI1_BASE, &InMux_Rx);
				SSIDataGet(SSI1_BASE, &InMux_Rx);
				SSIDataGet(SSI1_BASE, &InMux_Rx);

				//		UARTprintf("Input mux = %x\n", InMux_Rx);

				// Data rate register 0x04, write = 0x44, read = 0x24
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x44, 0x00, 0x1E);	// Set low latency filter 0x10, Set 2000 SPS data rate 0x0C, 4000 SPS data rate 0x0E
				SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x24, 0x00, 0x00);	// Read back data rate register

				SSIDataGet(SSI1_BASE, &DataRate_Rx);
				SSIDataGet(SSI1_BASE, &DataRate_Rx);
				SSIDataGet(SSI1_BASE, &DataRate_Rx);

				//		UARTprintf("Data rate = %x\n", DataRate_Rx);

				if(counter < 5)
					counter++;
				else if(counter == 5 && (InMux_Rx != 0x4C || DataRate_Rx != 0x1E))
				{
					counter++;

					DEBUG_PRINT(
					UARTprintf("Tried writing ADC 5 registers 5 times, not writing!\n");
					UARTprintf("InMux Rx = %x, should be 0x4c\n");
					UARTprintf("Data rate Rx = %x, should be 0x1c\n");
					UARTprintf("Resetting analog board!\n");
					)

					AnalogOff();

					userDelay(1000, 1);

#ifdef MCU_ZXR
					GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

					// Set pin low, turn on analog board, then set high
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
					GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

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

					if(gBoard == V6_2 || gBoard == V6_3)
					{
						// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
						GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
						GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
					}
#else
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

					// Set pin low, turn on analog board, then set high
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

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

					if(gBoard == V6_2 || gBoard == V6_3)
					{
						// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
						GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
						GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
					}
#endif

					InitIO_Ext();		// Sets up IO Extenders and sets initial values

					if(gBoard == V6_2 || gBoard == V6_3)
						InitLED_Ext();
					else
						SetLED(0, 1);

					InitDAC();			// Resets device and writes configuration register

					// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
					SSIDisable(SSI1_BASE);
					SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
					SSIEnable(SSI1_BASE);
				}
				else if(counter < 10)
					counter++;
				else if(counter == 10 && (InMux_Rx != 0x4C || DataRate_Rx != 0x1E))
				{
					DEBUG_PRINT(UARTprintf("ADC5 failed to initialize input mux or data rate!\nUpdating error and moving on!\n");)
					counter++;
					gui32Error |= ADC5_FAIL;
					update_Error();
				}
			}

			//
			// Once ADC is setup, do the actual measurement
			//
			while(SSIDataGetNonBlocking(SSI1_BASE, &bits07)); // Clear FIFO

			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 0);	// Set CS_B low while sending/receiving data from ADC5
			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 1);

			SysCtlDelay(SysCtlClockGet()/3000);	// Wait 2 ms after raising start pin

			g_TimerPeriodicInterruptFlag = 0;
			TimerLoadSet(TIMER1_BASE, TIMER_A, 6250); // Set periodic timer
			TimerEnable(TIMER1_BASE, TIMER_A);

			uint16_t i;
			float Voltage[2];
//			uint32_t Cross_zero = 0;
			uint16_t count = 0;
			uint8_t Direction = 0; // Flag to indicate the direction we are moving
			for(i = 0; i < 4001; i++)
			{
				//					while(g_TimerPeriodicInterruptFlag == 0);
				while(g_TimerPeriodicInterruptFlag == 0)
				{
					// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
					// it will prevent the BT from reading incorrect data into the app, TODO: Redesign app to wait for data rather than write read move on
#ifdef MCU_ZXR
					if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
						ConnectMemory(1);
#else
					if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
						ConnectMemory(1);
#endif
				}
				g_TimerPeriodicInterruptFlag = 0;

				SSIDataPut(SSI1_BASE, 0x12); // Send read data command
				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
				SSIDataGet(SSI1_BASE, &bits07);

				SSIDataPut(SSI1_BASE, 0x00); // Send read data command
				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
				SSIDataPut(SSI1_BASE, 0x00); // Send read data command
				while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

				SSIDataGet(SSI1_BASE, &bits815);
				SSIDataGet(SSI1_BASE, &bits07);

				Data = bits07 | (bits815 << 8);
				if(Data != 0)
					Non_Zero_counter++;
				Voltage[i & 1] = ((Data * 2.0 * 3000.0 / 65536.0) - 1500) * 2;

//				// Check that data crossed zero
//				if(i > 0)
//					if((Voltage[0] * Voltage[1]) < 0)	// If sign changed this will be negative
//						Cross_zero++;	// Count number of times it crosses zero to calculate frequency

//				// Check that voltage is increasing, should happen twice every cycle (, switched to this because AC divider wasn't centered on 1.5V like voltage shifter was
//				if(i & 1)
//				{
//					if(Voltage[1] > Voltage[0])
//						count++;
//				}
//				else
//				{
//					if(Voltage[0] > Voltage[1])
//						count++;
//				}

				// 12/14/2023: Changing to counting direction changes... the previous method was specific to 1kHz sample...
				// count will now be the number of times it changes direction... this should happen twice per cycle
				if(Voltage[i & 1] - Voltage[1 - (i & 1)] >= 0)	// Current reading - previous reading, > 0 (increasing)
				{
					if(Direction != 1)
					{
						Direction = 1;
						count++;	//
					}
				}
				else	// Decreasing
				{
					if(Direction != 0)
					{
						Direction = 0;
						count++;	//
					}
				}

				//			SysCtlDelay(SysCtlClockGet()/6000);

				//			UARTprintf("%d\n", (int) Voltage[i%2]);
			}

			TimerDisable(TIMER1_BASE, TIMER_A);
			g_TimerPeriodicInterruptFlag = 0;

//			frequency = Cross_zero / 2;
//			if(gDiagnostics >= 1)
//				UARTprintf("Frequency = %d\n", frequency);

			frequency = count / 2;	// Dividing by 2 because the event I'm looking for happens twice per cycle

			frequency += 4000 * (int) (ui16freq / 4000);

			DEBUG_PRINT(UARTprintf("Frequency = %d\n", frequency);)

			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 0);
			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 1);	// Set CS_B high after completion

			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x04);	// Send power-down command to ADC

			if(Non_Zero_counter < 3000 && attempts < 3)
			{
				DEBUG_PRINT(
				UARTprintf("Conductivity frequency check ADC returned 0s!\n");
				UARTprintf("Resetting Analog Board!\n");
				)

				AnalogOff();

				SysCtlDelay(SysCtlClockGet()/3);	// Wait to give analog board time to power-down before turning back on

				InitAnalog();
			}
		}

		if(Non_Zero_counter < 3000 && attempts == 3)
		{
			gui32Error |= ADC5_FAIL;
			update_Error();
			DEBUG_PRINT(UARTprintf("Conductivity frequency check ADC always read 0's, updating error!\n");)
		}

//		WaveGenSet(0);	// Turn off waveform generator

//		if(frequency >= 990 && frequency <= 1010)
//			return 1;

		if(frequency >= (ui16freq * .99) && frequency <= (ui16freq * 1.01))
			return 1;

		return 0;
	}
	else
		userDelay(1000, 1);

	return 1;
}
#endif

//**************************************************************************
// Uses ADC 5 to calculate resistance on thermistor and calculate temperature
// based on that
// Created 11/2/2020
// Parameters:	NONE
// Returns:		Temperature
//**************************************************************************
float ReadThermistor(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Reading thermistor temperature!\n");
	)

	uint32_t bits07 = 0, bits815 = 0;
	int16_t Data = 0;
	uint16_t Non_Zero_counter = 0;	// Count up how many times ADC returns something besides 0 to make sure its working

	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	ADCCurrentSet(1);

	// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x02);	// Send wakeup command to ADC

	//
	// After waking ADC setup input channel to COND_AC_DRV, and set data rate
	//
	uint32_t InMux_Rx = 0, DataRate_Rx = 0;
	uint8_t counter = 0;
	uint8_t InMux_Tx = 0x3C; // // Set positive input to AIN 3 (0011 = 0x3) (TURB_DETECT_180), negative input to AINCOM (1100 = 0xC) (GND)
	if(gABoard >= AV7_2)
		InMux_Tx = 0x5C;
	uint8_t DataRate_Tx = 0x1E;	// Set low latency filter 0x10, Set 2000 SPS data rate 0x0C, 4000 SPS data rate 0x0E

	while((InMux_Rx != InMux_Tx || DataRate_Rx != DataRate_Tx) && counter <= 10)
	{
		// Write register 010r rrrr
		// Read register 001r rrrr

		// Input Multiplexer register 0x02, write = 0x42, read = 0x22
		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x42, 0x00, InMux_Tx);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)
		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x22, 0x00, 0x00);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)

		SSIDataGet(SSI1_BASE, &InMux_Rx);
		SSIDataGet(SSI1_BASE, &InMux_Rx);
		SSIDataGet(SSI1_BASE, &InMux_Rx);

		//		UARTprintf("Input mux = %x\n", InMux_Rx);

		// Data rate register 0x04, write = 0x44, read = 0x24
		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x44, 0x00, DataRate_Tx);	// Set low latency filter 0x10, Set 2000 SPS data rate 0x0C, 4000 SPS data rate 0x0E
		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x24, 0x00, 0x00);	// Read back data rate register

		SSIDataGet(SSI1_BASE, &DataRate_Rx);
		SSIDataGet(SSI1_BASE, &DataRate_Rx);
		SSIDataGet(SSI1_BASE, &DataRate_Rx);

		//		UARTprintf("Data rate = %x\n", DataRate_Rx);

		if(counter < 5)
			counter++;
		else if(counter == 5 && (InMux_Rx != InMux_Tx || DataRate_Rx != DataRate_Tx))
		{
			counter++;

			DEBUG_PRINT(
			UARTprintf("Tried writing ADC 5 registers 5 times, not writing!\n");
			if(gABoard >= AV7_2)
				UARTprintf("InMux Rx = %x, should be 0x5c\n");
			else
				UARTprintf("InMux Rx = %x, should be 0x3c\n");
			UARTprintf("Data rate Rx = %x, should be 0x1c\n");
			UARTprintf("Resetting analog board!\n");
			)

			AnalogOff();

			userDelay(1000, 1);

#ifdef MCU_ZXR
			GPIOPinWrite(IO_RESET_ANALOG_BASE, IO_RESET_ANALOG_PIN, IO_RESET_ANALOG_PIN); // Analog Reset Set high for normal operation

			// Set pin low, turn on analog board, then set high
			GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, 0x00); // Conductivity ADC CNV Pin _B
			GPIOPinWrite(IO_ANALOG_ON_BASE, IO_ANALOG_ON_PIN, IO_ANALOG_ON_PIN); // Analog On

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

			if(gBoard == V6_2 || gBoard == V6_3)
			{
				// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
				GPIOPinWrite(IO_LED_EXT_CS_B_BASE, IO_LED_EXT_CS_B_PIN, IO_LED_EXT_CS_B_PIN); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
				GPIOPinWrite(IO_LED_EXT_RST_B_BASE, IO_LED_EXT_RST_B_PIN, IO_LED_EXT_RST_B_PIN); // LED Ext RST _B
			}
#else
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7); // Analog Reset Set high for normal operation

			// Set pin low, turn on analog board, then set high
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00); // Conductivity ADC CNV Pin _B
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Analog On

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

			if(gBoard == V6_2 || gBoard == V6_3)
			{
				// Raise extender reset line to activate device, specs say device active after reset = 0 ns @ 5V
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED Ext CS _B	// Make sure CS_B is high, will only be low if resetting analog board on V6_2 and V6_3
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // LED Ext RST _B
			}
#endif


			InitIO_Ext();		// Sets up IO Extenders and sets initial values

			if(gBoard == V6_2 || gBoard == V6_3)
				InitLED_Ext();
			else
				SetLED(0, 1);

			InitDAC();			// Resets device and writes configuration register

			// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);
		}
		else if(counter < 10)
			counter++;
		else if(counter == 10 && (InMux_Rx != InMux_Tx || DataRate_Rx != DataRate_Tx))
		{
			DEBUG_PRINT(UARTprintf("ADC5 failed to initialize input mux or data rate!\nUpdating error and moving on!\n");)
			counter++;
			gui32Error |= ADC5_FAIL;
			update_Error();
		}
	}

	//
	// Once ADC is setup, do the actual measurement
	//
	uint8_t attempts = 0;
	float Voltage = 0;
	uint16_t cycles = 2000;

	while(Non_Zero_counter < (cycles/2) && attempts < 3)
	{
		while(SSIDataGetNonBlocking(SSI1_BASE, &bits07)); // Clear FIFO

		IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 0);	// Set CS_B low while sending/receiving data from ADC5
		IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 1);

		SysCtlDelay(SysCtlClockGet()/3000);	// Wait 1 ms after raising start pin

		g_TimerPeriodicInterruptFlag = 0;
		TimerLoadSet(TIMER1_BASE, TIMER_A, 6250); // Set periodic timer
		TimerEnable(TIMER1_BASE, TIMER_A);

		uint16_t i;
		Voltage = 0;
		for(i = 0; i < (cycles + 1); i++)
		{
			//					while(g_TimerPeriodicInterruptFlag == 0);
			while(g_TimerPeriodicInterruptFlag == 0)
			{
				// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
				// it will prevent the BT from reading incorrect data into the app, TODO: Redesign app to wait for data rather than write read move on
#ifdef MCU_ZXR
				if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
					ConnectMemory(1);
#else
				if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
					ConnectMemory(1);
#endif

			}
			g_TimerPeriodicInterruptFlag = 0;

			SSIDataPut(SSI1_BASE, 0x12); // Send read data command
			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
			SSIDataGet(SSI1_BASE, &bits07);

			SSIDataPut(SSI1_BASE, 0x00); // Send read data command
			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
			SSIDataPut(SSI1_BASE, 0x00); // Send read data command
			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

			SSIDataGet(SSI1_BASE, &bits815);
			SSIDataGet(SSI1_BASE, &bits07);

			Data = bits07 | (bits815 << 8);
			Voltage += ((Data * 3000.0 / 65536.0)) * 2;
			if(Data != 0)
				Non_Zero_counter++;
		}

		TimerDisable(TIMER1_BASE, TIMER_A);
		g_TimerPeriodicInterruptFlag = 0;

		Voltage /= cycles;	// Find average voltage by dividing by number of cycles

		IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 0);
		IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 1);	// Set CS_B high after completion

		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x04);	// Send power-down command to ADC

		if(Non_Zero_counter < (cycles / 2))
		{
			DEBUG_PRINT(
			UARTprintf("Thermistor ADC returned 0s!\n");
			UARTprintf("Resetting Analog Board!\n");
			)

			AnalogOff();

			SysCtlDelay(SysCtlClockGet()/3);	// Wait to give analog board time to power-down before turning back on

			InitAnalog();
		}
	}
	ADCCurrentSet(0);

	if(Non_Zero_counter < (cycles/2) && attempts >= 3)
	{
		gui32Error |= ADC5_FAIL;
		update_Error();
		DEBUG_PRINT(UARTprintf("Thermistor ADC always read 0's, updating error!\n");)
	}

//	EEPROMProgram((uint32_t *) &Therm_I, OFFSET_THERM_CURRENT, 4);
	float Therm_I = 100e-6;
	EEPROMRead((uint32_t *) &Therm_I, OFFSET_THERM_CURRENT, 4);
	if(Therm_I != Therm_I)
		Therm_I = 100e-6;

//	UARTprintf("Average Voltage: %d\n", (int) (Voltage));
	float Resistance = (Voltage / 1000) / Therm_I;	// Resistance in Ohms
	float R25 = 10000;	// Resistance of thermistor at 25C
	float Temp = 1/(3.3540154e-3 + 2.5627725e-4 * log(Resistance/R25) + 2.0829210e-6 * pow(log(Resistance/R25), 2) + 7.3003206e-8 * pow(log(Resistance/R25), 3)) - 273.15;

	if(Temp < 2 || Temp > 50)
	{
		DEBUG_PRINT(UARTprintf("Thermistor reading seems wrong!\n");)
		gui32Error |= THERMISTOR_FAILED;
	}
	return Temp;
}

//**************************************************************************
// Reads current time directly from RTC and prints it over UART
// Created 6/17/2020
// Parameters:	NONE
// Returns:		DateTime; as 7 uint8_t formatted as M/D/YY H:M:S
//**************************************************************************
uint8_t * GetTime(void)
{
	if(gBoard > V1)
	{
		uint32_t ms_waited = 0;
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
#endif
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("IO Ext Set");
			)

			// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
			// it will prevent the BT from reading incorrect data into the app, TODO: Redesign app to wait for data rather than write read move on
			if(gui8MemConnected == 0)
				ConnectMemory(1);

			userDelay(1, 1);
			ms_waited++;
			if(ms_waited == 10000)
			{
				DEBUG_PRINT(UARTprintf("Waiting for BT to finish with I2C, now waited 10 seconds for Print Time\n");)
				ms_waited = 0;
			}
		}
		update_TivaI2C(1);
		ms_waited = 0;
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish using I2C
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
#endif
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("IO Ext Set");
			)

			userDelay(1, 1);
			ms_waited++;
			if(ms_waited == 10000)
			{
				DEBUG_PRINT(UARTprintf("Told BT that Tiva needs I2C, now waited 10 seconds for Print Time\n");)
				ms_waited = 0;
			}
		}
	}

	uint8_t ui8RTC_Addr = 0x68;
	uint32_t Error = I2C_MASTER_ERR_NONE;

	DEBUG_PRINT(
	if(gBoard > V1 && gDiagnostics >= 2)
		UARTprintf("Reading from RTC\n");
	)

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE)){
    	DEBUG_PRINT(
    	if(gDiagnostics >= 2)
    		UARTprintf("Print Time, I2CMasterBusy \n");
    	)
    }

    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8RTC_Addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, 0x00);	// Start reading at register 0, seconds register

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE)){
    	DEBUG_PRINT(
    	if(gDiagnostics >= 2)
    		UARTprintf("Print Time, I2CMasterBusy \n");
    	)
    }

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, ui8RTC_Addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE)){
    	DEBUG_PRINT(
    	if(gDiagnostics >= 2)
    		UARTprintf("Print Time, I2CMasterBusy \n");
    	)
    }

    uint8_t readBuffer[7];

    //return data pulled from the specified register
    readBuffer[0] = I2CMasterDataGet(I2C0_BASE);

    uint8_t i;
    for(i = 1; i < 6; i++)
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(I2C0_BASE)){
        	DEBUG_PRINT(
        	if(gDiagnostics >= 2)
        		UARTprintf("Print Time, I2CMasterBusy \n");
        	)
        }

        //
        readBuffer[i] = I2CMasterDataGet(I2C0_BASE);
    }

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE)){
    	DEBUG_PRINT(
    	if(gDiagnostics >= 2)
    		UARTprintf("Print Time, I2CMasterBusy \n");
    	)
    }

    //
    readBuffer[6] = I2CMasterDataGet(I2C0_BASE);

    Error = I2CMasterErr(I2C0_BASE);
    if(Error != I2C_MASTER_ERR_NONE)
    {
    	gui32Error |= I2C_FAILED;

    	update_Error();
    	DEBUG_PRINT(
    	if(gDiagnostics >= 1)
    		UARTprintf("Print Time failed!\n");
    	)
    }

    update_TivaI2C(0);

    static uint8_t DateTime[7];
    DateTime[0] = (10 * ((readBuffer[5] >> 4) & 0x01)) + (readBuffer[5] & 0x0F);
    DateTime[1] = (10 * ((readBuffer[4] >> 4) & 0x03)) + (readBuffer[4] & 0x0F);
    DateTime[2] = 20;
    DateTime[3] = (10 * (uint16_t) (readBuffer[6] >> 4)) + (uint16_t) (readBuffer[6] & 0x0F);
    DateTime[4] = (10 * ((readBuffer[2] >> 4) & 0x03)) + (readBuffer[2] & 0x0F);
    DateTime[5] = (10 * ((readBuffer[1] >> 4) & 0x07)) + (readBuffer[1] & 0x0F);
    DateTime[6] = (10 * ((readBuffer[0] >> 4) & 0x07)) + (readBuffer[0] & 0x0F);

    return DateTime;
}

//**************************************************************************
// Reads current time directly from RTC and prints it over UART
// Created 6/17/2020
// Parameters:	NONE
// Returns:		NONE
//**************************************************************************
uint8_t PrintTime(void)
{
//	if(gBoard > V1)
//	{
//		uint32_t ms_waited = 0;
//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
//		{
//			if(gDiagnostics >= 1)
//				UARTprintf("IO Ext Set");
//
//			// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
//			// it will prevent the BT from reading incorrect data into the app, TODO: Redesign app to wait for data rather than write read move on
//			if(gui8MemConnected == 0)
//				ConnectMemory(1);
//
//			userDelay(1, 1);
//			ms_waited++;
//			if(ms_waited == 10000)
//			{
//				UARTprintf("Waiting for BT to finish with I2C, now waited 10 seconds for Print Time\n");
//				ms_waited = 0;
//			}
//		}
//		update_TivaI2C(1);
//		ms_waited = 0;
//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish using I2C
//		{
//			if(gDiagnostics >= 1)
//				UARTprintf("IO Ext Set");
//
//			userDelay(1, 1);
//			ms_waited++;
//			if(ms_waited == 10000)
//			{
//				UARTprintf("Told BT that Tiva needs I2C, now waited 10 seconds for Print Time\n");
//				ms_waited = 0;
//			}
//		}
//	}
//
//	uint8_t ui8RTC_Addr = 0x68;
//	uint32_t Error = I2C_MASTER_ERR_NONE;
//
//	if(gBoard > V1 && gDiagnostics >= 2)
//		UARTprintf("Reading from RTC\n");
//
//    //wait for MCU to finish transaction
//    while(I2CMasterBusy(I2C0_BASE)){
//    	if(gDiagnostics >= 2)
//    		UARTprintf("Print Time, I2CMasterBusy \n");
//    }
//
//    //specify that we are writing (a register address) to the
//    //slave device
//    I2CMasterSlaveAddrSet(I2C0_BASE, ui8RTC_Addr, false);
//
//    //specify register to be read
//    I2CMasterDataPut(I2C0_BASE, 0x00);	// Start reading at register 0, seconds register
//
//    //send control byte and register address byte to slave device
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//
//    //wait for MCU to finish transaction
//    while(I2CMasterBusy(I2C0_BASE)){
//    	if(gDiagnostics >= 2)
//    		UARTprintf("Print Time, I2CMasterBusy \n");
//    }
//
//    //specify that we are going to read from slave device
//    I2CMasterSlaveAddrSet(I2C0_BASE, ui8RTC_Addr, true);
//
//    //send control byte and read from the register we
//    //specified
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
//
//    //wait for MCU to finish transaction
//    while(I2CMasterBusy(I2C0_BASE)){
//    	if(gDiagnostics >= 2)
//    		UARTprintf("Print Time, I2CMasterBusy \n");
//    }
//
//    uint8_t readBuffer[7];
//
//    //return data pulled from the specified register
//    readBuffer[0] = I2CMasterDataGet(I2C0_BASE);
//
//    uint8_t i;
//    for(i = 1; i < 6; i++)
//    {
//        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//
//        //wait for MCU to finish transaction
//        while(I2CMasterBusy(I2C0_BASE)){
//        	if(gDiagnostics >= 2)
//        		UARTprintf("Print Time, I2CMasterBusy \n");
//        }
//
//        //
//        readBuffer[i] = I2CMasterDataGet(I2C0_BASE);
//    }
//
//    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
//
//    //wait for MCU to finish transaction
//    while(I2CMasterBusy(I2C0_BASE)){
//    	if(gDiagnostics >= 2)
//    		UARTprintf("Print Time, I2CMasterBusy \n");
//    }
//
//    //
//    readBuffer[6] = I2CMasterDataGet(I2C0_BASE);
//
//    Error = I2CMasterErr(I2C0_BASE);
//    if(Error != I2C_MASTER_ERR_NONE)
//    {
//    	gui32Error |= I2C_FAILED;
//
//    	update_Error();
//    	if(gDiagnostics >= 1)
//    		UARTprintf("Print Time failed!\n");
//    }
//
//    update_TivaI2C(0);

	uint32_t Error = ~gui32Error;	// Get opposite of any error codes before grabbing time
	uint8_t * DateTime = GetTime();
	Error &= gui32Error;	// Anding error code with the opposite I grabbed earlier will only result in change in error, giving me any new error that occurred between then and now

	UARTprintf("Date and Time: %d/%d/%d%d, %d:%d:%d\n", DateTime[0], DateTime[1], DateTime[2], DateTime[3], DateTime[4], DateTime[5], DateTime[6]);

//    UARTprintf("Date and Time: %d/%d/%d, %d:%d:%d\n", (10 * ((readBuffer[5] >> 4) & 0x01)) + (readBuffer[5] & 0x0F), (10 * ((readBuffer[4] >> 4) & 0x03)) + (readBuffer[4] & 0x0F), 2000 + (10 * (uint16_t) (readBuffer[6] >> 4)) + (uint16_t) (readBuffer[6] & 0x0F), (10 * ((readBuffer[2] >> 4) & 0x03)) + (readBuffer[2] & 0x0F), (10 * ((readBuffer[1] >> 4) & 0x07)) + (readBuffer[1] & 0x0F), (10 * ((readBuffer[0] >> 4) & 0x07)) + (readBuffer[0] & 0x0F));
//    if((readBuffer[1] & 0x80) != 0)
//    	UARTprintf("RTC set oscillator flag fail, time may be incorrect :(\n");

    if((Error & I2C_FAILED) != 0)
    {
    	DEBUG_PRINT(UARTprintf("RTC didn't read back correctly!\n");)
    	return 0;
    }
    else
    	return 1;
}
