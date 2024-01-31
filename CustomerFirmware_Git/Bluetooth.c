//*****************************************************************************
//
// Bluetooth.c - Functions used in e-sens firmware that deal directly with BT
//
// Author: Jason Castillo
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "inc/hw_hibernate.h"
#include "inc/hw_memmap.h"
#include "driverlib/eeprom.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "Bluetooth.h"
#include "Components.h"
#include "Communication.h"
#include "Helper.h"
#include "main.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_nvic.h"
#ifdef MCU_ZXR
#include "PinMap.h"
#endif

uint8_t gCartridge = 1;	// Whether a cartridge with memory is present or not
//uint8_t gMemory = 0;
uint8_t gStatus = 0;
uint8_t gOperation = 0;

uint8_t gPumping = 0;	// Flag to set while pumping, used to prevent battery interrupt taking too long as it stalls the pump

//**************************************************************************
// Verifies BT has initialized I2C values and is working
// Parameters: NONE
//**************************************************************************
void InitBT(void)
{
	int counter = 0;
//	g_ui32DataRx0[0] = 0;	// Don't set incoming data here, will be set while BT is being reset to catch the first command
	uint8_t attempt = 0;

	SysCtlDelay(SysCtlClockGet()/300);

	if(gBoard > V1 && HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// Don't wait for BT chip after waking from hibernate, it is already set up
	{
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == 0 && (g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && g_ui32DataRx0[0] != BOOTUP) && attempt < 3)	// Wait for BT to read memory at startup
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1 && counter == 0)
				UARTprintf("BT hasn't raised pin, is this OAD?\n");
			)
			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
			counter++;

			if(counter == 20000)
			{
				DEBUG_PRINT(
				if(gDiagnostics >= 1)
					UARTprintf("Waited 20 seconds and BT never raised pin... Resetting BT \n");
				)
				counter = 0;
				attempt++;
				GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, IO_RESET_BT_PIN); // Reset BT
				SysCtlDelay(10000);
				GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, 0x00); // Reset BT

				SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
			}
		}

		attempt = 0;
		counter = 0;

		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && (g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && g_ui32DataRx0[0] != BOOTUP) && attempt < 3)	// Wait for BT to read memory at startup
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Init BT \n");
			)
			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
			counter++;

			if(counter == 5000)
			{
				DEBUG_PRINT(
				if(gDiagnostics >= 1)
					UARTprintf("Waited 5 seconds and BT never responded... Resetting BT \n");
				)
				counter = 0;
				attempt++;
				GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, IO_RESET_BT_PIN); // Reset BT
				SysCtlDelay(10000);
				GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, 0x00); // Reset BT

				SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
			}
		}

		if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && (g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && g_ui32DataRx0[0] != BOOTUP))
		{
			DEBUG_PRINT(UARTprintf("Bluetooth chip failed to boot up and toggle I2C used by BT pin\n");)
			gui32Error |= BT_FAILED;
		}
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0 && (g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && g_ui32DataRx0[0] != BOOTUP) && attempt < 3)	// Wait for BT to read memory at startup
		{
			if(gDiagnostics >= 1 && counter == 0)
				UARTprintf("BT hasn't raised pin, is this OAD?\n");
			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
			counter++;

			if(counter == 20000)
			{
				if(gDiagnostics >= 1)
					UARTprintf("Waited 20 seconds and BT never raised pin... Resetting BT \n");
				counter = 0;
				attempt++;
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT
				SysCtlDelay(10000);
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT

				SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
			}
		}

		attempt = 0;
		counter = 0;

		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && (g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && g_ui32DataRx0[0] != BOOTUP) && attempt < 3)	// Wait for BT to read memory at startup
		{
			if(gDiagnostics >= 1)
				UARTprintf("Init BT \n");
			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
			counter++;

			if(counter == 5000)
			{
				if(gDiagnostics >= 1)
					UARTprintf("Waited 5 seconds and BT never responded... Resetting BT \n");
				counter = 0;
				attempt++;
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT
				SysCtlDelay(10000);
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT

				SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
			}
		}

		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && (g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && g_ui32DataRx0[0] != BOOTUP))
		{
			UARTprintf("Bluetooth chip failed to boot up and toggle I2C used by BT pin\n");
			gui32Error |= BT_FAILED;
		}
#endif


		// Update status after BT has finished booting up, BT will ignore first command sent to it on bootup so this is to get that first command out of way
		// BT ignores first command in case BT freezes and has to be reset, SSI buffer needs to be cleared and BT must clock out bits for that to happen
		update_Status(STATUS_IDLE, OPERATION_IDLE);

//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish booting up
	}
//	else if(gBoard > V1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)
//		update_Status(STATUS_IDLE, OPERATION_IDLE);
	if(gBoard > V1)
	{
		update_Dev_Info();	// Dev info must be completed first to get BT services up and running
		update_MonoCl();
		update_Alkalinity();
		update_Error();		// Update error here in case there are any errors that occurred before BT initialization (possibly EEPROM)
//		update_Auto_Cal();	// Doesn't trigger when inside InitBT function, works when placed after all other initialization
	}
}

////**************************************************************************
//// Verifies BT has initialized I2C values and is working
//// Parameters: NONE
////**************************************************************************
//void InitBT(void)
//{
//	int counter = 0;
//	g_ui32DataRx0[0] = 0;
//
//	SysCtlDelay(SysCtlClockGet()/300);
//
//	if(gBoard > V1 && HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// Don't wait for BT chip after waking from hibernate, it is already set up
//	{
//		uint8_t attempts = 0;
//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && attempts < 3)	// Wait for BT to read memory at startup
//		{
//			if(gDiagnostics >= 1)
//				UARTprintf("Init BT \n");
//			SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
//			counter++;
//
//			if(counter == 5000)
//			{
//				if(gDiagnostics >= 1)
//					UARTprintf("Waited 5 seconds and BT never... Resetting BT \n");
//				counter = 0;
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT
//				SysCtlDelay(10000);
//				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT
//
//				SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
//			}
//		}
//
//		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && g_ui32DataRx0[0] != REQUEST_TIVA_INSTRUCTION && attempts == 3)
//		{
//			UARTprintf("Bluetooth chip failed to boot up and toggle I2C used by BT pin\n");
//			gui32Error |= BT_FAILED;
//		}
//
//		// Update status after BT has finished booting up, BT will ignore first command sent to it on bootup so this is to get that first command out of way
//		// BT ignores first command in case BT freezes and has to be reset, SSI buffer needs to be cleared and BT must clock out bits for that to happen
//		update_Status(STATUS_IDLE, OPERATION_IDLE);
//
////		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish booting up
//	}
////	else if(gBoard > V1 && HibernateIntStatus(1) == HIBERNATE_INT_PIN_WAKE)
////		update_Status(STATUS_IDLE, OPERATION_IDLE);
//	if(gBoard > V1)
//	{
//		update_Dev_Info();	// Dev info must be completed first to get BT services up and running
//		update_MonoCl();
////		update_Auto_Cal();	// Doesn't trigger when inside InitBT function, works when placed after all other initialization
//	}
//}

//**************************************************************************
// Resets the BT chip and reinitializes BT values
// Parameters: NONE
//**************************************************************************
void Reset_BT(void)
{
	DEBUG_PRINT(UARTprintf("Resetting BT Chip!\n");)

	int counter = 0;
	g_ui32DataRx0[0] = 0;

#ifdef MCU_ZXR
	// Pulse BT reset signal
	GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, IO_RESET_BT_PIN);
	SysCtlDelay(SysCtlClockGet()/3000);
	GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, 0x00);

	SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working

	while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to read memory at startup
	{
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
		counter++;

		if(counter == 5000)
		{
			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Waited 5 seconds... Resetting BT again \n");
			)
			counter = 0;
			GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, IO_RESET_BT_PIN); // Reset BT
			SysCtlDelay(10000);
			GPIOPinWrite(IO_RESET_BT_BASE, IO_RESET_BT_PIN, 0x00); // Reset BT

			SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
		}
	}

	// Pulse BT chip and make sure it responds, don't load anything into SSI buffer in case there is already something there it will be cleared out
	uint8_t Command_received = 0;
	uint8_t attempts = 0;
	while(Command_received == 0 && attempts < 3)
	{
		counter = 0;
		GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // Tiva Request for SPI
		SysCtlDelay(2000);
		GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // Tiva Request for SPI

		while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
		{
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
			counter++;
		}
		if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
			Command_received = 1;

		attempts++;
	}
#else
	// Pulse BT reset signal
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
	SysCtlDelay(SysCtlClockGet()/3000);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);

	SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working

	while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to read memory at startup
	{
		SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
		counter++;

		if(counter == 5000)
		{
			if(gDiagnostics >= 1)
				UARTprintf("Waited 5 seconds... Resetting BT again \n");
			counter = 0;
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT
			SysCtlDelay(10000);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT

			SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
		}
	}

	// Pulse BT chip and make sure it responds, don't load anything into SSI buffer in case there is already something there it will be cleared out
	uint8_t Command_received = 0;
	uint8_t attempts = 0;
	while(Command_received == 0 && attempts < 3)
	{
		counter = 0;
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // Tiva Request for SPI
		SysCtlDelay(2000);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // Tiva Request for SPI

		while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
		{
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
			counter++;
		}
		if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
			Command_received = 1;

		attempts++;
	}
#endif


	if(Command_received == 1)
	{
		userDelay(10, 0);

		update_Dev_Info();	// Update device info must be the first command sent

		// Update cartridge status even when waking from hibernate just in case cartridge was plugged in while in hibernate
		update_Cartridge_Status(gCartridge);
		if(gCartridge == 1)
			update_TCNumbers();

		update_Status(gStatus, gOperation);
		update_MonoCl();
		update_Alkalinity();

		update_Auto_Cal();
		update_Battery(gPumping);
		update_Error();
	}
	else
		gui32Error |= BT_FAILED;
}

//**************************************************************************
// Updates BT chip on current status and operation of device; Macro defines
// for availabe status and operation can be found in Bluetooth.h
// Parameters: Status; Idle, Calibration, or Test
//			   Operation; pre-check etc.
//**************************************************************************
void update_Status(uint8_t Status, uint8_t Operation)
{
	g_ulSSI0RXTO = 0;
	int counter = 0;
	uint8_t Command_received = 0;

	DEBUG_PRINT(
	if(gDiagnostics == 1)
		UARTprintf("Update Status\n");
	)

	// Save the status and operation in global variables so other functions know where we are
	gStatus = Status;
	gOperation = Operation;

	// Empty SSI FIFO
	while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
	{
	}

	SSIDataPut(SSI0_BASE, CURRENT_STATUS);
	SSIDataPut(SSI0_BASE, Status);
	SSIDataPut(SSI0_BASE, Operation);

	uint8_t attempts = 0;
	while(Command_received == 0 && attempts < 3)
	{
		counter = 0;
#ifdef MCU_ZXR
		GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
		SysCtlDelay(2000);
		GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
		SysCtlDelay(2000);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif


		while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
		{
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
			counter++;
		}
		if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
			Command_received = 1;

		attempts++;
	}

	g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

	if(Command_received == 0)
	{
		DEBUG_PRINT(UARTprintf("Update Status not working! ");)
		Reset_BT();
	}
}

//**************************************************************************
// Updates BT chip on what the most recent Calibration and Test numbers are
// Parameters: NONE
//**************************************************************************
void update_TCNumbers(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("update test and cal numbers \n");
	)

	g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
	int counter = 0;
	uint8_t Command_received = 0;

	uint16_t Test = FindTestNumber();
	uint16_t Cal = FindCalNumber();

//	// TODO: Remove this when not erasing memory on reset
//	uint8_t * No_of_tests = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_USAGE, 1);
//	if(*No_of_tests == 0xFF)
//		*No_of_tests = 0;
//	uint8_t Test = *No_of_tests;

//	if(Test == 0)	// TODO: Fix app to recognize empty memory
//		Test = 1;

	// Empty SSI FIFO
    while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
    {
    }

	SSIDataPut(SSI0_BASE, UPDATE_TEST_CAL_NUMBER);
	SSIDataPut(SSI0_BASE, Test);
	SSIDataPut(SSI0_BASE, Cal);

	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Updating Test %d and Cal %d \n", Test, Cal);
	)

	uint8_t attempts = 0;
    while(Command_received == 0 && attempts < 3)
    {
    	counter = 0;
#ifdef MCU_ZXR
    	GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
    	SysCtlDelay(2000);
    	GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
    	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
    	SysCtlDelay(2000);
    	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif


		while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
		{
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
			counter++;
		}
		if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
			Command_received = 1;

		attempts++;
    }

    if(Cal != 0 && Command_received == 1)	// If cal is 0 then BT won't read from memory
    {
#ifdef MCU_ZXR
    	while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == 0x00);	// Wait for BT to raise pin indicating its working
    	while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);	// Wait for BT to finish
#else
    	while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0x00);	// Wait for BT to raise pin indicating its working
    	while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish
#endif
    }

	g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

	if(Command_received == 0)
	{
		DEBUG_PRINT(UARTprintf("Update TC numbers not working! ");)
		Reset_BT();
	}
}

//**************************************************************************
// Tell BT chip to read test data from memory and display over BT
// Parameters: Test; Test number to have BT display
//**************************************************************************
void update_Test(int Test)
{
	if(gCartridge == 1)
	{
		update_TCNumbers();

		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("update test data \n");
		)

		if(Test > 0)
		{
			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
			uint32_t counter = 0;
			uint8_t Command_received = 0;

			// Empty SSI FIFO
			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
			{
			}


#ifdef MEMORY_V3
			//			uint16_t page = PAGE_TEST_INFO + PAGES_FOR_TEST * (Test - 1);	// Calculate which page data is on so BT knows where to read
			uint16_t page = ((PAGE_TEST_INFO + Test * PAGES_FOR_TEST) - PAGES_FOR_TEST); // Calculate which page data is on so BT knows where to read
			while(page > 508)
				page -= (511 - PAGE_TEST_INFO);
#else
			//			uint16_t page = PAGE_TEST_INFO + PAGES_FOR_TEST * (Test - 1);	// Calculate which page data is on so BT knows where to read
			uint16_t page = ((PAGE_TEST + Test * PAGES_FOR_TEST) - PAGES_FOR_TEST); // Calculate which page data is on so BT knows where to read
			while(page > 508)
				page -= (511 - PAGE_TEST);
#endif


			// Send command followed by 9-bit page number in little endian format
			SSIDataPut(SSI0_BASE, UPDATE_TEST);
			SSIDataPut(SSI0_BASE, (page & 0xFF));
			SSIDataPut(SSI0_BASE, ((page >> 8) & 0x01));

			uint8_t attempt = 0;
			while(Command_received == 0 && attempt < 3)
			{
				counter = 0;
#ifdef MCU_ZXR
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

				while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
				{
					SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
					counter++;
				}
				if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
					Command_received = 1;

				attempt++;
			}

			if(Command_received == 1)
			{
#ifdef MCU_ZXR
				while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == 0x00);	// Wait for BT to raise pin indicating its working
				counter = 0;
				while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish
#else
				while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0x00);	// Wait for BT to raise pin indicating its working
				counter = 0;
				while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish
#endif
				{
					SysCtlDelay(SysCtlClockGet()/3000);
					if(counter < 5000)
						counter++;
					if(counter == 5000)
					{
						counter++;
						DEBUG_PRINT(UARTprintf("Waiting for BT to read memory...\n");)
					}
				}
			}


			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

			if(Command_received == 0)
			{
				DEBUG_PRINT(UARTprintf("Update test not working! ");)
				Reset_BT();
			}
		}
	}

}

//**************************************************************************
// Tell BT chip to read cal data from memory and display over BT
// Parameters: Cal; Calibration number to have BT display
//**************************************************************************
void update_Cal(int Cal)
{
	if(gCartridge == 1)
	{
		update_TCNumbers();

		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("update cal data\n");
		)

		if(Cal > 0)
		{
			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
			uint32_t counter = 0;
			uint8_t Command_received = 0;

			// Empty SSI FIFO
			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
			{
			}

#ifdef MEMORY_V3
			//			uint16_t page = PAGE_CAL_INFO + PAGES_FOR_CAL * (Cal - 1);	// Calculate which page cal is on to tell BT where to read
			uint16_t page = (PAGE_CAL_INFO + Cal * PAGES_FOR_CAL) - PAGES_FOR_CAL;
			while(page > (PAGE_TEST_INFO - PAGES_FOR_CAL))
				page -= (PAGE_TEST_INFO - PAGE_CAL_INFO);
#else
			//			uint16_t page = PAGE_CAL_INFO + PAGES_FOR_CAL * (Cal - 1);	// Calculate which page cal is on to tell BT where to read
			uint16_t page = (PAGE_CAL + Cal * PAGES_FOR_CAL) - PAGES_FOR_CAL;
			while(page > (PAGE_TEST - PAGES_FOR_CAL))
				page -= (PAGE_TEST - PAGE_CAL);
#endif


			// Send command followed by 9-bit page number in little endian format
			SSIDataPut(SSI0_BASE, UPDATE_CAL);
			SSIDataPut(SSI0_BASE, (page & 0xFF));
			SSIDataPut(SSI0_BASE, ((page >> 8) & 0x01));

			uint8_t attempt = 0;
			while(Command_received == 0 && attempt < 3)
			{
				counter = 0;
#ifdef MCU_ZXR
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif


				while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
				{
					SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
					counter++;
				}
				if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
					Command_received = 1;

				attempt++;
			}

			if(Command_received == 1)
			{
#ifdef MCU_ZXR
				while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == 0x00);	// Wait for BT to raise pin indicating its working
				counter = 0;
				while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish
#else
				while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0x00);	// Wait for BT to raise pin indicating its working
				counter = 0;
				while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish
#endif
				{
					SysCtlDelay(SysCtlClockGet()/3000);
					if(counter < 5000)
						counter++;
					if(counter == 5000)
					{
						counter++;
						DEBUG_PRINT(UARTprintf("Waiting for BT to read memory...\n");)
					}
				}
			}

			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

			if(Command_received == 0)
			{
				DEBUG_PRINT(UARTprintf("Update calibration not working! ");)
				Reset_BT();
			}
		}
	}

}

////**************************************************************************
//// Tell BT chip to read device info from memory and display over BT
//// Parameters: NONE
////**************************************************************************
//void update_Dev_Info(void)
//{
//	if(gCartridge == 1)
//	{
//		if(gDiagnostics >= 1)
//			UARTprintf("Update Dev Info \n");
//
//		uint8_t * Mem_Test = MemoryRead(0, 20, 1);
//
//		if(*Mem_Test == 0xFF)
//		{
//			// Enter Device Information
//			unsigned char Manufacturer_Name[20] = "e-sens              ";
//			unsigned char Model_Number[20] = "ES-1000             ";
//			unsigned char Serial_Number[20] = "E200106";
//			unsigned char Hardware_Revision[20] = "Rev_B               ";
//			unsigned char Firmware_Revision[20] = "v0.7                ";
//			uint8_t Zero = 0;
//			uint16_t Sensor_Max_No_of_Tests = 100;
//			// check RSSI
//			// Mac address
//
//			// Write Device Information to Memory
//			MemoryWrite(PAGE_DEVICE_INFO, 0, 20, Manufacturer_Name);
//			MemoryWrite(PAGE_DEVICE_INFO, 20, 20, Model_Number);
//			MemoryWrite(PAGE_DEVICE_INFO, 40, 20, Serial_Number);
//			MemoryWrite(PAGE_DEVICE_INFO, 60, 20, Hardware_Revision);
//			MemoryWrite(PAGE_DEVICE_INFO, 80, 20, Firmware_Revision);
//			MemoryWrite(PAGE_DEVICE_INFO, 100, 1, &Zero);
//
//			// TODO: Remove this eventually
//			// Write Cartridge Information to Memory
//			MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_TESTS, 2, (uint8_t *) &Sensor_Max_No_of_Tests);
//			MemoryWrite(PAGE_CARTRIDGE_INFO, 24, 1, &Zero);
//
//			// check RSSI
//			// Mac address
//			if(gDiagnostics >= 1)
//				UARTprintf("update dev info \n");
//
//			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
//			int counter = 0;
//			uint8_t Command_received = 0;
//
//			// Empty SSI FIFO
//			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
//			{
//			}
//
//			SSIDataPut(SSI0_BASE, UPDATE_DEV_INFO);
//			SSIDataPut(SSI0_BASE, 0xFF);
//			SSIDataPut(SSI0_BASE, 0xFF);
//
//			while(Command_received == 0)
//			{
//				counter = 0;
//				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
//				SysCtlDelay(2000);
//				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
//
//				while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
//				{
//					SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
//					counter++;
//				}
//				if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
//					Command_received = 1;
//			}
//
//			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0x00);	// Wait for BT to raise pin indicating its working
//			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish
//
//			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
//		}
//	}
//
//}

#ifdef DEV_INFO_UNIQUE_COMMAND
//**************************************************************************
// Tell BT chip to read device info from memory and display over BT
// Parameters: NONE
// 11/15/2018: Modified to read dev info from EEPROM and send directly to BT
// 3/12/2020: Changed to sending serial number first so it can be used in
//	creating advertised bluetooth name, always send device info even if none
//	is found so BT knows to boot up
// 5/19/2022: Updating to look for a unique command, adding a way to recognize
// if the BT isn't updated and trying to handle it..
//**************************************************************************
void update_Dev_Info(void)
{
	if(gDiagnostics >= 1)
		UARTprintf("Update Dev Info \n");

	// Check if device has programmed in device information, if it does write to memory for BT
	uint8_t Transmit_buffer[20];
	EEPROMRead((uint32_t *) &Transmit_buffer, OFFEST_MANUFACTURER_NAME, 20);
//	if(Transmit_buffer[0] != 0xFF)
//	{
		if(gDiagnostics >= 1)
			UARTprintf("Found device info, sending to BT chip!\n");

		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
		int counter = 0;
		uint8_t Command_received = 0;

		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, UPDATE_DEV_INFO);
		SSIDataPut(SSI0_BASE, 0xFF);
		SSIDataPut(SSI0_BASE, 0xFF);

		uint8_t attempts = 0;
		uint8_t interrupts = 0;
		uint8_t firmware_mismatch = 0;

		// BT will raise pin when getting this command, check for that as well as the generic Tiva request command so we know the correct command was received
		while((Command_received == 0 && attempts < 10) && firmware_mismatch == 0) // Increase attempts to greater than
		{
			counter = 0;
//			UARTprintf("Pinging BT...\n");
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}


//			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
//				Command_received = 1;

			// Because we are waiting for UPDATE_DEV_INFO there will be 2 transactions between CPUs
			// first will receive Request Tiva, second will receive Update Dev Info back, this is to make
			// sure we are in the dev info case in the bluetooth and not lost somewhere else
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == UPDATE_DEV_INFO)
				Command_received = 1;

			// If the Tiva is updated but the BT is not the BT will setup the 20 byte transaction, then wait to be pinged
			// if that happens I can recognize it here, interrupt will fire an incorrect number of times (20 bytes total, 3 at a time = 6.66 interrupts)
			// count the interrupts, if they get too high then BT isn't updated
			interrupts += g_ulSSI0RXTO;
			if(interrupts >= 7)
			{
				UARTprintf("BT triggering a lot of data, seems it needs to be updated, resetting and using old protocol!\n");
				firmware_mismatch = 1;
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT, raise to stop BT where it's at
			}

			attempts++;

			g_ulSSI0RXTO == 0;	// Have to reset flag otherwise the while loop above will be skipped after first interrupt occurs

		}

		if(firmware_mismatch)
		{
			UARTprintf("Resetting BT Chip!\n");
			userDelay(5, 0);

			int counter = 0;
			g_ui32DataRx0[0] = 0;

			// Pulse BT reset signal
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
			SysCtlDelay(SysCtlClockGet()/3000);
			g_ulSSI0RXTO == 0;	// Have to reset flag otherwise the while loop above will be skipped after first interrupt occurs
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);

			SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working

			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to read memory at startup
			{
				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms, kept short to not significantly slow polling
				counter++;

				if(counter == 5000)
				{
					if(gDiagnostics >= 1)
						UARTprintf("Waited 5 seconds... Resetting BT again \n");
					counter = 0;
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // Reset BT
					SysCtlDelay(10000);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00); // Reset BT

					SysCtlDelay(SysCtlClockGet()/300);	// Give time for BT chip to reset and raise line to indicate it's working
				}
			}

			// Pulse BT chip and make sure it responds, don't load anything into SSI buffer in case there is already something there it will be cleared out
			Command_received = 0;
			attempts = 0;
			while(Command_received == 0 && attempts < 3)
			{
				counter = 0;
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // Tiva Request for SPI
				SysCtlDelay(2000);
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // Tiva Request for SPI

				while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
				{
					SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
					counter++;
				}
				if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
					Command_received = 1;

				attempts++;
			}

			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
			counter = 0;
			Command_received = 0;

			// Empty SSI FIFO
			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
			{
			}

			SSIDataPut(SSI0_BASE, UPDATE_DEV_INFO);
			SSIDataPut(SSI0_BASE, 0xFF);
			SSIDataPut(SSI0_BASE, 0xFF);

			attempts = 0;

			// BT will raise pin when getting this command, check for that as well as the generic Tiva request command so we know the correct command was received
			while((Command_received == 0 && attempts < 5))
			{
				counter = 0;
	//			UARTprintf("Pinging BT...\n");
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA

				while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
				{
					SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
					counter++;
				}
				if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
					Command_received = 1;

				attempts++;
			}
		}

		if(Command_received == 1)
		{
			// Disable SSI interrupt because this is a unique SSI transaction
			// and we are not collecting data, only sending
			IntDisable(INT_SSI0);

			uint8_t i;

			//		// Send Manufacturer name
			//		for(i = 0; i < 8; i++)
			//			SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA

			// Send Serial number
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_SERIAL_NUMBER, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send Manufacturer name
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFEST_MANUFACTURER_NAME, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send Model number
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_MODEL_NUMBER, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send hardware revision
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_HARDWARE_REV, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send firmware revision
			//		EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_FIRMWARE_REV, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) FIRMWARE_REVISION[i]);
			//			SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			counter = 0;
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				if(counter < 5000)
					counter++;
				if(counter == 5000)
				{
					counter++;
					if(gDiagnostics >= 1)
						UARTprintf("Waiting for BT to receive all dev info...\n");
				}
			}

			if(gDiagnostics >= 1)
				UARTprintf("Sent all dev info... \n");

			// Empty SSI FIFO don't want any of the data received while sending device info
			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
			{
			}

			SSIIntClear(SSI0_BASE, SSI_RXTO);
			IntEnable(INT_SSI0);

			//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0x00);	// Wait for BT to raise pin indicating its working
			//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish
		}

		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

		if(Command_received == 0)
		{
			UARTprintf("Update device info not working! ");
			Reset_BT();
		}
//	}
//	else
//		if(gDiagnostics >= 1)
//			UARTprintf("Didn't find device info, not updating BT chip!\n");
}
#else
//**************************************************************************
// Tell BT chip to read device info from memory and display over BT
// Parameters: NONE
// 11/15/2018: Modified to read dev info from EEPROM and send directly to BT
// 3/12/2020: Changed to sending serial number first so it can be used in
//	creating advertised bluetooth name, always send device info even if none
//	is found so BT knows to boot up
//**************************************************************************
void update_Dev_Info(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Update Dev Info \n");
	)

	// Check if device has programmed in device information, if it does write to memory for BT
	uint8_t Transmit_buffer[20];
	EEPROMRead((uint32_t *) &Transmit_buffer, OFFEST_MANUFACTURER_NAME, 20);
//	if(Transmit_buffer[0] != 0xFF)
//	{
	DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Found device info, sending to BT chip!\n");
	)

		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
		int counter = 0;
		uint8_t Command_received = 0;

		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, UPDATE_DEV_INFO);
		SSIDataPut(SSI0_BASE, 0xFF);
		SSIDataPut(SSI0_BASE, 0xFF);

		uint8_t attempts = 0;

		// BT will raise pin when getting this command, check for that as well as the generic Tiva request command so we know the correct command was received
		while((Command_received == 0 && attempts < 3))
		{
			counter = 0;
//			UARTprintf("Pinging BT...\n");
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

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
			// and we are not collecting data, only sending
			IntDisable(INT_SSI0);

			uint8_t i;

			//		// Send Manufacturer name
			//		for(i = 0; i < 8; i++)
			//			SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

			// Send Serial number
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_SERIAL_NUMBER, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send Manufacturer name
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFEST_MANUFACTURER_NAME, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send Model number
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_MODEL_NUMBER, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send hardware revision
			EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_HARDWARE_REV, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			// Send firmware revision
			//		EEPROMRead((uint32_t *) &Transmit_buffer, OFFSET_FIRMWARE_REV, 20);
			for(i = 0; i < 20; i++)
				SSIDataPut(SSI0_BASE, (uint32_t) FIRMWARE_REVISION[i]);
			//			SSIDataPut(SSI0_BASE, (uint32_t) Transmit_buffer[i]);

			counter = 0;
#ifdef MCU_ZXR
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish
#else
			while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish
#endif
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				if(counter < 5000)
					counter++;
				if(counter == 5000)
				{
					counter++;
					DEBUG_PRINT(
					if(gDiagnostics >= 1)
						UARTprintf("Waiting for BT to receive all dev info...\n");
					)
				}
			}

			DEBUG_PRINT(
			if(gDiagnostics >= 1)
				UARTprintf("Sent all dev info... \n");
			)

			// Empty SSI FIFO don't want any of the data received while sending device info
			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
			{
			}

			SSIIntClear(SSI0_BASE, SSI_RXTO);
			IntEnable(INT_SSI0);

			//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0x00);	// Wait for BT to raise pin indicating its working
			//		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6);	// Wait for BT to finish
		}

		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

		if(Command_received == 0)
		{
			DEBUG_PRINT(UARTprintf("Update device info not working! ");)
			Reset_BT();
		}
//	}
//	else
//		if(gDiagnostics >= 1)
//			UARTprintf("Didn't find device info, not updating BT chip!\n");
}
#endif

//**************************************************************************
// Updates BT chip on error,
// Parameters: NONE
//**************************************************************************
void update_Error(void)
{
	g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
	uint32_t counter = 0;
	uint8_t Command_received = 0;

	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Sending error to BT\n");
	)

	// Empty SSI FIFO
    while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
    {
    }

//    UARTprintf("Send: ");
//    UARTprintf("%d,%d,%d,%d\n", (gui32Error >> 0) & 0x000000FF, (gui32Error >> 8) & 0x000000FF, (gui32Error >> 16) & 0x000000FF, (gui32Error >> 24) & 0x000000FF);
	SSIDataPut(SSI0_BASE, ERROR_CODE_1);
	SSIDataPut(SSI0_BASE, (gui32Error >> 0) & 0x000000FF);
	SSIDataPut(SSI0_BASE, (gui32Error >> 8) & 0x000000FF);
	SSIDataPut(SSI0_BASE, ERROR_CODE_2);
	SSIDataPut(SSI0_BASE, (gui32Error >> 16) & 0x000000FF);
	SSIDataPut(SSI0_BASE, (gui32Error >> 24) & 0x000000FF);

	uint8_t i;
	uint8_t attempts = 0;
	for(i = 0; i < 2; i++)	// Repeat twice to send all 6 bytes of data (2 commands, 4 for error code)
	{
		Command_received = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif


			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

		if(Command_received == 0)
		{
			DEBUG_PRINT(UARTprintf("Update error not working! ");)
			Reset_BT();
			break;	// Break out of for loop if BT did not respond
		}
	}
}

//**************************************************************************
// Updates BT chip on current battery level and charging state
// Parameters: NONE
//**************************************************************************
void update_Battery(uint8_t Pumping)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Updating battery status \n");
	)

	int counter = 0;
	uint8_t Command_received = 0;

	g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
	uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
	uint8_t Charging;
#ifdef MCU_ZXR
	if(gBoard >= V7)
		Charging = GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN);	// Checks that box is plugged in
	else
		Charging = GPIOPinRead(IO_BAT_CHARGING_BASE, IO_BAT_CHARGING_PIN);	// Checks that box is charging
#else
	if(gBoard >= V7)
		Charging = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4);	// Checks that box is plugged in
	else
		Charging = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0);	// Checks that box is charging
#endif

	if(Charging != 0)
		Charging = 1;

	// Turn off all charging LEDs then turn on required ones
//	SetLED(RED_CHARGE | RED_CHARGE_BLINK | GREEN_CHARGE | YELLOW_CHARGE, 0);
	uint16_t LED_State = 0;

	if(gBoard >= V6_2 && HibernateIsActive())
	{
		// Don't allow device to hibernate when plugged in
		if(Charging == 0 && g_state == STATE_IDLE)
			TimerDisable(WTIMER0_BASE, TIMER_A);
#ifdef MCU_ZXR
		else if(g_state == STATE_IDLE && GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN) == IO_POWER_GOOD_B_PIN)	// Else if only useful for digital boards before V7
#else
		else if(g_state == STATE_IDLE && GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4) == GPIO_PIN_4)	// Else if only useful for digital boards before V7
#endif
		{
			uint64_t Timer_begin = TimerValueGet64(WTIMER0_BASE);
			if(Timer_begin == TimerValueGet64(WTIMER0_BASE))
			{
				TimerLoadSet64(WTIMER0_BASE, (uint64_t) SysCtlClockGet() * HIBERNATE_TIMEOUT); // Set timer for 5 minutes, if timer expires hibernate device
				TimerEnable(WTIMER0_BASE, TIMER_A);
			}
		}
	}

	if(Charging == 0)	// If we are charging set yellow light unless above 95% then set green light
	{
		if(Battery_Percent > 95)
			LED_State |= GREEN_CHARGE; //SetLED(GREEN_CHARGE, 1);
		else
			LED_State |= YELLOW_CHARGE; //SetLED(YELLOW_CHARGE, 1);
	}
	else if(Battery_Percent < 20)	// If battery is less than 20% and NOT charging blink red light
	{
		if(gBoard >= V6_2)
			LED_State |= RED_CHARGE_BLINK; //SetLED(RED_CHARGE_BLINK, 1);
		else
			LED_State |= RED_CHARGE; //SetLED(RED_CHARGE, 1);
	}
	else if(gBoard >= V6_2)	// If battery is not charging but plugged in (full) leave on green light // Unnecessary for digital V7 and onward
#ifdef MCU_ZXR
		if(GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN) == 0)	// Power good _B
			LED_State |= GREEN_CHARGE; //SetLED(GREEN_CHARGE, 1);
#else
		if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4) == 0)	// Power good _B
			LED_State |= GREEN_CHARGE; //SetLED(GREEN_CHARGE, 1);
#endif


	if((gLED_State & (RED_CHARGE | GREEN_CHARGE | YELLOW_CHARGE | RED_CHARGE_BLINK)) != LED_State)
	{
		SetLED(RED_CHARGE | RED_CHARGE_BLINK | GREEN_CHARGE | YELLOW_CHARGE, 0);
		SetLED(LED_State, 1);
	}

	if(Pumping == 0)
	{
		SSIDataPut(SSI0_BASE, BATTERY_STATUS);
		SSIDataPut(SSI0_BASE, Battery_Percent);
		SSIDataPut(SSI0_BASE, Charging);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif


			while(g_ulSSI0RXTO == 0 && counter < 1000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/3000);	// Delay 1 ms
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

		if(Command_received == 0)
		{
			DEBUG_PRINT(UARTprintf("Update battery not working! ");)
			Reset_BT();
		}
	}
	else
		gPumping = 2;
}

//**************************************************************************
// Sends Auto Cal data read from on chip memory to BT chip when BT chip is
// reset because of writing auto cal
// Parameters: Auto_Cal; 1st byte: 0 or 1
//						 Next 3 bytes: H:M:S
//**************************************************************************
void update_Auto_Cal(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("update auto cal data \n");
	)

	g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
	int counter = 0;
	uint8_t Command_received = 0;

    uint32_t Auto_Cal;

	// Read auto cal data from on-chip memory and send to BT chip
	EEPROMRead(&Auto_Cal, OFFSET_AUTO_CAL, 4);

	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// We did NOT wake from hibernate, this is initial boot sequence
	{
		if((Auto_Cal & 0xFF) == 1 || (Auto_Cal & 0xFF) == 0)	// Check that the first byte is either 0 or 1 to know it has been written to
		{
			// Empty SSI FIFO
			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
			{
			}

			// Send command followed by first two bytes of Auto_Cal data
			SSIDataPut(SSI0_BASE, UPDATE_AUTO_CAL_1);
			SSIDataPut(SSI0_BASE, (Auto_Cal & 0xFF));
			SSIDataPut(SSI0_BASE, ((Auto_Cal >> 8) & 0xFF));

			uint8_t attempts = 0;
			while(Command_received == 0 && attempts < 3)
			{
				counter = 0;
#ifdef MCU_ZXR
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

				while(g_ulSSI0RXTO == 0 & counter < 10000)	// Wait for BT to reply or 1 second to pass
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
				SysCtlDelay(SysCtlClockGet()/3000); // Delay 1 ms
				g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

				counter = 0;
				Command_received = 0;

				// Empty SSI FIFO
				while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
				{
				}

				// Send command followed by last two bytes of Auto_Cal data
				SSIDataPut(SSI0_BASE, UPDATE_AUTO_CAL_2);
				SSIDataPut(SSI0_BASE, ((Auto_Cal >> 16) & 0xFF));
				SSIDataPut(SSI0_BASE, ((Auto_Cal >> 24) & 0xFF));

				attempts = 0;
				while(Command_received == 0 && attempts < 3)
				{
					counter = 0;
#ifdef MCU_ZXR
					GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
					SysCtlDelay(2000);
					GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
					SysCtlDelay(2000);
					GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

					while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
					{
						SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
						counter++;
					}
					if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
						Command_received = 1;

					attempts++;
				}
			}

			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

			if(Command_received == 0)
			{
				DEBUG_PRINT(UARTprintf("Update auto cal not working! ");)
				Reset_BT();
			}
		}
	}
}

//**************************************************************************
// Reads from on-chip EEPROM if Monochloramine is activated and sends this
// status to BT chip
//**************************************************************************
void update_MonoCl(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("update mono cl setting \n");
	)

	g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
	int counter = 0;
	uint8_t Command_received = 0;
	uint32_t holder;

	EEPROMRead(&holder, OFFSET_MONO_CL_CYCLE, 4);	// Save data into holder variable, otherwise this freezes if memory hasn't been written before
	if(holder != 0xFFFFFFFF)
		g_MonoCl = holder & 0x01;

	EEPROMRead(&holder, OFFSET_FREE_CL_CYCLE, 4);	// Save data into holder variable, otherwise this freezes if memory hasn't been written before
	if(holder != 0xFFFFFFFF)
		g_FreeCl = holder & 0x01;

	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// We did NOT wake from hibernate, this is initial boot sequence
	{
		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

//		UARTprintf("Free: %d\n", g_FreeCl);
//		UARTprintf("Mono: %d\n", g_MonoCl);

		// Send command followed by Monochloramine setting followed by Free Chlorine setting
		SSIDataPut(SSI0_BASE, MONO_CL);
		SSIDataPut(SSI0_BASE, g_MonoCl);
		SSIDataPut(SSI0_BASE, g_FreeCl);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

		if(Command_received == 0)
		{
			DEBUG_PRINT(UARTprintf("Update monochloramine not working! ");)
			Reset_BT();
		}
	}
}

//**************************************************************************
// Reads from on-chip EEPROM if Alkalinity is activated and sends this
// status to BT chip
//**************************************************************************
void update_Alkalinity(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("update Alkalinity setting \n");
	)

	g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side
	int counter = 0;
	uint8_t Command_received = 0;
	uint32_t holder;

	EEPROMRead(&holder, OFFSET_ALKALINITY_CYCLE, 4);	// Save data into holder variable, otherwise this freezes if memory hasn't been written before
	if(holder != 0xFFFFFFFF)
		g_Alkalinity = holder & 0x01;

	if(HibernateIntStatus(1) != HIBERNATE_INT_PIN_WAKE)	// We did NOT wake from hibernate, this is initial boot sequence
	{
		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		// Send command followed by 9-bit page number in little endian format
		SSIDataPut(SSI0_BASE, ALKALINITY);
		SSIDataPut(SSI0_BASE, g_Alkalinity);
		SSIDataPut(SSI0_BASE, 0);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag because this was initiated on Tiva side

		if(Command_received == 0)
		{
			DEBUG_PRINT(UARTprintf("Update alkalinity not working! ");)
			Reset_BT();
		}
	}
}

//**************************************************************************
// Updates BT chip on whether there is a cartridge connected or not
// Parameters: Connected: 0 if not connected, 1 if connected
//**************************************************************************
void update_Cartridge_Status(uint8_t Connected)
{
	g_ulSSI0RXTO = 0;
	int counter = 0;
	uint8_t Command_received = 0;

	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Updating cartridge status \n");
	)

	// Empty SSI FIFO
	while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
	{
	}

	SSIDataPut(SSI0_BASE, CARTRIDGE_CONNECTED);
	SSIDataPut(SSI0_BASE, Connected);
	SSIDataPut(SSI0_BASE, 0xFF);

	uint8_t attempts = 0;
	while(Command_received == 0 && attempts < 3)
	{
		counter = 0;
#ifdef MCU_ZXR
		GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
		SysCtlDelay(2000);
		GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
		SysCtlDelay(2000);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

		while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
		{
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
			counter++;
		}
		if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
			Command_received = 1;

		attempts++;
	}

	g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

	if(Command_received == 1)
	{
		SysCtlDelay(SysCtlClockGet()/300);
		//	while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == 0x00);	// Wait for BT to raise pin indicating its working
#ifdef MCU_ZXR
		while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN)	// Wait for BT to finish
#else
		while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)	// Wait for BT to finish
#endif
		{
			if(gDiagnostics >= 2)
			{
				DEBUG_PRINT(UARTprintf("Update cartridge status... waiting for BT! \n");)
			}
		}
	}
	else
	{
		DEBUG_PRINT(UARTprintf("Update cartridge status not working! ");)
		Reset_BT();
	}


//	if(gDiagnostics >= 1)
//		UARTprintf("Exiting update_Cartridge_Status! \n");
}

//**************************************************************************
// Tells BT chip if we are or are not using the I2C bus currently, also
// lets the BT chip know if the memory is currently connected
// Parameters: State; 1 for using bus, 0 for not using bus
//			   Memory; 1 if memory is available, 0 if not available
//**************************************************************************
void update_TivaI2C(uint8_t State)//, uint8_t Memory)
{
	if(gBoard >= V7)
	{
		// Can't print anything inside this function because it is called too often and slows MCU below an acceptable level
#ifdef MCU_ZXR
		// Disable battery interrupt here because it is possible for interrupt to fire while BT and Tiva are in middle of sharing and it will freeze
		if(State == 1)
			IntDisable(INT_BATTERY_BASE);

		if(gui8MemConnected)
			GPIOPinWrite(IO_I2C_USED_BY_TIVA_BASE, IO_I2C_USED_BY_TIVA_PIN, (IO_I2C_USED_BY_TIVA_PIN * State)); // I2C used by TIVA

		// Enable battery interrupt here because it is possible for interrupt to fire while BT and Tiva are in middle of sharing and it will freeze
		if(State == 0)
			IntEnable(INT_BATTERY_BASE);
#else
		// Disable battery interrupt here because it is possible for interrupt to fire while BT and Tiva are in middle of sharing and it will freeze
		if(State == 1)
			IntDisable(INT_GPIOE);

		if(gui8MemConnected)
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, (GPIO_PIN_0 * State)); // I2C used by TIVA

		// Enable battery interrupt here because it is possible for interrupt to fire while BT and Tiva are in middle of sharing and it will freeze
		if(State == 0)
			IntEnable(INT_GPIOE);
#endif
	}
	else if(gBoard >= V6_2 && gui8MemConnected)
	{
		DEBUG_PRINT(
		if(gDiagnostics >= 2 && State == 1)
			UARTprintf("Telling BT I2C in use!\n");
		else if(gDiagnostics >= 2 && State == 0)
			UARTprintf("Telling BT I2C is free!\n");
		)


		g_ulSSI0RXTO = 0;
		int counter = 0;
		uint8_t Command_received = 0;

		while(Command_received == 0)	// Second loop to make sure this worked, need to update BT on this parameter if reset is required
		{
			// Empty SSI FIFO
			while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
			{
			}

			SSIDataPut(SSI0_BASE, USING_I2C);
			SSIDataPut(SSI0_BASE, State);
			SSIDataPut(SSI0_BASE, 0);

			uint8_t attempts = 0;
			while(Command_received == 0 && attempts < 3)
			{
				counter = 0;
#ifdef MCU_ZXR
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

				while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
				{
					SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
					counter++;
				}
				if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
					Command_received = 1;

				attempts++;
			}

			g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

			if(Command_received == 0)
			{
				DEBUG_PRINT(UARTprintf("Update Tiva I2C not working! ");)
				Reset_BT();
			}
		}
	}
}

//**************************************************************************
// Tells BT chip there are new cartridge temp extremes available and to
// update the characteristic to reflect these new numbers
//**************************************************************************
void update_CartTemp(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 2)
		UARTprintf("Telling BT new cartridge temp data needs to be updated!\n");
	)

	g_ulSSI0RXTO = 0;
	int counter = 0;
	uint8_t Command_received = 0;

	while(Command_received == 0)	// Second loop to make sure this worked, need to update BT on this parameter if reset is required
	{
		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, UPDATE_CART_TEMP);
		SSIDataPut(SSI0_BASE, 0);
		SSIDataPut(SSI0_BASE, 0);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif


			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

		if(Command_received == 0)
		{
			DEBUG_PRINT(UARTprintf("Update Cartridge Temp not working! ");)
			Reset_BT();
		}
	}

}

//**************************************************************************
// Tells BT chip to reset tiva by holding button until Tiva resets, this will
// cause Tiva to reset BT so there is no need for BT to stop holding the line
// Parameters: NONE
//**************************************************************************
void BT_ResetTiva(void)
{
	DEBUG_PRINT(
	if(gDiagnostics >= 1)
		UARTprintf("Asking BT to reset Tiva!\n");
	)

	g_ulSSI0RXTO = 0;
	int counter = 0;
	uint8_t Command_received = 0;

	while(Command_received == 0)	// Second loop to make sure this worked, need to update BT on this parameter if reset is required
	{
		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, RESET_TIVA);
		SSIDataPut(SSI0_BASE, 0);
		SSIDataPut(SSI0_BASE, 0);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
#ifdef MCU_ZXR
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA
#else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA
#endif

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

		if(Command_received == 0)
		{
			DEBUG_PRINT(UARTprintf("BT not responding to Reset Tiva command!\n");)
			Reset_BT();
		}
	}

//	// Delay to lock up Tiva until reset happens
//	userDelay(10000, 1);
}

#ifndef OAD
//**************************************************************************
// Tells BT chip to test flash memory by reading device ID from memory and
// passing it to Tiva
// Parameters: NONE
//**************************************************************************
uint8_t BT_TestFlash(void)
{
	if(gDiagnostics >= 1)
		UARTprintf("Asking BT to test flash memory!\n");

	g_ulSSI0RXTO = 0;
	int counter = 0;
	uint8_t Command_received = 0;

	while(Command_received == 0)	// Second loop to make sure this worked, need to update BT on this parameter if reset is required
	{
		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, TEST_FLASH);
		SSIDataPut(SSI0_BASE, 0);
		SSIDataPut(SSI0_BASE, 0);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

		if(Command_received == 0)
		{
			UARTprintf("BT not responding to test flash memory command!\n");
			Reset_BT();
		}
	}

	while(g_ulSSI0RXTO == 0 && counter < 10000)
	{
		counter++;
		SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
	}

	if(g_ulSSI0RXTO == 0 && counter == 10000)
	{
		UARTprintf("Waiting BT to send flash ID timed out!\n");
		return 0;
	}
	else if(g_ui32DataRx0[0] == TEST_FLASH && g_ui32DataRx0[2] == 0x12)
	{
		UARTprintf("BT responded to request for flash ID with: %d, %d, %d\n", g_ui32DataRx0[0], g_ui32DataRx0[1], g_ui32DataRx0[2]);
		return 1;
	}
	else
		UARTprintf("BT responded to request for flash ID with: %d, %d, %d\n", g_ui32DataRx0[0], g_ui32DataRx0[1], g_ui32DataRx0[2]);

	return 0;
}

//**************************************************************************
// Tells BT chip to test cartridge memory by reading manufacturer from memory
// and passing it to Tiva, should be reading "e-sens_
// Parameters: NONE
// Returns: Bitwise 0,0,0,0,0,I2C Used by BT, I2C Used by Tiva, Cart memory
//**************************************************************************
uint8_t BT_TestCartMem(void)
{
	if(gDiagnostics >= 1)
		UARTprintf("Asking BT to test cartridge memory!\n");

//	IntDisable(INT_GPIOE);

	g_ulSSI0RXTO = 0;
	int counter = 0;
	uint8_t Command_received = 0;

	IntDisable(INT_GPIOE);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, (GPIO_PIN_0)); // I2C used by TIVA

	while(Command_received == 0)	// Second loop to make sure this worked, need to update BT on this parameter if reset is required
	{
		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, TEST_CART_MEM);
		SSIDataPut(SSI0_BASE, 0);
		SSIDataPut(SSI0_BASE, 0);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

		if(Command_received == 0)
		{
			UARTprintf("BT not responding to test cartridge memory command!\n");
			Reset_BT();
		}
	}

	counter = 0;
	uint8_t I2C_Used_BT = 0;
	userDelay(1, 1);	// Delay 1 ms to give BT time to turn on I2C used by BT signal then wait for Tiva to finish
	while(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && counter < 10000)
//	while(counter < 10000 && g_ulSSI0RXTO == 0)
	{
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6)
			I2C_Used_BT = 4;

		counter++;
		SysCtlDelay(SysCtlClockGet()/30000);

		if(counter == 2000)
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0); // I2C used by TIVA
	}
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0); // I2C used by TIVA
//	IntEnable(INT_GPIOE);

	UARTprintf("Counter: %d\n", counter);
	UARTprintf("gRXCount: %d\n", g_ulSSI0RXTO);

	uint8_t return_value = I2C_Used_BT;
	if(counter >= 2000 && counter < 10000)
		return_value |= 2;	// Check if I2C used by Tiva works

	if(g_ulSSI0RXTO == 0 && counter == 10000)
		return return_value;
	else if(g_ui32DataRx0[0] == TEST_CART_MEM && g_ui32DataRx0[1] == 0x65 && g_ui32DataRx0[2] == 0x2D)	// Checking that it read "e-" the first 2 characters for e-sens
	{
		UARTprintf("BT read from cartridge memory: %c, %c\n", g_ui32DataRx0[1], g_ui32DataRx0[2]);
		return_value |= 1;
		return return_value;
	}
	else
		UARTprintf("BT read from cartridge memory: %c, %c\n", g_ui32DataRx0[1], g_ui32DataRx0[2]);

	IntEnable(INT_GPIOE);

	return return_value;
}

//**************************************************************************
// Tells BT chip to test hibernate by having BT try to talk to Tiva after
// its gone to sleep and making sure it doesn't respond, then waking tiva
// and trying to talk to it again
// Parameters: NONE
//**************************************************************************
uint8_t BT_TestHibernate(uint8_t Test)
{
	if(gDiagnostics >= 1)
		UARTprintf("Asking BT to test hibernate!\n");

	if(Test)
		UARTprintf("Testing Hibernate, device must be unplugged, if it's not unplug and reset!\n");
	else
		UARTprintf("Requesting hibernate test result from BT!\n");
	g_ulSSI0RXTO = 0;
	int counter = 0;
	uint8_t Command_received = 0;

	while(Command_received == 0)	// Second loop to make sure this worked, need to update BT on this parameter if reset is required
	{
		// Empty SSI FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]))
		{
		}

		SSIDataPut(SSI0_BASE, TEST_HIBERNATE);
		SSIDataPut(SSI0_BASE, Test);
		SSIDataPut(SSI0_BASE, 0);

		uint8_t attempts = 0;
		while(Command_received == 0 && attempts < 3)
		{
			counter = 0;
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7); // I2C used by TIVA
			SysCtlDelay(2000);
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA

			while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
			{
				SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
				counter++;
			}
			if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
				Command_received = 1;

			attempts++;
		}

		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

		if(Command_received == 0)
		{
			UARTprintf("BT not responding to Test Hibernate command!\n");
			Reset_BT();
		}
	}

	// Puts device into hibernate after sending command
	if(Test)
		HibernateTimeoutIntHandler();
	else
	{
		counter = 0;
		while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
		{
			SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
			counter++;
		}

		g_ulSSI0RXTO = 0;	// Reset SSI interrupt flag

		if(g_ui32DataRx0[0] == TEST_HIBERNATE && g_ui32DataRx0[1] == 2)
			return g_ui32DataRx0[1];

		if(counter == 10000)
			UARTprintf("BT didn't respond with hibernate test results!\n");
	}

	return 0;
}
#endif
