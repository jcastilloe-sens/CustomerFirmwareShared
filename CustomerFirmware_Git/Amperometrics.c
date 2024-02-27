//*****************************************************************************
//
// Amperometrics.c - Functions used by e-SENS firmware to control amperometric
// array
// Includes functions to clean array
//
// Author: Jason Castillo
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "driverlib/eeprom.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "Amperometrics.h"
#include "Bluetooth.h"
#include "Components.h"
#include "Helper.h"
#include "Communication.h"
#include "Steppers.h"
#include "main.h"

#ifdef MCU_ZXR
#include "PinMap.h"
#endif

////**************************************************************************
//// Calculates the cumulative current flowing through all working electrodes
//// Parameters: 	NONE
//// Outputs:		current; calculated current in mA
////**************************************************************************
//float FindCurrent(void)
//{
//	// Set SPI communication to capture on falling edge of clock (ADC captures on rising edge)
//	SSIDisable(SSI1_BASE);
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
//	SSIEnable(SSI1_BASE);
//
//	SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0x88, 0x00, 0x00);	// Request Read from DAC
//	SPISend(SSI1_BASE, 1, DAC1_CS_B, 0, 3, 0x00, 0x00, 0x20);	// Set NOP bit and clock out bytes
//
//	uint32_t a, b;
//	SSIDataGet(SSI1_BASE,&a);
//	SSIDataGet(SSI1_BASE,&a);
//	SSIDataGet(SSI1_BASE,&b);
//
//	int16_t DAC_Input = 0 | (a << 8) | b;
//
//	float DAC_Voltage = (float) DAC_Input / 32768 * 3000;	// Set dac voltage
//
//	// Each pin is controlled by bit in register, read register to know state of all pins
//	uint8_t ui8PinValues = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
//
//	float Resistance;
//	if((ui8PinValues & 0x01))
//	{
//		Resistance = 99502.5; // 486853
//	}
//	else
//		Resistance = 20000000;
//
//	float current;
//	// Read the voltage at the ADC
//	float ADC_Voltage = ADCRead(0, ADC4_CS_B);
////	ADC_Voltage = ADCRead(0, ADC4_CS_B);
////	UARTprintf("ADC voltage is %d \n", (int) ADC_Voltage);
//
//	if(ADC_Voltage > 2998)
//	{
//		UARTprintf("WARNING! ADC Voltage positively saturated, current reading not reliable.\n");
//		UARTprintf("ADC voltage is %d \n", (int) ADC_Voltage);
//	}
//
//
//	if(ADC_Voltage < 1)
//	{
//		UARTprintf("WARNING! ADC Voltage negatively saturated, current reading not reliable.\n");
//		UARTprintf("ADC voltage is %d \n", (int) ADC_Voltage);
//	}
//
//
//	// current = (-V_DAC_SET + 2*V_ADC_IN - V_ref_3)/R_f // Equation used
//
//	current = ((2 * ADC_Voltage) - DAC_Voltage - 3000) / Resistance; // R_f = Two 10 MOhms resistors in series
//
//	return current;
//}

//**************************************************************************
// Calculates the cumulative current flowing through all working electrodes
// Parameters: 	ADC_Voltage; Voltage at ADC
//				DAC_Voltage; Voltage set by DAC
//				High_current_switch; what state the switches that control R are in
// Outputs:		current; calculated current in nA
//**************************************************************************
float CalculateCurrent(float ADC_Voltage, float DAC_Voltage, uint8_t High_current_switch)
{
	float Resistance;
	if(High_current_switch == 1)
	{
		if(gABoard == AV7_3)
			Resistance = 178394.45;
		else if(gABoard >= AV6_4)
			Resistance = 7497.2; // 486853
		else
			Resistance = 99502.5; // 486853
	}
	else if(High_current_switch == 2)
		Resistance = 4063745.02;
	else
		Resistance = 20000000;

	float current;

//	if(ADC_Voltage > 2999)
//	{
//		UARTprintf("WARNING! ADC Voltage positively saturated, current reading not reliable. \t");
//	}
//
//
//	if(ADC_Voltage < 1)
//	{
//		UARTprintf("WARNING! ADC Voltage negatively saturated, current reading not reliable. \t");
//	}


	// current = (-V_DAC_SET + 2*V_ADC_IN - V_ref_3)/R_f // Equation used

	current = ((2 * ADC_Voltage) - DAC_Voltage - 3000) / Resistance; // R_f = Two 10 MOhms resistors in series

	return current * 1000000;
}

//**************************************************************************
// Calculates the cumulative current flowing through all working electrodes
// Parameters: 	ADC_Voltage; Voltage at ADC
//				DAC_Voltage; Voltage set by DAC
//				High_current_switch; what state the switches that control R are in
// Outputs:		current; calculated current in nA
//**************************************************************************
float CalculateCurrentCalibrated(float ADC_Voltage, float DAC_Voltage, uint8_t High_current_switch)
{
	float Resistance;
	float ResistorRatio;

	if(High_current_switch == 1)
	{
		EEPROMRead((uint32_t *) &Resistance, OFFSET_CL_CLEAN_R, 4);

		if(Resistance != Resistance)
		{
			if(gABoard == AV7_3)
				Resistance = 178394.45;
			else if(gABoard >= AV6_4)
				Resistance = 7497.2; // 486853
			else
				Resistance = 99502.5; // 486853
		}

		EEPROMRead((uint32_t *) &ResistorRatio, OFFSET_CL_RESIST_RATIO, 4);
		if(ResistorRatio != ResistorRatio)
		{
			ResistorRatio = 1;
		}
	}
	else if(High_current_switch == 2)
	{
		EEPROMRead((uint32_t *) &Resistance, OFFSET_CL_MID_R, 4);

		if(Resistance != Resistance)
		{
			Resistance = 4063745.02;
		}

		EEPROMRead((uint32_t *) &ResistorRatio, OFFSET_CL_MID_R_RATIO, 4);
		if(ResistorRatio != ResistorRatio)
		{
			ResistorRatio = 1;
		}
	}
#ifdef HIGH_PLATING_CURRENT
	else if(High_current_switch == 3)
	{
			Resistance = 2200;
			ResistorRatio = 1;
	}
#endif
	else
	{
		EEPROMRead((uint32_t *) &Resistance, OFFSET_CL_LOW_R, 4);

		if(Resistance != Resistance)
		{
			Resistance = 20000000;
		}

		EEPROMRead((uint32_t *) &ResistorRatio, OFFSET_CL_LOW_R_RATIO, 4);
		if(ResistorRatio != ResistorRatio)
		{
			ResistorRatio = 1;
		}
	}

	float current;

//	if(ADC_Voltage > 2999)
//	{
//		UARTprintf("WARNING! ADC Voltage positively saturated, current reading not reliable. \t");
//	}
//
//
//	if(ADC_Voltage < 1)
//	{
//		UARTprintf("WARNING! ADC Voltage negatively saturated, current reading not reliable. \t");
//	}


	// current = (-V_DAC_SET + 2*V_ADC_IN - V_ref_3)/R_f // Equation used

//	current = ((2 * ADC_Voltage) - DAC_Voltage - 3000) / Resistance; // R_f = Two 10 MOhms resistors in series
	current = (ResistorRatio * (ADC_Voltage - 3000) + ADC_Voltage - DAC_Voltage) / Resistance; // R_f = Two 10 MOhms resistors in series

	return current * 1000000;
}

////**************************************************************************
//// Reads Channels 1 - 6 on amperometric ADC
//// Parameters:  NONE
//// Outputs:		fVoltages; pointer to 6 voltages read from ADC
////**************************************************************************
//float * ADCReadAll(void)
//{
//	uint32_t ui32ADC_MSB; // Most Significant byte of ADC_DOUT
//	uint32_t ui32ADC_LSB; // Least Signficant byte of ADC_DOUT
//	int16_t ADC_DOUT = 0; // signed 16 bit data assembled from two ADC_DOUT registers
//	static float fVoltages[6];
//	uint32_t ADC_DONE = 0xFF;
//	uint32_t CH_Rx;
//	int i;
//
//	int delay = SysCtlClockGet()/3000 * .072; // 72 us between consecutive ADC_DOUT register reads
//
//	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//
//	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//	SSIDisable(SSI1_BASE);
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//	SSIEnable(SSI1_BASE);
//
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
//	SysCtlDelay(20000); // Delay so channel scan register can update
//
//	// Must read channel scan register after setting it
//	// Assume this is to update values as register is buffered
//	// Don't actually know why this is required
//	SPISend(SSI1_BASE, 1, ADC4_CS_B, 0, 4, 0x10, 0x01, 0x8F, 0x00); // Read Channel Scan Register
//
//	IO_Ext_Set(IO_EXT1_ADDR, 2, ADC4_CS_B, 0); // Assert CSB
//
//	SSIDataPut(SSI1_BASE, 0xE8); // Tell which register to start reading from
//	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//
//	for(i = 0; i < 6; i++)
//	{
//		while(ADC_DONE == 0xFF)
//		{
//			while(SSIDataGetNonBlocking(SSI1_BASE, &ui32ADC_MSB)){} // Clear FIFO
//
//			// Read from ADC_DOUT registers
//			SSIDataPut(SSI1_BASE, 0x00);
//			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
//			SSIDataPut(SSI1_BASE, 0x00);
//			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
//			SSIDataPut(SSI1_BASE, 0x00);
//			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
//			SSIDataPut(SSI1_BASE, 0x00);
//			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring
//
//			SSIDataGet(SSI1_BASE, &ADC_DONE);	 // First byte is ADC_DONE register
//			SSIDataGet(SSI1_BASE, &CH_Rx); 	 // Second byte is junk, data overwritten
//			SSIDataGet(SSI1_BASE, &ui32ADC_MSB); // Save MSB
//			SSIDataGet(SSI1_BASE, &ui32ADC_LSB); // Save LSB
//
//			ADC_DOUT = 0 | (ui32ADC_MSB << 8); // Set MSB
//			ADC_DOUT |= ui32ADC_LSB; // Set LSB
//
//			if((CH_Rx & 0x07) == (i+1))
//				fVoltages[i] = (ADC_DOUT * (3000) / 32768.0) + 1500; // mV
//
////			fVoltages[i] = (CH_Rx & 0x07);
//
//			SysCtlDelay(delay);
//		}
//
//		ADC_DONE = 0xFF;
//	}
//
//	IO_Ext_Set(IO_EXT1_ADDR, 2, ADC4_CS_B, 1); // Deassert CSB
//
//	return fVoltages;
//}

//**************************************************************************
// Function to read in voltage on specific channel
// Samples at ~200 SPS, max of 25 samples
// Parameters:	Channel; [0,6] 	0: All amperometrics
//								1: Amp 1
//								2: Amp 2... etc
//				ADC_CS_PIN; ADC1_CS_B, ADC2_CS_B, or ADC4_CS_B
//				seconds; Number of seconds to sample; 20
//				DAC_Voltage; High current switch status for calculation
//				period; the period in between taking samples
//				Current_switch; 0 = 20M; 1 = 100k; 2 = 5.1M
// Outputs:		Start Current; First point read in nA
//				Final Current; Last point read in nA
//**************************************************************************
float * CurrentTimeRead(uint8_t ui8Channel, int ADC_CS_PIN, float seconds, int DAC_Voltage, uint8_t High_current_switch, float period)
{
	float fVoltage;
	static float Current[2]; // {Start Current, Final Current}

	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	SysCtlDelay(2000);

	uint8_t ADC_Address = 0x10; // Instruction 1, must be sent whenever URA Changes
	uint8_t URA = 0x01; // Upper Register Address; Channel Scan Register and ADC_DOUT register have same URA
	uint8_t LRA_ch_scan_w = 0x0F; // Lower Register Address; write 1 byte to channel scan register
	uint8_t LRA_ch_scan_r = 0x8F; // Read 1 byte from channel scan register
	uint8_t LRA_ADC_DOUT = 0xAA; // Read 2 bytes from ADC_DOUT registers

	uint8_t ui8Channel_Tx = (ui8Channel | ((ui8Channel) << 3)); // Set first and last channel to scan

	uint32_t ui32ADC_MSB; // Most Significant byte of ADC_DOUT
	uint32_t ui32ADC_LSB; // Least Signficant byte of ADC_DOUT
	int16_t ADC_DOUT = 0; // signed 16 bit data assembled from two ADC_DOUT registers
	uint32_t ui32Channel_Rx = ui8Channel_Tx + 1;

	uint8_t counter = 0;

	// Random data points seem really far off, as if the channel wasn't changed successfully and wrong channel was being read
	// Created loop to verify the channel register was updated correctly
	while(ui8Channel_Tx != ui32Channel_Rx && counter <= 10)
	{
		// Set ADC we are using into active mode
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x00); // Register 0x08, 0x3 stand-by mode, 0x0 active mode

		// Write the channel scan register to set which channel to read
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_w, ui8Channel_Tx); // Set channel scan register
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
		SysCtlDelay(30000); // Delay so channel scan register can update

		userDelay(1, 1);	// Delay 1 ms after activating ADC before reading, get bad first point if no delay

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
			DEBUG_PRINT(UARTprintf("Tried writing ADC channel 5 times, failed everytime! \n");)
			DEBUG_PRINT(UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);)
			DEBUG_PRINT(UARTprintf("Resetting analog board!\n");)
			counter++;

			AnalogOff();

			SSIDisable(SSI1_BASE);
			SysCtlDelay(SysCtlClockGet());	// Wait for analog board to power down

//			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SysCtlDelay(2000);

			InitAnalog();
		}
		else if(counter < 10)
			counter++;
		else if(counter == 10)
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

	g_TimerInterruptFlag = 0;
	TimerLoadSet(TIMER0_BASE, TIMER_A, (seconds * SysCtlClockGet()));		// Timer set
	TimerEnable(TIMER0_BASE, TIMER_A);

	TimerLoadSet(TIMER1_BASE, TIMER_A, (period * SysCtlClockGet()));		// Timer set
	TimerEnable(TIMER1_BASE, TIMER_A);

	uint8_t First_point = 1;	// Flag that we need to get the first point
	while(g_TimerInterruptFlag == 0)
	{
		// Read from ADC_DOUT registers
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 5, 0x10, 0x01, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers

		// First three bytes are junk data
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);

		// Collect ADC data
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); 	// Save MSB
		SSIDataGet(SSI1_BASE, &ui32ADC_LSB); 	// Save LSB

		ADC_DOUT = 0;	// Clear variable before OR'ing to it again
		ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
		ADC_DOUT |= ui32ADC_LSB; // Set LSB

		fVoltage = (ADC_DOUT * (1500.0) / 32768.0) + 1500.0; // Convert ADC_DOUT data into voltage
		if(First_point == 0)
		{
			Current[1] = CalculateCurrent(fVoltage, DAC_Voltage, High_current_switch);

			//		UARTprintf("Voltage: \t %d \t DOUT: \t %d \n", (int) fVoltage, ADC_DOUT);

			#ifdef PRINT_CURRENT
			float Calibrated_Current = CalculateCurrentCalibrated(fVoltage, DAC_Voltage, High_current_switch);
			DEBUG_PRINT(
					if(High_current_switch == 1)
						UARTprintf("Current:\t%d\t%d\tnA\n", (int) (Current[1]), (int) (Calibrated_Current));
					else
						UARTprintf("Current:\t%d\t%d\tnA * 1000\n", (int) (Current[1] * 1000), (int) (Calibrated_Current * 1000));
			)
			#endif
		}
		else
		{
			Current[0] = CalculateCurrent(fVoltage, DAC_Voltage, High_current_switch);
			First_point = 0;

			//		UARTprintf("Voltage: \t %d \t DOUT: \t %d \n", (int) fVoltage, ADC_DOUT);

			#ifdef PRINT_CURRENT
			float Calibrated_Current = CalculateCurrentCalibrated(fVoltage, DAC_Voltage, High_current_switch);
			DEBUG_PRINT(
					if(High_current_switch == 1)
						UARTprintf("Current:\t%d\t%dnA\n", (int) (Current[0]), (int) (Calibrated_Current));
					else
						UARTprintf("Current:\t%d\t%dnA * 1000\n", (int) (Current[0] * 1000), (int) (Calibrated_Current * 1000));
			)
			#endif
		}

		//					while(g_TimerPeriodicInterruptFlag == 0);
		while(g_TimerPeriodicInterruptFlag == 0)
		{
			// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
			// it will prevent the BT from reading incorrect data into the app, TOD: Redesign app to wait for data rather than write read move on
#ifdef MCU_ZXR
			if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
				ConnectMemory(1);
#else
			if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
				ConnectMemory(1);
#endif

		}
		g_TimerPeriodicInterruptFlag = 0;

		if((gui32Error & ABORT_ERRORS) != 0)
			break;
	}

	// Set ADC back into stand-by mode
	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode

	TimerDisable(TIMER1_BASE, TIMER_A);
	g_TimerInterruptFlag = 0;

	return Current;
}

//**************************************************************************
// Function to read in voltage on specific channel
// Samples at ~200 SPS, max of 25 samples
// Parameters:	Channel; [0,6] 	0: All amperometrics
//								1: Amp 1
//								2: Amp 2... etc
//				ADC_CS_PIN; ADC1_CS_B, ADC2_CS_B, or ADC4_CS_B
//				min_seconds; Minimum number of seconds to sample; 20
//				max_seconds; Maximum number of seconds to sample
//				DAC_Voltage; High current switch status for calculation
//				period; the period in between taking samples
//				Current_switch; 0 = 20M; 1 = 100k; 2 = 5.1M
//				Minimum Current; nA minimum current, assuming a decaying current using abs_val
// Outputs:		Start Current; First point read in nA
//				Final Current; Last point read in nA
//**************************************************************************
float * CurrentTimeRead_CurrentLimited(uint8_t ui8Channel, int ADC_CS_PIN, float min_seconds, float max_seconds, int DAC_Voltage, uint8_t High_current_switch, float period, float MinI, uint16_t page)
{
	float fVoltage;
	static float Current[2];

	while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

	// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
	SSIDisable(SSI1_BASE);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
	SSIEnable(SSI1_BASE);

	SysCtlDelay(2000);

	uint8_t ADC_Address = 0x10; // Instruction 1, must be sent whenever URA Changes
	uint8_t URA = 0x01; // Upper Register Address; Channel Scan Register and ADC_DOUT register have same URA
	uint8_t LRA_ch_scan_w = 0x0F; // Lower Register Address; write 1 byte to channel scan register
	uint8_t LRA_ch_scan_r = 0x8F; // Read 1 byte from channel scan register
	uint8_t LRA_ADC_DOUT = 0xAA; // Read 2 bytes from ADC_DOUT registers

	uint8_t ui8Channel_Tx = (ui8Channel | ((ui8Channel) << 3)); // Set first and last channel to scan

	uint32_t ui32ADC_MSB; // Most Significant byte of ADC_DOUT
	uint32_t ui32ADC_LSB; // Least Signficant byte of ADC_DOUT
	int16_t ADC_DOUT = 0; // signed 16 bit data assembled from two ADC_DOUT registers
	uint32_t ui32Channel_Rx = ui8Channel_Tx + 1;

	uint8_t counter = 0;

	// Random data points seem really far off, as if the channel wasn't changed successfully and wrong channel was being read
	// Created loop to verify the channel register was updated correctly
	while(ui8Channel_Tx != ui32Channel_Rx && counter <= 10)
	{
		// Set ADC we are using into active mode
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x00); // Register 0x08, 0x3 stand-by mode, 0x0 active mode

		// Write the channel scan register to set which channel to read
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_w, ui8Channel_Tx); // Set channel scan register
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
		SysCtlDelay(30000); // Delay so channel scan register can update

		userDelay(1, 1);	// Delay 1 ms after activating ADC before reading, get bad first point if no delay

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
			DEBUG_PRINT(
			UARTprintf("Tried writing ADC channel 5 times, failed everytime! \n");
			UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);
			UARTprintf("Resetting analog board!\n");
			)
			counter++;

			AnalogOff();

			SSIDisable(SSI1_BASE);
			SysCtlDelay(SysCtlClockGet());	// Wait for analog board to power down

//			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin

			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SysCtlDelay(2000);

			InitAnalog();
		}
		else if(counter < 10)
			counter++;
		else if(counter == 10)
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

	g_TimerInterruptFlag = 0;
	TimerLoadSet(TIMER0_BASE, TIMER_A, (min_seconds * SysCtlClockGet()));		// Timer set
	TimerEnable(TIMER0_BASE, TIMER_A);

	TimerLoadSet(TIMER1_BASE, TIMER_A, (period * SysCtlClockGet()));		// Timer set
	TimerEnable(TIMER1_BASE, TIMER_A);

	while(g_TimerInterruptFlag == 0);
	g_TimerInterruptFlag = 0;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ((max_seconds - min_seconds) * SysCtlClockGet()));		// Timer set
	TimerEnable(TIMER0_BASE, TIMER_A);

	// Read from ADC_DOUT registers
	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 5, 0x10, 0x01, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers

	// First three bytes are junk data
	SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
	SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
	SSIDataGet(SSI1_BASE, &ui32ADC_MSB);

	// Collect ADC data
	SSIDataGet(SSI1_BASE, &ui32ADC_MSB); 	// Save MSB
	SSIDataGet(SSI1_BASE, &ui32ADC_LSB); 	// Save LSB

	ADC_DOUT = 0;	// Clear variable before OR'ing to it again
	ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
	ADC_DOUT |= ui32ADC_LSB; // Set LSB

	fVoltage = (ADC_DOUT * (1500.0) / 32768.0) + 1500.0; // Convert ADC_DOUT data into voltage
	Current[0] = CalculateCurrent(fVoltage, DAC_Voltage, High_current_switch);
	Current[1] = Current[0];

	float time = min_seconds;
	while(abs_val(Current[1]) > abs_val(MinI) && g_TimerInterruptFlag == 0)
	{
		// Read from ADC_DOUT registers
		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 5, 0x10, 0x01, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers

		// First three bytes are junk data
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);

		// Collect ADC data
		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); 	// Save MSB
		SSIDataGet(SSI1_BASE, &ui32ADC_LSB); 	// Save LSB

		ADC_DOUT = 0;	// Clear variable before OR'ing to it again
		ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
		ADC_DOUT |= ui32ADC_LSB; // Set LSB

		fVoltage = (ADC_DOUT * (1500.0) / 32768.0) + 1500.0; // Convert ADC_DOUT data into voltage
		//		Current = CalculateCurrent(fVoltage, DAC_Voltage, High_current_switch);
		//
		////		UARTprintf("Voltage: \t %d \t DOUT: \t %d \n", (int) fVoltage, ADC_DOUT);
		//
		////#ifdef PRINT_CURRENT
		//		if(High_current_switch == 1)
		//			UARTprintf("Current: \t %d \t nA \n", (int) (Current));
		//		else
		//			UARTprintf("Current: \t %d \t nA * 1000 \n", (int) (Current * 1000));
		////#endif


		Current[1] = CalculateCurrent(fVoltage, DAC_Voltage, High_current_switch);

		//		UARTprintf("Voltage: \t %d \t DOUT: \t %d \n", (int) fVoltage, ADC_DOUT);

#ifdef PRINT_CURRENT
		DEBUG_PRINT(
		if(High_current_switch == 1)
			UARTprintf("Current:\t%d\tnA\n", (int) (Current[1]));
		else
			UARTprintf("Current: \t %d \t nA * 1000 \n", (int) (Current[1] * 1000));
		)
#endif

		//					while(g_TimerPeriodicInterruptFlag == 0);
		while(g_TimerPeriodicInterruptFlag == 0)
		{
			// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
			// it will prevent the BT from reading incorrect data into the app, TOD: Redesign app to wait for data rather than write read move on
#ifdef MCU_ZXR
			if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
				ConnectMemory(1);
#else
			if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
				ConnectMemory(1);
#endif
		}
		g_TimerPeriodicInterruptFlag = 0;
		time += period;

		if((gui32Error & ABORT_ERRORS) != 0)
			break;
	}

	DEBUG_PRINT(UARTprintf("Ran for %d ms before reaching %d nA\n", (int) (time * 1000), (int) (Current[1]));)

#ifndef	MEMORY_V5
	if(page > 0)
	{
		uint8_t Time_to_save = time;
		MemoryWrite(page, OFFSET_CLEAN_CATH_TIME, 1, &Time_to_save);
	}
#endif

	// Set ADC back into stand-by mode
	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode

	TimerDisable(TIMER1_BASE, TIMER_A);
	g_TimerInterruptFlag = 0;

	return Current;
}

////**************************************************************************
//// Function to read in voltage on specific channel
//// Samples at ~200 SPS, max of 25 samples
//// Parameters:	Channel; [0,6] 	0: All amperometrics
////								1: Amp 1
////								2: Amp 2... etc
////				ADC_CS_PIN; ADC1_CS_B, ADC2_CS_B, or ADC4_CS_B
////				seconds; Number of seconds to sample; 20
////				DAC_Voltage; Voltage DAC is set at to be used for calculation
////				High current switch; status for calculation
////				period; the period in between taking samples
////				Current_switch; 0 = 20M; 1 = 100k; 2 = 5.1M
//// Outputs:		Current; Last point read in mA
////**************************************************************************
//float * CurrentTimeRead_SplitAmps(uint8_t ui8Channel, int ADC_CS_PIN, float seconds, int DAC_Voltage, uint8_t High_current_switch, float period)
//{
//	float fVoltage;
//	static float Current[2];
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
//	uint8_t counter = 0;
//
//	// Random data points seem really far off, as if the channel wasn't changed successfully and wrong channel was being read
//	// Created loop to verify the channel register was updated correctly
//	while(ui8Channel_Tx != ui32Channel_Rx && counter <= 10)
//	{
//		// Set ADC we are using into active mode
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x00); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//		// Write the channel scan register to set which channel to read
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, ADC_Address, URA, LRA_ch_scan_w, ui8Channel_Tx); // Set channel scan register
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
//		SysCtlDelay(30000); // Delay so channel scan register can update
//
//		userDelay(1, 1);	// Delay 1 ms after activating ADC before reading, get bad first point if no delay
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
//			UARTprintf("Tried writing ADC channel 5 times, failed everytime! \n");
//			UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);
//			UARTprintf("Resetting analog board!\n");
//			counter++;
//
//			AnalogOff();
//
//			SSIDisable(SSI1_BASE);
//			SysCtlDelay(SysCtlClockGet());	// Wait for analog board to power down
//
////			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//
//			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//			SSIEnable(SSI1_BASE);
//
//			SysCtlDelay(2000);
//
//			InitAnalog();
//		}
//		else if(counter < 10)
//			counter++;
//		else if(counter == 10)
//		{
//			counter++;
//			UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);
//
//			if(ADC_CS_PIN == ADC1_CS_B)
//			{
//				UARTprintf("ADC 1 didn't program channel scan register correctly! \n");
//
//				gui32Error |= ADC1_FAIL;
//				update_Error();
//			}
//			if(ADC_CS_PIN == ADC2_CS_B)
//			{
//				UARTprintf("ADC 2 didn't program channel scan register correctly! \n");
//
//				gui32Error |= ADC2_FAIL;
//				update_Error();
//			}
//			if(ADC_CS_PIN == ADC4_CS_B)
//			{
//				UARTprintf("ADC 4 didn't program channel scan register correctly! \n");
//
//				gui32Error |= ADC4_FAIL;
//				update_Error();
//			}
//
//			uint8_t ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 2, 1);
//			UARTprintf("IO Extender 1 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg2);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);
//			UARTprintf("IO Extender 1 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg3);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);
//			UARTprintf("IO Extender 2 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg2);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
//			UARTprintf("IO Extender 2 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg3);
//			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1));
//		}
//	}
//
//	ui8Channel_Tx = (5 | ((5) << 3)); // Set first and last channel to scan, 5 is channel for ISE 10 which is what modified board hooked up to
//
//	uint32_t ui32ADC_MSB_2; // Most Significant byte of ADC_DOUT
//	uint32_t ui32ADC_LSB_2; // Least Signficant byte of ADC_DOUT
//	int16_t ADC_DOUT_2 = 0; // signed 16 bit data assembled from two ADC_DOUT registers
//	ui32Channel_Rx = ui8Channel_Tx + 1;
//
//	counter = 0;
//
//	// Random data points seem really far off, as if the channel wasn't changed successfully and wrong channel was being read
//	// Created loop to verify the channel register was updated correctly
//	while(ui8Channel_Tx != ui32Channel_Rx && counter <= 10)
//	{
//		// Set ADC we are using into active mode
//		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x00); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//		// Write the channel scan register to set which channel to read
//		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, ADC_Address, URA, LRA_ch_scan_w, ui8Channel_Tx); // Set channel scan register
//		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x00, 0x0B, 0x01); // Send command to restart conversion
//		SysCtlDelay(30000); // Delay so channel scan register can update
//
//		userDelay(1, 1);	// Delay 1 ms after activating ADC before reading, get bad first point if no delay
//
//		// Must read channel scan register after setting it
//		// Assume this is to update values as register is buffered
//		// Don't actually know why this is required
//		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, ADC_Address, URA, LRA_ch_scan_r, 0x00); // Read Channel Scan Register
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, (uint32_t *) &ui32Channel_Rx);
//
//		if(counter < 5)
//			counter++;
//		else if(counter == 5)
//		{
//			UARTprintf("Tried writing ADC channel 5 times, failed everytime! \n");
//			UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);
//			UARTprintf("Resetting analog board!\n");
//			counter++;
//
//			AnalogOff();
//
//			SSIDisable(SSI1_BASE);
//			SysCtlDelay(SysCtlClockGet());	// Wait for analog board to power down
//
////			while(SSIBusy(SSI1_BASE)){} // Wait for SSI to finish transferring before raising SS pin
//
//			// Set SPI communication to capture on rising edge of clock (DAC captures on falling edge)
//			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//			SSIEnable(SSI1_BASE);
//
//			SysCtlDelay(2000);
//
//			InitAnalog();
//		}
//		else if(counter < 10)
//			counter++;
//		else if(counter == 10)
//		{
//			counter++;
//			UARTprintf("Writing: 0x%x, receiving: 0x%x\n", ui8Channel_Tx, ui32Channel_Rx);
//
//			if(ADC2_CS_B == ADC1_CS_B)
//			{
//				UARTprintf("ADC 1 didn't program channel scan register correctly! \n");
//
//				gui32Error |= ADC1_FAIL;
//				update_Error();
//			}
//			if(ADC2_CS_B == ADC2_CS_B)
//			{
//				UARTprintf("ADC 2 didn't program channel scan register correctly! \n");
//
//				gui32Error |= ADC2_FAIL;
//				update_Error();
//			}
//			if(ADC2_CS_B == ADC4_CS_B)
//			{
//				UARTprintf("ADC 4 didn't program channel scan register correctly! \n");
//
//				gui32Error |= ADC4_FAIL;
//				update_Error();
//			}
//
//			uint8_t ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 2, 1);
//			UARTprintf("IO Extender 1 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg2);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT1_ADDR, 3, 1);
//			UARTprintf("IO Extender 1 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext1_Reg3);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 2, 1);
//			UARTprintf("IO Extender 2 reg 2: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg2);
//			ui8PinCheck = I2CReceive(I2C0_BASE, IO_EXT2_ADDR, 3, 1);
//			UARTprintf("IO Extender 2 reg 3: %x, global: %x\n", ui8PinCheck, gui8IO_Ext2_Reg3);
//			UARTprintf("LED IO Ext CS_B: %x\n", GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_1));
//		}
//	}
//
//	g_TimerInterruptFlag = 0;
//	TimerLoadSet(TIMER0_BASE, TIMER_A, (seconds * SysCtlClockGet()));		// Timer set
//	TimerEnable(TIMER0_BASE, TIMER_A);
//
//	TimerLoadSet(TIMER1_BASE, TIMER_A, (period * SysCtlClockGet()));		// Timer set
//	TimerEnable(TIMER1_BASE, TIMER_A);
//
//	while(g_TimerInterruptFlag == 0)
//	{
//		// Read from ADC_DOUT registers
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 5, 0x10, 0x01, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers
//
//		// First three bytes are junk data
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//
//		// Collect ADC data
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); 	// Save MSB
//		SSIDataGet(SSI1_BASE, &ui32ADC_LSB); 	// Save LSB
//
//		ADC_DOUT = 0;	// Clear variable before OR'ing to it again
//		ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
//		ADC_DOUT |= ui32ADC_LSB; // Set LSB
//
//		fVoltage = (ADC_DOUT * (1500.0) / 32768.0) + 1500.0; // Convert ADC_DOUT data into voltage
//		Current[0] = CalculateCurrent(fVoltage, DAC_Voltage, High_current_switch);
//
//
//
//		// Read from ADC_DOUT registers
//		SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 5, 0x10, 0x01, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers
//
//		// First three bytes are junk data
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//
//		// Collect ADC data
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); 	// Save MSB
//		SSIDataGet(SSI1_BASE, &ui32ADC_LSB); 	// Save LSB
//
//		ADC_DOUT = 0;	// Clear variable before OR'ing to it again
//		ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
//		ADC_DOUT |= ui32ADC_LSB; // Set LSB
//
//		fVoltage = (ADC_DOUT * (1500.0) / 32768.0) + 1500.0; // Convert ADC_DOUT data into voltage
//		Current[1] = CalculateCurrent(fVoltage, DAC_Voltage, High_current_switch);
//
//
//
//		//					while(g_TimerPeriodicInterruptFlag == 0);
//		while(g_TimerPeriodicInterruptFlag == 0)
//		{
//			// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
//			// it will prevent the BT from reading incorrect data into the app, TOD: Redesign app to wait for data rather than write read move on
//			if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) == GPIO_PIN_6 && gui8MemConnected == 0)
//				ConnectMemory(1);
//		}
//		g_TimerPeriodicInterruptFlag = 0;
//
//		if((gui32Error & ABORT_ERRORS) != 0)
//			break;
//	}
//
//	// Set ADC back into stand-by mode
//	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//	// Set ADC back into stand-by mode
//	SPISend(SSI1_BASE, 1, ADC2_CS_B, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//	TimerDisable(TIMER1_BASE, TIMER_A);
//	g_TimerInterruptFlag = 0;
//
//	return Current;
//}

////**************************************************************************
//// Runs the cleaning procedure for gold array by setting 600 mV for 60 seconds
//// Parameters: 	NONE
//// Outputs:		NONE
////**************************************************************************
//void CleanGold(int8_t Ref_drift)
//{
//	// TODO: Set up Gold cleaning procedure here, Enter number of steps, mV to apply each step, and time to hold each step
//	#define CLEANING_STEPS	2
//	int16_t i16V_clean[CLEANING_STEPS] = {-400 - Ref_drift, 400 - Ref_drift}; // Enter mV values -3000->3000, decimals not accepted
//	uint8_t Step_Times[CLEANING_STEPS] = {20, 20}; // Enter seconds here up to 255, decimals not accepted
//
//	UARTprintf("Cleaning gold pads... \n");
//
//	// Set high current switch for cleaning
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);
//
//	// Make sure all platinum arrays are floating
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
//
//	// Connect metals electrodes for potential steps
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 1);
//
//	// Set RE + CE to amperometric loop
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
//	//
//	// Potential Steps
//	//
//	uint8_t i;
//	float * fI_WE; 	// Current flowing through WEs, nA
//	for(i = 0; i < CLEANING_STEPS; i++)
//	{
//		DACVoltageSet(0, i16V_clean[i], true);	// WEs
//
//		fI_WE = CurrentTimeRead(0, ADC4_CS_B, Step_Times[i], i16V_clean[i], 1, .1);	// nA
//		UARTprintf("Current ended at: %d nA\n", (int) fI_WE[1]);
//	}
//
//	// Turn off cathodic cleaning voltages
//	DACVoltageSet(0, 0, true);	// WEs
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
//
//	// Let Metals electrode float after cleaning
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 0);
//
//	// Let reference and counter electrodes float
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//	UARTprintf("Cleaning completed! \n");
//}

////**************************************************************************
//// Runs the cleaning procedure for all Amperometric arrays and ORP sensor
//// Parameters: 	Ref_drift; Calculated drift of the reference
////				Cal_Number; Calibration to save cleaning data to; 0 if not saving
////				Test_Number, Test to save cleaning data to; 0 if not saving
//// Parameters:	Oxide_Rebuild; Flag to control type of rebuild; 0 will NOT rebuild; 1 will do original 10 second rebuild, 2 will rebuild until its average hits a set mV
//// Outputs:		false if error occurs; current outside of expected range
////				true if cleaning successfully completed
////**************************************************************************
//void CleanAmperometrics(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number)
//{
//	uint16_t page;
//	if(Cal_Number != 0)
//		page = Find_Cal_page(Cal_Number);
//	if(Test_Number != 0)
//		page = Find_Test_page(Test_Number);
//
//	uint8_t step;
//	float * fI_WE; 	// Starting and Ending Current flowing through WEs, nA
////	int16_t i16Potential_Step[7] = {1300, -500, 1300, -500, 1300, -500, 1300}; // mV	// Bicarbonate rinse cleaning numbers
////	int16_t i16Potential_Step[7] = {1100, -800, 1100, -800, 1100, -800, 1100}; // mV	// 11/14/2019 HEPES rinse
////	int16_t i16Potential_Step[7] = {800, -700, 800, -700, 800, -700, 800}; // mV	// Changed 10/9/2019 HEPES rinse
////	int16_t i16Potential_Step[7] = {1200, -250, 1200, -250, 1200, -250, 1200}; // mV	// Nitric Acid
////	int16_t i16Potential_Step[7] = {1000, -250, 1000, -250, 1000, -250, 1000}; // mV	// T1
//	int16_t i16Potential_Step[7] = {1000 - Ref_drift, -700 - Ref_drift, 1000 - Ref_drift, -700 - Ref_drift, 1000 - Ref_drift, -700 - Ref_drift, 1000 - Ref_drift}; // mV	// 2/13/2020 Cleaning solution
////	int16_t i16V_cathodic_clean = -300; // mV	// Bicarbonate rinse cleaning numbers
////	int16_t i16V_cathodic_clean = -600; // mV	// 11/14/2019 HEPES rinse
////	int16_t i16V_cathodic_clean = -700; // mV	// Changed 10/9/2019 HEPES rinse
////	int16_t i16V_cathodic_clean = -250; // mV	// Nitric Acid
////	int16_t i16V_cathodic_clean = -250; // mV	// T1
//	int16_t i16V_cathodic_clean = -700 - Ref_drift; // mV	// 2/13/2020 Cleaning solution
//
//	float fI_drive = 0.7;	// uA to drive through each electrode when rebuilding oxide
//	if(gABoard >= AV6_4)
//		fI_drive = 3.0;
////		fI_drive = 1.0;	// Changed 10/9/2019
//
//	float Rebuild_time = 10;	// # of seconds to drive current when rebuilding oxide
//
//	UARTprintf("Cleaning Amperometrics using steps and oxide rebuild... \n");
//
//	int i;
//
//	// Connect all electrodes together for potential steps
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);
//
////	// Connect metals electrodes for potential steps
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 1);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 1);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 1);
//
//	// Set ORP switches to voltage setting and current reading position
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 1);
//
//	// Set RE + CE to amperometric loop
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
//	//
//	// Potential Steps
//	//
//	DACVoltageSet(0, i16Potential_Step[0], false);	// WEs
//	DACVoltageSet(5, i16Potential_Step[0], true);	// ORP
//	UARTprintf("Potential step %d... \n", 1);
//
//	fI_WE = CurrentTimeRead(0, ADC4_CS_B, 5, i16Potential_Step[0], 1, .1);	// nA
//
//	UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);
//
//	// Skip first step, cleaning solutions first step is around 0, sometimes positive sometimes negative
////	if(fI_WE < 0)
////	{
////		UARTprintf("Current should be positive!\n");
////		gui32Error |= CL_CLEANING_OUT_OF_RANGE;
////		update_Error();
////	}
//
//	for(step = 1; step < 7; step++)
//	{
//		DACVoltageSet(0, i16Potential_Step[step], false);	// WEs
//		DACVoltageSet(5, i16Potential_Step[step], true);	// ORP
//		UARTprintf("Potential step %d... \n", (step + 1));
//
//		fI_WE = CurrentTimeRead(0, ADC4_CS_B, 3, i16Potential_Step[step], 1, .1);	// nA
//		UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);
//
//		if((step % 2) == 0)	// Positive step
//		{
//			if(fI_WE[1] < -500)
//			{
//				UARTprintf("Current should be positive!\n");
//				gui32Error |= CL_CLEANING_OUT_OF_RANGE;
//				update_Error();
//			}
//		}
//		else	// Negative step
//		{
//			if(fI_WE[1] > 0)
//			{
//				UARTprintf("Current should be negative!\n");
//				gui32Error |= CL_CLEANING_OUT_OF_RANGE;
//				update_Error();
//			}
//
//			if((gui32Error & ABORT_ERRORS) != 0)
//				break;
//		}
//	}
//
//	//
//	// Cathodic Cleaning
//	//
//	if((gui32Error & ABORT_ERRORS) == 0)
//	{
//		DACVoltageSet(0, i16V_cathodic_clean, false);	// WEs
//		DACVoltageSet(5, i16V_cathodic_clean, true);	// ORP
//
//		UARTprintf("Cathodic Cleaning: \n");
//		fI_WE = CurrentTimeRead(0, ADC4_CS_B, 45, i16V_cathodic_clean, 1, .1);
//
//		UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);
//		if(fI_WE[1] < -7000 || fI_WE[1] > 0)
//		{
//			UARTprintf("Current should be between -7000 and 0 nA!\n");
//			gui32Error |= CL_CLEANING_OUT_OF_RANGE;
//			update_Error();
//		}
//	}
//
//	// Turn off cathodic cleaning voltages
//	DACVoltageSet(0, 0, false);	// WEs
//	DACVoltageSet(5, 0, true);	// ORP
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
//
//	// Disconnect ORP sensor from driving circuitry
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 0);
//
//#ifndef MEMORY_V5
//	// Write 8 bytes to save both the starting and final currents
//	MemoryWrite(page, OFFSET_CLEAN_CATH_START, 8, (uint8_t *) fI_WE);
//#endif
//
//	//
//	// Anodic Oxide Rebuild
//	//
//	float WEVoltages[5];
//	int16_t Start_V[5];
//		if((gui32Error & ABORT_ERRORS) == 0)
//		{
//			UARTprintf("Rebuilding Oxide... \n");
//
//			// Connect all electrodes separately to drive current through each
//			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
//			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
//
//			//		// Connect metals electrodes for current driving
//			//		IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 0);
//			//		IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 1);
//
//			//	float Voltage = fI_drive * 499; // fI_drive * 499000 / 1000	// Current mirror hardware
//			//	float Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware
//
//			float Voltage;
//			if(gABoard >= AV6_4)
//				Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays
//			else if(gABoard >= AV6)
//				//		Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays, Resistor changed from 430 kOhms to 30 kOhms to allow current for BES rinse
//				Voltage = 3000 - fI_drive * 430; // mV // Op-amp hardware driving 5 arrays
//			else
//				Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware with all arrays tied together
//
//			//		DACVoltageSet(1, Voltage, false);	// Gold array
//			DACVoltageSet(2, Voltage, true);	// 5 platinum arrays
//
//			int j;
//			TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * period); // Set periodic timer
//			TimerEnable(TIMER1_BASE, TIMER_A);
//			for(i = 0; i < Rebuild_time; i++)
//			{
//				while(g_TimerPeriodicInterruptFlag == 0);
//				g_TimerPeriodicInterruptFlag = 0;
//
//				UARTprintf("Voltages after %d second(s) are: ", (i + 1));
//				for(j = 0; j < 5; j++)
//				{
//					WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
//					UARTprintf("%d, ", (int) WEVoltages[j]);
//					if(i == 0)
//						Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
//				}
//				UARTprintf("\n");
//
//				// Program check for specific value range
//				if(i == (Rebuild_time - 1))
//				{
//					if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 100) || FindArrayMax(WEVoltages, 5) > 1600 || FindArrayMin(WEVoltages, 5) < 1000)
//					{
//						UARTprintf("Voltages should be closer together!\n");
//						gui32Error |= CL_CLEANING_OUT_OF_RANGE;
//						update_Error();
//					}
//				}
//
//				if((gui32Error & ABORT_ERRORS) != 0)
//					break;
//			}
//			TimerDisable(TIMER1_BASE, TIMER_A);
//			g_TimerPeriodicInterruptFlag = 0;
//		}
//
//		//	DACVoltageSet(1, 3000, false);	// Gold array
//		DACVoltageSet(2, 3000, true);	// 5 platinum arrays
//
//	SysCtlDelay(SysCtlClockGet()/6);	// Wait 1/2 second after turning off current to allow capacitor to discharge
//
//	// Let the working electrodes float after cleaning
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
//
////	// Let Metals electrode float after cleaning
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 0);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 0);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 0);
//
//	// Let reference and counter electrodes float
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//#ifndef MEMORY_V5
//	int16_t End_V[5];	// Changing voltages to int16_t to save space in memory
//	for(i = 0; i < 5; i++)
//		End_V[i] = WEVoltages[i];
//
//	MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_START, 10, (uint8_t *) Start_V);
//	MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_FINAL, 10, (uint8_t *) End_V);
//#endif
//
//	UARTprintf("Cleaning completed! \n");
//}

//**************************************************************************
// Runs the cleaning procedure for all Amperometric arrays and ORP sensor
// Parameters: 	Ref_drift; Calculated drift of the reference
//				Cal_Number; Calibration to save cleaning data to; 0 if not saving
//				Test_Number, Test to save cleaning data to; 0 if not saving
// Parameters:	Oxide_Rebuild; Flag to control type of rebuild; 0 will NOT rebuild; 1 will do original 10 second rebuild, 2 will rebuild until its average hits a set mV
// Outputs:		false if error occurs; current outside of expected range
//				true if cleaning successfully completed
//**************************************************************************
void CleanAmperometrics(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number, uint8_t Oxide_Rebuild)
{
	if((gui32Error & ABORT_ERRORS) == 0)
	{
#ifndef MEMORY_V5
		uint16_t page;
		if(Cal_Number != 0)
			page = Find_Cal_page(Cal_Number);
		if(Test_Number != 0)
			page = Find_Test_page(Test_Number);
#endif

		int16_t Positive_Step = 1000;
		int16_t Negative_Step = -700;
		float Clean_pH = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
		if(Clean_pH < 7)
		{
			Negative_Step = -650;
			DEBUG_PRINT(UARTprintf("pH 6 Clean, adjusting cleaning voltage!\n");)
		}
		else if(Clean_pH > 8.5)
		{
			Positive_Step = 1000;
			Negative_Step = -800;
			DEBUG_PRINT(UARTprintf("pH 9 Clean, adjusting cleaning voltage!\n");)
		}


		uint8_t step;
		float * fI_WE; 	// Starting and Ending Current flowing through WEs, nA
		//	int16_t i16Potential_Step[7] = {1300, -500, 1300, -500, 1300, -500, 1300}; // mV	// Bicarbonate rinse cleaning numbers
		//	int16_t i16Potential_Step[7] = {1100, -800, 1100, -800, 1100, -800, 1100}; // mV	// 11/14/2019 HEPES rinse
		//	int16_t i16Potential_Step[7] = {800, -700, 800, -700, 800, -700, 800}; // mV	// Changed 10/9/2019 HEPES rinse
		//	int16_t i16Potential_Step[7] = {1200, -250, 1200, -250, 1200, -250, 1200}; // mV	// Nitric Acid
		//	int16_t i16Potential_Step[7] = {1000, -250, 1000, -250, 1000, -250, 1000}; // mV	// T1
		int16_t i16Potential_Step[7] = {Positive_Step - Ref_drift, Negative_Step - Ref_drift, Positive_Step - Ref_drift, Negative_Step - Ref_drift, Positive_Step - Ref_drift, Negative_Step - Ref_drift, Positive_Step - Ref_drift}; // mV	// 2/13/2020 Cleaning solution
		//	int16_t i16V_cathodic_clean = -300; // mV	// Bicarbonate rinse cleaning numbers
		//	int16_t i16V_cathodic_clean = -600; // mV	// 11/14/2019 HEPES rinse
		//	int16_t i16V_cathodic_clean = -700; // mV	// Changed 10/9/2019 HEPES rinse
		//	int16_t i16V_cathodic_clean = -250; // mV	// Nitric Acid
		//	int16_t i16V_cathodic_clean = -250; // mV	// T1
		int16_t i16V_cathodic_clean = Negative_Step - Ref_drift; // mV	// 2/13/2020 Cleaning solution

		float fI_drive = 0.7;	// uA to drive through each electrode when rebuilding oxide
		if(gABoard >= AV6_4)
			//		fI_drive = 2.25;
			fI_drive = 3.0;
		//		fI_drive = 1.0;	// Changed 10/9/2019

		float Rebuild_time = 10;	// # of seconds to drive current when rebuilding oxide

		DEBUG_PRINT(UARTprintf("Cleaning Amperometrics using steps and oxide rebuild... \n");)

		int i;

		// Set RE + CE to amperometric loop
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

		// Connect all electrodes together for potential steps
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
		IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);
		if(gABoard >= AV7_3)
			IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);

		//	// Connect metals electrodes for potential steps
		//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 1);
		//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 1);
		//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 1);

		// Set ORP switches to voltage setting and current reading position
		IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 1);
		IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 1);

		//
		// Potential Steps
		//
		DACVoltageSet(0, i16Potential_Step[0], false);	// WEs
		DACVoltageSet(5, i16Potential_Step[0], true);	// ORP
		DEBUG_PRINT(UARTprintf("Potential step %d... \n", 1);)

		if(gABoard >= AV7_3)
		{
#ifndef MIX_WHILE_CLEANING
			userDelay(5000, 0);
#ifdef PUMP_CLEAN
			PumpStepperRunStepSpeed_AbortReady(FW, 1000, 6000);
#endif
#else
			g_TimerInterruptFlag = 0;
			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 5); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);
			// Assuming we are 250 steps ahead of 0 when we enter cleaning, rather move it forward than backwards to keep arrays and reference connected
			PumpStepperRunStepSpeed_AbortReady(FW, 300, 3000);

			while(g_TimerInterruptFlag == 0)
				PumpStepperMix(BW, 600, 3000, 1);
#endif
		}
		else
		{
			fI_WE = CurrentTimeRead(0, ADC4_CS_B, 5, i16Potential_Step[0], 1, .1);	// nA
			DEBUG_PRINT(UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)
		}


		// Skip first step, cleaning solutions first step is around 0, sometimes positive sometimes negative
		//	if(fI_WE < 0)
		//	{
		//		UARTprintf("Current should be positive!\n");
		//		gui32Error |= CL_CLEANING_OUT_OF_RANGE;
		//		update_Error();
		//	}

		for(step = 1; step < 7; step++)
		{
			DACVoltageSet(0, i16Potential_Step[step], false);	// WEs
			DACVoltageSet(5, i16Potential_Step[step], true);	// ORP
			DEBUG_PRINT(UARTprintf("Potential step %d... \n", (step + 1));)

			if(gABoard >= AV7_3)
			{
#ifndef MIX_WHILE_CLEANING
#ifndef ANODIC_CLEANING
				userDelay(3000, 0);
#else
				if(step == 6)
				{
					IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);
					fI_WE = CurrentTimeRead(0, ADC4_CS_B, 3, i16Potential_Step[step], 1, .1);	// nA
					DEBUG_PRINT(UARTprintf("Anodic current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)
				}
				else
					userDelay(3000, 0);
#endif
#else
				g_TimerInterruptFlag = 0;
				TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 3); // Set periodic timer
				TimerEnable(TIMER0_BASE, TIMER_A);

				while(g_TimerInterruptFlag == 0)
					PumpStepperMix(BW, 600, 3000, 1);
				g_TimerInterruptFlag = 0;
#endif

				// Only break out of this function after a negative step
				if(step % 2 != 0)
					if((gui32Error & ABORT_ERRORS) != 0)
						break;

#ifdef PUMP_CLEAN
				PumpStepperRunStepSpeed_AbortReady(FW, 1000, 6000);
#endif
			}
			else
			{
				fI_WE = CurrentTimeRead(0, ADC4_CS_B, 3, i16Potential_Step[step], 1, .1);	// nA
				DEBUG_PRINT(UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)

				if((step % 2) == 0)	// Positive step
				{
					if(fI_WE[1] < -500)
					{
						DEBUG_PRINT(UARTprintf("Current should be positive!\n");)
							gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}
				}
				else	// Negative step
				{
					if(fI_WE[1] > 0)
					{
						DEBUG_PRINT(UARTprintf("Current should be negative!\n");)
							gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}

					if((gui32Error & ABORT_ERRORS) != 0)
						break;
				}
			}
		}

		//
		// Cathodic Cleaning
		//
		if((gui32Error & ABORT_ERRORS) == 0)
		{
			DACVoltageSet(0, i16V_cathodic_clean, false);	// WEs
			DACVoltageSet(5, i16V_cathodic_clean, true);	// ORP

			DEBUG_PRINT(UARTprintf("Cathodic Cleaning: \n");)
			//		if(gABoard >= AV7_3)
			//		{
			//			userDelay(40000, 1);
			//			IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);
			//			userDelay(2000, 1);
			//			fI_WE = CurrentTimeRead(0, ADC4_CS_B, 3, i16V_cathodic_clean, 1, .1);
			//		}
			//		else
			//		{
			//			fI_WE = CurrentTimeRead(0, ADC4_CS_B, 45, i16V_cathodic_clean, 1, .1);
			//		}

			IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);
			fI_WE = CurrentTimeRead(0, ADC4_CS_B, 90, i16V_cathodic_clean, 1, 1);


			DEBUG_PRINT(UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)
			if(fI_WE[1] < -7000 || fI_WE[1] > 0)
			{
				DEBUG_PRINT(UARTprintf("Current should be between -7000 and 0 nA!\n");)
					gui32Error |= CL_CLEANING_OUT_OF_RANGE;
				update_Error();
			}
		}

		// Turn off cathodic cleaning voltages
		DACVoltageSet(0, 0, false);	// WEs
		DACVoltageSet(5, 0, true);	// ORP
		IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);

		// Disconnect ORP sensor from driving circuitry
		IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 0);
		IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 0);

#ifndef MEMORY_V5
		// Write 8 bytes to save both the starting and final currents
		MemoryWrite(page, OFFSET_CLEAN_CATH_START, 8, (uint8_t *) fI_WE);
#endif

		//
		// Anodic Oxide Rebuild
		//
		float WEVoltages[5];
#ifndef MEMORY_V5
		int16_t Start_V[5];
#endif

		if(Oxide_Rebuild > 0)
		{
			if((gui32Error & ABORT_ERRORS) == 0 && Oxide_Rebuild == 1)
			{
				DEBUG_PRINT(
						UARTprintf("Rebuilding Oxide for %d seconds... \n", (int) Rebuild_time);
				UARTprintf("Current at %d nA... \n", (int) (fI_drive * 1000));
				)

					// Connect all electrodes separately to drive current through each
					IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

				float Voltage;
				EEPROMRead((uint32_t *) &Voltage, OFFSET_AMP_I_VSET, 4);
				if(Voltage == Voltage && Voltage <= 3000 && Voltage >= 0)
				{

					if(fI_drive != 3)
					{
						Voltage = 3000 - fI_drive / 3.0 * (3000 - Voltage);
						DEBUG_PRINT(UARTprintf("Vset pulled from EEPROM, calculated %d uV!\n", (int) (Voltage * 1000));)
					}
					else
					{
						DEBUG_PRINT(UARTprintf("Vset pulled from EEPROM, theory 1410, using %d uV!\n", (int) (Voltage * 1000));)
					}
				}
				else
				{
					DEBUG_PRINT(UARTprintf("Vset not saved to EEPROM, calculating based on theory!\n");)
						if(gABoard >= ARV1_0B)
							Voltage = 3000 - fI_drive * 180;//180; // mV // Op-amp hardware
						else if(gABoard >= AV6_4)
							Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays
						else if(gABoard >= AV6)
							//		Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays, Resistor changed from 430 kOhms to 30 kOhms to allow current for BES rinse
							Voltage = 3000 - fI_drive * 430; // mV // Op-amp hardware driving 5 arrays
						else
							Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware with all arrays tied together
				}

				//		DACVoltageSet(1, Voltage, false);	// Gold array
				DACVoltageSet(2, Voltage, true);	// 5 platinum arrays

				int j;
				TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 1); // Set periodic timer
				TimerEnable(TIMER1_BASE, TIMER_A);
				for(i = 0; i < Rebuild_time; i++)
				{
					while(g_TimerPeriodicInterruptFlag == 0);
					g_TimerPeriodicInterruptFlag = 0;

					DEBUG_PRINT(UARTprintf("Voltages after %d second(s) are: ", (i + 1));)
					for(j = 0; j < 5; j++)
					{
						WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
						DEBUG_PRINT(UARTprintf("%d, ", (int) WEVoltages[j]);)
#ifndef MEMORY_V5
						if(i == 0)
							Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
					}
					DEBUG_PRINT(UARTprintf("\n");)

					// Program check for specific value range
					if(i == (Rebuild_time - 1))
					{
						if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 500))
						{
							DEBUG_PRINT(UARTprintf("Voltages should be closer together!\n");)
								gui32Error |= CL_CLEANING_OUT_OF_RANGE;
							update_Error();
						}

						if(FindArrayMax(WEVoltages, 5) > 2000 || FindArrayMin(WEVoltages, 5) < 1000)
						{
							DEBUG_PRINT(UARTprintf("Voltages outside expected range!\n");)
								gui32Error |= CL_CLEANING_OUT_OF_RANGE;
							update_Error();
						}
					}

					if((gui32Error & ABORT_ERRORS) != 0)
						break;
				}
				TimerDisable(TIMER1_BASE, TIMER_A);
				g_TimerPeriodicInterruptFlag = 0;

				//	DACVoltageSet(1, 3000, false);	// Gold array
				DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
				uint8_t Rebuild_time = 10;
				MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Rebuild_time);
#endif
			}
			if((gui32Error & ABORT_ERRORS) == 0 && Oxide_Rebuild == 2)
			{
				DEBUG_PRINT(UARTprintf("Rebuilding Oxide until a set mV... \n");)

					// Connect all electrodes separately to drive current through each
					IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

				float Voltage;
				EEPROMRead((uint32_t *) &Voltage, OFFSET_AMP_I_VSET, 4);
				if(Voltage == Voltage && Voltage <= 3000 && Voltage >= 0)
				{
					DEBUG_PRINT(UARTprintf("Vset pulled from EEPROM, theory 1410, using %d uV!\n", (int) (Voltage * 1000));)
				}
				else
				{
					DEBUG_PRINT(UARTprintf("Vset not saved to EEPROM, calculating based on theory!\n");)
						if(gABoard >= AV6_4)
							Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays
						else if(gABoard >= AV6)
							//		Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays, Resistor changed from 430 kOhms to 30 kOhms to allow current for BES rinse
							Voltage = 3000 - fI_drive * 430; // mV // Op-amp hardware driving 5 arrays
						else
							Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware with all arrays tied together
				}

				//		DACVoltageSet(1, Voltage, false);	// Gold array
				DACVoltageSet(2, Voltage, true);	// 5 platinum arrays

				int j;
				TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * .1); // Set periodic timer
				TimerEnable(TIMER1_BASE, TIMER_A);

				i = 0;
				uint8_t check = 1;
				float Avg = 0;
				while((i + 1) < 30 || (i < 300 && check == 1 && Avg < 1470))	// Set max of 30 seconds oxide rebuild TODO: Set the average mV to rebuild to in this while loop
				{
					while(g_TimerPeriodicInterruptFlag == 0);
					g_TimerPeriodicInterruptFlag = 0;

					DEBUG_PRINT(
							if((i + 1) % 10 == 0)
								UARTprintf("Voltages after %d second(s) are: ", (i + 1)/10);
					)

					for(j = 0; j < 5; j++)
					{
						WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
						DEBUG_PRINT(
								if((i + 1) % 10 == 0)
									UARTprintf("%d, ", (int) WEVoltages[j]);
						)
#ifndef MEMORY_V5
						if(i == 0)
							Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
					}
					DEBUG_PRINT(
							if((i + 1) % 10 == 0)
								UARTprintf("\n");
					)

					// Check after 3 seconds that things look ok, if they do continue until the set voltage is reached
					if((i + 1) == 30)
					{
						if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 500) || FindArrayMax(WEVoltages, 5) > 1600 || FindArrayMin(WEVoltages, 5) < 750)
						{
							DEBUG_PRINT(UARTprintf("Voltages should be closer together or aren't in the right range! Leaving Oxide Rebuild!\n");)
								check = 0;
							gui32Error |= CL_CLEANING_OUT_OF_RANGE;
							update_Error();
						}
					}
					if((i + 1) > 30)	// 3 seconds have elapsed, start watching current to reach level
					{
						uint8_t index;
						Avg = 0;
						for(index = 0; index < 5; index++)
						{
							Avg += WEVoltages[index];
						}
						Avg /= 5;
					}

					i++;

					if((gui32Error & ABORT_ERRORS) != 0)
						break;
				}
				TimerDisable(TIMER1_BASE, TIMER_A);
				g_TimerPeriodicInterruptFlag = 0;

				//	DACVoltageSet(1, 3000, false);	// Gold array
				DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
				uint8_t Rebuild_time = (i / 10);
				MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Rebuild_time);
#endif
			}

		}

		SysCtlDelay(SysCtlClockGet()/6);	// Wait 1/2 second after turning off current to allow capacitor to discharge

		// Let the working electrodes float after cleaning
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);

		//	// Let Metals electrode float after cleaning
		//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 0);
		//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 0);
		//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 0);

		// Let reference and counter electrodes float
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

#ifndef MEMORY_V5
		int16_t End_V[5];	// Changing voltages to int16_t to save space in memory
		for(i = 0; i < 5; i++)
			End_V[i] = WEVoltages[i];

		MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_START, 10, (uint8_t *) Start_V);
		MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_FINAL, 10, (uint8_t *) End_V);
#endif

		DEBUG_PRINT(UARTprintf("Cleaning completed! \n");)
	}

}

//**************************************************************************
// Runs the cleaning procedure for all Amperometric arrays and ORP sensor
// Parameters: 	Ref_drift; reference drift adjustment in mV
//				Oxide_Rebuild; Flag 0 or 1; to rebuild the oxide layer
// Outputs:		false if error occurs; current outside of expected range
//				true if cleaning successfully completed
//**************************************************************************
void CleanAmperometrics_CurrentLimited(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number, uint8_t Oxide_Rebuild)
{
	uint16_t page = 0;
#ifndef MEMORY_V5
	if(Cal_Number != 0)
		page = Find_Cal_page(Cal_Number);
	if(Test_Number != 0)
		page = Find_Test_page(Test_Number);
#endif

	int16_t Positive_Step = 1000;
	int16_t Negative_Step = -700;
	float Clean_pH = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
	if(Clean_pH < 7)
	{
		Negative_Step = -650;
		DEBUG_PRINT(UARTprintf("pH 6 Clean, adjusting cleaning voltage!\n");)
	}
	else if(Clean_pH > 8.5)
	{
		Positive_Step = 1000;
		Negative_Step = -800;
		DEBUG_PRINT(UARTprintf("pH 9 Clean, adjusting cleaning voltage!\n");)
	}


	uint8_t step;
	float * fI_WE; 	// Starting and Ending Current flowing through WEs, nA
//	int16_t i16Potential_Step[7] = {1300, -500, 1300, -500, 1300, -500, 1300}; // mV	// Bicarbonate rinse cleaning numbers
//	int16_t i16Potential_Step[7] = {1100, -800, 1100, -800, 1100, -800, 1100}; // mV	// 11/14/2019 HEPES rinse
//	int16_t i16Potential_Step[7] = {800, -700, 800, -700, 800, -700, 800}; // mV	// Changed 10/9/2019 HEPES rinse
//	int16_t i16Potential_Step[7] = {1200, -250, 1200, -250, 1200, -250, 1200}; // mV	// Nitric Acid
//	int16_t i16Potential_Step[7] = {1000, -250, 1000, -250, 1000, -250, 1000}; // mV	// T1
	int16_t i16Potential_Step[7] = {Positive_Step - Ref_drift, Negative_Step - Ref_drift, Positive_Step - Ref_drift, Negative_Step - Ref_drift, Positive_Step - Ref_drift, Negative_Step - Ref_drift, Positive_Step - Ref_drift}; // mV	// 2/13/2020 Cleaning solution
//	int16_t i16V_cathodic_clean = -300; // mV	// Bicarbonate rinse cleaning numbers
//	int16_t i16V_cathodic_clean = -600; // mV	// 11/14/2019 HEPES rinse
//	int16_t i16V_cathodic_clean = -700; // mV	// Changed 10/9/2019 HEPES rinse
//	int16_t i16V_cathodic_clean = -250; // mV	// Nitric Acid
//	int16_t i16V_cathodic_clean = -250; // mV	// T1
	int16_t i16V_cathodic_clean = Negative_Step - Ref_drift; // mV	// 2/13/2020 Cleaning solution

	float fI_drive = 0.7;	// uA to drive through each electrode when rebuilding oxide
	if(gABoard >= AV6_4)
		fI_drive = 3.0;
//		fI_drive = 1.0;	// Changed 10/9/2019

	float Rebuild_time = 10;	// # of seconds to drive current when rebuilding oxide

	DEBUG_PRINT(UARTprintf("Cleaning Amperometrics using steps and oxide rebuild... \n");)

	int i;

	// Connect all electrodes together for potential steps
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);
	if(gABoard >= AV7_3)
		IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);

//	// Connect metals electrodes for potential steps
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 1);

	// Set ORP switches to voltage setting and current reading position
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 1);
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 1);

	// Set RE + CE to amperometric loop
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

	//
	// Potential Steps
	//
	DACVoltageSet(0, i16Potential_Step[0], false);	// WEs
	DACVoltageSet(5, i16Potential_Step[0], true);	// ORP
	DEBUG_PRINT(UARTprintf("Potential step %d... \n", 1);)

	if(gABoard >= AV7_3)
	{
#ifndef MIX_WHILE_CLEANING
		userDelay(5000, 0);
#ifdef PUMP_CLEAN
		PumpStepperRunStepSpeed_AbortReady(FW, 1000, 6000);
#endif
#else
		g_TimerInterruptFlag = 0;
		TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 5); // Set periodic timer
		TimerEnable(TIMER0_BASE, TIMER_A);
		// Assuming we are 250 steps ahead of 0 when we enter cleaning, rather move it forward than backwards to keep arrays and reference connected
		PumpStepperRunStepSpeed_AbortReady(FW, 300, 3000);

		while(g_TimerInterruptFlag == 0)
			PumpStepperMix(BW, 600, 3000, 1);
#endif
	}
	else
	{
		fI_WE = CurrentTimeRead(0, ADC4_CS_B, 5, i16Potential_Step[0], 1, .1);	// nA
		DEBUG_PRINT(UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)
	}

	// Skip first step, cleaning solutions first step is around 0, sometimes positive sometimes negative
//	if(fI_WE < 0)
//	{
//		UARTprintf("Current should be positive!\n");
//		gui32Error |= CL_CLEANING_OUT_OF_RANGE;
//		update_Error();
//	}

	for(step = 1; step < 7; step++)
	{
		DACVoltageSet(0, i16Potential_Step[step], false);	// WEs
		DACVoltageSet(5, i16Potential_Step[step], true);	// ORP
		DEBUG_PRINT(UARTprintf("Potential step %d... \n", (step + 1));)

		if(gABoard >= AV7_3)
		{
#ifndef MIX_WHILE_CLEANING
#ifndef ANODIC_CLEANING
			userDelay(3000, 0);
#else
			if(step == 6)
			{
				IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);
				fI_WE = CurrentTimeRead(0, ADC4_CS_B, 3, i16Potential_Step[step], 1, .1);	// nA
				DEBUG_PRINT(UARTprintf("Anodic current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)
			}
			else
				userDelay(3000, 0);
#endif
#else
			g_TimerInterruptFlag = 0;
			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 3); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);

			while(g_TimerInterruptFlag == 0)
				PumpStepperMix(BW, 600, 3000, 1);
			g_TimerInterruptFlag = 0;
#endif

			// Only break out of this function after a negative step
			if(step % 2 != 0)
				if((gui32Error & ABORT_ERRORS) != 0)
					break;

#ifdef PUMP_CLEAN
		PumpStepperRunStepSpeed_AbortReady(FW, 1000, 6000);
#endif
		}
		else
		{
			fI_WE = CurrentTimeRead(0, ADC4_CS_B, 3, i16Potential_Step[step], 1, .1);	// nA
			DEBUG_PRINT(UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)

			if((step % 2) == 0)	// Positive step
			{
				if(fI_WE[1] < -500)
				{
					DEBUG_PRINT(UARTprintf("Current should be positive!\n");)
					gui32Error |= CL_CLEANING_OUT_OF_RANGE;
					update_Error();
				}
			}
			else	// Negative step
			{
				if(fI_WE[1] > 0)
				{
					DEBUG_PRINT(UARTprintf("Current should be negative!\n");)
					gui32Error |= CL_CLEANING_OUT_OF_RANGE;
					update_Error();
				}

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}
		}
	}

	//
	// Cathodic Cleaning
	//
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		DACVoltageSet(0, i16V_cathodic_clean, false);	// WEs
		DACVoltageSet(5, i16V_cathodic_clean, true);	// ORP

		DEBUG_PRINT(UARTprintf("Cathodic Cleaning: \n");)

		if(gABoard >= AV7_3)
		{
			IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);

#ifndef MIX_WHILE_CLEANING
			userDelay(3000, 1);
			if(Clean_pH < 7)
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 87, i16V_cathodic_clean, 1, .1, -2500, page);
			else
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 87, i16V_cathodic_clean, 1, .1, -1800, page);

#ifdef PUMP_CLEAN
		PumpStepperRunStepSpeed_AbortReady(FW, 1000, 6000);
#endif

#else
			// Set time for 3 seconds, mix during this without reading reading current, then take first current measurement
			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 3); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;

			while(g_TimerInterruptFlag == 0)
				PumpStepperMix(BW, 600, 3000, 1);

			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 87); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;

			// Take starting current value
			float fI_Watch[2];
			fI_Watch[0] = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), i16V_cathodic_clean, 1);
			fI_Watch[1] = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), i16V_cathodic_clean, 1);

			while(abs_val(fI_Watch[1]) > abs_val(-1800) && g_TimerInterruptFlag == 0)
			{
				PumpStepperMix(BW, 600, 3000, 1);
				fI_Watch[1] = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), i16V_cathodic_clean, 1);
				DEBUG_PRINT(UARTprintf("Current: %d nA\n", (int) fI_Watch[1]);)
			}
			TimerDisable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;
			fI_WE = fI_Watch;
#ifndef	MEMORY_V5
			if(page > 0)
			{
				uint8_t Time_to_save = TimerLoadGet(TIMER0_BASE, TIMER_A) / SysCtlClockGet();
				MemoryWrite(page, OFFSET_CLEAN_CATH_TIME, 1, &Time_to_save);
			}
#endif
#endif
		}
		else
		{
			if(Clean_pH < 7)
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 90, i16V_cathodic_clean, 1, .1, -2500, page);
			else
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 90, i16V_cathodic_clean, 1, .1, -1800, page);
		}

		DEBUG_PRINT(UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)
		if(fI_WE[1] < -7000 || fI_WE[1] > 0)
		{
			DEBUG_PRINT(UARTprintf("Current should be between -7000 and 0 nA!\n");)
			gui32Error |= CL_CLEANING_OUT_OF_RANGE;
			update_Error();
		}
	}

	// Turn off cathodic cleaning voltages
	DACVoltageSet(0, 0, false);	// WEs
	DACVoltageSet(5, 0, true);	// ORP
	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);

	// Disconnect ORP sensor from driving circuitry
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 0);
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 0);

#ifndef MEMORY_V5
	// Write 8 bytes to save both the starting and final currents
	MemoryWrite(page, OFFSET_CLEAN_CATH_START, 8, (uint8_t *) fI_WE);
#endif

	//
	// Anodic Oxide Rebuild
	//
	float WEVoltages[5];
#ifndef MEMORY_V5
	int16_t Start_V[5];
#endif

	if(Oxide_Rebuild > 0)
	{
		if((gui32Error & ABORT_ERRORS) == 0 && Oxide_Rebuild == 1)
		{
			DEBUG_PRINT(UARTprintf("Rebuilding Oxide for %d seconds... \n", (int) Rebuild_time);)

			// Connect all electrodes separately to drive current through each
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

			float Voltage;
			EEPROMRead((uint32_t *) &Voltage, OFFSET_AMP_I_VSET, 4);
			if(Voltage == Voltage && Voltage <= 3000 && Voltage >= 0)
			{
				DEBUG_PRINT(UARTprintf("Vset pulled from EEPROM, theory 1410, using %d uV!\n", (int) (Voltage * 1000));)
			}
			else
			{
				DEBUG_PRINT(UARTprintf("Vset not saved to EEPROM, calculating based on theory!\n");)
				if(gABoard >= AV6_4)
					Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays
				else if(gABoard >= AV6)
					//		Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays, Resistor changed from 430 kOhms to 30 kOhms to allow current for BES rinse
					Voltage = 3000 - fI_drive * 430; // mV // Op-amp hardware driving 5 arrays
				else
					Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware with all arrays tied together
			}

			//		DACVoltageSet(1, Voltage, false);	// Gold array
			DACVoltageSet(2, Voltage, true);	// 5 platinum arrays

			int j;
			TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 1); // Set periodic timer
			TimerEnable(TIMER1_BASE, TIMER_A);
			for(i = 0; i < Rebuild_time; i++)
			{
				while(g_TimerPeriodicInterruptFlag == 0);
				g_TimerPeriodicInterruptFlag = 0;

				DEBUG_PRINT(UARTprintf("Voltages after %d second(s) are: ", (i + 1));)
				for(j = 0; j < 5; j++)
				{
					WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
					DEBUG_PRINT(UARTprintf("%d, ", (int) WEVoltages[j]);)
#ifndef MEMORY_V5
					if(i == 0)
						Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
				}
				DEBUG_PRINT(UARTprintf("\n");)

				// Program check for specific value range
				if(i == (Rebuild_time - 1))
				{
					if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 300) || FindArrayMax(WEVoltages, 5) > 2000 || FindArrayMin(WEVoltages, 5) < 1000)
					{
						DEBUG_PRINT(UARTprintf("Voltages should be closer together!\n");)
						gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}
				}

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}
			TimerDisable(TIMER1_BASE, TIMER_A);
			g_TimerPeriodicInterruptFlag = 0;

			//	DACVoltageSet(1, 3000, false);	// Gold array
			DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
			uint8_t Mem_Rebuild_time = Rebuild_time;
			MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Mem_Rebuild_time);
#endif
		}
		if((gui32Error & ABORT_ERRORS) == 0 && Oxide_Rebuild == 2)
		{
			DEBUG_PRINT(UARTprintf("Rebuilding Oxide until a set mV... \n");)

			// Connect all electrodes separately to drive current through each
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

			float Voltage;
			EEPROMRead((uint32_t *) &Voltage, OFFSET_AMP_I_VSET, 4);
			if(Voltage == Voltage && Voltage <= 3000 && Voltage >= 0)
			{
				DEBUG_PRINT(UARTprintf("Vset pulled from EEPROM, theory 1410, using %d uV!\n", (int) (Voltage * 1000));)
			}
			else
			{
				DEBUG_PRINT(UARTprintf("Vset not saved to EEPROM, calculating based on theory!\n");)
				if(gABoard >= AV6_4)
					Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays
				else if(gABoard >= AV6)
					//		Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays, Resistor changed from 430 kOhms to 30 kOhms to allow current for BES rinse
					Voltage = 3000 - fI_drive * 430; // mV // Op-amp hardware driving 5 arrays
				else
					Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware with all arrays tied together
			}

			//		DACVoltageSet(1, Voltage, false);	// Gold array
			DACVoltageSet(2, Voltage, true);	// 5 platinum arrays

#ifndef MIX_WHILE_CLEANING
			int j;
			TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * .1); // Set periodic timer
			TimerEnable(TIMER1_BASE, TIMER_A);

			i = 0;
			uint8_t check = 1;
			float Avg = 0;
			uint16_t mV_target = 1470;
			if(Clean_pH < 7)
				mV_target = 1520;
			while((i + 1) < 30 || (i < 300 && check == 1 && Avg < mV_target))	// Set max of 30 seconds oxide rebuild TODO: Set the average mV to rebuild to in this while loop
			{
				while(g_TimerPeriodicInterruptFlag == 0);
				g_TimerPeriodicInterruptFlag = 0;

				if((i + 1) % 10 == 0)
				{
					DEBUG_PRINT(UARTprintf("Voltages after %d second(s) are: ", (i + 1)/10);)
				}
				for(j = 0; j < 5; j++)
				{
					WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
					DEBUG_PRINT(
					if((i + 1) % 10 == 0)
						UARTprintf("%d, ", (int) WEVoltages[j]);
					)
#ifndef MEMORY_V5
					if(i == 0)
						Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
				}
				DEBUG_PRINT(
				if((i + 1) % 10 == 0)
					UARTprintf("\n");
				)

				// Check after 3 seconds that things look ok, if they do continue until the set voltage is reached
				if((i + 1) == 30)
				{
					if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 300) || FindArrayMax(WEVoltages, 5) > 1600 || FindArrayMin(WEVoltages, 5) < 750)
					{
						DEBUG_PRINT(UARTprintf("Voltages should be closer together or aren't in the right range! Leaving Oxide Rebuild!\n");)
						check = 0;
						gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}
				}
				if((i + 1) > 30)	// 3 seconds have elapsed, start watching current to reach level
				{
					uint8_t index;
					Avg = 0;
					for(index = 0; index < 5; index++)
					{
						Avg += WEVoltages[index];
					}
					Avg /= 5;
				}

				i++;

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}
			TimerDisable(TIMER1_BASE, TIMER_A);
			g_TimerPeriodicInterruptFlag = 0;

			//	DACVoltageSet(1, 3000, false);	// Gold array
			DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
			uint8_t Rebuild_time = (i / 10);
			MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Rebuild_time);
#endif
#else
			g_TimerInterruptFlag = 0;
			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 30); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);

			int j;
			uint8_t check = 1;
			float Avg = 0;
			i = 0;
			while(g_TimerInterruptFlag == 0 && (check == 1 && Avg < 1470))
			{
				PumpStepperMix(BW, 600, 3000, 1);
				i++;

				DEBUG_PRINT(UARTprintf("Voltages after %d mix are: ", i);)
				for(j = 0; j < 5; j++)
				{
					WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
					DEBUG_PRINT(UARTprintf("%d, ", (int) WEVoltages[j]);)
#ifndef MEMORY_V5
					if(i == 1)
						Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
				}
				DEBUG_PRINT(UARTprintf("\n");)

				// Check after 3 mixes that things look ok, if they do continue until the set voltage is reached
				if(i > 3)
				{
					if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 150) || FindArrayMax(WEVoltages, 5) > 1600 || FindArrayMin(WEVoltages, 5) < 750)
					{
						DEBUG_PRINT(UARTprintf("Voltages should be closer together or aren't in the right range! Leaving Oxide Rebuild!\n");)
						check = 0;
						gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}
				}
				if(i > 3)	// 3 mixes have elapsed, start watching current to reach level
				{
					uint8_t index;
					Avg = 0;
					for(index = 0; index < 5; index++)
					{
						Avg += WEVoltages[index];
					}
					Avg /= 5;
				}

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}
			TimerDisable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;

			//	DACVoltageSet(1, 3000, false);	// Gold array
			DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
			uint8_t Rebuild_time = TimerLoadGet(TIMER0_BASE, TIMER_A) / SysCtlClockGet();
			DEBUG_PRINT(UARTprintf("Rebuild time: %d\n", Rebuild_time);)
			MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Rebuild_time);
#endif
#endif
		}
	}

	SysCtlDelay(SysCtlClockGet()/6);	// Wait 1/2 second after turning off current to allow capacitor to discharge

	// Let the working electrodes float after cleaning
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);

//	// Let Metals electrode float after cleaning
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 0);

	// Let reference and counter electrodes float
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

#ifndef MEMORY_V5
	int16_t End_V[5];	// Changing voltages to int16_t to save space in memory
	for(i = 0; i < 5; i++)
		End_V[i] = WEVoltages[i];

	MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_START, 10, (uint8_t *) Start_V);
	MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_FINAL, 10, (uint8_t *) End_V);
#endif

	DEBUG_PRINT(UARTprintf("Cleaning completed! \n");)
}

#ifdef SWEEP_CLEAN
//**************************************************************************
// Cleans the amperometric arrays by sweeping from high to low back to high
// at 20 mV/sec in 1mV steps
// Parameters: 	Ref_drift; Calculated drift of the reference
//				Cal_Number; Calibration to save cleaning data to; 0 if not saving
//				Test_Number, Test to save cleaning data to; 0 if not saving
// Parameters:	NONE
// Outputs:		NONE
//**************************************************************************
void CleanAmperometricsSweep(int8_t Ref_drift)
{
//#ifndef MEMORY_V5
//	uint16_t page;
//	if(Cal_Number != 0)
//		page = Find_Cal_page(Cal_Number);
//	if(Test_Number != 0)
//		page = Find_Test_page(Test_Number);
//#endif

//	uint8_t step;
	float * fI_WE; 	// Starting and Ending Current flowing through WEs, nA
//	int16_t i16Potential_Step[7] = {1000 - Ref_drift, -700 - Ref_drift, 1000 - Ref_drift, -700 - Ref_drift, 1000 - Ref_drift, -700 - Ref_drift, 1000 - Ref_drift}; // mV	// 2/13/2020 Cleaning solution
//	int16_t i16V_cathodic_clean = -700 - Ref_drift; // mV	// 2/13/2020 Cleaning solution

	UARTprintf("Cleaning Amperometrics by sweeping the potential... \n");

	// Connect all electrodes together for potential steps
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);

	// Set RE + CE to amperometric loop
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

	//
	// Potential Sweep
	//
	TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet() * 1) / 20); // Set periodic timer
	TimerEnable(TIMER1_BASE, TIMER_A);
	int16_t Voltage = 1000 - Ref_drift;

	while(Voltage > (-700 - Ref_drift))
	{
		if((gui32Error & ABORT_ERRORS) != 0)
			break;

		DACVoltageSet(0, Voltage, true);	// WEs
		Voltage--;

		while(g_TimerPeriodicInterruptFlag == 0);
		g_TimerPeriodicInterruptFlag = 0;
	}

	DACVoltageSet(0, Voltage, true);	// WEs
	fI_WE = CurrentTimeRead(0, ADC4_CS_B, 5, Voltage, .15, .05);	// nA
	Voltage++;
	UARTprintf("Current at bottom of negative sweep read: %d nA\n", (int) fI_WE[1]);

	// Reactivate timer here becasue CurrentTimeRead uses it
	TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet() * 1) / 20); // Set periodic timer
	TimerEnable(TIMER1_BASE, TIMER_A);

	while(Voltage <= (1000 - Ref_drift))
	{
		if((gui32Error & ABORT_ERRORS) != 0)
			break;

		DACVoltageSet(0, Voltage, true);	// WEs
		Voltage++;

		while(g_TimerPeriodicInterruptFlag == 0);
		g_TimerPeriodicInterruptFlag = 0;
	}

	fI_WE = CurrentTimeRead(0, ADC4_CS_B, 5, Voltage, .15, .05);	// nA
	UARTprintf("Current at end of positive sweep read: %d nA\n", (int) fI_WE[1]);

	TimerDisable(TIMER1_BASE, TIMER_A);
	g_TimerPeriodicInterruptFlag = 0;
	DACVoltageSet(0, 0, true);	// WEs

	// Turn off cathodic cleaning voltages
	DACVoltageSet(0, 0, false);	// WEs
	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);

	SysCtlDelay(SysCtlClockGet()/6);	// Wait 1/2 second after turning off current to allow capacitor to discharge

	// Let the working electrodes float after cleaning
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);

	// Let reference and counter electrodes float
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

	UARTprintf("Cleaning completed! \n");
}
#endif

////**************************************************************************
//// Runs the cleaning procedure for all Amperometric arrays and ORP sensor
//// Parameters: 	ui8attempt; Attempt at cleaning, always call with 1
//// Outputs:		false if error occurs; current outside of expected range
////				true if cleaning successfully completed
////**************************************************************************
//void CleanAmperometrics(void)
//{
//	UARTprintf("Cleaning Amperometrics using ramp... \n");
//
//	// Connect all electrodes together for potential steps
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);
//
////	// Connect metals electrodes for potential steps
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 1);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 1);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 1);
//
////	// Set ORP switches to voltage setting and current reading position
////	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 1);
////	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 1);
//
//	// Set RE + CE to amperometric loop
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
//	float period = .01;		// Period between each voltage step, (5 mV steps, 500 mV/s rate, period = 5 mV/500 (mV/s) = .01 s)
//	int16_t bottom = -600;	// Minimum voltage to go to on ramp
//	int16_t top = 900;		// Maximum voltage to go to on ramp
//	int8_t step = 5;		// mV / step
//	uint8_t cycles = 39;	// Times to go each direction (39 cycles = 19 times up, 20 times down)
//
//	int16_t V_set = top;	// Start on the positive side
//	uint8_t up = 0;			// Start by decreasing voltage
//
//	float fI_WE;
//
//	TimerLoadSet(TIMER1_BASE, TIMER_A, (period * SysCtlClockGet()));		// Timer set
//	TimerEnable(TIMER1_BASE, TIMER_A);
//	g_TimerPeriodicInterruptFlag = 0;
//
//	uint32_t i;
//	if((gui32Error & ABORT_ERRORS) == 0)
//	{
//		for(i = 0; i < ((top - bottom)/step * cycles) + 1; i++) // Add 1 so it sets at initial state
//		{
//			DACVoltageSet(0, V_set, false);	// WEs
//			//		DACVoltageSet(5, V_set, true);	// ORP
//
//			if(up)
//				V_set += step;
//			else
//				V_set -= step;
//
//			if(V_set >= top)
//				up = 0;
//			else if(V_set <= bottom)
//				up = 1;
//
//			while(g_TimerPeriodicInterruptFlag == 0);
//			g_TimerPeriodicInterruptFlag = 0;
//
//			if((gui32Error & ABORT_ERRORS) != 0)
//				break;
//		}
//
//		//	UARTprintf("V Set = %d\n", V_set);
//		fI_WE = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), bottom, 1) * 1000000;
//		UARTprintf("Current = %d nA\n", (int) (fI_WE));
//
//		if(fI_WE > 0)
//		{
//			UARTprintf("Current should be negative!\n");
//			gui32Error |= CL_CLEANING_OUT_OF_RANGE;
//			update_Error();
//		}
//	}
//
//	if((gui32Error & ABORT_ERRORS) == 0)
//	{
//		for(i = 0; i < ((top - bottom)/step); i++)
//		{
//			DACVoltageSet(0, V_set, false);	// WEs
//			//		DACVoltageSet(5, V_set, true);	// ORP
//
//			if(up)
//				V_set += step;
//			else
//				V_set -= step;
//
//			if(V_set >= top)
//				up = 0;
//			else if(V_set <= bottom)
//				up = 1;
//
//			while(g_TimerPeriodicInterruptFlag == 0);
//			g_TimerPeriodicInterruptFlag = 0;
//
//			if((gui32Error & ABORT_ERRORS) != 0)
//				break;
//		}
//
//		//	UARTprintf("V Set = %d\n", V_set);
//		fI_WE = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), top, 1) * 1000000;
//		UARTprintf("Current = %d nA\n", (int) (fI_WE));
//
//		if(fI_WE < 0)
//		{
//			UARTprintf("Current should be positive!\n");
//			gui32Error |= CL_CLEANING_OUT_OF_RANGE;
//			update_Error();
//		}
//	}
//
//	DACVoltageSet(0, 0, false);	// WEs
//
//	TimerDisable(TIMER1_BASE, TIMER_A);
//	g_TimerInterruptFlag = 0;
//
//	// Let the working electrodes float after cleaning
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
//
////	// Let Metals electrode float after cleaning
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 0);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 0);
////	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 0);
//
//	// Let reference and counter electrodes float
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//	UARTprintf("Cleaning completed! \n");
//}

#ifdef CONST_COND_FREQ
//**************************************************************************
// Function to calculate moving average of conductivity ADC and return the
// max or min, whichever has the greatest magnitude
// Takes a sample every ~11.5 us or ~87 kHz
// Parameters:	ssMovingAvgBufferPtrInit; pointer to start of buffer; &array[0]
//				usBufferSize, size of buffer being pointed at
// Outputs:		Max - Min of signal in uV
//				Used to do, but changed that Absolute value of Max or Min voltage in uV
//				NOT Peak-to-peak voltage
//**************************************************************************
float ConductivityMovingAvg(void)
{
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		// Set SPI communication to capture on falling edge of clock (other ADCs capture on rising edge)
		SSIDisable(SSI1_BASE);
		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
		SSIEnable(SSI1_BASE);

		uint16_t usBufferSize = 5;
		int16_t ssMovingAvgBuffer[5] = {0, 0, 0, 0, 0}; // Array has to be same size as usBufferSize
		int16_t* ssMovingAvgBufferPtr = &ssMovingAvgBuffer[0]; // Create and set a pointer to beginning of buffer

		uint16_t usNoOfPeriods = COND_FREQ;
		uint32_t usNoOfSamples = 87000 / COND_FREQ;
//		uint16_t usNoOfPeriods = 250;
//		uint16_t usNoOfSamples = 87; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
//		uint16_t usNoOfSamples = 174; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
//		uint16_t usNoOfSamples = 348; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
//		uint16_t usNoOfPeriods = 100;
//		uint16_t usNoOfSamples = 870; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
		uint32_t ulDataRx[2];
		uint32_t ulindex2;
		uint32_t ulindex3;
		int16_t ssData;
		int32_t slDataFiltered;
		int32_t slRunSum;
		int32_t slMax;
		int32_t slMin;

		uint32_t delay;
		if(gABoard >= AV6_1)
			delay = (SysCtlClockGet()/3000 * .0013) - 2; // 1.3 us delay for CNV pin pulse
		else
			delay = (SysCtlClockGet()/3000 * .0032) - 2; // 3.2 us delay for CNV pin pulse

		int32_t slSumMax = 0;
		int32_t slSumMin = 0;

//		int32_t abs_max = 0;
//		int32_t abs_min = 0;

		uint8_t attempts = 0;
		while(slSumMax == 0 && slSumMin == 0 && attempts < 3)
		{
			slRunSum = 0;
			attempts++;

#ifdef MCU_ZXR
			if(gABoard >= AV6_1)
				GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, ~IO_COND_ADC_CNV_PIN);				// CNV for ADC3 LOW (Turn low so it can be turned high)
#else
			if(gABoard >= AV6_1)
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);				// CNV for ADC3 LOW (Turn low so it can be turned high)
#endif

			while(SSIDataGetNonBlocking(SSI1_BASE, &ulDataRx[0]))  	{} // Clear FIFO

			// Transmit request to restart conversion on channel 0.
			for(ulindex3 = 0; ulindex3 < usNoOfPeriods; ulindex3++)
			{
				slMax = 0;
				slMin = 0;

				for(ulindex2 = 0; ulindex2 < usNoOfSamples; ulindex2++)
				{
#ifdef MCU_ZXR
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN);				// CNV for ADC3 HIGH (start conversion)
					SysCtlDelay(delay);	// t_conv is min .07us max 3.2us
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, ~IO_COND_ADC_CNV_PIN);				// CNV for ADC3 LOW (acquisition period)

#else
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);				// CNV for ADC3 HIGH (start conversion)
					SysCtlDelay(delay);	// t_conv is min .07us max 3.2us
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);				// CNV for ADC3 LOW (acquisition period)

#endif

					// Start clock to retrieve data (16 clock cycles)
					SSIDataPut(SSI1_BASE, 0x00);
					SSIDataPut(SSI1_BASE, 0x00);
					//			while(SSIBusy(SSI1_BASE))   {}

					SSIDataGet(SSI1_BASE, &ulDataRx[0]);
					SSIDataGet(SSI1_BASE, &ulDataRx[1]);

					// Compose data from received bytes
					ssData = ((ulDataRx[0] << 8) | ulDataRx[1]) & 0xFFFF;

//					if(ssData > abs_max)
//						abs_max = ssData;
//					if(ssData < abs_min)
//						abs_min = ssData;

					// Process data
					slRunSum -= *ssMovingAvgBufferPtr - ssData; //slRunSum - *ssMovingAvgBufferPtr + ssData;
					*ssMovingAvgBufferPtr = ssData;
					ssMovingAvgBufferPtr++; // = ssMovingAvgBufferPtr + 1;
					if (ssMovingAvgBufferPtr >= (&ssMovingAvgBuffer[0] + usBufferSize)) {
						ssMovingAvgBufferPtr = &ssMovingAvgBuffer[0];
					}

					slDataFiltered = slRunSum / usBufferSize;

					if (slDataFiltered > slMax)
						slMax = slDataFiltered;
					if (slDataFiltered < slMin)
						slMin = slDataFiltered;

				}//For loop - NoOfSamples
				if (ulindex3 > 1) {						// skip the first period to avoid error coming from pre-loading buffer with invalid numbers
					slSumMax = slSumMax + slMax;
					slSumMin = slSumMin - abs(slMin);
				}
			}//For loop - NoOfPeriods

			if(slSumMax == 0 && slSumMin == 0)
			{
//				gui32Error |= ADC3_FAIL;
//				update_Error();
				DEBUG_PRINT(
				UARTprintf("Conductivity ADC read back all 0's!\n");
				UARTprintf("Resetting Analog Board!\n");
				)

				AnalogOff();

				SysCtlDelay(SysCtlClockGet()/3);	// Wait to give analog board time to power-down before turning back on

				InitAnalog();
			}
		}

		if(slSumMax == 0 && slSumMin == 0)
		{
			gui32Error |= ADC3_FAIL;
			update_Error();
			DEBUG_PRINT(UARTprintf("Conductivity ADC always read 0's, updating error!\n");)
		}

		float Avg_Max = (float)slSumMax / (usNoOfPeriods - 1); // subtract 1 because we don't sum the first period
		float Avg_Min = (float)slSumMin / (usNoOfPeriods - 1); // subtract 1 because we don't sum the first period

		float Voltage = (Avg_Max - Avg_Min) * 3000000 / 32768;

		// JC: 6/30/2020: Previously we returned either min or max, whichever was bigger, but there is a small offset 0-2mV which makes it not even, switch to returning the full signal, max - min
//		if(Avg_Max > abs_val(Avg_Min))
//			Voltage = Avg_Max * 3000000 / 32768; // uV
//		else
//			Voltage = abs_val(Avg_Min) * 3000000 / 32768; // uV
//
//		UARTprintf("Max voltage read: %d\n", (int) ((float) Avg_Max * 3000000 / 32768));
//		UARTprintf("Min voltage read: %d\n", (int) ((float) Avg_Min * 3000000 / 32768));

#ifdef MCU_ZXR
		if(gABoard >= AV6_1)
			GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN);				// CNV for ADC3 HIGH (Leave this pin high when not collecting data)
#else
		if(gABoard >= AV6_1)
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);				// CNV for ADC3 HIGH (Leave this pin high when not collecting data)
#endif

		return Voltage;
	}

	return 0;
}
#else	// CONST_COND_FREQ
//**************************************************************************
// Function to calculate moving average of conductivity ADC and return the
// max or min, whichever has the greatest magnitude
// Takes a sample every ~11.5 us or ~87 kHz
// Parameters:	ssMovingAvgBufferPtrInit; pointer to start of buffer; &array[0]
//				usBufferSize, size of buffer being pointed at
// Outputs:		Max - Min of signal in uV
//				Used to do, but changed that Absolute value of Max or Min voltage in uV
//				NOT Peak-to-peak voltage
//**************************************************************************
float ConductivityMovingAvg(uint16_t ui16freq)
{
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		// Set SPI communication to capture on falling edge of clock (other ADCs capture on rising edge)
		SSIDisable(SSI1_BASE);
		SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
		SSIEnable(SSI1_BASE);

		uint16_t usBufferSize = 5;
		int16_t ssMovingAvgBuffer[5] = {0, 0, 0, 0, 0}; // Array has to be same size as usBufferSize
		int16_t* ssMovingAvgBufferPtr = &ssMovingAvgBuffer[0]; // Create and set a pointer to beginning of buffer

		uint16_t usNoOfPeriods = ui16freq;
		uint32_t usNoOfSamples = 87000 / ui16freq;
//		uint16_t usNoOfPeriods = 250;
//		uint16_t usNoOfSamples = 87; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
//		uint16_t usNoOfSamples = 174; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
//		uint16_t usNoOfSamples = 348; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
//		uint16_t usNoOfPeriods = 100;
//		uint16_t usNoOfSamples = 870; // sampling period is ~11us ~87 kHz		// Samples in ONE signal period!!! 270 for 100 Hz
		uint32_t ulDataRx[2];
		uint32_t ulindex2;
		uint32_t ulindex3;
		int16_t ssData;
		int32_t slDataFiltered;
		int32_t slRunSum;
		int32_t slMax;
		int32_t slMin;

		uint32_t delay;
		if(gABoard >= AV6_1)
			delay = (SysCtlClockGet()/3000 * .0013) - 2; // 1.3 us delay for CNV pin pulse
		else
			delay = (SysCtlClockGet()/3000 * .0032) - 2; // 3.2 us delay for CNV pin pulse

		int32_t slSumMax = 0;
		int32_t slSumMin = 0;

//		int32_t abs_max = 0;
//		int32_t abs_min = 0;

		uint8_t attempts = 0;
		while(slSumMax == 0 && slSumMin == 0 && attempts < 3)
		{
			slRunSum = 0;
			attempts++;

#ifdef MCU_ZXR
			if(gABoard >= AV6_1)
				GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, ~IO_COND_ADC_CNV_PIN);				// CNV for ADC3 LOW (Turn low so it can be turned high)
#else
			if(gABoard >= AV6_1)
				GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);				// CNV for ADC3 LOW (Turn low so it can be turned high)
#endif

			while(SSIDataGetNonBlocking(SSI1_BASE, &ulDataRx[0]))  	{} // Clear FIFO

			// Transmit request to restart conversion on channel 0.
			for(ulindex3 = 0; ulindex3 < usNoOfPeriods; ulindex3++)
			{
				slMax = 0;
				slMin = 0;

				for(ulindex2 = 0; ulindex2 < usNoOfSamples; ulindex2++)
				{
#ifdef MCU_ZXR
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN);				// CNV for ADC3 HIGH (start conversion)
					SysCtlDelay(delay);	// t_conv is min .07us max 3.2us
					GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, ~IO_COND_ADC_CNV_PIN);				// CNV for ADC3 LOW (acquisition period)

#else
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);				// CNV for ADC3 HIGH (start conversion)
					SysCtlDelay(delay);	// t_conv is min .07us max 3.2us
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);				// CNV for ADC3 LOW (acquisition period)

#endif

					// Start clock to retrieve data (16 clock cycles)
					SSIDataPut(SSI1_BASE, 0x00);
					SSIDataPut(SSI1_BASE, 0x00);
					//			while(SSIBusy(SSI1_BASE))   {}

					SSIDataGet(SSI1_BASE, &ulDataRx[0]);
					SSIDataGet(SSI1_BASE, &ulDataRx[1]);

					// Compose data from received bytes
					ssData = ((ulDataRx[0] << 8) | ulDataRx[1]) & 0xFFFF;

//					if(ssData > abs_max)
//						abs_max = ssData;
//					if(ssData < abs_min)
//						abs_min = ssData;

					// Process data
					slRunSum -= *ssMovingAvgBufferPtr - ssData; //slRunSum - *ssMovingAvgBufferPtr + ssData;
					*ssMovingAvgBufferPtr = ssData;
					ssMovingAvgBufferPtr++; // = ssMovingAvgBufferPtr + 1;
					if (ssMovingAvgBufferPtr >= (&ssMovingAvgBuffer[0] + usBufferSize)) {
						ssMovingAvgBufferPtr = &ssMovingAvgBuffer[0];
					}

					slDataFiltered = slRunSum / usBufferSize;

					if (slDataFiltered > slMax)
						slMax = slDataFiltered;
					if (slDataFiltered < slMin)
						slMin = slDataFiltered;

				}//For loop - NoOfSamples
				if (ulindex3 > 1) {						// skip the first period to avoid error coming from pre-loading buffer with invalid numbers
					slSumMax = slSumMax + slMax;
					slSumMin = slSumMin - abs(slMin);
				}
			}//For loop - NoOfPeriods

			if(slSumMax == 0 && slSumMin == 0)
			{
//				gui32Error |= ADC3_FAIL;
//				update_Error();
				DEBUG_PRINT(
				UARTprintf("Conductivity ADC read back all 0's!\n");
				UARTprintf("Resetting Analog Board!\n");
				)

				AnalogOff();

				SysCtlDelay(SysCtlClockGet()/3);	// Wait to give analog board time to power-down before turning back on

				InitAnalog();
			}
		}

		if(slSumMax == 0 && slSumMin == 0)
		{
			gui32Error |= ADC3_FAIL;
			update_Error();
			DEBUG_PRINT(UARTprintf("Conductivity ADC always read 0's, updating error!\n");)
		}

		float Avg_Max = (float)slSumMax / (usNoOfPeriods - 1); // subtract 1 because we don't sum the first period
		float Avg_Min = (float)slSumMin / (usNoOfPeriods - 1); // subtract 1 because we don't sum the first period

		float Voltage = (Avg_Max - Avg_Min) * 3000000 / 32768;

		// JC: 6/30/2020: Previously we returned either min or max, whichever was bigger, but there is a small offset 0-2mV which makes it not even, switch to returning the full signal, max - min
//		if(Avg_Max > abs_val(Avg_Min))
//			Voltage = Avg_Max * 3000000 / 32768; // uV
//		else
//			Voltage = abs_val(Avg_Min) * 3000000 / 32768; // uV
//
//		UARTprintf("Max voltage read: %d\n", (int) ((float) Avg_Max * 3000000 / 32768));
//		UARTprintf("Min voltage read: %d\n", (int) ((float) Avg_Min * 3000000 / 32768));

#ifdef MCU_ZXR
		if(gABoard >= AV6_1)
			GPIOPinWrite(IO_COND_ADC_CNV_BASE, IO_COND_ADC_CNV_PIN, IO_COND_ADC_CNV_PIN);				// CNV for ADC3 HIGH (Leave this pin high when not collecting data)
#else
		if(gABoard >= AV6_1)
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);				// CNV for ADC3 HIGH (Leave this pin high when not collecting data)
#endif

		return Voltage;
	}

	return 0;
}
#endif// CONST_COND_FREQ

#ifndef COND_SOLUTION_STRUCT
//********************************************************************************
// Reads conductivity slopes from memory and measures conductivity by switching
// through ranges until it finds correct range, returns measured conductivity,
// NOT a temperature corrected conductivity
// Created: 2/5/2020
// Inputs:	Cond_EEP_Rinse; Conductivity of Rinse at 25C
//			Cond_EEP_Cal_2; Conductivity of Cal 2 at 25C
//			Test_Number; Putting in test number will save raw data, 0 will skip save
// Outputs: Conductivity; returns measured conductivity, NOT temperature corrected
//********************************************************************************
float MeasureConductivity(float Cond_EEP_Rinse, float Cond_EEP_Cal_2, uint8_t Test_Number)
{
	float CalConductivitySlopeLow, CalConductivityKLow, CalConductivitySlopeMid, CalConductivityKMid, CalConductivitySlopeHigh, CalConductivityKHigh, CalConductivityC1, CalConductivityC2;

#ifdef MEMORY_V3
	uint8_t Cal_Number = FindCalNumber();

	// Write calibration data to memory so BT chip can transmit it
	uint16_t Cal_page = (PAGE_CAL_INFO + Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
	while(Cal_page > (PAGE_TEST_INFO - PAGES_FOR_CAL))
		Cal_page -= (PAGE_TEST_INFO - PAGE_CAL_INFO);

	CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R1_SLOPE, 4));
	CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R2_SLOPE, 4));
	CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R3_SLOPE, 4));
	CalConductivityKLow = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R1_INT, 4));
	CalConductivityKMid = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R2_INT, 4));
	CalConductivityKHigh = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R3_INT, 4));

	float T_EEP_Cal = Build_float(MemoryRead(Cal_page + 1, OFFSET_T_CAL, 4));
#else
	uint16_t Cal_Number = FindCalNumber();

	// Write calibration data to memory so BT chip can transmit it
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
//	uint16_t Cal_page = (PAGE_CAL + Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
//	while(Cal_page > (PAGE_TEST - PAGES_FOR_CAL))
//		Cal_page -= (PAGE_TEST - PAGE_CAL);

	CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
	if(CalConductivitySlopeLow == 0)	// Slope would be 0 if calibration was reset partway through, for customers this would prevent tests from running but for our testing I will have it use the last calibration with data in it
	{
		while(Cal_Number > 1 && CalConductivitySlopeLow == 0)
		{
			Cal_Number--;
			Cal_page = Find_Cal_page(Cal_Number);
			CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
		}
	}
	CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
	CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));
	CalConductivityKLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_INT, 4));
	CalConductivityKMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_INT, 4));
	CalConductivityKHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_INT, 4));

	float T_EEP_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));
#endif

	// Measure Conductivity
	float ConductivityReading;
	float Conductivity;
	uint8_t ConductivityRegion = 1;
	uint8_t ConductivityMeasurementGood = 0;

	// To avoid possible infinite loop make sure we are not repeatedly switching between same regions (ex. mid->high->mid->high) in this case go with lower region
	uint8_t Mid_flag = 0;	// Flag to set to know that mid range has been checked
	uint8_t High_flag = 0;	// Flag to set to know that high range has been checked

	// Set RE and CE floating and close RE/CE loop
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

	if((gui32Error & ABORT_ERRORS) == 0)
	{
		// Temperature compensate conductivity of calibrants
		CalConductivityC1 = Cond_EEP_Cal_2 * (1 + COND_TCOMP_CAL_2*(T_EEP_Cal - 25));	// Set at top as global until memory can be read
		CalConductivityC2 = Cond_EEP_Rinse * (1 + COND_TCOMP_RINSE*(T_EEP_Cal - 25));	// Set at top as global until memory can be read
		//			float CalConductivityC3 = Cond_EEP_Cal_1;	// Set at top as global until memory can be read

		ConnectMemory(0);

		uint8_t i = 0;
		while(ConductivityMeasurementGood == 0 && i < 5)
		{
			i++;
			WaveGenSet(0);	// Turn off waveform generator when switching ranges

			switch(ConductivityRegion){
			case 1:		// Set low current range
				// 10.7 uApp R = 309k + 499k = 808k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

//						WaveGenSet(1);	// Turn on waveform generator
				break;
			case 2:		// Set mid current range
				// 20 uApp R = 430k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

//						WaveGenSet(1);	// Turn on waveform generator
				break;
			case 3:		// Set high current range
				// 45 uApp R = 180k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

//						WaveGenSet(1);	// Turn on waveform generator
				break;
			} // switch ConductivityRegion

//					uint8_t j;
//					for(j = 0; j < 10; j++)
//					{
				//					userDelay(Cond_delay, 1);
			uint8_t Check = 0;
			while(Check != 1)
			{
				WaveGenSet(1);

				Check = CheckCond();
				if(Check != 1)
				{
					InitWaveGen(1);
					break;
				}
			}

			ConductivityReading = ConductivityMovingAvg();		// uV
//			UARTprintf("Cond Raw: %d\n", (int) (ConductivityReading * 1000));
				//				ConductivityReadingInv = 1000000000 / ConductivityReading;
//					}

			switch(ConductivityRegion){
			case 1:
			{
				//					Conductivity = ((CalConductivitySlopeLow * ConductivityReadingInv) + (CalConductivityKLow * 1000)) / 1000000;
				Conductivity = (((CalConductivitySlopeLow * 1000000000 / ConductivityReading) + (CalConductivityKLow * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
				break;
			}
			case 2:
			{
				//					Conductivity = ((CalConductivitySlopeMid * ConductivityReadingInv) + (CalConductivityKMid * 1000)) / 1000000;
				Conductivity = (((CalConductivitySlopeMid * 1000000000 / ConductivityReading) + (CalConductivityKMid * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
				Mid_flag = 1;
				break;
			}
			case 3:
			{
				//					Conductivity = ((CalConductivitySlopeHigh * ConductivityReadingInv) + (CalConductivityKHigh * 1000)) / 1000000;
				Conductivity = (((CalConductivitySlopeHigh * 1000000000 / ConductivityReading) + (CalConductivityKHigh * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
				High_flag = 1;
				break;
			}
			}// switch ConductivityRegion

			// Check if the result landed in the right region. If so, end the while loop by setting the "good" flag to 1
			// Also check if we moved back a region, in this case use lower current region
			if (((Conductivity < CalConductivityC1) && (ConductivityRegion == 1)) || (ConductivityRegion == 1 && Mid_flag == 1))
				ConductivityMeasurementGood = 1;
			if (((Conductivity >= CalConductivityC1) && (Conductivity < CalConductivityC2) && (ConductivityRegion == 2)) || (ConductivityRegion == 2 && High_flag == 1))
				ConductivityMeasurementGood = 1;
			if ((Conductivity >= CalConductivityC2) && (ConductivityRegion == 3))
				ConductivityMeasurementGood = 1;

			// Adjust current to the right range, if measurement was in the right range these if statements are irrelevant
			if (Conductivity < CalConductivityC1)
				ConductivityRegion = 1;
			if ((Conductivity >= CalConductivityC1) && (Conductivity < CalConductivityC2))
				ConductivityRegion = 2;
			if (Conductivity >= CalConductivityC2)
				ConductivityRegion = 3;

			if((gui32Error & ABORT_ERRORS) != 0)
				break;
		}// while ConductivityMeasurementGood
	}

	ConnectMemory(1);
	WaveGenSet(0);	// Turn off waveform generator

	if(Test_Number > 0)
	{

#ifdef MEMORY_V3
		uint16_t Test_page = ((PAGE_TEST_INFO + Test_Number * PAGES_FOR_TEST) - PAGES_FOR_TEST);
		while(Test_page > 508)
			Test_page -= (511 - PAGE_TEST_INFO);

		MemoryWrite(Test_page + 2, OFFSET_RAW_COND, 4, (uint8_t *) &ConductivityReading);
//		MemoryWrite(Test_page + 2, OFFSET_COND_REG, 1, &ConductivityRegion);
#else
		uint16_t Test_page = Find_Test_page(Test_Number);
//		uint16_t Test_page = ((PAGE_TEST + Test_Number * PAGES_FOR_TEST) - PAGES_FOR_TEST);
//		while(Test_page > 508)
//			Test_page -= (511 - PAGE_TEST);

		MemoryWrite(Test_page, OFFSET_RAW_COND, 4, (uint8_t *) &ConductivityReading);
		MemoryWrite(Test_page, OFFSET_RAW_COND_REG, 1, &ConductivityRegion);
#endif


	}

	return Conductivity;
}
#else
//********************************************************************************
// Reads conductivity slopes from memory and measures conductivity by switching
// through ranges until it finds correct range, returns measured conductivity,
// NOT a temperature corrected conductivity
// Created: 2/5/2020
// Inputs:	Cond_EEP_Rinse; Conductivity of Rinse at 25C
//			Cond_EEP_Cal_2; Conductivity of Cal 2 at 25C
//			Test_Number; Putting in test number will save raw data, 0 will skip save
// Outputs: Conductivity; returns measured conductivity, NOT temperature corrected
//********************************************************************************
float MeasureConductivity(struct SolutionVals* Sols, uint8_t Test_Number)
{
	float CalConductivitySlopeLow, CalConductivityKLow, CalConductivitySlopeMid, CalConductivityKMid, CalConductivitySlopeHigh, CalConductivityKHigh, CalConductivityC1, CalConductivityC2;

#ifdef MEMORY_V3
	uint8_t Cal_Number = FindCalNumber();

	// Write calibration data to memory so BT chip can transmit it
	uint16_t Cal_page = (PAGE_CAL_INFO + Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
	while(Cal_page > (PAGE_TEST_INFO - PAGES_FOR_CAL))
		Cal_page -= (PAGE_TEST_INFO - PAGE_CAL_INFO);

	CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R1_SLOPE, 4));
	CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R2_SLOPE, 4));
	CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R3_SLOPE, 4));
	CalConductivityKLow = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R1_INT, 4));
	CalConductivityKMid = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R2_INT, 4));
	CalConductivityKHigh = Build_float(MemoryRead(Cal_page + 1, OFFSET_COND_R3_INT, 4));

	float T_EEP_Cal = Build_float(MemoryRead(Cal_page + 1, OFFSET_T_CAL, 4));
#else
	uint16_t Cal_Number = FindCalNumber();

	// Write calibration data to memory so BT chip can transmit it
	uint16_t Cal_page = Find_Cal_page(Cal_Number);
//	uint16_t Cal_page = (PAGE_CAL + Cal_Number * PAGES_FOR_CAL) - PAGES_FOR_CAL;
//	while(Cal_page > (PAGE_TEST - PAGES_FOR_CAL))
//		Cal_page -= (PAGE_TEST - PAGE_CAL);

	CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
	if(CalConductivitySlopeLow == 0)	// Slope would be 0 if calibration was reset partway through, for customers this would prevent tests from running but for our testing I will have it use the last calibration with data in it
	{
		while(Cal_Number > 1 && CalConductivitySlopeLow == 0)
		{
			Cal_Number--;
			Cal_page = Find_Cal_page(Cal_Number);
			CalConductivitySlopeLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_SLOPE, 4));
		}
	}
	CalConductivitySlopeMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_SLOPE, 4));
	CalConductivitySlopeHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_SLOPE, 4));
	CalConductivityKLow = Build_float(MemoryRead(Cal_page, OFFSET_COND_R1_INT, 4));
	CalConductivityKMid = Build_float(MemoryRead(Cal_page, OFFSET_COND_R2_INT, 4));
	CalConductivityKHigh = Build_float(MemoryRead(Cal_page, OFFSET_COND_R3_INT, 4));

	float T_EEP_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));
#endif

	float I_Low, I_Mid, I_High;

	// Read the currents off the memory, these should be saved during the QC process
	EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
	EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
	EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);

	if(I_Low != I_Low)
		I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B
	if(I_Mid != I_Mid)
		I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
	if(I_High != I_High)
		I_High = 43.57 * .812;	// Average from circuits before ARV1_0B

	// Measure Conductivity
	float ConductivityReading;
	float Conductivity;
	uint8_t ConductivityRegion = 1;
	uint8_t ConductivityMeasurementGood = 0;

	// To avoid possible infinite loop make sure we are not repeatedly switching between same regions (ex. mid->high->mid->high) in this case go with lower region
	uint8_t Mid_flag = 0;	// Flag to set to know that mid range has been checked
	uint8_t High_flag = 0;	// Flag to set to know that high range has been checked

	// Set RE and CE floating and close RE/CE loop
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

	if((gui32Error & ABORT_ERRORS) == 0)
	{
		// Temperature compensate conductivity of calibrants
		if(Sols->Cond_EEP_Cal_2 < Sols->Cond_EEP_Cal_1)
			CalConductivityC1 = Sols->Cond_EEP_Cal_2 * (1 + Sols->Cal_2_Cond_TComp*(T_EEP_Cal - 25));	// Set at top as global until memory can be read
		else
			CalConductivityC1 = Sols->Cond_EEP_Cal_1 * (1 + Sols->Cal_1_Cond_TComp*(T_EEP_Cal - 25));	// Set at top as global until memory can be read
		CalConductivityC2 = Sols->Cond_EEP_Clean * (1 + Sols->Clean_Cond_TComp*(T_EEP_Cal - 25));	// Set at top as global until memory can be read
		//			float CalConductivityC3 = Cond_EEP_Cal_1;	// Set at top as global until memory can be read

		ConnectMemory(0);

#ifndef CONST_COND_FREQ
		// Because 5kHz can't read DI, take a point at 1kHz first to check for DI
		if(gABoard >= ARV1_0B)
		{
			float I_Low_Alt;

			// Read the currents off the memory, these should be saved during the QC process
			EEPROMRead((uint32_t *) &I_Low_Alt, OFFSET_COND_ALT_I_LOW, 4);

			if(I_Low_Alt != I_Low_Alt)
				I_Low_Alt = I_Low * 1.22;	// Average from circuits before ARV1_0B

			InitWaveGen(0, 1000);	// Change frequency to 1kHz

			// Set low current range
			// 10.7 uApp R = 309k + 499k = 808k
			IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
			IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

			uint8_t Check = 0, attempt = 0;

			while(Check != 1)
			{
				WaveGenSet(1);

				Check = CheckCond(1000);
				if(attempt == 5)
				{
					gui32Error |= WAVE_GEN_FAIL;
					break;
				}

				if(Check != 1)
				{
					InitWaveGen(1, 1000);
					attempt++;
				}
			}

			ConductivityReading = ConductivityMovingAvg(1000);		// uV

			ConnectMemory(1);
			float CalConductivitySlopeLow_1k = Build_float(MemoryRead(PAGE_CAL, OFFSET_CAL_COND_LOW_ALT_SLOPE, 4));
			ConnectMemory(0);

			if(CalConductivitySlopeLow_1k == CalConductivitySlopeLow_1k)
				Conductivity = (I_Low_Alt / ConductivityReading - CalConductivityKLow) * 1000000 / CalConductivitySlopeLow_1k;
			else
				Conductivity = (I_Low_Alt / ConductivityReading - CalConductivityKLow) * 1000000 / CalConductivitySlopeLow;

			if(Conductivity < 50)
				ConductivityMeasurementGood = 1;

			InitWaveGen(0, 5000);	// Change frequency back to 5kHz
		}
#endif


		uint8_t i = 0;
		while(ConductivityMeasurementGood == 0 && i < 5)
		{
			i++;
			WaveGenSet(0);	// Turn off waveform generator when switching ranges

			switch(ConductivityRegion){
			case 1:		// Set low current range
				// 10.7 uApp R = 309k + 499k = 808k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

//						WaveGenSet(1);	// Turn on waveform generator
				break;
			case 2:		// Set mid current range
				// 20 uApp R = 430k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

//						WaveGenSet(1);	// Turn on waveform generator
				break;
			case 3:		// Set high current range
				// 45 uApp R = 180k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

//						WaveGenSet(1);	// Turn on waveform generator
				break;
			} // switch ConductivityRegion

//					uint8_t j;
//					for(j = 0; j < 10; j++)
//					{
				//					userDelay(Cond_delay, 1);

//			uint8_t Check = 0;
//			while(Check != 1)
//			{
//				WaveGenSet(1);
//
//				Check = CheckCond();
//				if(Check != 1)
//				{
//					InitWaveGen(1);
//					break;
//				}
//			}

			uint8_t Check = 0, attempt = 0;

#ifdef CONST_COND_FREQ
			while(Check != 1)
			{
				WaveGenSet(1);

				Check = CheckCond();
				if(attempt == 5)
				{
					gui32Error |= WAVE_GEN_FAIL;
					break;
				}

				if(Check != 1)
				{
					InitWaveGen(1);
					attempt++;
				}
			}

			ConductivityReading = ConductivityMovingAvg();		// uV
//			UARTprintf("Cond Raw: %d\n", (int) (ConductivityReading * 1000));
				//				ConductivityReadingInv = 1000000000 / ConductivityReading;
//					}
#else
			while(Check != 1)
			{
				WaveGenSet(1);

				Check = CheckCond(COND_FREQ);
				if(attempt == 5)
				{
					gui32Error |= WAVE_GEN_FAIL;
					break;
				}

				if(Check != 1)
				{
					InitWaveGen(1, COND_FREQ);
					attempt++;
				}
			}

			ConductivityReading = ConductivityMovingAvg(COND_FREQ);		// uV
#endif

			if(CalConductivitySlopeLow < 1)
			{
				switch(ConductivityRegion){
				case 1:
				{
					Conductivity = (I_Low / ConductivityReading - CalConductivityKLow) * 1000000 / CalConductivitySlopeLow;
					break;
				}
				case 2:
				{
					Conductivity = (I_Mid / ConductivityReading - CalConductivityKMid) * 1000000 / CalConductivitySlopeMid;
					Mid_flag = 1;
					break;
				}
				case 3:
				{
					Conductivity = (I_High / ConductivityReading - CalConductivityKHigh) * 1000000 / CalConductivitySlopeHigh;
					High_flag = 1;
					break;
				}
				}// switch ConductivityRegion
			}
			else
			{
				switch(ConductivityRegion){
				case 1:
				{
					//					Conductivity = ((CalConductivitySlopeLow * ConductivityReadingInv) + (CalConductivityKLow * 1000)) / 1000000;
					Conductivity = (((CalConductivitySlopeLow * 1000000000 / ConductivityReading) + (CalConductivityKLow * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
					break;
				}
				case 2:
				{
					//					Conductivity = ((CalConductivitySlopeMid * ConductivityReadingInv) + (CalConductivityKMid * 1000)) / 1000000;
					Conductivity = (((CalConductivitySlopeMid * 1000000000 / ConductivityReading) + (CalConductivityKMid * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
					Mid_flag = 1;
					break;
				}
				case 3:
				{
					//					Conductivity = ((CalConductivitySlopeHigh * ConductivityReadingInv) + (CalConductivityKHigh * 1000)) / 1000000;
					Conductivity = (((CalConductivitySlopeHigh * 1000000000 / ConductivityReading) + (CalConductivityKHigh * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
					High_flag = 1;
					break;
				}
				}// switch ConductivityRegion
			}
//#ifdef CURRENT_ADJUSTED_COND
//			switch(ConductivityRegion){
//			case 1:
//			{
//				Conductivity = (I_Low / ConductivityReading - CalConductivityKLow) * 1000000 / CalConductivitySlopeLow;
//				break;
//			}
//			case 2:
//			{
//				Conductivity = (I_Mid / ConductivityReading - CalConductivityKMid) * 1000000 / CalConductivitySlopeMid;
//				Mid_flag = 1;
//				break;
//			}
//			case 3:
//			{
//				Conductivity = (I_High / ConductivityReading - CalConductivityKHigh) * 1000000 / CalConductivitySlopeHigh;
//				High_flag = 1;
//				break;
//			}
//			}// switch ConductivityRegion
//#else
//			switch(ConductivityRegion){
//			case 1:
//			{
//				//					Conductivity = ((CalConductivitySlopeLow * ConductivityReadingInv) + (CalConductivityKLow * 1000)) / 1000000;
//				Conductivity = (((CalConductivitySlopeLow * 1000000000 / ConductivityReading) + (CalConductivityKLow * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
//				break;
//			}
//			case 2:
//			{
//				//					Conductivity = ((CalConductivitySlopeMid * ConductivityReadingInv) + (CalConductivityKMid * 1000)) / 1000000;
//				Conductivity = (((CalConductivitySlopeMid * 1000000000 / ConductivityReading) + (CalConductivityKMid * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
//				Mid_flag = 1;
//				break;
//			}
//			case 3:
//			{
//				//					Conductivity = ((CalConductivitySlopeHigh * ConductivityReadingInv) + (CalConductivityKHigh * 1000)) / 1000000;
//				Conductivity = (((CalConductivitySlopeHigh * 1000000000 / ConductivityReading) + (CalConductivityKHigh * 1000)) / 1000000);// /(1 + Cond_TComp_Samp*(T_Samp - 25));
//				High_flag = 1;
//				break;
//			}
//			}// switch ConductivityRegion
//#endif


			// Check if the result landed in the right region. If so, end the while loop by setting the "good" flag to 1
			// Also check if we moved back a region, in this case use lower current region
			if (((Conductivity < CalConductivityC1) && (ConductivityRegion == 1)) || (ConductivityRegion == 1 && Mid_flag == 1))
				ConductivityMeasurementGood = 1;
			if (((Conductivity >= CalConductivityC1) && (Conductivity < CalConductivityC2) && (ConductivityRegion == 2)) || (ConductivityRegion == 2 && High_flag == 1))
				ConductivityMeasurementGood = 1;
			if ((Conductivity >= CalConductivityC2) && (ConductivityRegion == 3))
				ConductivityMeasurementGood = 1;

			// Adjust current to the right range, if measurement was in the right range these if statements are irrelevant
			if (Conductivity < CalConductivityC1)
				ConductivityRegion = 1;
			if ((Conductivity >= CalConductivityC1) && (Conductivity < CalConductivityC2))
				ConductivityRegion = 2;
			if (Conductivity >= CalConductivityC2)
				ConductivityRegion = 3;

			if((gui32Error & ABORT_ERRORS) != 0)
				break;
		}// while ConductivityMeasurementGood
	}

	ConnectMemory(1);
	WaveGenSet(0);	// Turn off waveform generator

	if(Test_Number > 0)
	{

#ifdef MEMORY_V3
		uint16_t Test_page = ((PAGE_TEST_INFO + Test_Number * PAGES_FOR_TEST) - PAGES_FOR_TEST);
		while(Test_page > 508)
			Test_page -= (511 - PAGE_TEST_INFO);

		MemoryWrite(Test_page + 2, OFFSET_RAW_COND, 4, (uint8_t *) &ConductivityReading);
//		MemoryWrite(Test_page + 2, OFFSET_COND_REG, 1, &ConductivityRegion);
#else
		uint16_t Test_page = Find_Test_page(Test_Number);
//		uint16_t Test_page = ((PAGE_TEST + Test_Number * PAGES_FOR_TEST) - PAGES_FOR_TEST);
//		while(Test_page > 508)
//			Test_page -= (511 - PAGE_TEST);

		if(CalConductivitySlopeLow < 1)
		{
			if(ConductivityRegion == 1)
				ConductivityReading = (1000000 * I_Low) / ConductivityReading;
			else if(ConductivityRegion == 2)
				ConductivityReading = (1000000 * I_Mid) / ConductivityReading;
			else
				ConductivityReading = (1000000 * I_High) / ConductivityReading;

			MemoryWrite(Test_page, OFFSET_RAW_COND, 4, (uint8_t *) &ConductivityReading);
		}
		else
			MemoryWrite(Test_page, OFFSET_RAW_COND, 4, (uint8_t *) &ConductivityReading);
		MemoryWrite(Test_page, OFFSET_RAW_COND_REG, 1, &ConductivityRegion);
#endif


	}

	return Conductivity;
}
#endif

////**************************************************************************
//// Runs amperometric sensor to get chlorine nA value
//// Parameters:	*Range_flag; pointer to flag to set outside function
////				Cl_nA_cutoff; the value to check for when deciding whether
////				to switch ranges or not
//// Outputs:		Cl_nA; nA read from chlorine senso
////**************************************************************************
//float ReadCl(uint8_t *Range_flag, float Cl_nA_cutoff)
//{
//	// Turn on short and off parallel resistor to allow large current flows
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);	// Parallel switch must be on with short switch to work
//	IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 1);	// Always start with wide range and adjust if necessary
//
//	// Set reference for amperometric mode
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
//	DACVoltageSet(0, 470, true);
//
//	// Connect all electrodes together for measuring
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
//
//	SysCtlDelay(SysCtlClockGet()/3 * 3);			// Let run 3 seconds with short to allow large current at beginning
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);	// Turn off short switch
//	float Cl_nA = CurrentTimeRead(0, ADC4_CS_B, 2, 470, 2, .005) * 1000000;	// nA
//
//	if(Cl_nA < Cl_nA_cutoff)
//		Cl_nA = CurrentTimeRead(0, ADC4_CS_B, 10, 470, 2, .005) * 1000000;	// nA
//	else
//	{
//		*Range_flag = 1;	// 0 is starting large range, 1 is small range
//		IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 0);
//		Cl_nA = CurrentTimeRead(0, ADC4_CS_B, 10, 470, 0, .005) * 1000000;	// nA
//	}
//
//	// Let the working electrodes float when not measuring
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 0);
//
//	DACVoltageSet(0, 0, true);
//
//	// RE and CE floating
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating
//
//	return Cl_nA;
//}

////**************************************************************************
//// Calculates the current flowing through the ORP sensors
//// Parameters: 	NONE
//// Outputs:		current; calculated current in mA
////**************************************************************************
//float CalculateCurrent_ORP(float ADC_Voltage, float DAC_Voltage)
//{
//	float Resistance = 100000;
//
//	float current;
//
//	if(ADC_Voltage > 2999)
//	{
//		UARTprintf("WARNING! ADC Voltage positively saturated, current reading not reliable. \t");
//	}
//
//
//	if(ADC_Voltage < 1)
//	{
//		UARTprintf("WARNING! ADC Voltage negatively saturated, current reading not reliable. \t");
//	}
//
//
//	// current = (-V_DAC_SET + 2*V_ADC_IN - V_ref_3)/R_f // Equation used
//
//	current = ((2 * ADC_Voltage) - DAC_Voltage - 3000) / Resistance; // R_f = Two 10 MOhms resistors in series
//
//	return current;	// mA
//}
//
////**************************************************************************
//// Function to read in voltage on specific channel
//// Samples at ~200 SPS, max of 25 samples
//// Parameters:	Channel; [0,6] 	0: All amperometrics
////								1: Amp 1
////								2: Amp 2... etc
////				ADC_CS_PIN; ADC1_CS_B, ADC2_CS_B, or ADC4_CS_B
////				seconds; Number of seconds to sample; 20
////				DAC_Voltage; High current switch status for calculation
////				period; the period in between taking samples
////				Current_switch; 0 = 20M; 1 = 100k; 2 = 5.1M
//// Outputs:		fVoltage; Last point read
////**************************************************************************
//float CurrentTimeRead_ORP(uint8_t ui8Channel, int ADC_CS_PIN, float seconds, int DAC_Voltage, float period)
//{
//	float fVoltage;
//	float Current;
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
//	// Created loop to verify the channel register was updated correctly
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
//		SSIDataGet(SSI1_BASE, &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, &ui32Channel_Rx);
//		SSIDataGet(SSI1_BASE, &ui32Channel_Rx);
//	}
//
//	g_TimerInterruptFlag = 0;
//	TimerLoadSet(TIMER0_BASE, TIMER_A, (seconds * SysCtlClockGet()));		// Timer set
//	TimerEnable(TIMER0_BASE, TIMER_A);
//
//	TimerLoadSet(TIMER1_BASE, TIMER_A, (period * SysCtlClockGet()));		// Timer set
//	TimerEnable(TIMER1_BASE, TIMER_A);
//
//	while(g_TimerInterruptFlag == 0)
//	{
//		// Read from ADC_DOUT registers
//		SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 5, 0x10, 0x01, LRA_ADC_DOUT, 0x00, 0x00); // Read 2 bytes from ADC_DOUT registers
//
//		// First three bytes are junk data
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB);
//
//		// Collect ADC data
//		SSIDataGet(SSI1_BASE, &ui32ADC_MSB); 	// Save MSB
//		SSIDataGet(SSI1_BASE, &ui32ADC_LSB); 	// Save LSB
//
//		ADC_DOUT = 0;	// Clear variable before OR'ing to it again
//		ADC_DOUT |= (ui32ADC_MSB << 8); // Set MSB
//		ADC_DOUT |= ui32ADC_LSB; // Set LSB
//
//		fVoltage = (ADC_DOUT * (1500) / 32768.0) + 1500; // Convert ADC_DOUT data into voltage
//		Current = CalculateCurrent_ORP(fVoltage, DAC_Voltage);
//
////		UARTprintf("Voltage: \t %d \t DOUT: \t %d \n", (int) fVoltage, ADC_DOUT);
//
//		UARTprintf("Current: \t %d \t nA \n", (int) (Current * 1000000));
//
//		while(g_TimerPeriodicInterruptFlag == 0);
//		g_TimerPeriodicInterruptFlag = 0;
//	}
//
//	// Set ADC back into stand-by mode
//	SPISend(SSI1_BASE, 1, ADC_CS_PIN, 0, 4, 0x10, 0x00, 0x08, 0x03); // Register 0x08, 0x3 stand-by mode, 0x0 active mode
//
//	TimerDisable(TIMER1_BASE, TIMER_A);
//	g_TimerInterruptFlag = 0;
//
//	return Current;
//}
//
////**************************************************************************
//// Runs the cleaning procedure for the ORP sensor, same cleaning procedure
//// as used for the amperometrics but with adjusted current, hardware set up
//// for analog V6.0
//// Parameters: 	NONE
//// Outputs:		false if error occurs; current outside of expected range
////				true if cleaning successfully completed
////**************************************************************************
//void CleanORP(void)
//{
//	uint8_t step;
//	float fI_cell; // Current flowing through cell, nA
//	int16_t i16Potential_Step[7] = {1300, -500, 1300, -500, 1300, -500, 1300}; // mV
//	int16_t i16V_cathodic_clean = -300; // mV
//
////	float fI_drive = 0.7;	// uA to drive through each electrode when rebuilding oxide
////	float Rebuild_time = 10;	// # of seconds to drive current when rebuilding oxide
//
//	UARTprintf("Cleaning ORP... \n");
//
////	int i;
//
//	// Set reference and counter in amperometric mode
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
////	// Set reference and counter in amperometric mode
////	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);
////	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//	// Set ORP switches to current reading position
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 1);
//
//	//
//	// Potential Steps
//	//
//	DACVoltageSet(5, i16Potential_Step[0], true);
//	UARTprintf("Potential step %d... \n", 1);
//
//	fI_cell = CurrentTimeRead_ORP(6, ADC2_CS_B, 5, i16Potential_Step[0], .1) * 1000000;	// nA
//	UARTprintf("Current ended at: [%d] nA\n", (int) fI_cell);
//
//	for(step = 1; step < 7; step++)
//	{
//		DACVoltageSet(5, i16Potential_Step[step], true);
//		UARTprintf("Potential step %d... \n", (step + 1));
//
//		fI_cell = CurrentTimeRead_ORP(6, ADC2_CS_B, 3, i16Potential_Step[step], .1) * 1000000;	// nA
//		UARTprintf("Current ended at: [%d] nA\n", (int) fI_cell);
//	}
//
//	//
//	// Cathodic Cleaning
//	//
//	DACVoltageSet(5, i16V_cathodic_clean, true);
//
//	UARTprintf("Cathodic Cleaning: \n");
//	fI_cell = CurrentTimeRead_ORP(6, ADC2_CS_B, 45, i16V_cathodic_clean, .1);
//
//	DACVoltageSet(5, 0, true);
//
////	//
////	// Anodic Oxide Rebuild
////	//
////	UARTprintf("Rebuilding Oxide... \n");
////
////	// Set ORP swithces to current driving position
////	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 0);
////	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 1);
////
////	float Voltage = 3000 - fI_drive * 5100; // mV // Op-amp hardware
////
////	DACVoltageSet(3, Voltage, true);
////
////	float ORPVoltage;
////
////	int k = 0;
////	for(i = 0; i < 10 * Rebuild_time; i++)
////	{
////		SysCtlDelay(SysCtlClockGet()/3 * .1);	// Wait 10 seconds for oxide to rebuild
////
////		k++;
////		if(k == 10)
////		{
////			UARTprintf("Voltages after %d second(s) are: ", (i + 1)/10);
////
////			ORPVoltage = (2 * ADCReadAvg(ORP_CH, ORP_ADC, 20) - 3000)/2;
////			UARTprintf("%d ", (int) ORPVoltage);
////
////			UARTprintf("\n");
////			k = 0;
////		}
////	}
//
//	// Set ORP swithces to let sensor float
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 0);
//
//	// Let reference and counter electrodes float
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//	DACVoltageSet(3, 3000, true);
//
//	UARTprintf("Cleaning completed! \n");
//}

#if defined CV_CLEANING || defined CV_MEASUREMENT
//**************************************************************************
// Runs a CV on the platinum arrays
// Parameters: 	Ref_drift; mV the reference is estimated to have drifted to adjust voltages applied
//				Cal_Number; Cal number to save cleaning numbers to
//				Test_Number; Test number to save cleaning numbers to
// Outputs:		NONE
//**************************************************************************
void RunCVCleaningCycle(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number)
{
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		CleanAmperometrics_CurrentLimited(Ref_drift, Cal_Number, Test_Number, 2);

		int16_t Positive_Step = 1000;
		int16_t Negative_Step = -700;
		float Clean_pH = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
		if(Clean_pH < 7)
		{
			Negative_Step = -650;
			DEBUG_PRINT(UARTprintf("pH 6 Clean, adjusting cleaning voltage!\n");)
		}
		else if(Clean_pH > 8.5)
		{
			Positive_Step = 1000;
			Negative_Step = -800;
			DEBUG_PRINT(UARTprintf("pH 9 Clean, adjusting cleaning voltage!\n");)
		}

		CVCleaning(Positive_Step - Ref_drift, Negative_Step - Ref_drift, 100, 1, 1);

		CleanAmperometrics_CurrentLimited_CCOROnly(Ref_drift, Cal_Number, Test_Number, 2);
	}
}

//**************************************************************************
// Runs a CV on the platinum arrays
// Parameters: 	Start_mV; mV integer value for where to start the CV, can be positive or negative
//				End_mV; mV integer value for where to end the CV, can be positive or negative
//				period; s period to switch
//				BothDirections;	1 to go from start to end back to start, 0 to go from start to end
//				rate; mV/s for the CV to run, for 50 or less code will step 1 mV, higher the code will make it run with a period of 2ms and adjust step to match
// Outputs:		NONE
//**************************************************************************
void CVCleaning(int Start_mV, int End_mV, float rate, uint8_t BothDirections, uint8_t High_current_switch)
{
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		DEBUG_PRINT(UARTprintf("Running CV at %d mV/s\n", (int) rate);)

		// Start with range IO extender
		if(High_current_switch == 1)
			IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);
		else if(High_current_switch == 2)
			IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 1);

		// Set RE + CE to amperometric loop
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

		DEBUG_PRINT(UARTprintf("Period\tVoltage\tCurrent\n");)

		float period = 1/rate;	// ms per step
		float step = 1;	// Plan on stepping 1 mV at a time

		if(period < .02)	// Max speed our electronics can run at currently is 20 ms per step
		{
			period = 0.02;
			step = 0.02 * rate;	// Set step size to be the mV step necessary to scan at the max rate
		}

		if(Start_mV > End_mV)	// If we are starting more negative the steps should be negative
			step *= -1;

		int total_steps = (Start_mV - End_mV) / -step;	// Calculate total # of steps, divide by negative step to keep this number positive

		float CV_potential = Start_mV;
		TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * period); // Set periodic timer
		TimerEnable(TIMER1_BASE, TIMER_A);

		DACVoltageSet(0, CV_potential, true);	// WEs

		// Setup IO Extenders into the proper configuration
		// Connect all electrodes together for potential steps
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);

		int i;
		float fVoltage, Current;
		for(i = 0; i <= total_steps; i++)	// <= because we want to include the final mV step
		{
			DACVoltageSet(0, CV_potential, true);	// WEs

			while(g_TimerPeriodicInterruptFlag == 0);
			g_TimerPeriodicInterruptFlag = 0;

			fVoltage = ADCReadAvg(0, ADC4_CS_B, 1);
			Current = CalculateCurrent(fVoltage, CV_potential, High_current_switch);
			DEBUG_PRINT(UARTprintf("%d\t%d\t%d\tnA\n", (int) period, (int) CV_potential, (int) Current);)
			CV_potential += step;	// Move by step size
			period++;
		}

		if(BothDirections)	// If we are going both directions flip the sign of the step and run through total steps again
		{
			step *= -1;

			// Step twice to cancel out the last change as well as start moving us in the right direction
			CV_potential += step;	// Move by step size
			CV_potential += step;	// Move by step size

			for(i = 0; i < total_steps; i++)	// < only because we do not want to include the first step as the above loop already completed it
			{
				DACVoltageSet(0, CV_potential, true);	// WEs

				while(g_TimerPeriodicInterruptFlag == 0);
				g_TimerPeriodicInterruptFlag = 0;

				fVoltage = ADCReadAvg(0, ADC4_CS_B, 1);
				Current = CalculateCurrent(fVoltage, CV_potential, High_current_switch);
				DEBUG_PRINT(UARTprintf("%d\t%d\t%d\tnA\n", (int) period, (int) CV_potential, (int) Current);)
				CV_potential += step;	// Move by step size
				period++;
			}
		}

		TimerDisable(TIMER1_BASE, TIMER_A);

		// Let the working electrodes float after cleaning
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
		IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);

		// RE and CE floating
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
		IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating

		// Start with range IO extender
		IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
		IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 0);
	}
}

//**************************************************************************
// Runs the cleaning procedure for all Amperometric arrays and ORP sensor
// Parameters: 	Ref_drift; reference drift adjustment in mV
//				Oxide_Rebuild; Flag 0 or 1; to rebuild the oxide layer
// Outputs:		false if error occurs; current outside of expected range
//				true if cleaning successfully completed
//**************************************************************************
void CleanAmperometrics_CurrentLimited_CCOROnly(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number, uint8_t Oxide_Rebuild)
{
	uint16_t page = 0;
#ifndef MEMORY_V5
	if(Cal_Number != 0)
		page = Find_Cal_page(Cal_Number);
	if(Test_Number != 0)
		page = Find_Test_page(Test_Number);
#endif

//	int16_t Positive_Step = 1000;
	int16_t Negative_Step = -700;
	float Clean_pH = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
	if(Clean_pH < 7)
	{
		Negative_Step = -650;
		DEBUG_PRINT(UARTprintf("pH 6 Clean, adjusting cleaning voltage!\n");)
	}
	else if(Clean_pH > 8.5)
	{
//		Positive_Step = 1000;
		Negative_Step = -800;
		DEBUG_PRINT(UARTprintf("pH 9 Clean, adjusting cleaning voltage!\n");)
	}

	float * fI_WE; 	// Starting and Ending Current flowing through WEs, nA
	int16_t i16V_cathodic_clean = Negative_Step - Ref_drift; // mV	// 2/13/2020 Cleaning solution

	float fI_drive = 0.7;	// uA to drive through each electrode when rebuilding oxide
	if(gABoard >= AV6_4)
		fI_drive = 3.0;
//		fI_drive = 1.0;	// Changed 10/9/2019

	float Rebuild_time = 10;	// # of seconds to drive current when rebuilding oxide

	DEBUG_PRINT(UARTprintf("Cleaning Amperometrics using steps and oxide rebuild... \n");)

	int i;

	// Connect all electrodes together for potential steps
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);
	if(gABoard >= AV7_3)
		IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);

//	// Connect metals electrodes for potential steps
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 1);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 1);

	// Set ORP switches to voltage setting and current reading position
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 1);
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 1);

	// Set RE + CE to amperometric loop
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

	//
	// Cathodic Cleaning
	//
	if((gui32Error & ABORT_ERRORS) == 0)
	{
		DACVoltageSet(0, i16V_cathodic_clean, false);	// WEs
		DACVoltageSet(5, i16V_cathodic_clean, true);	// ORP

		DEBUG_PRINT(UARTprintf("Cathodic Cleaning: \n");)

		if(gABoard >= AV7_3)
		{
			IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);

#ifndef MIX_WHILE_CLEANING
			userDelay(3000, 1);
			if(Clean_pH < 7)
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 87, i16V_cathodic_clean, 1, .1, -2500, page);
			else
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 87, i16V_cathodic_clean, 1, .1, -1800, page);

#ifdef PUMP_CLEAN
		PumpStepperRunStepSpeed_AbortReady(FW, 1000, 6000);
#endif

#else
			// Set time for 3 seconds, mix during this without reading reading current, then take first current measurement
			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 3); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;

			while(g_TimerInterruptFlag == 0)
				PumpStepperMix(BW, 600, 3000, 1);

			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 87); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;

			// Take starting current value
			float fI_Watch[2];
			fI_Watch[0] = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), i16V_cathodic_clean, 1);
			fI_Watch[1] = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), i16V_cathodic_clean, 1);

			while(abs_val(fI_Watch[1]) > abs_val(-1800) && g_TimerInterruptFlag == 0)
			{
				PumpStepperMix(BW, 600, 3000, 1);
				fI_Watch[1] = CalculateCurrent(ADCReadAvg(0, ADC4_CS_B, 1), i16V_cathodic_clean, 1);
				DEBUG_PRINT(UARTprintf("Current: %d nA\n", (int) fI_Watch[1]);)
			}
			TimerDisable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;
			fI_WE = fI_Watch;
#ifndef	MEMORY_V5
			if(page > 0)
			{
				uint8_t Time_to_save = TimerLoadGet(TIMER0_BASE, TIMER_A) / SysCtlClockGet();
				MemoryWrite(page, OFFSET_CLEAN_CATH_TIME, 1, &Time_to_save);
			}
#endif
#endif
		}
		else
		{
			if(Clean_pH < 7)
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 90, i16V_cathodic_clean, 1, .1, -2500, page);
			else
				fI_WE = CurrentTimeRead_CurrentLimited(0, ADC4_CS_B, 2, 90, i16V_cathodic_clean, 1, .1, -1800, page);
		}

		DEBUG_PRINT(UARTprintf("Current started and ended at:\t%d\t%d\tnA\n", (int) fI_WE[0], (int) fI_WE[1]);)
		if(fI_WE[1] < -7000 || fI_WE[1] > 0)
		{
			UARTprintf("Current should be between -7000 and 0 nA!\n");
			gui32Error |= CL_CLEANING_OUT_OF_RANGE;
			update_Error();
		}
	}

	// Turn off cathodic cleaning voltages
	DACVoltageSet(0, 0, false);	// WEs
	DACVoltageSet(5, 0, true);	// ORP
	IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);

	// Disconnect ORP sensor from driving circuitry
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A0, 0);
	IO_Ext_Set(IO_EXT2_ADDR, 2, ORP_SW_A1, 0);

#ifndef MEMORY_V5
	// Write 8 bytes to save both the starting and final currents
	MemoryWrite(page, OFFSET_CLEAN_CATH_START, 8, (uint8_t *) fI_WE);
#endif

	//
	// Anodic Oxide Rebuild
	//
	float WEVoltages[5];
#ifndef MEMORY_V5
	int16_t Start_V[5];
#endif

	if(Oxide_Rebuild > 0)
	{
		if((gui32Error & ABORT_ERRORS) == 0 && Oxide_Rebuild == 1)
		{
			DEBUG_PRINT(UARTprintf("Rebuilding Oxide for 10 seconds... \n");)

			// Connect all electrodes separately to drive current through each
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

			float Voltage;
			EEPROMRead((uint32_t *) &Voltage, OFFSET_AMP_I_VSET, 4);
			if(Voltage == Voltage && Voltage <= 3000 && Voltage >= 0)
			{
				DEBUG_PRINT(UARTprintf("Vset pulled from EEPROM, theory 1410, using %d uV!\n", (int) (Voltage * 1000));)
			}
			else
			{
				DEBUG_PRINT(UARTprintf("Vset not saved to EEPROM, calculating based on theory!\n");)
				if(gABoard >= AV6_4)
					Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays
				else if(gABoard >= AV6)
					//		Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays, Resistor changed from 430 kOhms to 30 kOhms to allow current for BES rinse
					Voltage = 3000 - fI_drive * 430; // mV // Op-amp hardware driving 5 arrays
				else
					Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware with all arrays tied together
			}

			//		DACVoltageSet(1, Voltage, false);	// Gold array
			DACVoltageSet(2, Voltage, true);	// 5 platinum arrays

			int j;
			TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 1); // Set periodic timer
			TimerEnable(TIMER1_BASE, TIMER_A);
			for(i = 0; i < Rebuild_time; i++)
			{
				while(g_TimerPeriodicInterruptFlag == 0);
				g_TimerPeriodicInterruptFlag = 0;

				DEBUG_PRINT(UARTprintf("Voltages after %d second(s) are: ", (i + 1));)
				for(j = 0; j < 5; j++)
				{
					WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
					DEBUG_PRINT(UARTprintf("%d, ", (int) WEVoltages[j]);)
#ifndef MEMORY_V5
					if(i == 0)
						Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
				}
				DEBUG_PRINT(UARTprintf("\n");)

				// Program check for specific value range
				if(i == (Rebuild_time - 1))
				{
					if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 100) || FindArrayMax(WEVoltages, 5) > 1600 || FindArrayMin(WEVoltages, 5) < 1000)
					{
						DEBUG_PRINT(UARTprintf("Voltages should be closer together!\n");)
						gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}
				}

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}
			TimerDisable(TIMER1_BASE, TIMER_A);
			g_TimerPeriodicInterruptFlag = 0;

			//	DACVoltageSet(1, 3000, false);	// Gold array
			DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
			uint8_t Rebuild_time = 10;
			MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Rebuild_time);
#endif
		}
		if((gui32Error & ABORT_ERRORS) == 0 && Oxide_Rebuild == 2)
		{
			DEBUG_PRINT(UARTprintf("Rebuilding Oxide until a set mV... \n");)

			// Connect all electrodes separately to drive current through each
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
			IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

			float Voltage;
			EEPROMRead((uint32_t *) &Voltage, OFFSET_AMP_I_VSET, 4);
			if(Voltage == Voltage && Voltage <= 3000 && Voltage >= 0)
			{
				DEBUG_PRINT(UARTprintf("Vset pulled from EEPROM, theory 1410, using %d uV!\n", (int) (Voltage * 1000));)
			}
			else
			{
				DEBUG_PRINT(UARTprintf("Vset not saved to EEPROM, calculating based on theory!\n");)
				if(gABoard >= AV6_4)
					Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays
				else if(gABoard >= AV6)
					//		Voltage = 3000 - fI_drive * 30; // mV // Op-amp hardware driving 5 arrays, Resistor changed from 430 kOhms to 30 kOhms to allow current for BES rinse
					Voltage = 3000 - fI_drive * 430; // mV // Op-amp hardware driving 5 arrays
				else
					Voltage = 3300 - fI_drive * 430; // mV // Op-amp hardware with all arrays tied together
			}

			//		DACVoltageSet(1, Voltage, false);	// Gold array
			DACVoltageSet(2, Voltage, true);	// 5 platinum arrays

#ifndef MIX_WHILE_CLEANING
			int j;
			TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * .1); // Set periodic timer
			TimerEnable(TIMER1_BASE, TIMER_A);

			i = 0;
			uint8_t check = 1;
			float Avg = 0;
			uint16_t mV_target = 1470;
			if(Clean_pH < 7)
				mV_target = 1520;
			while((i + 1) < 30 || (i < 300 && check == 1 && Avg < mV_target))	// Set max of 30 seconds oxide rebuild TODO: Set the average mV to rebuild to in this while loop
			{
				while(g_TimerPeriodicInterruptFlag == 0);
				g_TimerPeriodicInterruptFlag = 0;

				DEBUG_PRINT(
				if((i + 1) % 10 == 0)
					UARTprintf("Voltages after %d second(s) are: ", (i + 1)/10);
				)
				for(j = 0; j < 5; j++)
				{
					WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
					DEBUG_PRINT(
					if((i + 1) % 10 == 0)
						UARTprintf("%d, ", (int) WEVoltages[j]);
					)
#ifndef MEMORY_V5
					if(i == 0)
						Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
				}
				DEBUG_PRINT(
				if((i + 1) % 10 == 0)
					UARTprintf("\n");
				)

				// Check after 3 seconds that things look ok, if they do continue until the set voltage is reached
				if((i + 1) == 30)
				{
					if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 150) || FindArrayMax(WEVoltages, 5) > 1600 || FindArrayMin(WEVoltages, 5) < 750)
					{
						DEBUG_PRINT(UARTprintf("Voltages should be closer together or aren't in the right range! Leaving Oxide Rebuild!\n");)
						check = 0;
						gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}
				}
				if((i + 1) > 30)	// 3 seconds have elapsed, start watching current to reach level
				{
					uint8_t index;
					Avg = 0;
					for(index = 0; index < 5; index++)
					{
						Avg += WEVoltages[index];
					}
					Avg /= 5;
				}

				i++;

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}
			TimerDisable(TIMER1_BASE, TIMER_A);
			g_TimerPeriodicInterruptFlag = 0;

			//	DACVoltageSet(1, 3000, false);	// Gold array
			DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
			uint8_t Rebuild_time = (i / 10);
			MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Rebuild_time);
#endif
#else
			g_TimerInterruptFlag = 0;
			TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 30); // Set periodic timer
			TimerEnable(TIMER0_BASE, TIMER_A);

			int j;
			uint8_t check = 1;
			float Avg = 0;
			i = 0;
			while(g_TimerInterruptFlag == 0 && (check == 1 && Avg < 1470))
			{
				PumpStepperMix(BW, 600, 3000, 1);
				i++;

				DEBUG_PRINT(UARTprintf("Voltages after %d mix are: ", i);)
				for(j = 0; j < 5; j++)
				{
					WEVoltages[j] = ADCReadAvg((j+1), ADC4_CS_B, 1);
					UARTprintf("%d, ", (int) WEVoltages[j]);
#ifndef MEMORY_V5
					if(i == 1)
						Start_V[j] = WEVoltages[j];	// Keep the starting voltages so they can be saved in memory later
#endif
				}
				UARTprintf("\n");

				// Check after 3 mixes that things look ok, if they do continue until the set voltage is reached
				if(i > 3)
				{
					if(((FindArrayMax(WEVoltages, 5) - FindArrayMin(WEVoltages, 5)) > 150) || FindArrayMax(WEVoltages, 5) > 1600 || FindArrayMin(WEVoltages, 5) < 750)
					{
						DEBUG_PRINT(UARTprintf("Voltages should be closer together or aren't in the right range! Leaving Oxide Rebuild!\n");)
						check = 0;
						gui32Error |= CL_CLEANING_OUT_OF_RANGE;
						update_Error();
					}
				}
				if(i > 3)	// 3 mixes have elapsed, start watching current to reach level
				{
					uint8_t index;
					Avg = 0;
					for(index = 0; index < 5; index++)
					{
						Avg += WEVoltages[index];
					}
					Avg /= 5;
				}

				if((gui32Error & ABORT_ERRORS) != 0)
					break;
			}
			TimerDisable(TIMER0_BASE, TIMER_A);
			g_TimerInterruptFlag = 0;

			//	DACVoltageSet(1, 3000, false);	// Gold array
			DACVoltageSet(2, 3000, true);	// 5 platinum arrays

#ifndef MEMORY_V5
			uint8_t Rebuild_time = TimerLoadGet(TIMER0_BASE, TIMER_A) / SysCtlClockGet();
			DEBUG_PRINT(UARTprintf("Rebuild time: %d\n", Rebuild_time);)
			MemoryWrite(page, OFFSET_CLEAN_REBUILD_TIME, 1, &Rebuild_time);
#endif
#endif
		}
	}

	SysCtlDelay(SysCtlClockGet()/6);	// Wait 1/2 second after turning off current to allow capacitor to discharge

	// Let the working electrodes float after cleaning
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
	IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);

//	// Let Metals electrode float after cleaning
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_SW_EN, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWB, 0);
//	IO_Ext_Set(IO_EXT2_ADDR, 3, METALS_WE_SWA, 0);

	// Let reference and counter electrodes float
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

#ifndef MEMORY_V5
	int16_t End_V[5];	// Changing voltages to int16_t to save space in memory
	for(i = 0; i < 5; i++)
		End_V[i] = WEVoltages[i];

	MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_START, 10, (uint8_t *) Start_V);
	MemoryWrite(page, OFFSET_CLEAN_ARRAY_1_FINAL, 10, (uint8_t *) End_V);
#endif

	DEBUG_PRINT(UARTprintf("Cleaning completed! \n");)
}
#endif

#ifdef TESTING_MODE
//**************************************************************************
// Read voltage on reference guard, Ondrej wants to make sure the reference
// is stable during the chlorine measurement
// ADC runs at 4000 Hz
// Created: 5/25/2023
// Parameters:	NONE
// Returns:		Voltage as a float
//**************************************************************************
float ReadRefGuard(float fSec)
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

		while((InMux_Rx != 0x0C || DataRate_Rx != 0x1E) && counter <= 10)
		{
			// Write register 010r rrrr
			// Read register 001r rrrr

			// Input Multiplexer register 0x02, write = 0x42, read = 0x22
			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 3, 0x42, 0x00, 0x0C);	// Set positive input to AIN 4 (0100 = 0x4) (COND ADC DR), negative input to AINCOM (1100 = 0xC) (GND)
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
			else if(counter == 5 && (InMux_Rx != 0x0C || DataRate_Rx != 0x1E))
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
			else if(counter == 10 && (InMux_Rx != 0x0C || DataRate_Rx != 0x1E))
			{
				DEBUG_PRINT(UARTprintf("ADC5 failed to initialize input mux or data rate!\nUpdating error and moving on!\n");)
				counter++;
				gui32Error |= ADC5_FAIL;
				update_Error();
			}
		}

		SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x04);	// Put ADC into power-down mode until it is used
	}


	if(gABoard >= AV6_6)
	{
		DEBUG_PRINT(
		if(gDiagnostics >= 1)
			UARTprintf("Reading reference!\n");
		)

		uint8_t attempts = 0;
		uint16_t Non_Zero_counter = 0;	// Count up how many times ADC returns something besides 0 to make sure its working

		float Voltage;
//		while(attempts < 3 && Non_Zero_counter < 3000)
//		{
			attempts++;
			uint32_t bits07 = 0, bits815 = 0;
			int16_t Data = 0;

			// Set SPI communication to mode 1 for ADC5, capturing on the falling edge
			SSIDisable(SSI1_BASE);
			SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, SPI_CLOCK, 8);
			SSIEnable(SSI1_BASE);

			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x02);	// Send wakeup command to ADC

			while(SSIDataGetNonBlocking(SSI1_BASE, &bits07)); // Clear FIFO

			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 0);	// Set CS_B low while sending/receiving data from ADC5
			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 1);

			SysCtlDelay(SysCtlClockGet()/3000);	// Wait 2 ms after raising start pin

			float period = 0.1;	// Set a sampling period of 100 ms, don't need much data to see if it's stable
			uint32_t i;
			int32_t samples = fSec / period;

			g_TimerPeriodicInterruptFlag = 0;
			TimerLoadSet(TIMER1_BASE, TIMER_A, (period * SysCtlClockGet())); // Set periodic timer
			TimerEnable(TIMER1_BASE, TIMER_A);

			//		uint32_t Cross_zero = 0;
			for(i = 0; i < samples; i++)
			{
				//					while(g_TimerPeriodicInterruptFlag == 0);
				while(g_TimerPeriodicInterruptFlag == 0)
				{
					// Poll if BT wants to use I2C, if it does reconnect memory and leave it connected, this will make the signal more noisy during this read but
					// it will prevent the BT from reading incorrect data into the app
					if(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN && gui8MemConnected == 0)
						ConnectMemory(1);
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

//				Voltage = ((Data * 2.0 * 3000.0 / 65536.0) - 1500) * 2;
				Voltage = ((Data * 2.0 * 3000.0 / 65536.0) - 1500) * 2;

				DEBUG_PRINT(UARTprintf("%d\n", (int) (Voltage));)
			}

			TimerDisable(TIMER1_BASE, TIMER_A);
			g_TimerPeriodicInterruptFlag = 0;

			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_START, 0);
			IO_Ext_Set(IO_EXT2_ADDR, 2, ADC5_CS_B, 1);	// Set CS_B high after completion

			SPISend(SSI1_BASE, 2, ADC5_CS_B, 0, 1, 0x04);	// Send power-down command to ADC

//			if(Non_Zero_counter < 1 && attempts < 3)
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

		if(Non_Zero_counter < 1 && attempts == 3)
		{
			gui32Error |= ADC5_FAIL;
			update_Error();
			DEBUG_PRINT(UARTprintf("Conductivity frequency check ADC always read 0's, updating error!\n");)
		}

		InitTurbidityADC();

		return Voltage;
	}
	DEBUG_PRINT(UARTprintf("Didn't find correct analog board version!\n");)
	return nanf("");
}

////**************************************************************************
//// Attempting to clean the conductivity pads by holding them at 0 and
//// driving the CE, leaving the Ref unconnected for the first attempt
//// Created: 12/7/2023
//// Parameters:	NONE
//// Returns:		NONE
////**************************************************************************
//void CleanCond(void)
//{
//	// Measure conductivity
//	// Set RE and CE floating and close RE/CE loop for conductivity
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//	ConnectMemory(0);
//
//	IO_Ext_Set(IO_EXT2_ADDR, 3, COND_SHORT_SW, 1);		// Reconnect feedback switch over conductivity op-amp
//	SysCtlDelay(SysCtlClockGet()/3000 * 1000);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_SW, 1);	// Input for switch connecting circuit
//
//	DACVoltageSet(6, 2500, true);	// Set the DAC COUNTER_V_DRIVE channel
//	IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 1);
//
//	DEBUG_PRINT(UARTprintf("Cleaning Conductivity:\n");)
//	uint8_t attempt = 0;
//	uint8_t stable = 0;
//	float Prev_Diff = ConductivityMovingAvg();
//	DEBUG_PRINT(UARTprintf("%d\n", (int) (Prev_Diff));)
//	while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && attempt < 15 && stable < 5)
//	{
//		float Diff = ConductivityMovingAvg();
//		if(abs_val(Diff - Prev_Diff) < 5)
//		{
//			stable++;
//		}
//		Prev_Diff = Diff;
//
//		DEBUG_PRINT(UARTprintf("%d\n", (int) (Diff));)
//	}
//
//	IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 0);	// Let go of Counter electrode
//	DACVoltageSet(6, 0, true);	// Set the DAC COUNTER_V_DRIVE channel
//
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_SW, 0);	// Disconnect switch connecting circuit
//}
#endif
