//*****************************************************************************
//
// Communication.c - Functions used in e-SENS firmware to communicate with
// components and bluetooth chip
// Includes functions related to I2C and SPI
//
// Author: Jason Castillo
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ssi.h"
#include "inc/hw_gpio.h"
#include "driverlib/eeprom.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "Communication.h"
#include "Components.h"
#include "inc/tm4c123gh6pm.h"
#include "Helper.h"
#include "Bluetooth.h"
#include "main.h"

// Global Variables
volatile unsigned long g_ulSSI0RXTO = 0; // SPI receive timeout flag
uint32_t g_ui32DataRx0[NUM_SSI_DATA] = {0, 0, 0}; // Received data on SPI slave port

//uint8_t g_error_byte_2 = 0;
//uint8_t g_error_byte_3 = 0;
volatile uint8_t g_MonoCl = 1;
volatile uint8_t g_FreeCl = 1;
volatile uint8_t g_Alkalinity = 1;

//**************************************************************************
// Enable I2C Peripherals 0
// Parameters: NONE
//**************************************************************************
void InitI2C(void)
{
	DEBUG_PRINT(
	if(gBoard > V1)
	{
		if(gDiagnostics >= 1)
			UARTprintf("Init I2C \n");
	}
	)

    //enable I2C module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0); // IO Extenders/Memory/Battery/Accelerometer

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C functions
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Configure Data line to be open drain
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
}

//**************************************************************************
// Function to send data on I2C line
// If writing to device registers the first byte sent must be register address
// After register address sending more than one byte rolls into next register
// Parameters:	ulI2CBase; base address of I2C module; I2C1_BASE
//				slave_addr; address for slave device; BAT_SLAVE_ADDR
//				num_of_args; number of bytes being sent to slave
//				...; data to send, can be as many arguments as needed
//**************************************************************************
void I2CSend(unsigned long ulI2CBase, uint8_t slave_addr, unsigned int num_of_args, ...)
{
	unsigned int i;
	uint32_t Error = I2C_MASTER_ERR_NONE;

	DEBUG_PRINT(
	if(gBoard > V1 && gDiagnostics >= 2)
		UARTprintf("I2CSend called \n");
	)

    //wait for MCU to finish transaction
    while(I2CMasterBusy(ulI2CBase)){
    	DEBUG_PRINT(
		if(gDiagnostics >= 2)
			UARTprintf("I2CSend, I2CMasterBusy \n");
    	)
    }

//	SysCtlDelay(50000); // Removed to improve speed of IO extenders

	// Tell the master module what address it will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(ulI2CBase, slave_addr, false);

	//stores list of variable number of arguments
	va_list vargs;

	//specifies the va_list to "open" and the last fixed argument
	//so vargs knows where to start looking
	va_start(vargs, num_of_args);

	//put data to be sent into FIFO
	I2CMasterDataPut(ulI2CBase, va_arg(vargs, uint32_t));

	//if there is only one argument, we only need to use the
	//single send I2C function
	if(num_of_args == 1)
	{
		//Initiate send of data from the MCU
		I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_SINGLE_SEND);

    	Error = I2CMasterErr(ulI2CBase);
    	if(Error != I2C_MASTER_ERR_NONE)
    	{
    		gui32Error |= I2C_FAILED;

    		update_Error();
    		DEBUG_PRINT(
    		if(gDiagnostics >= 1)
    			UARTprintf("I2C Send failed!\n");
    		)
    	}

		// Wait until MCU is done transferring.
		while(I2CMasterBusy(ulI2CBase)){
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("I2CSend, I2CMasterBusy \n");
			)
		}

		//"close" variable argument list
		va_end(vargs);
	}

	//otherwise, we start transmission of multiple bytes on the
	//I2C bus
	else
	{
		//Initiate send of data from the MCU
		I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_SEND_START);

		// Wait until MCU is done transferring.
		while(I2CMasterBusy(ulI2CBase)){
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("I2CSend, I2CMasterBusy \n");
			)
		}

		//send num_of_args-2 pieces of data, using the
		//BURST_SEND_CONT command of the I2C module
		for(i = 1; i < (num_of_args - 1); i++)
		{
			//put next piece of data into I2C FIFO
			I2CMasterDataPut(ulI2CBase, va_arg(vargs, uint32_t));
			//send next data that was just placed into FIFO
			I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_SEND_CONT);

			// Wait until MCU is done transferring.
			while(I2CMasterBusy(ulI2CBase)){
				DEBUG_PRINT(
				if(gDiagnostics >= 2)
					UARTprintf("I2CSend, I2CMasterBusy \n");
				)
			}
		}

		//put last piece of data into I2C FIFO
		I2CMasterDataPut(ulI2CBase, va_arg(vargs, uint32_t));
		//send next data that was just placed into FIFO
		Error = I2CMasterErr(ulI2CBase);
		I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
		// Wait until MCU is done transferring.
		while(I2CMasterBusy(ulI2CBase)){
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("I2CSend, I2CMasterBusy \n");
			)
		}

    	if(Error != I2C_MASTER_ERR_NONE)
    	{
    		gui32Error |= I2C_FAILED;

    		update_Error();
    		DEBUG_PRINT(
    		if(gDiagnostics >= 1)
    			UARTprintf("I2C Send failed!\n");
    		)
    	}

		//"close" variable args list
		va_end(vargs);
	}

}

//**************************************************************************
// Function to read data from I2C slave device
// Can only read from one register at a time; 8-bit or 16-bit registers
// Assumes if 16-bit register that LSB comes first and MSB comes second
// Parameters:	ulI2CBase; base address of I2C module; I2C1_BASE
//				slave_addr; address for slave device; BAT_SLAVE_ADDR
//				reg; register address to read from
//				num_of_bytes; number of bytes to read
//**************************************************************************
unsigned int I2CReceive(unsigned long ulI2CBase, uint8_t slave_addr, unsigned char reg, unsigned int num_of_bytes)
{
	uint32_t Error = I2C_MASTER_ERR_NONE;

	DEBUG_PRINT(
	if(gBoard > V1 && gDiagnostics >= 2)
		UARTprintf("I2CReceive called \n");
	)

	// Making the bold assumption no registers will be 0xFF, so if it is entered (for LED IO Ext digital V7.2) it's because a register doesn't need to be written
	if(reg != 0xFF)
	{
		//wait for MCU to finish transaction
		while(I2CMasterBusy(ulI2CBase)){
			DEBUG_PRINT(
			if(gDiagnostics >= 2)
				UARTprintf("I2CReceive, I2CMasterBusy \n");
			)
		}

		//specify that we are writing (a register address) to the
		//slave device
		I2CMasterSlaveAddrSet(ulI2CBase, slave_addr, false);

		//specify register to be read
		I2CMasterDataPut(ulI2CBase, reg);

		//send control byte and register address byte to slave device
		I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_SEND_START);
	}

    //wait for MCU to finish transaction
    while(I2CMasterBusy(ulI2CBase)){
    	DEBUG_PRINT(
    	if(gDiagnostics >= 2)
    		UARTprintf("I2CReceive, I2CMasterBusy \n");
    	)
    }

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(ulI2CBase, slave_addr, true);

    if(num_of_bytes == 1)
    {
    	//send control byte and read from the register we
    	//specified
    	I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_SINGLE_RECEIVE);

    	//wait for MCU to finish transaction
    	while(I2CMasterBusy(ulI2CBase)){
    		DEBUG_PRINT(
        	if(gDiagnostics >= 2)
        		UARTprintf("I2CReceive, I2CMasterBusy \n");
    		)
    	}

    	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00); // I2C used by TIVA

    	//return data pulled from the specified register
    	uint32_t Value = I2CMasterDataGet(ulI2CBase);

    	Error = I2CMasterErr(ulI2CBase);
    	if(Error != I2C_MASTER_ERR_NONE)
    	{
    		gui32Error |= I2C_FAILED;

    		update_Error();
    		DEBUG_PRINT(
    		if(gDiagnostics >= 1)
    			UARTprintf("I2C Receive failed!\n");
    		)
    	}


    	return Value;
    }
    else
    {
    	//send control byte and read from the register we
    	//specified
    	I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_RECEIVE_START);

    	//wait for MCU to finish transaction
    	while(I2CMasterBusy(ulI2CBase)){
    		DEBUG_PRINT(
        	if(gDiagnostics >= 2)
        		UARTprintf("I2CReceive, I2CMasterBusy \n");
    		)
    	}

    	//return data pulled from the specified register
    	unsigned int LSB = I2CMasterDataGet(ulI2CBase);

    	I2CMasterControl(ulI2CBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    	//wait for MCU to finish transaction
    	while(I2CMasterBusy(ulI2CBase)){
    		DEBUG_PRINT(
        	if(gDiagnostics >= 2)
        		UARTprintf("I2CReceive, I2CMasterBusy \n");
    		)
    	}

    	//
    	unsigned int MSB = I2CMasterDataGet(ulI2CBase);

    	Error = I2CMasterErr(ulI2CBase);
    	if(Error != I2C_MASTER_ERR_NONE)
    	{
    		gui32Error |= I2C_FAILED;

    		update_Error();
    		DEBUG_PRINT(
    		if(gDiagnostics >= 1)
    			UARTprintf("I2C Receive failed!\n");
    		)
    	}

    	unsigned int reg_val = (MSB << 8) + LSB;

    	return reg_val;
    }
}

//**************************************************************************
// Initialize SPI Peripheral including all GPIO configuration and interrupt
// for slave port
// Parameters: NONE
//**************************************************************************
void InitSPI(void)
{
	// Enable the SSI0 pheripheral and GPIO Pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Unlock PF0 pin so data can be read in
	HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;

	// Wait until the SSI modules are ready
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1));

	// Set up pin muxing for SSI module
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PF0_SSI1RX);
	GPIOPinConfigure(GPIO_PF1_SSI1TX);
	GPIOPinConfigure(GPIO_PF2_SSI1CLK);

	// Set GPIO pins as type SSI
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
	GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

	// Configure the SSI
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_SLAVE, SPI_CLOCK, 8);
	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);

	// Enable the SSI module
	SSIEnable(SSI0_BASE);
	SSIEnable(SSI1_BASE);

    // Enable RX timeout interrupt.
    SSIIntEnable(SSI0_BASE, SSI_RXTO);

    // Clear FIFO
    while(SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[2]));

    // Clear any pending interrupt
    SSIIntClear(SSI0_BASE, SSI_RXTO);

    // Enable the SSI0 interrupts to ARM core.
    IntEnable(INT_SSI0);
}

////**************************************************************************
//// Reset SPI Peripheral communicating with analog board
//// Parameters: NONE
////**************************************************************************
//void SPIReset(void)
//{
//	// Enable the SSI0 pheripheral and GPIO Pins
//	SSIDisable(SSI1_BASE);
//	SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI1);
//
//	SysCtlDelay(SysCtlClockGet()/3000 * 5);
//
//	// Enable the SSI0 pheripheral and GPIO Pins
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
//
//	// Wait until the SSI1 module is ready
//	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1));
//
//	// Set up pin muxing for SSI module
//	GPIOPinConfigure(GPIO_PF0_SSI1RX);
//	GPIOPinConfigure(GPIO_PF1_SSI1TX);
//	GPIOPinConfigure(GPIO_PF2_SSI1CLK);
//
//	// Set GPIO pins as type SSI
//	GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
//
//	// Configure the SSI
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_CLOCK, 8);
//
//	// Enable the SSI module
//	SSIEnable(SSI1_BASE);
//}

////**************************************************************************
//// Initialize SPI Peripheral including all GPIO configuration and interrupt
//// for slave port
//// Parameters: NONE
////**************************************************************************
//void InitSPI(void)
//{
//	// Enable the SSI0 pheripheral and GPIO Pins
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//
//	// Unlock PF0 pin so data can be read in
//	HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
//	HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;
//
//	// Wait until the SSI1 module is ready
//	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1));
//
//	// Set up pin muxing for SSI module
//	GPIOPinConfigure(GPIO_PF0_SSI1RX);
//	GPIOPinConfigure(GPIO_PF1_SSI1TX);
//	GPIOPinConfigure(GPIO_PF2_SSI1CLK);
//	GPIOPinConfigure(GPIO_PF3_SSI1FSS);
//
//	// Set GPIO pins as type SSI
//	GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
//
//	// Configure the SSI
//	SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_SLAVE, SPI_CLOCK, 8);
//
//	// Enable the SSI module
//	SSIEnable(SSI1_BASE);
//
//    // Enable RX timeout interrupt.
//    SSIIntEnable(SSI1_BASE, SSI_RXTO);
//
//    // Clear FIFO
//    while(SSIDataGetNonBlocking(SSI1_BASE, &g_ulDataRx0[0]))
//    {
//    }
//
//    // Clear any pending interrupt
//    SSIIntClear(SSI1_BASE, SSI_RXTO);
//
//    // Enable the SSI0 interrupts to ARM core.
//    IntEnable(INT_SSI1);
//}

//**************************************************************************
// Function to send data on SPI line
// Parameters:	ulSSIBase; base address of SSI module; SSI0_BASE
//				ulSlaveGPIOBase; base address of SS GPIO Pin; GPIO_PORTB_BASE
//								 if on IO extender set to 1 or 2
//				ucSlaveGPIOPin; GPIO Pin for slave to be written to; GPIO_PIN_0
//				active; determines if slave's CS pin is active high or low
//						during transmission; low = 0; high = 1;
//				num_of_args; number of bytes being sent to slave; 3
//				...; data to send, can be as many arguments as needed
//**************************************************************************
void SPISend(unsigned long ulSSIBase, unsigned long ulSlaveGPIOBase, unsigned char ucSlaveGPIOPin, unsigned int active, unsigned int num_of_args, ...)
{
	unsigned int index;
	uint8_t Pin_State;
	uint32_t ui32DataRx[1];

	while(SSIDataGetNonBlocking(ulSSIBase, &ui32DataRx[0])); // Clear FIFO

	//stores list of variable number of arguments
	va_list vargs;

	//specifies the va_list to "open" and the last fixed argument
	//so vargs knows where to start looking
	va_start(vargs, num_of_args);

	if(active == 1) // CNV pin for ADC7687 goes high before data transfer and low after
		Pin_State = ~ucSlaveGPIOPin;
	else
		Pin_State = ucSlaveGPIOPin; // CS pin for DAC goes low before data transfer and high after

	// If statement to determine if CS pin is on IO extender or regular GPIO pin
	if(ulSlaveGPIOBase == 1)
	{
		IO_Ext_Set(IO_EXT1_ADDR, 2, ucSlaveGPIOPin, active);
	}
	else if(ulSlaveGPIOBase == 2)
	{
		IO_Ext_Set(IO_EXT2_ADDR, 2, ucSlaveGPIOPin, active);
	}
	else
	{
		GPIOPinWrite(ulSlaveGPIOBase, ucSlaveGPIOPin, ~Pin_State); // Toggle control pin for component we are communicating with
	}


	// Loop num_of_args times sending new byte each loop
	for(index = 0; index < num_of_args; index++)
	{
		SSIDataPut(ulSSIBase, va_arg(vargs, uint32_t)); // Send data

		while(SSIBusy(ulSSIBase)){} // Wait for SSI to finish transferring before raising SS pin
	}

	// If statement to determine if SS pin is on IO extender or regular GPIO pin
	if(ulSlaveGPIOBase == 1)
	{
		IO_Ext_Set(IO_EXT1_ADDR, 2, ucSlaveGPIOPin, 1 - active);
	}
	else if(ulSlaveGPIOBase == 2)
	{
		IO_Ext_Set(IO_EXT2_ADDR, 2, ucSlaveGPIOPin, 1 - active);
	}
	else
	{
		GPIOPinWrite(ulSlaveGPIOBase, ucSlaveGPIOPin, Pin_State); // Toggle control pin for component we are communicating with
	}

	//"close" variable args list
	va_end(vargs);
}

//*****************************************************************************
// Interrupt handler for SSI0 peripheral in slave mode.  It reads the interrupt
// status and if the interrupt is fired by a RX time out interrupt it reads the
// SSI0 RX FIFO and increments a counter to tell the main loop that RX timeout
// interrupt was fired.
//*****************************************************************************
void SSI0IntHandler(void)
{
	uint32_t ui32Status, ui32Index;
	static uint32_t Auto_Cal;

	// Read interrupt status.
	ui32Status = SSIIntStatus(SSI0_BASE, 1);

	// Clear interrupts.
	SSIIntClear(SSI0_BASE, ui32Status);

	// Check the reason for the interrupt.
	if(ui32Status & SSI_RXTO)
	{
//		// Read NUM_SSI_DATA bytes of data from SSI0 RX FIFO.
//		for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
//		{
//			SSIDataGet(SSI0_BASE, &g_ui32DataRx0[ui32Index]);
//		}
		ui32Index = 0;
		uint32_t cycles = 0;
		while(ui32Index < NUM_SSI_DATA)
		{
			ui32Index += SSIDataGetNonBlocking(SSI0_BASE, &g_ui32DataRx0[ui32Index]);
			cycles++;

			if(cycles > 30)	// Usually on takes 3 cycles to collect data, (1 cycle for each byte)
			{
				DEBUG_PRINT(UARTprintf("BT didn't send enough bytes, exiting interrupt!\n");)
				return;
			}

		}
//		UARTprintf("Loops = %d\n", cycles);

		// Interrupt is because of RX time out.  So increment counter to tell
		// main loop that RX timeout interrupt occurred.
		g_ulSSI0RXTO++;

		DEBUG_PRINT(
		if(gBoard > V1 && gDiagnostics >= 1)
//		if(g_ui32DataRx0[0] != 2)
			UARTprintf("Bytes received: %d, %d, %d \n", g_ui32DataRx0[0], g_ui32DataRx0[1], g_ui32DataRx0[2]);
		)

//		UARTprintf("BT: %d, %d, %d \n", g_ui32DataRx0[0], g_ui32DataRx0[1], g_ui32DataRx0[2]);
	}

//	// Check if command received was abort, handle this here because it is just setting a flag and doesn't take much processing time
//	if(g_ui32DataRx0[0] == ABORT)
//	{
//		gui32Error |= USER_CANCELLED;
//		g_state = STATE_IDLE;
//		g_next_state = STATE_IDLE;
//		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
//	}
//	// Check if command received was setting monochloramine cycle, handle this here because it is just setting a flag and doesn't take much processing time
//	else if(g_ui32DataRx0[0] == MONO_CL)
//	{
//		g_MonoCl = g_ui32DataRx0[1];
//		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
//
//		EEPROMProgram((uint32_t *) &g_MonoCl, OFFSET_MONO_CL_CYCLE, 4);	// This gets read back in update_MonoCl() which is called in InitBT()
//	}
//	else if(g_ui32DataRx0[0] == SAVE_AUTO_CAL_1)
//	{
//		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
//		Auto_Cal = g_ui32DataRx0[1];
//		Auto_Cal |= (g_ui32DataRx0[2] << 8) & 0x0000FF00;
//	}
//	else if(g_ui32DataRx0[0] == SAVE_AUTO_CAL_2)
//	{
//		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
//		Auto_Cal |= (g_ui32DataRx0[1] << 16) & 0x00FF0000;
//		Auto_Cal |= (g_ui32DataRx0[2] << 24) & 0xFF000000;
//
//		EEPROMProgram(&Auto_Cal, OFFSET_AUTO_CAL, 4);
//	}
//	else if(g_ui32DataRx0[0] == STARTING_OAD && g_ui32DataRx0[1] == 0 && g_ui32DataRx0[2] == 0)
//	{
//		g_state = STATE_UPDATING;
//		g_next_state = STATE_IDLE;
//
//		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
//	}
//	// Check if command received was setting alkalinity cycle, handle this here because it is just setting a flag and doesn't take much processing time
//	else if(g_ui32DataRx0[0] == ALKALINITY)
//	{
//		g_Alkalinity = g_ui32DataRx0[1];
//		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
//
//		EEPROMProgram((uint32_t *) &g_Alkalinity, OFFSET_ALKALINITY_CYCLE, 4);	// This gets read back in update_MonoCl() which is called in InitBT()
//	}

	switch(g_ui32DataRx0[0])
	{
	// Check if command received was abort, handle this here because it is just setting a flag and doesn't take much processing time
	case ABORT:
	{
		gui32Error |= USER_CANCELLED;
		g_state = STATE_IDLE;
		g_next_state = STATE_IDLE;
		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag

		break;
	}
	// Check if command received was setting monochloramine cycle, handle this here because it is just setting a flag and doesn't take much processing time
	case MONO_CL:
	{
		g_MonoCl = g_ui32DataRx0[1];
		g_FreeCl = g_ui32DataRx0[2];
		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag

		if(g_FreeCl != 0 && g_FreeCl != 1)
			g_FreeCl = 1;

		EEPROMProgram((uint32_t *) &g_MonoCl, OFFSET_MONO_CL_CYCLE, 4);	// This gets read back in update_MonoCl() which is called in InitBT()
		EEPROMProgram((uint32_t *) &g_FreeCl, OFFSET_FREE_CL_CYCLE, 4);	// This gets read back in update_MonoCl() which is called in InitBT()

//		UARTprintf("Free: %d\n", g_FreeCl);
//		UARTprintf("Mono: %d\n", g_MonoCl);

		break;
	}
	case SAVE_AUTO_CAL_1:
	{
		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
		Auto_Cal = g_ui32DataRx0[1];
		Auto_Cal |= (g_ui32DataRx0[2] << 8) & 0x0000FF00;

		break;
	}
	case SAVE_AUTO_CAL_2:
	{
		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
		Auto_Cal |= (g_ui32DataRx0[1] << 16) & 0x00FF0000;
		Auto_Cal |= (g_ui32DataRx0[2] << 24) & 0xFF000000;

		EEPROMProgram(&Auto_Cal, OFFSET_AUTO_CAL, 4);

//		UARTprintf("AC: %d; %d:%d:%d", Auto_Cal & 0xFF, (Auto_Cal >> 8) & 0xFF, (Auto_Cal >> 16) & 0xFF, (Auto_Cal >> 24) & 0xFF);

		break;
	}
	case STARTING_OAD:
	{
		if(/*g_ui32DataRx0[1] == 0 &&*/ g_ui32DataRx0[2] == 0)
		{
			g_state = STATE_UPDATING;
			g_next_state = STATE_IDLE;

			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
		}

		break;
	}
	// Check if command received was setting alkalinity cycle, handle this here because it is just setting a flag and doesn't take much processing time
	case ALKALINITY:
	{
		g_Alkalinity = g_ui32DataRx0[1];
		g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag

		EEPROMProgram((uint32_t *) &g_Alkalinity, OFFSET_ALKALINITY_CYCLE, 4);	// This gets read back in update_Alkalinity() which is called in InitBT()

		break;
	}
	default:
		break;
	}
}

////*****************************************************************************
//// Interrupt handler for SSI1 peripheral in slave mode.  It reads the interrupt
//// status and if the interrupt is fired by a RX time out interrupt it reads the
//// SSI1 RX FIFO and increments a counter to tell the main loop that RX timeout
//// interrupt was fired.
////*****************************************************************************
//void SSI1IntHandler(void)
//{
//	unsigned long ulStatus, ulIndex;
//
//	// Read interrupt status.
//	ulStatus = SSIIntStatus(SSI1_BASE, 1);
//
//	// Delay to give SPI transaction time to finish
//	SysCtlDelay(2000);
//
//	// Check the reason for the interrupt.
//	if(ulStatus & SSI_RXTO)
//	{
//		// Interrupt is because of RX time out.  So increment counter to tell
//		// main loop that RX timeout interrupt occurred.
//		g_ulSSI0RXTO++;
//
//		// Read NUM_SSI_DATA bytes of data from SSI1 RX FIFO.
//		for(ulIndex = 0; ulIndex < NUM_SSI_DATA; ulIndex++)
//		{
//			SSIDataGet(SSI1_BASE, &g_ulDataRx0[ulIndex]);
//		}
//	}
//
//	// Clear interrupts.
//	SSIIntClear(SSI1_BASE, ulStatus);
//}

//*****************************************************************************
//
//! This function sets up UART0 to be used for a console I/O
//!
//! \param none.
//!
//! This function sets up UART0 to be used for a console to display information
//! as the application is running. Useful not only during code development!
//!
//! The following UART signals are configured only for displaying console
//! messages. These functionality may not not be required for production use.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! \return None.
//
//*****************************************************************************
void
InitConsole(void)
{
	//
	// Enable GPIO port A which is used for UART0 pins.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port PA0 and PA1.
    // This step is not necessary if your part does not support pin muxing.
    //
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

//    // Set up UART interrupt, need this to send information in
//    IntEnable(INT_UART0);
//    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); // Enable Receive Interrupt and Receive Timeout Interrupt
//    IntPrioritySet(INT_UART0, 0x20); // Set to interrupt priority 1, interrupt priority is upper 3 bits

	//
	// Print a string to show that the console is active.
	//
//	UARTprintf("UARTStdioInit ...\n");
	SysCtlDelay(2000000);
}

//void UARTIntHandler(void)
//{
//	uint32_t ui32Status;
//	ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
//	UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
//	while(UARTCharsAvail(UART0_BASE)) //loop while there are chars
//	{
//		UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE)); //echo character
////		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //blink LED
////		SysCtlDelay(SysCtlClockGet() / (1000 * 3)); //delay ~1 msec
////		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); //turn off LED
//	}
//}
