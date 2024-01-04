//*****************************************************************************
//
// Communication.h - Functions used by e-SENS firmware to communicate with
// components and bluetooth chip
// Includes functions related to I2C and SPI
//
// Author: Jason Castillo
//
//*****************************************************************************

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************
#define SPI_CLOCK		2500000 // 10 MHz Max
#define NUM_SSI_DATA 	3
#define BT_ADDR			0x0E

extern volatile unsigned long g_ulSSI0RXTO;
extern uint32_t g_ui32DataRx0[NUM_SSI_DATA];

//extern uint8_t g_error_byte_2;
//extern uint8_t g_error_byte_3;
extern volatile uint8_t g_MonoCl;
extern volatile uint8_t g_FreeCl;
extern volatile uint8_t g_Alkalinity;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void InitI2C(void);

extern void I2CSend(unsigned long ulI2CBase, uint8_t slave_addr, unsigned int num_of_args, ...);

extern unsigned int I2CReceive(unsigned long ulI2CBase, uint8_t slave_addr, unsigned char reg, unsigned int num_of_bytes);

extern void InitSPI(void);

//extern void SPIReset(void);

extern void SPISend(unsigned long ulSSIBase, unsigned long ulSlaveGPIOBase, unsigned char ucSlaveGPIOPin, unsigned int active, unsigned int num_of_args, ...);

extern void SSI0IntHandler(void);

extern void InitConsole(void);

//extern void UARTIntHandler(void);

#endif /* COMMUNICATION_H_ */
