//*****************************************************************************
//
// Bluetooth.h - Functions used in e-SENS firmware that deal directly with BT
//
// Author: Jason Castillo
//
//*****************************************************************************

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************
extern uint8_t gCartridge;
//extern uint8_t gMemory;
extern uint8_t gStatus;
extern uint8_t gOperation;

extern uint8_t gPumping;	// Flag to set while pumping, used to prevent battery interrupt taking too long as it stalls the pump

// Define operation and status
#define STATUS_IDLE			0
#define OPERATION_IDLE		0
#define OPERATION_HIBERNATE	1

#define STATUS_CALIBRATION			1
#define OPERATION_CAL_PRECHECK		1
#define OPERATION_CAL_RINSE			2
#define OPERTAION_CAL_1				5
#define OPERATION_CAL_2				4
#define OPERATION_FCL_ACTIVATION	3
#define OPERATION_CAL_POSTCHECK		6
#define OPERATION_CAL_STORE			7
#define OPERATION_EMPTY_WASTE		8
#define OPERATION_CAL_COMPLETE		9

//#define STATUS_CALIBRATION			1
//#define OPERATION_CAL_PRECHECK		1
//#define OPERATION_CAL_RINSE			2
//#define OPERTAION_CAL_1				3
//#define OPERATION_CAL_2				4
//#define OPERATION_FCL_ACTIVATION	5
//#define OPERATION_CAL_POSTCHECK		6
//#define OPERATION_CAL_STORE			7
//#define OPERATION_EMPTY_WASTE		8
//#define OPERATION_CAL_COMPLETE		9
#define OPERATION_CAL_FAILED		99

#define STATUS_TEST					2
#define OPERATION_TEST_PRECHECK		1
#define OPERATION_TEST_RINSE		2
#define OPERATION_SAMPLE			3
#define OPERATION_ALKALINITY		4
#define OPERATION_CL_ACTIVATION		5
#define OPERATION_SAMPLE_B1			6
#define OPERATION_SAMPLE_B2			7
#define OPERATION_TEST_POSTCHECK	8
#define OPERATION_TEST_STORE		9
#define OPERATION_TEST_EMPTY_WASTE	10
#define OPERATION_TEST_COMPLETE		11
#define OPERATION_TEST_FAILED		99

//#define STATUS_CALIBRATION			1
//#define OPERATION_CAL_PRECHECK		1
//#define OPERATION_CAL_RINSE			2
//#define OPERATION_FCL_ACTIVATION	3
//#define OPERATION_CAL_3				4
//#define OPERATION_CAL_3_B1			5
//#define OPERATION_CAL_2				6
//#define OPERATION_CAL_2_B1			7
//#define OPERTAION_CAL_1				8
//#define OPERATION_CAL_POSTCHECK		9
//#define OPERATION_TCL_ACTIVATION	10
////#define OPERATION_MCL_CAL			11
//#define OPERATION_CAL_STORE			11
//#define OPERATION_EMPTY_WASTE		12
//#define OPERATION_CAL_COMPLETE		13
//#define OPERATION_CAL_FAILED		99
//
//#define STATUS_TEST					2
//#define OPERATION_TEST_PRECHECK		1
//#define OPERATION_TEST_RINSE		2
//#define OPERATION_CL_ACTIVATION		3
//#define OPERATION_SAMPLE			4
//#define OPERATION_SAMPLE_B1			5
//#define OPERATION_SAMPLE_B2			6
//#define OPERATION_TEST_POSTCHECK	8
//#define OPERATION_EMPTY_SAMPLE		7
//#define OPERATION_TEST_STORE		9
//#define OPERATION_TEST_EMPTY_WASTE	10
//#define OPERATION_TEST_COMPLETE		11
//#define OPERATION_TEST_FAILED		99

// BT to Tiva instructions
#define START_CALIBRATION			0x00
#define START_TEST					0x01
#define REQUEST_TIVA_INSTRUCTION	0x02
#define ABORT						0x03
#define HARDWARE_ALERT				0x05
#define UPDATE_TIVA_FIRMWARE		0x07
#define MONO_CL						0x08
#define RESET						0x09
#define SAVE_AUTO_CAL_1				0x0A
#define SAVE_AUTO_CAL_2				0x0B
#define BLINK_LED					0x0D	// This command is received when a device connects to the BT chip
#define CONTINUE_CAL				0x0E
#define CONTINUE_TEST				0x0F
#define STARTING_OAD				0x11
#define BOOTUP						0x18
//#define IGNORE_1					0x40
//#define IGNORE_2					0x41
#define ERROR_CODE_1				0xFE
#define ERROR_CODE_2				0xFF

// Parameters coming with start calibration
#define START_AUTO_CAL	0x01
#define START_USER_CAL	0x02

// Tiva to BT instructions
#define UPDATE_TEST_CAL_NUMBER	0x00
#define UPDATE_DEV_INFO			0x01
#define UPDATE_CAL				0x02
#define UPDATE_TEST				0x03
#define REQUEST_SYS_STATUS		0x04
#define CURRENT_STATUS			0x05
#define BATTERY_STATUS			0x06
#define UPDATE_AUTO_CAL_1		0x0A
#define UPDATE_AUTO_CAL_2		0x0B
#define CARTRIDGE_CONNECTED		0x0C
#define INIT_LED_EXT			0x0E
#define SET_LED					0x0F
#define USING_I2C				0x10
#define RESET_TIVA				0x12
#define TEST_FLASH				0x13
#define TEST_HIBERNATE			0x14
#define TEST_CART_MEM			0x15
#define ALKALINITY				0x16
#define UPDATE_CART_TEMP		0x17

// Set offsets for bitwise saving of Calibration status to use in the app
#define APP_CALIBRATED	0
#define APP_ALKALINITY	1
#define APP_PH			2
#define APP_CALCIUM		3
#define APP_MAGNESIUM	4
#define APP_AMMONIUM	5
#define APP_COND		6
#define APP_CHLORINE	7

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void InitBT(void);

extern void Reset_BT(void);

extern void update_Status(uint8_t Status, uint8_t Operation);

extern void update_Test(int Test);

extern void update_Cal(int Cal);

extern void update_Dev_Info(void);

extern void update_TCNumbers(void);

extern void update_Error(void);

extern void update_Battery(uint8_t Pumping);

extern void update_Auto_Cal(void);

extern void update_MonoCl(void);

extern void update_Alkalinity(void);

extern void update_Cartridge_Status(uint8_t Connected);

extern void update_TivaI2C(uint8_t State);

extern void BT_ResetTiva(void);

extern void update_CartTemp(void);

#ifndef OAD
extern uint8_t BT_TestFlash(void);

extern uint8_t BT_TestCartMem(void);

extern uint8_t BT_TestHibernate(uint8_t Test);
#endif

#endif /* BLUETOOTH_H_ */
