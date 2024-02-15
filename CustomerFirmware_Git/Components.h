//*****************************************************************************
//
// Components.h - Functions used in e-SENS firmware to control physical components
// Includes functions controlling: IO Extenders, DAC, ADC, and Memory
//
// Author: Jason Castillo
//
//*****************************************************************************

#ifndef COMPONENTS_H_
#define COMPONENTS_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************
//#define COND_FREQ	1000
#define COND_FREQ	(gABoard >= ARV1_0B ? 5000 : 1000)

//// This is the normal configuration
//// Defines for which sensor is on ADC channels
//#define ORP_CH		0
//#define TEMP_CH		0
//#define TEMP_CH_2	6
//#define PH_2_CH		3
//#define PH_1_CH		5
//#define PH_3_CH		2
//#define CA_2_CH		4
//#define CA_1_CH		1
//#define NH4_CH		1
//#define CO2_2_CH	2
//#define CO2_1_CH	4
//#define TH_2_CH 	3
//#define TH_1_CH 	5
//
//// Defines for which sensor is on which ADC
//#define ORP_ADC		ADC2_CS_B
//#define TEMP_ADC	ADC1_CS_B
//#define PH_2_ADC	ADC2_CS_B
//#define PH_1_ADC	ADC2_CS_B
//#define PH_3_ADC	ADC1_CS_B
//#define CA_2_ADC	ADC1_CS_B
//#define CA_1_ADC	ADC2_CS_B
//#define NH4_ADC		ADC1_CS_B
//#define CO2_2_ADC	ADC2_CS_B
//#define CO2_1_ADC	ADC2_CS_B
//#define TH_2_ADC	ADC1_CS_B
//#define TH_1_ADC	ADC1_CS_B



// This reordered configuration is for sensors in order down snake channel, Ca, NH4, TH, pH
// Defines for which sensor is on ADC channels
#define ORP_CH		0
#define ORP_IREAD_CH		6
#define TEMP_CH		0
#define TEMP_CH_2	6
#define PH_2_CH		4
#define PH_1_CH		5
#define PH_3_CH		2
#define CA_2_CH		1
#define CA_1_CH		2
#define NH4_CH		5
#define CO2_2_CH	4
#define CO2_1_CH	3
#define TH_2_CH 	1
#define TH_1_CH 	3

// Defines for which sensor is on which ADC
#define ORP_ADC		ADC2_CS_B
#define ORP_IREAD_ADC		ADC2_CS_B
#define TEMP_ADC	ADC1_CS_B
#define PH_2_ADC	ADC2_CS_B
#define PH_1_ADC	ADC2_CS_B
#define PH_3_ADC	ADC2_CS_B
#define CA_2_ADC	ADC1_CS_B
#define CA_1_ADC	ADC1_CS_B
#define NH4_ADC		ADC1_CS_B
#define CO2_2_ADC	ADC1_CS_B
#define CO2_1_ADC	ADC1_CS_B
#define TH_2_ADC	ADC2_CS_B
#define TH_1_ADC	ADC2_CS_B


//// This reordered configuration is for sensors including hydrogen 2 ionophore
//// Defines for which sensor is on ADC channels
//#define ORP_CH		0
//#define TEMP_CH		0
//#define TEMP_CH_2	6
//#define PH_2_CH		4
//#define PH_1_CH		5
//#define PH_3_CH		2
//#define CA_2_CH		1
//#define CA_1_CH		2
//#define NH4_CH		5
//#define CO2_2_CH	4
//#define CO2_1_CH	3
//#define TH_2_CH 	1
//#define TH_1_CH 	3
//
//// Defines for which sensor is on which ADC
//#define ORP_ADC		ADC2_CS_B
//#define TEMP_ADC	ADC1_CS_B
//#define PH_2_ADC	ADC2_CS_B
//#define PH_1_ADC	ADC2_CS_B
//#define PH_3_ADC	ADC2_CS_B
//#define CA_2_ADC	ADC1_CS_B
//#define CA_1_ADC	ADC1_CS_B
//#define NH4_ADC		ADC1_CS_B
//#define CO2_2_ADC	ADC1_CS_B
//#define CO2_1_ADC	ADC1_CS_B
//#define TH_2_ADC	ADC2_CS_B
//#define TH_1_ADC	ADC2_CS_B


//// This is for 180 turned reordered configuration
//// Defines for which sensor is on ADC channels
//#define ORP_CH		0
//#define TEMP_CH		0
//#define TEMP_CH_2	6
//#define PH_2_CH		2
//#define PH_1_CH		1
//#define PH_3_CH		4
//#define CA_2_CH		5
//#define CA_1_CH		4
//#define NH4_CH		1
//#define CO2_2_CH	2
//#define CO2_1_CH	3
//#define TH_2_CH 	5
//#define TH_1_CH 	3
//
//// Defines for which sensor is on which ADC
//#define ORP_ADC		ADC2_CS_B
//#define TEMP_ADC	ADC1_CS_B
//#define PH_2_ADC	ADC1_CS_B
//#define PH_1_ADC	ADC1_CS_B
//#define PH_3_ADC	ADC1_CS_B
//#define CA_2_ADC	ADC2_CS_B
//#define CA_1_ADC	ADC2_CS_B
//#define NH4_ADC		ADC2_CS_B
//#define CO2_2_ADC	ADC2_CS_B
//#define CO2_1_ADC	ADC2_CS_B
//#define TH_2_ADC	ADC1_CS_B
//#define TH_1_ADC	ADC1_CS_B


//// This is the configuration with pH 3 and Ca 2 swapped
//// Defines for which sensor is on ADC channels
//#define ORP_CH		0
//#define TEMP_CH		0
//#define TEMP_CH_2	6
//#define PH_2_CH		3
//#define PH_1_CH		5
//#define PH_3_CH		4
//#define CA_2_CH		2
//#define CA_1_CH		1
//#define NH4_CH		1
//#define CO2_2_CH	2
//#define CO2_1_CH	4
//#define TH_2_CH 	3
//#define TH_1_CH 	5
//
//// Defines for which sensor is on which ADC
//#define ORP_ADC		ADC2_CS_B
//#define TEMP_ADC	ADC1_CS_B
//#define PH_2_ADC	ADC2_CS_B
//#define PH_1_ADC	ADC2_CS_B
//#define PH_3_ADC	ADC1_CS_B
//#define CA_2_ADC	ADC1_CS_B
//#define CA_1_ADC	ADC2_CS_B
//#define NH4_ADC		ADC1_CS_B
//#define CO2_2_ADC	ADC2_CS_B
//#define CO2_1_ADC	ADC2_CS_B
//#define TH_2_ADC	ADC1_CS_B
//#define TH_1_ADC	ADC1_CS_B

extern uint16_t gLED_State;
extern uint8_t gui8IO_Ext1_Reg2;
extern uint8_t gui8IO_Ext1_Reg3;
extern uint8_t gui8IO_Ext2_Reg2;
extern uint8_t gui8IO_Ext2_Reg3;

#define IO_EXT1_ADDR  	0x74
#define IO_EXT2_ADDR  	0x75
#define MEM_ADDR 		0x50
#define ACCEL_ADDR		0x1D
#define RTC_ADDR		0x68
// Digital V7.2 LED IO Extender is 0x20
// Digital V7.3 LED IO Extender is 0x74 THIS NEEDS TO BE CHANGED

// I/O Extender 1 Pins
#define ADC1_CS_B     	0x01
#define ADC2_CS_B     	0x02
#define WAVE_GEN_CS_B  	0x04
#define ADC4_CS_B     	0x08
#define DAC1_CS_B     	0x10
#define DAC1_LDAC_B   	0x20
#define DAC1_WAKEUP_b 	0x40
#define DAC1_CLR_b    	0x80

#define REF_EL_SWA		0x01
#define REF_EL_SWB		0x02
#define WORK_EL_SWA		0x04
#define WORK_EL_SWB		0x08
#define OSC_EN			0x10
#define COND_GAIN_SWB	0x20
#define COND_GAIN_SWA	0x40
#define COND_SW			0x80

// I/O Extender 2 Pins
#define WORK_EL_SHORT			0x01
#define ORP_SW_A0				0x02
#define ORP_SW_A1				0x04
#define I2C_SENSOR_ON_B			0x08
#define ADC5_CS_B				0x10
#define ADC5_START				0x20
#define TURB_ON_B				0x80

#define WORK_EL_HIGH_CURRENT	0x01
#define WORK_EL_MID_CURRENT		0x02
#define WORK_EL_LOW_CURRENT		0x04
#define COUNTER_GND				0x08
#define COUNTER_EL_DRIVE		0x08
#define METALS_WE_SWB			0x10
#define METALS_WE_SWA			0x20
#define METALS_SW_EN			0x40
#define COND_SHORT_SW			0x80

//#ifndef SENSOR_PCB_V6
//
//// Defines for which sensor is on ADC channels
//#define ORP_CH		0
//#define TEMP_CH		0
//#define PH_2_CH		1
//#define PH_1_CH		3
//#define PH_3_CH		5
//#define TEMP_CH_2	6
//#define CA_2_CH		2
//#define CA_1_CH		4
//#define NH4_CH		1
//#define CO2_2_CH	2
//#define CO2_1_CH	4
//#define TH_2_CH 	3
//#define TH_1_CH 	5
//
//// Defines for which sensor is on which ADC
//#define ORP_ADC		ADC2_CS_B
//#define TEMP_ADC	ADC1_CS_B
//#define PH_2_ADC	ADC2_CS_B
//#define PH_1_ADC	ADC2_CS_B
//#define PH_3_ADC	ADC2_CS_B
//#define CA_2_ADC	ADC1_CS_B
//#define CA_1_ADC	ADC1_CS_B
//#define NH4_ADC		ADC1_CS_B
//#define CO2_2_ADC	ADC2_CS_B
//#define CO2_1_ADC	ADC2_CS_B
//#define TH_2_ADC	ADC1_CS_B
//#define TH_1_ADC	ADC1_CS_B
//
//#else
//
//// Defines for which sensor is on ADC channels
//#define ORP_CH		0
//#define TEMP_CH		0
//#define TEMP_CH_2	6
//#define PH_2_CH		3
//#define PH_1_CH		5
//#define PH_3_CH		2
//#define CA_2_CH		4
//#define CA_1_CH		1
//#define NH4_CH		1
//#define CO2_2_CH	2
//#define CO2_1_CH	4
//#define TH_2_CH 	3
//#define TH_1_CH 	5
//
//// Defines for which sensor is on which ADC
//#define ORP_ADC		ADC2_CS_B
//#define TEMP_ADC	ADC1_CS_B
//#define PH_2_ADC	ADC2_CS_B
//#define PH_1_ADC	ADC2_CS_B
//#define PH_3_ADC	ADC1_CS_B
//#define CA_2_ADC	ADC1_CS_B
//#define CA_1_ADC	ADC2_CS_B
//#define NH4_ADC		ADC1_CS_B
//#define CO2_2_ADC	ADC2_CS_B
//#define CO2_1_ADC	ADC2_CS_B
//#define TH_2_ADC	ADC1_CS_B
//#define TH_1_ADC	ADC1_CS_B
//#endif

// Battery Address and Registers
#define BAT_SLAVE_ADDR 	0x36 // Slave address of battery IC; 8-bit address 0x6C
#define REP_CAP_REG 	0x05 // Reported capacity register
#define REP_SOC_REG 	0x06 // State of charge register
#define TTE_REG 		0x11 // Estimated time to empty register
#define TTF_REG 		0x20 // Estimated time to full register
#define AGE_REG			0x07 // Age % of battery
#define CYC_REG			0x17 // Cycles of battery

#ifndef MCU_ZXR	// Moved these defines to the PinMap.h file
// Button LEDs
#define LED_BUT_RED_BASE	GPIO_PORTD_BASE
#define LED_BUT_RED_PIN		GPIO_PIN_0
#define LED_BUT_GREEN_BASE	GPIO_PORTD_BASE
#define LED_BUT_GREEN_PIN	GPIO_PIN_1
#define LED_BUT_BLUE_BASE	GPIO_PORTE_BASE
#define LED_BUT_BLUE_PIN	GPIO_PIN_5

// Charging LEDs
#define LED_CHG_RED_BASE	GPIO_PORTB_BASE
#define LED_CHG_RED_PIN		GPIO_PIN_4
#define LED_CHG_GREEN_BASE	GPIO_PORTB_BASE
#define LED_CHG_GREEN_PIN	GPIO_PIN_5
#define LED_CHG_YELLOW_BASE	GPIO_PORTE_BASE
#define LED_CHG_YELLOW_PIN	GPIO_PIN_4
#endif	// MCU_ZXR

// LED defines to be used with SetLED function
#define RED_BUTTON			0x0010
#define RED_BUTTON_BLINK	0x0030
#define BLUE_BUTTON			0x0004
#define BLUE_BUTTON_BLINK	0x000C
#define GREEN_BUTTON		0x0001
#define GREEN_BUTTON_BLINK	0x0003
#define RED_BUTTON_V			0x1000
#define RED_BUTTON_BLINK_V		0x3000
#define BLUE_BUTTON_V			0x0100
#define BLUE_BUTTON_BLINK_V		0x0300
#define GREEN_BUTTON_V			0x0400
#define GREEN_BUTTON_BLINK_V	0x0C00
#define LED_BLINK				0x2AAA

#define RED_CHARGE			0x0040
#define RED_CHARGE_BLINK	0x00C0
#define GREEN_CHARGE		0x4000
#define YELLOW_CHARGE		0x8000

#ifdef MEMORY_V6
// Pages in memory
#define PAGE_DEVICE_INFO	0
#define PAGE_CARTRIDGE_INFO	1
#define PAGE_FACTORY_CAL	2
#define PAGE_SOLUTIONS		3
#define PAGE_CAL			4
#define PAGE_TEST			112

#define PAGES_FOR_TEST	2
#define PAGES_FOR_CAL	2

// Memory offset for device information page
//#define OFFSET_MEMORY_CONFIG	101
#define MEMORY_512K		1
#define MEMORY_1M		2
#define GMEMORY			MEMORY_1M

// Memory offset of Cartridge Information
#define OFFSET_CARTRIDGE_SN				0
#define OFFSET_SENSOR_SN				7
#define OFFSET_SENSOR_MAX_DAYS			14
#define OFFSET_COMPLETED_TESTS			20
#define OFFSET_COMPLETED_CALS			22
//#define OFFSET_SENSOR_ASSEMBLY_DATE		24
#define OFFSET_SENSOR_EXPIRATION_DATE		24
#define OFFSET_SENSOR_HYDRATION_DATE	28
#define OFFSET_SENSOR_USAGE				32
#define OFFSET_SENSOR_MAX_TESTS			33
//#define OFFSET_SOLUTIONS_BATCH			35
#define OFFSET_SENSOR_MAX_CALS			35
#define OFFSET_SENSOR_CONFIGURATION		39
#define OFFSET_MIN_CART_TEMP			40
#define OFFSET_MAX_CART_TEMP			44
#define OFFSET_MIN_TEMP_DATE			48
#define OFFSET_MIN_TEMP_TIME			52
#define OFFSET_MAX_TEMP_DATE			54
#define OFFSET_MAX_TEMP_TIME			58
#define OFFSET_CI_ZERO					60

#define OFFSET_VALVE_SETUP				128

// Memory offset of Factory Calibration data
#define OFFSET_TEMP_COEFFICIENT_A	0
#define OFFSET_TEMP_COEFFICIENT_B	4
#define OFFSET_TEMP_COEFFICIENT_C	8
#define OFFSET_COND_LOW_POINT_CAL	12
#define OFFSET_COND_READ_LOW_POINT	16
#define OFFSET_TCL_SLOPE			20
#define OFFSET_TCL_INT				24
#define OFFSET_TCL_SLOPE_HIGH		28
#define OFFSET_TCL_INT_HIGH			32
#define OFFSET_FCL_SLOPE			36
#define OFFSET_FCL_INT				40
#define OFFSET_FCL_SLOPE_HIGH		44
#define OFFSET_FCL_INT_HIGH			48
//#define OFFSET_NITRITE_BLANK		52
#define OFFSET_FACTORY_COND_SLOPE	52
#define OFFSET_THERM_CORRECTION		56
#define OFFSET_FC_ZERO				60	// Placed in the middle because the following data is used during factory calibration but will not be used after and will not be displayed over BT

#define OFFSET_TCL_MID_POINT		88
#define OFFSET_FCL_MID_POINT		92
#define OFFSET_FCL_HIGH_READ		96
#define OFFSET_TCL_HIGH_READ		100
#define OFFSET_TEMP_LOW					104
#define OFFSET_TEMP_RESISTANCE_LOW		108
#define OFFSET_TEMP_MID					112
#define OFFSET_TEMP_RESISTANCE_MID		116
#define OFFSET_TEMP_HIGH				120
#define OFFSET_TEMP_RESISTANCE_HIGH		124
#define OFFSET_CAL_COND_HIGH_CURRENT	128

// Memory offset of solution data
#define OFFSET_T1_HCL_N		0
#define OFFSET_RINSE_PH		4
#define OFFSET_RINSE_NH4	8
#define OFFSET_RINSE_CA		12
#define OFFSET_RINSE_TH		16
#define OFFSET_RINSE_COND	20
#define OFFSET_CAL_1_PH		24
#define OFFSET_CAL_1_NH4	28
#define OFFSET_CAL_1_CA		32
#define OFFSET_CAL_1_TH		36
#define OFFSET_CAL_1_COND	40
#define OFFSET_NITRITE_SA	44
#define OFFSET_CLEAN_CA		44
#define OFFSET_CAL_2_PH		48
#define OFFSET_CAL_2_NH4	52
#define OFFSET_CAL_2_CA		56
#define OFFSET_CAL_2_TH		60
#define OFFSET_CAL_2_COND	64
#define OFFSET_CLEAN_PH		68
#define OFFSET_CLEAN_NH4	72
#define OFFSET_CLEAN_COND	76
#define OFFSET_SOL_ZERO		80
#define OFFSET_CLEAN_TH		80

#define OFFSET_IS_RINSE		84
#define OFFSET_IS_CLEAN		88
#define OFFSET_IS_CAL_1		92
#define OFFSET_IS_CAL_2		96
#define OFFSET_KT_RINSE		100
#define OFFSET_KT_CAL_1		104
#define OFFSET_KT_CAL_2		108
#define OFFSET_KT_CLEAN_SQ	112
#define OFFSET_KT_CLEAN_LN	116

#define OFFSET_REPUMP_CLEAN		124
#define OFFSET_REPUMP_RINSE		125
#define OFFSET_REPUMP_CAL_1		126
#define OFFSET_REPUMP_CAL_2		127

#define OFFSET_RINSE_COND_TCOMP		128
#define OFFSET_CAL_1_COND_TCOMP		132
#define OFFSET_CAL_2_COND_TCOMP		136
#define OFFSET_CLEAN_COND_TCOMP		140

// Memory offset of calibration info
#define OFFSET_CAL_NUMBER	0
#define OFFSET_CAL_USER		2
#define OFFSET_RINSE_MID_RAW	2
#define OFFSET_RINSE_HIGH_RAW	6
#define OFFSET_CLEAN_MID_RAW	10
#define OFFSET_CLEAN_HIGH_RAW	14
#define OFFSET_CAL_COND_LOW_ALT_SLOPE	18
#define OFFSET_CAL_LOCATION		22
#define OFFSET_CALIBRATED_STATUS	29
#define OFFSET_ALK_SLOPE_PER	30
#define OFFSET_PH_SLOPE_PER		32
#define OFFSET_CA_SLOPE_PER		34
#define OFFSET_MG_SLOPE_PER		36
#define OFFSET_NH4_SLOPE_PER	38
#define OFFSET_COND_SLOPE_PER	40
#define OFFSET_CAL_DATE		42
#define OFFSET_CAL_TIME		46
#define OFFSET_CAL_BATTERY	49
#define OFFSET_CAL_LOG_K	50
#define OFFSET_CAL_GPS		54
#define OFFSET_CAL_ERROR	62
#define OFFSET_CAL_ZERO		66

#define OFFSET_CR_ISE_1_POST		67
#define OFFSET_CR_ISE_2_POST		71
#define OFFSET_CR_ISE_3_POST		75
#define OFFSET_CR_ISE_4_POST		79
#define OFFSET_CR_ISE_5_POST		83
#define OFFSET_CR_ISE_6_POST		87
#define OFFSET_CR_ISE_7_POST		91
#define OFFSET_CR_ISE_8_POST		95
#define OFFSET_CR_ISE_9_POST		99
#define OFFSET_CR_ISE_10_POST		103
#define OFFSET_CR_T_POSTRINSE		107

#define OFFSET_PH_1_LAST_P_CAL			118
#define OFFSET_PH_2_LAST_P_CAL			119
#define OFFSET_PH_3_LAST_P_CAL			120
#define OFFSET_TH_1_LAST_P_CAL			121
#define OFFSET_TH_2_LAST_P_CAL			122
#define OFFSET_NH4_1_LAST_P_CAL			123
#define OFFSET_NH4_2_LAST_P_CAL			124
#define OFFSET_NH4_3_LAST_P_CAL			125
#define OFFSET_CA_1_LAST_P_CAL			126
#define OFFSET_CA_2_LAST_P_CAL			127

// Memory offset of calibration data
#define OFFSET_T_CAL			(0 + 128)
#define OFFSET_ISE_1_SLOPE		(4 + 128)
#define OFFSET_ISE_2_SLOPE		(8 + 128)
#define OFFSET_ISE_3_SLOPE		(12 + 128)
#define OFFSET_ISE_4_SLOPE		(16 + 128)
#define OFFSET_ISE_5_SLOPE		(20 + 128)
#define OFFSET_ISE_6_SLOPE		(24 + 128)
#define OFFSET_ISE_7_SLOPE		(28 + 128)
#define OFFSET_ISE_8_SLOPE		(32 + 128)
#define OFFSET_ISE_9_SLOPE		(36 + 128)
#define OFFSET_ISE_10_SLOPE		(40 + 128)
#define OFFSET_COND_R1_SLOPE	(44 + 128)
#define OFFSET_COND_R1_INT		(48 + 128)
#define OFFSET_COND_R2_SLOPE	(52 + 128)
#define OFFSET_COND_R2_INT		(56 + 128)
#define OFFSET_COND_R3_SLOPE	(60 + 128)
#define OFFSET_COND_R3_INT		(64 + 128)
#define OFFSET_CAL_DEVICE_SERIAL	(68 + 128)
#define OFFSET_CAL_CHOSEN_SENSORS	(75 + 128)
#define OFFSET_CAL_STATUS		(76 + 128)
#define OFFSET_CAL_DATA_ZERO	(80 + 128)

// Memory offset of calibration RAW data
#define OFFSET_ISE_1_INT		(0 + 256)
#define OFFSET_ISE_2_INT		(4 + 256)
#define OFFSET_ISE_3_INT		(8 + 256)
#define OFFSET_ISE_4_INT		(12 + 256)
#define OFFSET_ISE_5_INT		(16 + 256)
#define OFFSET_ISE_6_INT		(20 + 256)
#define OFFSET_ISE_7_INT		(24 + 256)
#define OFFSET_ISE_8_INT		(28 + 256)
#define OFFSET_ISE_9_INT		(32 + 256)
#define OFFSET_ISE_10_INT		(36 + 256)
#define OFFSET_CR_ISE_1_RINSE	(40 + 256)
#define OFFSET_CR_ISE_2_RINSE	(44 + 256)
#define OFFSET_CR_ISE_3_RINSE	(48 + 256)
#define OFFSET_CR_ISE_4_RINSE	(52 + 256)
#define OFFSET_CR_ISE_5_RINSE	(56 + 256)
#define OFFSET_CR_ISE_6_RINSE	(60 + 256)
#define OFFSET_CR_ISE_7_RINSE	(64 + 256)
#define OFFSET_CR_ISE_8_RINSE	(68 + 256)
#define OFFSET_CR_ISE_9_RINSE	(72 + 256)
#define OFFSET_CR_ISE_10_RINSE	(76 + 256)
#define OFFSET_CR_ZERO			(80 + 256)

#define OFFSET_CR_CAL_2_MV	(88 + 256)
#define OFFSET_CR_CAL_1_MV	(198 + 256)
#define OFFSET_CR_CLEAN_MV	216

#define OFFSET_MG_1_SLOPE		(156 + 256)
#define OFFSET_MG_2_SLOPE		(160 + 256)
#define OFFSET_MG_1_INT			(164 + 256)
#define OFFSET_MG_2_INT			(168 + 256)
#define OFFSET_MG_1_LOG_K		(174 + 256)
#define OFFSET_MG_2_LOG_K		(178 + 256)
#define OFFSET_MG_1_MV_CAL_1	(182 + 256)
#define OFFSET_MG_1_PH_SLOPE	(182 + 256)
#define OFFSET_MG_2_MV_CAL_1	(186 + 256)
#define OFFSET_MG_2_PH_SLOPE	(186 + 256)
#define OFFSET_CA_1_MV_CAL_2	(190 + 256)
#define OFFSET_CA_2_MV_CAL_2	(194 + 256)

#define OFFSET_TH_1_LOG_K		(80 + 256)
#define OFFSET_TH_2_LOG_K		(84 + 256)
#define OFFSET_CA_1_LOG_K		(238 + 256)
#define OFFSET_CA_2_LOG_K		(242 + 256)
#define OFFSET_NH4_1_LOG_K		(246 + 256)
#define OFFSET_NH4_2_LOG_K		(250 + 256)

// Saving this in the same offset for both calibrations and tests
#define OFFSET_CLEAN_CATH_START			(128 + 256)
#define OFFSET_CLEAN_CATH_FINAL			(132 + 256)

#define OFFSET_CLEAN_ARRAY_1_START		(136 + 256)
#define OFFSET_CLEAN_ARRAY_2_START		(138 + 256)
#define OFFSET_CLEAN_ARRAY_3_START		(140 + 256)
#define OFFSET_CLEAN_ARRAY_4_START		(142 + 256)
#define OFFSET_CLEAN_ARRAY_5_START		(144 + 256)

#define OFFSET_CLEAN_ARRAY_1_FINAL		(146 + 256)
#define OFFSET_CLEAN_ARRAY_2_FINAL		(148 + 256)
#define OFFSET_CLEAN_ARRAY_3_FINAL		(150 + 256)
#define OFFSET_CLEAN_ARRAY_4_FINAL		(152 + 256)
#define OFFSET_CLEAN_ARRAY_5_FINAL		(154 + 256)

#define OFFSET_CLEAN_CATH_TIME			(172 + 256)
#define OFFSET_CLEAN_REBUILD_TIME		(173 + 256)

//// Create reference for pH die
//#define OFFSET_PH_4_SLOPE		OFFSET_TH_1_SLOPE
//#define OFFSET_PH_4_INT			OFFSET_TH_1_INT
//#define OFFSET_PH_5_SLOPE		OFFSET_TH_2_SLOPE
//#define OFFSET_PH_5_INT			OFFSET_TH_2_INT
//#define OFFSET_PH_6_SLOPE		OFFSET_NH4_1_SLOPE
//#define OFFSET_PH_6_INT			OFFSET_NH4_1_INT
//#define OFFSET_PH_7_SLOPE		OFFSET_NH4_2_SLOPE
//#define OFFSET_PH_7_INT			OFFSET_NH4_2_INT
//#define OFFSET_PH_8_SLOPE		OFFSET_NH4_3_SLOPE
//#define OFFSET_PH_8_INT			OFFSET_NH4_3_INT
//#define OFFSET_PH_9_SLOPE		OFFSET_CA_1_SLOPE
//#define OFFSET_PH_9_INT			OFFSET_CA_1_INT
//#define OFFSET_PH_10_SLOPE		OFFSET_CA_2_SLOPE
//#define OFFSET_PH_10_INT		OFFSET_CA_2_INT

// Memory offset of test info
#define OFFSET_TEST_NUMBER		0
#define OFFSET_TEST_USER_NAME	2
#define OFFSET_TEST_LOCATION	22
#define OFFSET_TEST_DATE		42
#define OFFSET_TEST_TIME		46
#define OFFSET_TEST_CAL			49
//#define OFFSET_TEST_GPS			50
#define OFFSET_TEST_LOG_K		58
#define OFFSET_TEST_ERROR		62
#define OFFSET_TEST_ZERO		66

#define OFFSET_ISE_4_T1_1		72
#define OFFSET_ISE_5_T1_1		76
#define OFFSET_ISE_6_T1_1		80
#define OFFSET_ISE_7_T1_1		84
#define OFFSET_ISE_8_T1_1		88
#define OFFSET_ISE_9_T1_1		92
#define OFFSET_ISE_10_T1_1		96

#define OFFSET_ISE_4_T1_2		100
#define OFFSET_ISE_5_T1_2		104
#define OFFSET_ISE_6_T1_2		108
#define OFFSET_ISE_7_T1_2		112
#define OFFSET_ISE_8_T1_2		116
#define OFFSET_ISE_9_T1_2		120
#define OFFSET_ISE_10_T1_2		124

#define OFFSET_TEST_NITRITE_BLANK_1_NA	112
#define OFFSET_TEST_NITRITE_BLANK_2_NA	116
#define OFFSET_TEST_NITRITE_NA		120
#define OFFSET_TEST_NITRITE_BACK_NA	124

// Memory offset of test data
#define OFFSET_TEST_CPU_TEMP		(0 + 128)
#define OFFSET_TEST_PH				(4 + 128)
#define OFFSET_TEST_TEMP			(8 + 128)
#define OFFSET_TEST_COND			(12 + 128)
#define OFFSET_TEST_ALKALINITY		(16 + 128)
#define OFFSET_TEST_CALCIUM			(20 + 128)
#define OFFSET_TEST_MAGNESIUM		(24 + 128)
#define OFFSET_TEST_MAG_HARDNESS	(28 + 128)
#define OFFSET_TEST_TOTAL_HARDNESS	(32 + 128)
#define OFFSET_TEST_CAL_HARDNESS	(36 + 128)
#define OFFSET_TEST_CO2				(40 + 128)
#define OFFSET_TEST_FREE_NH4		(44 + 128)
#define OFFSET_TEST_TOTAL_NH4		(48 + 128)
#define OFFSET_TEST_PH_1_T1_1		(52 + 128)
#define OFFSET_TEST_FREE_CL			(56 + 128)
#define OFFSET_TEST_TOTAL_CL		(60 + 128)
#define OFFSET_TEST_TOTAL_NH4_MONO	(64 + 128)
#define OFFSET_TEST_PH_1_T1_2		(68 + 128)
#define OFFSET_TEST_ORP				(72 + 128)
#define OFFSET_TEST_CL_NH4_RATIO	(76 + 128)
//#define OFFSET_TEST_LSI				(80 + 128)
//#define OFFSET_TEST_RSI				(84 + 128)
#define OFFSET_TEST_T_THERM			(80 + 128)
#define OFFSET_TEST_NITRITE			(84 + 128)
#define OFFSET_TEST_MONO			(88 + 128)
#define OFFSET_TEST_BFR				(92 + 128)
#define OFFSET_RAW_T_SAMP_T1_1		(96 + 128)
#define OFFSET_TEST_DEVICE_SERIAL	(100 + 128)
#define OFFSET_CHOSEN_SENSORS		(107 + 128)
#define OFFSET_RAW_T_SAMP_T1_2		(108 + 128)
#define OFFSET_TEST_COND_T1			(112 + 128)
#define OFFSET_STEPS_T1_1			(116 + 128)
#define OFFSET_STEPS_T1_2			(118 + 128)
#define OFFSET_TEST_DATA_ZERO		(120 + 128)

// Memory offset of raw test data
#define OFFSET_RAW_T_RINSE			(0 + 256)
#define OFFSET_RAW_ISE_1_RINSE		(4 + 256)
#define OFFSET_RAW_ISE_2_RINSE		(8 + 256)
#define OFFSET_RAW_ISE_3_RINSE		(12 + 256)
#define OFFSET_RAW_ISE_4_RINSE		(16 + 256)
#define OFFSET_RAW_ISE_5_RINSE		(20 + 256)
#define OFFSET_RAW_ISE_6_RINSE		(24 + 256)
#define OFFSET_RAW_ISE_7_RINSE		(28 + 256)
#define OFFSET_RAW_ISE_8_RINSE		(32 + 256)
#define OFFSET_RAW_ISE_9_RINSE		(36 + 256)
#define OFFSET_RAW_ISE_10_RINSE		(40 + 256)

#define OFFSET_RAW_ISE_1_SAMP		(44 + 256)
#define OFFSET_RAW_ISE_2_SAMP		(48 + 256)
#define OFFSET_RAW_ISE_3_SAMP		(52 + 256)
#define OFFSET_RAW_ISE_4_SAMP		(56 + 256)
#define OFFSET_RAW_ISE_5_SAMP		(60 + 256)
#define OFFSET_RAW_ISE_6_SAMP		(64 + 256)
#define OFFSET_RAW_ISE_7_SAMP		(68 + 256)
#define OFFSET_RAW_ISE_8_SAMP		(72 + 256)
#define OFFSET_RAW_ISE_9_SAMP		(76 + 256)
#define OFFSET_RAW_ISE_10_SAMP		(80 + 256)

#define OFFSET_TEST_PH_2_T1_1		(84 + 256)
#define OFFSET_TEST_PH_2_T1_2		(88 + 256)
#define OFFSET_TEST_PH_3_T1_1		(92 + 256)
#define OFFSET_TEST_PH_3_T1_2		(96 + 256)
#define OFFSET_RAW_CL_FCL			(100 + 256)
#define OFFSET_RAW_CL_TCL			(104 + 256)
#define OFFSET_RAW_NH4_1_T1			(108 + 256)
#define OFFSET_RAW_NH4_2_T1			(112 + 256)
#define OFFSET_RAW_NH4_3_T1			(116 + 256)
#define OFFSET_RAW_TEST_ZERO		(120 + 256)

#define OFFSET_RAW_COND_REG		(123 + 256)
#define OFFSET_RAW_COND			(124 + 256)

#define OFFSET_START_THERM		50
#define OFFSET_FINAL_THERM		54
#define OFFSET_TEST_CLEAN_COND_TCOR	(164 + 256)

#define OFFSET_B1_MIX_COND		(176 + 256)
#define OFFSET_B2_MIX_COND		(180 + 256)
#define OFFSET_B1_PRIME_COND	(184 + 256)
#define OFFSET_C2_PRIME_COND	(188 + 256)
#define OFFSET_B1_MIX_TEMP	(192 + 256)
#define OFFSET_B2_MIX_TEMP	(196 + 256)
#define OFFSET_B1_MIX_START_TEMP	(200 + 256)
#define OFFSET_B2_MIX_START_TEMP	(204 + 256)
#define OFFSET_B1_THERM_TEMP	(208 + 256)
#define OFFSET_B2_THERM_TEMP	(212 + 256)
#define OFFSET_B2_PRIME_COND	(216 + 256)

#endif

#ifdef MEMORY_V5
// Pages in memory
#define PAGE_DEVICE_INFO	0
#define PAGE_CARTRIDGE_INFO	1
#define PAGE_FACTORY_CAL	2
#define PAGE_SOLUTIONS		3
#define PAGE_CAL			4
#define PAGE_TEST			139

#define PAGES_FOR_TEST	3
#define PAGES_FOR_CAL	3

// Memory offset for device information page
//#define OFFSET_MEMORY_CONFIG	101
#define MEMORY_512K		1
#define MEMORY_1M		2
#define GMEMORY			MEMORY_512K

// Memory offset of Cartridge Information
#define OFFSET_CARTRIDGE_SN				0
#define OFFSET_SENSOR_SN				7
#define OFFSET_SENSOR_MAX_DAYS			14
#define OFFSET_COMPLETED_TESTS			20
#define OFFSET_COMPLETED_CALS			22
#define OFFSET_SENSOR_ASSEMBLY_DATE		24
#define OFFSET_SENSOR_HYDRATION_DATE	28
#define OFFSET_SENSOR_USAGE				32
#define OFFSET_SENSOR_MAX_TESTS			33
#define OFFSET_SOLUTIONS_BATCH			35
#define OFFSET_SENSOR_CONFIGURATION		39
#define OFFSET_MIN_CART_TEMP			40
#define OFFSET_MAX_CART_TEMP			44
#define OFFSET_MIN_TEMP_DATE			48
#define OFFSET_MIN_TEMP_TIME			52
#define OFFSET_MAX_TEMP_DATE			54
#define OFFSET_MAX_TEMP_TIME			58
#define OFFSET_CI_ZERO					60

// Memory offset of Factory Calibration data
#define OFFSET_TEMP_COEFFICIENT_A	0
#define OFFSET_TEMP_COEFFICIENT_B	4
#define OFFSET_TEMP_COEFFICIENT_C	8
#define OFFSET_COND_LOW_POINT_CAL	12
#define OFFSET_COND_READ_LOW_POINT	16
#define OFFSET_TCL_SLOPE			20
#define OFFSET_TCL_INT				24
#define OFFSET_TCL_SLOPE_HIGH		28
#define OFFSET_TCL_INT_HIGH			32
#define OFFSET_FCL_SLOPE			36
#define OFFSET_FCL_INT				40
#define OFFSET_FCL_SLOPE_HIGH		44
#define OFFSET_FCL_INT_HIGH			48
#define OFFSET_NITRITE_BLANK		52
#define OFFSET_FC_ZERO				56	// Placed in the middle because the following data is used during factory calibration but will not be used after and will not be displayed over BT

#define OFFSET_TCL_MID_POINT		88
#define OFFSET_FCL_MID_POINT		92
#define OFFSET_FCL_HIGH_READ		96
#define OFFSET_TCL_HIGH_READ		100
#define OFFSET_TEMP_LOW					104
#define OFFSET_TEMP_RESISTANCE_LOW		108
#define OFFSET_TEMP_MID					112
#define OFFSET_TEMP_RESISTANCE_MID		116
#define OFFSET_TEMP_HIGH				120
#define OFFSET_TEMP_RESISTANCE_HIGH		124

// Memory offset of solution data
#define OFFSET_T1_HCL_N		0
#define OFFSET_RINSE_PH		4
#define OFFSET_RINSE_NH4	8
#define OFFSET_RINSE_CA		12
#define OFFSET_RINSE_TH		16
#define OFFSET_RINSE_COND	20
#define OFFSET_CAL_1_PH		24
#define OFFSET_CAL_1_NH4	28
#define OFFSET_CAL_1_CA		32
#define OFFSET_CAL_1_TH		36
#define OFFSET_CAL_1_COND	40
#define OFFSET_NITRITE_SA	44
#define OFFSET_CAL_2_PH		48
#define OFFSET_CAL_2_NH4	52
#define OFFSET_CAL_2_CA		56
#define OFFSET_CAL_2_TH		60
#define OFFSET_CAL_2_COND	64
#define OFFSET_CLEAN_PH		68
#define OFFSET_CLEAN_NH4	72
#define OFFSET_SOL_ZERO		80

#define OFFSET_REPUMP_CLEAN		124
#define OFFSET_REPUMP_RINSE		125
#define OFFSET_REPUMP_CAL_1		126
#define OFFSET_REPUMP_CAL_2		127

// Memory offset of calibration info
#define OFFSET_CAL_NUMBER	0
#define OFFSET_CAL_USER		2
#define OFFSET_CAL_LOCATION	22
#define OFFSET_CAL_DATE		42
#define OFFSET_CAL_TIME		46
#define OFFSET_CAL_BATTERY	49
#define OFFSET_CAL_LOG_K	50
#define OFFSET_CAL_GPS		54
#define OFFSET_CAL_ERROR	62
#define OFFSET_CAL_ZERO		66

#define OFFSET_CR_ISE_1_POST		67
#define OFFSET_CR_ISE_2_POST		71
#define OFFSET_CR_ISE_3_POST		75
#define OFFSET_CR_ISE_4_POST		79
#define OFFSET_CR_ISE_5_POST		83
#define OFFSET_CR_ISE_6_POST		87
#define OFFSET_CR_ISE_7_POST		91
#define OFFSET_CR_ISE_8_POST		95
#define OFFSET_CR_ISE_9_POST		99
#define OFFSET_CR_ISE_10_POST		103
#define OFFSET_CR_T_POSTRINSE		107

#define OFFSET_PH_1_LAST_P_CAL			118
#define OFFSET_PH_2_LAST_P_CAL			119
#define OFFSET_PH_3_LAST_P_CAL			120
#define OFFSET_TH_1_LAST_P_CAL			121
#define OFFSET_TH_2_LAST_P_CAL			122
#define OFFSET_NH4_1_LAST_P_CAL			123
#define OFFSET_NH4_2_LAST_P_CAL			124
#define OFFSET_NH4_3_LAST_P_CAL			125
#define OFFSET_CA_1_LAST_P_CAL			126
#define OFFSET_CA_2_LAST_P_CAL			127

// Memory offset of calibration data
#define OFFSET_T_CAL			(0 + 128)
#define OFFSET_ISE_1_SLOPE		(4 + 128)
#define OFFSET_ISE_2_SLOPE		(8 + 128)
#define OFFSET_ISE_3_SLOPE		(12 + 128)
#define OFFSET_ISE_4_SLOPE		(16 + 128)
#define OFFSET_ISE_5_SLOPE		(20 + 128)
#define OFFSET_ISE_6_SLOPE		(24 + 128)
#define OFFSET_ISE_7_SLOPE		(28 + 128)
#define OFFSET_ISE_8_SLOPE		(32 + 128)
#define OFFSET_ISE_9_SLOPE		(36 + 128)
#define OFFSET_ISE_10_SLOPE		(40 + 128)
#define OFFSET_COND_R1_SLOPE	(44 + 128)
#define OFFSET_COND_R1_INT		(48 + 128)
#define OFFSET_COND_R2_SLOPE	(52 + 128)
#define OFFSET_COND_R2_INT		(56 + 128)
#define OFFSET_COND_R3_SLOPE	(60 + 128)
#define OFFSET_COND_R3_INT		(64 + 128)
#define OFFSET_CAL_DEVICE_SERIAL	(68 + 128)
#define OFFSET_CAL_CHOSEN_SENSORS	(75 + 128)
#define OFFSET_CAL_STATUS		(76 + 128)
#define OFFSET_CAL_DATA_ZERO	(80 + 128)

// Memory offset of calibration RAW data
#define OFFSET_ISE_1_INT		(0 + 256)
#define OFFSET_ISE_2_INT		(4 + 256)
#define OFFSET_ISE_3_INT		(8 + 256)
#define OFFSET_ISE_4_INT		(12 + 256)
#define OFFSET_ISE_5_INT		(16 + 256)
#define OFFSET_ISE_6_INT		(20 + 256)
#define OFFSET_ISE_7_INT		(24 + 256)
#define OFFSET_ISE_8_INT		(28 + 256)
#define OFFSET_ISE_9_INT		(32 + 256)
#define OFFSET_ISE_10_INT		(36 + 256)
#define OFFSET_CR_ISE_1_RINSE	(40 + 256)
#define OFFSET_CR_ISE_2_RINSE	(44 + 256)
#define OFFSET_CR_ISE_3_RINSE	(48 + 256)
#define OFFSET_CR_ISE_4_RINSE	(52 + 256)
#define OFFSET_CR_ISE_5_RINSE	(56 + 256)
#define OFFSET_CR_ISE_6_RINSE	(60 + 256)
#define OFFSET_CR_ISE_7_RINSE	(64 + 256)
#define OFFSET_CR_ISE_8_RINSE	(68 + 256)
#define OFFSET_CR_ISE_9_RINSE	(72 + 256)
#define OFFSET_CR_ISE_10_RINSE	(76 + 256)
#define OFFSET_CR_ZERO			(80 + 256)

//// Create reference for pH die
//#define OFFSET_PH_4_SLOPE		OFFSET_TH_1_SLOPE
//#define OFFSET_PH_4_INT			OFFSET_TH_1_INT
//#define OFFSET_PH_5_SLOPE		OFFSET_TH_2_SLOPE
//#define OFFSET_PH_5_INT			OFFSET_TH_2_INT
//#define OFFSET_PH_6_SLOPE		OFFSET_NH4_1_SLOPE
//#define OFFSET_PH_6_INT			OFFSET_NH4_1_INT
//#define OFFSET_PH_7_SLOPE		OFFSET_NH4_2_SLOPE
//#define OFFSET_PH_7_INT			OFFSET_NH4_2_INT
//#define OFFSET_PH_8_SLOPE		OFFSET_NH4_3_SLOPE
//#define OFFSET_PH_8_INT			OFFSET_NH4_3_INT
//#define OFFSET_PH_9_SLOPE		OFFSET_CA_1_SLOPE
//#define OFFSET_PH_9_INT			OFFSET_CA_1_INT
//#define OFFSET_PH_10_SLOPE		OFFSET_CA_2_SLOPE
//#define OFFSET_PH_10_INT		OFFSET_CA_2_INT

// Memory offset of test info
#define OFFSET_TEST_NUMBER		0
#define OFFSET_TEST_USER_NAME	2
#define OFFSET_TEST_LOCATION	22
#define OFFSET_TEST_DATE		42
#define OFFSET_TEST_TIME		46
#define OFFSET_TEST_CAL			49
#define OFFSET_TEST_GPS			50
#define OFFSET_TEST_LOG_K		58
#define OFFSET_TEST_ERROR		62
#define OFFSET_TEST_ZERO		66

#define OFFSET_ISE_4_T1_1		72
#define OFFSET_ISE_5_T1_1		76
#define OFFSET_ISE_6_T1_1		80
#define OFFSET_ISE_7_T1_1		84
#define OFFSET_ISE_8_T1_1		88
#define OFFSET_ISE_9_T1_1		92
#define OFFSET_ISE_10_T1_1		96

#define OFFSET_ISE_4_T1_2		100
#define OFFSET_ISE_5_T1_2		104
#define OFFSET_ISE_6_T1_2		108
#define OFFSET_ISE_7_T1_2		112
#define OFFSET_ISE_8_T1_2		116
#define OFFSET_ISE_9_T1_2		120
#define OFFSET_ISE_10_T1_2		124

#define OFFSET_TEST_NITRITE_BLANK_1_NA	112
#define OFFSET_TEST_NITRITE_BLANK_2_NA	116
#define OFFSET_TEST_NITRITE_NA		120
#define OFFSET_TEST_NITRITE_BACK_NA	124

// Memory offset of test data
#define OFFSET_TEST_CPU_TEMP		(0 + 128)
#define OFFSET_TEST_PH				(4 + 128)
#define OFFSET_TEST_TEMP			(8 + 128)
#define OFFSET_TEST_COND			(12 + 128)
#define OFFSET_TEST_ALKALINITY		(16 + 128)
#define OFFSET_TEST_CALCIUM			(20 + 128)
#define OFFSET_TEST_MAGNESIUM		(24 + 128)
#define OFFSET_TEST_MAG_HARDNESS	(28 + 128)
#define OFFSET_TEST_TOTAL_HARDNESS	(32 + 128)
#define OFFSET_TEST_CAL_HARDNESS	(36 + 128)
#define OFFSET_TEST_CO2				(40 + 128)
#define OFFSET_TEST_FREE_NH4		(44 + 128)
#define OFFSET_TEST_TOTAL_NH4		(48 + 128)
#define OFFSET_TEST_PH_1_T1_1		(52 + 128)
#define OFFSET_TEST_FREE_CL			(56 + 128)
#define OFFSET_TEST_TOTAL_CL		(60 + 128)
#define OFFSET_TEST_TOTAL_NH4_MONO	(64 + 128)
#define OFFSET_TEST_PH_1_T1_2		(68 + 128)
#define OFFSET_TEST_ORP				(72 + 128)
#define OFFSET_TEST_CL_NH4_RATIO	(76 + 128)
//#define OFFSET_TEST_LSI				(80 + 128)
//#define OFFSET_TEST_RSI				(84 + 128)
#define OFFSET_TEST_T_THERM			(80 + 128)
#define OFFSET_TEST_NITRITE			(84 + 128)
#define OFFSET_TEST_MONO			(88 + 128)
#define OFFSET_TEST_BFR				(92 + 128)
#define OFFSET_RAW_T_SAMP_T1_1		(96 + 128)
#define OFFSET_TEST_DEVICE_SERIAL	(100 + 128)
#define OFFSET_CHOSEN_SENSORS		(107 + 128)
#define OFFSET_RAW_T_SAMP_T1_2		(108 + 128)
#define OFFSET_TEST_COND_T1			(112 + 128)
#define OFFSET_STEPS_T1_1			(116 + 128)
#define OFFSET_STEPS_T1_2			(118 + 128)
#define OFFSET_TEST_DATA_ZERO		(120 + 128)

// Memory offset of raw test data
#define OFFSET_RAW_T_RINSE			(0 + 256)
#define OFFSET_RAW_ISE_1_RINSE		(4 + 256)
#define OFFSET_RAW_ISE_2_RINSE		(8 + 256)
#define OFFSET_RAW_ISE_3_RINSE		(12 + 256)
#define OFFSET_RAW_ISE_4_RINSE		(16 + 256)
#define OFFSET_RAW_ISE_5_RINSE		(20 + 256)
#define OFFSET_RAW_ISE_6_RINSE		(24 + 256)
#define OFFSET_RAW_ISE_7_RINSE		(28 + 256)
#define OFFSET_RAW_ISE_8_RINSE		(32 + 256)
#define OFFSET_RAW_ISE_9_RINSE		(36 + 256)
#define OFFSET_RAW_ISE_10_RINSE		(40 + 256)

#define OFFSET_RAW_ISE_1_SAMP		(44 + 256)
#define OFFSET_RAW_ISE_2_SAMP		(48 + 256)
#define OFFSET_RAW_ISE_3_SAMP		(52 + 256)
#define OFFSET_RAW_ISE_4_SAMP		(56 + 256)
#define OFFSET_RAW_ISE_5_SAMP		(60 + 256)
#define OFFSET_RAW_ISE_6_SAMP		(64 + 256)
#define OFFSET_RAW_ISE_7_SAMP		(68 + 256)
#define OFFSET_RAW_ISE_8_SAMP		(72 + 256)
#define OFFSET_RAW_ISE_9_SAMP		(76 + 256)
#define OFFSET_RAW_ISE_10_SAMP		(80 + 256)

#define OFFSET_TEST_PH_2_T1_1		(84 + 256)
#define OFFSET_TEST_PH_2_T1_2		(88 + 256)
#define OFFSET_TEST_PH_3_T1_1		(92 + 256)
#define OFFSET_TEST_PH_3_T1_2		(96 + 256)
#define OFFSET_RAW_CL_FCL			(100 + 256)
#define OFFSET_RAW_CL_TCL			(104 + 256)
#define OFFSET_RAW_NH4_1_T1			(108 + 256)
#define OFFSET_RAW_NH4_2_T1			(112 + 256)
#define OFFSET_RAW_NH4_3_T1			(116 + 256)
#define OFFSET_RAW_TEST_ZERO		(120 + 256)

#define OFFSET_RAW_COND		(124 + 256)

#endif

#ifdef MEMORY_V4

#define MEMORY_512K		1
#define MEMORY_1M		2
#define GMEMORY			MEMORY_512K

// Pages in memory
#define PAGE_DEVICE_INFO	0
#define PAGE_CARTRIDGE_INFO	1
#define PAGE_FACTORY_CAL	2
#define PAGE_SOLUTIONS		3
#define PAGE_CAL			4
#define PAGE_TEST			139

#define PAGES_FOR_TEST	3
#define PAGES_FOR_CAL	3

// Memory offset of Cartridge Information
#define OFFSET_CARTRIDGE_SN				0
#define OFFSET_SENSOR_SN				7
#define OFFSET_COMPLETED_TESTS			14
#define OFFSET_COMPLETED_CALS			16
#define OFFSET_SENSOR_ASSEMBLY_DATE		18
#define OFFSET_SENSOR_HYDRATION_DATE	22
#define OFFSET_SENSOR_USAGE				26
#define OFFSET_SENSOR_MAX_TESTS			27
#define OFFSET_SOLUTIONS_BATCH			29
#define OFFSET_SENSOR_CONFIGURATION		33
#define OFFSET_MIN_CART_TEMP			34
#define OFFSET_MAX_CART_TEMP			38
#define OFFSET_MIN_TEMP_DATE			42
#define OFFSET_MIN_TEMP_TIME			46
#define OFFSET_MAX_TEMP_DATE			48
#define OFFSET_MAX_TEMP_TIME			52
#define OFFSET_CI_ZERO					54

// Memory offset of Factory Calibration data
#define OFFSET_TEMP_COEFFICIENT_A	0
#define OFFSET_TEMP_COEFFICIENT_B	4
#define OFFSET_TEMP_COEFFICIENT_C	8
#define OFFSET_COND_LOW_POINT_CAL	12
#define OFFSET_COND_READ_LOW_POINT	16
#define OFFSET_TCL_SLOPE			20
#define OFFSET_TCL_INT				24
#define OFFSET_TCL_SLOPE_HIGH		28
#define OFFSET_TCL_INT_HIGH			32
#define OFFSET_FCL_SLOPE			36
#define OFFSET_FCL_INT				40
#define OFFSET_FCL_SLOPE_HIGH		44
#define OFFSET_FCL_INT_HIGH			48
#define OFFSET_NITRITE_BLANK		52
#define OFFSET_FC_ZERO				56	// Placed in the middle because the following data is used during factory calibration but will not be used after and will not be displayed over BT

#define OFFSET_TCL_MID_POINT		88
#define OFFSET_FCL_MID_POINT		92
#define OFFSET_FCL_HIGH_READ		96
#define OFFSET_TCL_HIGH_READ		100
#define OFFSET_TEMP_LOW					104
#define OFFSET_TEMP_RESISTANCE_LOW		108
#define OFFSET_TEMP_MID					112
#define OFFSET_TEMP_RESISTANCE_MID		116
#define OFFSET_TEMP_HIGH				120
#define OFFSET_TEMP_RESISTANCE_HIGH		124

// Memory offset of solution data
#define OFFSET_T1_HCL_N		0
#define OFFSET_RINSE_PH		4
#define OFFSET_RINSE_NH4	8
#define OFFSET_RINSE_CA		12
#define OFFSET_RINSE_TH		16
#define OFFSET_RINSE_COND	20
#define OFFSET_CAL_1_PH		24
#define OFFSET_CAL_1_NH4	28
#define OFFSET_CAL_1_CA		32
#define OFFSET_CAL_1_TH		36
#define OFFSET_CAL_1_COND	40
#define OFFSET_NITRITE_SA	44
#define OFFSET_CAL_2_PH		48
#define OFFSET_CAL_2_NH4	52
#define OFFSET_CAL_2_CA		56
#define OFFSET_CAL_2_TH		60
#define OFFSET_CAL_2_COND	64
#define OFFSET_CLEAN_PH		68
#define OFFSET_CLEAN_NH4	72
#define OFFSET_SOL_ZERO		80

#define OFFSET_REPUMP_CLEAN	124
#define OFFSET_REPUMP_RINSE	125
#define OFFSET_REPUMP_CAL_1	126
#define OFFSET_REPUMP_CAL_2	127

// Memory offset of calibration info
#define OFFSET_CAL_NUMBER	0
#define OFFSET_CAL_USER		2
#define OFFSET_CAL_LOCATION	22
#define OFFSET_CAL_DATE		42
#define OFFSET_CAL_TIME		46
#define OFFSET_CAL_BATTERY	49
#define OFFSET_CAL_LOG_K	50
#define OFFSET_CAL_GPS		54
#define OFFSET_CAL_ERROR	62
#define OFFSET_CAL_ZERO		66

#define OFFSET_CR_PH_1_POSTRINSE		67
#define OFFSET_CR_PH_2_POSTRINSE		71
#define OFFSET_CR_CA_1_POSTRINSE		75
#define OFFSET_CR_CA_2_POSTRINSE		79
#define OFFSET_CR_TH_1_POSTRINSE		83
#define OFFSET_CR_TH_2_POSTRINSE		87
#define OFFSET_CR_NH4_1_POSTRINSE		91
#define OFFSET_CR_NH4_2_POSTRINSE		95
#define OFFSET_CR_NH4_3_POSTRINSE		99
#define OFFSET_CR_T_POSTRINSE			103
#define OFFSET_CR_PH_3_POSTRINSE		107

#define OFFSET_PH_1_LAST_P_CAL			118
#define OFFSET_PH_2_LAST_P_CAL			119
#define OFFSET_PH_3_LAST_P_CAL			120
#define OFFSET_TH_1_LAST_P_CAL			121
#define OFFSET_TH_2_LAST_P_CAL			122
#define OFFSET_NH4_1_LAST_P_CAL			123
#define OFFSET_NH4_2_LAST_P_CAL			124
#define OFFSET_NH4_3_LAST_P_CAL			125
#define OFFSET_CA_1_LAST_P_CAL			126
#define OFFSET_CA_2_LAST_P_CAL			127

// Memory offset of calibration data
#define OFFSET_T_CAL			(0 + 128)
#define OFFSET_NH4_2_SLOPE		(4 + 128)
#define OFFSET_NH4_2_INT		(8 + 128)
#define OFFSET_PH_1_SLOPE		(12 + 128)
#define OFFSET_PH_1_INT			(16 + 128)
#define OFFSET_NH4_1_SLOPE		(20 + 128)
#define OFFSET_NH4_1_INT		(24 + 128)
#define OFFSET_CA_1_SLOPE		(28 + 128)
#define OFFSET_CA_1_INT			(32 + 128)
#define OFFSET_TH_1_SLOPE		(36 + 128)
#define OFFSET_TH_1_INT			(40 + 128)
#define OFFSET_COND_R1_SLOPE	(44 + 128)
#define OFFSET_COND_R1_INT		(48 + 128)
#define OFFSET_COND_R2_SLOPE	(52 + 128)
#define OFFSET_COND_R2_INT		(56 + 128)
#define OFFSET_COND_R3_SLOPE	(60 + 128)
#define OFFSET_COND_R3_INT		(64 + 128)
#define OFFSET_CAL_DEVICE_SERIAL	(68 + 128)
#define OFFSET_CAL_CHOSEN_SENSORS	(75 + 128)
#define OFFSET_CR_NH4_3_RINSE		(76 + 128)
#define OFFSET_CAL_DATA_ZERO	(80 + 128)

// Memory offset of calibration RAW data
#define OFFSET_CAL_STATUS		(0 + 256)
#define OFFSET_NH4_3_SLOPE		(4 + 256)
#define OFFSET_NH4_3_INT		(8 + 256)
#define OFFSET_PH_2_SLOPE		(12 + 256)
#define OFFSET_PH_2_INT			(16 + 256)
#define OFFSET_CA_2_SLOPE		(20 + 256)
#define OFFSET_CA_2_INT			(24 + 256)
#define OFFSET_TH_2_SLOPE		(28 + 256)
#define OFFSET_TH_2_INT			(32 + 256)
#define OFFSET_PH_3_SLOPE		(36 + 256)
#define OFFSET_PH_3_INT			(40 + 256)
#define OFFSET_CR_PH_1_RINSE	(44 + 256)
#define OFFSET_CR_PH_2_RINSE	(48 + 256)
#define OFFSET_CR_PH_3_RINSE	(52 + 256)
#define OFFSET_CR_CA_1_RINSE	(56 + 256)
#define OFFSET_CR_CA_2_RINSE	(60 + 256)
#define OFFSET_CR_TH_1_RINSE	(64 + 256)
#define OFFSET_CR_TH_2_RINSE	(68 + 256)
#define OFFSET_CR_NH4_1_RINSE	(72 + 256)
#define OFFSET_CR_NH4_2_RINSE	(76 + 256)
#define OFFSET_CR_ZERO			(80 + 256)

// Create reference for pH die
#define OFFSET_PH_4_SLOPE		OFFSET_TH_1_SLOPE
#define OFFSET_PH_4_INT			OFFSET_TH_1_INT
#define OFFSET_PH_5_SLOPE		OFFSET_TH_2_SLOPE
#define OFFSET_PH_5_INT			OFFSET_TH_2_INT
#define OFFSET_PH_6_SLOPE		OFFSET_NH4_1_SLOPE
#define OFFSET_PH_6_INT			OFFSET_NH4_1_INT
#define OFFSET_PH_7_SLOPE		OFFSET_NH4_2_SLOPE
#define OFFSET_PH_7_INT			OFFSET_NH4_2_INT
#define OFFSET_PH_8_SLOPE		OFFSET_NH4_3_SLOPE
#define OFFSET_PH_8_INT			OFFSET_NH4_3_INT
#define OFFSET_PH_9_SLOPE		OFFSET_CA_1_SLOPE
#define OFFSET_PH_9_INT			OFFSET_CA_1_INT
#define OFFSET_PH_10_SLOPE		OFFSET_CA_2_SLOPE
#define OFFSET_PH_10_INT		OFFSET_CA_2_INT

// Memory offset of test info
#define OFFSET_TEST_NUMBER		0
#define OFFSET_TEST_USER_NAME	2
#define OFFSET_TEST_LOCATION	22
#define OFFSET_TEST_DATE		42
#define OFFSET_TEST_TIME		46
#define OFFSET_TEST_CAL			49
#define OFFSET_TEST_GPS			50
#define OFFSET_TEST_LOG_K		58
#define OFFSET_TEST_ERROR		62
#define OFFSET_TEST_ZERO		66

#define OFFSET_TEST_NITRITE_NA		120
#define OFFSET_TEST_NITRITE_BACK_NA	124

// Memory offset of test data
#define OFFSET_TEST_CPU_TEMP		(0 + 128)
#define OFFSET_TEST_PH				(4 + 128)
#define OFFSET_TEST_TEMP			(8 + 128)
#define OFFSET_TEST_COND			(12 + 128)
#define OFFSET_TEST_ALKALINITY		(16 + 128)
#define OFFSET_TEST_CALCIUM			(20 + 128)
#define OFFSET_TEST_MAGNESIUM		(24 + 128)
#define OFFSET_TEST_MAG_HARDNESS	(28 + 128)
#define OFFSET_TEST_TOTAL_HARDNESS	(32 + 128)
#define OFFSET_TEST_CAL_HARDNESS	(36 + 128)
#define OFFSET_TEST_CO2				(40 + 128)
#define OFFSET_TEST_FREE_NH4		(44 + 128)
#define OFFSET_TEST_TOTAL_NH4		(48 + 128)
#define OFFSET_TEST_PH_1_T1_1		(52 + 128)
#define OFFSET_TEST_FREE_CL			(56 + 128)
#define OFFSET_TEST_TOTAL_CL		(60 + 128)
#define OFFSET_TEST_TOTAL_NH4_MONO	(64 + 128)
#define OFFSET_TEST_PH_1_T1_2		(68 + 128)
#define OFFSET_TEST_ORP				(72 + 128)
#define OFFSET_TEST_CL_NH4_RATIO	(76 + 128)
//#define OFFSET_TEST_LSI				(80 + 128)
//#define OFFSET_TEST_RSI				(84 + 128)
#define OFFSET_TEST_T_THERM			(80 + 128)
#define OFFSET_TEST_NITRITE			(84 + 128)
#define OFFSET_TEST_MONO			(88 + 128)
#define OFFSET_TEST_BFR				(92 + 128)
#define OFFSET_RAW_T_SAMP_T1_1		(96 + 128)
#define OFFSET_TEST_DEVICE_SERIAL	(100 + 128)
#define OFFSET_CHOSEN_SENSORS		(107 + 128)
#define OFFSET_RAW_T_SAMP_T1_2		(108 + 128)
#define OFFSET_TEST_COND_T1			(112 + 128)
#define OFFSET_STEPS_T1_1			(116 + 128)
#define OFFSET_STEPS_T1_2			(118 + 128)
#define OFFSET_TEST_DATA_ZERO		(120 + 128)

// Memory offset of raw test data
#define OFFSET_RAW_PH_3_RINSE		(0 + 256)
#define OFFSET_RAW_T_RINSE			(4 + 256)
#define OFFSET_RAW_PH_1_RINSE		(8 + 256)
#define OFFSET_RAW_PH_2_RINSE		(12 + 256)
#define OFFSET_RAW_NH4_2_RINSE		(16 + 256)
#define OFFSET_RAW_NH4_3_RINSE		(20 + 256)
#define OFFSET_RAW_CA_1_RINSE		(24 + 256)
#define OFFSET_RAW_CA_2_RINSE		(28 + 256)
#define OFFSET_RAW_TH_1_RINSE		(32 + 256)
#define OFFSET_RAW_TH_2_RINSE		(36 + 256)
#define OFFSET_RAW_NH4_1_RINSE		(40 + 256)
#define OFFSET_TEST_PH_2_T1_1		(44 + 256)
#define OFFSET_TEST_PH_2_T1_2		(48 + 256)
#define OFFSET_TEST_PH_3_T1_1		(52 + 256)
#define OFFSET_TEST_PH_3_T1_2		(56 + 256)
#define OFFSET_RAW_NH4_3_SAMP		(60 + 256)
#define OFFSET_RAW_PH_3_SAMP		(64 + 256)
#define OFFSET_RAW_PH_1_SAMP		(68 + 256)
#define OFFSET_RAW_PH_2_SAMP		(72 + 256)
#define OFFSET_RAW_CA_1_SAMP		(76 + 256)
#define OFFSET_RAW_CA_2_SAMP		(80 + 256)
#define OFFSET_RAW_TH_1_SAMP		(84 + 256)
#define OFFSET_RAW_TH_2_SAMP		(88 + 256)
#define OFFSET_RAW_NH4_1_SAMP		(92 + 256)
#define OFFSET_RAW_NH4_2_SAMP		(96 + 256)
#define OFFSET_RAW_CL_FCL			(100 + 256)
#define OFFSET_RAW_CL_TCL			(104 + 256)
#define OFFSET_RAW_NH4_1_T1			(108 + 256)
#define OFFSET_RAW_NH4_2_T1			(112 + 256)
#define OFFSET_RAW_NH4_3_T1			(116 + 256)
#define OFFSET_RAW_TEST_ZERO		(120 + 256)

#define OFFSET_RAW_COND		(124 + 256)

#endif

#ifdef MEMORY_V3
// Pages in memory
#define PAGE_DEVICE_INFO	0
#define PAGE_CARTRIDGE_INFO	1
#define PAGE_FACTORY_CAL	2
#define PAGE_SOLUTIONS		3
#define PAGE_CAL_INFO		4
#define PAGE_CAL_DATA		5
#define PAGE_CAL_RAW		6
#define PAGE_TEST_INFO		139
#define PAGE_TEST_DATA		140
#define PAGE_TEST_RAW		141

#define PAGES_FOR_TEST	3
#define PAGES_FOR_CAL	3

// Memory offset of Cartridge Information
#define OFFSET_CARTRIDGE_SN				0
#define OFFSET_SENSOR_SN				4
#define OFFSET_SENSOR_ASSEMBLY_DATE		8
#define OFFSET_SENSOR_HYDRATION_DATE	12
#define OFFSET_SENSOR_USAGE				16
#define OFFSET_SENSOR_MAX_TESTS			17
#define OFFSET_SOLUTIONS_BATCH			19
#define OFFSET_SENSOR_CONFIGURATION		23
#define OFFSET_CI_ZERO					24

// Memory offset of Factory Calibration data
#define OFFSET_TEMP_COEFFICIENT_A	0
#define OFFSET_TEMP_COEFFICIENT_B	4
#define OFFSET_TEMP_COEFFICIENT_C	8
#define OFFSET_COND_LOW_POINT_CAL	12
#define OFFSET_COND_READ_LOW_POINT	16
#define OFFSET_TCL_SLOPE			20
#define OFFSET_TCL_INT				24
#define OFFSET_FCL_INT				28
#define OFFSET_FC_ZERO				32	// Placed in the middle because the following data is used during factory calibration but will not be used after and will not be displayed over BT
#define OFFSET_TEMP_LOW					33
#define OFFSET_TEMP_RESISTANCE_LOW		37
#define OFFSET_TEMP_MID					41
#define OFFSET_TEMP_RESISTANCE_MID		45
#define OFFSET_TEMP_HIGH				49
#define OFFSET_TEMP_RESISTANCE_HIGH		53
#define OFFSET_FCL_SLOPE			57
#define OFFSET_TCL_SLOPE_HIGH		61
#define OFFSET_TCL_INT_HIGH			65
#define OFFSET_FCL_SLOPE_HIGH		69
#define OFFSET_FCL_INT_HIGH			73
#define OFFSET_TCL_MID_POINT		77
#define OFFSET_FCL_MID_POINT		81
#define OFFSET_FCL_HIGH_READ		85
#define OFFSET_TCL_HIGH_READ		89


// Memory offset of solution data
//#define OFFSET_RINSE_CO2	0
#define OFFSET_T1_HCL_N		0
#define OFFSET_RINSE_PH		4
#define OFFSET_RINSE_NH4	8
#define OFFSET_RINSE_CA		12
#define OFFSET_RINSE_TH		16
#define OFFSET_RINSE_COND	20
#define OFFSET_CAL_1_PH		24
#define OFFSET_CAL_1_NH4	28
#define OFFSET_CAL_1_CA		32
#define OFFSET_CAL_1_TH		36
#define OFFSET_CAL_1_COND	40
#define OFFSET_CAL_2_CO2	44
#define OFFSET_CAL_2_PH		48
#define OFFSET_CAL_2_NH4	52
#define OFFSET_CAL_2_CA		56
#define OFFSET_CAL_2_TH		60
#define OFFSET_CAL_2_COND	64
#define OFFSET_CAL_3_CO2	68
#define OFFSET_CAL_3_PH		72
#define OFFSET_CAL_3_TH		76
#define OFFSET_CAL_3_COND	80
#define OFFSET_SOL_ZERO		84

// Memory offset of calibration info
#define OFFSET_CAL_NUMBER	0
#define OFFSET_VALID		1
#define OFFSET_CAL_DATE		2
#define OFFSET_CAL_TIME		6
#define OFFSET_CAL_USER		9
#define OFFSET_CAL_GPS		29
#define OFFSET_CAL_LOCATION	37
#define OFFSET_CAL_CPU_TEMP	57
#define OFFSET_CAL_BATTERY	61
#define OFFSET_CAL_ERROR	62
#define OFFSET_CAL_ZERO		66

// Memory offset of calibration data
#define OFFSET_T_CAL			0
#define OFFSET_NH4_2_SLOPE		4	// OFFSET_CO2_1_SLOPE
#define OFFSET_COND_RANGE_1_HIGH_RAW	8
#define OFFSET_PH_1_SLOPE		12
#define OFFSET_COND_RANGE_2_LOW_RAW	16
#define OFFSET_NH4_SLOPE		20
#define OFFSET_COND_RANGE_2_HIGH_RAW	24
#define OFFSET_CA_1_SLOPE		28
#define OFFSET_COND_RANGE_3_LOW_RAW	32
#define OFFSET_TH_1_SLOPE		36
#define OFFSET_COND_RANGE_3_HIGH_RAW	40
#define OFFSET_COND_R1_SLOPE	44
#define OFFSET_COND_R1_INT		48
#define OFFSET_COND_R2_SLOPE	52
#define OFFSET_COND_R2_INT		56
#define OFFSET_COND_R3_SLOPE	60
#define OFFSET_COND_R3_INT		64
#define OFFSET_CAL_DEVICE_SERIAL	68
//#define OFFSET_FCL_INT			68
//#define OFFSET_TCL_INT			72
#define OFFSET_CAL_STATUS		76
#define OFFSET_NH4_3_SLOPE		80	// OFFSET_CO2_2_SLOPE
//#define CO2_2_INT				84
#define OFFSET_PH_2_SLOPE		88
#define OFFSET_PH_2_INT			92
#define OFFSET_CA_2_SLOPE		96
#define OFFSET_CA_2_INT			100
#define OFFSET_PH_3_SLOPE		100
#define OFFSET_TH_2_SLOPE		104
#define OFFSET_TH_2_INT			108
#define OFFSET_PH_CHOSEN		112
#define OFFSET_CA_CHOSEN		113
#define OFFSET_TH_CHOSEN		114
#define OFFSET_NH4_CHOSEN		115	// OFFSET_CO2_CHOSEN
#define OFFSET_NH4_EEPROM_RINSE	116
#define OFFSET_CAL_DATA_ZERO	120

// Memory offset of calibration RAW data
#define OFFSET_CR_PH_1_RINSE	0
#define OFFSET_CR_PH_2_RINSE	4
#define OFFSET_CR_CA_1_RINSE	8
#define OFFSET_CR_CA_2_RINSE	12
#define OFFSET_CR_TH_1_RINSE	16
#define OFFSET_CR_TH_2_RINSE	20
#define OFFSET_CR_NH4_1_RINSE	24
#define OFFSET_CR_NH4_2_RINSE	28
#define OFFSET_CR_NH4_3_RINSE	32
#define OFFSET_CR_T_RINSE		36
#define OFFSET_CR_PH_1_CAL_1	40
#define OFFSET_CR_PH_2_CAL_1	44
#define OFFSET_CR_CA_1_CAL_1	48
#define OFFSET_CR_CA_2_CAL_1	52
#define OFFSET_CR_TH_1_CAL_1	56
#define OFFSET_CR_TH_2_CAL_1	60
#define OFFSET_CR_NH4_1_CAL_1	64
#define OFFSET_CR_NH4_2_CAL_1	68
#define OFFSET_CR_NH4_3_CAL_1	72
#define OFFSET_CR_TEMP_CAL_1	76
#define OFFSET_CR_PH_1_CAL_2	80
#define OFFSET_CR_PH_2_CAL_2	84
#define OFFSET_CR_CA_1_CAL_2	88
#define OFFSET_CR_CA_2_CAL_2	92
#define OFFSET_CR_TH_1_CAL_2	96
#define OFFSET_CR_TH_2_CAL_2	100
#define OFFSET_CR_NH4_1_CAL_2	104
#define OFFSET_CR_NH4_2_CAL_2	108
#define OFFSET_CR_NH4_3_CAL_2	112
#define OFFSET_CR_TEMP_CAL_2	116
#define OFFSET_CR_ZERO			120

#define OFFSET_CR_PH_3_RINSE	72
#define OFFSET_CR_PH_3_CAL_1	92
#define OFFSET_CR_PH_3_CAL_2	84

#define OFFSET_CR_PH_1_B2		111
#define OFFSET_CR_PH_2_B2		115
#define OFFSET_CR_PH_3_B2		119

#define OFFSET_CR_PH_1_POSTRINSE		67
#define OFFSET_CR_PH_2_POSTRINSE		71
#define OFFSET_CR_CA_1_POSTRINSE		75
#define OFFSET_CR_CA_2_POSTRINSE		79
#define OFFSET_CR_TH_1_POSTRINSE		83
#define OFFSET_CR_TH_2_POSTRINSE		87
#define OFFSET_CR_NH4_1_POSTRINSE		91
#define OFFSET_CR_NH4_2_POSTRINSE		95
#define OFFSET_CR_NH4_3_POSTRINSE		99
#define OFFSET_CR_T_POSTRINSE			103
#define OFFSET_CR_PH_3_POSTRINSE		107

// Memory offset of test info
#define OFFSET_TEST_NUMBER		0
//#define OFFSET_MOST_RECENT		1
#define OFFSET_TEST_DATE		2
#define OFFSET_TEST_TIME		6
#define OFFSET_TEST_USER_NAME	9
#define OFFSET_TEST_GPS			29
#define OFFSET_TEST_LOCATION	37
#define OFFSET_TEST_CAL			57
#define OFFSET_TEST_ERROR		58
#define OFFSET_TEST_ZERO		62

#define OFFSET_RAW_PH_1_POSTRINSE		63
#define OFFSET_RAW_PH_2_POSTRINSE		67
#define OFFSET_RAW_CA_1_POSTRINSE		71
#define OFFSET_RAW_CA_2_POSTRINSE		75
#define OFFSET_RAW_TH_1_POSTRINSE		79
#define OFFSET_RAW_TH_2_POSTRINSE		83
#define OFFSET_RAW_NH4_1_POSTRINSE		87
#define OFFSET_RAW_NH4_2_POSTRINSE		91
#define OFFSET_RAW_NH4_3_POSTRINSE		95
#define OFFSET_RAW_T_POSTRINSE			99
#define OFFSET_RAW_T_SAMP_T1_1			103
#define OFFSET_RAW_T_SAMP_T1_2			107

#define OFFSET_TEST_NH4_1_B1		111
#define OFFSET_TEST_NH4_2_B1		115
#define OFFSET_TEST_NH4_3_B1		119

#define OFFSET_TEST_PH_NOT_USED_2	123

// Memory offset of test data
#define OFFSET_TEST_CPU_TEMP		0
#define OFFSET_TEST_PH				4
#define OFFSET_TEST_TEMP			8
#define OFFSET_TEST_COND			12
#define OFFSET_TEST_ALKALINITY		16
#define OFFSET_TEST_CALCIUM			20
#define OFFSET_TEST_MAGNESIUM		24
#define OFFSET_TEST_MAG_HARDNESS	28
#define OFFSET_TEST_TOTAL_HARDNESS	32
#define OFFSET_TEST_CAL_HARDNESS	36
#define OFFSET_TEST_CO2				40
#define OFFSET_TEST_FREE_NH4		44
#define OFFSET_TEST_TOTAL_NH4		48
//	#define OFFSET_TEST_TOTAL_NH3_NH4	52
#define OFFSET_TEST_PH_T1_1			52
#define OFFSET_TEST_FREE_CL			56
#define OFFSET_TEST_TOTAL_CL		60
#define OFFSET_TEST_TOTAL_NH4_MONO	64
#define OFFSET_TEST_PH_T1_2			68
//	#define OFFSET_TEST_TDS				68
#define OFFSET_TEST_ORP				72
#define OFFSET_TEST_CL_NH4_RATIO	76
#define OFFSET_TEST_LSI				80
#define OFFSET_TEST_RSI				84
#define OFFSET_TEST_MONO			88
#define OFFSET_TEST_BFR				92
#define OFFSET_TEST_DEVICE_SERIAL	96
#define OFFSET_TEST_PH_NOT_USED		100
#define OFFSET_TEST_CA_NOT_USED		104
#define OFFSET_TEST_TH_NOT_USED		108
#define OFFSET_TEST_NH4_NOT_USED_1	112
#define OFFSET_TEST_NH4_NOT_USED_2	116
#define OFFSET_TEST_ALK_NOT_USED	120
#define OFFSET_RAW_ALK_NOT_USED_2	124
//#define OFFSET_TEST_DATA_ZERO		124
//#define OFFSET_TEST_ORP_2			125

// Memory offset of raw test data
//#define OFFSET_RAW_T_CAL_B1			0
#define OFFSET_RAW_PH_3_RINSE		0
#define OFFSET_RAW_T_RINSE			4
#define OFFSET_RAW_PH_1_RINSE		8
#define OFFSET_RAW_PH_2_RINSE		12
#define OFFSET_RAW_NH4_2_RINSE		16 // OFFSET_RAW_CO2_1_RINSE
#define OFFSET_RAW_NH4_3_RINSE		20 // OFFSET_RAW_CO2_2_RINSE
#define OFFSET_RAW_CA_1_RINSE		24
#define OFFSET_RAW_CA_2_RINSE		28
#define OFFSET_RAW_TH_1_RINSE		32
#define OFFSET_RAW_TH_2_RINSE		36
#define OFFSET_RAW_NH4_RINSE		40
#define OFFSET_RAW_T_SAMP_B1		44
#define OFFSET_RAW_PH_1_SAMP_B1		48
#define OFFSET_RAW_PH_2_SAMP_B1		52
#define OFFSET_RAW_CO2_1_SAMP_B1	56
#define OFFSET_RAW_CO2_2_SAMP_B1	60
#define OFFSET_RAW_T_SAMP			64
#define OFFSET_RAW_PH_1_SAMP		68
#define OFFSET_RAW_PH_2_SAMP		72
#define OFFSET_RAW_CA_1_SAMP		76
#define OFFSET_RAW_CA_2_SAMP		80
#define OFFSET_RAW_TH_1_SAMP		84
#define OFFSET_RAW_TH_2_SAMP		88
#define OFFSET_RAW_NH4_SAMP			92
#define OFFSET_RAW_CL_FCL			96
#define OFFSET_RAW_CL_TCL			100
#define OFFSET_RAW_COND				104
#define OFFSET_CHOSEN_SENSORS		108
#define	OFFSET_STEPS_B1				109
#define OFFSET_STEPS_B2				111
#define OFFSET_STEPS_T1_1			113
#define OFFSET_STEPS_T1_2			115
//#define OFFSET_PH_T1_1_NOT_USED		117
//#define OFFSET_PH_T1_2_NOT_USED		121
#define OFFSET_RAW_PH_3_SAMP		117
#define OFFSET_RAW_PH_3_POSTRINSE	121
#define OFFSET_ALK_METHOD_1			125
#define OFFSET_ALK_METHOD_2			126
#define OFFSET_ALK_METHOD_3			127
//#define	OFFSET_RAW_TEST_ZERO		127

#endif



// On-chip Memory offsets goes up to 0 -> 2047
#define OFFSET_PH_1_LAST_PRERINSE	0
#define OFFSET_PH_2_LAST_PRERINSE	4
#define OFFSET_CA_1_LAST_PRERINSE	8
#define OFFSET_CA_2_LAST_PRERINSE	12
#define OFFSET_T_CAL_2				16
#define OFFSET_PH_1_MV_RINSE		20
#define OFFSET_PH_2_MV_RINSE		24
#define OFFSET_EEPROM_RINSE			28
#define OFFSET_CALCONDSLOPELOW		32
#define OFFSET_CALCONDSLOPEMID		36
#define OFFSET_CALCONDSLOPEHIGH		40
#define OFFSET_CALCONDKLOW			44
#define OFFSET_CALCONDKMID			48
#define OFFSET_CALCONDKHIGH			52

#define OFFSET_COND_CAL_NUMBER			56
#define OFFSET_COND_TEST_NUMBER			60
#define OFFSET_COND_TEST_FLAG			64

#define OFFSET_CHOSEN_PH		68
#define OFFSET_CHOSEN_NH4		72
#define OFFSET_CHOSEN_CA		76
#define OFFSET_CHOSEN_TH_POS	80
#define OFFSET_CHOSEN_TH_NEG	84
#define OFFSET_PS_CHOSEN		88
#define OFFSET_HIBERNATE_FLAG	92

#define OFFSET_T_EEP_CAL		100
#define OFFSET_CO2_1_EEP_SLOPE	104
#define OFFSET_CO2_2_EEP_SLOPE	108
#define OFFSET_PH_1_EEP_SLOPE	112
#define OFFSET_PH_2_EEP_SLOPE	116
#define OFFSET_NH4_EEP_SLOPE	120
#define OFFSET_CA_1_EEP_SLOPE	124
#define OFFSET_CA_2_EEP_SLOPE	128
#define OFFSET_TH_1_EEP_SLOPE_POS	132
#define OFFSET_TH_2_EEP_SLOPE_POS	136
#define OFFSET_NH4_1_EEP_SLOPE	140
#define OFFSET_NH4_2_EEP_SLOPE	144
#define OFFSET_EEPROM_RINSE_1	148
#define OFFSET_EEPROM_RINSE_2	152
#define OFFSET_TH_1_EEP_SLOPE_NEG	156
#define OFFSET_TH_2_EEP_SLOPE_NEG	160
// 164 -> 200 used in amperometric V8 and V9 to save raw rinse data
// 164 -> 208 used in amperometric V11
#define OFFSET_MONO_CL_CYCLE	204
#define OFFSET_AUTO_CAL			208
#define OFFSET_ALKALINITY_CYCLE	212
#define OFFSET_FREE_CL_CYCLE	216
#define OFFSET_ISE_TIME			1024
// 1024->1816 used to save ISE data over time
// pH_1, pH_2, Ca_1, Ca_2, TH_1, TH_2, NH4_1, NH4_2, NH4_3
// 1024, 1028, 1032, 1036, 1040, 1044, 1048,  1052,  1056
// 1060, 1064, 1068... etc.

// 1820 and 1824 are used to save both pH differences
#define OFFSET_PH_B2_MV_DIFF	1820
#define OFFEST_MANUFACTURER_NAME	1828 //-> 1847
#define OFFSET_MODEL_NUMBER		1848	//20
#define OFFSET_SERIAL_NUMBER	1868	//20
#define OFFSET_HARDWARE_REV		1888	//20
#define OFFSET_FIRMWARE_REV		1908	//20

#define OFFSET_TEMP_CAL			1928	// 4	// Current Correction Coefficient
#define OFFSET_FC_STEPS_B1			1932	// 4
#define OFFSET_FC_STEPS_B2			1946	// 4
#define OFFSET_AMP_LOW_R_CAL		1950	// 4	// Resistance Correction Coefficient
#define OFFSET_AMP_HIGH_R_CAL		1954	// 4	// Resistance Correction Coefficient
#define OFFSET_THERM_CURRENT		1958	// 4 Thermistor current to be used in temperature calculation
#define OFFSET_AMP_I_VSET			1962	// 4 Vset for the current driving on the amperometric arrays to get the current to 3uA

#define OFFSET_DBOARD_V				1966
#define OFFSET_ABOARD_V				1970

#define OFFSET_PUMP_VOL_PER_REV		1974
#define OFFSET_PUMP_DEAD_SPOT		1978
#define OFFSET_CL_RESIST_RATIO		1982	// Float for the ratio of the voltage shifter on the Cl measurement channel, using this for the cleaning channel
#define OFFSET_CL_MID_R				1986	// Float for the measurement resistance on the mid range chlorine measurement
#define OFFSET_CL_CLEAN_R			1990	// Float for the measurement resistance on the cleaning range for chlorine
#define OFFSET_CL_LOW_R				1994	// Float for the measurement resistance on the Low range chlorine measurement

#define OFFSET_COND_I_LOW			1998	// Float for the measured current from QC process in uA
#define OFFSET_COND_I_MID			2002	// Float for the measured current from QC process in uA
#define OFFSET_COND_I_HIGH			2006	// Float for the measured current from QC process in uA

#define OFFSET_CL_MID_R_RATIO		2010	// Float for the ratio of the voltage shifter on the Cl measurement channel, using this for the high measurement channel
#define OFFSET_CL_LOW_R_RATIO		2014	// Float for the ratio of the voltage shifter on the Cl measurement channel, using this for the low measurement channel

#define OFFSET_CAL_RERUN			2018	// Flag to hold whether to rerun calibration or not, need this so it carries over on resets and can be set with UART

#define OFFSET_COND_ALT_I_LOW		2022
#define OFFSET_COND_ALT_I_MID		2026
#define OFFSET_COND_ALT_I_HIGH		2030

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void IO_Ext_Set(unsigned int IO_EXT_ADDR, uint8_t ui8Register, unsigned int uiPin, bool bState);

extern void InitIO_Ext(void);

extern void InitDAC(void);

extern void DACVoltageSet(uint8_t DAC_port, float V_out, bool update);

extern void InitADC(void);

extern void InitTurbidityADC(void);

extern float ADCRead(uint8_t ui8Channel, int ADC_CS_PIN);

extern float ADCReadAvg(uint8_t ui8Channel, int ADC_CS_PIN, int samples);

extern float StreamADCChannel(uint8_t ui8Channel, int ADC_CS_PIN, int time);

extern void ADCCurrentSet(bool state);

extern void MemoryWrite(unsigned int page, unsigned int byte, unsigned int num_of_bytes, uint8_t *pui8DataTx);

extern uint8_t * MemoryRead(unsigned int page, unsigned int byte, unsigned int num_of_bytes);

extern void InitBattery();

extern unsigned int BatteryRead(uint8_t ui8Register);

extern void BuzzerSound(unsigned int uiCycles);

#ifdef CONST_COND_FREQ
extern void InitWaveGen(uint8_t Check_freq);
#else
extern void InitWaveGen(uint8_t Check_freq, uint16_t ui16freq);
#endif

extern void WaveGenSet(bool state);

extern void BatteryIntHandler(void);

//extern void InitAccel(void);
//
//extern float * AccelRead(void);
//
//extern void AccelIntHandler(void);

extern void InitLED_Ext(void);

extern void SetLED(uint16_t LED, uint8_t state);

#ifdef CONST_COND_FREQ
extern uint8_t CheckCond(void);
#else
extern uint8_t CheckCond(uint16_t ui16freq);
#endif

extern float ReadThermistor(void);

extern uint8_t * GetTime(void);

extern uint8_t PrintTime(void);

#endif /* COMPONENTS_H_ */
