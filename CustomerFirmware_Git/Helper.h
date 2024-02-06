//*****************************************************************************
//
// Helper.h - Functions used in e-SENS firmware to support other functions and
// to initialize parts of the device not used for specific systems
//
// Author: Jason Castillo
//
//*****************************************************************************

#ifndef HELPER_H_
#define HELPER_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************
#ifdef PRINT_UART
#define DEBUG_PRINT(x) x
#else
#define DEBUG_PRINT(X)
#endif

extern uint32_t g_TimerInterruptFlag;
extern uint32_t g_TimerPeriodicInterruptFlag;
extern uint32_t g_TimerTemperatureInterruptFlag;
extern uint32_t gui32Error;

extern uint8_t gui8MemConnected;

extern uint32_t gBoard;
extern uint32_t gABoard;

// Define which digital board is in use
#define V1			1
#define V2			2
#define V3			3
#define CAL1		4
#define V5			5
#define V6			6
#define V6_2		7
#define V6_3		8
#define V6_4		9
#define V7			10
#define V7_2		11
#define V7_3		12
#define V7_3E		13
#define RV1_0		14
//#define gBoard		V7

//#define gDiagnostics 	0	// 1 to see most function calls, 2 to see loops we may get stuck in
//#define gHibernate_ON	0	// 0 to not hibernate, 1 to hibernate; this variable only affects hibernate timeout interrupt handler

#define AV4			1
#define AV5			2
#define AV6			3
#define AV6_1		4
#define AV6_2		5
#define AV6_3		6
#define AV6_4		7
#define AV6_5		8
#define AV6_6		9
#define AV7			10
#define AV7_2		11
#define AV7_3		12
#define ARV1_0B		13
//#define gABoard		AV7_3

// Structure holding variables defining each type of ISE
struct ISEType{
	uint8_t size;		// Number of this type of sensor
	uint8_t index;		// Index in the length 10 arrays that hold ISE data this sensor type starts on
	uint8_t StorBit;	// Bit in
	uint8_t type;		// Type of sensor, used in
};

#ifdef SOLUTION_IN_STRUCT
struct SolutionVals{
	float pH_EEP_Rinse, Ca_EEP_Rinse, TH_EEP_Rinse, NH4_EEP_Rinse, Cond_EEP_Rinse;
	float pH_EEP_Cal_2, Ca_EEP_Cal_2, TH_EEP_Cal_2, NH4_EEP_Cal_2, Cond_EEP_Cal_2;
	float pH_EEP_Cal_1, Ca_EEP_Cal_1, TH_EEP_Cal_1, NH4_EEP_Cal_1, Cond_EEP_Cal_1;
	float pH_EEP_Clean, Ca_EEP_Clean, TH_EEP_Clean, NH4_EEP_Clean, Cond_EEP_Clean;
	float HCl_N;

	float IS_RINSE, IS_CLEAN, IS_CAL_1, IS_CAL_2;
	float K_T_pH_Rinse, K_T_pH_Cal_1, K_T_pH_Cal_2, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln;
	float Rinse_Cond_TComp, Cal_1_Cond_TComp, Cal_2_Cond_TComp, Clean_Cond_TComp;
};

//extern struct SolutionsVals Sols;
#endif

#define TYPE_PH_H2	1
#define TYPE_PH_CR	2
#define TYPE_TH		3
#define TYPE_NH4	4
#define	TYPE_CA		5

// Structure defining how the whole ISE die is setup.. Including solution values so this is more of a cartridge setup now...
struct ISEConfig{
	uint8_t Config;			// Number defining which die dispense this is ORIGINAL_CART, PH_CL_CART... etc.
	uint8_t Location[10];	// Map between the hardware ISE numbers and the firmware organization
	uint8_t CalDelay;		// Days between calibrations
	uint8_t RunAlk;			// Flag whether this sensor configuration runs alkalinity or not
	struct ISEType pH_H2, pH_Cr, TH, NH4, Ca;	// Structures defining each sensor type
//	struct SolutionVals Rinse, Clean, Cal_1, Cal_2;
//	float T1_HCl_N;
};

//extern float K_T_pH_Cal_1; //0.0063; // Temperature coefficient Cal 1
//extern float K_T_pH_Cal_2; //0.0053; // Temperature coefficient Cal 2
//extern float K_T_pH_Rinse;
//extern float K_T_pH_Samp;
//extern float pKa_NH4;

#ifndef SOLUTION_IN_STRUCT
#define K_T_pH_Cal_1  	-0.0025 //0.0063; // Temperature coefficient Cal 1
#define K_T_pH_Cal_2  	-0.0243 //-0.0155 //0.0053; // Temperature coefficient Cal 2
#define K_T_pH_Rinse 	-0.0129 // -0.0127
#define K_T_pH_Clean	-0.0127
#define K_T_pH_Clean_Sq	.00007
#define K_T_pH_Clean_Ln	-.0071

#define pKa_NH4			9.246
#define	IS_CAL_1			0.0321
#define	IS_CAL_1_MG_ADJ		0.03968
#define IS_CAL_2		0.00335
#define IS_RINSE		0.0115
#define IS_CLEAN		0.0187 // pH 6 Clean: .009
#endif

#define K_T_pH_Samp		-0.009
#define K_T_pH_Samp_Sq	0.0001
#define K_T_pH_Samp_Ln	-0.0134

#define LOG_K			0.125	// Define log K here, it will be used in both calibration and tests
#define LOG_K_CA_MG		-.5	// Define log K here, it will be used in both calibration and tests
#define LOG_K_K_NH4		-0.85
#define LOG_K_NA_NH4	-2.9
#define SM_NA_CAL_1		0 // 0.010928

// Define Sensor Configurations
#define ORIGINAL_CART		0	// All ISEs, runs the normal test
#define PH_CL_CART			1	// pH only die, runs pH ISE, Chlorines, and Amps, no Ca TH NH4 Alk
#define H2_CART				2	// Die that includes 2 spots for H2 and reduces pH and NH4 both to 2
#define PH_H2_CART			3	// 5 spots for H2 and 5 spots for pH, want to run alkalinity
#define H2_ONLY				4	// All 10 spots are H2
#define CA_ONLY				5
#define TH_ONLY				6
#define NH4_ONLY			7
#define CR_CA_SWAP			8	// 1/5/2021: Swapped Chromo and Ca spots, because Ca is related to chip cracking
#define ACROSS				9	// 2/22/2022: Each sensor type is across the die from each other, going Ca, NH4, TH, Cr, H2
#define ACROSS_V2			10	// 3/16/2022: Each sensor type is across the die from each other, going H2, Cr, Ca, NH4, TH
#define ACROSS_V3			11	// 3/30/2022: H2 in spots 1 and 3, Cr in spots 2 and 4, rest of sensors across from each other going Ca, NH4, then TH moving toward reference
#define ACROSS_V4			12	// 5/2/2022: NH4 in spots 1 and 3, TH in spots 2 and 4, rest of sensors across from each other going Ca, Cr, then H2 moving toward reference
#define ACROSS_V5			13	// 5/18/2022: NH4 in spots 1 and 3, TH in spots 2 and 4, rest of sensors across from each other going H2, Cr, then Ca moving toward reference
#define ACROSS_V6			14	// 5/18/2022: H2 in spots 1 and 3, TH in spots 2 and 4, rest of sensors across from each other going NH4, Cr, then Ca moving toward reference
#define ACROSS_V7			15	// 6/30/2022: Across going Cr, Ca, TH, NH4, H2 moving toward reference
#define CONFIG_V8			16	// 7/14/2022: Play on the Normal Configuration
#define CONFIG_V9			17	// 7/14/2022: Across V9 going TH, Cr, Ca, Cr3, NH4 moving toward reference
#define CONFIG_V10			18	// 8/24/2022: Across V10 going TH, Cr, NH4, Cr3, Ca moving toward reference
#define ACROSS_V7_BACKWARDS	19	// 9/26/2022: Across V7 mounted backwards
#define DISINFECTION_CART	20	// 12/19/2023: Disinfection cartrdige 6 pH then 4 NH4s

// Define states in shared file
#define STATE_IDLE				0
#define STATE_CALIBRATION		1
#define STATE_MEASUREMENT		2
#define STATE_UPDATING			3

#define LARGE_AIR		1	// 1 if push large air bubble right before plug to be measured, 0 for regular pumping
#define GND_CE_ON		1	// 0 if not grounding counter, 1 if grounding counter, in WatchISEs function
#define CLEAN_AMPS		1	// 0 will NOT clean WEs+ORP sensor, 1 will clean WEs+ORP sensor in prerinse of test
#define DISCONNECT_RE	1	// 0 will leave RE connected while pumping/idle; 1 will leave RE disconnected when not measuring
#define CHECK_RE		1	// 0 will not check for RE connection, 1 will check for RE connection, in WatchISEs function
//#define RE_THRESHOLD	15	// mV threshold that will determine if there is connection to RE or not; NOT used on Cal 2
//#define PUMP_ATTEMPTS	3	// Number of times to re-pump in solution if checks don't pass

#define T_ASSUME 	25
#define HIBERNATE_TIMEOUT	(60 * 1)	// 5 minute timeout
#define TEMP_TIMEOUT		(60 * 15)	// 15 minute timeout

//// Define Cal solutions and Test solutions
//#define CALIBRATION		1
//#define CAL_PRERINSE	2
//#define CAL_3			3
//#define CAL_3_B1		4
//#define CAL_2			5
//#define CAL_2_B1		6
//#define CAL_1			7
//#define CAL_POSTRINSE	8
//#define RINSE_B2		9
//#define TEST			10
//#define TEST_PRERINSE	11
//#define TEST_SAMPLE		12
//#define TEST_POSTRINSE	13
//
//// Define which sensors are tested in each solution
//#define SENSORS_CAL_PRERINSE 	0x01FF
//#define SENSORS_CAL_3			0x0003
//#define SENSORS_CAL_3_B1		0x0000
//#define SENSORS_CAL_2			0x01FC
//#define SENSORS_CAL_2_B1		0x0000
//#define SENSORS_CAL_1			0x01FF
//#define SENSORS_CAL_POSTRINSE	0x01FF
//#define SENSORS_RINSE_B2		0x0000
//#define SENSORS_TEST_PRERINSE	0x01FF
//#define SENSORS_TEST_SAMPLE		0x01FF
//#define SENSORS_TEST_POSTRINSE	0x01FF

// Define acceptable slope ranges
#define PH_SLOPE_HIGH	-40
#define PH_SLOPE_LOW	-65
#define CA_SLOPE_HIGH	-20
#define CA_SLOPE_LOW	-34
#define TH_SLOPE_HIGH	-18
#define TH_SLOPE_LOW	-34
#define NH4_SLOPE_HIGH	-40
#define NH4_SLOPE_LOW	-66

#ifdef CURRENT_ADJUSTED_COND
#define COND_SLOPE_1_LOW		0.1
#define COND_SLOPE_1_HIGH		0.4
#define COND_SLOPE_2_LOW		0.1
#define COND_SLOPE_2_HIGH		0.35
#define COND_SLOPE_3_LOW		0.1
#define COND_SLOPE_3_HIGH		0.3
#else
#define COND_SLOPE_1_LOW		25000
#define COND_SLOPE_1_HIGH		75000
#define COND_SLOPE_2_LOW		50000
#define COND_SLOPE_2_HIGH		150000
#define COND_SLOPE_3_LOW		150000
#define COND_SLOPE_3_HIGH		400000
#endif

// Define errors
#define MAX_CALS		30	// Max number of calibration allowed before
#define MIN_BAT_LEVEL	15	// Minimum % to allow either test or cal to run
#define APP_FILTER_ERROR				0x00000001
#define MAX_TESTS_REACHED				0x00000002
#define MAX_CALS_REACHED				0x00000002
#define CARTRIDGE_EXPIRED				0x00000004
#define BATTERY_TOO_LOW					0x00000008
#define I2C_FAILED						0x00000010
#define IO_EXT1_FAIL					0x00000020
#define IO_EXT2_FAIL					0x00000040
#define DAC_FAIL						0x00000080
#define ADC1_FAIL						0x00000100
#define ADC2_FAIL						0x00000200
#define ADC3_FAIL						0x00000400
#define ADC4_FAIL						0x00000800
#define WAVE_GEN_FAIL					0x00001000
#define BT_FAILED						0x00002000
#define ROAM_RESET						0x00004000
#define CL_CLEANING_OUT_OF_RANGE		0x00008000
#define PUMP_DRIFT_FAIL					0x00010000
#define CAL_FAILED_PH_H2_SLOPES			0x00020000
#define T1_PRIME_COND_ERROR				0x00020000
#define CAL_FAILED_PH_CR_SLOPES			0x00040000
#define THERMISTOR_FAILED				0x00040000
#define CAL_FAILED_CA_SLOPES			0x00080000
#define FCL_MIX_OUT_OF_RANGE			0x00080000
#define CAL_FAILED_TH_SLOPES			0x00100000
#define TCL_MIX_OUT_OF_RANGE			0x00100000
#define CAL_FAILED_NH4_SLOPES			0x00200000
#define B1_PRIME_COND_ERROR				0x00200000
#define CAL_FAILED_COND_SLOPES			0x00400000
#define C2_PRIME_COND_ERROR				0x00400000
#define MEMORY_FAILED					0x00800000
#define USER_CANCELLED					0x01000000
#define ALK_MIX_OUT_OF_RANGE			0x02000000
#define LED_EXTENDER_FAIL				0x04000000
#define ADC5_FAIL						0x08000000
#define EEPROM_FAIL						0x10000000
#define BATTERY_FAIL					0x20000000
#define CAL_REPUMP_FAIL					0x40000000
#define B2_PRIME_COND_ERROR				0x40000000
#define VALVE_DRIFT_FAIL				0x80000000

// Or together errors that would make us skip test/calibration so we can if((gui32Error & ABORT_ERROR) == 0) to check if we should run
//#define ABORT_ERRORS	(USER_CANCELLED | IO_EXT1_FAIL | IO_EXT2_FAIL | DAC_FAIL | ADC1_FAIL | ADC2_FAIL | ADC3_FAIL | ADC4_FAIL | WAVE_GEN_FAIL | MEMORY_FAILED | VALVE_DRIFT_FAIL)
#define ABORT_ERRORS		(USER_CANCELLED | VALVE_DRIFT_FAIL)
#ifdef TESTING_MODE
#define STARTING_ERRORS		((MEMORY_FAILED | BATTERY_TOO_LOW) | INITILIZATION_FAILED)
#define FAILED_TEST		(BATTERY_TOO_LOW|I2C_FAILED|/*SPI_FAILED|*/MEMORY_FAILED|USER_CANCELLED|CL_CLEANING_OUT_OF_RANGE|FCL_MIX_OUT_OF_RANGE|TCL_MIX_OUT_OF_RANGE)
#else
#define STARTING_ERRORS		((MEMORY_FAILED | BATTERY_TOO_LOW | MAX_TESTS_REACHED | MAX_CALS_REACHED | CARTRIDGE_EXPIRED) | INITILIZATION_FAILED)
#define FAILED_TEST		(MAX_TESTS_REACHED|CARTRIDGE_EXPIRED|BATTERY_TOO_LOW|I2C_FAILED|/*SPI_FAILED|*/MEMORY_FAILED|USER_CANCELLED|CL_CLEANING_OUT_OF_RANGE|FCL_MIX_OUT_OF_RANGE|TCL_MIX_OUT_OF_RANGE)
#endif
#define INITILIZATION_FAILED		(EEPROM_FAIL | IO_EXT1_FAIL | IO_EXT2_FAIL | DAC_FAIL | ADC1_FAIL | ADC2_FAIL | ADC4_FAIL | ADC3_FAIL | WAVE_GEN_FAIL | BT_FAILED | LED_EXTENDER_FAIL | ADC5_FAIL | BATTERY_FAIL)

#define ERRORS_TO_FILTER_TEST	(IO_EXT1_FAIL | IO_EXT2_FAIL | DAC_FAIL | ADC1_FAIL | ADC2_FAIL | ADC4_FAIL | ADC3_FAIL | WAVE_GEN_FAIL | USER_CANCELLED | ADC5_FAIL | BATTERY_FAIL)

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void Init_all(uint8_t Steppers);

extern void InitDigital(uint8_t Valve_home);

extern void InitAnalog(void);

extern void AnalogOff(void);

extern void InitGPIOPins(void);

//extern void InitReset(void);

extern void InitFPU(void);

extern void InitTimer();

extern void Timer0IntHandler(void);

extern void Timer1IntHandler(void);

extern void TempRecordTimerIntHandler(void);

extern void InitHibernate(void);

extern void HibernateTimeoutIntHandler(void);

#ifdef MEMORY_V3
extern uint8_t FindTestNumber(void);

extern uint8_t FindCalNumber(void);
#else
extern uint16_t FindTestNumber(void);

extern uint16_t FindCalNumber(void);
#endif

extern float Build_float(uint8_t *pui8bytes);

extern uint8_t * RequestSystemStatus(void);

extern void CheckCartridge(uint8_t *pui8SysStatus);

extern bool CheckCalibration(uint8_t *SysStatus, uint8_t Acceptable_Days);

extern float abs_val(float number);

//extern float Rinse_Table_Lookup(float T_rinse);

extern void InitEEPROM(void);

#ifndef OAD
extern void CalibrateTemperature(void);
#endif

extern float MeasureTemperature(int seconds);

extern void JumpToBootLoader(void);

extern int cmpfunc (const void * a, const void * b);

extern float FindMedian(float * array, unsigned int size);

extern float FindStdDev(float * array, uint8_t size);

//extern float FindStdDevISEs(uint32_t EEPROM_addr, uint8_t size);

extern void InitCPUTemp(void);

extern float GetCPUTemp(void);

extern void InitCartridge(void);

extern void CartridgeIntHandler(void);

extern float FindArrayMax(float *Array, int size);

extern float FindArrayMin(float *Array, int size);

//extern void WaitCO2Steady(uint8_t samples, float period, float max_spread, uint16_t Min_time, uint16_t Max_time);
//
//extern uint16_t CheckConnection(int16_t RE_Threshold);
//
//extern uint8_t CheckRE(int16_t RE_Threshold);
//
//extern void Repump_Solution(uint8_t Solution);
//
//extern uint8_t WatchISEs(uint8_t ui8Seconds, uint8_t Solution, uint16_t * SensorStatus);

extern float RSQ(float *xs, float *ys, int n);

extern float FindBestFitSlope(float *xs, float *ys, uint8_t n);

extern void SortArray(float * Array, uint8_t size);

extern void ConnectMemory(uint8_t Connect);

extern void userDelay(uint32_t milliseconds, uint8_t Abort);

#ifdef TESTING_MODE
extern void MemoryDump(uint8_t Print_as_mg_hardness, uint8_t Die_RevD);
#endif

extern void PrintErrors(uint32_t ui32ErrorCode, uint8_t new_lines, uint8_t state);

extern uint16_t Find_Cal_page(uint16_t Cal_Number);

extern uint16_t Find_Test_page(uint16_t Test_Number);

#ifdef MEMORY_V4
extern uint8_t Choose_pH_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse);

extern uint8_t Choose_pH_Sensor_pHDie(uint16_t Cal_Number, float * pH_Samp);

extern uint8_t Choose_Ca_Sensor(uint16_t Cal_Number, float * Ca_Hardness, float * Ca_R);

extern uint8_t Choose_TH_Sensor(uint16_t Cal_Number, float * TH_corr, float * TH_R);

extern uint8_t Choose_NH4_Sensor(uint16_t Cal_Number, float * NH4_NH3_N_Total, float * NH4_R);

extern uint8_t Choose_Alk_Sensor(uint16_t Cal_Number, float * Alk_Samp, float * pH_R, float T_Rinse, uint8_t * method, float * Alk_Slope);
#else
#ifdef SOLUTION_IN_STRUCT
#ifndef UNIVERSAL_PICKING_FUNCTION
extern uint8_t Choose_pH_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs, struct SolutionVals *Sols);

extern uint8_t Choose_pH_H2_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs, struct SolutionVals *Sols);

extern uint8_t Choose_pH_Sensor_pHDie(uint16_t Cal_Number, float * pH_Samp);

extern uint8_t Choose_Ca_Sensor(uint16_t Cal_Number, float * Ca_Hardness, float * Ca_R, struct ISEConfig ISEs, struct SolutionVals *Sols);

extern uint8_t Choose_TH_Sensor(uint16_t Cal_Number, float * TH_corr, float * TH_R, struct ISEConfig ISEs, struct SolutionVals *Sols);

extern uint8_t Choose_NH4_Sensor(uint16_t Cal_Number, float * NH4_NH3_N_Total, float * NH4_R, struct ISEConfig ISEs, struct SolutionVals *Sols);
#else
extern uint8_t Choose_Sensor(uint16_t Cal_Number, float * Samp_Reading, float * E_Rinse, float T_Rinse, struct ISEType ISE_Type, struct SolutionVals *Sols);

#endif
extern uint8_t Choose_Alk_Sensor(uint16_t Cal_Number, float * Alk_Samp, float * pH_R, float T_Rinse, uint8_t * method, float * Alk_Slope, struct ISEConfig ISEs, struct SolutionVals *Sols);

#else
extern uint8_t Choose_pH_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs);

extern uint8_t Choose_pH_H2_Sensor(uint16_t Cal_Number, float * pH_Samp, float * pH_E_Rinse, float T_Rinse, struct ISEConfig ISEs);

extern uint8_t Choose_pH_Sensor_pHDie(uint16_t Cal_Number, float * pH_Samp);

extern uint8_t Choose_Ca_Sensor(uint16_t Cal_Number, float * Ca_Hardness, float * Ca_R, struct ISEConfig ISEs);

extern uint8_t Choose_TH_Sensor(uint16_t Cal_Number, float * TH_corr, float * TH_R, struct ISEConfig ISEs);

extern uint8_t Choose_NH4_Sensor(uint16_t Cal_Number, float * NH4_NH3_N_Total, float * NH4_R, struct ISEConfig ISEs);

extern uint8_t Choose_Alk_Sensor(uint16_t Cal_Number, float * Alk_Samp, float * pH_R, float T_Rinse, uint8_t * method, float * Alk_Slope, struct ISEConfig ISEs);
#endif // SOLUTION_IN_STRUCT
#endif // MEMORY_V4

extern void RecordTemp(void);

//extern uint8_t CheckSensorSat(void);

extern float Lambda_Ca(float Temp, float IS);

extern float Lambda_Mg(float Temp, float IS);

extern float Lambda_NH4(float Temp, float IS);

extern float Lambda_Na(float Temp, float IS);

extern float Lambda_K(float Temp, float IS);

extern float Calc_pH_TCor(float pH, float Temp_Cor, float Temp_Meas, float K_T_pH_Sq, float K_T_pH_Ln);

extern float Calc_pCa(float Ca_EEP_Conc, float Temp, float IS);

#ifdef PH_LOG_K
extern float Calc_pTHpH(float Ca_EEP_Conc, float TH_EEP_Conc, float Log_K_Ca_Mg, float Temp, float IS, float pH, float Log_K_pH_TH);
#endif

extern float Calc_pTH(float Ca_EEP_Conc, float TH_EEP_Conc, float Log_K_Ca_Mg, float Temp, float IS);

extern float Calc_pMg(float Ca_EEP_Conc, float TH_EEP_Conc, float Temp, float IS);

extern float Calc_pNH4(float NH4_EEP_Conc, float pH_TCor, float SM_Na, float Temp, float IS);

extern void CollectISEmV(float * ISE_mV_destination, uint16_t SpotsToSave, uint16_t time_to_wait, uint8_t PRINT_ISE_TIME_DATA, struct ISEConfig *ISEConfig);

extern void CollectISEmV_WhilePumping(uint8_t Direction, uint32_t EndDelay, float * ISE_mV_destination, uint16_t SpotsToSave, uint16_t time, uint8_t PRINT_ISE_TIME_DATA, struct ISEConfig *ISEConfig);

//#ifndef MEMORY_V4
//extern int32_t PumpUntilpH(uint8_t Direction, uint32_t EndDelay, float * ISE_mV_destination, uint16_t SpotsToSave, uint32_t Max_Steps, float Min_pH, float Max_pH, uint8_t PRINT_ISE_TIME_DATA, struct ISEConfig *ISEConfig, uint16_t Cal_Number, float pH_TCor_Rinse, float * ISE_E_Rinse);
//#endif
extern void FillISEStruct(struct ISEConfig *ISEConfig);

#ifdef SOLUTION_IN_STRUCT
extern struct SolutionVals* FillSolutionStruct(void);
//extern void FillSolutionStruct(struct SolutionVals *Sols);
#endif

extern int8_t Calculate_Ref_Drift(uint8_t Saturated_KCl, float Temperature);

#endif /* HELPER_H_ */
