//*****************************************************************************
//
// Steppers.h - Functions used in e-SENS firmware to control valve and pump
//
// Author: Jason Castillo
//
//*****************************************************************************

#ifndef STEPPERS_H_
#define STEPPERS_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************
#define VALVE_STEP_PER_REV 			1818 // Accounting for gear ratio
#define NO_OF_VALVE_PORTS 			10
#define VALVE_STEPS_PER_POSITION	182 //(VALVE_STEP_PER_REV / NO_OF_VALVE_PORTS)

#define STEP_PER_REV_PUMP		1000
//#define ES_STEP_PER_REV_PUMP		2046

#define VALVE 1
#define PUMP  2

//#define ESPUMP 0	// Define which pump stepper is in box, 0 = 1000 step, 1 = 2046 step

#define FW 1	// Counter-Clockwise
#define BW 2	// Clockwise

#ifdef VALVE_STRUCT
struct ValvePort{
	uint8_t Air, Samp, B1, B2, C2, Clean, Cal_1, Cal_2, T1, Rinse;
};

extern struct ValvePort ValveSetup;

#define V_AIR		ValveSetup.Air
#define V_SAMP		ValveSetup.Samp
#define V_B1		ValveSetup.B1
#define V_C2		ValveSetup.C2
#define V_B2		ValveSetup.B2
#define V_CLEAN		ValveSetup.Clean
#define V_CAL_1 	ValveSetup.Cal_1
#define V_CAL_2		ValveSetup.Cal_2
#define V_T1		ValveSetup.T1
#define V_RINSE		ValveSetup.Rinse

#else
//extern float gPump_Ratio;

//// Define valve ports solutions are on
//#define V_C2		1
//#define V_B2		2	// B2 must be on port before B1, if changed RunValveToPossition_Bidirectional needs to be changed
//#define V_B1		3
//#define V_CAL_1 	4
//#define V_CAL_2		5
//#define V_CAL_3 	6
//#define V_SAMP		7
//#define V_RINSE		8
//#define V_STORE		9
//#define V_AIR		10

//// Define valve ports solutions are on
//#define V_AIR		1
//#define V_B1		2	// B1 and B2 must be on ports 2 and 3, no particular order, if changed RunValveToPossition_Bidirectional needs to be changed
//#define V_B2		3
//#define V_C2		4
//#define V_STORE		5
//#define V_CAL_1 	6
//#define V_CAL_3 	7
//#define V_CAL_2		8
//#define V_RINSE		9
//#define V_SAMP		10
//
// Define valve ports solutions are on
#define V_AIR		1
#define V_SAMP		2
#define V_B1		3
#define V_C2		5
#define V_B2		4
#define V_CLEAN		6
#define V_CAL_1 	7
#define V_CAL_3 	9
#define V_CAL_2		8
#define V_T1		9
#define V_RINSE		10

//// Define valve ports solutions are on
//#define V_AIR		10
//#define V_SAMP		1
//#define V_B1		3
//#define V_C2		5
//#define V_B2		4
//#define V_CLEAN		8
//#define V_CAL_1 	9
////#define V_CAL_3 	9
//#define V_CAL_2		6
//#define V_T1		2
//#define V_RINSE		7
#endif

extern uint32_t g_CurrentValvePossition;
extern int32_t g_ValveStepsTravelled;
extern int32_t g_PumpStepsTravelled;
extern int32_t g_CurrentValveStep;
extern uint32_t g_ValveADCAvg;
extern uint8_t g_ValveIndex;
extern uint32_t g_ValveADCAvg_FW;
extern uint8_t g_ValveIndex_FW;
extern uint32_t g_ValveADCAvg_BW;
extern uint8_t g_ValveIndex_BW;
extern uint8_t g_ValveHigh;
extern uint8_t g_ValveDirection;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void InitSteppers(uint8_t Valve_home);

extern void StepperRun(uint8_t motor, uint8_t Direction, uint32_t NumberOfSteps);

#ifndef OAD
extern void PumpStepperRun(uint8_t Direction, float ulSeconds);

extern void PumpStepperRunFast(uint8_t Direction, float ulSeconds);

extern void PumpStepperRunStep(uint8_t Direction, uint32_t NumberOfSteps);

#endif

extern void PumpStepperRunStepSlow(uint8_t Direction, uint32_t NumberOfSteps);

extern void PumpStepperRunStepSlow_AbortReady(uint8_t Direction, uint32_t NumberOfSteps);

extern void PumpStepperRunStepSpeed(uint8_t Direction, uint32_t NumberOfSteps, uint32_t EndDelay);

extern void PumpStepperRunStepSpeed_AbortReady(uint8_t Direction, uint32_t NumberOfSteps, uint32_t EndDelay);

extern void PumpStepperRunTimeSpeed_AbortReady(uint8_t Direction, uint32_t Time, uint32_t EndDelay);

extern void PumpStepperMix(uint8_t StartDirection, uint32_t NumberOfSteps, uint32_t EndDelay, uint8_t cycles);

//extern void ESPumpStepperRunStepSlow(int Direction, int NumberOfSteps);

extern uint8_t FindPossitionZeroPump(void);

//extern void ESFindPossitionZeroPump(void);

extern void ValveStepperRunSlow(int Direction, int NumberOfSteps);

#ifndef OAD
extern void RunValveToPossition(int Direction, int Possition, int valve_steps_per_possition);
#endif

extern uint8_t FindPossitionOneValve(void);

extern void ValveStepperRunSlow_Bidirectional(int Direction, int NumberOfSteps);

extern void RunValveToPossition_Bidirectional(int Possition, int valve_steps_per_possition);

extern void SleepValve(void);

extern void RunValveToPossition_Bidirectional_AbortReady(int Possition, int valve_steps_per_possition);

//extern void SpinValve(int Times);
//
//extern void StoreValve(void);

//extern void PumpBubbleSolutionSequence(uint8_t Valve_port, uint8_t Number_of_bubbles, uint32_t Steps_air_bubble, uint32_t Steps_solution_bubble, uint32_t Steps_solution, uint32_t Steps_center, uint8_t valve_delay, uint8_t pause_on);

//extern void PumpSingleMix(uint8_t Solution_port, uint8_t Buffer_port, uint32_t Steps_Solution_1, uint32_t Steps_Buffer, uint32_t Steps_Solution_2, uint32_t Steps_MixingChamber, uint32_t Steps_Center, uint8_t mix_cycles, uint16_t Steps_mix, uint8_t diffusion_time, uint8_t valve_delay);

//extern uint32_t StepsToConductivity(void);

//extern uint32_t StepsToRE(void);

extern int16_t TestValveDrift(void);

extern void TurnValveToStore(void);

extern void PumpVolume(uint8_t Direction, float Volume, uint32_t EndDelay, uint8_t AbortReady);

#endif /* STEPPERS_H_ */
