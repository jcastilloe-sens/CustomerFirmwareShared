//*****************************************************************************
//
// Amperometrics.h - Functions used by e-SENS firmware to control amperometric
// array
// Includes functions to clean array
//
// Author: Jason Castillo
//
//*****************************************************************************

#ifndef AMPEROMETRICS_H_
#define AMPEROMETRICS_H_

#ifdef SOLUTION_IN_STRUCT
#include "Helper.h"
#endif
//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//extern float FindCurrent(void);

#define COND_TCOMP_RINSE	0.02097
#define COND_TCOMP_CLEAN	0.02369
#define COND_TCOMP_CAL_1	0.02033
#define COND_TCOMP_CAL_2 	0.02253
#define COND_TCOMP_B1_MIX	0.0225
#define COND_TCOMP_B2_MIX	0.0226
#define COND_TCOMP_SAMP 	0.0198
#define COND_TCOMP_CONDLOW	0.0242

#define	CL_FCL_INT			-1.625
#define	CL_FCL_SLOPE 		-40.592
#define	CL_FCL_INT_HIGH 	8.388
#define	CL_FCL_SLOPE_HIGH 	-70.932

#define CL_TCL_SLOPE		-65.987
#define CL_TCL_INT			-7.547
#define CL_TCL_SLOPE_HIGH	-74.374
#define CL_TCL_INT_HIGH		-4.779

extern float CalculateCurrent(float ADC_Voltage, float DAC_Voltage, uint8_t High_current_switch);

//extern float * ADCReadAll(void);

extern float * CurrentTimeRead(uint8_t ui8Channel, int ADC_CS_PIN, float seconds, int DAC_Voltage, uint8_t High_current_switch, float period);

//extern void CleanGold(int8_t Ref_drift);

extern void CleanAmperometrics(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number, uint8_t Oxide_Rebuild);

extern void CleanAmperometrics_CurrentLimited(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number, uint8_t Oxide_Rebuild);

#ifdef SWEEP_CLEAN
extern void CleanAmperometricsSweep(int8_t Ref_drift);
#endif

extern float ConductivityMovingAvg(void);

#ifndef COND_SOLUTION_STRUCT
extern float MeasureConductivity(float Cond_EEP_Rinse, float Cond_EEP_Cal_2, uint8_t Test_Number);
#else
extern float MeasureConductivity(struct SolutionVals * Sols, uint8_t Test_Number);
#endif

//extern float ReadCl(uint8_t *Range_flag, float Cl_nA_cutoff);

//extern float CalculateCurrent_ORP(float ADC_Voltage, float DAC_Voltage);

//extern float CurrentTimeRead_ORP(uint8_t ui8Channel, int ADC_CS_PIN, float seconds, int DAC_Voltage, float period);

//extern void CleanORP(void);

#if defined CV_CLEANING || defined CV_MEASUREMENT
extern void RunCVCleaningCycle(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number);

extern void CVCleaning(int Start_mV, int End_mV, float rate, uint8_t BothDirections, uint8_t High_current_switch);

extern void CleanAmperometrics_CurrentLimited_CCOROnly(int8_t Ref_drift, uint16_t Cal_Number, uint16_t Test_Number, uint8_t Oxide_Rebuild);
#endif

#ifdef TESTING_MODE
extern float ReadRefGuard(float fSec);

extern void CleanCond(void);
#endif

#endif /* AMPEROMETRICS_H_ */
