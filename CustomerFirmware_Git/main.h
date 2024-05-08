/*
 * main.h
 *
 *  Created on: Dec 20, 2016
 *      Author: Jason
 */

#ifndef MAIN_H_
#define MAIN_H_

extern uint8_t g_state;
extern uint8_t g_next_state;

//// Define which digital board is in use
//#define V1			1
//#define V2			2
//#define V3			3
//#define CAL1		4
//#define V5			5
//#define V6			6
//#define V6_2		7
//#define V6_3		8
//#define V6_4		9
//#define V7			10
//#define V7_2		11
//#define gBoard		V7
//
#define gDiagnostics 	0	// 1 to see most function calls, 2 to see loops we may get stuck in
#define gHibernate_ON	0	// 0 to not hibernate, 1 to hibernate; this variable only affects hibernate timeout interrupt handler

//
//#define AV4			1
//#define AV5			2
//#define AV6			3
//#define AV6_1		4
//#define AV6_2		5
//#define AV6_3		6
//#define AV6_4		7
//#define AV6_5		8
//#define AV6_6		9
//#define AV7			10
//#define AV7_2		11
//#define AV7_3		12
//#define gABoard		AV7_3

//static uint8_t FIRMWARE_REVISION[20] =	"v00.22.01-b00.15.00 ";
static uint8_t FIRMWARE_REVISION[20] =	"T01.04.02-b01.02.03 ";

#endif /* MAIN_H_ */
