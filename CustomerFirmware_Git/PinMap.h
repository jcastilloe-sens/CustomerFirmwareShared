//*****************************************************************************
//
// PinMap.h - Define the GPIO bases and pins used
//
// Author: Jason Castillo
// Created: 12/27/2022, Created for the Roam Digital V1.0 board revision
//
//*****************************************************************************

#ifndef PINMAP_H_
#define PINMAP_H_

//*****************************************************************************
//
// Additional Defines for the API.
//
//*****************************************************************************
//
// Pin and Macro (if necessary) defines
//
// Did NOT include communication pins, I2C, SSI, and UART in this table, they are still hardcoded
#define IO_BUZZER_BASE(Board_Version)	(Board_Version >= RV1_0 ? GPIO_PORTE_BASE:GPIO_PORTB_BASE)
#define IO_BUZZER_PIN		GPIO_PIN_6	// Pin number didn't change, just base

#define IO_VALVE_SLEEP_BASE(Board_Version) (Board_Version >= RV1_0 ? GPIO_PORTE_BASE:GPIO_PORTB_BASE)
#define IO_VALVE_SLEEP_PIN 	GPIO_PIN_7	// Pin number didn't change, just base

#define	IO_ACC_INT_1_BASE(Board_Version)	(Board_Version >= V6_2 ? GPIO_PORTB_BASE:GPIO_PORTD_BASE)
#define	IO_ACC_INT_1_PIN					GPIO_PIN_4	// Pin number didn't change, just pin base

#define IO_I2C_USED_BY_BT_BASE		GPIO_PORTA_BASE
#define IO_I2C_USED_BY_BT_PIN		GPIO_PIN_6
#define IO_TIVA_RQST_BASE			GPIO_PORTA_BASE
#define IO_TIVA_RQST_PIN			GPIO_PIN_7

#define IO_LED_FULL_INTENSITY_BASE	GPIO_PORTB_BASE	// Only existed on V6
#define IO_LED_FULL_INTENSITY_PIN	GPIO_PIN_1		// Only existed on V6
#define IO_CART_PRES_B_BASE					GPIO_PORTB_BASE
#define IO_CART_PRES_B_PIN(Board_Version)	(Board_Version >= V6_2 ? GPIO_PIN_5:GPIO_PIN_0)
#define INT_CART_PRES_B				INT_GPIOB

#define IO_PUMP_STEP_BASE	GPIO_PORTC_BASE
#define IO_PUMP_STEP_PIN	GPIO_PIN_4
#define IO_PUMP_DIR_BASE	GPIO_PORTC_BASE
#define IO_PUMP_DIR_PIN		GPIO_PIN_5
#define IO_VALVE_STEP_BASE	GPIO_PORTC_BASE
#define IO_VALVE_STEP_PIN	GPIO_PIN_6
#define IO_VALVE_DIR_BASE	GPIO_PORTC_BASE
#define IO_VALVE_DIR_PIN	GPIO_PIN_7

#define IO_LED_EXT_RST_B_BASE	GPIO_PORTD_BASE	// Only existed from V6.2->V7.3, Removed extender after, was LEDs before
#define IO_LED_EXT_RST_B_PIN	GPIO_PIN_0		// Only existed from V6.2->V7.3, Removed extender after, was LEDs before
#define IO_LED_EXT_CS_B_BASE	GPIO_PORTD_BASE // Only existed from V6.2->V7.3, Removed extender after, was LEDs before
#define IO_LED_EXT_CS_B_PIN		GPIO_PIN_1		// Only existed from V6.2->V7.3, Removed extender after, was LEDs before
#define IO_COND_ADC_CNV_BASE	GPIO_PORTD_BASE
#define IO_COND_ADC_CNV_PIN		GPIO_PIN_2
#define IO_RESET_BT_BASE		GPIO_PORTD_BASE
#define IO_RESET_BT_PIN			GPIO_PIN_3
#define IO_ANALOG_ON_BASE		GPIO_PORTD_BASE
#define IO_ANALOG_ON_PIN		GPIO_PIN_6
#define IO_RESET_ANALOG_BASE	GPIO_PORTD_BASE	// Locked pin, InitGPIO has unlocking steps hardcoded
#define IO_RESET_ANALOG_PIN		GPIO_PIN_7


#define IO_I2C_USED_BY_TIVA_BASE	GPIO_PORTE_BASE	// This is only applicable for >= V7
#define IO_I2C_USED_BY_TIVA_PIN		GPIO_PIN_0	// This is only applicable for >= V7
#define IO_BAT_CHARGING_BASE		GPIO_PORTE_BASE // This pin in only applicable for < V7
#define IO_BAT_CHARGING_PIN			GPIO_PIN_0	// This pin in only applicable for < V7
#define IO_BAT_ALERT_BASE			GPIO_PORTE_BASE
#define	IO_BAT_ALERT_PIN			GPIO_PIN_1
#define IO_PUMP_MAG_SENS_BASE		GPIO_PORTE_BASE
#define IO_PUMP_MAG_SENS_PIN		GPIO_PIN_2
#define IO_VALVE_MAG_SENS_BASE		GPIO_PORTE_BASE
#define IO_VALVE_MAG_SENS_PIN		GPIO_PIN_3
#define IO_POWER_GOOD_B_BASE		GPIO_PORTE_BASE	// Only exists >= V6.2, before it was used for battery charging light
#define IO_POWER_GOOD_B_PIN			GPIO_PIN_4
#define IO_LED_OSC_RST_B_BASE		GPIO_PORTE_BASE	// Only exists >= V6.2, before it was used for button light
#define IO_LED_OSC_RST_B_PIN		GPIO_PIN_5
#define INT_BATTERY_BASE			INT_GPIOE

#define IO_BUTTON_BASE			GPIO_PORTF_BASE
#define IO_BUTTON_PIN			GPIO_PIN_3
#define IO_PUMP_SLEEP_BASE		GPIO_PORTF_BASE
#define IO_PUMP_SLEEP_PIN		GPIO_PIN_4

// LED defines from Roam Digital V1 Ports G and M
#define LED_BUT_B_V_SW_BASE		GPIO_PORTG_BASE	// Using this define to identify the whole LED bank
#define LED_BUT_B_V_SW_PIN		GPIO_PIN_0
#define LED_BUT_B_V_SEL_BASE	GPIO_PORTG_BASE
#define LED_BUT_B_V_SEL_PIN		GPIO_PIN_1

#define LED_BUT_R_V_SW_BASE		GPIO_PORTG_BASE
#define LED_BUT_R_V_SW_PIN		GPIO_PIN_2
#define LED_BUT_R_V_SEL_BASE	GPIO_PORTG_BASE
#define LED_BUT_R_V_SEL_PIN		GPIO_PIN_3

#define LED_BUT_G_V_SW_BASE		GPIO_PORTG_BASE
#define LED_BUT_G_V_SW_PIN		GPIO_PIN_4
#define LED_BUT_G_V_SEL_BASE	GPIO_PORTG_BASE
#define LED_BUT_G_V_SEL_PIN		GPIO_PIN_5

#define LED_CHG_G_BASE			GPIO_PORTG_BASE
#define LED_CHG_G_PIN			GPIO_PIN_6
#define LED_CHG_Y_BASE			GPIO_PORTG_BASE
#define LED_CHG_Y_PIN			GPIO_PIN_7

#define LED_BUT_R_H_SW_BASE		GPIO_PORTM_BASE	// Using this define to identify the whole LED bank
#define LED_BUT_R_H_SW_PIN		GPIO_PIN_0
#define LED_BUT_R_H_SEL_BASE	GPIO_PORTM_BASE
#define LED_BUT_R_H_SEL_PIN		GPIO_PIN_1

#define LED_BUT_B_H_SW_BASE		GPIO_PORTM_BASE
#define LED_BUT_B_H_SW_PIN		GPIO_PIN_2
#define LED_BUT_B_H_SEL_BASE	GPIO_PORTM_BASE
#define LED_BUT_B_H_SEL_PIN		GPIO_PIN_3

#define LED_BUT_G_H_SW_BASE		GPIO_PORTM_BASE
#define LED_BUT_G_H_SW_PIN		GPIO_PIN_4
#define LED_BUT_G_H_SEL_BASE	GPIO_PORTM_BASE
#define LED_BUT_G_H_SEL_PIN		GPIO_PIN_5

#define LED_CHG_R_SW_BASE		GPIO_PORTM_BASE
#define LED_CHG_R_SW_PIN		GPIO_PIN_6
#define LED_CHG_R_SEL_BASE		GPIO_PORTM_BASE
#define LED_CHG_R_SEL_PIN		GPIO_PIN_7

// LED defines from before V6.2, before osciallator and split horizontal vs vertical
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

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

#endif /* PINMAP_H_ */
