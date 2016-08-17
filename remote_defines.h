/*
 * remote_defines.h
 *
 *  Created on: May 9, 2013
 *      Author: joao
 */

#ifndef REMOTE_DEFINES_H_
#define REMOTE_DEFINES_H_

#include "stdint.h"

#define COM_DRV_ZERO		690
#define COM_DRV_FRONT		784
#define COM_DRV_REAR		598
#define COM_SERVO_ZERO		645
#define COM_SERVO_RIGHT		733
#define COM_SERVO_LEFT		537
#define ADC_TH			10

#define DRV_ZERO		0
#define DRV_REAR		-99
#define DRV_FRONT		99

#define ADC_ANALOG_LEFT		3
#define ADC_ANALOG_RIGHT	0

#define SEND_MSG_TIME		40

#define ASK_BIT			0x10

#define LED_BIT			0x01
#define AUTO_OFF_BIT	0x02
#define FORCE_OFF_BIT	0x04


// PORT 2
#define L1_BUTTON		0x02
#define R1_BUTTON		0x04
#define L2_BUTTON		0x10
#define R2_BUTTON		0x20

// PORT 2
#define RED_LED			0x80
#define GREEN_LED		0x40
#define RED_LED_ON		P2OUT |= RED_LED;
#define RED_LED_OFF		P2OUT &= ~RED_LED;
#define RED_LED_BLINK	P2OUT ^= RED_LED;
#define GREEN_LED_ON	P2OUT |= GREEN_LED;
#define GREEN_LED_OFF	P2OUT &= ~GREEN_LED;
#define GREEN_LED_BLINK	P2OUT ^= GREEN_LED;


typedef enum
{
	RF24_ONLY = 0,
	BLT_MOUSE,
	BLT_JOYPAD
}PadType;

/**
 * RXD pin
 */
#define UART_RXD   		BIT1

/**
 * TXD pin
 */
#define UART_TXD   		BIT2

typedef struct ROSpberryRemote
{
	int16_t linear;
	int16_t steer;
	uint8_t buttons;

}RC_remote;


typedef struct Analog_properties
{
	int max;
	int min;
	int center;
	int deadzone;
}Analog;

#endif /* REMOTE_DEFINES_H_ */
