/*
 * blt_hid.h
 *
 *  Created on: 16/08/2016
 *      Author: joao
 */

#ifndef BLT_HID_H_
#define BLT_HID_H_

#include <msp430.h>
extern "C"
{
#include "conio/conio.h"
#include "timer_msp.h"
}
#include "remote_defines.h"
#include "stdint.h"

#define BLINK_TIMES		10

void setBLTHIDMouse(void);
void setBLTHIDJoypad(void);
void sendRawMouse(RC_remote r);
void sendRawJoypad(RC_remote r);
PadType getPadType(void);

#endif /* BLT_HID_H_ */
