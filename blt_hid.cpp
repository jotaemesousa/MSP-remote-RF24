/*
 * blt_hid.cpp
 *
 *  Created on: 16/08/2016
 *      Author: joao
 */

#include "blt_hid.h"

void delayMS(unsigned long msec)
{
	while(msec--)
	{
		__delay_cycles(1000);
	}
}

void printByte(uint8_t *array, uint8_t len)
{
	for (uint8_t var = 0; var < len; ++var)
	{
		cio_printc((char)array[var]);
	}
}

void setBLTHIDMouse(void)
{
	cio_print((char *)"$$$");
	delayMS(10000);
	cio_print((char *)"SN,JMoteS_Mouse\r\n");
	delayMS(1000);
	cio_print((char *)"SH,0220\r\n");
	delayMS(1000);
	cio_print((char *)"R,1\r\n");
	delayMS(10000);
	for (int var = 0; var < BLINK_TIMES; ++var)
	{
		RED_LED_BLINK
		delayMS(1000);
	}
}

void setBLTHIDJoypad(void)
{
	cio_print((char *)"$$$");
	delayMS(10000);
	cio_print((char *)"SN,JMoteS_JoyPad\r\n");
	delayMS(1000);
	cio_print((char *)"SH,0240\r\n");
	delayMS(1000);
	cio_print((char *)"R,1\r\n");
	delayMS(10000);
	for (int var = 0; var < BLINK_TIMES; ++var)
	{
		GREEN_LED_BLINK
		delayMS(1000);
	}

}

void sendRawMouse(RC_remote r)
{
	uint8_t msg[7];
	int8_t an;
	msg[0] = 0xFD;
	msg[1] = 0x05;
	msg[2] = 0x02;
	msg[3] = r.buttons & 0x0F;
	an = r.steer & 0xFF;
	msg[4] = an;
	an = -r.linear & 0xFF;
	msg[5] = an;
	msg[6] = 0;
	printByte(msg, 7);
}

void sendRawJoypad(RC_remote r)
{
	uint8_t msg[8];
	int8_t an;
	msg[0] = 0xFD;
	msg[1] = 0x06;
	an = r.steer & 0xFF;
	msg[2] = an;
	an = -r.linear & 0xFF;
	msg[3] = an;
	an = r.steer & 0xFF;
	msg[4] = an;
	an = -r.linear & 0xFF;
	msg[5] = an;
	msg[6] = r.buttons & 0x0F;
	msg[7] = 0;

	printByte(msg, 8);
}

PadType getPadType(void)
{
	if((~P2IN & L1_BUTTON) == L1_BUTTON)
	{
		return BLT_JOYPAD;
	}
	else if((~P2IN & R1_BUTTON) == R1_BUTTON)
	{
		return BLT_MOUSE;
	}

	return RF24_ONLY;
}
