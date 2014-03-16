/*
 * main.cpp
 *
 *  Created on: May 6, 2013
 *      Author: joao
 */

//debug
//#define MSP430_SERIAL_DEBUG
#include "spi_msp430.h"

#include "RF24.h"
#include <msp430.h>
#include "remote_defines.h"
#include "flash.h"
#include <math.h>

extern "C"
{
#include "spi.h"
#include "conio/conio.h"
#include "serial/serial.h"
#include "timer_msp.h"
}


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


// Function prototypes
void setup_adc(void);
void adc_sample(unsigned int *ADC_ptr);
void convert_values(RC_remote *RC_cmd, unsigned int *adc);
void setup_push_buttons(void);
void setup_leds(void);
void setup_power(void);
void power_on(void);
void power_off(void);
void calibrate(void);
void read_buttons(uint8_t &buttons, RC_remote &remote);
//struct timer_msp subtract(struct timer_msp a, struct timer_msp b);
void refresh_activity(void);
void write_calibration_to_flash(Analog p1, Analog p2);
void read_calibration_from_flash(Analog &p1, Analog &p2);
int16_t map_value(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max, bool trunc);

struct timer_msp timer_sleep;
unsigned int ADC_values[4];
bool calibrated = false;
bool remote_on = true;
bool activity = true;
unsigned long int last_blink_millis = 0;
Analog analog_left, analog_right;
unsigned long int activity_time;
unsigned long int switch_off_button_timer;
bool start_list = 0;
bool send_request= 0;

// main loop
int main(void)
{
	//WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT~
	WDTCTL = WDT_MDLY_32;                     // Set Watchdog Timer interval to ~30ms
	IE1 |= WDTIE;

	BCSCTL1 = CALBC1_8MHZ;            // Set DCO to 1MHz
	DCOCTL = CALDCO_8MHZ;

	// setup power pin
	setup_power();
	GREEN_LED_ON;

	unsigned long int last_millis = 0;

	// initialize timer
	default_timer();

	// initialize flash
	flash_init();

#ifdef MSP430_SERIAL_DEBUG
	serial_init(9600);
#endif

	// Setup ADC
	setup_adc();

#ifdef MSP430_SERIAL_DEBUG
	cio_printf("Init- ADC\n");
#endif

	// Setup push buttons
	setup_push_buttons();

	// Setup LEDs
	setup_leds();

	// interrupts enabled
	__bis_SR_register(GIE);

	// ____________________________________________________________
	RC_remote ferrari;
	ferrari.steer = 0;
	ferrari.linear = 0;
	ferrari.buttons = 0;

	RF24 radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[3] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

	// Setup and configure rf radio
	radio.begin();

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	radio.setPayloadSize(sizeof(RC_remote));

	radio.setDataRate(RF24_250KBPS);

	// Open pipes to other nodes for communication
	radio.openWritingPipe(pipes[0]);
	radio.openReadingPipe(1,pipes[1]);

	// Start listening
	radio.startListening();

	// Dump the configuration of the rf unit for debugging
	radio.printDetails();

	analog_left.deadzone = 15;
	analog_right.deadzone = 15;

	read_calibration_from_flash(analog_left, analog_right);

	GREEN_LED_ON;
	delay(100);
	GREEN_LED_OFF;

	while(1)
	{

		// First, scan potentiometers
		uint8_t buttons;
		adc_sample(ADC_values);
		read_buttons(buttons, ferrari);
		ferrari.buttons = buttons;

		if(buttons == 0x0F)			// All buttons pressed...
		{
			// Then, stop listening so we can talk.
			radio.startListening();
			radio.stopListening();

			// calibrate remote
			calibrate();
			write_calibration_to_flash(analog_left, analog_right);
		}

		if( millis()- last_millis > SEND_MSG_TIME)
		{
			last_millis = millis();
#ifdef MSP430_SERIAL_DEBUG
			cio_printf("%i %i %i %i\n", ADC_values[0], ADC_values[1], ADC_values[2], ADC_values[3]);
#endif

			// Convert values
			convert_values(&ferrari, ADC_values);
#ifdef MSP430_SERIAL_DEBUG
			cio_printf("%i %i \n", ferrari.linear, ferrari.steer);
#endif
			// Then, stop listening so we can talk.
			radio.startListening();
			radio.stopListening();

			//send car control cmd
			if(send_request)
			{
				send_request = 0;
				start_list = 1;
				ferrari.buttons |= ASK_BIT;
			}
			radio.write(&ferrari, sizeof(RC_remote));

		}

		if(start_list)
		{
			start_list = 0;
			radio.startListening();

			__bis_SR_register(GIE);
			// Wait here until we get a response, or timeout (250ms)
			bool timeout = false;
			last_millis = millis();

			while(!radio.available() && !timeout)
			{

				if(millis() - last_millis > 100)
				{
					timeout = true;
				}

			}

#ifdef MSP430_SERIAL_DEBUG
			cio_printf("%i\n", millis());
#endif
			if(!timeout)
			{
				uint8_t response;
				radio.read( &response, sizeof(uint8_t) );

				if(response)
				{
					RED_LED_ON;
				}
				else
				{
					RED_LED_OFF;
				}
			}
			radio.stopListening();
		}

		// update activity timer
		refresh_activity();
	}



	return 0;


}


void setup_adc(void)
{
	// Scan P1.3 and P1.0

	ADC10CTL1 = INCH_3 + CONSEQ_3;
	ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;
	ADC10DTC1 = 0x04;
	ADC10AE0 |= 0x09;
}

void setup_push_buttons(void)
{
	P2DIR &= ~(L1_BUTTON + R1_BUTTON + L2_BUTTON + R2_BUTTON);
	P2OUT |= (L1_BUTTON + R1_BUTTON + L2_BUTTON + R2_BUTTON);
	P2REN |= (L1_BUTTON + R1_BUTTON + L2_BUTTON + R2_BUTTON);
}

void setup_leds(void)
{
	P2SEL &= ~(RED_LED + GREEN_LED);
	P2SEL2 &= ~(RED_LED + GREEN_LED);
	P2DIR |= RED_LED + GREEN_LED;
	P2OUT &= ~(RED_LED + GREEN_LED);
}

// Power functions
void setup_power(void)
{
	POWER_DIR |= POWER_PIN;
	power_on();
}
void power_on(void)
{
	POWER_PORT |= POWER_PIN;
}
void power_off(void)
{
	POWER_PORT &= ~POWER_PIN;
}

void calibrate(void)
{
	GREEN_LED_OFF;
	RED_LED_OFF;
	activity = true;
	activity_time = millis();
	calibrated = false;
	adc_sample(ADC_values);

	// initial position
	analog_left.center = (int)ADC_values[ADC_ANALOG_LEFT];
	analog_right.center = (int)ADC_values[ADC_ANALOG_RIGHT];
	analog_left.max = (int)ADC_values[ADC_ANALOG_LEFT];
	analog_right.max = (int)ADC_values[ADC_ANALOG_RIGHT];
	analog_left.min = (int)ADC_values[ADC_ANALOG_LEFT];
	analog_right.min = (int)ADC_values[ADC_ANALOG_RIGHT];

	// wait until L2 and R2 are clear
	while(((L2_BUTTON  & ~P2IN) == L2_BUTTON) || ((R2_BUTTON  & ~P2IN) == R2_BUTTON))
	{
		refresh_activity();
	}

	// wait until L1 and R1 are set
	while((((L1_BUTTON + R1_BUTTON) & ~P2IN) != (L1_BUTTON + R1_BUTTON)) || !calibrated )
	{
		if(millis() - last_blink_millis > 500)
		{
			last_blink_millis = millis();
			RED_LED_BLINK;
		}

		if(analog_left.max - analog_left.center > 50 && analog_right.max - analog_right.center > 50 &&
				analog_left.center - analog_left.min > 50 && analog_right.center - analog_right.min > 50)
		{
			GREEN_LED_ON;
			calibrated = true;
		}
		else
		{
			GREEN_LED_OFF;
		}

		adc_sample(ADC_values);

		if(ADC_values[ADC_ANALOG_LEFT] > (unsigned int)analog_left.max)
		{
			analog_left.max = ADC_values[ADC_ANALOG_LEFT];
		}
		if(ADC_values[ADC_ANALOG_RIGHT] > (unsigned int)analog_right.max)
		{
			analog_right.max = ADC_values[ADC_ANALOG_RIGHT];
		}
		if(ADC_values[ADC_ANALOG_LEFT] < (unsigned int)analog_left.min)
		{
			analog_left.min = ADC_values[ADC_ANALOG_LEFT];
		}
		if(ADC_values[ADC_ANALOG_RIGHT] < (unsigned int)analog_right.min)
		{
			analog_right.min = ADC_values[ADC_ANALOG_RIGHT];
		}
		refresh_activity();
	}

	adc_sample(ADC_values);
	analog_left.center = ADC_values[ADC_ANALOG_LEFT];
	analog_right.center = ADC_values[ADC_ANALOG_RIGHT];

	// wait until L1 and R1 are clear
	while(((L1_BUTTON & ~P2IN) == L1_BUTTON) || ((R1_BUTTON & ~P2IN) == R1_BUTTON))
	{
		refresh_activity();
	}

	//	analog_left.min -= 5;
	//	analog_right.max -= 5;
	//	analog_left.min += 5;
	//	analog_right.min += 5;

	GREEN_LED_OFF;
	RED_LED_OFF;
}

void read_buttons(uint8_t &buttons, RC_remote &remote)
{
	buttons = 0x00;
	static uint8_t switch_off_button_on = 0;
	static uint8_t ready_to_power_off= 0;

	if((~P2IN & L1_BUTTON) == L1_BUTTON)
	{
		buttons |= 0x01;
		activity = true;
	}
	if((~P2IN & L2_BUTTON) == L2_BUTTON)
	{
		buttons |= 0x02;
		activity = true;
	}

	if((~P2IN & R1_BUTTON) == R1_BUTTON)
	{
		buttons |= 0x04;
		activity = true;
	}
	if((~P2IN & R2_BUTTON) == R2_BUTTON)
	{
		buttons |= 0x08;
		activity = true;

		if(!switch_off_button_on)
		{
			switch_off_button_timer = millis();
			ready_to_power_off = 1;
		}
		switch_off_button_on = 1;
	}
	else
	{
		switch_off_button_on = 0;
	}

	// switch off procedure
	if(ready_to_power_off)
	{
		if(fabs(remote.linear) <= 20 && fabs(remote.steer) <= 20 && switch_off_button_on &&
				(P2IN & (L1_BUTTON | L2_BUTTON | R1_BUTTON)) == (L1_BUTTON | L2_BUTTON | R1_BUTTON))
		{
			if(millis() - switch_off_button_timer > 2500)
			{
				power_off();
			}
		}
	}
	else
	{
		switch_off_button_timer = millis();
		ready_to_power_off = 0;
	}
}

void adc_sample( unsigned int *ADC_ptr)
{
	ADC10CTL0 &= ~ENC;

	while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active

	ADC10SA = (unsigned int)ADC_ptr;     	// Data buffer start

	ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
	__bis_SR_register(LPM0_bits + GIE);        			// LPM0, ADC10_ISR will force exit
}
void convert_values(RC_remote *RC_cmd, unsigned int *adc)
{
	int d,s;
	d = adc[ADC_ANALOG_LEFT];
	s = adc[ADC_ANALOG_RIGHT];
	RC_cmd->linear = 0;
	RC_cmd->steer = s;

	int16_t d_centered = adc[ADC_ANALOG_LEFT] - analog_left.center;
	int16_t s_centered = adc[ADC_ANALOG_RIGHT] - analog_right.center;

	if(d_centered < analog_left.deadzone && d_centered > -analog_left.deadzone)
	{
		RC_cmd->linear = 0;
	}
	else if(d_centered >= analog_left.max - analog_left.center)
	{
		RC_cmd->linear = 127;
		activity = true;
	}
	else if(d_centered < 0 && d_centered > analog_left.min - analog_left.center)
	{
		RC_cmd->linear = map_value(d, analog_left.min, (analog_left.center - analog_left.deadzone), -127, 0, false);
		//RC_cmd->linear = (-d * 127)/ (analog_left.min - analog_left.center);
		activity = true;
	}
	else if(d_centered > 0 && d_centered < analog_left.max - analog_left.center)
	{
		RC_cmd->linear = map_value(d, analog_left.center + analog_left.deadzone, analog_left.max, 0, 127, false);
		//RC_cmd->linear = (d * 127)/ (analog_left.max - analog_left.center);
		activity = true;
	}
	else if(d_centered <= (analog_left.min - analog_left.center))
	{
		RC_cmd->linear = -127;
		activity = true;
	}

	if(s_centered < analog_right.deadzone && s_centered > -analog_right.deadzone)
	{
		RC_cmd->steer = 0;
	}
	else if(s_centered >= analog_right.max - analog_right.center)
	{
		RC_cmd->steer = 127;
		activity = true;
	}
	else if(s_centered < 0 && s_centered >= analog_right.min - analog_right.center)
	{
		RC_cmd->steer = map_value(s, analog_right.min, (analog_right.center - analog_right.deadzone), -127, 0, false);
		//RC_cmd->steer = (-s * 127)/ (analog_right.min - analog_right.center);
		activity = true;
	}
	else if(s_centered > 0 && s_centered <= analog_right.max - analog_right.center)
	{
		RC_cmd->steer = map_value(s, analog_right.center + analog_right.deadzone, analog_right.max, 0, 127, false);
		//RC_cmd->steer = (s * 127)/ (analog_right.max - analog_right.center);
		activity = true;
	}
	else if(s_centered <= analog_right.min - analog_right.center)
	{
		RC_cmd->steer = -127;
		activity = true;
	}

#ifdef MSP430_SERIAL_DEBUG
	cio_printf("%i %i \n", RC_cmd->linear, RC_cmd->steer);
#endif
}

void refresh_activity(void)
{
	if(activity)
	{
		activity_time = millis();
		activity = false;
	}
	else
	{
		if(millis() - activity_time > 20000)
		{
			power_off();
		}
	}
}

struct timer_msp subtract( struct timer_msp a, struct timer_msp b)
{
	struct timer_msp temp;
	temp.s = a.s - b.s;
	temp.ms = a.ms - b.ms;
	if(temp.ms < 0)
	{
		temp.ms = -temp.ms;
		temp.s--;
	}
	return temp;
}

void write_calibration_to_flash(Analog p1, Analog p2)
{
	unsigned char temp[12];
	temp[0] = p1.min & 0xFF;
	temp[1] = (p1.min & 0xFF00) >> 8;
	temp[2] = p1.center & 0xFF;
	temp[3] = (p1.center & 0xFF00) >> 8;
	temp[4] = p1.max & 0xFF;
	temp[5] = (p1.max & 0xFF00) >> 8;

	temp[6] = p2.min & 0xFF;
	temp[7] = (p2.min & 0xFF00) >> 8;
	temp[8] = p2.center & 0xFF;
	temp[9] = (p2.center & 0xFF00) >> 8;
	temp[10] = p2.max & 0xFF;
	temp[11] = (p2.max & 0xFF00) >> 8;

	write_SegC(temp, 12);
}

void read_calibration_from_flash(Analog &p1, Analog &p2)
{
	unsigned char frase[12];
	read_SegC(frase, 12, 0);
	int temp_value;

	temp_value = frase[1];
	temp_value = temp_value << 8;
	temp_value |= frase[0];
	p1.min = temp_value;

	temp_value = frase[3];
	temp_value = temp_value << 8;
	temp_value |= frase[2];
	p1.center = temp_value;

	temp_value = frase[5];
	temp_value = temp_value << 8;
	temp_value |= frase[4];
	p1.max = temp_value;

	temp_value = frase[7];
	temp_value = temp_value << 8;
	temp_value |= frase[6];
	p2.min = temp_value;

	temp_value = frase[9];
	temp_value = temp_value << 8;
	temp_value |= frase[8];
	p2.center = temp_value;

	temp_value = frase[11];
	temp_value = temp_value << 8;
	temp_value |= frase[10];
	p2.max = temp_value;
}

int16_t map_value(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max, bool trunc)
{
	if(trunc)
	{
		int16_t temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
		if(temp > out_max) temp = out_max;
		if(temp < out_min) temp = out_min;
		return temp;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
	__bic_SR_register_on_exit(LPM0_bits + GIE);        // Clear CPUOFF bit from 0(SR)
}

// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
	static int c = 0;
	c++;
	if(c > 264)
	{
		c = 0;
		send_request = 1;
		GREEN_LED_BLINK
	}
}

