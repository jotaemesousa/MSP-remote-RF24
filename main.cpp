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


//#define MSP430_SERIAL_DEBUG
extern "C"
{
#include "spi.h"
#include "conio/conio.h"
#include "serial/serial.h"
#include "timer_msp.h"
}


// PORT 2
#define L1_BUTTON	0x02
#define R1_BUTTON	0x04
#define L2_BUTTON	0x10
#define R2_BUTTON	0x20

// PORT 2
#define RED_LED		0x80
#define GREEN_LED	0x40



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
void read_buttons(uint8_t &buttons);
struct timer_msp subtract(struct timer_msp a, struct timer_msp b);
void refresh_activity(void);

struct timer_msp timer_sleep;
unsigned int ADC_values[4];
bool calibrated = false;
bool remote_on = true;
bool activity = true;
unsigned long int last_blink_millis = 0;
Analog analog_left, analog_right;
struct timer_msp activity_time;

// main loop
int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT~
	BCSCTL1 = CALBC1_1MHZ;            // Set DCO to 1MHz
	DCOCTL = CALDCO_1MHZ;

	unsigned long int last_millis = 0;
	default_timer();


#ifdef MSP430_SERIAL_DEBUG
	serial_init(9600);
#endif
	// Setup ADC
	setup_adc();
	setup_power();
#ifdef MSP430_SERIAL_DEBUG
	cio_printf("Init- ADC\n");
#endif

	// Setup push buttons
	setup_push_buttons();
	setup_leds();

	__bis_SR_register(GIE);       // Enter LPM0, interrupts enabled

	// ____________________________________________________________
	RC_remote ferrari;
	ferrari.steer = 0;
	ferrari.linear = 0;
	//ferrari.buttons = 0;

	RF24 radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

	// Setup and configure rf radio
	radio.begin();

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	radio.setPayloadSize(sizeof(ferrari));

	radio.setDataRate(RF24_250KBPS);

	// Open pipes to other nodes for communication
	radio.openWritingPipe(pipes[0]);
	radio.openReadingPipe(1,pipes[1]);

	// Start listening
	radio.startListening();

	// Dump the configuration of the rf unit for debugging
	radio.printDetails();

	P2OUT |= GREEN_LED;

	analog_left.deadzone = 15;
	analog_right.deadzone = 15;

	// calibrate remote
	calibrate();

	while(remote_on)
	{
		// First, scan potentiometers
		uint8_t buttons;
		adc_sample(ADC_values);
		read_buttons(buttons);



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

			//send car controll cmd
			radio.write(&ferrari, sizeof(ferrari));

			//__bis_SR_register(CPUOFF + GIE);

		}
		refresh_activity();
	}



	return 0;


}


void setup_adc(void)
{
	// Scan P1.3 and P1.4

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
	activity = true;
	activity_time.s = timer0.s;
	activity_time.ms = timer0.ms;

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
			P2OUT ^= RED_LED;
		}

		if(analog_left.max - analog_left.center > 50 && analog_right.max - analog_right.center > 50 &&
				analog_left.center - analog_left.min > 50 && analog_right.center - analog_right.min > 50)
		{
			P2OUT |= GREEN_LED;
			calibrated = true;
		}
		else
		{
			P2OUT &= ~GREEN_LED;
		}

		adc_sample(ADC_values);

		if(ADC_values[ADC_ANALOG_LEFT] > analog_left.max)
		{
			analog_left.max = ADC_values[ADC_ANALOG_LEFT];
		}
		if(ADC_values[ADC_ANALOG_RIGHT] > analog_right.max)
		{
			analog_right.max = ADC_values[ADC_ANALOG_RIGHT];
		}
		if(ADC_values[ADC_ANALOG_LEFT] < analog_left.min)
		{
			analog_left.min = ADC_values[ADC_ANALOG_LEFT];
		}
		if(ADC_values[ADC_ANALOG_RIGHT] < analog_right.min)
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
	P2OUT &= ~(RED_LED + GREEN_LED);
}

void read_buttons(uint8_t &buttons)
{
	buttons = 0x00;
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
	d = adc[ADC_ANALOG_LEFT] - analog_left.center;
	s = adc[ADC_ANALOG_RIGHT] - analog_right.center;
	RC_cmd->linear = 0;
	RC_cmd->steer = 0;

	if(d < analog_left.deadzone && d > -analog_left.deadzone)
	{
		RC_cmd->linear = 0;
	}
	else if(d >= analog_left.max - analog_left.center)
	{
		RC_cmd->linear = 127;
		activity = true;
	}
	else if(d < 0 && d > analog_left.min - analog_left.center)
	{
		RC_cmd->linear = (-d * 127)/ (analog_left.min - analog_left.center);
		activity = true;
	}
	else if(d > 0 && d < analog_left.max - analog_left.center)
	{
		RC_cmd->linear = (d * 127)/ (analog_left.max - analog_left.center);
		activity = true;
	}
	else if(d <= (analog_left.min - analog_left.center))
	{
		RC_cmd->linear = -127;
		activity = true;
	}

	if(s < analog_right.deadzone && s > -analog_right.deadzone)
	{
		RC_cmd->steer = 0;
	}
	else if(s >= analog_right.max - analog_right.center)
	{
		RC_cmd->steer = 127;
		activity = true;
	}
	else if(s < 0 && s >= analog_right.min - analog_right.center)
	{
		RC_cmd->steer = (-s * 127)/ (analog_right.min - analog_right.center);
		activity = true;
	}
	else if(s > 0 && s <= analog_right.max - analog_right.center)
	{
		RC_cmd->steer = (s * 127)/ (analog_right.max - analog_right.center);
		activity = true;
	}
	else if(s <= analog_right.min - analog_right.center)
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
		activity_time.s = timer0.s;
		activity_time.ms = timer0.ms;
		activity = false;
	}
	else
	{
		struct timer_msp temp;
		temp = subtract(timer0, activity_time);

		if(temp.s > 120)
		{
			power_off();
			//__bis_SR_register(LPM4_bits);
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

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(LPM0_bits + GIE);        // Clear CPUOFF bit from 0(SR)
}


