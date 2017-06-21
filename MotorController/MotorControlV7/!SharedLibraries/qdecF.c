
#include <avr/io.h>
#include "qdecF.h"

#define DEGREES_PER_REV 360
#define CLOCK_CPU (uint32_t)32000000
#define PRESCALER 256

/* /// PERIPHERAL USAGE ///
 * TE0	qdec0 counter
 * TC1	freq0
 *
 * TF0	qdec1 counter
 * TD1	freq1
 */

/* /// PIN USAGE ///
 * PC4	QA0
 * PC5	QB0
 *
 * PD4	QA1
 * PD5	QB1
 */

static uint16_t cpr0 = 100;
static float lastVel0 = 0;

static uint16_t cpr1 = 100;
static float lastVel1 = 0;

void qdec_init(uint16_t cpr0p, uint16_t cpr1p)
{
	//Initializing qdec0
	cpr0 = cpr0p;
	PORTC.DIRCLR = PIN4_bm | PIN5_bm;

	// Set QDPH0 and QDPH1 sensing level.
	PORTCFG.MPCMASK = PIN4_bm | PIN5_bm;
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc;

	// Configure event
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN4_gc;
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;

	// Configure TC as a quadrature counter.
	TCE0.CTRLD = (uint8_t) TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
	TCE0.CTRLA = TC_CLKSEL_DIV1_gc;

	// Configure channel 2 to input pin for freq calculation.
	EVSYS.CH1MUX = EVSYS_CHMUX_PORTC_PIN4_gc;
	EVSYS.CH1CTRL = EVSYS_DIGFILT_4SAMPLES_gc;

	// Configure TC to capture frequency.
	TCC1.CTRLD = (uint8_t) TC_EVACT_FRQ_gc | TC_EVSEL_CH1_gc;
	TCC1.CTRLB = TC1_CCAEN_bm;
	TCC1.CTRLA = TC_CLKSEL_DIV256_gc;


	//Initializing qdec1 
	cpr1 = cpr1p;
	PORTD.DIRCLR = PIN4_bm | PIN5_bm;

	// Set QDPH0 and QDPH1 sensing level.
	PORTCFG.MPCMASK = PIN4_bm | PIN5_bm;
	PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_LEVEL_gc;

	// Configure event
	EVSYS.CH2MUX = EVSYS_CHMUX_PORTD_PIN4_gc;
	EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;

	// Configure TC as a quadrature counter.
	TCF0.CTRLD = (uint8_t) TC_EVACT_QDEC_gc | TC_EVSEL_CH2_gc;
	TCF0.CTRLA = TC_CLKSEL_DIV1_gc;

	// Configure channel 3 to input pin for freq calculation.
	EVSYS.CH3MUX = EVSYS_CHMUX_PORTD_PIN4_gc;
	EVSYS.CH3CTRL = EVSYS_DIGFILT_4SAMPLES_gc;

	// Configure TC to capture frequency.
	TCD1.CTRLD = (uint8_t) TC_EVACT_FRQ_gc | TC_EVSEL_CH3_gc;
	TCD1.CTRLB = TC1_CCAEN_bm;
	TCD1.CTRLA = TC_CLKSEL_DIV256_gc;

}

uint16_t qdec_pos0()
{
	return TCE0.CNT;
}
uint16_t qdec_pos1()
{
	return TCF0.CNT;
}

float qdec_vel0()
{
	if(TCC1.INTFLAGS & TC1_OVFIF_bm)
	{
		//TCC1.INTFLAGS |= TC1_OVFIF_bm;
		lastVel0 = 0;
	}
	else if(TCC1.INTFLAGS & TC1_CCAIF_bm)
	{
		lastVel0 = (DEGREES_PER_REV*(CLOCK_CPU/PRESCALER))/((float)cpr0*TCC1.CCA);
		if (TCE0.CTRLFSET & TC0_DIR_bm) //Yes, this should be TCE0 ('quadrature timer)
		{
			//We're going clockwise (negative)
			lastVel0*=-1;
		}
	}//else - No new velocity, use the last one

	return lastVel0;
}
float qdec_vel1()
{
	if(TCD1.INTFLAGS & TC1_OVFIF_bm)
	{
		lastVel1 = 0;
	}
	else if(TCD1.INTFLAGS & TC1_CCAIF_bm)
	{
		lastVel1 = (DEGREES_PER_REV*(CLOCK_CPU/PRESCALER))/((float)cpr1*TCD1.CCA);
		if (TCF0.CTRLFSET & TC0_DIR_bm) //Yes, this should be TCF0 (quadrature timer)
		{
			//We're going clockwise (negative)
			lastVel1*=-1;
		}
	}//else - No new velocity, use the last one

	return lastVel1;
}

uint8_t qdec_dir0()
{
	if (TCE0.CTRLFSET & TC0_DIR_bm)
	{
		return CW_DIR;
	}
	else
	{
		return CCW_DIR;
	}
}
uint8_t qdec_dir1()
{
	if (TCF0.CTRLFSET & TC0_DIR_bm)
	{
		return CW_DIR;
	}
	else
	{
		return CCW_DIR;
	}
}