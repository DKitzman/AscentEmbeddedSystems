#include <avr/io.h>
#include "motor16.h"

#define PHASE_A 0
#define PHASE_B 1

/* Uses TimerC0 and TimerD0
 */
void motor_init()
{
	//Initializing Motor0
	PORTC.DIRSET = 0x0F;
	TCC0.CTRLB = TC_WGMODE_DS_T_gc; //Chose 'top' arbitrarily
	TCC0.CTRLB |= TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCC0.PER = 0x0FFF; //Using 12 bits
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;

	//Initializing Motor1
	PORTD.DIRSET = 0x0F;
	TCD0.CTRLB = TC_WGMODE_DS_T_gc;
	TCD0.CTRLB |= TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 0x0FFF; //Using 12 bits
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
}

void motor0_set(int16_t value,  uint8_t stopBehavior)
{
	static uint8_t lastDir = PHASE_A;
	uint8_t smoothStop = 0;
	uint8_t brakeStop = 0;
	if(value == 0)
	{
		if(stopBehavior == STOP_SMOOTH)
		{
			smoothStop = 1;
		}
		else if(stopBehavior == STOP_BRAKE)
		{
			brakeStop = 1;
		}
	}
	if(brakeStop)
	{
		PORTC.OUTCLR = PIN2_bm | PIN0_bm; //Disable BHI and BHI

		TCC0.CCB = TCC0.PER;	//Hold ALO HIGH
		TCC0.CCD = TCC0.PER;	//Hold BLO HIGH
		TCC0.CNT = TCC0.PER;
	}
	else if(value > 0 || (smoothStop && (lastDir == PHASE_A))  )
	{
		//Commanded Phase A
		lastDir = PHASE_A;
		if(value > 4095)
		{
			value = 4095;
		}

		PORTC.OUTCLR = PIN2_bm; //Disable BHI
		TCC0.CCB = 0;			//Hold ALO low
		TCC0.CNT = TCC0.PER;

		uint16_t pwmVal = (uint16_t)value;
		TCC0.CCD = pwmVal;		//Set BLO pwm
		TCC0.CNT = TCC0.PER;
		PORTC.OUTSET = PIN0_bm; //Set AHI
	}
	else
	{
		//Commanded Phase B
		lastDir = PHASE_B;
		if(value < -4095)
		{
			value = -4095;
		}
		value*=-1;

		PORTC.OUTCLR = PIN0_bm;	//Clear AHI
		TCC0.CCD = 0;			//Hold BLO low
		TCC0.CNT = TCC0.PER;

		uint16_t pwmVal = (uint16_t)value;
		TCC0.CCB = pwmVal;		//Set ALO pwm
		TCC0.CNT = TCC0.PER;
		PORTC.OUTSET = PIN2_bm;
	}

}

void motor1_set(int16_t value, uint8_t stopBehavior)
{
	static uint8_t lastDir = PHASE_A;
	uint8_t smoothStop = 0;
	uint8_t brakeStop = 0;
	if(value == 0)
	{
		if(stopBehavior == STOP_SMOOTH)
		{
			smoothStop = 1;
		}
		else if(stopBehavior == STOP_BRAKE)
		{
			brakeStop = 1;
		}
	}

	if(brakeStop)
	{
		PORTD.OUTCLR = PIN2_bm | PIN0_bm; //Disable BHI and AHI

		TCD0.CCB = TCD0.PER;		//Hold ALO high
		TCD0.CCD = TCD0.PER;		//Hold BLO high
		TCD0.CNT = TCD0.PER;
	}
	else if(value > 0 || (smoothStop && (lastDir == PHASE_A))  )
	{
		//Commanded Phase A
		lastDir = PHASE_A;
		if(value > 4095)
		{
			value = 4095;
		}
		PORTD.OUTCLR = PIN2_bm; //Disable BHI
		TCD0.CCB = 0;			//Hold ALO low
		TCD0.CNT = TCD0.PER;

		uint16_t pwmVal = (uint16_t)value;
		TCD0.CCD = pwmVal;		//Set BLO pwm
		TCD0.CNT = TCD0.PER;
		PORTD.OUTSET = PIN0_bm; //Set AHI
	}
	else
	{
		//Commanded Phase B
		lastDir = PHASE_B;
		if(value < -4095)
		{
			value = -4095;
		}
		value*=-1;

		PORTD.OUTCLR = PIN0_bm;	//Clear AHI
		TCD0.CCD = 0;			//Hold BLO low
		TCD0.CNT = TCD0.PER;

		uint16_t pwmVal = (uint16_t)value;
		TCD0.CCB = pwmVal;		//Set ALO pwm
		TCD0.CNT = TCD0.PER;
		PORTD.OUTSET = PIN2_bm;
	}
}
