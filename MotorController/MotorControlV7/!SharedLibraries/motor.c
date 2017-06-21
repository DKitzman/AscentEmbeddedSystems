#include <avr/io.h>
#include "motor.h"

/* Uses TimerC0 and TimerD0
 */
void motor_init()
{
	//Initializing Motor0
	PORTC.DIRSET = 0x0F;
	TCC0.CTRLB = TC_WGMODE_DS_T_gc; //Chose 'top' arbitrarily
	TCC0.CTRLB |= TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCC0.PER = 0xFF; //Shortening to 1 byte
	TCC0.CTRLA = TC_CLKSEL_DIV8_gc;

	//Initializing Motor1
	PORTD.DIRSET = 0x0F;
	TCD0.CTRLB = TC_WGMODE_DS_T_gc;
	TCD0.CTRLB |= TC0_CCBEN_bm | TC0_CCDEN_bm;
	TCD0.PER = 0xFF; //Shortening to 1 byte
	TCD0.CTRLA = TC_CLKSEL_DIV8_gc;
}

void motor0_set(int8_t value)
{
	if(value > 0)
	{
		//Commanded Phase A
		PORTC.OUTCLR = PIN2_bm; //Disable BHI
		TCC0.CCB = 0;			//Hold ALO low
		TCC0.CNT = TCC0.PER;

		uint8_t pwmVal = (uint8_t)value*2;
		TCC0.CCD = pwmVal;		//Set BLO pwm
		TCC0.CNT = TCC0.PER;
		PORTC.OUTSET = PIN0_bm; //Set AHI
	}
	else
	{
		//Commanded Phase B
		if(value == -128)
		{
			value = -127;
		}
		value*=-1;

		PORTC.OUTCLR = PIN0_bm;	//Clear AHI
		TCC0.CCD = 0;			//Hold BLO low
		TCC0.CNT = TCC0.PER;

		uint8_t pwmVal = (uint8_t)value*2;
		TCC0.CCB = pwmVal;		//Set ALO pwm
		TCC0.CNT = TCC0.PER;
		PORTC.OUTSET = PIN2_bm;
	}
}

void motor1_set(int8_t value)
{
	if(value > 0)
	{
		//Commanded Phase A
		PORTD.OUTCLR = PIN2_bm; //Disable BHI
		TCD0.CCB = 0;			//Hold ALO low
		TCD0.CNT = TCD0.PER;

		uint8_t pwmVal = (uint8_t)value*2;
		TCD0.CCD = pwmVal;		//Set BLO pwm
		TCD0.CNT = TCD0.PER;
		PORTD.OUTSET = PIN0_bm; //Set AHI
	}
	else
	{
		//Commanded Phase B
		if(value == -128)
		{
			value = -127;
		}
		value*=-1;

		PORTD.OUTCLR = PIN0_bm;	//Clear AHI
		TCD0.CCD = 0;			//Hold BLO low
		TCD0.CNT = TCD0.PER;

		uint8_t pwmVal = (uint8_t)value*2;
		TCD0.CCB = pwmVal;		//Set ALO pwm
		TCD0.CNT = TCD0.PER;
		PORTD.OUTSET = PIN2_bm;
	}
}
