/*
 * LimitSwitchTest.c
 *
 * Created: 5/23/2017 4:48:39 PM
 * Author : Steven
 */ 

#include <avr/io.h>
#include "led.h"
#include "limitSwitch.h"


int main(void)
{
	led_init();
	limitSwitch_init();

    while (1) 
    {
		uint8_t outMask = 0;
		if(atLimitA0())
		{
			outMask |= 0x01;
		}
		if(atLimitB0())
		{
			outMask |= 0x02;
		}
		if(atLimitA1())
		{
			outMask |= 0x04;
		}
		if(atLimitB1())
		{
			outMask |= 0x08;
		}
		led_dataOut(outMask);
    }
}

