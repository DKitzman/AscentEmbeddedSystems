/*
 * WatchdogResetTest.c
 *
 * Created: 6/2/2017 6:48:03 PM
 * Author : Steven
 */ 

#define F_CPU 2000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "led.h"
#include "i2cAPI.h"


int main(void)
{
	led_init();
	i2cAPI_init(0x01);

	for(uint8_t i = 0; i < 6; i++)
	{
		led_cmdToggle();
		_delay_ms(200);
	}

    while (1)
    {
		i2cAPI_resetIfBusHogging();
		led_pingOn();
		_delay_ms(500);
		i2cAPI_resetIfBusHogging();
		led_pingOff();
		_delay_ms(500);
    }
}
