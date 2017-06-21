/*
 * qdecTest.c
 *
 * Created: 4/26/2017 5:01:06 PM
 * Author : Steven
 */ 
#define F_CPU 2000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "qdec.h"
#include "led.h"


//From testing: If qdec_velN() returns negative, qdec_dirN() returns true

int main(void)
{

	qdec_init(2400, 2400);
	led_init();

    while (1) 
    {
		
		if(qdec_dir0())
		{
			led_aliveOn();
		}
		else
		{
			led_aliveOff();
		}

		//led_dataOut(qdec_pos0());
		
	
		int16_t speed = qdec_vel0();
		if(speed < 0)
		{
			led_cmdOn();
			speed*=-1;
		}
		else
		{
			led_cmdOff();
		}

		uint8_t ledMask = 0;
		if(speed > 0)
		{
			ledMask |= 0x01;
		}
		if(speed > 50)
		{
			ledMask |= 0x02;
		}
		if(speed > 100)
		{
			ledMask |= 0x04;
		}
		if(speed > 150)
		{
			ledMask |= 0x08;
		}
		if(speed > 200)
		{
			ledMask |= 0x10;
		}
		if(speed > 250)
		{
			ledMask |= 0x20;
		}
		if(speed > 300)
		{
			ledMask |= 0x40;
		}
		if(speed > 350)
		{
			ledMask |= 0x80;
		}
		led_dataOut(ledMask);
    }
}

