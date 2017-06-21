/*
 * motorDriveTest.c
 *
 * Created: 4/26/2017 3:57:09 PM
 * Author : Steven
 */ 

#define F_CPU 2000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "motor.h"
#include "led.h"
#include "sysClock.h"

int main(void)
{
	motor_init();
	led_init();
	//sysClock_set32();
	/*
	led_dataOut(1);
	motor0_set(127);
	_delay_ms(3000);
	for(int i = 0; i < 10; i++)
	{
		led_cmdToggle();
		_delay_ms(100);
	}
	led_dataOut(2);
	motor0_set(-127);

	*/
	//HARDCORE TEST
    while (1) 
    {
		led_dataOut(1);
		motor0_set(127);
		//motor1_set(64);
		_delay_ms(1000);

		led_dataOut(2);
		motor0_set(-127);
		//motor1_set(-64);
		_delay_ms(1000);
    }
}

