/*
 * selectionHeaderTest.c
 *
 * Created: 4/26/2017 12:02:10 PM
 * Author : Steven
 */ 

#include <avr/io.h>
#include "led.h"
#include "idHeader.h"


int main(void)
{
	led_init();
	idHeader_init();

	uint8_t headerVal = idHeader_getValue();

	led_dataOut(headerVal);

    while (1) 
    {
    }
}

