/*
 * i2cLedTest.c
 *
 * Created: 4/24/2017 3:10:23 PM
 * Author : Steven
 */ 

#define F_CPU 2000000UL
#define ADDRESS 0x06

#define CMD_PING_ON 0x01
#define CMD_PING_OFF 0x02
#define CMD_DATA 0x03

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "led.h"
#include "i2cAPI.h"


int main(void)
{
    led_init();
	i2cAPI_init(ADDRESS);
	
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
	
	uint8_t aliveCounter = 50;
	uint8_t cmdCounter = 0;

    while (1) 
    {
		i2cAPI_checkForPackets();
		if(i2cAPI_hasPacket())
		{
			struct packet receivedPacket;
			i2cAPI_getPacket(&receivedPacket);

			switch(receivedPacket.cmd)
			{
				case CMD_PING_ON:
					led_pingOn();
					break;
				case CMD_PING_OFF:
					led_pingOff();
					break;
				case CMD_DATA:
					led_cmdOn();
					cmdCounter = 10;
					led_dataOut(receivedPacket.buffer[0]);
					break;
				default:
					//Bad command
					break;
			}
		}

		aliveCounter--;
		if(aliveCounter == 0)
		{
			led_aliveToggle();
			aliveCounter = 50;
		}
		if(cmdCounter != 0)
		{
			cmdCounter--;
			if(cmdCounter == 0)
			{
				led_cmdOff();
			}
		}
		_delay_ms(10);
    }//end while(1)

}//end main()

