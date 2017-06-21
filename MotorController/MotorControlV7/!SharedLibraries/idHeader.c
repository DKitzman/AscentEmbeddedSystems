#include <avr/io.h>
#include "idHeader.h"

void idHeader_init()
{
	PORTF.DIRCLR = 0x1F;
	//Setting pins 0-5 as totemPoll pullup
	PORTCFG.MPCMASK |= 0x1F;
	PORTF.PIN0CTRL = PORT_OPC_PULLUP_gc;
}

uint8_t idHeader_getValue()
{
	uint8_t headerVal = (~PORTF.IN) & 0x1F;
	return headerVal;
}