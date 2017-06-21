
#include <avr/io.h>
#include "limitSwitch.h"

/* /// PERIPHERAL USAGE ///
 * None
 */

/* /// PIN USAGE ///
 * PE4	SW_A0
 * PE5	SW_B0
 *
 * PE6	SW_A1
 * PE7	SW_B1
 */

void limitSwitch_init()
{
	PORTE.DIRCLR = 0xF0;
	//Setting Pins 4-7 as totemPoll pullup
	PORTCFG.MPCMASK |= 0xF0;
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;
}

uint8_t atLimitA0()
{
	return (~PORTE.IN) & PIN4_bm;
}
uint8_t atLimitB0()
{
	return (~PORTE.IN) & PIN5_bm;
}

uint8_t atLimitA1()
{
	return (~PORTE.IN) & PIN6_bm;
}
uint8_t atLimitB1()
{
	return (~PORTE.IN) & PIN7_bm;
}
