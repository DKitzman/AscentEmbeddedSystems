#include <avr/io.h>
#include "led.h"

#define CMD_MASK (1<<2)
#define PING_MASK (1<<1)
#define ALIVE_MASK (1<<0)

/* /// PIN USAGE ///
PORTA - data out
PB0 - ALIVE
PB1 - PING
PB2 - CMD
 */

void led_init()
{
	PORTA.OUTSET = 0xFF;
	PORTA.DIRSET = 0xFF;

	PORTB.OUTSET = 0x07;
	PORTB.DIRSET = 0x07;
}

void led_cmdOn()
{
	PORTB.OUTCLR = CMD_MASK;
}
void led_cmdOff()
{
	PORTB.OUTSET = CMD_MASK;
}
void led_cmdToggle()
{
	PORTB.OUTTGL = CMD_MASK;
}

void led_pingOn()
{
	PORTB.OUTCLR = PING_MASK;
}
void led_pingOff()
{
	PORTB.OUTSET = PING_MASK;
}
void led_pingToggle()
{
	PORTB.OUTTGL = PING_MASK;
}

void led_aliveOn()
{
	PORTB.OUTCLR = ALIVE_MASK;
}
void led_aliveOff()
{
	PORTB.OUTSET = ALIVE_MASK;
}
void led_aliveToggle()
{
	PORTB.OUTTGL = ALIVE_MASK;
}

void led_dataOut(uint8_t value)
{
	PORTA.OUT = ~value;
}
void led_bitSet(uint8_t bit)
{
	PORTA.OUTCLR = (1<<bit);
}
void led_bitClear(uint8_t bit)
{
	PORTA.OUTSET = (1<<bit);
}
void led_bitToggle(uint8_t bit)
{
	PORTA.OUTTGL = (1<<bit);
}