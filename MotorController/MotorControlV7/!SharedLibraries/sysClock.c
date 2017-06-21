#include <avr/io.h>

//I stole this from Atmel so I don't have to do this myself
/*! \brief CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the timecritial
 *  operation of writing to the registers.
 *
 *  \param address A pointer to the address to write to.
 *  \param value   The value to put in to the register.
 */
static void CCPWrite( volatile uint8_t * address, uint8_t value )
{
#ifdef __ICCAVR__

	// Store global interrupt setting in scratch register and disable interrupts.
        asm("in  R1, 0x3F \n"
	    "cli"
	    );

	// Move destination address pointer to Z pointer registers.
	asm("movw r30, r16");
#ifdef RAMPZ
	asm("ldi  R16, 0 \n"
            "out  0x3B, R16"
	    );

#endif
	asm("ldi  r16,  0xD8 \n"
	    "out  0x34, r16  \n"
#if (__MEMORY_MODEL__ == 1)
	    "st     Z,  r17  \n");
#elif (__MEMORY_MODEL__ == 2)
	    "st     Z,  r18  \n");
#else // (__MEMORY_MODEL__ == 3) || (__MEMORY_MODEL__ == 5)
	    "st     Z,  r19  \n");
#endif // __MEMORY_MODEL__ 

	// Restore global interrupt setting from scratch register.
        asm("out  0x3F, R1");

#elif defined __GNUC__
	//AVR_ENTER_CRITICAL_REGION( );
	uint8_t volatile saved_sreg = SREG;
	SREG &= ~((uint8_t)CPU_I_bm);
	//cli();
	volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
		);

	//AVR_LEAVE_CRITICAL_REGION( );
	SREG = saved_sreg;
#endif
}

void sysClock_set32()
{
	OSC.CTRL |= OSC_RC32MEN_bm;
	CCPWrite( &CLK.PSCTRL, ( (uint8_t) CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc));
	//Wait for 32 MHz clock to startup
	while(!(OSC.STATUS & OSC_RC32MRDY_bm)){}
	//Select 32 MHz clock as main clock source
	CCPWrite( &CLK.CTRL, CLK_SCLKSEL_RC32M_gc);

}