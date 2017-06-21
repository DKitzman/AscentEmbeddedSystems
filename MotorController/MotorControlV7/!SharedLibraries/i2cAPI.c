#include <avr/interrupt.h>
#include "twi_slave_driver.h"
#include "i2cAPI.h"
#include <util/delay.h>


#define START 0xAB
#define STOP 0xCD

#define BYTE_ARRAY_SIZE		64
#define PACKET_ARRAY_SIZE	16

#define RECEIVE_ARRAY_SIZE  20

struct queueData
{
	uint8_t volatile head;
	uint8_t volatile tail;
	uint8_t volatile numData;
	uint8_t volatile dataArray[BYTE_ARRAY_SIZE];
	uint8_t volatile overflowFlag;
};

struct packetQueueData
{
	uint8_t head;
	uint8_t tail;
	uint8_t numData;
	struct packet packetArray[PACKET_ARRAY_SIZE];
	uint8_t overflowFlag;
};



// Global variables
TWI_Slave_t twiSlave;      // TWI slave module
static volatile struct queueData byteQueue;
static struct packetQueueData packetQueue;

// TWIE Slave Interrupt vector.
ISR(TWIE_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

/********************
 * Static functions
 ********************/

//Places data at head, increments head
static void queue_enQueue(uint8_t data)
{
	byteQueue.dataArray[byteQueue.head] = data;
	byteQueue.head++;
	if(byteQueue.head == BYTE_ARRAY_SIZE)
	{
		byteQueue.head = 0;
	}

	if(byteQueue.numData < BYTE_ARRAY_SIZE)
	{
		byteQueue.numData++;
	}
	else
	{
		byteQueue.overflowFlag = 1;
		byteQueue.tail++;
		if(byteQueue.tail == BYTE_ARRAY_SIZE)
		{
			byteQueue.tail = 0;
		}
	}
}

//This is what happens whenever a new byte is received 
static void TWIC_SlaveProcessData(void)
{
	//PORTD.OUT = twiSlave.receivedData[twiSlave.bytesReceived];
	queue_enQueue(twiSlave.receivedData[twiSlave.bytesReceived]);
}



//Returns data at tail, increments tail
static uint8_t byteQueue_deQueue()
{
	if(byteQueue.numData == 0)
	{
		return 0;
	}
	byteQueue.numData--;

	uint8_t dataToReturn = byteQueue.dataArray[byteQueue.tail];
	byteQueue.tail++;
	if(byteQueue.tail == BYTE_ARRAY_SIZE)
	{
		byteQueue.tail = 0;
	}

	return dataToReturn;
}

static uint8_t i2c_getByte()
{
	return byteQueue_deQueue();
}

//returns how many items are in the queue
static volatile uint8_t i2c_hasData()
{
	return byteQueue.numData;
}

static void byteQueue_flushQueue()
{
	//Re-Initialize byteQueueVariables
	byteQueue.head = 0;
	byteQueue.tail = 0;
	byteQueue.numData = 0;
	byteQueue.overflowFlag = 0;
}

//Currently this code never resorts to this
//Commented out to avoid compiler warning
//static void packetQueue_flushQueue()
//{
//	packetQueue.head = 0;
//	packetQueue.tail = 0;
//	packetQueue.numData = 0;
//	packetQueue.overflowFlag = 0;
//}

static uint8_t i2c_GetByteWithTimeout(uint8_t *destination, uint8_t timeout_ms)
{
	while (timeout_ms != 0)
	{
		if(byteQueue.numData)
		{
			*destination = byteQueue_deQueue();
			return 1;
		}
		timeout_ms--;
		_delay_ms(1);
	}
	return 0;
}

//Used for resetIfBusHogging()
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

/************************
 * Non-static functions *
 ************************/

void i2cAPI_init(uint8_t addressP)
{
	//Initialize TWI slave on Port E
	TWI_SlaveInitializeDriver(&twiSlave, &TWIE, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, addressP, TWI_SLAVE_INTLVL_LO_gc);
}

void i2cAPI_checkForPackets()
{
	//uint8_t pathTrace = 0;
	//uint8_t timedOut = 0;
	if(i2c_hasData())
	{
		//pathTrace++;
		uint8_t success = 0;
		uint8_t receiveArray[RECEIVE_ARRAY_SIZE];
		for(int i = 0; i < RECEIVE_ARRAY_SIZE; i++)
		{
			receiveArray[i] = 0x00;
		}
		while(i2c_hasData() && (receiveArray[0] != START) )
		{
			receiveArray[0] = i2c_getByte();
		}
		if(receiveArray[0] == START)
		{
			//pathTrace++;
			uint8_t length;
			if(i2c_GetByteWithTimeout(&length, 10))
			{
				//pathTrace++;
				receiveArray[1] = length;
				if(length <= ( RECEIVE_ARRAY_SIZE-2 ) )
				{
					//pathTrace++;
					int i;
					for(i = 2; i < length+2; i++)
					{
						if(!i2c_GetByteWithTimeout(&receiveArray[i], 10))
						{
							//Timed out!
							//timedOut = 1;
							break;
						}
					}
					if(i == length+2)
					{
						//pathTrace++;
						if(receiveArray[length+1] == STOP)
						{
							//pathTrace++;
							//Calculate parity
							uint8_t parityByte = 0;
							for(int j = 1; j < length; j++)
							{
								parityByte ^= receiveArray[j];
							}

							if(parityByte == receiveArray[length])
							{
								//Packet is good!
								success = 1;
								packetQueue.packetArray[packetQueue.head].cmd = receiveArray[2];

								for(int i = 3; i < length; i++)
								{
									packetQueue.packetArray[packetQueue.head].buffer[i-3] = receiveArray[i];
								}

								packetQueue.head++;
								if(packetQueue.head == PACKET_ARRAY_SIZE)
								{
									packetQueue.head = 0;
								}

								if(packetQueue.numData < PACKET_ARRAY_SIZE)
								{
									packetQueue.numData++;
								}
								else
								{
									packetQueue.overflowFlag = 1;
									packetQueue.tail++;
									if(packetQueue.tail == PACKET_ARRAY_SIZE)
									{
										packetQueue.tail = 0;
									}
								}

							}
							//else the parity doesn't match!
						}
						//else the array doesn't end with a stop byte!
					}
					//else one of the bytes timed out!
				}
				//The length byte was too long!
			}
			//else the length byte timed out!
		}
		//else the first byte was NOT the start Byte!
		if(!success)
		{
			byteQueue_flushQueue();
		}
	}
}

uint8_t i2cAPI_hasPacket()
{
	return packetQueue.numData;
}

uint8_t i2cAPI_getPacket(struct packet *packetDestination)
{
	if(packetQueue.numData == 0)
	{
		return 0;
	}

	*packetDestination = packetQueue.packetArray[packetQueue.tail];

	packetQueue.tail++;
	if(packetQueue.tail == PACKET_ARRAY_SIZE)
	{
		packetQueue.tail = 0;
	}
	packetQueue.numData--;
	return 1;
}

void i2cAPI_setReturnPacket(const struct packet * const packetPointer, uint8_t numDataBytes)
{
	twiSlave.sendData[0] = START;
	twiSlave.sendData[1] = numDataBytes+3;
	twiSlave.sendData[2] = packetPointer->cmd;
	uint8_t i;
	for(i = 3; i < numDataBytes+3; i++)
	{
		twiSlave.sendData[i] = packetPointer->buffer[i-3];
	}

	uint8_t parityByte = 0;
	parityByte ^= twiSlave.sendData[1];
	parityByte ^= twiSlave.sendData[2];
	for(i = 3; i < numDataBytes+3; i++)
	{
		parityByte ^= twiSlave.sendData[i];
	}

	twiSlave.sendData[i] = parityByte;
	twiSlave.sendData[i+1] = STOP;
}

void i2cAPI_resetIfBusHogging()
{
	uint16_t us_countdown = 1000;
	while( (us_countdown != 0) && !(PORTE.IN & PIN0_bm) )
	{
		us_countdown--;
		_delay_us(1);
	}
	if(us_countdown == 0)
	{
		//reset our shit
		CCPWrite(&RST.CTRL, RST_SWRST_bm);
		while(1){}
	}
}