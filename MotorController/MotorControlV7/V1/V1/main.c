/*
 * V1.c
 *
 * Created: 5/10/2017 3:27:20 PM
 * Author : Steven
 */ 

#define F_CPU 32000000UL


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "led.h"
#include "motor16.h"
#include "qdecF.h"
#include "idHeader.h"
#include "i2cAPI.h"
#include "sysClock.h"
#include "limitSwitch.h"

#define ADDRESS_DEFAULT 0x04

#define CMD_PING_ON 0x01
#define CMD_PING_OFF 0x02
#define CMD_SET_VEL0 0x04
#define CMD_SET_VEL1 0x05

#define MAGNITUDE_LIMIT 4095
#define INTEGRAL_TERM_MAX 4095

//For convenience in Encoder array
#define NONE 100


void displaySpeed();
void updateM0(int16_t targetVelocity);
void updateM1(int16_t targetVelocity);
void displayCommands(int16_t target0, int16_t target1);


/*
Motor		Phase A	Phase B
ttable		Right	Left
shoulder	Down	Up
elbow		Down	Up
wristUD		Up		Down
wristLR		Right	Left
claw		Open	Close
*/

/* Current Data
			Encoder	Gearbox	Belt	Total
ttable		7		188		3		3948
shoulder	7		188		3		3948
elbow		7		188		3		3948
wristUD		7		188		3		3948
wristLR		7		60		NA		420
claw(temp)	48		?		NA		?
claw		48		46.85	NA		2249
*/

/*	Address					1		2		3		4		5		6		7
					blank	ttable	shoulderelbow	wristUD	wristLR	claw	noload	*/
//float kp0_A[] =		{0,		1,		1,		2.5,	0.2,	1,		1,		10		};
//float ki0_A[] =		{0,		3,		0,		1,		0.1,	3,		3,		30		};
//float kp0_B[] =		{0,		1,		0.5,	20,		0.2,	1,		1,		10		};
//float ki0_B[] =		{0,		3,		0,		5,		0.1,	3,		3,		30		};

//	Address			blank	1			2			3			4
//	usage					ttable		elbow		wristLR		noLoad
float kp0_A[] =		{0,		40,			20,			25,			20		};
float ki0_A[] =		{0,		0,			0,			0,			0		};
float kp0_B[] =		{0,		40,			60,			25,			20		};
float ki0_B[] =		{0,		0,			0,			0,			00		};
//	usage					shoulder	wristUD		claw		noLoad
float kp1_A[] =		{0,		30,			35,			40,			20		};
float ki1_A[] =		{0,		0,			0,			0,			00		};
float kp1_B[] =		{0,		30,			25,			40,			20		};
float ki1_B[] =		{0,		0,			0,			0,			00		};


//	Address			blank	1			2			3			4
//	usage					ttable		elbow		wristLR		noLoad
uint32_t cpr0[] =	{1,		3948,		3948,		NONE,		1316};
//	usage					shoulder	wristUD		claw		noLoad
uint32_t cpr1[] =	{1,		NONE,		3948,		NONE,		1316};

//	Address					blank	1					2					3				4
//	usage							ttable				elbow				wristLR			noLoad
uint8_t stopBehavior0[] =	{0,		STOP_SMOOTH,		STOP_BRAKE,			STOP_BRAKE,		STOP_SMOOTH};
//	usage							shoulder			wristUD				claw			noLoad
uint8_t stopBehavior1[] =	{0,		STOP_SMOOTH,		STOP_SMOOTH,		STOP_BRAKE,		STOP_SMOOTH};

static const float dt = 0.040; //delay in each loop
static uint8_t i2c_address;

int main(void)
{
	sysClock_set32();

	led_init();
	motor_init();
	idHeader_init();
	limitSwitch_init();

	i2c_address = ADDRESS_DEFAULT;
	if(idHeader_getValue() != 0)
	{
		i2c_address = idHeader_getValue();
	}

	qdec_init(cpr0[i2c_address], cpr1[i2c_address]);
	i2cAPI_init(i2c_address);
	led_dataOut(i2c_address);
	
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();

	for(uint8_t i = 0; i < 6; i ++)
	{
		led_aliveToggle();
		_delay_ms(100);
	}

	int16_t targetVelocity0 = 0;
	int16_t targetVelocity1 = 0;

	//Variables to track the led's
	uint8_t delayCounter = 25;
	uint8_t cmdCounter = 0;

	//The packet returned by the controller if requested
	struct packet returnPacket;
	returnPacket.cmd = 66;
	returnPacket.buffer[0] = 0;
	returnPacket.buffer[1] = 0;
	i2cAPI_setReturnPacket(&returnPacket, 3);

    while (1) 
    {
		i2cAPI_resetIfBusHogging();
		i2cAPI_checkForPackets();
		if(i2cAPI_hasPacket())
		{
			struct packet receivedPacket;
			i2cAPI_getPacket(&receivedPacket);

			switch(receivedPacket.cmd)
			{
				case CMD_SET_VEL0:
					led_cmdOn();
					cmdCounter = 2;
					targetVelocity0 = receivedPacket.buffer[0];
					if(receivedPacket.buffer[1] == 1)
					{
						targetVelocity0 *= -1;
					}
					break;
				case CMD_SET_VEL1:
					led_cmdOn();
					cmdCounter = 2;
					targetVelocity1 = receivedPacket.buffer[0];
					if(receivedPacket.buffer[1] == 1)
					{
						targetVelocity1 *= -1;
					}
				break;
				case CMD_PING_ON:
					led_pingOn();
					break;
				case CMD_PING_OFF:
					led_pingOff();
					break;
				default:
					break;
			}
		}//end if(i2cAPI_hasPacket())




		/// Begin Speed Control Code ///
		updateM0(targetVelocity0);
		updateM1(targetVelocity1);

		if(i2c_address == ADDRESS_DEFAULT)
		{
			//If no header, test the limit switches
			uint8_t outMask = 0;
			if(atLimitA0())
			{
				outMask |= 0x01;
			}
			if(atLimitB0())
			{
				outMask |= 0x02;
			}
			if(atLimitA1())
			{
				outMask |= 0x04;
			}
			if(atLimitB1())
			{
				outMask |= 0x08;
			}
			led_dataOut(outMask);
		}
		else
		{
				displayCommands(targetVelocity0, targetVelocity1);
		}
		

		returnPacket.cmd = 66;
		returnPacket.buffer[0] = (int8_t)qdec_vel0();
		returnPacket.buffer[1] = (int8_t)qdec_vel1();
		i2cAPI_setReturnPacket(&returnPacket, 2);



		/// LED Control ///
		delayCounter--;
		if(delayCounter == 0)
		{
			delayCounter = 25;
			led_aliveToggle();
		}

		if(cmdCounter != 0)
		{
			cmdCounter--;
			if(cmdCounter == 0)
			{
				led_cmdOff();
			}
		}

		//because the 12 bit PWM generator takes 0.12 ms to run,
		//make sure this delay is larger than that!
		//displaySpeed();
		//led_dataOut(qdec_vel1());
		_delay_ms(40);
	}
}

void updateM0(int16_t targetVelocity)
{
	static float integralTerm = 0;
	static float lastError = 0;

	//float currentVelocity = qdec_vel0();
	float currentVelocity = 0;
	float error = (float)targetVelocity - currentVelocity;
	integralTerm += error*dt;
	float derivativeTerm = (error - lastError) / dt;
	lastError = error;

	float kp;
	float ki;
	if(error > 0)
	{
		//We should use the "A" constants
		kp = kp0_A[i2c_address];
		ki = ki0_A[i2c_address];
	}
	else
	{
		//Using the "B" constants
		kp = kp0_B[i2c_address];
		ki = ki0_B[i2c_address];
	}

	//Prevent crazy windup
	if(integralTerm > INTEGRAL_TERM_MAX/ki)
	{
	integralTerm = INTEGRAL_TERM_MAX/ki;
	}
	else if (integralTerm < -INTEGRAL_TERM_MAX/ki)
	{
	integralTerm = -INTEGRAL_TERM_MAX/ki;
	}

	float kd = 0;
	int16_t magnitude = kp*error + kd*derivativeTerm + ki*integralTerm;


	
	//Filtering to acceptable 12 bit range
	if(magnitude > 4095)
	{
	magnitude = 4095;
	}
	else if (magnitude < -4095)
	{
	magnitude = -4095;
	}

	
	//make sure we are not trying to plow through a limit switch
	if((magnitude > 0) && atLimitA0())
	{
		magnitude = 0;
	}
	if((magnitude < 0) && atLimitB0())
	{
		magnitude = 0;
	}

	
	//int16_t valToDisplay = (int16_t)error;
	int16_t valToDisplay = magnitude;
	if(valToDisplay < 0)
	{
		valToDisplay *= -1;
		valToDisplay |= 0x80;
	}
	led_dataOut(valToDisplay);
	

	motor0_set(magnitude, stopBehavior0[i2c_address]);
}

void updateM1(int16_t targetVelocity)
{
	static float integralTerm = 0;

	//float currentVelocity = qdec_vel1();
	float currentVelocity = 0;
	float error = (float)targetVelocity - currentVelocity;
	integralTerm += error*dt;

	float kp;
	float ki;
	if(error > 0)
	{
		//We should use the "A" constants
		kp = kp1_A[i2c_address];
		ki = ki1_A[i2c_address];
	}
	else
	{
		//Using the "B" constants
		kp = kp1_B[i2c_address];
		ki = ki1_B[i2c_address];
	}

	//Prevent crazy windup
	if(integralTerm > INTEGRAL_TERM_MAX/ki)
	{
		integralTerm = INTEGRAL_TERM_MAX/ki;
	}
	else if (integralTerm < -INTEGRAL_TERM_MAX/ki)
	{
		integralTerm = -INTEGRAL_TERM_MAX/ki;
	}

	int16_t magnitude = kp*error + ki*integralTerm;


	
	//Filtering to acceptable 12 bit range
	if(magnitude > 4095)
	{
		magnitude = 4095;
	}
	else if (magnitude < -4095)
	{
		magnitude = -4095;
	}

	
	if((magnitude > 0) && atLimitA1())
	{
		magnitude = 0;
	}
	if((magnitude < 0) && atLimitB1())
	{
		magnitude = 0;
	}
	
	//int16_t valToDisplay = (int16_t)error;
	//int16_t valToDisplay = magnitude;
	//if(valToDisplay < 0)
	//{
	//	valToDisplay *= -1;
	//	valToDisplay |= 0x80;
	//}
	//led_dataOut(valToDisplay);

	motor1_set(magnitude, stopBehavior1[i2c_address]);
}

void displayCommands(int16_t target0, int16_t target1)
{
	if(target0 < 0)
	{
		target0 *= -1;
	}
	if(target1 < 0)
	{
		target1 *= -1;
	}

	uint8_t ledMask = 0;

	if(target0 > 0)
	{
		ledMask |= 0x01;
	}
	if(target0 > 40)
	{
		ledMask |= 0x02;
	}
	if(target0 > 80)
	{
		ledMask |= 0x04;
	}
	if(target0 > 120)
	{
		ledMask |= 0x08;
	}

	if(target1 > 0)
	{
		ledMask |= 0x10;
	}
	if(target1 > 40)
	{
		ledMask |= 0x20;
	}
	if(target1 > 80)
	{
		ledMask |= 0x40;
	}
	if(target1 > 120)
	{
		ledMask |= 0x80;
	}

	led_dataOut(ledMask);
}

void displaySpeed() 
{
	int16_t speed = qdec_vel1();
	if(speed < 0)
	{
		//led_cmdOn();
		speed*=-1;
	}
	else
	{
		//led_cmdOff();
	}

	uint8_t ledMask = 0;
	if(speed > 0)
	{
		ledMask |= 0x01;
	}
	if(speed > 25)
	{
		ledMask |= 0x02;
	}
	if(speed > 50)
	{
		ledMask |= 0x04;
	}
	if(speed > 75)
	{
		ledMask |= 0x08;
	}
	if(speed > 100)
	{
		ledMask |= 0x10;
	}
	if(speed > 125)
	{
		ledMask |= 0x20;
	}
	if(speed > 150)
	{
		ledMask |= 0x40;
	}
	if(speed > 175)
	{
		ledMask |= 0x80;
	}
	led_dataOut(ledMask);
}

