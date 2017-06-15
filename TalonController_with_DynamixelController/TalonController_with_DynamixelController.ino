//This code is meant to control 6 talon motor controllers
//and 2-3 Dynamixel servos
#define F_CPU 32000000

#include <Wire.h>
#include <avr/wdt.h>
#define I2C_ADDRESS 8
#define START 0xAB
#define STOP 0xCD
#define TEMP_ARRAY_SIZE 20
static uint8_t address_g = 1;

#define PING_ON 0x01
#define PING_OFF 0x02
#define CMD_SET_SPEED 0x04
#define CMD_DYNAMIXEL_SET_POSITION 0X05
#define CMD_DYNAMIXEL_SET_LED 0X06
#define CMD_DYNAMIXEL_SET_SCIENCE_POSITION 0x08
#define CMD_DYNAMIXEL_SCIENCE_CLOSE 0x0A
#define CMD_DYNAMIXEL_SCIENCE_OPEN 0x0B
#define CMD_DYNAMIXEL_SET_SCIENCE_LED 0x09

// Pin Numbering //
#define TALON_0 11
#define TALON_1 10
#define TALON_2 9
#define TALON_3 6
#define TALON_4 5
#define TALON_5 3

#define DYNAMIXEL_UD_ADDRESS 0
#define DYNAMIXEL_LR_ADDRESS 1
#define DYNAMIXEL_SCIENCE_ADDRESS 2

// Dynamixel Command Defines //
#define AX12A_TX_BUFFER_LENGTH 32
#define AX12A_RX_BUFFER_LENGTH 32

#define CMD_PING    0x01
#define CMD_READ_DATA 0x02
#define CMD_WRITE_DATA  0x03
#define CMD_REG_WRITE 0x04
#define CMD_ACTION    0x05
#define CMD_RESET   0x06
#define CMD_SYNC_WRITE  0x83

#define ADDR_ID     0x03
#define ADDR_BAUD_RATE  0x04
#define ADDR_CW_ANGLE_LIMIT 0x06
#define ADDR_CCW_ANGLE_LIMIT 0x08
#define ADDR_LED    0x19
#define ADDR_GOAL_POSITION  0x1E
#define ADDR_MOVING_SPEED 0x20
#define ADDR_PRESENT_POSITION_L 0x24
#define ADDR_PRESENT_POSITION_H 0x25

#define BROADCASTING_ID 0xFE

#define LED_ON      1
#define LED_OFF     0

#define DIR_CCW     0
#define DIR_CW      1

// Dynamixel Addresses //
/*
 * Addr   Position
 * 1      Camera Left/Right
 * 2      Camera Up/Down
 * 3      Science?
 */

uint8_t TALON[] = {TALON_0, TALON_1, TALON_2, TALON_3, TALON_4, TALON_5};


#define SCIENCE_INDEX 2
#define DYNAMIXEL_UD_INDEX 0
#define DYNAMIXEL_LR_INDEX 1
//                    UP/DOWN       LEFT/RIGHT
uint8_t DYNAMIXEL[] = {DYNAMIXEL_UD_ADDRESS, DYNAMIXEL_LR_ADDRESS, DYNAMIXEL_SCIENCE_ADDRESS};
uint16_t DYNAMIXEL_MAX[] = {1023, 1023, 1023};
uint16_t DYNAMIXEL_MIN[] = {0, 0, 0};
uint16_t DYNAMIXEL_CENTER[] = {0x1ff, 0x299, 0x1ff};

#define SCIENCE_POS_CLOSED 204
#define SCIENCE_POS_OPEN 336

#define FIRST_TALON_NUM 0
#define NUM_TALONS 6

#define NUM_DYNAMIXELS 3

#define NUM_DYNAMIXELS_CAMERA_MAST 2

#define PWM_NEUTRAL 187
#define PWM_FORWARD 250
#define PWM_REVERSE 125

#define DIR_FORWARD 0
#define DIR_REVERSE 1

#define FOR_EACH_TALON for(int i = FIRST_TALON_NUM; i < (FIRST_TALON_NUM + NUM_TALONS); i++)

#define MECHANICAL_COMPENSATION 1.16

struct packet
{
  uint8_t cmd;
  uint8_t buffer[15];
};



#define RECEIVE_ARRAY_SIZE 128
volatile uint8_t receiveArray[RECEIVE_ARRAY_SIZE];
volatile uint8_t head = 0;
volatile uint8_t tail = 0;
volatile uint8_t bytesReceived = 0;

#define PACKET_ARRAY_SIZE 32
packet packetArray[PACKET_ARRAY_SIZE];
uint8_t head_packetArray = 0;
uint8_t tail_packetArray = 0;
uint8_t packetsReady = 0;

// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();
  return;
}


void i2c_checkForBusHogging()
{
  uint16_t timeout = 1000;
  //SDA is on PIN C4
  while (  (timeout != 0) && !(PINC & (1 << PINC4))  )
  {
    timeout--;
    _delay_us(1);
  }

  if (timeout == 0)
  {
    wdt_enable(WDTO_15MS);
    while (1) {};

    //led_dataOut(0x0F);  //Debug, show off that this has happened
  }
}
void checkForPackets()
{
  i2c_checkForBusHogging();
  if (bytesReceived)
  {
    uint8_t success = 0;
    uint8_t tempArray[TEMP_ARRAY_SIZE];
    for (int i = 0; i < TEMP_ARRAY_SIZE; i++)
    {
      tempArray[i] = 0x00;
    }
    while ((bytesReceived > 0) && (tempArray[0] != START) )
    {
      tempArray[0] = dequeue();
    }
    if (tempArray[0] == START)
    {
      uint8_t packetLength;
      if (dequeueWithTimeout(&packetLength, 10))
      {
        tempArray[1] = packetLength;
        if (packetLength <= ( TEMP_ARRAY_SIZE - 2 ) )
        {
          int i;
          for (i = 2; i < packetLength + 2; i++)
          {
            if (!dequeueWithTimeout(&tempArray[i], 10))
            {
              //Timed out!
              break;
            }
          }
          if (i == packetLength + 2)
          {
            if (tempArray[packetLength + 1] == STOP)
            {
              //Calculate parity
              uint8_t parityByte = 0;
              for (int j = 1; j < packetLength; j++)
              {
                parityByte ^= tempArray[j];
              }

              if (parityByte == tempArray[packetLength])
              {
                //Packet is good!
                success = 1;
                packetArray[head_packetArray].cmd = tempArray[2];

                for (int i = 3; i < packetLength; i++)
                {
                  packetArray[head_packetArray].buffer[i - 3] = tempArray[i];
                }

                head_packetArray++;
                if (head_packetArray == PACKET_ARRAY_SIZE)
                {
                  head_packetArray = 0;
                }

                if (packetsReady < PACKET_ARRAY_SIZE)
                {
                  packetsReady++;
                }
                else
                {
                  tail_packetArray++;
                  if (tail_packetArray == PACKET_ARRAY_SIZE)
                  {
                    tail_packetArray = 0;
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
    if (!success)
    {
      head = 0;
      tail = 0;
      bytesReceived = 0;
    }
  }
}

uint8_t dequeueWithTimeout(uint8_t *destination, uint8_t timeout_ms)
{
  while (timeout_ms != 0)
  {
    if (bytesReceived)
    {
      *destination = dequeue();
      return 1;
    }
    timeout_ms--;
    delay(1);
  }
  return 0;
}
void enqueue(uint8_t dataByte)
{
  receiveArray[head] = dataByte;
  head++;
  if (head == RECEIVE_ARRAY_SIZE)
  {
    head = 0;
  }
  if (bytesReceived == RECEIVE_ARRAY_SIZE)
  {
    tail++;
    if (tail == RECEIVE_ARRAY_SIZE)
    {
      tail = 0;
    }
  }
  else
  {
    bytesReceived++;
  }
}

void dequeuePacket(struct packet* destination)
{
  if (packetsReady == 0)
  {
    return;
  }
  *destination = packetArray[tail_packetArray];
  tail_packetArray++;
  if (tail_packetArray == PACKET_ARRAY_SIZE)
  {
    tail_packetArray = 0;
  }
  packetsReady--;
}

uint8_t dequeue()
{
  if (bytesReceived == 0)
  {
    return 0;
  }
  uint8_t toReturn = receiveArray[tail];
  tail++;
  if (tail == RECEIVE_ARRAY_SIZE)
  {
    tail = 0;
  }
  bytesReceived--;
  return toReturn;
}

void receiveEvent(int param)
{
  while (Wire.available())
  {
    enqueue(Wire.read());
  }
}
void setTalonSpeedFromPacket(struct packet* speedPacket)
{
  if (speedPacket->cmd != CMD_SET_SPEED)
  {
    return;
  }
  uint8_t talonIndex = FIRST_TALON_NUM;
  for (int l = 0; l < 12; l += 2)
  {
    /*
    Serial.print("Setting Talon [");
    Serial.print(talonIndex);
    Serial.print("] to [");
    Serial.print(speedPacket->buffer[l]);
    Serial.print(", ");
    Serial.print(speedPacket->buffer[l+1]);
    Serial.println("]");
    */
    setTalonSpeed(TALON[talonIndex], speedPacket->buffer[l], speedPacket->buffer[l + 1]);
    talonIndex++;
  }
}
void setTalonSpeed(uint8_t talonNum, uint8_t value, uint8_t dir)
{
  uint8_t pwmVal = PWM_NEUTRAL;
  float tempVal;
  switch (talonNum)
  {
  case 11:
  case 10:
  case 9:
    //No need to reverse direction
    tempVal = ((float)value) * MECHANICAL_COMPENSATION; //Need to speed this side up because mechanical fucked up
    if (tempVal > 255)
    {
      tempVal = 255;
    }
    value = tempVal;
    //value = ((float)value)*1.5; //Need to speed this side up because mechanical fucked up
    //No need to reverse direction
    break;
  case 6:
  case 5:
  case 3:
    //These motors are hooked up backward, reverse the commanded direction!
    if (dir == DIR_FORWARD)
    {
      dir = DIR_REVERSE;
    }
    else
    {
      dir = DIR_FORWARD;
    }
    break;
  }

  if (dir == DIR_FORWARD)
  {
    pwmVal = (value / 255.0) * (PWM_FORWARD - PWM_NEUTRAL) + PWM_NEUTRAL;
  }
  else
  {
    pwmVal = PWM_NEUTRAL - (value / 255.0) * (PWM_NEUTRAL - PWM_REVERSE);
  }

  setPWM(talonNum, pwmVal);
}

void setPWM(uint8_t talonNum, uint8_t value)
{
  switch (talonNum)
  {
  case 5:
    OCR0B = value;
    break;
  case 6:
    OCR0A = value;
    break;
  case 11:
    OCR2A = value;
    break;
  case 10:
    OCR1BL = value;
    break;
  case 9:
    OCR1AL = value;
    break;
  case 3:
    OCR2B = value;
    break;
  default:
    break;
  }

}

void setupTalons()
{
  FOR_EACH_TALON
  {
    int talonNum = TALON[i];
    switch (talonNum)
    {
    case 5:
      DDRD |= _BV(DDD5);
      break;
    case 6:
      DDRD |= _BV(DDD6);
      break;
    case 11:
      DDRB |= _BV(DDB3);
      break;
    case 10:
      DDRB |= _BV(DDB2);
      break;
    case 9:
      DDRB |= _BV(DDB1);
      break;
    case 3:
      DDRD |= _BV(DDD3);
      break;
    default:
      break;
    }
  }

  //Need to manually configure the Timer0 to get the right pulse length
  TCCR0A = _BV(WGM00) | _BV(COM0A1) | _BV(COM0B1); //Phase Correct PWM, Clear while upcounting
  TCCR0B = _BV(CS00) | _BV(CS01); // Set prescaler 64

  //Setting Timer1
  TCCR1A = _BV(WGM10) | _BV(COM1A1) | _BV(COM1B1); //8Bit phase correct PWM, clear while upcounting;
  TCCR1B = _BV(CS10) | _BV(CS11); // Set Prescaler 64

  //Setting Timer2
  TCCR2A = _BV(WGM20) | _BV(COM2A1) | _BV(COM2B1); //Phase Correct PWM
  TCCR2B = _BV(CS22);

  FOR_EACH_TALON
  {
    uint8_t talonNum = TALON[i];
    setTalonSpeed(talonNum, 0, DIR_FORWARD);
  }

}

void setupDynamixels()
{
  for (int l = 0; l < NUM_DYNAMIXELS_CAMERA_MAST; l++)
  {
    AX12A_setPositionRotation(DYNAMIXEL[l], DYNAMIXEL_MAX[l], DYNAMIXEL_MIN[l]);
  }
  AX12A_setPositionRotation(DYNAMIXEL_SCIENCE_ADDRESS, SCIENCE_POS_OPEN, SCIENCE_POS_CLOSED);

  for (int j = 0; j < 3; j++)
  {
    for (int l = 0; l < NUM_DYNAMIXELS; l++)
    {
      AX12A_led(DYNAMIXEL[l], LED_ON);
    }
    delay(200);
    for (int l = 0; l < NUM_DYNAMIXELS; l++)
    {
      AX12A_led(DYNAMIXEL[l], LED_OFF);
    }
    delay(200);
  }
  centerCameraMastDynamixels();
  AX12A_setGoalPosition(DYNAMIXEL_SCIENCE_ADDRESS, SCIENCE_POS_CLOSED);
}



void setDynamixelPositionFromPacket(struct packet* dynaPacket)
{
  if (dynaPacket->cmd != CMD_DYNAMIXEL_SET_POSITION)
  {
    return;
  }

  uint16_t goalPositionUD = ((uint16_t)dynaPacket->buffer[0]) << 8 | dynaPacket->buffer[1];
  AX12A_setGoalPosition(DYNAMIXEL[DYNAMIXEL_UD_INDEX], goalPositionUD);

  uint16_t goalPositionLR = ((uint16_t)dynaPacket->buffer[2]) << 8 | dynaPacket->buffer[3];
  AX12A_setGoalPosition(DYNAMIXEL[DYNAMIXEL_LR_INDEX], goalPositionLR);

  return;
}

void setDynamixelLedFromPacket(struct packet* dynaPacket)
{
  if (dynaPacket->cmd != CMD_DYNAMIXEL_SET_LED)
  {
    return;
  }

  AX12A_led(DYNAMIXEL[DYNAMIXEL_UD_INDEX], dynaPacket->buffer[0]);
  AX12A_led(DYNAMIXEL[DYNAMIXEL_LR_INDEX], dynaPacket->buffer[1]);

  return;
}

void setScienceDynamixelPosition(struct packet* dynaPacket)
{
  uint16_t goalPosition = (uint16_t)(dynaPacket->buffer[0]) << 8 | dynaPacket->buffer[1];
  AX12A_setGoalPosition(DYNAMIXEL[SCIENCE_INDEX], goalPosition);
  return;
}

void setDynamixelScienceLedFromPacket(struct packet* dynaPacket)
{
  AX12A_led(DYNAMIXEL[SCIENCE_INDEX], dynaPacket->buffer[0]);
}

void centerCameraMastDynamixels()
{
  AX12A_setGoalPosition(DYNAMIXEL[DYNAMIXEL_UD_INDEX], DYNAMIXEL_CENTER[DYNAMIXEL_UD_INDEX]);
  AX12A_setGoalPosition(DYNAMIXEL[DYNAMIXEL_LR_INDEX], DYNAMIXEL_CENTER[DYNAMIXEL_LR_INDEX]);
}

void AX12A_setPositionRotation(uint8_t ID, uint16_t maxPosition, uint16_t minPosition)
{
  uint8_t parameters[3];
  parameters[0] = ADDR_CCW_ANGLE_LIMIT;
  parameters[1] = maxPosition;
  parameters[2] = maxPosition >> 8;
  AX12A_sendCommand(ID, CMD_WRITE_DATA, parameters, 3);
  parameters[0] = ADDR_CW_ANGLE_LIMIT;
  parameters[1] = minPosition;
  parameters[2] = minPosition >> 8;
  AX12A_sendCommand(ID, CMD_WRITE_DATA, parameters, 3);
}

void AX12A_setGoalPosition(uint8_t ID, uint16_t position)
{
  uint8_t highByte = (uint8_t)(position >> 8);
  uint8_t lowByte = (uint8_t)position;

  uint8_t parameters[3];
  parameters[0] = ADDR_GOAL_POSITION;
  parameters[1] = lowByte;
  parameters[2] = highByte;
  AX12A_sendCommand(ID, CMD_WRITE_DATA, parameters, 3);
}

void AX12A_setBaud(uint8_t ID, uint16_t desiredBaud)
{
  uint8_t dataValue = (2000000 / desiredBaud) - 1;
  uint8_t parameters[2];
  parameters[0] = ADDR_BAUD_RATE;
  parameters[1] = dataValue;
  AX12A_sendCommand(ID, CMD_WRITE_DATA, parameters, 2);
}

void AX12A_led(uint8_t ID, uint8_t ledState)
{
  uint8_t ledTemp = ledState;
  if (ledTemp != LED_ON && ledTemp != LED_OFF)
  {
    return;
  }
  uint8_t parameters[2];
  parameters[0] = ADDR_LED;
  parameters[1] = ledTemp;
  AX12A_sendCommand(ID, CMD_WRITE_DATA, parameters, 2);
}

void AX12A_setID(uint8_t oldID, uint8_t newID)
{
  uint8_t numParams = 2;
  uint8_t parameters[numParams];
  parameters[0] = ADDR_ID;
  parameters[1] = newID;

  AX12A_sendCommand(oldID, CMD_WRITE_DATA, parameters, numParams);
}

void AX12A_sendCommand(uint8_t ID, uint8_t command,
                       uint8_t *cmdParameters, uint8_t parameterLength)
{
  uint8_t txBuffer[32];
  txBuffer[0] = 0xFF;
  txBuffer[1] = 0xFF;
  txBuffer[2] = ID;
  txBuffer[3] = parameterLength + 2;
  txBuffer[4] = command;

  uint8_t i;
  for (i = 0; i < parameterLength; i++)
  {
    txBuffer[i + 5] = cmdParameters[i];
  }
  uint8_t packetLength = parameterLength + 4 + 2; //Length of entire packet
  uint8_t checksum = 0;
  for (i = 2; i < packetLength - 1; i++)
  {
    checksum += txBuffer[i];
  }

  txBuffer[packetLength - 1] = ~checksum;
  //TODO Maybe do some weird transmit?

  //Transmit Data
  for (i = 0; i < packetLength; i++)
  {
    //TODO their code clears something weird
    //TODO consider disabling interrupts before the last transmisson
    Serial.write(txBuffer[i]);
    //Serial.print(txBuffer[i]);
    //Serial.print(" ");
  }
  //Serial.println();
}

/*
int16_t AX12A_getPosition(uint8_t ID)
{
  uint8_t numParams = 2;
  uint8_t parameters[numParams];
  //0x24 Present Position(L)
  //0x25 Present Position(H)
  parameters[0] = ADDR_PRESENT_POSITION_L;
  parameters[1] = 2; //Read low and high bytes

  AX12A_sendCommand(ID, CMD_READ_DATA, parameters, numParams);

  uint8_t returnPacket[8];
  int i;
  for(i = 0; i < 8; i++)
  {
    //Serial.print("Waiting for data ... ");
    while(!Serial.available())
    {
      //wait for next byte
    }
    returnPacket[i] = Serial.read();
    Serial.print("Read data: ");
    Serial.write(returnPacket[i]);
    Serial.println();
  }
  Serial.write(0xBEEF);
  while(1)
  {
    //Get stuck!
  }
  return 0;
}
*/


//#define INCLUDE_SERIAL

void setup()
{
  pinMode(13, OUTPUT);

  Serial.begin(1000000);
  while (!Serial)
  {
    //Do nothing
    ;
  }


  setupTalons();
  delay(3000); //Wait for Dynamixels to be powered
  setupDynamixel  s();

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);


  /*
  FOR_EACH_TALON
  {
    setPWM(TALON[i], 255);
    delay(250);
    setPWM(TALON[i], 0);
    delay(250);
  }
  */
  /*
  FOR_EACH_TALON
  {
    setPWM(TALON[i], PWM_REVERSE);
    //analogWrite(TALON[i], 128);
  }
  */
}

void loop()
{

  checkForPackets();
  if (packetsReady > 0)
  {
    struct packet currentPacket;
    dequeuePacket(&currentPacket);

    /*
        #ifdef INCLUDE_SERIAL
        Serial.print("Packet CMD: ");
        Serial.print(currentPacket.cmd);
        Serial.print(" Data: [");
        for(int k = 0; k < 15; k++)
        {
          Serial.print(currentPacket.buffer[k]);
          Serial.print(", ");
        }
        Serial.println("]");
        #endif
     */
    switch (currentPacket.cmd)
    {
    case CMD_SET_SPEED:
      setTalonSpeedFromPacket(&currentPacket);
      break;
    case CMD_DYNAMIXEL_SET_POSITION:
      setDynamixelPositionFromPacket(&currentPacket);
      break;
    case CMD_DYNAMIXEL_SET_LED:
      setDynamixelLedFromPacket(&currentPacket);
      break;
    case CMD_DYNAMIXEL_SET_SCIENCE_POSITION:
      setScienceDynamixelPosition(&currentPacket);
      break;
    case CMD_DYNAMIXEL_SCIENCE_CLOSE:
      AX12A_setGoalPosition(DYNAMIXEL_SCIENCE_ADDRESS, SCIENCE_POS_CLOSED);
      break;
    case CMD_DYNAMIXEL_SCIENCE_OPEN:
      AX12A_setGoalPosition(DYNAMIXEL_SCIENCE_ADDRESS, SCIENCE_POS_OPEN);
      break;
    case CMD_DYNAMIXEL_SET_SCIENCE_LED:
      setDynamixelScienceLedFromPacket(&currentPacket);
      break;
    case PING_ON:
      digitalWrite(13, HIGH);
      break;
    case PING_OFF:
      digitalWrite(13, LOW);
      break;
    default:
      break;
    }

  }
  else
  {
    //Serial.println("No bytes in queue");
  }

  //Individual Tests
  /*
  FOR_EACH_TALON
  {
    Serial.print("Testing Talon ");
    Serial.println(i);
    uint8_t talonNum = TALON[i];
    Serial.println("Spinning up Forward");
    for(int j = 0; j < 255; j++)
    {
      setTalonSpeed(talonNum, j, DIR_FORWARD);
      delay(300);
    }
    for(int j = 255; j > 0; j--)
    {
      setTalonSpeed(talonNum, j, DIR_FORWARD);
      delay(300);
    }
    Serial.println("Spinning up Reverse");
    for(int j = 0; j < 255; j++)
    {
      setTalonSpeed(talonNum, j, DIR_REVERSE);
      delay(300);
    }
    for(int j = 255; j > 0; j--)
    {
      setTalonSpeed(talonNum, j, DIR_REVERSE);
      delay(300);
    }


    Serial.println("Full Forward");
    setTalonSpeed(talonNum, 255, DIR_FORWARD);
    delay(100000);
    Serial.println("Full Reverse");
    setTalonSpeed(talonNum, 255, DIR_REVERSE);
    delay(100000);
    Serial.print("Stopping, testing complete for Talon: ");
    Serial.println(i);
    setTalonSpeed(talonNum, 0, DIR_FORWARD);
    delay(100000);
  }
  Serial.println("All tests complete. Reset to try again");
  while(1)
  {

  }
  */
}

