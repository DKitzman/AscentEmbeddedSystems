#ifndef I2CAPI_H
#define I2CAPI_H


struct packet
{
	uint8_t cmd;
	uint8_t buffer[15];
};

#ifndef F_CPU
	#define F_CPU 2000000UL
#endif

void i2cAPI_init(uint8_t addressP);
void i2cAPI_checkForPackets();
uint8_t i2cAPI_hasPacket();
uint8_t i2cAPI_getPacket(struct packet *packetDestination);
void i2cAPI_setReturnPacket(const struct packet * const packetPointer, uint8_t numDataBytes);
void i2cAPI_resetIfBusHogging();

#endif