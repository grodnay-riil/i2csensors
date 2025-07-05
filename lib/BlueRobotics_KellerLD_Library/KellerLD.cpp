#include "KellerLD.h"
#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
extern IMX_RT1060_I2CMaster Master;
const uint8_t LD_ADDR = 0x40;
const uint8_t LD_REQUEST = 0xAC;
const uint8_t LD_CUST_ID0 = 0x00;
const uint8_t LD_CUST_ID1 = 0x01;
const uint8_t LD_SCALING0 = 0x12;
const uint8_t LD_SCALING1 = 0x13;
const uint8_t LD_SCALING2 = 0x14;
const uint8_t LD_SCALING3 = 0x15;
const uint8_t LD_SCALING4 = 0x16;

KellerLD::KellerLD(I2CMaster &_Master) : _master(_Master) {
	fluidDensity = 1029;
}

void KellerLD::init() {
	// Request memory map information
	cust_id0 = readMemoryMap(LD_CUST_ID0);
	cust_id1 = readMemoryMap(LD_CUST_ID1);

	code = (uint32_t(cust_id1) << 16) | cust_id0;
	equipment = cust_id0 >> 10;
	place = cust_id0 & 0b000000111111111;
	file = cust_id1;
	Serial.printf("KellerLD equipment: %d, place: %d, file: %d, code: %08X\n", 
		equipment, place, file, code);
	uint16_t scaling0;
	scaling0 = readMemoryMap(LD_SCALING0);

	mode = scaling0 & 0b00000011;
	year = scaling0 >> 11;
	month = (scaling0 & 0b0000011110000000) >> 7;
	day = (scaling0 & 0b0000000001111100) >> 2;
	Serial.printf("KellerLD mode: %d, date: %04d-%02d-%02d\n", 
		mode, year+2000, month, day);
	// handle P-mode pressure offset (to vacuum pressure)

	if (mode == 0) { 
		// PR mode, Vented Gauge. Zero when front pressure == rear pressure
		// TODO: allow updating to variable local atmosphere/enclosure pressure
		//       from an air pressure sensor
		P_mode = 1.01325;
	} else if (mode == 1) {
		// PA mode, Sealed Gauge. Zero at 1.0 bar
		P_mode = 1.0;
	} else {
		// PAA mode, Absolute. Zero at vacuum
		// (or undefined mode)
		P_mode = 0;
	}

	uint32_t scaling12 = (uint32_t(readMemoryMap(LD_SCALING1)) << 16) | readMemoryMap(LD_SCALING2);

	P_min = *reinterpret_cast<float*>(&scaling12);
	Serial.printf("KellerLD P_min: %.2f mbar\n", P_min);

	uint32_t scaling34 = (uint32_t(readMemoryMap(LD_SCALING3)) << 16) | readMemoryMap(LD_SCALING4);

	P_max = *reinterpret_cast<float*>(&scaling34);
	Serial.printf("KellerLD P_max: %.2f mbar\n", P_max);
}

void KellerLD::setFluidDensity(float density) {
	fluidDensity = density;
}

void KellerLD::read() {
	uint8_t status;

	
	_master.write_async(LD_ADDR, &LD_REQUEST, sizeof(LD_REQUEST), true);
	finishOperation();

	delay(9); // Max conversion time per datasheet
	uint8_t buffer[5];
	_master.read_async(LD_ADDR, buffer, sizeof(buffer), true);
	finishOperation();
	if (!checkStatus("KellerLD read() failed")) {
		return;
	}
	status = buffer[0];
	P = (buffer[1] << 8) | buffer[2];
	uint16_t T = (buffer[3] << 8) | buffer[4];

	P_bar = (float(P)-16384)*(P_max-P_min)/32768 + P_min + P_mode;
	T_degc = ((T>>4)-24)*0.05-50;
}

uint16_t KellerLD::readMemoryMap(uint8_t mtp_address) {
	uint8_t status;

	_master.write_async(LD_ADDR, &mtp_address, sizeof(mtp_address), true);
	finishOperation();

	delay(1); // allow for response to come in
	uint8_t buffer[3]={0};
	_master.read_async(LD_ADDR, buffer, sizeof(buffer), true);
	finishOperation();

	if (!checkStatus("KellerLD readMemoryMap() failed")) {
		return 0;
	}
	status = buffer[0];
	return ((buffer[1] << 8) | buffer[2]);
}

bool KellerLD::status() {
	if (equipment <= 62 ) {
		return true;
	} else {
		return false;
	}
}

float KellerLD::range() {
	return P_max-P_min;
}

float KellerLD::pressure(float conversion) {
	return P_bar*1000.0f*conversion;
}

float KellerLD::temperature() {
	return T_degc;
}

float KellerLD::depth() {
	return (pressure(KellerLD::Pa)-101325)/(fluidDensity*9.80665);
}

float KellerLD::altitude() {
	return (1-pow((pressure()/1013.25),0.190284))*145366.45*.3048;
}

bool KellerLD::isInitialized() {
	return (cust_id0 >> 10) != 63; // If not connected, equipment code == 63
}


bool KellerLD::checkStatus(const char *message)
{
	if (_master.has_error())
	{
		Serial.print(message);
		Serial.print(" Error: ");
		Serial.println((int)_master.error());
		return false;
	}
	return true;
}

void KellerLD::finishOperation()
{
	elapsedMillis timeout;
	while (timeout < 200)
	{
		if (_master.finished())
		{
			return;
		}
	}
	Serial.println("Master: ERROR timed out waiting for transfer to finish.");
}