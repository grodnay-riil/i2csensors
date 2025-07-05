#include "TSYS01.h"
#include <Arduino.h>
#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"

const uint8_t TSYS01_ADDR = 0x77;
const uint8_t TSYS01_RESET = 0x1E;
const uint8_t TSYS01_ADC_READ = 0x00;
const uint8_t TSYS01_ADC_TEMP_CONV = 0x48;
const uint8_t TSYS01_PROM_READ = 0XA0;


TSYS01::TSYS01(I2CMaster &Master)
	: _master(&Master), _readState(READ_IDLE), _conversionTimer(0), _operationTimer(0)
{
	
}

uint16_t TSYS01::getIntFromBuffer()
{
	uint16_t result = ((uint16_t)_rxBuffer[0] << 8U) + ((uint16_t)_rxBuffer[1]);
	return result;
}

bool TSYS01::checkStatus(const char *message)
{
	if (_master->has_error())
	{
		Serial.print(message);
		Serial.print(" Error: ");
		Serial.println((int)_master->error());
		return false;
	}
	return true;
}

void TSYS01::finishOperation()
{
	elapsedMillis timeout;
	while (timeout < 200)
	{
		if (_master->finished())
		{
			return;
		}
	}
	Serial.println("Master: ERROR timed out waiting for transfer to finish.");
}

bool TSYS01::init()
{
	// Initialize the I2C bus
	_master->begin(100 * 1000U);

	// Reset the TSYS01, per datasheet
	_master->write_async(TSYS01_ADDR, (uint8_t *)&TSYS01_RESET, sizeof(TSYS01_RESET), false);
	finishOperation();

	delay(10);
	int received_bytes = 0;
	// Read calibration values
	for (uint8_t i = 0; i < 8; i++)
	{
		uint8_t register_number = TSYS01_PROM_READ + i * 2;
		_master->write_async(TSYS01_ADDR, &register_number, 1, false);
		finishOperation();

		_master->read_async(TSYS01_ADDR, _rxBuffer, 2, true);
		finishOperation();
		C[i] = getIntFromBuffer();
		received_bytes += _master->get_bytes_transferred();
	}
	return received_bytes > 0;
}

TSYS01::ReadState TSYS01::read()
{
	switch (_readState)
	{
	case READ_IDLE:
		// Start temperature conversion
		_master->write_async(TSYS01_ADDR, (uint8_t *)&TSYS01_ADC_TEMP_CONV, sizeof(TSYS01_ADC_TEMP_CONV), true);
		_operationTimer = 0;
		_readState = READ_START_CONVERSION;
		return _readState;

	case READ_START_CONVERSION:
		// Wait for conversion command to complete
		if (_master->finished())
		{
			if (!checkStatus("Temperature conversion start"))
			{
				_readState = READ_ERROR;
				return _readState;
			}
			_conversionTimer = 0;
			_readState = READ_WAIT_CONVERSION;
		}
		else if (_operationTimer > OPERATION_TIMEOUT_MS)
		{
			Serial.println("Timeout waiting for conversion start");
			_readState = READ_ERROR;
		}
		return _readState;

	case READ_WAIT_CONVERSION:
		// Wait for conversion to complete (10ms max per datasheet)
		if (_conversionTimer >= 10)
		{
			// Start reading the converted data
			_master->write_async(TSYS01_ADDR, (uint8_t *)&TSYS01_ADC_READ, sizeof(TSYS01_ADC_READ), true);
			_operationTimer = 0;
			_readState = READ_REQUEST_DATA;
		}
		return _readState;

	case READ_REQUEST_DATA:
		// Wait for read request to complete
		if (_master->finished())
		{
			if (!checkStatus("Temperature data request"))
			{
				_readState = READ_ERROR;
				return _readState;
			}
			// Now read the actual data
			_master->read_async(TSYS01_ADDR, _rxBuffer, 3, true);
			_operationTimer = 0;
			_readState = READ_WAIT_DATA;
		}
		else if (_operationTimer > OPERATION_TIMEOUT_MS)
		{
			Serial.println("Timeout waiting for data request");
			_readState = READ_ERROR;
		}
		return _readState;

	case READ_WAIT_DATA:
		// Wait for data read to complete
		if (_master->finished())
		{
			if (!checkStatus("Temperature data read"))
			{
				_readState = READ_ERROR;
				return _readState;
			}
			// Process the received data
			D1 = ((uint32_t)_rxBuffer[0] << 16U) + ((uint32_t)_rxBuffer[1] << 8U) + (uint32_t)_rxBuffer[2];
			calculate();
			_readState = READ_COMPLETE;
		}
		else if (_operationTimer > OPERATION_TIMEOUT_MS)
		{
			Serial.println("Timeout waiting for data read");
			_readState = READ_ERROR;
		}
		return _readState;

	case READ_COMPLETE:
		// Reading is complete, stay in this state until reset
		return _readState;

	case READ_ERROR:
		// Error occurred, stay in this state until reset
		return _readState;

	default:
		_readState = READ_ERROR;
		return _readState;
	}
}

bool TSYS01::isReadComplete() const
{
	return _readState == READ_COMPLETE;
}

void TSYS01::resetRead()
{
	_readState = READ_IDLE;
}

void TSYS01::readTestCase()
{
	C[0] = 0;
	C[1] = 28446; // 0xA2 K4
	C[2] = 24926; // 0XA4 k3
	C[3] = 36016; // 0XA6 K2
	C[4] = 32791; // 0XA8 K1
	C[5] = 40781; // 0XAA K0
	C[6] = 0;
	C[7] = 0;

	D1 = 9378708.0f;

	adc = D1 / 256;

	calculate();
}

void TSYS01::calculate()
{
	adc = D1 / 256;

	TEMP = (-2) * float(C[1]) / 1000000000000000000000.0f * pow(adc, 4) +
		   4 * float(C[2]) / 10000000000000000.0f * pow(adc, 3) +
		   (-2) * float(C[3]) / 100000000000.0f * pow(adc, 2) +
		   1 * float(C[4]) / 1000000.0f * adc +
		   (-1.5) * float(C[5]) / 100;
}

float TSYS01::temperature()
{
	return TEMP;
}

int32_t TSYS01::rawD1()
{
	// Return -1 if an error occurred during reading
	if (_readState == READ_ERROR)
	{
		return -1;
	}
	
	// Return the raw D1 value
	return (int32_t)D1;
}

TSYS01::TickResult TSYS01::tick()
{
	// Call read() to advance the state machine
	ReadState state = read();
	
	switch (state)
	{
	case READ_COMPLETE:
		return SUCCESS;
	case READ_ERROR:
		return FAILURE;
	default:
		return RUNNING;
	}
}
