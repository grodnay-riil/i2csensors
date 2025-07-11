/* Blue Robotics Arduino TSYS01 Temperature Sensor Library
------------------------------------------------------------
 
Title: Blue Robotics Arduino TSYS01 Temperature Sensor Library
Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties TSYS01 temperature 
sensor.
Authors: Rustom Jehangir, Blue Robotics Inc.
		 Jonathan Newman, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.
-------------------------------
The MIT License (MIT)
Copyright (c) 2016 Blue Robotics Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/ 

#ifndef TSYS01_H_BLUEROBOTICS
#define TSYS01_H_BLUEROBOTICS

#include "Arduino.h"
#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
extern IMX_RT1060_I2CMaster Master;     // Pins 19 and 18; SCL0 and SDA0

class TSYS01 {
public:

	TSYS01(I2CMaster &_Master=Master);

	bool init();

	/** The read from I2C takes up for 40 ms, so use sparingly is possible.
	 * Now implemented as async behavior tree style.
	 */
	enum ReadState {
		READ_IDLE,
		READ_START_CONVERSION,
		READ_WAIT_CONVERSION,
		READ_REQUEST_DATA,
		READ_WAIT_DATA,
		READ_COMPLETE,
		READ_ERROR
	};
	
	enum TickResult {
		RUNNING,
		SUCCESS,
		FAILURE
	};
	
	ReadState read();
	bool isReadComplete() const;
	void resetRead();

	/** This function loads the datasheet test case values to verify that
	 *  calculations are working correctly. No example checksum is provided
	 *  so the checksum test may fail.
	 */
	void readTestCase();

	/** Temperature returned in deg C.
	 */
	float temperature();

	/** Raw D1 value from sensor. Returns -1 if an error occurred.
	 */
	int32_t rawD1();

	/** Behavior tree style tick function. Returns RUNNING, SUCCESS, or FAILURE.
	 */
	TickResult tick();

private:
	uint16_t C[8];
	uint32_t D1;
	float TEMP;
	uint32_t adc;
	
	// I2C master reference and buffer
	I2CMaster* _master;
	uint8_t _rxBuffer[10]={0};
	
	// Async behavior tree state variables
	ReadState _readState;
	elapsedMillis _conversionTimer;
	elapsedMillis _operationTimer;
	static const uint32_t CONVERSION_TIMEOUT_MS = 50;
	static const uint32_t OPERATION_TIMEOUT_MS = 200;

	/** Performs calculations per the sensor data sheet for conversion and
	 *  second order compensation.
	 */
	void calculate();
	
	/** Helper functions */
	uint16_t getIntFromBuffer();
	bool checkStatus(const char* message);
	void finishOperation();

};

#endif
