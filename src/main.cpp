
/* Blue Robotics TSYS01 Library Example
-----------------------------------------------------

Title: Blue Robotics TSYS01 Library Example

Description: This example demonstrates the TSYS01 Library with a connected
sensor. The example reads the sensor and prints the resulting values
to the serial terminal.

The code is designed for the Arduino Uno board and can be compiled and
uploaded via the Arduino 1.0+ software.

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

#include "TSYS01.h"
#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
TSYS01 sensor(Master);

void setup()
{
    Serial.begin(9600);
    Serial.println("Starting");
	// Initialize the I2C bus
	Master.begin(100 * 1000U);
    while (!sensor.init())
    {
        Serial.println("TSYS01 device failed to initialize!");
        delay(2000);
    }
}
unsigned long lastPrint = 0;
bool readingStarted = false;

void loop()
{
    // Call update to advance the async state machine
    sensor.tick();

    // Check if reading is complete
    if (millis() - lastPrint > 1000)
    {

        printf("Temperature: %.2f C, Raw D1: %ld\n", sensor.temperature(), sensor.rawD1());
        lastPrint = millis();
        sensor.resetRead(); // Reset the read state after printing
    }
}
