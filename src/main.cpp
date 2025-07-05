

#include "TSYS01.h"
#include "KellerLD.h"
#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
TSYS01 tempratureSensor(Master);
KellerLD pressureSensor(Master);

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // Wait for Serial to be ready (needed for some boards)
    Serial.println("Starting");
    // Initialize the I2C bus
    Master.begin(100 * 1000U);
    while (!tempratureSensor.init())
    {
        Serial.println("TSYS01 device failed to initialize!");
        delay(2000);
    }
    pressureSensor.init();
    pressureSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    if (pressureSensor.isInitialized())
    {
        Serial.println("Sensor connected.");
    }
    else
    {
        Serial.println("Sensor not connected.");
    }
}
unsigned long lastPrint = 0;
bool readingStarted = false;

void loop()
{
    // Call update to advance the async state machine
    tempratureSensor.tick();

    // Check if reading is complete
    if (millis() - lastPrint > 1000)
    {

        printf("Temperature: %.2f C, Raw D1: %ld\n", tempratureSensor.temperature(), tempratureSensor.rawD1());
        lastPrint = millis();
        tempratureSensor.resetRead(); // Reset the read state after printing
        pressureSensor.read(); // Read pressure data
        printf("Pressure: %.2f mbar, Depth: %.2f m, Altitude: %.2f m\n",
               pressureSensor.pressure(), pressureSensor.depth(), pressureSensor.altitude());
    }
}
