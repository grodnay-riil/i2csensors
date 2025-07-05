

#ifndef KELLERLD_H_BLUEROBOTICS
#define KELLERLD_H_BLUEROBOTICS

#include "Arduino.h"

#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
extern IMX_RT1060_I2CMaster Master;

class KellerLD
{
public:
	static constexpr float Pa = 100.0f;
	static constexpr float bar = 0.001f;
	static constexpr float mbar = 1.0f;

	KellerLD(I2CMaster &_Master = Master);

	/** Reads the onboard memory map to determine min and max pressure as
	 *  well as manufacture date, mode, and customer ID.
	 */
	void init();

	/** Provide the density of the working fluid in kg/m^3. Default is for
	 * seawater. Should be 997 for freshwater.
	 */
	void setFluidDensity(float density);

	/** The read from I2C takes up for 40 ms, so use sparingly is possible.
	 */
	void read();

	/** Checks if the attached sensor is connectored or not. */
	bool status();

	/** Returns current range of the attached sensor. */
	float range();

	/** Pressure returned in mbar or mbar*conversion rate.
	 */
	float pressure(float conversion = 1.0f);

	/** Temperature returned in deg C.
	 */
	float temperature();

	/** Depth returned in meters (valid for operation in incompressible
	 *  liquids only. Uses density that is set for fresh or seawater.
	 */
	float depth();

	/** Altitude returned in meters (valid for operation in air only).
	 */
	float altitude();

	/** Return true if the sensor has been initialized and detected. */
	bool isInitialized();

	uint16_t equipment;
	uint16_t place;
	uint16_t file;

	uint8_t mode;
	uint16_t year;
	uint8_t month;
	uint8_t day;

	uint32_t code;

	uint16_t P;
	float P_bar;
	float P_mode;
	float P_min;
	float P_max;

private:
	I2CMaster &_master;
	float fluidDensity;
	float T_degc;

	uint16_t cust_id0;
	uint16_t cust_id1;

	uint16_t readMemoryMap(uint8_t mtp_address);

	bool checkStatus(const char *message);
	void finishOperation();
};

#endif
