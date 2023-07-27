/*
 * InoTrident.h
 *
 *  Created on: 10 May 2023
 *      Author: KOlasmics
 */

#ifndef SRC_HEATING_SENSORS_INOTRIDENT_H_
#define SRC_HEATING_SENSORS_INOTRIDENT_H_

#include "SpiTemperatureSensor.h"

#include <Heating/Heater.h>

#if SUPPORT_SPI_SENSORS

class InoTrident : public SpiTemperatureSensor
{
public:
	InoTrident(unsigned int sensorNum) noexcept;
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) override THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override; // configure the sensor from M308 parameters
#endif

	void Poll() noexcept override;
	const char *GetShortSensorType() const noexcept override { return TypeName; }

	static constexpr const char *TypeName = "inotrident";

private:
	TemperatureError TryInitThermocouple() const noexcept;
	GCodeResult FinishConfiguring(bool changed, const StringRef& reply) noexcept;
	void PIDUpdate(float P, float I, float D) noexcept;

	uint8_t cr0;
	uint8_t thermocoupleType;

	uint8_t INO_Sensornum;
	uint8_t old_error_flag;
	uint32_t old_target_temp = 0;
	uint8_t pid_changed_flag = 0;

	uint32_t target_temp = 0;
	//HeaterStatus *error_flag = 0;

	uint32_t rawVal;
	float rawTemp = 0;
	//float lastTemp = 0;

	uint32_t err_rcv;
	uint8_t error_bits;

	//PID for INO
	float INO_kP = 0;
	float INO_kI = 0;
	float INO_kD = 0;
};

#endif // SUPPORT_SPI_SENSORS

#endif /* SRC_HEATING_SENSORS_INOTRIDENT_H_ */
