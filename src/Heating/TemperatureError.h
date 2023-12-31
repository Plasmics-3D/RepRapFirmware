/*
 * TemperatureError.h
 *
 *  Created on: 21 Apr 2016
 *      Author: David
 */

#ifndef TEMPERATUREERROR_H_
#define TEMPERATUREERROR_H_

#include <cstdint>

// Result codes returned by temperature sensor drivers
enum class TemperatureError : uint8_t
{
	success,
	shortCircuit,
	shortToVcc,
	shortToGround,
	openCircuit,
	timeout,
	ioError,
	hardwareError,
	notReady,
	invalidOutputNumber,
	busBusy,
	badResponse,
	unknownPort,
	notInitialised,
	unknownSensor,
	overOrUnderVoltage,
	badVref,
	badVssa,
	InoNoHeartBeat,
	InoNoSteadyHeat,
	InoHeatSlow,
	InoHeatFast
};

const char* TemperatureErrorString(TemperatureError err) noexcept;

#endif /* TEMPERATUREERROR_H_ */
