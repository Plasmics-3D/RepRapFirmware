/*
 * InoTrident.cpp
 *
 *  Created on: 10 May 2023
 *      Author: KOlasmics
 */

#include "InoTrident.h"

#if SUPPORT_SPI_SENSORS

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#include <Heating/Heat.h>


#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

const uint32_t INO_Frequency = 2000000;	// maximum for INO is 2MHz

static const char * const TypeLetters = "JKN";		// INO mapping of AVGSEWL bits to thermocouple types
const uint8_t TypeK = 3;

// SPI mode:
const SpiMode INO_SpiMode = SpiMode::mode1;

// Define the minimum interval between readings
const uint32_t MinimumReadInterval = 100;			// minimum interval between reads, in milliseconds

const uint8_t DefaultCr0 = 0b10010111;
const uint8_t Cr0ReadMask = 0b10111101;			// bits 1 and 6 auto clear, so ignore the value read

//uint8_t number = 99;
//uint8_t *old_error_flag = &number; // It's been used for error resetting feature

InoTrident::InoTrident(unsigned int sensorNum) noexcept
	: SpiTemperatureSensor(sensorNum, "Induction Nozzle (INO)", INO_SpiMode, INO_Frequency),
	  cr0(DefaultCr0), thermocoupleType(TypeK)
{
	INO_Sensornum = sensorNum;
}

// Configure this temperature sensor
GCodeResult InoTrident::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed)
{
	if (!ConfigurePort(gb, reply, changed))
	{
		return GCodeResult::error;
	}
	TryConfigureSensorName(gb, changed);
	if (gb.Seen('F'))
	{
		changed = true;
		if (gb.GetIValue() == 60)
		{
			cr0 &= ~0x01;		// set 60Hz rejection
		}
		else
		{
			cr0 |= 0x01;		// default to 50Hz rejection
		}
	}

	String<2> buf;
	if (gb.TryGetQuotedString('K', buf.GetRef(), changed))
	{
		const char *p;
		if (buf.strlen() == 1 && (p = strchr(TypeLetters, toupper(buf.c_str()[0]))) != nullptr)
		{
			thermocoupleType = p - TypeLetters;
		}
		else
		{
			reply.copy("Bad thermocouple type letter in M305 command");
			return GCodeResult::error;
		}
	}

	return FinishConfiguring(changed, reply);
}

#if SUPPORT_REMOTE_COMMANDS
// HAVENT TRIED WITH EXPANSION BOARD WHATSOEVER
GCodeResult InoTrident::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = false;
	if (!ConfigurePort(parser, reply, seen))
	{
		return GCodeResult::error;
	}

	uint8_t paramF;
	if (parser.GetUintParam('F', paramF))
	{
		seen = true;
		if (paramF == 60)
		{
			cr0 &= ~0x01;		// set 60Hz rejection
		}
		else
		{
			cr0 |= 0x01;		// default to 50Hz rejection
		}
	}

	char paramK;
	if (parser.GetCharParam('K', paramK))
	{
		seen = true;
		const char *p;
		if ((p = strchr(TypeLetters, toupper(paramK))) != nullptr)
		{
			thermocoupleType = p - TypeLetters;
		}
		else
		{
			reply.copy("Bad thermocouple type letter in M305 command");
			return GCodeResult::error;
		}
	}

	return FinishConfiguring(seen, reply);
}

#endif

GCodeResult InoTrident::FinishConfiguring(bool changed, const StringRef& reply) noexcept
{
	if (changed)
	{
		// Initialize the sensor
		InitSpi();

		TemperatureError rslt;
		for (unsigned int i = 0; i < 3; ++i)		// try 3 times
		{
			rslt = TryInitThermocouple();
			if (rslt == TemperatureError::success)
			{
				break;
			}
			delay(MinimumReadInterval);
		}

		lastReadingTime = millis();
		lastResult = rslt;
		lastTemperature = 0.0;

		if (rslt != TemperatureError::success)
		{
			reply.printf("Failed to initialise thermocouple: %s", TemperatureErrorString(rslt));
			return GCodeResult::error;
		}
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", thermocouple type %c, reject %dHz", TypeLetters[thermocoupleType], (cr0 & 0x01) ? 50 : 60);
	}
	return GCodeResult::ok;
}

TemperatureError InoTrident::TryInitThermocouple() const noexcept
{
	uint32_t dummy_Val;
	uint8_t thermoData[3] = { 0b00001001, 0x00, 0x00 };

	if(thermocoupleType == 1){		  // K
		thermoData[2] = 0x00;
	}else if(thermocoupleType == 0){  // J
		thermoData[2] = 0x01;
	}else if(thermocoupleType == 2){  // N
		thermoData[2] = 0x02;
	}

	TemperatureError sts = DoSpiTransaction(thermoData, ARRAY_SIZE(thermoData), dummy_Val);
	delayMicroseconds(1000);

	return sts;
}

void InoTrident::Poll() noexcept
{
	delayMicroseconds(10);

	uint32_t dummy_rx_val; // Used for reading empty/unimportant responses

	Heat& heat = reprap.GetHeat();

	target_temp = heat.GetTargetTemperature(INO_Sensornum);

	HeaterStatus error_flag = heat.GetStatus(INO_Sensornum);

	const auto h = heat.GetHeaterAddress(INO_Sensornum);

	if (h.IsNotNull()){
		const FopDt& model = h->GetModel();
		M301PidParameters pp = model.GetM301PidParameters(false);

		if(INO_kP != pp.kP || INO_kI != pp.kI || INO_kD != pp.kD){
			INO_kP = pp.kP;
			INO_kI = pp.kI;
			INO_kD = pp.kD;
			pid_changed_flag = 1;
		}
	}

	if(error_flag == HeaterStatus::fault){
		old_error_flag = 3; // error_flag
	}

	if(old_error_flag == 3 && error_flag == HeaterStatus::off){ //old_error_flag == HeaterStatus::fault && error_flag == HeaterStatus::off
		// RESET ERROR REGISTER after re-enabling the heater
		static const uint8_t err_reset[3] = {0b00001001, 0xFF, 0xFF};
		DoSpiTransaction(err_reset, ARRAY_SIZE(err_reset), dummy_rx_val);
		delayMicroseconds(1000);
		old_error_flag = 0;
		pid_changed_flag = 1; // Send the PID Values just in case
	}

	if(pid_changed_flag == 1){
		PIDUpdate(INO_kP, INO_kI, INO_kD);
		pid_changed_flag = 0;
	}

	if(target_temp != old_target_temp ){ // Temp target is changed? -> set the new one
		old_target_temp = target_temp;

		uint8_t target_temp_msb = (target_temp >> 8) & 0x01;
		uint8_t target_temp_lsb = target_temp & 0xFF;
		uint8_t temptarget[3] = {0b00000101, target_temp_msb, target_temp_lsb};
		// Set Register -> Temperature Target
		DoSpiTransaction(temptarget, ARRAY_SIZE(temptarget), dummy_rx_val);
		delayMicroseconds(1000);
	}

	// Read Register -> Temperature Actual
	static const uint8_t dataOut[3] = {0b00000010, 0x00, 0x00};
	TemperatureError sts = DoSpiTransaction(dataOut, ARRAY_SIZE(dataOut), dummy_rx_val);
	delayMicroseconds(1000);

	//uint32_t rawVal;
	//int32_t rawTemp = 0;
	static const uint8_t readData[3] = {0x00, 0x00, 0x00};

	if (sts == TemperatureError::success)
	{
		sts = TemperatureError::notInitialised;
		// Get a response from register
		sts = DoSpiTransaction(readData, ARRAY_SIZE(readData), rawVal);
		delayMicroseconds(1000);
	}

	if (sts != TemperatureError::success)
	{
		SetResult(sts);

	}
	else
	{
		lastReadingTime = millis();

		// CHECK IF SOME ERROR OCCURED
		if((rawVal & 0x01) == 1){
			// ERROR HAPPENED -> Read Error + Configuration Register
			static const uint8_t error_Data[3] = {0b00001000, 0x00, 0x00};
			sts = DoSpiTransaction(error_Data, ARRAY_SIZE(error_Data), dummy_rx_val);
			delayMicroseconds(1000);

			//uint32_t err_rcv;

			// Get a response from error register
			sts = DoSpiTransaction(readData, ARRAY_SIZE(readData), err_rcv);
			delayMicroseconds(1000);

			//uint8_t error_bits;
			error_bits = ((err_rcv >> 8) & 0xFF);

			if(sts != TemperatureError::success){ // There could be a connection error at this point
				SetResult(TemperatureError::ioError);
			}else{
				// 0 x 0 | 0 | OpenCircuit | NoHeartbeat | HeatingSlow | HeatingFast |  NoTempRead | 0
				if((error_bits & 0b00100000) != 0){ 		 // OpenCircuit
					SetResult(TemperatureError::openCircuit);
				}else if((error_bits & 0b00010000) != 0){ 	 // NoHeartbeat
					SetResult(TemperatureError::InoNoHeartBeat);
				}else if((error_bits & 0b00001100) == 0x0C){ // No Steady Heat
					SetResult(TemperatureError::InoNoSteadyHeat);
				}else if((error_bits & 0b00001000) != 0){	 // HeatingSlow
					SetResult(TemperatureError::InoHeatSlow);
				}else if((error_bits & 0b00000100) != 0){    // HeatingFast
					SetResult(TemperatureError::InoHeatFast);
				}else if((error_bits & 0b00000010) != 0){ 	 // NoTempRead
					SetResult(TemperatureError::badResponse);
				}

			}
			delayMicroseconds(1);

		}else{
			rawTemp = (((float)((rawVal >> 8) & 0x7F)) * 16) + (((float)(rawVal & 0xFC)) / 16);
			//lastTemp = (float) rawTemp;
			SetResult(rawTemp, TemperatureError::success);
		}

	}

}

void InoTrident::PIDUpdate(float P, float I, float D) noexcept
{
	uint32_t dummy_rx_val;
	uint32_t pid_buf = 0;
	uint8_t pid_buf_lsb = 0;
	uint8_t pid_buf_msb = 0;

	pid_buf = P * 100;
	pid_buf_lsb |= pid_buf;
	pid_buf_msb |= pid_buf >> 8;

	//write kP
	uint8_t P_val[3] = {0b00010001, pid_buf_msb, pid_buf_lsb};
	DoSpiTransaction(P_val, ARRAY_SIZE(P_val), dummy_rx_val);
	delayMicroseconds(1000);

	pid_buf = I * 100;
	pid_buf_lsb = 0;
	pid_buf_msb = 0;
	pid_buf_lsb |= pid_buf;
	pid_buf_msb |= pid_buf >> 8;

	//write kI
	uint8_t I_val[3] = {0b00100001, pid_buf_msb, pid_buf_lsb};
	DoSpiTransaction(I_val, ARRAY_SIZE(I_val), dummy_rx_val);
	delayMicroseconds(1000);

	pid_buf = D * 100;
	pid_buf_lsb = 0;
	pid_buf_msb = 0;
	pid_buf_lsb |= pid_buf;
	pid_buf_msb |= pid_buf >> 8;

	//write kD
	uint8_t D_val[3] = {0b01000001, pid_buf_msb, pid_buf_lsb};
	DoSpiTransaction(D_val, ARRAY_SIZE(D_val), dummy_rx_val);
	delayMicroseconds(1000);

}

#endif // SUPPORT_SPI_SENSORS

// End
