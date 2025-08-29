/*
 * GNSS.h
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *Copyright 2020 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include <cstdint>
#include <optional>

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

struct UniqueID
{
	uint8_t id[6]; // Unique ID of the GNSS module
};

struct UTCTime
{
	uint16_t year;
	uint8_t month, day, hour, min, sec;
	int32_t nano;
	uint32_t tAcc;
	uint8_t valid; // Flags related to time validity
};

struct PositionLLH
{
	int32_t lon, lat;		// Geodetic position (degrees * 10^-7)
	int32_t height, hMSL; 	// Geodetic position (millimeters)
	uint32_t hAcc, vAcc;	// Accuracy estimates (millimeters)
};

struct PositionECEF
{
	int32_t ecefX, ecefY, ecefZ; // Earth-centered, Earth-fixed coordinates (centimeters)
	uint32_t pAcc;				 // Position accuracy (centimeters)
};

struct VelocityNED
{
	int32_t velN, velE, velD; // Velocity components (centimeters/second)
	int32_t headMot;		  // Heading of motion (degrees * 10^-5)
	uint32_t speed, gSpeed;	  // Ground speed (centimeters/second)
	uint32_t sAcc, headAcc;	  // Speed and heading accuracy (centimeters/second, degrees * 10^-5)
};

struct VelocityECEF
{
	int32_t ecefVX, ecefVY, ecefVZ; // ECEF velocity components (centimeters/second)
	uint32_t sAcc;					// Speed accuracy (centimeters/second)
};

struct NavigationPVT
{
	UTCTime utcTime;
	PositionLLH position;
	VelocityNED velocity;
	int8_t fixType; // Navigation fix type
	uint8_t numSV;	// Number of satellites used
};

enum GNSSMode
{
	Portable = 0,
	Stationary = 1,
	Pedestrian = 2,
	Automotiv = 3,
	Sea = 4,
	Airbone1G = 5,
	Airbone2G = 6,
	Airbone4G = 7,
	Wrist = 8,
	Bike = 9
};

// Helper function to calculate checksum for static asserts
static constexpr void CalculateChecksum(const uint8_t *payload, size_t size, uint8_t *cka, uint8_t *ckb)
{
	*cka = 0;
	*ckb = 0;
	for (size_t i = 2; i < size - 2; ++i)
	{ // Skip sync chars and last 2 checksum bytes
		*cka += payload[i];
		*ckb += *cka;
	}
}

// Helper function to validate checksum within static assert
template <size_t Size>
static constexpr bool ValidateChecksum(const uint8_t (&payload)[Size])
{
	uint8_t cka_calc, ckb_calc;
	CalculateChecksum(payload, Size, &cka_calc, &ckb_calc);
	return (cka_calc == payload[Size - 2]) && (ckb_calc == payload[Size - 1]);
}

class GNSS
{
private:
	static constexpr uint16_t GNSS_BUFFER_SIZE = 201U;
	static constexpr uint16_t UBLOX_HEADER_SIZE = 6U;

public:
	GNSS() = delete;
	GNSS(UART_HandleTypeDef *huart);

	void SetMode(GNSSMode gnssMode);
	std::optional<UniqueID> getUniqID();
	std::optional<UTCTime> GetNavTimeUTC();
	std::optional<PositionLLH> GetNavPosLLH();
	std::optional<PositionECEF> GetNavPosECEF();
	std::optional<NavigationPVT> GetNavPVT();
	std::optional<VelocityECEF> GetNavVelECEF();
	std::optional<VelocityNED> GetNavVelNED();

private:
	void LoadConfig();
	uint8_t *FindHeader(uint8_t classID, uint8_t messageID);
	uint8_t *Request(const uint8_t *request, uint16_t size, uint8_t classID, uint8_t messageID);
	UniqueID ParseUniqID(uint8_t *messageBuffer);
	UTCTime ParseNavTimeUTC(uint8_t *messageBuffer);
	PositionLLH ParseNavPosLLH(uint8_t *messageBuffer);
	PositionECEF ParseNavPosECEF(uint8_t *messageBuffer);
	NavigationPVT ParseNavPVT(uint8_t *messageBuffer);
	VelocityECEF ParseNavVelECEF(uint8_t *messageBuffer);
	VelocityNED ParseNavVelNED(uint8_t *messageBuffer);

	uint8_t GetUByte(const uint8_t *uartWorkingBuffer, uint16_t offset);
	int8_t GetIByte(const uint8_t *uartWorkingBuffer, uint16_t offset);
	uint16_t GetUShort(const uint8_t *uartWorkingBuffer, uint16_t offset);
	int16_t GetIShort(const uint8_t *uartWorkingBuffer, uint16_t offset);
	uint32_t GetULong(const uint8_t *uartWorkingBuffer, uint16_t offset);
	int32_t GetILong(const uint8_t *uartWorkingBuffer, uint16_t offset);

	void Checksum(uint16_t dataLength, const uint8_t *payload, uint8_t *cka, uint8_t *ckb);

private:
	UART_HandleTypeDef *huart;
	uint8_t uartBuffer[GNSS_BUFFER_SIZE];

	static constexpr uint8_t configUBX[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9A, 0x79};
	static constexpr uint8_t setNMEA410[] = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x57};
	// Activation of navigation system: Galileo, Glonass, GPS, SBAS, IMES

	static constexpr uint8_t setGNSS[] = {0xB5, 0x62, 0x06, 0x3E, 0x24, 0x00, 0x00, 0x00, 0x20, 0x04, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0xDF, 0xFB};
	static constexpr uint8_t getUniqueID[] = {0xB5, 0x62, 0x27, 0x03, 0x00, 0x00, 0x2A, 0xA5};
	static constexpr uint8_t getNavTimeUTC[] = {0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67};
	static constexpr uint8_t getNavPosLLH[] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A};
	static constexpr uint8_t getNavPosECEF[] = {0xB5, 0x62, 0x01, 0x01, 0x00, 0x00, 0x02, 0x07};
	static constexpr uint8_t getNavPVT[] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
	static constexpr uint8_t getNavVelECEF[] = {0xB5, 0x62, 0x01, 0x11, 0x00, 0x00, 18, 55};
	static constexpr uint8_t getNavVelNED[] = {0xB5, 0x62, 0x01, 0x12, 0x00, 0x00, 19, 58};

	static constexpr uint8_t setPortableMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x3C};
	static constexpr uint8_t setStationaryMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80};
	static constexpr uint8_t setPedestrianMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xA2};
	static constexpr uint8_t setAutomotiveMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82, 0xC4};
	static constexpr uint8_t setSeaMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x05, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x83, 0xE6};
	static constexpr uint8_t setAirbone1GMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x08};
	static constexpr uint8_t setAirbone2GMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x2A};
	static constexpr uint8_t setAirbone4GMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0x4C};
	static constexpr uint8_t setWristMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x09, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x6E};
	static constexpr uint8_t setBikeMode[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x0A, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x5E, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x90};

	static_assert(ValidateChecksum(configUBX), "configUBX checksum validation failed");
	static_assert(ValidateChecksum(setNMEA410), "setNMEA410 checksum validation failed");
	static_assert(ValidateChecksum(setGNSS), "setGNSS checksum validation failed");
	static_assert(ValidateChecksum(getUniqueID), "getUniqueID checksum validation failed");
	static_assert(ValidateChecksum(getNavTimeUTC), "getNavTimeUTC checksum validation failed");
	static_assert(ValidateChecksum(getNavPosLLH), "getNavPosLLH checksum validation failed");
	static_assert(ValidateChecksum(getNavPosECEF), "getNavPosECEF checksum validation failed");
	static_assert(ValidateChecksum(getNavPVT), "getNavPVT checksum validation failed");
	static_assert(ValidateChecksum(getNavVelECEF), "getNavVelECEF checksum validation failed");
	static_assert(ValidateChecksum(getNavVelNED), "getNavVelNED checksum validation failed");
	static_assert(ValidateChecksum(setPortableMode), "setPortableMode checksum validation failed");
	static_assert(ValidateChecksum(setStationaryMode), "setStationaryMode checksum validation failed");
	static_assert(ValidateChecksum(setPedestrianMode), "setPedestrianMode checksum validation failed");
	static_assert(ValidateChecksum(setAutomotiveMode), "setAutomotiveMode checksum validation failed");
	static_assert(ValidateChecksum(setSeaMode), "setSeaMode checksum validation failed");
	static_assert(ValidateChecksum(setAirbone1GMode), "setAirbone1GMode checksum validation failed");
	static_assert(ValidateChecksum(setAirbone2GMode), "setAirbone2GMode checksum validation failed");
	static_assert(ValidateChecksum(setAirbone4GMode), "setAirbone4GMode checksum validation failed");
	static_assert(ValidateChecksum(setWristMode), "setWristMode checksum validation failed");
	static_assert(ValidateChecksum(setBikeMode), "setBikeMode checksum validation failed");
};

#include "au.hpp"

struct PositionECEF_AU
{
	au::QuantityF<au::MetersInEcefFrame> x, y, z;
	au::QuantityF<au::MetersInEcefFrame> acc;
};

struct VelocityECEF_AU
{
	au::QuantityF<au::MetersPerSecondInEcefFrame> x, y, z;
	au::QuantityF<au::MetersPerSecondInEcefFrame> acc;
};

PositionECEF_AU ConvertPositionECEF(const PositionECEF &pos);
VelocityECEF_AU ConvertVelocityECEF(const VelocityECEF &vel);

#endif /* INC_GNSS_H_ */
