/*
 * GNSS.c
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *  Copyright 2020 SimpleMethod
 *
 *  adapted for C++ and streamlined
 *  Matthias G Imhof
 *  01.06.2025
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

#include "GNSS.hpp"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "utilities.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

union u_Short
{
	uint8_t bytes[2];
	uint16_t uShort;
} uShort;

union i_Short
{
	uint8_t bytes[2];
	int16_t iShort;
} iShort;

union u_Long
{
	uint8_t bytes[4];
	uint32_t uLong;
} uLong;

union i_Long
{
	uint8_t bytes[4];
	int32_t iLong;
} iLong;

GNSS::GNSS(UART_HandleTypeDef *huart) : huart(huart) {}

// https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf

/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
uint8_t *GNSS::FindHeader(uint8_t classID, uint8_t messageID)
{
	constexpr uint16_t SIZE = sizeof(uartBuffer) / 2;
	for (uint16_t var = 0; var <= SIZE; ++var)
	{
		if (uartBuffer[var] == 0xB5 && uartBuffer[var + 1] == 0x62)
		{
			uint16_t length = GetUShort(uartBuffer, static_cast<uint16_t>(var + 4));
			uint8_t cka_message = GetUByte(uartBuffer, static_cast<uint16_t>(var + length + UBLOX_HEADER_SIZE));
			uint8_t ckb_message = GetUByte(uartBuffer, static_cast<uint16_t>(var + length + UBLOX_HEADER_SIZE + 1));
			uint8_t cka, ckb;
			Checksum(length + 4U, &uartBuffer[var + 2], &cka, &ckb);
			if (cka != cka_message && ckb != ckb_message)
			{
				return nullptr;
			}
			if (uartBuffer[var + 2] == classID && uartBuffer[var + 3] == messageID)
			{
				return uartBuffer + var + UBLOX_HEADER_SIZE;
			}
		}
	}
	return nullptr;
}

uint8_t *GNSS::Request(const uint8_t *request, uint16_t size, uint8_t classID, uint8_t messageID)
{
	HAL_UART_Transmit(huart, const_cast<uint8_t *>(request), size, 50);
	HAL_UART_Receive(huart, uartBuffer, sizeof(uartBuffer), 500);
	return FindHeader(classID, messageID);
}

/*!
 * Make request for unique chip ID data.
 */
// UBX Class and Message ID constants
constexpr uint8_t UBX_CLASS_MON = 0x27;
constexpr uint8_t UBX_CLASS_NAV = 0x01;

constexpr uint8_t UBX_ID_UNIQID = 0x03;
constexpr uint8_t UBX_ID_TIMEUTC = 0x21;
constexpr uint8_t UBX_ID_POSLLH = 0x02;
constexpr uint8_t UBX_ID_POSECEF = 0x01;
constexpr uint8_t UBX_ID_PVT = 0x07;
constexpr uint8_t UBX_ID_VELECEF = 0x11;
constexpr uint8_t UBX_ID_VELNED = 0x12;

std::optional<UniqueID> GNSS::getUniqID()
{
	uint8_t *messageBuffer = Request(getUniqueID, sizeof(getUniqueID), UBX_CLASS_MON, UBX_ID_UNIQID);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return ParseUniqID(messageBuffer);
}

/*!
 * Make request for UTC time solution data.
 */
std::optional<UTCTime> GNSS::GetNavTimeUTC()
{
	uint8_t *messageBuffer = Request(getNavTimeUTC, sizeof(getNavTimeUTC), UBX_CLASS_NAV, UBX_ID_TIMEUTC);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return ParseNavTimeUTC(messageBuffer);
}

/*!
 * Make request for geodetic position solution data.
 */
std::optional<PositionLLH> GNSS::GetNavPosLLH()
{
	uint8_t *messageBuffer = Request(getNavPosLLH, sizeof(getNavPosLLH), UBX_CLASS_NAV, UBX_ID_POSLLH);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return ParseNavPosLLH(messageBuffer);
}

/*!
 * Make request for earth centric position solution data.
 */
std::optional<PositionECEF> GNSS::GetNavPosECEF()
{
	uint8_t *messageBuffer = Request(getNavPosECEF, sizeof(getNavPosECEF), UBX_CLASS_NAV, UBX_ID_POSECEF);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return ParseNavPosECEF(messageBuffer);
}

/*!
 * Make request for navigation position velocity time solution data.
 */
std::optional<NavigationPVT> GNSS::GetNavPVT()
{
	uint8_t *messageBuffer = Request(getNavPVT, sizeof(getNavPVT), UBX_CLASS_NAV, UBX_ID_PVT);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return ParseNavPVT(messageBuffer);
}

/*!
 * Make request for geocentric navigation velocity solution data.
 */
std::optional<VelocityNED> GNSS::GetNavVelNED()
{
	uint8_t *messageBuffer = Request(getNavVelNED, sizeof(getNavVelNED), UBX_CLASS_NAV, UBX_ID_VELNED);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return ParseNavVelNED(messageBuffer);
}

/*!
 * Make request for earth centric navigation velocity solution data.
 */
std::optional<VelocityECEF> GNSS::GetNavVelECEF()
{
	uint8_t *messageBuffer = Request(getNavVelECEF, sizeof(getNavVelECEF), UBX_CLASS_NAV, UBX_ID_VELECEF);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return ParseNavVelECEF(messageBuffer);
}

/*!
 * Changing the GNSS mode.
 */
void GNSS::SetMode(GNSSMode gnssMode)
{
	uint8_t *dataToSend = nullptr;
	size_t dataSize = 0;

	switch (gnssMode)
	{
	case 0:
		dataToSend = const_cast<uint8_t *>(setPortableMode);
		dataSize = sizeof(setPortableMode);
		break;
	case 1:
		dataToSend = const_cast<uint8_t *>(setStationaryMode);
		dataSize = sizeof(setStationaryMode);
		break;
	case 2:
		dataToSend = const_cast<uint8_t *>(setPedestrianMode);
		dataSize = sizeof(setPedestrianMode);
		break;
	case 3:
		dataToSend = const_cast<uint8_t *>(setAutomotiveMode);
		dataSize = sizeof(setAutomotiveMode);
		break;
	case 4:
		dataToSend = const_cast<uint8_t *>(setSeaMode);
		dataSize = sizeof(setSeaMode);
		break;
	case 5:
		dataToSend = const_cast<uint8_t *>(setAirbone1GMode);
		dataSize = sizeof(setAirbone1GMode);
		break;
	case 6:
		dataToSend = const_cast<uint8_t *>(setAirbone2GMode);
		dataSize = sizeof(setAirbone2GMode);
		break;
	case 7:
		dataToSend = const_cast<uint8_t *>(setAirbone4GMode);
		dataSize = sizeof(setAirbone4GMode);
		break;
	case 8:
		dataToSend = const_cast<uint8_t *>(setWristMode);
		dataSize = sizeof(setWristMode);
		break;
	case 9:
		dataToSend = const_cast<uint8_t *>(setBikeMode);
		dataSize = sizeof(setBikeMode);
		break;
	default:
		return;
	}

	if (dataToSend != nullptr)
	{
		HAL_UART_Transmit_DMA(huart, dataToSend, static_cast<uint16_t>(dataSize));
	}
}

/*!
 * Parse data to unique chip ID standard.
 */
UniqueID GNSS::ParseUniqID(uint8_t *messageBuffer)
{
	return UniqueID{
		.id = {GetUByte(messageBuffer, 4),
			   GetUByte(messageBuffer, 5),
			   GetUByte(messageBuffer, 6),
			   GetUByte(messageBuffer, 7),
			   GetUByte(messageBuffer, 8),
			   GetUByte(messageBuffer, 9)}};
}

/*!
 * Parse data to navigation position velocity time solution standard.
 */
NavigationPVT GNSS::ParseNavPVT(uint8_t *messageBuffer)
{
	return NavigationPVT{
		.utcTime = {
			.year = GetUShort(messageBuffer, 4),							  // year
			.month = GetUByte(messageBuffer, 6),							  // month
			.day = GetUByte(messageBuffer, 7),								  // day
			.hour = GetUByte(messageBuffer, 8),								  // hour
			.min = GetUByte(messageBuffer, 9),								  // min
			.sec = GetUByte(messageBuffer, 10),								  // sec
			.nano = GetILong(messageBuffer, 16),							  // nano seconds
			.tAcc = GetULong(messageBuffer, 12),							  // time accuracy
			.valid = static_cast<uint8_t>(GetUByte(messageBuffer, 11) & 0x0f) // valid flags
		},
		.position = {									// position are in mm, so we divide by 10 to get cm
			.lon = GetILong(messageBuffer, 24),			// longitude
			.lat = GetILong(messageBuffer, 28),			// latitude
			.height = GetILong(messageBuffer, 32) / 10, // height above ellipsoid
			.hMSL = GetILong(messageBuffer, 36) / 10,	// height above mean sea level
			.hAcc = GetULong(messageBuffer, 40) / 10,	// horizontal accuracy estimate
			.vAcc = GetULong(messageBuffer, 44) / 10	// vertical accuracy estimate
		},
		.velocity = {									// velocities are in mm/s, so we divide by 10 to get cm/s
			.velN = GetILong(messageBuffer, 48) / 10, 	// north velocity component
			.velE = GetILong(messageBuffer, 52) / 10, 	// east velocity component
			.velD = GetILong(messageBuffer, 56) / 10, 	// down velocity component
			.headMot = GetILong(messageBuffer, 64),	  	// heading of motion (2-D)
			.speed = 0,
			.gSpeed = GetULong(messageBuffer, 60) / 10, // ground speed estimate
			.sAcc = GetULong(messageBuffer, 68),		// speed accuracy estimate
			.headAcc = GetULong(messageBuffer, 72)		// heading accuracy estimate
		},
		.fixType = GetIByte(messageBuffer, 20), // fix type
		.numSV = GetUByte(messageBuffer, 23)	// number of satellites used
	};
}

/*!
 * Parse data to UTC time solution standard.
 */
UTCTime GNSS::ParseNavTimeUTC(uint8_t *messageBuffer)
{
	return UTCTime{
		.year = GetUShort(messageBuffer, 12),							  // year
		.month = GetUByte(messageBuffer, 14),							  // month
		.day = GetUByte(messageBuffer, 15),								  // day
		.hour = GetUByte(messageBuffer, 16),							  // hour
		.min = GetUByte(messageBuffer, 17),								  // min
		.sec = GetUByte(messageBuffer, 18),								  // sec
		.nano = GetILong(messageBuffer, 8),								  // nano seconds
		.tAcc = GetULong(messageBuffer, 4),								  // time accuracy
		.valid = static_cast<uint8_t>(GetUByte(messageBuffer, 19) & 0x0f) // valid flags
	};
}

/*!
 * Parse data to geodetic position solution standard.
 */
PositionLLH GNSS::ParseNavPosLLH(uint8_t *messageBuffer)
{
	return PositionLLH{
		.lon = GetILong(messageBuffer, 4),	   // lon
		.lat = GetILong(messageBuffer, 8),	   // lat
		.height = GetILong(messageBuffer, 12), // height
		.hMSL = GetILong(messageBuffer, 16),   // hMSL
		.hAcc = GetULong(messageBuffer, 20),   // hAcc
		.vAcc = GetULong(messageBuffer, 24)	   // vAcc
	};
}

/*!
 * Parse data to earth centric position solution standard.
 */
PositionECEF GNSS::ParseNavPosECEF(uint8_t *messageBuffer)
{
	return PositionECEF{
		.ecefX = GetILong(messageBuffer, 4),  // ecefX
		.ecefY = GetILong(messageBuffer, 8),  // ecefY
		.ecefZ = GetILong(messageBuffer, 12), // ecefZ
		.pAcc = GetULong(messageBuffer, 16)	  // pAcc
	};
}

/*!
 * Parse data to geodetic velocity solution standard.
 */
VelocityNED GNSS::ParseNavVelNED(uint8_t *messageBuffer)
{
	return VelocityNED{
		.velN = GetILong(messageBuffer, 4),		// velN
		.velE = GetILong(messageBuffer, 8),		// velE
		.velD = GetILong(messageBuffer, 12),	// velD
		.headMot = GetILong(messageBuffer, 24), // headMot
		.speed = GetULong(messageBuffer, 16),	// gSpeed
		.gSpeed = GetULong(messageBuffer, 20),	// gSpeed
		.sAcc = GetULong(messageBuffer, 28),	// sAcc
		.headAcc = GetULong(messageBuffer, 32)	// headAcc
	};
}

/*!
 * Parse data to earth centric velocity solution standard.
 */
VelocityECEF GNSS::ParseNavVelECEF(uint8_t *messageBuffer)
{
	return VelocityECEF{
		.ecefVX = GetILong(messageBuffer, 4),  // velX
		.ecefVY = GetILong(messageBuffer, 8),  // velY
		.ecefVZ = GetILong(messageBuffer, 12), // velZ
		.sAcc = GetULong(messageBuffer, 16)	   // sAcc
	};
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 */
void GNSS::LoadConfig()
{
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(configUBX), sizeof(configUBX));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(setNMEA410), sizeof(setNMEA410));
	HAL_Delay(250);
	HAL_UART_Transmit_DMA(huart, const_cast<uint8_t *>(setGNSS), sizeof(setGNSS));
	HAL_Delay(250);
}

/*!
 *  Creates a checksum based on UBX standard.
 */
void GNSS::Checksum(uint16_t length, const uint8_t *payload, uint8_t *cka, uint8_t *ckb)
{
	uint8_t cka_ = 0;
	uint8_t ckb_ = 0;

	for (int i = 0; i < length; ++i)
	{
		cka_ += payload[i];
		ckb_ += cka_;
	}
	*cka = cka_;
	*ckb = ckb_;
}

uint8_t GNSS::GetUByte(const uint8_t *messageBuffer, uint16_t offset)
{
	return static_cast<uint8_t>(messageBuffer[offset]);
}

int8_t GNSS::GetIByte(const uint8_t *messageBuffer, uint16_t offset)
{
	return static_cast<int8_t>(messageBuffer[offset]);
}

uint16_t GNSS::GetUShort(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 2; ++var)
	{
		uShort.bytes[var] = messageBuffer[var + offset];
	}
	return uShort.uShort;
}

int16_t GNSS::GetIShort(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 2; ++var)
	{
		iShort.bytes[var] = messageBuffer[var + offset];
	}
	return iShort.iShort;
}

uint32_t GNSS::GetULong(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 4; ++var)
	{
		uLong.bytes[var] = messageBuffer[var + offset];
	}
	return uLong.uLong;
}

int32_t GNSS::GetILong(const uint8_t *messageBuffer, uint16_t offset)
{
	for (uint16_t var = 0; var < 4; ++var)
	{
		iLong.bytes[var] = messageBuffer[var + offset];
	}
	return iLong.iLong;
}

PositionECEF_AU ConvertPositionECEF(const PositionECEF &pos)
{
	return PositionECEF_AU {
		.x = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.ecefX)),
		.y = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.ecefY)),
		.z = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.ecefZ)),
		.acc = au::make_quantity<au::Centi<au::MetersInEcefFrame>>(static_cast<float>(pos.pAcc))};
}

VelocityECEF_AU ConvertVelocityECEF(const VelocityECEF &vel)
{
	return VelocityECEF_AU {
		.x = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.ecefVX)),
		.y = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.ecefVY)),
		.z = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.ecefVZ)),
		.acc = au::make_quantity<au::Centi<au::MetersPerSecondInEcefFrame>>(static_cast<float>(vel.sAcc))};
}

