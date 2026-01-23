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

#include "GNSSCore.hpp"
#include "Transport.hpp"

#ifdef __arm__
#include "usb_device.h"
#include "usbd_cdc_if.h"
#elif __x86_64__
#include "mock_hal.h"
#endif

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

enum GNSSMode : uint8_t
{
	Portable = 0,
	Stationary = 1,
	Pedestrian = 2,
	Automotive = 3,
	Sea = 4,
	Airborne1G = 5,
	Airborne2G = 6,
	Airborne4G = 7,
	Wrist = 8,
	Bike = 9
};

enum UBXClass : uint8_t
{
	MON = 0x27,
	NAV = 0x01
};

enum UBXMessageID : uint8_t
{
	UNIQ_ID = 0x03,
	UTC_TIME = 0x21,
	POS_LLH = 0x02,
	POS_ECEF = 0x01,
	PVT = 0x07,
	VEL_ECEF = 0x11,
	VEL_NED = 0x12
};

template <typename Transport>
	requires StreamAccessTransport<Transport>
class GNSS
{
public:
	GNSS() = delete;
	GNSS(const Transport &transport) : transport_(transport) {}

	void setMode(GNSSMode gnssMode);
	std::optional<UniqueID> getUniqID();
	std::optional<UTCTime> getNavTimeUTC();
	std::optional<PositionLLH> getNavPosLLH();
	std::optional<PositionECEF> getNavPosECEF();
	std::optional<NavigationPVT> getNavPVT();
	std::optional<VelocityECEF> getNavVelECEF();
	std::optional<VelocityNED> getNavVelNED();

private:
	void loadConfig();
	uint8_t *findHeader(UBXClass classID, UBXMessageID messageID);
	uint8_t *request(const uint8_t *request, uint16_t size, UBXClass classID, UBXMessageID messageID);

private:
	static constexpr uint16_t GNSS_BUFFER_SIZE = 201U;
	static constexpr uint16_t UBLOX_HEADER_SIZE = 6U;

private:
	const Transport &transport_;
	uint8_t uart_buffer[GNSS_BUFFER_SIZE];

};

class SimulatedGNSS
{
public:
    SimulatedGNSS() = delete;
    explicit SimulatedGNSS(int32_t error_meters = 100);

    std::optional<PositionECEF> getNavPosECEF();

private:
	int32_t noise() const;

private:
    int32_t error_meters;
};

#
#
#

// https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf

/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
uint8_t *GNSS<Transport>::findHeader(UBXClass classID, UBXMessageID messageID)
{
	constexpr uint16_t SIZE = sizeof(uart_buffer) / 2;
	for (uint16_t var = 0; var <= SIZE; ++var)
	{
		if (uart_buffer[var] == 0xB5 && uart_buffer[var + 1] == 0x62)
		{
			uint16_t length = GNSSCore::getUShort(uart_buffer, static_cast<uint16_t>(var + 4));
			uint8_t cka_message = GNSSCore::getUByte(uart_buffer, static_cast<uint16_t>(var + length + UBLOX_HEADER_SIZE));
			uint8_t ckb_message = GNSSCore::getUByte(uart_buffer, static_cast<uint16_t>(var + length + UBLOX_HEADER_SIZE + 1));
			uint8_t cka, ckb;
			GNSSCore::checksum(length + 4U, &uart_buffer[var + 2], &cka, &ckb);
			if (cka != cka_message && ckb != ckb_message)
			{
				return nullptr;
			}
			if (uart_buffer[var + 2] == static_cast<uint8_t>(classID) && uart_buffer[var + 3] == static_cast<uint8_t>(messageID))
			{
				return uart_buffer + var + UBLOX_HEADER_SIZE;
			}
		}
	}
	return nullptr;
}

template <typename Transport>
	requires StreamAccessTransport<Transport>
uint8_t *GNSS<Transport>::request(const uint8_t *request, uint16_t size, UBXClass classID, UBXMessageID messageID)
{
	transport_.write(const_cast<uint8_t *>(request), size);
	transport_.read(uart_buffer, sizeof(uart_buffer));
	return findHeader(classID, messageID);
}

template <typename Transport>
	requires StreamAccessTransport<Transport>
std::optional<UniqueID> GNSS<Transport>::getUniqID()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_UNIQUE_ID, sizeof(GNSSCore::GET_UNIQUE_ID), UBXClass::MON, UBXMessageID::UNIQ_ID);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseUniqID(messageBuffer);
}

/*!
 * Make request for UTC time solution data.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
std::optional<UTCTime> GNSS<Transport>::getNavTimeUTC()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_TIME_UTC, sizeof(GNSSCore::GET_NAV_TIME_UTC), UBXClass::NAV, UBXMessageID::UTC_TIME);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavTimeUTC(messageBuffer);
}

/*!
 * Make request for geodetic position solution data.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
std::optional<PositionLLH> GNSS<Transport>::getNavPosLLH()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_POS_LLH, sizeof(GNSSCore::GET_NAV_POS_LLH), UBXClass::NAV, UBXMessageID::POS_LLH);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPosLLH(messageBuffer);
}

/*!
 * Make request for earth centric position solution data.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
std::optional<PositionECEF> GNSS<Transport>::getNavPosECEF()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_POS_ECEF, sizeof(GNSSCore::GET_NAV_POS_ECEF), UBXClass::NAV, UBXMessageID::POS_ECEF);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPosECEF(messageBuffer);
}

/*!
 * Make request for navigation position velocity time solution data.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
std::optional<NavigationPVT> GNSS<Transport>::getNavPVT()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_PVT, sizeof(GNSSCore::GET_NAV_PVT), UBXClass::NAV, UBXMessageID::PVT);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavPVT(messageBuffer);
}

/*!
 * Make request for geocentric navigation velocity solution data.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
std::optional<VelocityNED> GNSS<Transport>::getNavVelNED()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_VEL_NED, sizeof(GNSSCore::GET_NAV_VEL_NED), UBXClass::NAV, UBXMessageID::VEL_NED);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavVelNED(messageBuffer);
}

/*!
 * Make request for earth centric navigation velocity solution data.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
std::optional<VelocityECEF> GNSS<Transport>::getNavVelECEF()
{
	uint8_t *messageBuffer = request(GNSSCore::GET_NAV_VEL_ECEF, sizeof(GNSSCore::GET_NAV_VEL_ECEF), UBXClass::NAV, UBXMessageID::VEL_ECEF);
	if (messageBuffer == nullptr)
		return std::nullopt;
	return GNSSCore::parseNavVelECEF(messageBuffer);
}

/*!
 * Changing the GNSS mode.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
void GNSS<Transport>::setMode(GNSSMode gnssMode)
{
	uint8_t *dataToSend = nullptr;
	size_t dataSize = 0;

	switch (gnssMode)
	{
	case GNSSMode::Portable:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_PORTABLE_MODE);
		dataSize = sizeof(GNSSCore::SET_PORTABLE_MODE);
		break;
	case GNSSMode::Stationary:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_STATIONARY_MODE);
		dataSize = sizeof(GNSSCore::SET_STATIONARY_MODE);
		break;
	case GNSSMode::Pedestrian:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_PEDESTRIAN_MODE);
		dataSize = sizeof(GNSSCore::SET_PEDESTRIAN_MODE);
		break;
	case GNSSMode::Automotive:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AUTOMOTIVE_MODE);
		dataSize = sizeof(GNSSCore::SET_AUTOMOTIVE_MODE);
		break;
	case GNSSMode::Sea:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_SEA_MODE);
		dataSize = sizeof(GNSSCore::SET_SEA_MODE);
		break;
	case GNSSMode::Airborne1G:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AIRBORNE_1G_MODE);
		dataSize = sizeof(GNSSCore::SET_AIRBORNE_1G_MODE);
		break;
	case GNSSMode::Airborne2G:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AIRBORNE_2G_MODE);
		dataSize = sizeof(GNSSCore::SET_AIRBORNE_2G_MODE);
		break;
	case GNSSMode::Airborne4G:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_AIRBORNE_4G_MODE);
		dataSize = sizeof(GNSSCore::SET_AIRBORNE_4G_MODE);
		break;
	case GNSSMode::Wrist:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_WRIST_MODE);
		dataSize = sizeof(GNSSCore::SET_WRIST_MODE);
		break;
	case GNSSMode::Bike:
		dataToSend = const_cast<uint8_t *>(GNSSCore::SET_BIKE_MODE);
		dataSize = sizeof(GNSSCore::SET_BIKE_MODE);
		break;
	default:
		return;
	}

	if (dataToSend != nullptr)
	{
		transport_.write(dataToSend, static_cast<uint16_t>(dataSize));
	}
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 */
template <typename Transport>
	requires StreamAccessTransport<Transport>
void GNSS<Transport>::loadConfig()
{
	transport_.write(const_cast<uint8_t *>(GNSSCore::CONFIG_UBX), sizeof(GNSSCore::CONFIG_UBX));
	HAL_Delay(250);
	transport_.write(const_cast<uint8_t *>(GNSSCore::SET_NMEA_410), sizeof(GNSSCore::SET_NMEA_410));
	HAL_Delay(250);
	transport_.write(const_cast<uint8_t *>(GNSSCore::SET_GNSS), sizeof(GNSSCore::SET_GNSS));
	HAL_Delay(250);
}



#endif /* INC_GNSS_H_ */
