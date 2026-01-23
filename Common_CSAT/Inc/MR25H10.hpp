
#ifndef INC_MR25H10_H_
#define INC_MR25H10_H_

#include <cstdint>
#include <optional>
#include "Transport.hpp"

#ifdef __x86_64__
#include "mock_hal.h"
#endif

enum class MR25H10_COMMANDS : uint8_t
{
	MR25H10_WREN = 0x06,
	MR25H10_WRDI = 0x04,
	MR25H10_RDSR = 0x05,
	MR25H10_WRSR = 0x01,
	MR25H10_READ = 0x03,
	MR25H10_WRITE = 0x02,
	MR25H10_SLEEP = 0xB9,
	MR25H10_WAKE = 0xAB,
};

template <typename Transport>
	requires StreamModeTransport<Transport>
class MR25H10
{
public:
	MR25H10() = delete;
	explicit MR25H10(const Transport &transport) : transport_(transport) {}

	std::optional<uint8_t> readStatus() const;
	bool writeStatus(uint8_t status) const;

private:
	const Transport &transport_;
};

template <typename Transport>
	requires StreamModeTransport<Transport>
std::optional<uint8_t> MR25H10<Transport>::readStatus() const
{
	uint8_t tx[1] = {static_cast<uint8_t>(MR25H10_COMMANDS::MR25H10_RDSR)};
	uint8_t rx[1] = {0};

	if (!transport_.transfer(tx, rx, 1))
	{
		return std::nullopt;
	}

	return rx[0];
}

template <typename Transport>
	requires StreamModeTransport<Transport>
bool MR25H10<Transport>::writeStatus(uint8_t data) const
{
	uint8_t tx_buf[2] = {static_cast<uint8_t>(MR25H10_COMMANDS::MR25H10_WRSR), data};
	return transport_.write(tx_buf, sizeof(tx_buf));
}

#endif /* INC_MR25H10_H_ */
