#ifndef INC_TASK_HPP_
#define INC_TASK_HPP_

#include <cstdint>
#include <memory>
#include <tuple>

#include "cyphal.hpp"
#include <CircularBuffer.hpp>
#include <Logger.hpp>

#ifdef __arm__
#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#endif

#ifdef __x86_64__
#include "mock_hal.h"
#endif

class RegistrationManager;

class Task
{
public:
	Task() = delete;
	Task(uint32_t interval, uint32_t tick) : interval_(interval), last_tick_(0), shift_(tick) {}
	virtual ~Task() {};

	uint32_t getInterval() const { return interval_; }
	uint32_t getShift() const { return shift_; }
	uint32_t getLastTick() const { return last_tick_; }

	void setInterval(uint32_t interval) { interval_ = interval; }
	void setShift(uint32_t shift) { shift_ = shift; }
	void setLastTick(uint32_t last) { last_tick_ = last; }
	void initialize(uint32_t now) { last_tick_ = now + shift_; }

protected:
	bool check() { return HAL_GetTick() >= interval_ + last_tick_; }
	virtual void update(uint32_t now) { last_tick_ = now; }

public:
	void handleTask()
	{
		if (check())
		{
			handleTaskImpl();
			update(HAL_GetTick());
		}
	}
	virtual void handleMessage(std::shared_ptr<CyphalTransfer> transfer) = 0;

	// Non-template virtual function (using type erasure with a base class)
	virtual void registerTask(RegistrationManager *manager, std::shared_ptr<Task> task) = 0;
	virtual void unregisterTask(RegistrationManager *manager, std::shared_ptr<Task> task) = 0;

protected:
	virtual void handleTaskImpl() = 0;

protected:
	uint32_t interval_;
	uint32_t last_tick_;
	uint32_t shift_;
};

template <typename... Adapters>
class TaskWithPublication : public Task
{
public:
	TaskWithPublication() = delete;
	TaskWithPublication(uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : Task(interval, tick), transfer_id_(transfer_id), adapters_(adapters) {}
	virtual ~TaskWithPublication() {};

	uint8_t getTransferId() const { return transfer_id_; }

	void setTransferId(uint8_t transfer_id) { transfer_id_ = transfer_id; }

protected:
	virtual void update(uint32_t now) override
	{
		Task::update(now);
		++transfer_id_;
	}

	void publish(size_t payload_size, uint8_t *payload, void *data,
				 int8_t (*serialize)(const void *const, uint8_t *const, size_t *const), CyphalPortID port_id)
	{
		int8_t result = serialize(data, payload, &payload_size);
		if (result < 0)
			log(LOG_LEVEL_ERROR, "ERROR Task.publish serialization %d\r\n", result);

		CyphalTransferMetadata metadata =
			{
				CyphalPriorityNominal,
				CyphalTransferKindMessage,
				port_id,
				CYPHAL_NODE_ID_UNSET,
				transfer_id_,
			};

		bool all_successful = true;
		std::apply([&](auto &...adapter)
				   { ([&]()
					  {
				int8_t res = adapter.cyphalTxPush(static_cast<CyphalMicrosecond>(0), &metadata, payload_size, payload);
				all_successful = all_successful && (res > 0); }(), ...); }, adapters_);
		if (!all_successful)
			log(LOG_LEVEL_ERROR, "ERROR Task.publish push\r\n");
	}

protected:
	CyphalTransferID transfer_id_;
	std::tuple<Adapters...> &adapters_;
};

constexpr size_t CIRC_BUF_SIZE = 64;
typedef CircularBuffer<std::shared_ptr<CyphalTransfer>, CIRC_BUF_SIZE> CyphalBuffer;

class TaskFromBuffer : public Task
{
public:
	TaskFromBuffer() = delete;
	TaskFromBuffer(uint32_t interval, uint32_t tick) : Task(interval, tick) {}
	virtual ~TaskFromBuffer() {};

	virtual void handleMessage(std::shared_ptr<CyphalTransfer> transfer) { buffer.push(transfer); }

protected:
	CyphalBuffer buffer;
};

template <typename... Adapters>
class TaskForServer : public Task
{
public:
	TaskForServer() = delete;
	TaskForServer(uint32_t interval, uint32_t tick, std::tuple<Adapters...> &adapters) : Task(interval, tick), adapters_(adapters) {}
	virtual ~TaskForServer() {};

	virtual void handleMessage(std::shared_ptr<CyphalTransfer> transfer) { buffer.push(transfer); }

protected:
	void publish(size_t payload_size, uint8_t *payload, void *data,
				 int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
				 CyphalPortID port_id, CyphalNodeID node_id, CyphalTransferID transfer_id)
	{
		int8_t result = serialize(data, payload, &payload_size);
		if (result < 0)
			log(LOG_LEVEL_ERROR, "ERROR Task.publish serialization %d\r\n", result);

		CyphalTransferMetadata metadata =
			{
				CyphalPriorityNominal,
				CyphalTransferKindResponse,
				port_id,
				node_id,
				transfer_id,
			};

		bool all_successful = true;
		std::apply([&](auto &...adapter)
				   { ([&]()
					  {
				int8_t res = adapter.cyphalTxPush(static_cast<CyphalMicrosecond>(0), &metadata, payload_size, payload);
				all_successful = all_successful && (res > 0); }(), ...); }, adapters_);
		if (!all_successful)
			log(LOG_LEVEL_ERROR, "ERROR Task.publish push\r\n");
	}

protected:
	std::tuple<Adapters...> &adapters_;
	CyphalBuffer buffer;
};

typedef struct
{
	CyphalPortID port_id;
	std::shared_ptr<Task> task;
} TaskHandler;

constexpr CyphalPortID PURE_HANDLER = 0;
inline bool is_valid(CyphalPortID port_id) { return port_id > 0 && port_id < 8192 && port_id != PURE_HANDLER; }

#endif /* INC_TASK_HPP_ */
