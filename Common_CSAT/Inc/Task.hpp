#ifndef INC_TASK_HPP_
#define INC_TASK_HPP_

#include <cstdint>
#include <memory>
#include <tuple>

#include "cyphal.hpp"
#include <BufferLikeConcept.hpp>
#include <CircularBuffer.hpp>
#include <SingleSlotBuffer.hpp>
#include "Logger.hpp"

#ifdef __arm__
#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#endif

#ifdef __x86_64__
#include "mock_hal.h"
#endif

class RegistrationManager;

//
// baseclass
//

class Task
{
public:
	Task() = delete;
	Task(uint32_t interval, uint32_t tick) : interval_(interval), last_tick_(0), shift_(tick) {}
	virtual ~Task() = default;

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

	// Virtual function with a default implementation that does nothing
	virtual void handleMessage(std::shared_ptr<CyphalTransfer> /* transfer */) {}

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

//
// traits
//

class TaskPacing
{
public:
    TaskPacing(uint32_t sleep_interval, uint32_t operate_interval)
        : sleep_interval_(sleep_interval),
          operate_interval_(operate_interval)
    {}

protected:
    void sleep(Task& task)
    {
        task.setInterval(sleep_interval_);
    }

    void operate(Task& task)
    {
        task.setInterval(operate_interval_);
    }

private:
    uint32_t sleep_interval_;
    uint32_t operate_interval_;
};

template <typename... Adapters>
class Publisher
{
public:
	Publisher(std::tuple<Adapters...> &adapters) : adapters_(adapters) {}
	virtual ~Publisher() {}

protected:
	void publishImpl(size_t payload_size, uint8_t *payload, void *data,
					 int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
					 CyphalPortID port_id,
					 CyphalTransferKind transfer_kind,
					 CyphalNodeID node_id,
					 CyphalTransferID transfer_id)
	{
		int8_t result = serialize(data, payload, &payload_size);
		if (result < 0)
		{
			log(LOG_LEVEL_ERROR, "ERROR Task.publish serialization result %d with size %d \r\n", result, payload_size);
			return;
		}
		else
		{
			log(LOG_LEVEL_DEBUG, "Task.publish serialization result %d with size %d \r\n", result, payload_size);
		}
		CyphalTransferMetadata metadata =
			{
				CyphalPriorityNominal,
				transfer_kind,
				port_id,
				node_id,
				CYPHAL_NODE_ID_UNSET,
				CYPHAL_NODE_ID_UNSET,
				transfer_id,
			};

		bool all_successful = true;
		std::apply([&](auto &...adapter)
				   { ([&]()
					  {
                int32_t res = adapter.cyphalTxPush(static_cast<CyphalMicrosecond>(0), &metadata, payload_size, payload);
                all_successful = all_successful && (res > 0); }(), ...); }, adapters_);
		if (!all_successful)
			log(LOG_LEVEL_ERROR, "ERROR Task.publish push\r\n");
	}

protected:
	std::tuple<Adapters...> &adapters_;
};

typedef SingleSlotBuffer<std::shared_ptr<CyphalTransfer>> CyphalBuffer1;
typedef CircularBuffer<std::shared_ptr<CyphalTransfer>, 2> CyphalBuffer2;
typedef CircularBuffer<std::shared_ptr<CyphalTransfer>, 4> CyphalBuffer4;
typedef CircularBuffer<std::shared_ptr<CyphalTransfer>, 8> CyphalBuffer8;
typedef CircularBuffer<std::shared_ptr<CyphalTransfer>, 16> CyphalBuffer16;
typedef CircularBuffer<std::shared_ptr<CyphalTransfer>, 32> CyphalBuffer32;
typedef CircularBuffer<std::shared_ptr<CyphalTransfer>, 64> CyphalBuffer64;

template <typename Buffer>
class Receiver
{
	static_assert(BufferLike<Buffer, std::shared_ptr<CyphalTransfer>>, "Buffer must satisfy BufferLike");

public:
	Receiver() {}
	virtual ~Receiver() {}

	virtual void handleMessageImpl(std::shared_ptr<CyphalTransfer> transfer)
	{
		buffer_.push(transfer);
	}

protected:
	Buffer buffer_;
};

//
// Message Publication
//

template <typename... Adapters>
class TaskWithPublication : public Task, public Publisher<Adapters...>
{
public:
	TaskWithPublication() = delete;
	TaskWithPublication(uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters)
		: Task(interval, tick), Publisher<Adapters...>(adapters), transfer_id_(transfer_id) {}
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
		Publisher<Adapters...>::publishImpl(payload_size, payload, data, serialize, port_id, CyphalTransferKindMessage, CYPHAL_NODE_ID_UNSET, transfer_id_);
	}

private:
	CyphalTransferID transfer_id_;
};

//
// Message Reception
//

template <typename Buffer>
class TaskFromBuffer : public Task, public Receiver<Buffer>
{
public:
	TaskFromBuffer() = delete;
	TaskFromBuffer(uint32_t interval, uint32_t tick) : Task(interval, tick) {}
	virtual ~TaskFromBuffer() {};

	void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override
	{
		this->handleMessageImpl(transfer);
	}
};

//
// Message Publish and Receive
//

template <typename Buffer, typename... Adapters>
class TaskPublishReceive : public Task, public Publisher<Adapters...>, public Receiver<Buffer>
{
public:
	TaskPublishReceive() = delete;
	TaskPublishReceive(uint32_t interval, uint32_t tick, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters)
		: Task(interval, tick), Publisher<Adapters...>(adapters), transfer_id_(transfer_id) {}
	virtual ~TaskPublishReceive() {};

	void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override
	{
		this->handleMessageImpl(transfer);
	}

protected:
	virtual void update(uint32_t now) override
	{
		Task::update(now);
		++transfer_id_;
	}

	void publish(size_t payload_size, uint8_t *payload, void *data,
						int8_t (*serialize)(const void *const, uint8_t *const, size_t *const), CyphalPortID port_id)
	{
		Publisher<Adapters...>::publishImpl(payload_size, payload, data, serialize, port_id, CyphalTransferKindMessage, CYPHAL_NODE_ID_UNSET, transfer_id_);
	}

private:
	CyphalTransferID transfer_id_;
};

//
// Server Respond: Receive and Publish
//

template <typename Buffer, typename... Adapters>
class TaskForServer : public Task, public Receiver<Buffer>, public Publisher<Adapters...>
{
public:
	TaskForServer() = delete;
	TaskForServer(uint32_t interval, uint32_t tick, std::tuple<Adapters...> &adapters)
		: Task(interval, tick), Publisher<Adapters...>(adapters) {}
	virtual ~TaskForServer() {};

	void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override
	{
		this->handleMessageImpl(transfer);
	}

protected:
	void publish(size_t payload_size, uint8_t *payload, void *data,
						int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
						CyphalPortID port_id, CyphalNodeID node_id, CyphalTransferID transfer_id)
	{
		Publisher<Adapters...>::publishImpl(payload_size, payload, data, serialize, port_id, CyphalTransferKindResponse, node_id, transfer_id);
	}
};

//
// Client Request: Publish and Receive
//

template <typename Buffer, typename... Adapters>
class TaskForClient : public Task, public Receiver<Buffer>, public Publisher<Adapters...>
{
public:
	TaskForClient() = delete;
	TaskForClient(uint32_t interval, uint32_t tick, CyphalNodeID node_id, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters)
		: Task(interval, tick), Publisher<Adapters...>(adapters), node_id_(node_id), transfer_id_(transfer_id) {}
	virtual ~TaskForClient() {};

	void handleMessage(std::shared_ptr<CyphalTransfer> transfer) override
	{
		this->handleMessageImpl(transfer);
	}


protected:
	virtual void update(uint32_t now) override
	{
		Task::update(now);
		++transfer_id_;
	}

	void publish(size_t payload_size, uint8_t *payload, void *data,
						int8_t (*serialize)(const void *const, uint8_t *const, size_t *const),
						CyphalPortID port_id, CyphalNodeID node_id)
	{
		Publisher<Adapters...>::publishImpl(payload_size, payload, data, serialize, port_id, CyphalTransferKindRequest, node_id, transfer_id_);
	}

protected:
	CyphalNodeID node_id_;
	CyphalTransferID transfer_id_;
};

//
// helper functionality
//

typedef struct
{
	CyphalPortID port_id;
	std::shared_ptr<Task> task;
} TaskHandler;

constexpr CyphalPortID PURE_HANDLER = 0;
inline bool is_valid(CyphalPortID port_id) { return port_id > 0 && port_id < 8192 && port_id != PURE_HANDLER; }

#endif /* INC_TASK_HPP_ */
