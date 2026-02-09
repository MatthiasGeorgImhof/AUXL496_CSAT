#ifndef RX_PROCESSING_HPP
#define RX_PROCESSING_HPP

#include <tuple>
#include <memory>

#include "cyphal.hpp"
#include "canard_adapter.hpp"
#include "serard_adapter.hpp"
#include "loopard_adapter.hpp"

#include "CircularBuffer.hpp"
#include "ServiceManager.hpp"
#include "o1heap.h"
#include "Logger.hpp"
#include "CanTxQueueDrainer.hpp"

extern CanTxQueueDrainer tx_drainer;

constexpr size_t SERIAL_MTU = 640;
struct SerialFrame
{
    size_t size;
    uint8_t data[SERIAL_MTU];
};

constexpr size_t CAN_MTU = 8;
struct CanRxFrame
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[CAN_MTU];
};

extern CanardAdapter canard_adapter;

void drain_canard_tx_queue(CAN_HandleTypeDef *hcan)
{
    const CanardTxQueueItem* ti = nullptr;

    while ((ti = canardTxPeek(&canard_adapter.que)) != nullptr)
    {
        log(LOG_LEVEL_ERROR,
            "drain_canard_tx_queue: buffer size=%d\r\n",
            canard_adapter.que.size);

        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
            break;

        CAN_TxHeaderTypeDef header;
        header.ExtId = ti->frame.extended_can_id;
        header.DLC   = static_cast<uint8_t>(ti->frame.payload_size);
        header.RTR   = CAN_RTR_DATA;
        header.IDE   = CAN_ID_EXT;

        uint32_t mailbox;
        HAL_StatusTypeDef status =
            HAL_CAN_AddTxMessage(hcan, &header,
                                 (uint8_t*)ti->frame.payload,
                                 &mailbox);

        if (status != HAL_OK)
        {
            log(LOG_LEVEL_ERROR,
                "TX fail in callback, dropping frame extid=%08lx status=%d\r\n",
                header.ExtId, status);
        }

        canard_adapter.ins.memory_free(
            &canard_adapter.ins,
            canardTxPop(&canard_adapter.que, ti)
        );
    }
}

template <typename Allocator>
class LoopManager
{
private:
    Allocator &allocator_;

public:
    LoopManager(Allocator &allocator) : allocator_(allocator) {}

    // Common transfer processing function
    template <typename... Adapters>
    bool processTransfer(CyphalTransfer &transfer, ServiceManager *service_manager, std::tuple<Adapters...> &adapters)
    {
    	constexpr size_t BUFFER_SIZE = 512;
    	char hex_string_buffer[BUFFER_SIZE];
    	uchar_buffer_to_hex(static_cast<const unsigned char*>(transfer.payload), transfer.payload_size, hex_string_buffer, BUFFER_SIZE);
        log(LOG_LEVEL_DEBUG, "LoopManager::processTransfer: %4d %s\r\n", transfer.metadata.port_id, hex_string_buffer);

    	std::shared_ptr<CyphalTransfer> transfer_ptr = std::allocate_shared<CyphalTransfer>(allocator_, transfer);
        service_manager->handleMessage(transfer_ptr);

        bool all_successful = true;
        std::apply([&](auto &...adapter)
                   { ([&]()
                      {
            int32_t res = adapter.cyphalTxForward(static_cast<CyphalMicrosecond>(0), &transfer.metadata, transfer.payload_size, transfer.payload, CYPHAL_NODE_ID_UNSET);
            all_successful = all_successful && (res > 0); }(), ...); }, adapters);
        return all_successful; // Return success status
    }

    template <typename... Adapters>
    void CanProcessRxQueue(Cyphal<CanardAdapter> *cyphal, ServiceManager *service_manager, std::tuple<Adapters...> &adapters, CircularBuffer<CanRxFrame, 64> &can_rx_buffer)
    {
        size_t num_frames = can_rx_buffer.size();
        for (uint32_t n = 0; n < num_frames; ++n)
        {
            CanRxFrame frame = can_rx_buffer.pop();
            size_t frame_size = frame.header.DLC;

        	constexpr size_t BUFFER_SIZE = 256;
        	char hex_string_buffer[BUFFER_SIZE];
        	uchar_buffer_to_hex(frame.data, frame_size, hex_string_buffer, BUFFER_SIZE);
            log(LOG_LEVEL_DEBUG, "LoopManager::CanProcessRxQueue dump: %4x %s\r\n", frame.header.ExtId, hex_string_buffer);

            CyphalTransfer transfer;
            int32_t result = cyphal->cyphalRxReceive(frame.header.ExtId, &frame_size, frame.data, &transfer);
            if (result == 1)
            {
                processTransfer(transfer, service_manager, adapters);
            }
        }
    }

    template <typename... Adapters>
    void SerialProcessRxQueue(Cyphal<SerardAdapter> *cyphal, ServiceManager *service_manager, std::tuple<Adapters...> &adapters, CircularBuffer<SerialFrame, 4> &serial_buffer)
    {
        size_t num_frames = serial_buffer.size();
        log(LOG_LEVEL_TRACE, "LoopManager::SerialProcessRxQueue size: %d\r\n", num_frames);
        for (uint32_t n = 0; n < num_frames; ++n)
        {
            SerialFrame frame = serial_buffer.pop();
            size_t frame_size = frame.size;
            size_t shift = 0;

        	constexpr size_t BUFFER_SIZE = 256;
        	char hex_string_buffer[BUFFER_SIZE];
        	uchar_buffer_to_hex(frame.data + shift, frame_size, hex_string_buffer, BUFFER_SIZE);
            log(LOG_LEVEL_DEBUG, "LoopManager::SerialProcessRxQueue dump: %s\r\n", hex_string_buffer);

            CyphalTransfer transfer;
            for (;;)
            {
                int32_t result = cyphal->cyphalRxReceive(&frame_size, frame.data + shift, &transfer);

                if (result == 1)
                {
                    processTransfer(transfer, service_manager, adapters);
                }

                if (frame_size == 0)
                    break;
                shift = frame.size - frame_size;
            }
        }
    }

    template <typename... Adapters>
    void LoopProcessRxQueue(Cyphal<LoopardAdapter> *cyphal, ServiceManager *service_manager, std::tuple<Adapters...> &adapters)
    {
        CyphalTransfer transfer;
        while (cyphal->cyphalRxReceive(nullptr, nullptr, &transfer))
        {
            processTransfer(transfer, service_manager, adapters);
        }
    }

	void CanProcessTxQueue(CanardAdapter */*adapter*/, CAN_HandleTypeDef */*hcan*/)
	{
		tx_drainer.irq_safe_drain();
	}
};



#endif // RX_PROCESSING_HPP
