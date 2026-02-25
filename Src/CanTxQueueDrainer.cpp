#include "CanTxQueueDrainer.hpp"
#include "canard_adapter.hpp"
#include "Logger.hpp"
#include "IRQLock.hpp"

CanTxQueueDrainer::CanTxQueueDrainer(CanardAdapter* adapter,
                                     CAN_HandleTypeDef* hcan)
    : adapter_(adapter), hcan_(hcan)
{}

void CanTxQueueDrainer::drain()
{
	const CanardTxQueueItem* ti = nullptr;
    while ((ti = canardTxPeek(&adapter_->que)) != nullptr)
    {
    	auto num_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(hcan_);
    	if (num_mailboxes == 0)
            break;

        CAN_TxHeaderTypeDef header;
        header.ExtId = ti->frame.extended_can_id;
        header.DLC   = static_cast<uint8_t>(ti->frame.payload_size);
        header.RTR   = CAN_RTR_DATA;
        header.IDE   = CAN_ID_EXT;

        uint32_t mailbox;
        HAL_CAN_AddTxMessage(hcan_, &header, (uint8_t*)ti->frame.payload, &mailbox);

        CyphalHeader cyphal_header = parse_header(header.ExtId);
        uint8_t transfer_id = reinterpret_cast<const uint8_t*>(ti->frame.payload)[header.DLC-1];
        log(LOG_LEVEL_INFO, "CanTxQueueDrainer mailbox %d of %d available: %3d -> %3d subject %3d transfer_id %2x\r\n",
        			mailbox, num_mailboxes, cyphal_header.source_id, cyphal_header.destination_id, cyphal_header.port_id, transfer_id);

        adapter_->ins.memory_free(&adapter_->ins, canardTxPop(&adapter_->que, ti));
    }

//    if (adapter_->que.size > 0 && HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0)
//    {
//    	__HAL_CAN_ENABLE_IT(hcan_, CAN_IT_TX_MAILBOX_EMPTY);
//    }
//    else
//    {
//    	__HAL_CAN_DISABLE_IT(hcan_, CAN_IT_TX_MAILBOX_EMPTY);
//    }
}

void CanTxQueueDrainer::irq_safe_drain()
{
//	CanTxIrqLock::lock();
	drain();
//	CanTxIrqLock::unlock();
}

// #ifdef __x86_64__
// CanTxQueueDrainer tx_drainer;
// #endif // __x86_64__
