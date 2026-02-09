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
	CanTxIrqLock::lock();
	const CanardTxQueueItem* ti = nullptr;
    while ((ti = canardTxPeek(&adapter_->que)) != nullptr)
    {
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0)
            break;

        CAN_TxHeaderTypeDef header;
        header.ExtId = ti->frame.extended_can_id;
        header.DLC   = static_cast<uint8_t>(ti->frame.payload_size);
        header.RTR   = CAN_RTR_DATA;
        header.IDE   = CAN_ID_EXT;

        uint32_t mailbox;
        HAL_CAN_AddTxMessage(hcan_, &header,
                             (uint8_t*)ti->frame.payload,
                             &mailbox);

        adapter_->ins.memory_free(
            &adapter_->ins,
            canardTxPop(&adapter_->que, ti)
        );
    }
	CanTxIrqLock::unlock();
}

void CanTxQueueDrainer::irq_safe_drain()
{
	CanTxIrqLock::lock();
	drain();
	CanTxIrqLock::unlock();
}
