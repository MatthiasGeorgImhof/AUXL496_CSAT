#include <cppmain.h>
#include <cpphal.h>
#include "usb_device.h"

#include <memory>

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "usbd_cdc_if.h"

#include "o1heap.h"
#include "canard.h"
#include "serard.h"

#include "cyphal.hpp"
#include "canard_adapter.hpp"
#include "serard_adapter.hpp"
#include "loopard_adapter.hpp"

#ifndef NUNAVUT_ASSERT
#define NUNAVUT_ASSERT(x) assert(x)
#endif

#include "cyphal_subscriptions.hpp"
#include <CircularBuffer.hpp>
#include <ArrayList.hpp>
#include "HeapAllocation.hpp"
#include "RegistrationManager.hpp"
#include "SubscriptionManager.hpp"
#include "ServiceManager.hpp"
#include "ProcessRxQueue.hpp"
#include "CanTxQueueDrainer.hpp"
#include "TaskCheckMemory.hpp"
#include "TaskBlinkLED.hpp"
#include "TaskSendHeartBeat.hpp"
#include "TaskProcessHeartBeat.hpp"
#include "TaskSendNodePortList.hpp"
#include "TaskSubscribeNodePortList.hpp"
#include "TaskRespondGetInfo.hpp"
#include "TaskRequestGetInfo.hpp"

#include "TrivialImageBuffer.hpp"
#include "TaskSyntheticImageGenerator.hpp"
#include "Trigger.hpp"
#include "InputOutputStream.hpp"
#include "TaskRequestWrite.hpp"

#include <cppmain.h>
#include "Logger.hpp"

constexpr size_t O1HEAP_SIZE = 65536;
using LocalHeap = HeapAllocation<O1HEAP_SIZE>;

CanardAdapter canard_adapter;
SerardAdapter serard_adapter;
LoopardAdapter loopard_adapter;

CanTxQueueDrainer tx_drainer(&canard_adapter, &hcan1);

#ifndef CYPHAL_NODE_ID
#define CYPHAL_NODE_ID 11
#endif

constexpr CyphalNodeID cyphal_node_id = CYPHAL_NODE_ID;

constexpr uint32_t SERIAL_TIMEOUT = 1000;
constexpr size_t SERIAL_BUFFER_SIZE = 4;
using SerialCircularBuffer = CircularBuffer<SerialFrame, SERIAL_BUFFER_SIZE>;
SerialCircularBuffer serial_buffer;

constexpr size_t CAN_RX_BUFFER_SIZE = 64;
using CanCircularBuffer = CircularBuffer<CanRxFrame, CAN_RX_BUFFER_SIZE>;
CanCircularBuffer can_rx_buffer;

CanardTransferMetadata convert(const SerardTransferMetadata serard)
{
	CanardTransferMetadata canard;
	canard.port_id = serard.port_id;
	canard.priority = static_cast<CanardPriority>(serard.priority);
	canard.remote_node_id = serard.remote_node_id;
	canard.transfer_id = serard.transfer_id;
	canard.transfer_kind = static_cast<CanardTransferKind>(serard.transfer_kind);
	return canard;
}

CanardRxTransfer convert(const SerardRxTransfer serard)
{
	CanardRxTransfer canard;
	canard.metadata = convert(serard.metadata);
	canard.payload_size = serard.payload_size;
	canard.timestamp_usec = serard.timestamp_usec;
	canard.payload = serard.payload;
	return canard;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos)
{
    if (huart->Instance == USART2)
    {
        // Finalize in-flight slot
        SerialFrame& slot = serial_buffer.begin_write();
        slot.size = pos;
        serial_buffer.commit_write();

        // Reserve next slot for DMA
        SerialFrame& next_slot = serial_buffer.begin_write();
        HAL_UARTEx_ReceiveToIdle_DMA(huart, next_slot.data, SERIAL_MTU);
        HAL_GPIO_TogglePin(GPIOC, LED2_Pin);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t num_messages = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
//	log(LOG_LEVEL_TRACE, "HAL_CAN_RxFifo0MsgPendingCallback %d\r\n", num_messages);
	for(uint32_t n=0; n<num_messages; ++n)
	{
		if (can_rx_buffer.is_full()) return;

		CanRxFrame &frame = can_rx_buffer.next();
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &frame.header, frame.data);
//		log(LOG_LEVEL_DEBUG, "HAL_CAN_RxFifo0MsgPendingCallback %x\r\n", frame.header.ExtId);
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { drain_canard_tx_queue(hcan); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { drain_canard_tx_queue(hcan); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { drain_canard_tx_queue(hcan); }
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) { drain_canard_tx_queue(hcan); }
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) { drain_canard_tx_queue(hcan); }
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) { drain_canard_tx_queue(hcan); }

#ifdef __cplusplus
extern "C" {
#endif
bool serial_send(void* /*user_reference*/, uint8_t data_size, const uint8_t* data)
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, data, data_size, 1000);
	return status == HAL_OK;;
}
#ifdef __cplusplus
}
#endif

constexpr uint16_t endian_swap(uint16_t num) {return (num>>8) | (num<<8); };
constexpr int16_t endian_swap(int16_t num) {return (num>>8) | (num<<8); };

template<typename T, typename... Args>
static void register_task_with_heap(RegistrationManager& rm, Args&&... args)
{
    static SafeAllocator<T, LocalHeap> alloc;
    rm.add(alloc_unique_custom<T>(alloc, std::forward<Args>(args)...));
}

void cppmain()
{
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef filter {};
	filter.FilterIdHigh = 0x1fff;
	filter.FilterIdLow = 0xffff;
	filter.FilterMaskIdHigh = 0;
	filter.FilterMaskIdLow = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterBank = 0;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_16BIT;
	filter.FilterActivation = ENABLE;
	filter.SlaveStartFilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1, &filter);

	LocalHeap::initialize();
	O1HeapInstance *o1heap = LocalHeap::getO1Heap();

	using LoopardCyphal = Cyphal<LoopardAdapter>;
	loopard_adapter.memory_allocate = LocalHeap::loopardMemoryAllocate;
	loopard_adapter.memory_free = LocalHeap::loopardMemoryDeallocate;
	LoopardCyphal loopard_cyphal(&loopard_adapter);
	loopard_cyphal.setNodeID(cyphal_node_id);

	using CanardCyphal = Cyphal<CanardAdapter>;
	canard_adapter.ins = canardInit(LocalHeap::canardMemoryAllocate, LocalHeap::canardMemoryDeallocate);
	canard_adapter.que = canardTxInit(512, CANARD_MTU_CAN_CLASSIC);
	CanardCyphal canard_cyphal(&canard_adapter);
	canard_cyphal.setNodeID(cyphal_node_id);
	canard_adapter.ins.forward_start_id = 64;
	canard_adapter.ins.forward_end_id = 127;

	struct SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, LocalHeap::serardMemoryDeallocate, LocalHeap::serardMemoryAllocate};
	using SerardCyphal = Cyphal<SerardAdapter>;
	serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
	serard_adapter.emitter = serial_send;
	SerardCyphal serard_cyphal(&serard_adapter);
	serard_cyphal.setNodeID(cyphal_node_id);
	serard_adapter.ins.forward_start_id = 0;
	serard_adapter.ins.forward_end_id = 63;

	std::tuple<Cyphal<SerardAdapter>, Cyphal<CanardAdapter>> sercan_adapters = { serard_cyphal, canard_cyphal };
	std::tuple<Cyphal<SerardAdapter>> serard_adapters = { serard_cyphal };
	std::tuple<Cyphal<CanardAdapter>> canard_adapters = { canard_cyphal };
	std::tuple<> empty_adapters = {} ;

	SerialFrame& slot = serial_buffer.begin_write();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, slot.data, SERIAL_MTU);

	RegistrationManager registration_manager;
	SubscriptionManager subscription_manager;

	static SafeAllocator<CyphalTransfer, LocalHeap> allocator;
	LoopManager loop_manager(allocator);

	HAL_Delay(5000);

//	constexpr uint8_t uuid[] = {0xc8, 0x03, 0x52, 0xa6, 0x1d, 0x94, 0x40, 0xc9, 0x9b, 0x1d, 0xea, 0xac, 0xfd, 0xdd, 0xb2, 0x85};
//	constexpr char node_name[50] = "AUXL496_CSAT";

	using TSHeart = TaskSendHeartBeat<SerardCyphal, CanardCyphal>;
	register_task_with_heap<TSHeart>(registration_manager, 2000, 100, 0, sercan_adapters);

//	using TPHeart = TaskProcessHeartBeat<SerardCyphal, CanardCyphal>;
//	register_task_with_heap<TPHeart>(registration_manager, 2000, 100, sercan_adapters);

	using TSendNodeList = TaskSendNodePortList<SerardCyphal, CanardCyphal>;
	register_task_with_heap<TSendNodeList>(registration_manager, &registration_manager, 10000, 100, 0, sercan_adapters);

//	using TSubscribeNodeList = TaskSubscribeNodePortList<SerardCyphal, CanardCyphal>;
//	register_task_with_heap<TSubscribeNodeList>(registration_manager, &subscription_manager, 10000, 100, sercan_adapters);

//	using TRespondInfo = TaskRespondGetInfo<SerardCyphal, CanardCyphal>;
//	register_task_with_heap<TRespondInfo>(registration_manager, uuid, node_name, 10000, 100, sercan_adapters);

//	using TRequestInfoCanard = TaskRequestGetInfo<SerardCyphal, CanardCyphal>;
//	register_task_with_heap<TRequestInfoCanard>(registration_manager, 10000, 100, 21, 0, sercan_adapters);
//	register_task_with_heap<TRequestInfoCanard>(registration_manager, 10000, 800, 31, 0, sercan_adapters);
//
//	using TRequestInfoSerard = TaskRequestGetInfo<SerardCyphal, CanardCyphal>;
//	register_task_with_heap<TRequestInfoSerard>(registration_manager, 10000, 800, 121, 0, sercan_adapters);

//	using TTrivialImageBuffer = TrivialImageBuffer<1024>;
//	using TSyntheticImageGenerator = TaskSyntheticImageGenerator<TTrivialImageBuffer, ContinuousTrigger, 640>;
//	TTrivialImageBuffer img_buf;
//	register_task_with_heap<TSyntheticImageGenerator>(registration_manager, img_buf, ContinuousTrigger{}, 2000, 200);
//
//	using TrivialPipeline = ImageInputStream<TTrivialImageBuffer>;
//	using TRequestWrite = TaskRequestWrite<TrivialPipeline, SerardCyphal>;
//	ImageInputStream<TTrivialImageBuffer> stream(img_buf);
//	register_task_with_heap<TRequestWrite>(registration_manager, stream, 1000, 100, 0, 121, 0, serard_adapters);

	using TBlink = TaskBlinkLED;
	register_task_with_heap<TBlink>(registration_manager, GPIOC, LED1_Pin, 1000, 100);

	using TCheckMem = TaskCheckMemory;
	register_task_with_heap<TCheckMem>(registration_manager, o1heap, 250, 100);

	subscription_manager.subscribeAll(registration_manager, sercan_adapters);
	subscription_manager.subscribe<SubscriptionManager::MessageTag>(static_cast<CyphalPortID>(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_), sercan_adapters);
//	subscription_manager.subscribe<SubscriptionManager::MessageTag>(static_cast<CyphalPortID>(uavcan_node_port_List_1_0_FIXED_PORT_ID_), sercan_adapters);
//	subscription_manager.subscribe<SubscriptionManager::MessageTag>(static_cast<CyphalPortID>(uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_), sercan_adapters);
//
//	subscription_manager.subscribe<SubscriptionManager::RequestTag>(static_cast<CyphalPortID>(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_), sercan_adapters);
	subscription_manager.subscribe<SubscriptionManager::RequestTag>(static_cast<CyphalPortID>(uavcan_file_Write_1_1_FIXED_PORT_ID_), sercan_adapters);
//	subscription_manager.subscribe<SubscriptionManager::RequestTag>(static_cast<CyphalPortID>(uavcan_file_Read_1_1_FIXED_PORT_ID_), sercan_adapters);
//
//	subscription_manager.subscribe<SubscriptionManager::ResponseTag>(static_cast<CyphalPortID>(uavcan_node_GetInfo_1_0_FIXED_PORT_ID_), sercan_adapters);
	subscription_manager.subscribe<SubscriptionManager::ResponseTag>(static_cast<CyphalPortID>(uavcan_file_Write_1_1_FIXED_PORT_ID_), sercan_adapters);
//	subscription_manager.subscribe<SubscriptionManager::ResponseTag>(static_cast<CyphalPortID>(uavcan_file_Read_1_1_FIXED_PORT_ID_), sercan_adapters);

    ServiceManager service_manager(registration_manager.getHandlers());
	service_manager.initializeServices(HAL_GetTick());

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		Error_Handler();
	}

	while(1)
	{
//		log(LOG_LEVEL_DEBUG, "while loop: %d\r\n", HAL_GetTick());
//		log(LOG_LEVEL_DEBUG, "RegistrationManager: (%d %d) (%d %d) \r\n",
//				registration_manager.getHandlers().capacity(), registration_manager.getHandlers().size(),
//				registration_manager.getSubscriptions().capacity(), registration_manager.getSubscriptions().size());
//		log(LOG_LEVEL_DEBUG, "ServiceManager: (%d %d) \r\n",
//				service_manager.getHandlers().capacity(), service_manager.getHandlers().size());
//		log(LOG_LEVEL_DEBUG, "CanProcessRxQueue: (%d %d) \r\n",
//				can_rx_buffer.capacity(), can_rx_buffer.size());
//		log(LOG_LEVEL_DEBUG, "SerialProcessRxQueue: (%d %d) \r\n",
//				serial_buffer.capacity(), serial_buffer.size());
		loop_manager.CanProcessTxQueue(&canard_adapter, &hcan1);
		loop_manager.SerialProcessRxQueue(&serard_cyphal, &service_manager, canard_adapters, serial_buffer);
		loop_manager.CanProcessRxQueue(&canard_cyphal, &service_manager, serard_adapters, can_rx_buffer);
		loop_manager.LoopProcessRxQueue(&loopard_cyphal, &service_manager, empty_adapters);
		service_manager.handleServices();
		HAL_Delay(1);

//		char buffer[1024];
//		size_t pos = 0;
//		for (auto s : canard_adapter.subscriptions)
//		{
//		    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "%d %d\r\n", s.port_id, s.extent);
//
//		    if (pos >= sizeof(buffer))
//		        break;
//		}
//		CDC_Transmit_FS((uint8_t*)buffer, pos);
	}
//	uint32_t last = 0;
//	while(1)
//	{
//		  if (HAL_GetTick() < last+1000) continue;
//		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//		  last = HAL_GetTick();
//	}
}
