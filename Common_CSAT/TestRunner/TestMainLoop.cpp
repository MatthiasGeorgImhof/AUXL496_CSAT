#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "mock_hal.h"

#include "o1heap.h"
#include "HeapAllocation.hpp"
#include "Logger.hpp"
#include "cyphal.hpp"
#include "canard_adapter.hpp"
#include "serard_adapter.hpp"
#include "loopard_adapter.hpp"
#include "RegistrationManager.hpp"
#include "ServiceManager.hpp"
#include "SubscriptionManager.hpp"
#include "ProcessRxQueue.hpp"
#include "Task.hpp"
#include "TaskCheckMemory.hpp"
#include "TaskBlinkLED.hpp"
#include "TaskSendHeartBeat.hpp"

CanardAdapter canard_adapter;
LoopardAdapter loopard_adapter;
SerardAdapter serard_adapter;

constexpr size_t O1HEAP_SIZE = 16384;
uint8_t o1heap_buffer[O1HEAP_SIZE] __attribute__((aligned(O1HEAP_ALIGNMENT)));
O1HeapInstance *o1heap;

// Add this small heap adapter for SafeAllocator
struct LocalHeap
{
    static void *heapAllocate(void * /*handle*/, const size_t amount)
    {
        return o1heapAllocate(o1heap, amount);
    }

    static void heapFree(void * /*handle*/, void *pointer)
    {
        o1heapFree(o1heap, pointer);
    }
};

template <typename T, typename... Args>
static void register_task_with_heap(RegistrationManager &rm, Args &&...args)
{
    static SafeAllocator<T, LocalHeap> alloc;
    rm.add(alloc_unique_custom<T>(alloc, std::forward<Args>(args)...));
}
using CanardCyphal = Cyphal<CanardAdapter>;
using SerardCyphal = Cyphal<SerardAdapter>;

constexpr CyphalNodeID cyphal_node_id = 11;
UART_HandleTypeDef *huart2_;
UART_HandleTypeDef *huart3_;

constexpr uint32_t SERIAL_TIMEOUT = 1000;
constexpr size_t SERIAL_BUFFER_SIZE = 4;
using SerialCircularBuffer = CircularBuffer<SerialFrame, SERIAL_BUFFER_SIZE>;
SerialCircularBuffer serial_buffer;

void *canardMemoryAllocate(CanardInstance *const /*canard*/, const size_t size)
{
    return o1heapAllocate(o1heap, size);
}

void canardMemoryDeallocate(CanardInstance *const /*canard*/, void *const pointer)
{
    o1heapFree(o1heap, pointer);
}

void *serardMemoryAllocate(void *const /*user_reference*/, const size_t size)
{
    return o1heapAllocate(o1heap, size);
}

void serardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer)
{
    o1heapFree(o1heap, pointer);
};

bool serialSendHuart2(void * /*user_reference*/, uint8_t data_size, const uint8_t *data)
{
    return (HAL_UART_Transmit(huart2_, const_cast<uint8_t *>(data), data_size, 1000) == HAL_OK);
}

bool serialSendHuart3(void * /*user_reference*/, uint8_t data_size, const uint8_t *data)
{
    return (HAL_UART_Transmit(huart3_, const_cast<uint8_t *>(data), data_size, 1000) == HAL_OK);
}

#ifdef __cplusplus
extern "C"
{
#endif
    bool serial_send(void * /*user_reference*/, uint8_t data_size, const uint8_t *data)
    {
        HAL_StatusTypeDef status = HAL_UART_Transmit(huart2_, const_cast<uint8_t *>(data), data_size, 1000);
        // 	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart2_, const_cast<uint8_t*>(data), data_size);

        return status == HAL_OK;
        ;
    }
#ifdef __cplusplus
}
#endif

GPIO_TypeDef *GPIOC = nullptr;

#define AVI_RST_Pin GPIO_PIN_0
#define AVI_RST_GPIO_Port GPIOC
#define SCI_RST_Pin GPIO_PIN_1
#define SCI_RST_GPIO_Port GPIOC
#define COMM1_RST_Pin GPIO_PIN_2
#define COMM1_RST_GPIO_Port GPIOC
#define COMM2_RST_Pin GPIO_PIN_3
#define COMM2_RST_GPIO_Port GPIOC
#define ATTENTION_Pin GPIO_PIN_4
#define ATTENTION_GPIO_Port GPIOA
#define EPS_RST_Pin GPIO_PIN_5
#define EPS_RST_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_9
#define LED4_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_8
#define LED5_GPIO_Port GPIOA

TEST_CASE("TaskMainLoop: TaskSendHeartBeat TaskBlinkLED TaskCheckMemory")
{
    o1heap = o1heapInit(o1heap_buffer, O1HEAP_SIZE);
    CHECK(o1heap != nullptr);

    LoopardAdapter loopard_adapter;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard_adapter);
    loopard_cyphal.setNodeID(cyphal_node_id);

    SerardAdapter serard_adapter;
    struct SerardMemoryResource serard_memory_resource = {&serard_adapter.ins, serardMemoryAllocate, serardMemoryDeallocate};
    serard_adapter.ins = serardInit(serard_memory_resource, serard_memory_resource);
    serard_adapter.emitter = serial_send;
    Cyphal<SerardAdapter> serard_cyphal(&serard_adapter);
    serard_cyphal.setNodeID(cyphal_node_id);

    std::tuple<Cyphal<SerardAdapter>> adapters = {serard_cyphal};

    RegistrationManager registration_manager;

    using TSHeartSerard = TaskSendHeartBeat<SerardCyphal>;
    register_task_with_heap<TSHeartSerard>(registration_manager, 1000U, 100U, static_cast<CyphalTransferID>(0), adapters);

    using TBlink = TaskBlinkLED;
    register_task_with_heap<TBlink>(registration_manager, GPIOC, LED1_Pin, 1000U, 100U);

    using TCheckMem = TaskCheckMemory;
    register_task_with_heap<TCheckMem>(registration_manager, o1heap, 2000U, 100U);

    REQUIRE(registration_manager.getHandlers().size() == 3);

    ServiceManager service_manager(registration_manager.getHandlers());
    SubscriptionManager subscription_manager;
    subscription_manager.subscribe<SubscriptionManager::MessageTag>(registration_manager.getSubscriptions(), adapters);
    subscription_manager.subscribe<SubscriptionManager::RequestTag>(registration_manager.getServers(), adapters);
    subscription_manager.subscribe<SubscriptionManager::ResponseTag>(registration_manager.getClients(), adapters);

    SafeAllocator<CyphalTransfer, LocalHeap> allocator;
    LoopManager loop_manager(allocator);

    O1HeapDiagnostics diagnostic_before = o1heapGetDiagnostics(o1heap);
    clear_uart_tx_buffer();

    REQUIRE(service_manager.getHandlers().size() == 3);
    REQUIRE(subscription_manager.getSubscriptions().size() == 0);

    for (uint32_t tick = 3000; tick <= 90000; tick += 3000)
    {
        HAL_SetTick(tick);
        log(LOG_LEVEL_TRACE, "while loop: %d\r\n", HAL_GetTick());
        loop_manager.SerialProcessRxQueue(&serard_cyphal, &service_manager, adapters, serial_buffer);
        loop_manager.LoopProcessRxQueue(&loopard_cyphal, &service_manager, adapters);
        service_manager.handleServices();

        CHECK(get_uart_tx_buffer_count() != 0);
        clear_uart_tx_buffer();
    }
    O1HeapDiagnostics diagnostic_after = o1heapGetDiagnostics(o1heap);
    CHECK(diagnostic_before.allocated == diagnostic_after.allocated);
}