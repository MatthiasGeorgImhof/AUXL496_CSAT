#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <string>
#include <sstream>
#include "mock_hal.h" // For x86 testing
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "uavcan/diagnostic/Record_1_1.h"
#include "Logger.hpp"

#include "CanTxQueueDrainer.hpp"

CanTxQueueDrainer::CanTxQueueDrainer(CanardAdapter* /*adapter*/, CAN_HandleTypeDef* /*hcan*/) {}

void CanTxQueueDrainer::drain() {}

void CanTxQueueDrainer::irq_safe_drain() {}

CanTxQueueDrainer tx_drainer{nullptr, nullptr};

#ifdef LOGGER_OUTPUT_CYPHAL
void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("Cyphal Logger Loopard")
{
  constexpr CyphalNodeID node_id = 13;
  LoopardAdapter adapter;
  adapter.memory_allocate = loopardMemoryAllocate;
  adapter.memory_free = loopardMemoryFree;
  Cyphal<LoopardAdapter> loopard_cyphal(&adapter);
  loopard_cyphal.setNodeID(node_id);
  loopard_cyphal.cyphalRxSubscribe(CyphalTransferKindMessage, uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_, uavcan_diagnostic_Record_1_1_EXTENT_BYTES_, 1000);

  Logger::setCyphalLoopardAdapter(&loopard_cyphal);

  SUBCASE("Log message at INFO level")
  {
    log(LOG_LEVEL_INFO, "This is a test info message: %d", 42);
    uint8_t frame[1024];
    CyphalTransfer out_transfer = {};
    size_t frame_size = 0;
    loopard_cyphal.cyphalRxReceive(frame, &frame_size, &out_transfer);
    CHECK(out_transfer.metadata.remote_node_id == node_id);
    CHECK(out_transfer.metadata.port_id == uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_);

    uavcan_diagnostic_Record_1_1 record;
    size_t size = out_transfer.payload_size;
    CHECK(uavcan_diagnostic_Record_1_1_deserialize_(&record, static_cast<const uint8_t *>(out_transfer.payload), &size) == NUNAVUT_SUCCESS);

    CHECK(record.severity.value == uavcan_diagnostic_Severity_1_0_INFO);
    CHECK(strncmp(reinterpret_cast<const char *>(record.text.elements), "This is a test info message: 42", record.text.count) == 0);
    std::cout.write(reinterpret_cast<const char *>(record.text.elements), static_cast<std::streamsize>(record.text.count)) << std::endl;
  }

  SUBCASE("Log message at ERROR level")
  {
    log(LOG_LEVEL_ERROR, "This is an error message: %d", 666);
    uint8_t frame[1024];
    CyphalTransfer out_transfer = {};
    size_t frame_size = 0;
    loopard_cyphal.cyphalRxReceive(frame, &frame_size, &out_transfer);
    CHECK(out_transfer.metadata.remote_node_id == node_id);
    CHECK(out_transfer.metadata.port_id == uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_);

    uavcan_diagnostic_Record_1_1 record;
    size_t size = out_transfer.payload_size;
    CHECK(uavcan_diagnostic_Record_1_1_deserialize_(&record, static_cast<const uint8_t *>(out_transfer.payload), &size) == NUNAVUT_SUCCESS);

    CHECK(record.severity.value == uavcan_diagnostic_Severity_1_0_ERROR);
    CHECK(strncmp(reinterpret_cast<const char *>(record.text.elements), "This is an error message: 666", record.text.count) == 0);
    std::cout.write(reinterpret_cast<const char *>(record.text.elements), static_cast<std::streamsize>(record.text.count)) << std::endl;
  }
}
#endif