#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "Task.hpp"
#include "TaskRegisterServer.hpp"
#include "BlobStore.hpp"
#include "RegistrationManager.hpp"
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "cyphal_adapter_api.hpp"
#include "nunavut_assert.h"
#include "uavcan/_register/Access_1_0.h"
#include "uavcan/_register/Name_1_0.h"
#include "uavcan/_register/Value_1_0.h"

#include <array>
#include <variant>
#include <iostream>
#include <string_view>

constexpr uint8_t UAVCAN_PRIMITIVE_UNSTRUCTURED_1_0 = 2U;

struct BlobStoreDirectory
{
    uint8_t blob1[10]; // Example blob
    uint8_t blob2[12]; // Example 
    
    enum class FieldIndex : size_t
    {
        blob1,
        blob2
    };
};

static constexpr std::array<BlobMemberInfo, 2> blob_map = {{{"blob1", offsetof(BlobStoreDirectory, blob1), sizeof(BlobStoreDirectory::blob1)},
                                                            {"blob2", offsetof(BlobStoreDirectory, blob2), sizeof(BlobStoreDirectory::blob2)}}};

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("TaskRegisterServer Tests")
{
    // Setup
    constexpr CyphalNodeID id = 11;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(id);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    BlobStoreDirectory store {{'H', 'e', 'l', 'l', 'o', 'W', 'o', 'r', 'l', 'd'}, {'1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c'}}; 
    constexpr size_t flash_size = sizeof(BlobStoreDirectory);
    uint8_t *memory = reinterpret_cast<uint8_t*>(&store);

    SPIBlobStoreAccess memory_access(flash_size, memory);
    REQUIRE(memory_access.isValid());

    NamedBlobStore<SPIBlobStoreAccess, BlobStoreDirectory, BlobMemberInfo, blob_map.size()> named_store(memory_access, blob_map);
    uint32_t interval = 100;
    uint32_t tick = 0;

    // Create TaskRegisterServer
    TaskRegisterServer<SPIBlobStoreAccess, BlobStoreDirectory, blob_map.size(), Cyphal<LoopardAdapter>>
        task_register_server(named_store, memory_access, interval, tick, adapters);

    const uint8_t test_data1[] = {'!', 'T', 'e', 's', 't', 'D', 'a', 't', 'a', '!'};
    constexpr size_t test_data_size1 = sizeof(test_data1);
    const uint8_t test_data2[] = {'A', 'a', 'B', 'b', 'C', 'c', 'D', 'd', 'E', 'e', 'F', 'f'};
    constexpr size_t test_data_size2 = sizeof(test_data2);
    static_assert(sizeof(BlobStoreDirectory::blob1) == test_data_size1);
    static_assert(sizeof(BlobStoreDirectory::blob2) == test_data_size2);

    SUBCASE("Task request blob1")
    {
        uint8_t payload[uavcan_register_Access_Request_1_0_EXTENT_BYTES_];
        size_t payload_size{sizeof(payload)};

        uavcan_register_Access_Request_1_0 access_request{
            .name = {.name = {.elements = "blob1", .count = 5}},
            .value = {.empty = {}, ._tag_ = 0U}};

        REQUIRE(loopard.buffer.size() == 0);
        std::shared_ptr<CyphalTransfer> request = std::make_shared<CyphalTransfer>(createTransfer(payload_size, payload, &access_request,
                                                                                                  reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_register_Access_Request_1_0_serialize_),
                                                                                                  uavcan_register_Access_1_0_FIXED_PORT_ID_, CyphalTransferKindRequest, static_cast<CyphalNodeID>(11)));
        task_register_server.handleMessage(request);
        task_register_server.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        CyphalTransfer response = loopard.buffer.pop();
        uavcan_register_Access_Response_1_0 access_response;
        unpackTransfer(&response, reinterpret_cast<int8_t (*)(uint8_t *, const uint8_t *, size_t *)>(uavcan_register_Access_Response_1_0_deserialize_), reinterpret_cast<uint8_t *>(&access_response));
        
        CHECK(access_response.value._tag_ == UAVCAN_PRIMITIVE_UNSTRUCTURED_1_0);
        CHECK(access_response.value.unstructured.value.count == sizeof(BlobStoreDirectory::blob1));
        CHECK_FALSE(strncmp(reinterpret_cast<const char*>(access_response.value.unstructured.value.elements), reinterpret_cast<const char*>(store.blob1), sizeof(BlobStoreDirectory::blob1)));
    }

    SUBCASE("Task change and request blob1")
    {
        uint8_t payload[uavcan_register_Access_Request_1_0_EXTENT_BYTES_];
        size_t payload_size{sizeof(payload)};

        uavcan_register_Access_Request_1_0 access_request{
            .name = {.name = {.elements = "blob1", .count = 5}},
            .value = {.unstructured = {.value = {.elements = {}, .count = 7}}, ._tag_ = UAVCAN_PRIMITIVE_UNSTRUCTURED_1_0}};
        memcpy(access_request.value.unstructured.value.elements, "01234578", sizeof("01234578"));

        REQUIRE(loopard.buffer.size() == 0);
        std::shared_ptr<CyphalTransfer> request = std::make_shared<CyphalTransfer>(createTransfer(payload_size, payload, &access_request,
                                                                                                  reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_register_Access_Request_1_0_serialize_),
                                                                                                  uavcan_register_Access_1_0_FIXED_PORT_ID_, CyphalTransferKindRequest, static_cast<CyphalNodeID>(11)));
        task_register_server.handleMessage(request);
        task_register_server.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        CyphalTransfer response = loopard.buffer.pop();
        uavcan_register_Access_Response_1_0 access_response;
        unpackTransfer(&response, reinterpret_cast<int8_t (*)(uint8_t *, const uint8_t *, size_t *)>(uavcan_register_Access_Response_1_0_deserialize_), reinterpret_cast<uint8_t *>(&access_response));
        
        CHECK(access_response.value._tag_ == UAVCAN_PRIMITIVE_UNSTRUCTURED_1_0);
        CHECK(access_response.value.unstructured.value.count == sizeof(BlobStoreDirectory::blob1));
        CHECK_FALSE(strncmp(reinterpret_cast<const char*>(access_response.value.unstructured.value.elements), reinterpret_cast<const char*>(store.blob1), sizeof(BlobStoreDirectory::blob1)));
    }

        SUBCASE("Task change and request blob2")
    {
        uint8_t payload[uavcan_register_Access_Request_1_0_EXTENT_BYTES_];
        size_t payload_size{sizeof(payload)};

        uavcan_register_Access_Request_1_0 access_request{
            .name = {.name = {.elements = "blob2", .count = 5}},
            .value = {.unstructured = {.value = {.elements = {}, .count = 7}}, ._tag_ = UAVCAN_PRIMITIVE_UNSTRUCTURED_1_0}};
        memcpy(access_request.value.unstructured.value.elements, "AASSDDFFGG", sizeof("AASSDDFFGG"));

        REQUIRE(loopard.buffer.size() == 0);
        std::shared_ptr<CyphalTransfer> request = std::make_shared<CyphalTransfer>(createTransfer(payload_size, payload, &access_request,
                                                                                                  reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_register_Access_Request_1_0_serialize_),
                                                                                                  uavcan_register_Access_1_0_FIXED_PORT_ID_, CyphalTransferKindRequest, static_cast<CyphalNodeID>(11)));
        task_register_server.handleMessage(request);
        task_register_server.handleTaskImpl();
        REQUIRE(loopard.buffer.size() == 1);

        CyphalTransfer response = loopard.buffer.pop();
        uavcan_register_Access_Response_1_0 access_response;
        unpackTransfer(&response, reinterpret_cast<int8_t (*)(uint8_t *, const uint8_t *, size_t *)>(uavcan_register_Access_Response_1_0_deserialize_), reinterpret_cast<uint8_t *>(&access_response));
        
        CHECK(access_response.value._tag_ == UAVCAN_PRIMITIVE_UNSTRUCTURED_1_0);
        CHECK(access_response.value.unstructured.value.count == sizeof(BlobStoreDirectory::blob2));
        CHECK_FALSE(strncmp(reinterpret_cast<const char*>(access_response.value.unstructured.value.elements), reinterpret_cast<const char*>(store.blob2), sizeof(BlobStoreDirectory::blob2)));
    }


}