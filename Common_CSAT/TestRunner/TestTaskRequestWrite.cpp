#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <memory>
#include <tuple>
#include <vector>
#include <array>
#include <cstring> // For std::memcpy
#include <iostream>

#include "TaskRequestWrite.hpp" // Include the header for the class being tested
#include "Task.hpp"             // Include necessary headers
#include "cyphal.hpp"
#include "loopard_adapter.hpp" // Or your specific adapter header
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"

#include "uavcan/file/Write_1_1.h"
#include "uavcan/primitive/Unstructured_1_0.h"
#include "imagebuffer/image.hpp"

// Mock Buffer implementation
class MockBuffer
{
public:
    MockBuffer()
        : is_empty_(true),
          image_size_(0),
          data_index_(0)
    {
    }

    // ------------------------------------------------------------
    // State queries
    // ------------------------------------------------------------
    bool is_empty() const { return is_empty_; }

    size_t count() const { return is_empty_ ? 0 : 1; }

    bool has_room_for(size_t size) const
    {
        // Single-slot buffer: room only when empty
        // and the incoming payload fits
        return is_empty_ && size <= data_.capacity();
    }

    size_t size() const { return image_size_; }

    // ------------------------------------------------------------
    // Producer API
    // ------------------------------------------------------------
    ImageBufferError add_image(const ImageMetadata &meta)
    {
        if (!is_empty_)
            return ImageBufferError::FULL_BUFFER;

        metadata_ = meta;
        data_.clear();
        image_size_ = 0;
        data_index_ = 0;
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError add_data_chunk(const uint8_t *src, size_t size)
    {
        if (!is_empty_)
            return ImageBufferError::FULL_BUFFER;

        data_.insert(data_.end(), src, src + size);
        image_size_ = data_.size();
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError push_image()
    {
        if (!is_empty_)
            return ImageBufferError::FULL_BUFFER;

        is_empty_ = false;
        return ImageBufferError::NO_ERROR;
    }

    // ------------------------------------------------------------
    // Consumer API
    // ------------------------------------------------------------
    ImageBufferError get_image(ImageMetadata &out)
    {
        if (is_empty_)
            return ImageBufferError::EMPTY_BUFFER;

        out = metadata_;
        data_index_ = 0;
        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError get_data_chunk(uint8_t *dst, size_t &size)
    {
        if (is_empty_)
        {
            size = 0;
            return ImageBufferError::EMPTY_BUFFER;
        }

        size_t remaining = image_size_ - data_index_;
        size = std::min(size, remaining);

        if (size > 0)
        {
            std::memcpy(dst, data_.data() + data_index_, size);
            data_index_ += size;
        }

        // Auto-pop when fully consumed
        if (data_index_ >= image_size_)
        {
            is_empty_ = true;
            data_index_ = 0;
        }

        return ImageBufferError::NO_ERROR;
    }

    ImageBufferError pop_image()
    {
        if (is_empty_)
            return ImageBufferError::EMPTY_BUFFER;

        is_empty_ = true;
        data_index_ = 0;
        return ImageBufferError::NO_ERROR;
    }

private:
    bool is_empty_;
    size_t image_size_;
    size_t data_index_;
    std::vector<uint8_t> data_;
    ImageMetadata metadata_;
};

static_assert(ImageBufferConcept<MockBuffer>,
              "MockBuffer does not satisfy ImageBufferConcept");

// Mock ImageInputStream with chunksize
template <typename Buffer>
class MockImageInputStream : public ImageInputStream<Buffer>
{
public:
    MockImageInputStream(Buffer &buffer, size_t chunk_size) : ImageInputStream<Buffer>(buffer), chunk_size_(chunk_size) {}

    size_t chunkSize(size_t max_chunk_size) const
    {
        return std::min(chunk_size_, max_chunk_size);
    }

    void nextChunk(size_t max_chunk_size)
    {
        (void)max_chunk_size;
        // do nothing, mock only
    }

private:
    size_t chunk_size_;
};

// Helper function to create a Write Response
std::shared_ptr<CyphalTransfer> createWriteResponse(uint16_t error_code)
{
    uavcan_file_Write_Response_1_1 response;
    response._error.value = error_code;

    constexpr size_t PAYLOAD_SIZE = uavcan_file_Write_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];
    size_t payload_size = PAYLOAD_SIZE;

    int8_t serialization_result = uavcan_file_Write_Response_1_1_serialize_(&response, payload, &payload_size);
    REQUIRE(serialization_result >= 0);

    auto transfer = std::make_shared<CyphalTransfer>();
    transfer->metadata.transfer_kind = CyphalTransferKindResponse;
    transfer->metadata.port_id = uavcan_file_Write_1_1_FIXED_PORT_ID_;
    transfer->metadata.remote_node_id = 123; // Some dummy node ID
    transfer->metadata.transfer_id = 0;      // Dummy Transfer ID
    transfer->payload_size = payload_size;
    transfer->payload = malloc(payload_size); // Use malloc directly for simplicity
    std::memcpy(transfer->payload, payload, payload_size);

    return transfer;
}

// Class to expose protected members for testing
template <typename ImageInputStream, typename... Adapters>
class MockTaskRequestWrite : public TaskRequestWrite<ImageInputStream, Adapters...>
{
public:
    MockTaskRequestWrite(ImageInputStream &metadata_producer, uint32_t sleep_interval, uint32_t operate_interval, uint32_t tick, CyphalNodeID node_id, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters)
        : TaskRequestWrite<ImageInputStream, Adapters...>(metadata_producer, sleep_interval, operate_interval, tick, node_id, transfer_id, adapters)
    {
    }

    using TaskRequestWrite<ImageInputStream, Adapters...>::handleTaskImpl;
    using TaskRequestWrite<ImageInputStream, Adapters...>::buffer_;
};

uavcan_file_Write_Response_1_1 unpackResponse(std::shared_ptr<CyphalTransfer> transfer)
{
    uavcan_file_Write_Response_1_1 data{};
    size_t payload_size = transfer->payload_size;

    int retval = uavcan_file_Write_Response_1_1_deserialize_(&data, static_cast<const uint8_t *>(transfer->payload), &payload_size);
    (void)retval;
    return data;
}

uavcan_file_Write_Request_1_1 unpackRequest(CyphalTransfer transfer)
{
    uavcan_file_Write_Request_1_1 data{};
    size_t payload_size = transfer.payload_size;

    int retval = uavcan_file_Write_Request_1_1_deserialize_(&data, static_cast<const uint8_t *>(transfer.payload), &payload_size);
    (void)retval;
    return data;
}

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

TEST_CASE("TaskRequestWrite: Handles Write Request Lifecycle")
{
    LocalHeap::initialize();

    uavcan_file_Write_Response_1_1 response{};
    uavcan_file_Write_Request_1_1 request{};

    // Create adapter
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal); // Single adapter

    // Mock Buffer and Stream
    MockBuffer mock_buffer;
    MockImageInputStream<MockBuffer> mock_stream(mock_buffer, 16);
    constexpr const char filename[] = "0000000012345678_03";

    // Task parameters
    CyphalNodeID node_id = 42;
    CyphalTransferID transfer_id = 7;
    uint32_t tick = 0;
    uint32_t interval = 1000;

    // Instantiate the TaskRequestWrite
    MockTaskRequestWrite task(mock_stream, interval, interval, tick, node_id, transfer_id, adapters);

    // Prepare test data and metadata
    std::vector<uint8_t> test_data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    ImageMetadata metadata;
    metadata.producer = METADATA_PRODUCER::THERMAL;
    metadata.timestamp = 0x12345678;
    metadata.payload_size = static_cast<uint>(test_data.size());
    metadata.meta_crc = 0xff0000ff;
    CHECK(mock_buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    CHECK(mock_buffer.add_data_chunk(test_data.data(), test_data.size()) == ImageBufferError::NO_ERROR);
    CHECK(mock_buffer.push_image() == ImageBufferError::NO_ERROR);

    // Initial state
    CHECK(loopard.buffer.size() == 0);
    CHECK(task.buffer_.size() == 0);

    // First Task Execution: Sends the initial transfer
    task.handleTaskImpl(); // Calls Request() and Respond()
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    CyphalTransfer transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+0); // Request with defined transfer_id
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == sizeof(ImageMetadata));
    CHECK(strncmp(reinterpret_cast<char *>(request.data.value.elements), reinterpret_cast<char *>(&metadata), sizeof(ImageMetadata)) == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    // Second Task Execution: Sends the data

    auto ret_transfer = createWriteResponse(uavcan_file_Error_1_0_OK);
    response = unpackResponse(ret_transfer);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+1); // Request with defined transfer_id
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == test_data.size());
    CHECK(strncmp(reinterpret_cast<char *>(request.data.value.elements), reinterpret_cast<char *>(test_data.data()), test_data.size()) == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    // Third Task Execution: Sends Null

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_OK);
    response = unpackResponse(ret_transfer);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+2); // Request with defined transfer_id
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    // Fourth Task Execution: Receive OK

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_OK);
    response = unpackResponse(ret_transfer);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 0);
    CHECK(task.buffer_.size() == 0);
}

TEST_CASE("TaskRequestWrite: Handles Write Request Lifecycle with Errors")
{
    LocalHeap::initialize();
    
    uavcan_file_Write_Response_1_1 response{};
    uavcan_file_Write_Request_1_1 request{};

    // Create adapter
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal); // Single adapter

    // Mock Buffer and Stream
    MockBuffer mock_buffer;
    MockImageInputStream<MockBuffer> mock_stream(mock_buffer, 16);
    constexpr const char filename[] = "0000000012345678_03";

    // Task parameters
    CyphalNodeID node_id = 42;
    CyphalTransferID transfer_id = 7;
    uint32_t tick = 0;
    uint32_t interval = 1000;

    // Instantiate the TaskRequestWrite
    MockTaskRequestWrite task(mock_stream, interval, interval, tick, node_id, transfer_id, adapters);

    // Prepare test data and metadata
    std::vector<uint8_t> test_data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    ImageMetadata metadata;
    metadata.producer = METADATA_PRODUCER::THERMAL;
    metadata.timestamp = 0x12345678;
    metadata.payload_size = static_cast<uint32_t>(test_data.size());
    metadata.meta_crc = 0xff0000ff;
    CHECK(mock_buffer.add_image(metadata) == ImageBufferError::NO_ERROR);
    CHECK(mock_buffer.add_data_chunk(test_data.data(), test_data.size()) == ImageBufferError::NO_ERROR);
    CHECK(mock_buffer.push_image() == ImageBufferError::NO_ERROR);

    // Initial state
    CHECK(loopard.buffer.size() == 0);
    CHECK(task.buffer_.size() == 0);

    // First Task Execution: Sends the initial transfer
    task.handleTaskImpl(); // Calls Request() and Respond()
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    CyphalTransfer transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+0); // Request with defined transfer_id
    request = unpackRequest(transfer);
    free(transfer.payload);
    loopard.buffer.clear();

    auto ret_transfer = createWriteResponse(uavcan_file_Error_1_0_IO_ERROR);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+1); // Request with defined transfer_id
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == sizeof(ImageMetadata));
    CHECK(strncmp(reinterpret_cast<char *>(request.data.value.elements), reinterpret_cast<char *>(&metadata), sizeof(ImageMetadata)) == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    // Second Task Execution: Sends the data

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_OK);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+2); // Request with defined transfer_id
    CHECK(strncmp(&static_cast<char *>(transfer.payload)[27], reinterpret_cast<char *>(test_data.data()), 24) == 0);
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == test_data.size());
    CHECK(strncmp(reinterpret_cast<char *>(request.data.value.elements), reinterpret_cast<char *>(test_data.data()), test_data.size()) == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_IO_ERROR);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+3); // Request with defined transfer_id
    CHECK(strncmp(&static_cast<char *>(transfer.payload)[27], reinterpret_cast<char *>(test_data.data()), 24) == 0);
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == test_data.size());
    CHECK(strncmp(reinterpret_cast<char *>(request.data.value.elements), reinterpret_cast<char *>(test_data.data()), test_data.size()) == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    // Third Task Execution: Sends Null

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_OK);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+4); // Request with defined transfer_id
    CHECK(transfer.payload_size == 27);
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_IO_ERROR);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);

    transfer = loopard.buffer.pop();
    CHECK(transfer.metadata.port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_);
    CHECK(transfer.metadata.transfer_kind == CyphalTransferKindRequest);
    CHECK(transfer.metadata.remote_node_id == 11);       // Request with client node_id
    CHECK(transfer.metadata.transfer_id == transfer_id+5); // Request with defined transfer_id
    CHECK(transfer.payload_size == 27);
    request = unpackRequest(transfer);
    CHECK(request.path.path.count == NAME_LENGTH);
    CHECK(strncmp(reinterpret_cast<char *>(request.path.path.elements), filename, sizeof(filename)) == 0);
    CHECK(request.data.value.count == 0);
    free(transfer.payload);
    loopard.buffer.clear();

    // Third Task Execution: Receive OK

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_IO_ERROR);
    response = unpackResponse(ret_transfer);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 1);
    CHECK(task.buffer_.size() == 0);
    loopard.buffer.clear();

    ret_transfer = createWriteResponse(uavcan_file_Error_1_0_OK);
    response = unpackResponse(ret_transfer);
    task.handleMessage(ret_transfer);
    task.handleTaskImpl();
    CHECK(loopard.buffer.size() == 0);
    CHECK(task.buffer_.size() == 0);
}

TEST_CASE("TaskRequestWrite: Registers and Unregisters correctly")
{
    LocalHeap::initialize();
    
    // Create adapter
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal); // Single adapter

    // Mock Buffer and Stream
    MockBuffer mock_buffer;
    MockImageInputStream<MockBuffer> mock_stream(mock_buffer, 16);

    // Task parameters
    CyphalNodeID node_id = 42;
    CyphalTransferID transfer_id = 7;
    uint32_t tick = 0;
    uint32_t interval = 1000;

    // Instantiate Registration Manager
    RegistrationManager registration_manager;

    // Instantiate the TaskRequestWrite
    auto task = std::make_shared<MockTaskRequestWrite<MockImageInputStream<MockBuffer>, Cyphal<LoopardAdapter>>>(mock_stream, interval, interval, tick, node_id, transfer_id, adapters);

    // Initial state
    CHECK(registration_manager.getClients().size() == 0);

    // Register the task
    task->registerTask(&registration_manager, task);
    CHECK(registration_manager.getClients().size() == 1);
    CHECK(registration_manager.getClients().containsIf([&](const CyphalPortID &port_id)
                                                       { return port_id == uavcan_file_Write_1_1_FIXED_PORT_ID_; }));

    // Unregister the task
    task->unregisterTask(&registration_manager, task);
    CHECK(registration_manager.getClients().size() == 0);
}