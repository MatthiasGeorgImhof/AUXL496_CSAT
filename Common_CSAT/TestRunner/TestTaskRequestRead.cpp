#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include <memory>
#include <tuple>
#include <vector>
#include <array>
#include <cstring> // For std::memcpy
#include <iostream>
#include <numeric>

#include "TaskRequestRead.hpp" // Client
#include "TaskRespondRead.hpp" // Server
#include "Task.hpp"
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "RegistrationManager.hpp"
#include "InputOutputStream.hpp"
#include "FileAccess.hpp"
#include "FileSource.hpp"
#include "uavcan/file/Read_1_1.h"
#include "uavcan/file/Error_1_0.h"
// Helper Files

void *loopardMemoryAllocate(size_t amount) { return static_cast<void *>(malloc(amount)); };
void loopardMemoryFree(void *pointer) { free(pointer); };

// // Class to expose protected members for testing
// template <FileSourceConcept FileSource, OutputStreamConcept OutputStream, typename... Adapters>
// class MockTaskRequestRead : public TaskRequestRead<OutputStream, Adapters...>
// {
// public:
//     MockTaskRequestRead(FileSource &source, OutputStream &output, uint32_t interval, uint32_t tick, CyphalNodeID node_id, CyphalTransferID transfer_id, std::tuple<Adapters...> &adapters) : TaskRequestRead<FileSource, OutputStream, Adapters...>(source, interval, tick, node_id, transfer_id, adapters) {}

//     using TaskRequestRead<FileSource, OutputStream, Adapters...>::handleTaskImpl;
//     using TaskRequestRead<FileSource, OutputStream, Adapters...>::buffer_;
// };

// Class to expose protected members for testing
template <FileAccessConcept Accessor, typename... Adapters>
class RespondWithError : public TaskRespondRead<Accessor, Adapters...>
{
public:
    RespondWithError() = delete;
    RespondWithError(Accessor& accessor, uint32_t interval, uint32_t tick, std::tuple<Adapters...> &adapters)
        : TaskRespondRead<Accessor, Adapters...>(accessor, interval, tick, adapters) {}

    void handleTaskImpl()
    {
        if (TaskForServer<CyphalBuffer8, Adapters...>::buffer_.is_empty())
            return;

        std::shared_ptr<CyphalTransfer> transfer = TaskForServer<CyphalBuffer8, Adapters...>::buffer_.pop();
        if (transfer->metadata.transfer_kind != CyphalTransferKindRequest)
        {
            log(LOG_LEVEL_ERROR, "TaskRespondRead: Expected Request transfer kind\r\n");
            return;
        }

        uavcan_file_Read_Response_1_1 response_data = {}; // Initialize

        // Read from the file
        response_data.data.value.count = 0; // No data
        response_data._error.value = uavcan_file_Error_1_0_IO_ERROR;

        // Serialize and publish the response
        constexpr size_t PAYLOAD_SIZE = uavcan_file_Read_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t payload[PAYLOAD_SIZE];
        TaskForServer<CyphalBuffer8, Adapters...>::publish(PAYLOAD_SIZE, payload, &response_data,
                                            reinterpret_cast<int8_t (*)(const void *const, uint8_t *const, size_t *const)>(uavcan_file_Read_Response_1_1_serialize_),
                                            transfer->metadata.port_id, transfer->metadata.remote_node_id, transfer->metadata.transfer_id);
    }
};

class MockFileSource
{
public:
    MockFileSource(const std::string &content = "", const std::string &filepath = "test.txt")
        : content_(content), offset_(0)
    {
        std::strncpy(path_.data(), filepath.c_str(), NAME_LENGTH - 1);
        path_[NAME_LENGTH - 1] = '\0';
    }

    bool open(const std::array<char, NAME_LENGTH> & /*path*/)
    {
        offset_ = 0; // Reset the offset when opening
        return true;
    }

    size_t read(size_t offset, uint8_t *buffer, size_t size)
    {
        if (offset >= content_.size())
        {
            size = 0;
            return 0; // End of file or invalid offset
        }

        size_t bytes_to_read = std::min(size, content_.size() - offset);
        std::memcpy(buffer, content_.data() + offset, bytes_to_read);
        return bytes_to_read; // return the bytes
    }

    void setPath(const std::array<char, NAME_LENGTH> &path)
    {
        std::copy(path.begin(), path.end(), path_.begin());
    }

    std::array<char, NAME_LENGTH> getPath() const
    {
        return path_;
    }

    size_t offset() const { return offset_; }

    void setOffset(size_t offset)
    {
        offset_ = offset;
    }

    size_t chunkSize() const { return chunk_size_; }

    void setChunkSize(size_t chunk_size)
    {
        chunk_size_ = chunk_size;
    }

    void setContent(const std::string &content)
    {
        content_ = content;
    }

private:
    std::string content_;
    size_t offset_;
    std::array<char, NAME_LENGTH> path_;
    size_t chunk_size_ = 256;
};

static_assert(FileSourceConcept<MockFileSource>, "MockFileSource does not satisfy FileSourceConcept");

class MockOutputStream
{
public:
    MockOutputStream() {}
    ~MockOutputStream() = default;

    bool initialize(const std::array<char, NAME_LENGTH> &name)
    {
        path_ = name;
        return true;
    }

    bool output(uint8_t *data, size_t size)
    {
        if (data == nullptr || size == 0)
            return true; // Nothing to write

        received_data_.insert(received_data_.end(), data, data + size);
        return true;
    }

    bool finalize()
    {
        is_finalized_ = true;
        return true;
    }

    const std::vector<uint8_t> &getReceivedData() const
    {
        return received_data_;
    }

    bool isFinalized() const
    {
        return is_finalized_;
    }

private:
    std::vector<uint8_t> received_data_;
    std::array<char, NAME_LENGTH> path_;
    bool is_finalized_ = false;
};

static_assert(OutputStreamConcept<MockOutputStream>, "MockOutputStream does not satisfy OutputStreamConcept");

class MockAccessor
{
public:
    MockAccessor(size_t file_size = 1024, const std::vector<uint8_t> &pattern = {0xAA, 0x55}) : file_size_(file_size), data_(file_size), pattern_(pattern)
    {
        // Fill the data_ vector with the repeating pattern
        for (size_t i = 0; i < file_size_; ++i)
        {
            data_[i] = pattern_[i % pattern_.size()];
        }
    }

    bool read(const std::array<char, NAME_LENGTH> & /*path*/, size_t offset, uint8_t *buffer, size_t &size)
    {
        if (offset >= file_size_)
        {
            size = 0;
            return true; // EOF (or invalid offset)
        }

        size_t bytes_to_read = std::min(size, file_size_ - offset);
        std::memcpy(buffer, data_.data() + offset, bytes_to_read);
        size = bytes_to_read;
        return true;
    }

    size_t fileSize() const { return file_size_; }

private:
    size_t file_size_;
    std::vector<uint8_t> data_;
    std::vector<uint8_t> pattern_;
};

static_assert(FileAccessConcept<MockAccessor>, "MockAccessor does not satisfy FileAccessConcept");

TEST_CASE("TaskRequestRead - Handles a simple Read Request")
{
    // Setup
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    // These do nothing yet so there is little impact what they are doing
    MockFileSource file_source("hello");
    MockOutputStream output_stream;
    MockAccessor accessor;

    TaskRequestRead request(file_source, output_stream, 1000, 0, 42, 7, adapters);
    TaskRespondRead respond(accessor, 1000, 0, adapters);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 0);

    REQUIRE(loopard.buffer.size() == 1);
    auto transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 256);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 512);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 768);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 1024);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 1024);
    CHECK(output_stream.isFinalized());
}

TEST_CASE("TaskRequestRead - Handles a Read Request with Errors")
{
    // Setup
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);

    // These do nothing yet so there is little impact what they are doing
    MockFileSource file_source("hello");
    MockOutputStream output_stream;
    MockAccessor accessor;

    TaskRequestRead request(file_source, output_stream, 1000, 0, 42, 7, adapters);
    TaskRespondRead respond(accessor, 1000, 0, adapters);
    RespondWithError error(accessor, 1000, 0, adapters);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 0);

    REQUIRE(loopard.buffer.size() == 1);
    auto transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    error.handleMessage(transfer);
    error.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 0);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 256);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    error.handleMessage(transfer);
    error.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 256);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 512);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 768);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 1024);

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    error.handleMessage(transfer);
    error.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 1024);
    CHECK_FALSE(output_stream.isFinalized());

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == 1024);
    CHECK(output_stream.isFinalized());
}

TEST_CASE("TaskRequestRead - Handles a short file") {
    // Setup (same as before, but with an empty file)
    MockFileSource file_source("");
    MockOutputStream output_stream;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(11);

    constexpr size_t size = 11;
    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);
    MockAccessor accessor(size);

    TaskRequestRead request(file_source, output_stream, 1000, 0, 42, 7, adapters);
    TaskRespondRead respond(accessor, 1000, 0, adapters);

    // Act
    request.handleTaskImpl();
    REQUIRE(loopard.buffer.size() == 1);
    auto transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == size);

    // Finalize
    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();
    CHECK(output_stream.getReceivedData().size() == size);
    CHECK(output_stream.isFinalized());
}


TEST_CASE("TaskRequestRead - Handles a zero-length file") {
    // Setup (same as before, but with an empty file)
    MockFileSource file_source("");
    MockOutputStream output_stream;
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> loopard_cyphal(&loopard);
    loopard_cyphal.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(loopard_cyphal);
    MockAccessor accessor(0);

    TaskRequestRead request(file_source, output_stream, 1000, 0, 42, 7, adapters);
    TaskRespondRead respond(accessor, 1000, 0, adapters);

    // Act
    request.handleTaskImpl();
    REQUIRE(loopard.buffer.size() == 1);
    auto transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    respond.handleMessage(transfer);
    respond.handleTaskImpl();

    REQUIRE(loopard.buffer.size() == 1);
    transfer = std::make_shared<CyphalTransfer>(loopard.buffer.pop());
    request.handleMessage(transfer);
    request.handleTaskImpl();

    // Assert
    CHECK(output_stream.getReceivedData().empty()); // Should receive no data
    CHECK(output_stream.isFinalized());          // Should be finalized immediately
}

TEST_CASE("TaskRequestRead: Registers and Unregisters correctly")
{
    // Adapter
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> cy(&loopard);
    cy.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(cy);

    // File source + output stream
    MockFileSource file_source("hello");
    MockOutputStream output_stream;

    // Task parameters
    uint32_t interval = 1000;
    uint32_t tick = 0;
    CyphalNodeID node_id = 42;
    CyphalTransferID transfer_id = 7;

    RegistrationManager reg;

    auto task = std::make_shared<TaskRequestRead<MockFileSource, MockOutputStream, Cyphal<LoopardAdapter>>>(
        file_source, output_stream, interval, tick, node_id, transfer_id, adapters);

    CHECK(reg.getClients().size() == 0);

    task->registerTask(&reg, task);

    CHECK(reg.getClients().size() == 1);
    CHECK(reg.getClients().containsIf([](CyphalPortID id){
        return id == uavcan_file_Read_1_1_FIXED_PORT_ID_;
    }));

    task->unregisterTask(&reg, task);

    CHECK(reg.getClients().size() == 0);
}

TEST_CASE("TaskRespondRead: Registers and Unregisters correctly")
{
    // Adapter
    LoopardAdapter loopard;
    loopard.memory_allocate = loopardMemoryAllocate;
    loopard.memory_free = loopardMemoryFree;
    Cyphal<LoopardAdapter> cy(&loopard);
    cy.setNodeID(11);

    std::tuple<Cyphal<LoopardAdapter>> adapters(cy);

    // File accessor
    MockAccessor accessor(256);

    // Task parameters
    uint32_t interval = 1000;
    uint32_t tick = 0;

    RegistrationManager reg;

    auto task = std::make_shared<TaskRespondRead<MockAccessor, Cyphal<LoopardAdapter>>>(
        accessor, interval, tick, adapters);

    CHECK(reg.getServers().size() == 0);

    task->registerTask(&reg, task);

    CHECK(reg.getServers().size() == 1);
    CHECK(reg.getServers().containsIf([](CyphalPortID id){
        return id == uavcan_file_Read_1_1_FIXED_PORT_ID_;
    }));

    task->unregisterTask(&reg, task);

    CHECK(reg.getServers().size() == 0);
}
