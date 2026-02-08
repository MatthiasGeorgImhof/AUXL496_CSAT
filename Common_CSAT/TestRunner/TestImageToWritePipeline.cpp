#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "TrivialImageBuffer.hpp"
#include "InputOutputStream.hpp"
#include "TaskRequestWrite.hpp"
#include "RegistrationManager.hpp"
#include "Logger.hpp"

#include <vector>
#include <cstring>

using Buffer = TrivialImageBuffer<1024>;

// ------------------------------------------------------------
// Dummy Cyphal TX adapter (only needs cyphalTxPush)
// ------------------------------------------------------------
struct DummyAdapter
{
    int32_t cyphalTxPush(CyphalMicrosecond,
                         const CyphalTransferMetadata*,
                         size_t,
                         const uint8_t*)
    {
        return 1; // TX always "succeeds"
    }
};

// ------------------------------------------------------------
// Stub RegistrationManager methods for this test binary
// ------------------------------------------------------------
void RegistrationManager::subscribe(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::subscribe(const CyphalPortID) {}

void RegistrationManager::unsubscribe(const CyphalPortID, std::shared_ptr<Task>) {}

void RegistrationManager::publish(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::publish(const CyphalPortID) {}

void RegistrationManager::unpublish(const CyphalPortID, std::shared_ptr<Task>) {}

void RegistrationManager::client(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::client(const CyphalPortID) {}

void RegistrationManager::unclient(const CyphalPortID, std::shared_ptr<Task>) {}

void RegistrationManager::server(const CyphalPortID, std::shared_ptr<Task>) {}
void RegistrationManager::server(const CyphalPortID) {}

void RegistrationManager::unserver(const CyphalPortID, std::shared_ptr<Task>) {}

// ------------------------------------------------------------
// TestCyphalTransfer: owns its payload storage
// ------------------------------------------------------------
struct TestCyphalTransfer : public CyphalTransfer
{
    std::vector<uint8_t> storage;
};

// ------------------------------------------------------------
// MockTaskRequestWrite: exposes state + injectOkResponse()
// ------------------------------------------------------------
template <typename InputStream, typename... Adapters>
class MockTaskRequestWrite : public TaskRequestWrite<InputStream, Adapters...>
{
public:
    using Base  = TaskRequestWrite<InputStream, Adapters...>;
    using State = typename Base::State;

    MockTaskRequestWrite(InputStream& stream,
                         uint32_t sleep_interval,
                         uint32_t operate_interval,
                         uint32_t tick,
                         CyphalNodeID node_id,
                         CyphalTransferID transfer_id,
                         std::tuple<Adapters...>& adapters)
        : Base(stream, sleep_interval, operate_interval, tick, node_id, transfer_id, adapters)
    {}

    State getState() const { return this->state_; }

    size_t getOffset() const { return this->offset_; }
    size_t getSize()   const { return this->total_size_;   }

    // Inject a synthetic OK response into TaskForClient::buffer_
    void injectOkResponse()
    {
        auto t = std::make_shared<TestCyphalTransfer>();
        t->metadata.transfer_kind = CyphalTransferKindResponse;

        uavcan_file_Write_Response_1_1 resp{};
        resp._error.value = uavcan_file_Error_1_0_OK;

        uint8_t buf[uavcan_file_Write_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
        size_t sz = sizeof(buf);
        (void)uavcan_file_Write_Response_1_1_serialize_(&resp, buf, &sz);

        t->storage.assign(buf, buf + sz);
        t->payload      = t->storage.data();
        t->payload_size = sz;

        this->buffer_.push(t);
        (void)this->respond();
    }
};

// ------------------------------------------------------------
// Compile-time concept check
// ------------------------------------------------------------
static_assert(InputStreamConcept<ImageInputStream<Buffer>>,
              "ImageInputStream<Buffer> must satisfy InputStreamConcept");

// ------------------------------------------------------------
// Helper: create dummy metadata
// ------------------------------------------------------------
static ImageMetadata make_meta(uint32_t payload_size)
{
    ImageMetadata m{};
    m.timestamp    = 0xABCDEF00;
    m.payload_size = payload_size;
    m.latitude     = 11.11f;
    m.longitude    = 22.22f;
    m.producer     = METADATA_PRODUCER::CAMERA_1;
    m.format       = METADATA_FORMAT::UNKN;
    return m;
}

// ------------------------------------------------------------
// Test suite
// ------------------------------------------------------------
TEST_CASE("TaskRequestWrite end-to-end with Buffer")
{
    using Writer = MockTaskRequestWrite<
        ImageInputStream<Buffer>,
        DummyAdapter&
    >;

    // 1. Producer buffer
    Buffer buf;

    // 2. Stream adapter
    ImageInputStream<Buffer> stream(buf);

    // 3. Dummy Cyphal adapter
    DummyAdapter adapter;
    std::tuple<DummyAdapter&> adapters(adapter);

    // 4. Writer task
    Writer writer(stream,
                  /*sleep_interval*/ 0,
                  /*operate_interval*/ 0,
                  /*tick*/ 0,
                  /*node_id*/ 42,
                  /*transfer_id*/ 7,
                  adapters);

    // --------------------------------------------------------
    // Prepare one image in the buffer
    // --------------------------------------------------------
    const size_t payload_size = 32;
    auto meta = make_meta(payload_size);

    std::vector<uint8_t> payload(payload_size);
    for (size_t i = 0; i < payload_size; i++)
        payload[i] = static_cast<uint8_t>(i + 1);

    CHECK(buf.add_image(meta) == ImageBufferError::NO_ERROR);
    CHECK(buf.add_data_chunk(payload.data(), payload_size) == ImageBufferError::NO_ERROR);
    CHECK(buf.push_image() == ImageBufferError::NO_ERROR);

    CHECK(!stream.is_empty());

    // --------------------------------------------------------
    // Step 1: INIT
    // --------------------------------------------------------
    writer.handleTaskImpl(); // request() → SEND_INIT → WAIT_INIT
    CHECK(writer.getState() == Writer::State::WAIT_INIT);

    writer.injectOkResponse(); // WAIT_INIT → SEND_TRANSFER
    CHECK(writer.getState() == Writer::State::SEND_TRANSFER);

    // --------------------------------------------------------
    // Step 2: TRANSFER loop
    // --------------------------------------------------------
    while (writer.getState() == Writer::State::SEND_TRANSFER ||
           writer.getState() == Writer::State::WAIT_TRANSFER)
    {
        writer.handleTaskImpl(); // SEND_TRANSFER → WAIT_TRANSFER

        if (writer.getState() == Writer::State::WAIT_TRANSFER)
        {
            writer.injectOkResponse(); // WAIT_TRANSFER → SEND_TRANSFER or SEND_DONE
        }
    }

    // After loop, we should be at SEND_DONE
    CHECK(writer.getState() == Writer::State::WAIT_DONE);

    // --------------------------------------------------------
    // Step 3: DONE is fire-and-forget in current implementation
    // --------------------------------------------------------
    writer.handleTaskImpl();   // SEND_DONE → SEND_DONE (no WAIT_DONE / reset implemented)

    CHECK(writer.getState() == Writer::State::WAIT_DONE);

    // Confirm full stream logically consumed
    CHECK(writer.getOffset() == writer.getSize());
}

// ------------------------------------------------------------
// MockTaskMLX90640: produces exactly one frame via publishFrame()
// ------------------------------------------------------------
class MockTaskMLX90640 : public Task
{
public:
    MockTaskMLX90640(Buffer& buf) : Task(0,0), buf_(buf), published_(false)
    {}

    void handleTaskImpl() override
    {
        if (published_) return;

        // Build metadata
        ImageMetadata meta{};
        meta.timestamp    = 0x12345678;
        meta.payload_size = static_cast<uint32_t>(payload_.size());
        meta.latitude     = 1.0f;
        meta.longitude    = 2.0f;
        meta.producer     = METADATA_PRODUCER::CAMERA_1;
        meta.format       = METADATA_FORMAT::UNKN;

        // Publish metadata
        buf_.add_image(meta);

        // Publish payload
        buf_.add_data_chunk(payload_.data(), payload_.size());

        // Finalize
        buf_.push_image();

        published_ = true;
    }

    void setPayload(const std::vector<uint8_t>& p)
    {
        payload_ = p;
    }

    void registerTask(RegistrationManager*, std::shared_ptr<Task>) override {}
    void unregisterTask(RegistrationManager*, std::shared_ptr<Task>) override {}

private:
    Buffer& buf_;
    bool published_;
    std::vector<uint8_t> payload_;
};

// ------------------------------------------------------------
// Full pipeline test: MLX → Buffer → Stream → Writer
// ------------------------------------------------------------
TEST_CASE("Full pipeline: MockTaskMLX90640 → Buffer → ImageInputStream → TaskRequestWrite")
{
    using Writer = MockTaskRequestWrite<ImageInputStream<Buffer>, DummyAdapter&>;

    // 1. Trivial buffer
    Buffer buf;

    // 2. MLX mock task
    MockTaskMLX90640 mlx(buf);
    std::vector<uint8_t> payload(64);
    for (size_t i = 0; i < payload.size(); i++)
        payload[i] = static_cast<uint8_t>(i);
    mlx.setPayload(payload);

    // 3. Stream adapter
    ImageInputStream<Buffer> stream(buf);

    // 4. Dummy Cyphal adapter
    DummyAdapter adapter;
    std::tuple<DummyAdapter&> adapters(adapter);

    // 5. Writer task
    Writer writer(stream,
                  /*sleep_interval*/ 0,
                  /*operate_interval*/ 0,
                  /*tick*/ 0,
                  /*node_id*/ 42,
                  /*transfer_id*/ 7,
                  adapters);

    // --------------------------------------------------------
    // Step 1: MLX task publishes a frame
    // --------------------------------------------------------
    CHECK(stream.is_empty());
    mlx.handleTaskImpl();
    CHECK(!stream.is_empty());

    // --------------------------------------------------------
    // Step 2: Writer INIT
    // --------------------------------------------------------
    writer.handleTaskImpl(); // SEND_INIT → WAIT_INIT
    CHECK(writer.getState() == Writer::State::WAIT_INIT);

    writer.injectOkResponse(); // WAIT_INIT → SEND_TRANSFER
    CHECK(writer.getState() == Writer::State::SEND_TRANSFER);

    // --------------------------------------------------------
    // Step 3: TRANSFER loop
    // --------------------------------------------------------
    while (writer.getState() == Writer::State::SEND_TRANSFER ||
           writer.getState() == Writer::State::WAIT_TRANSFER)
    {
        writer.handleTaskImpl();

        if (writer.getState() == Writer::State::WAIT_TRANSFER)
        {
            writer.injectOkResponse();
        }
    }

    CHECK(writer.getState() == Writer::State::WAIT_DONE);

    // --------------------------------------------------------
    // Step 4: DONE (fire-and-forget in current MVP)
    // --------------------------------------------------------
    writer.handleTaskImpl();
    CHECK(writer.getState() == Writer::State::WAIT_DONE);

    // --------------------------------------------------------
    // Step 5: Confirm full stream consumed
    // --------------------------------------------------------
    CHECK(writer.getOffset() == writer.getSize());
}
