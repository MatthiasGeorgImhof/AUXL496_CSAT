#ifndef __TASKREGISTER_SERVER_HPP_
#define __TASKREGISTER_SERVER_HPP_

#include <string_view>
#include <array>
#include <memory>
#include "Task.hpp"
#include "RegistrationManager.hpp"
#include "BlobStore.hpp"
#include "nunavut_assert.h"
#include "uavcan/_register/Access_1_0.h"
#include "uavcan/_register/Name_1_0.h"
#include "uavcan/_register/Value_1_0.h"

template <typename Accessor, typename Dictionary, size_t MapSize, typename... Adapters>
class TaskRegisterServer : public TaskForServer<Adapters...> {
public:
    using MapEntry = BlobMemberInfo;

    TaskRegisterServer() = delete;

    TaskRegisterServer(NamedBlobStore<Accessor, Dictionary, MapEntry, MapSize> store,
                       Accessor& accessor,
                       uint32_t interval,
                       uint32_t tick,
                       std::tuple<Adapters...>& adapters)
        : TaskForServer<Adapters...>(interval, tick, adapters),
          accessor_(accessor),
          named_store_(store) {}

    void registerTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;
    void unregisterTask(RegistrationManager* manager, std::shared_ptr<Task> task) override;
    void handleTaskImpl() override;

private:
    Accessor& accessor_;
    NamedBlobStore<Accessor, Dictionary, MapEntry, MapSize> named_store_;

    std::shared_ptr<CyphalTransfer> receiveRequest(uavcan_register_Access_Request_1_0& request_data);
    void processRequest(const uavcan_register_Access_Request_1_0& request_data,
                        uavcan_register_Access_Response_1_0& response_data);
    void answerRequest(uavcan_register_Access_Response_1_0& response_data,
                       const std::shared_ptr<CyphalTransfer>& transfer);
};

// Receiver: Deserializes the request payload
template <typename Accessor, typename Dictionary, size_t MapSize, typename... Adapters>
std::shared_ptr<CyphalTransfer>
TaskRegisterServer<Accessor, Dictionary, MapSize, Adapters...>::receiveRequest(uavcan_register_Access_Request_1_0& request_data) {
    if (TaskForServer<Adapters...>::buffer_.is_empty()) return nullptr;

    auto transfer = TaskForServer<Adapters...>::buffer_.pop();
    if (transfer->metadata.transfer_kind != CyphalTransferKindRequest) return nullptr;

    size_t payload_size = transfer->payload_size;
    if (uavcan_register_Access_Request_1_0_deserialize_(&request_data,
                                                        static_cast<const uint8_t*>(transfer->payload),
                                                        &payload_size) < 0) {
        return nullptr;
    }

    return transfer;
}

// Process: Loads or stores register content
template <typename Accessor, typename Dictionary, size_t MapSize, typename... Adapters>
void TaskRegisterServer<Accessor, Dictionary, MapSize, Adapters...>::processRequest(
    const uavcan_register_Access_Request_1_0& request_data,
    uavcan_register_Access_Response_1_0& response_data) {
    
    const std::string_view key(reinterpret_cast<const char*>(request_data.name.name.elements),
                               request_data.name.name.count);

    if (uavcan_register_Value_1_0_is_unstructured_(&request_data.value)) {
        size_t size = request_data.value.unstructured.value.count;
        named_store_.write_blob_by_name(key, request_data.value.unstructured.value.elements, size);
    }

    response_data.timestamp.microsecond = 1234567890;
    response_data._mutable = true;
    response_data.persistent = true;
    uavcan_register_Value_1_0_select_unstructured_(&response_data.value);

    auto max_size = sizeof(response_data.value.unstructured.value.elements);
    response_data.value.unstructured.value.count = max_size;

    auto span = named_store_.read_blob_by_name(key, response_data.value.unstructured.value.elements, max_size);
    response_data.value.unstructured.value.count = span.size();
}

// Answer: Serializes and publishes the response
template <typename Accessor, typename Dictionary, size_t MapSize, typename... Adapters>
void TaskRegisterServer<Accessor, Dictionary, MapSize, Adapters...>::answerRequest(
    uavcan_register_Access_Response_1_0& response_data,
    const std::shared_ptr<CyphalTransfer>& transfer) {
    
    constexpr size_t PAYLOAD_SIZE = uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t payload[PAYLOAD_SIZE];

    TaskForServer<Adapters...>::publish(PAYLOAD_SIZE,
                                        payload,
                                        &response_data,
                                        reinterpret_cast<int8_t (*)(const void*, uint8_t*, size_t*)>(
                                            uavcan_register_Access_Response_1_0_serialize_),
                                        transfer->metadata.port_id,
                                        transfer->metadata.remote_node_id,
                                        transfer->metadata.transfer_id);
}

// Dispatcher: Ties the steps together
template <typename Accessor, typename Dictionary, size_t MapSize, typename... Adapters>
void TaskRegisterServer<Accessor, Dictionary, MapSize, Adapters...>::handleTaskImpl() {
    uavcan_register_Access_Request_1_0 request_data{};
    auto transfer = receiveRequest(request_data);
    if (!transfer) return;

    uavcan_register_Access_Response_1_0 response_data{};
    processRequest(request_data, response_data);
    answerRequest(response_data, transfer);
}

template <typename Accessor, typename Dictionary, size_t MapSize, typename... Adapters>
void TaskRegisterServer<Accessor, Dictionary, MapSize, Adapters...>::registerTask(
    RegistrationManager* manager, std::shared_ptr<Task> task) {
    manager->server(uavcan_register_Access_1_0_FIXED_PORT_ID_, task);
}

template <typename Accessor, typename Dictionary, size_t MapSize, typename... Adapters>
void TaskRegisterServer<Accessor, Dictionary, MapSize, Adapters...>::unregisterTask(
    RegistrationManager* manager, std::shared_ptr<Task> task) {
    manager->unserver(uavcan_register_Access_1_0_FIXED_PORT_ID_, task);
}

#endif // __TASKREGISTER_SERVER_HPP_
