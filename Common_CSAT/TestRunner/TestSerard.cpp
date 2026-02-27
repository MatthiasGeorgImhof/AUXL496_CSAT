#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "cyphal.hpp"
#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <algorithm>
#include <vector>
#include <sstream>

#include "serard.h"

std::string message =  "00 04 01 04 79 06 ff ff 55 1d 17 01 01 01 01 01 01 01 01 01 02 80 01 04 0e 10 17 01 01 01 01 01 05 42 20 80 3a 00";
std::string request =  "00 04 01 04 15 02 79 03 98 c1 01 01 01 01 01 01 01 01 01 01 02 80 01 03 31 59 01 01 01 01 10 0a 72 65 6d 6f 74 65 2e 74 78 74 62 9f b4 74 00 ";
std::string response = "00 04 01 04 79 02 15 03 98 81 01 01 01 01 01 01 01 01 01 01 02 80 01 03 14 66 01 01 ff 01 30 20 31 20 32 20 33 20 34 20 35 20 36 20 37 20 38 20 39 20 31 30 20 31 31 20 31 32 20 31 33 20 31 34 20 31 35 20 31 36 20 31 37 20 31 38 20 31 39 20 32 30 20 32 31 20 32 32 20 32 33 20 32 34 20 32 35 20 32 36 20 32 37 20 32 38 20 32 39 20 33 30 20 33 31 20 33 32 20 33 33 20 33 34 20 33 35 20 33 36 20 33 37 20 33 38 20 33 39 20 34 30 20 34 31 20 34 32 20 34 33 20 34 34 20 34 35 20 34 36 20 34 37 20 34 38 20 34 39 20 35 30 20 35 31 20 35 32 20 35 33 20 35 34 20 35 35 20 35 36 20 35 37 20 35 38 20 35 39 20 36 30 20 36 31 20 36 32 20 36 33 20 36 34 20 36 35 20 36 36 20 36 37 20 36 38 20 36 39 20 37 30 20 37 31 20 37 32 20 37 33 20 37 34 20 37 35 20 37 36 20 37 37 20 37 38 20 37 39 20 38 30 20 38 31 20 38 32 20 38 33 20 38 34 20 38 35 20 38 36 20 38 37 08 20 38 38 94 f5 41 d8 00 ";

std::vector<uint8_t> hexToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    std::stringstream ss(hex);
    std::string byteString;

    while (ss >> byteString) {
        bytes.push_back(static_cast<uint8_t>(std::stoul(byteString, nullptr, 16)));
    }
    return bytes;
}

void *serardMemoryAllocate(void *const /*user_reference*/, const size_t size) { return static_cast<void *>(malloc(size)); };
void serardMemoryDeallocate(void *const /*user_reference*/, const size_t /*size*/, void *const pointer) { free(pointer); };

TEST_CASE("Message")
{
    std::vector<uint8_t> binary = hexToBytes(message);
    size_t payload_size = binary.size();
    uint8_t *payload = binary.data();
    
    struct SerardRxTransfer transfer;
    struct SerardMemoryResource smr{nullptr, serardMemoryAllocate, serardMemoryDeallocate};
    struct SerardRx ins = serardInit(smr, smr);
    ins.node_id = 11;
    struct SerardReassembler reassembler = serardReassemblerInit();

    struct SerardRxSubscription msg_subscription;
    serardRxSubscribe(&ins, SerardTransferKindMessage, 7509, 1000, 0, &msg_subscription);

    struct SerardRxSubscription* subscription;
    auto result = serardRxAccept(&ins, &reassembler, 0, &payload_size, payload, 0, &transfer, &subscription);
    CHECK(result == 1);
}

TEST_CASE("Request")
{
    std::vector<uint8_t> binary = hexToBytes(request);
    size_t payload_size = binary.size();
    uint8_t *payload = binary.data();
    
    struct SerardRxTransfer transfer;
    struct SerardMemoryResource smr{nullptr, serardMemoryAllocate, serardMemoryDeallocate};
    struct SerardRx ins = serardInit(smr, smr);
    ins.node_id = 11;
    ins.forward_start_id = 0;
    ins.forward_end_id = 128;
    
    struct SerardReassembler reassembler = serardReassemblerInit();
    struct SerardRxSubscription req_subscription;
    serardRxSubscribe(&ins, SerardTransferKindRequest, 408, 1000, 0, &req_subscription);

    struct SerardRxSubscription* subscription;
    auto result = serardRxAccept(&ins, &reassembler, 0, &payload_size, payload, 0, &transfer, &subscription);
    CHECK(result == 1);
}

TEST_CASE("Response")
{
    std::vector<uint8_t> binary = hexToBytes(response);
    size_t payload_size = binary.size();
    uint8_t *payload = binary.data();
    
    struct SerardRxTransfer transfer;
    struct SerardMemoryResource smr{nullptr, serardMemoryAllocate, serardMemoryDeallocate};
    struct SerardRx ins = serardInit(smr, smr);
    ins.node_id = 11;
    ins.forward_start_id = 0;
    ins.forward_end_id = 128;
    
    struct SerardReassembler reassembler = serardReassemblerInit();
    struct SerardRxSubscription rep_subscription;
    serardRxSubscribe(&ins, SerardTransferKindResponse, 408, 1000, 0, &rep_subscription);

    struct SerardRxSubscription* subscription;
    auto result = serardRxAccept(&ins, &reassembler, 0, &payload_size, payload, 0, &transfer, &subscription);
    CHECK(result == 1);
}

