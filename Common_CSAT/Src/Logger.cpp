#include "Logger.hpp"
#include <cstdarg>
#include <cstring>
#include <cstdio>

#ifdef __arm__
#include "usbd_cdc_if.h"
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "canard_adapter.hpp"
#include "serard_adapter.hpp"
#include "udpard_adapter.hpp"
#include "uavcan/diagnostic/Record_1_1.h"
#endif

#ifdef LOGGER_OUTPUT_UART
UART_HandleTypeDef* Logger::huart_ = nullptr;
#endif

#ifdef LOGGER_OUTPUT_STDERR
std::ostream* Logger::stream_ = &std::cerr;
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
void* Logger::adapter_loopard_ = nullptr;
void* Logger::adapter_canard_ = nullptr;
void* Logger::adapter_serard_ = nullptr;
void* Logger::adapter_udpard_ = nullptr;
uint8_t Logger::cyphal_transfer_id_ = 0;
#endif

constexpr size_t BUFFER_SIZE = 256;

void Logger::log(uint8_t level, const char* format, va_list args)
{
    if (level < LOG_LEVEL) return;

    char buffer[BUFFER_SIZE];
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    if (len <= 0) return;

#ifdef LOGGER_OUTPUT_UART
    uart_transmit_log_message(buffer, static_cast<uint16_t>(strlen(buffer)));
#endif

#ifdef LOGGER_OUTPUT_USB
    usb_cdc_transmit_log_message(buffer, static_cast<uint16_t>(strlen(buffer)));
#endif

#ifdef LOGGER_OUTPUT_STDERR
    stream_transmit_log_message(buffer);
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
    can_transmit_log_message(buffer, strlen(buffer), level);
#endif
}

#ifdef LOGGER_OUTPUT_STDERR
void Logger::setLogStream(std::ostream* stream) {
    stream_ = stream;
}
#endif

#ifdef LOGGER_OUTPUT_UART
void Logger::setUartHandle(UART_HandleTypeDef* huart) {
    huart_ = huart;
}
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
void Logger::setCyphalLoopardAdapter(void* adapter) {
    adapter_loopard_ = adapter;
}

void Logger::setCyphalCanardAdapter(void* adapter) {
    adapter_canard_ = adapter;
}

void Logger::setCyphalSerardAdapter(void* adapter) {
    adapter_serard_ = adapter;
}

void Logger::setCyphalUdpardAdapter(void* adapter) {
    adapter_udpard_ = adapter;
}
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
void Logger::can_transmit_log_message(const char* str, size_t size, uint8_t level)
{
    uavcan_diagnostic_Record_1_1 record{};
    record.severity.value = level;
    memcpy(record.text.elements, str, size);
    record.text.count = size;

    uint8_t payload[uavcan_diagnostic_Record_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size = sizeof(payload);
    uavcan_diagnostic_Record_1_1_serialize_(&record, payload, &payload_size);

    CyphalTransferMetadata meta{
        static_cast<CyphalPriority>(LOG_LEVEL_ALERT - level),
        CyphalTransferKindMessage,
        uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_,
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        CYPHAL_NODE_ID_UNSET,
        cyphal_transfer_id_
    };

    if (adapter_loopard_) static_cast<Cyphal<LoopardAdapter>*>(adapter_loopard_)->cyphalTxPush(0, &meta, payload_size, payload);
    if (adapter_canard_) static_cast<Cyphal<CanardAdapter>*>(adapter_canard_)->cyphalTxPush(0, &meta, payload_size, payload);
    if (adapter_serard_) static_cast<Cyphal<SerardAdapter>*>(adapter_serard_)->cyphalTxPush(0, &meta, payload_size, payload);
    if (adapter_udpard_) static_cast<Cyphal<UdpardAdapter>*>(adapter_udpard_)->cyphalTxPush(0, &meta, payload_size, payload);

    ++cyphal_transfer_id_;
}
#endif

#ifdef LOGGER_OUTPUT_UART
void Logger::uart_transmit_log_message(const char* str, uint16_t size)
{
    HAL_UART_Transmit(huart_, (uint8_t*)str, size, 1000);
}
#endif

#ifdef LOGGER_OUTPUT_USB
void Logger::usb_cdc_transmit_log_message(const char* str, uint16_t size)
{
    CDC_Transmit_FS((uint8_t*)str, size);
}
#endif

#ifdef LOGGER_OUTPUT_STDERR
void Logger::stream_transmit_log_message(const char* str)
{
    *stream_ << str << std::endl;
}
#endif

void log(uint8_t level, const char* format, ...)
{
    va_list args;
    va_start(args, format);
    Logger::log(level, format, args);
    va_end(args);
}

int uchar_buffer_to_hex(const unsigned char* src, size_t len, char* dst, size_t dst_size)
{
    if (!src || !dst || len == 0 || dst_size == 0) return -1;

    size_t required = len * 3 + 1;
    if (dst_size < required) return -1;

    for (size_t i = 0; i < len; i++) {
        snprintf(dst + i*3, 3, "%02X", src[i]);
        dst[i*3 + 2] = ' ';
    }
    dst[len*3 - 1] = '\0';
    return 0;
}
