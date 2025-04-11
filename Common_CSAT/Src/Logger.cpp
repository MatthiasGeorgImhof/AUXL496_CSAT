#ifdef LOGGER_ENABLED
#include "Logger.hpp"
#include <cstdarg>
#include <cstring>
#include <iostream>

#ifdef __arm__
#include "usbd_cdc_if.h"
#endif
#ifdef __x86_64__
#include "mock_hal.h"
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
#include "cyphal.hpp"
#include "uavcan/diagnostic/Record_1_1.h"
#endif

#ifdef LOGGER_OUTPUT_UART
UART_HandleTypeDef *Logger::huart_ = nullptr;
#endif
#ifdef LOGGER_OUTPUT_STDERR
std::ostream *Logger::stream_ = &std::cerr;
#endif
#ifdef LOGGER_OUTPUT_CYPHAL
Cyphal<LoopardAdapter> *Logger::adapter_loopard_ = nullptr;
Cyphal<CanardAdapter> *Logger::adapter_canard_ = nullptr;
Cyphal<SerardAdapter> *Logger::adapter_serard_ = nullptr;
Cyphal<UdpardAdapter> *Logger::adapter_udpard_ = nullptr;

CyphalTransferID Logger::cyphal_transfer_id_ = 0;
#endif

constexpr size_t BUFFER_SIZE = 256;
void Logger::log(unsigned int level, const char *format, va_list args) // Added va_list arg
{
    if (level >= LOG_LEVEL)
    {
        char buffer[BUFFER_SIZE];
        int len = vsnprintf(buffer, sizeof(buffer), format, args);

        if (len > 0)
        {
#ifdef LOGGER_OUTPUT_UART
            uart_transmit_log_message(buffer, strlen(buffer));
#endif

#ifdef LOGGER_OUTPUT_USB
            usb_cdc_transmit_log_message(buffer, strlen(buffer));
#endif

#ifdef LOGGER_OUTPUT_STDERR
            stream_transmit_log_message(buffer);
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
            can_transmit_log_message(buffer, strlen(buffer), level);
#endif
        }
    }
}

#ifdef LOGGER_OUTPUT_CYPHAL
void Logger::can_transmit_log_message(const char *str, size_t size, unsigned int level)
{
    uavcan_diagnostic_Record_1_1 record{};
    record.severity.value = level;
    std::memcpy(record.text.elements, str, size);
    record.text.count = size;

    uint8_t payload[uavcan_diagnostic_Record_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t payload_size = uavcan_diagnostic_Record_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uavcan_diagnostic_Record_1_1_serialize_(&record, payload, &payload_size);

    CyphalTransferMetadata metadata =
        {
            static_cast<CyphalPriority>(LOG_LEVEL_ALERT - level),
            CyphalTransferKindMessage,
            uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_,
            CYPHAL_NODE_ID_UNSET,
            cyphal_transfer_id_,
        };

        if (adapter_loopard_ != nullptr) adapter_loopard_->cyphalTxPush(0, &metadata, payload_size, payload);
        if (adapter_canard_ != nullptr) adapter_canard_->cyphalTxPush(0, &metadata, payload_size, payload);
        if (adapter_serard_ != nullptr) adapter_serard_->cyphalTxPush(0, &metadata, payload_size, payload);
        if (adapter_udpard_ != nullptr) adapter_udpard_->cyphalTxPush(0, &metadata, payload_size, payload);
        ++cyphal_transfer_id_;
}
#endif /* LOGGER_OUTPUT_CYPHAL */

#ifdef LOGGER_OUTPUT_UART
void Logger::uart_transmit_log_message(const char *str, size_t size)
{
    HAL_UART_Transmit(huart_, (uint8_t *)str, size, 1000);
}
#endif

#ifdef LOGGER_OUTPUT_USB
void Logger::usb_cdc_transmit_log_message(const char *str, size_t size)
{
    CDC_Transmit_FS((uint8_t *)str, size);
}
#endif

#ifdef LOGGER_OUTPUT_STDERR
void Logger::stream_transmit_log_message(const char *str)
{
    *stream_ << str << std::endl;
}
#endif

void log(unsigned int level, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    Logger::log(level, format, args);
    va_end(args);
}
#endif /* LOGGER_ENABLED */
