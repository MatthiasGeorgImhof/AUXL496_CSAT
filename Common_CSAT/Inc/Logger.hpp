#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <cstdint>
#include <cstdarg>
#include <iostream>

#ifdef __arm__
#include "stm32l4xx_hal.h"
#endif

#ifdef __x86_64__
#include "mock_hal.h"
#endif

// ----------------------
// Log level definitions
// ----------------------
#define LOG_LEVEL_ALERT    (7U)
#define LOG_LEVEL_CRITICAL (6U)
#define LOG_LEVEL_ERROR    (5U)
#define LOG_LEVEL_WARNING  (4U)
#define LOG_LEVEL_NOTICE   (3U)
#define LOG_LEVEL_INFO     (2U)
#define LOG_LEVEL_DEBUG    (1U)
#define LOG_LEVEL_TRACE    (0U)

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

#ifdef LOGGER_ENABLED
class Logger
{
public:
    static void log(uint8_t level, const char* format, va_list args);

#ifdef LOGGER_OUTPUT_UART
    static void setUartHandle(UART_HandleTypeDef* huart);
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
    // opaque pointers to avoid including adapters
    static void setCyphalLoopardAdapter(void* adapter);
    static void setCyphalCanardAdapter(void* adapter);
    static void setCyphalSerardAdapter(void* adapter);
    static void setCyphalUdpardAdapter(void* adapter);
#endif

#ifdef LOGGER_OUTPUT_STDERR
    static void setLogStream(std::ostream* stream);
#endif

private:
#ifdef LOGGER_OUTPUT_UART
    static UART_HandleTypeDef* huart_;
    static void uart_transmit_log_message(const char* str, uint16_t size);
#endif

#ifdef LOGGER_OUTPUT_USB
    static void usb_cdc_transmit_log_message(const char* str, uint16_t size);
#endif

#ifdef LOGGER_OUTPUT_STDERR
    static std::ostream* stream_;
    static void stream_transmit_log_message(const char* str);
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
    static void* adapter_loopard_;
    static void* adapter_canard_;
    static void* adapter_serard_;
    static void* adapter_udpard_;
    static uint8_t cyphal_transfer_id_;

    static void can_transmit_log_message(const char* str, size_t size, uint8_t level);
#endif
};
#endif // LOGGER_ENABLED

#ifdef LOGGER_ENABLED
void log(uint8_t level, const char* format, ...);
#else
inline void log(uint8_t, const char*, ...) {}
#endif

int uchar_buffer_to_hex(const unsigned char* src, size_t len, char* dst, size_t dst_size);

#endif // LOGGER_HPP_
