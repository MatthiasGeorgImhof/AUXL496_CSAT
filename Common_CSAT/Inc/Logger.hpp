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

#ifdef LOGGER_OUTPUT_CYPHAL
#include "cyphal.hpp"
#include "loopard_adapter.hpp"
#include "canard_adapter.hpp"
#include "serard_adapter.hpp"
#include "udpard_adapter.hpp"
#endif

#define LOG_LEVEL_ALERT (7U)
#define LOG_LEVEL_CRITICAL (6U)
#define LOG_LEVEL_ERROR (5U)
#define LOG_LEVEL_WARNING (4U)
#define LOG_LEVEL_NOTICE (3U)
#define LOG_LEVEL_INFO (2U)
#define LOG_LEVEL_DEBUG (1U)
#define LOG_LEVEL_TRACE (0U)

// Configuration Options
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG // Set the current log level
#endif

#ifdef LOGGER_ENABLED
class Logger
{
private:
#ifdef LOGGER_OUTPUT_UART
    static UART_HandleTypeDef *huart_;
    static void uart_transmit_log_message(const char *str, size_t size);
#endif

#ifdef LOGGER_OUTPUT_USB
    static void usb_cdc_transmit_log_message(const char *str, size_t size);
#endif

#ifdef LOGGER_OUTPUT_CYPHAL
static Cyphal<LoopardAdapter> *adapter_loopard_;
static Cyphal<CanardAdapter> *adapter_canard_;
static Cyphal<SerardAdapter> *adapter_serard_;
static Cyphal<UdpardAdapter> *adapter_udpard_;
static CyphalTransferID cyphal_transfer_id_;

    static void can_transmit_log_message(const char *str, size_t size, unsigned int level);
#endif

#ifdef LOGGER_OUTPUT_STDERR
    static std::ostream *stream_;
    static void stream_transmit_log_message(const char *str);
#endif

public:
    static void log(unsigned int level, const char *format, va_list args);

#ifdef LOGGER_OUTPUT_UART
    static void setUartHandle(UART_HandleTypeDef *huart) { huart_ = huart; };
#endif
#ifdef LOGGER_OUTPUT_CYPHAL
static void setCyphalLoopardAdapter(Cyphal<LoopardAdapter> *adapter) { adapter_loopard_ = adapter; };
static void setCyphalCanardAdapter(Cyphal<CanardAdapter> *adapter) { adapter_canard_ = adapter; };
static void setCyphalSerardAdapter(Cyphal<SerardAdapter> *adapter) { adapter_serard_ = adapter; };
static void setCyphalUdpardAdapter(Cyphal<UdpardAdapter> *adapter) { adapter_udpard_ = adapter; };
#endif
#ifdef LOGGER_OUTPUT_STDERR
    static void setLogStream(std::ostream *stream) { stream_ = stream; };
#endif
};
#endif /* LOGGER_ENABLED */

#ifdef LOGGER_ENABLED
// inline log function for enabled case, using va_list
void log(unsigned int level, const char *format, ...);
#else
// Dummy inline log function for disabled case.
inline void log(unsigned int /*level*/, const char */*format*/, ...) {}
#endif /* LOGGER_ENABLED */

#ifdef __cplusplus
extern "C" {
#endif
extern int uchar_buffer_to_hex(const unsigned char* uchar_buffer, size_t uchar_len, char* hex_string_buffer, size_t hex_string_buffer_size);
#ifdef __cplusplus
}
#endif


#endif /* LOGGER_HPP_ */
