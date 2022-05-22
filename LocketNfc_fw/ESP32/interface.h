#pragma once

#include <cstddef>
#include <cstdint>

static const uint32_t INFINITE_TIMEOUT = UINT32_MAX;

typedef bool (*UartSetBaudRateCallback)(uint32_t baud_rate);
typedef bool (*UartSendByteCallback)(uint8_t byte);
typedef bool (*UartRecvByteCallback)(uint8_t &byte, uint32_t timeout_ms);

typedef size_t (*FileReadCallback)(void *descriptor,
                                   uint8_t *data,
                                   size_t length);
