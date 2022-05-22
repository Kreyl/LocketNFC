#pragma once

#include <cstdint>

#include "interface.h"

class SlipCodec
{
public:
    SlipCodec(UartSendByteCallback send_byte_callback,
              UartRecvByteCallback recv_byte_callback);

    bool send_byte(uint8_t byte);

    bool send_end();

    bool recv_byte_or_end(uint8_t &byte,
                          bool &end,
                          uint32_t timeout_ms = INFINITE_TIMEOUT);

private:
    UartSendByteCallback send_byte_callback_;
    UartRecvByteCallback recv_byte_callback_;
};
