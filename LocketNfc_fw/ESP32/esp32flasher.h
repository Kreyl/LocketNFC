#pragma once

#include <cstddef>
#include <cstdint>

#include "interface.h"
#include "commandexchanger.h"

class Esp32Flasher
{
public:
    static const size_t MD5_SIZE = CommandExchanger::MD5_SIZE;

public:
    Esp32Flasher(UartSetBaudRateCallback set_baud_rate_callback,
                 UartSendByteCallback send_byte_callback,
                 UartRecvByteCallback recv_byte_callback,
                 FileReadCallback read_callback);

    int flash(void *descriptor,
              size_t flash_offset,
              size_t image_size,
              uint8_t *md5_buffer = nullptr);

private:
    CommandExchanger exchanger_;

    UartSetBaudRateCallback set_baud_rate_callback_;

    FileReadCallback read_callback_;
};
