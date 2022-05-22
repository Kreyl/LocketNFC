#pragma once

#include <cstddef>
#include <cstdint>

#include "slipcodec.h"

class CommandExchanger
{
public:
    static const size_t PACKET_SIZE = 512;
    static const size_t MD5_SIZE = 32;

    enum class Direction : uint8_t
    {
        Unknown = 0x00,
        Request = 0x00,
        Response = 0x01
    };

    enum class Command : uint8_t
    {
        Unknown = 0x00,
        FlashBegin = 0x02,
        FlashData = 0x03,
        FlashEnd = 0x04,
        Sync = 0x08,
        SpiSetParams = 0x0b,
        SpiAttach = 0x0d,
        ChangeBaudrate = 0x0f,
        SpiFlashMd5 = 0x13
    };

    struct __attribute__((packed)) RequestHeader
    {
        Direction direction;
        Command command;
        uint16_t size;
        uint32_t checksum;
    };

    struct __attribute__((packed)) ResponseHeader
    {
        Direction direction;
        Command command;
        uint16_t size;
        uint32_t value;
    };

public:
    CommandExchanger(UartSendByteCallback send_byte_callback,
                     UartRecvByteCallback recv_byte_callback);

    bool sync(uint32_t timeout_ms);

    bool change_baudrate(uint32_t baudrate,
                         uint32_t timeout_ms);

    bool spi_attach(uint32_t timeout_ms);

    bool spi_set_params(size_t flash_size,
                        uint32_t timeout_ms);

    bool flash_begin(size_t flash_offset,
                     size_t image_size,
                     uint32_t timeout_ms);

    bool flash_data(const uint8_t *packet,
                    size_t packet_index,
                    uint32_t timeout_ms);

    bool flash_end(bool reboot,
                   uint32_t timeout_ms);

    bool spi_flash_md5(size_t flash_offset,
                       size_t image_size,
                       uint8_t *md5_buffer,
                       uint32_t timeout_ms);

private:
    bool command(const RequestHeader *request_header,
                 size_t request_length,
                 const uint8_t *additional_bytes,
                 size_t additional_length,
                 ResponseHeader *response_header,
                 size_t response_length,
                 uint32_t timeout_ms);

    bool send(const uint8_t *request_bytes,
              size_t request_length,
              const uint8_t *additional_bytes,
              size_t additional_length);

    size_t receive(uint8_t *response_bytes,
                   size_t response_length,
                   uint32_t timeout_ms);

    SlipCodec slip_codec_;
};
