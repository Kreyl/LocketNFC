#include "commandexchanger.h"

#include <cstring>
#include <sys/types.h>

#ifndef htole16
#if BYTE_ORDER == BIG_ENDIAN
#define htole16(x) __bswap_16(x)
#else
#define htole16(x) (x)
#endif
#endif

#ifndef le16toh
#if BYTE_ORDER == BIG_ENDIAN
#define le16toh(x) __bswap_16(x)
#else
#define le16toh(x) (x)
#endif
#endif

#ifndef htole32
#if BYTE_ORDER == BIG_ENDIAN
#define htole32(x) __bswap_32(x)
#else
#define htole32(x) (x)
#endif
#endif

#ifndef le32toh
#if BYTE_ORDER == BIG_ENDIAN
#define le32toh(x) __bswap_32(x)
#else
#define le32toh(x) (x)
#endif
#endif

static const size_t STATUS_SIZE = 4;

CommandExchanger::CommandExchanger(UartSendByteCallback send_byte_callback,
                                   UartRecvByteCallback recv_byte_callback)
    : slip_codec_(send_byte_callback, recv_byte_callback)
{
}

bool CommandExchanger::sync(uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 36;
    static const uint16_t OUTPUT_DATA_SIZE = STATUS_SIZE;

    const struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::Sync,
            .size = htole16(INPUT_DATA_SIZE),
            .checksum = 0
        },
        .data = {
            0x07, 0x07, 0x12, 0x20,
            0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
            0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
            0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
            0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55
        }
    };

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 nullptr,
                 0,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    return true;
}

bool CommandExchanger::change_baudrate(uint32_t baudrate,
                                       uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 8;
    static const uint16_t OUTPUT_DATA_SIZE = STATUS_SIZE;

    const struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::ChangeBaudrate,
            .size = htole16(INPUT_DATA_SIZE),
            .checksum = 0
        },
        .data = {
            static_cast<uint8_t>((baudrate >> 0) & 0xff),
            static_cast<uint8_t>((baudrate >> 8) & 0xff),
            static_cast<uint8_t>((baudrate >> 16) & 0xff),
            static_cast<uint8_t>((baudrate >> 24) & 0xff)
        }
    };

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 nullptr,
                 0,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    return true;
}

bool CommandExchanger::spi_attach(uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 8;
    static const uint16_t OUTPUT_DATA_SIZE = STATUS_SIZE;

    const struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::SpiAttach,
            .size = htole16(INPUT_DATA_SIZE),
            .checksum = 0
        },
        .data = {}
    };

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 nullptr,
                 0,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    return true;
}

bool CommandExchanger::spi_set_params(size_t flash_size,
                                      uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 24;
    static const uint16_t OUTPUT_DATA_SIZE = STATUS_SIZE;

    // Hardcoded as in esptool.py
    size_t flash_id = 0;
    size_t block_size = 64 * 1024;
    size_t sector_size = 4 * 1024;
    size_t page_size = 256;
    uint32_t status_mask = 0xffff;

    const struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::SpiSetParams,
            .size = htole16(INPUT_DATA_SIZE),
            .checksum = 0
        },
        .data = {
            static_cast<uint8_t>((flash_id >> 0) & 0xff),
            static_cast<uint8_t>((flash_id >> 8) & 0xff),
            static_cast<uint8_t>((flash_id >> 16) & 0xff),
            static_cast<uint8_t>((flash_id >> 24) & 0xff),
            static_cast<uint8_t>((flash_size >> 0) & 0xff),
            static_cast<uint8_t>((flash_size >> 8) & 0xff),
            static_cast<uint8_t>((flash_size >> 16) & 0xff),
            static_cast<uint8_t>((flash_size >> 24) & 0xff),
            static_cast<uint8_t>((block_size >> 0) & 0xff),
            static_cast<uint8_t>((block_size >> 8) & 0xff),
            static_cast<uint8_t>((block_size >> 16) & 0xff),
            static_cast<uint8_t>((block_size >> 24) & 0xff),
            static_cast<uint8_t>((sector_size >> 0) & 0xff),
            static_cast<uint8_t>((sector_size >> 8) & 0xff),
            static_cast<uint8_t>((sector_size >> 16) & 0xff),
            static_cast<uint8_t>((sector_size >> 24) & 0xff),
            static_cast<uint8_t>((page_size >> 0) & 0xff),
            static_cast<uint8_t>((page_size >> 8) & 0xff),
            static_cast<uint8_t>((page_size >> 16) & 0xff),
            static_cast<uint8_t>((page_size >> 24) & 0xff),
            static_cast<uint8_t>((status_mask >> 0) & 0xff),
            static_cast<uint8_t>((status_mask >> 8) & 0xff),
            static_cast<uint8_t>((status_mask >> 16) & 0xff),
            static_cast<uint8_t>((status_mask >> 24) & 0xff)
        }
    };

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 nullptr,
                 0,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    return true;
}

bool CommandExchanger::flash_begin(size_t flash_offset,
                                   size_t image_size,
                                   uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 16;
    static const uint16_t OUTPUT_DATA_SIZE = STATUS_SIZE;

    size_t erase_size = image_size;
    size_t packet_count = (image_size + PACKET_SIZE - 1) / PACKET_SIZE;

    const struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::FlashBegin,
            .size = htole16(INPUT_DATA_SIZE),
            .checksum = 0
        },
        .data = {
            static_cast<uint8_t>((erase_size >> 0) & 0xff),
            static_cast<uint8_t>((erase_size >> 8) & 0xff),
            static_cast<uint8_t>((erase_size >> 16) & 0xff),
            static_cast<uint8_t>((erase_size >> 24) & 0xff),
            static_cast<uint8_t>((packet_count >> 0) & 0xff),
            static_cast<uint8_t>((packet_count >> 8) & 0xff),
            static_cast<uint8_t>((packet_count >> 16) & 0xff),
            static_cast<uint8_t>((packet_count >> 24) & 0xff),
            static_cast<uint8_t>((PACKET_SIZE >> 0) & 0xff),
            static_cast<uint8_t>((PACKET_SIZE >> 8) & 0xff),
            static_cast<uint8_t>((PACKET_SIZE >> 16) & 0xff),
            static_cast<uint8_t>((PACKET_SIZE >> 24) & 0xff),
            static_cast<uint8_t>((flash_offset >> 0) & 0xff),
            static_cast<uint8_t>((flash_offset >> 8) & 0xff),
            static_cast<uint8_t>((flash_offset >> 16) & 0xff),
            static_cast<uint8_t>((flash_offset >> 24) & 0xff)
        }
    };

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 nullptr,
                 0,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    return true;
}

bool CommandExchanger::flash_data(const uint8_t *packet,
                                  size_t packet_index,
                                  uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 16;
    static const uint16_t OUTPUT_DATA_SIZE = STATUS_SIZE;

    struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::FlashData,
            .size = htole16(INPUT_DATA_SIZE + PACKET_SIZE),
            .checksum = 0
        },
        .data = {
            static_cast<uint8_t>((PACKET_SIZE >> 0) & 0xff),
            static_cast<uint8_t>((PACKET_SIZE >> 8) & 0xff),
            static_cast<uint8_t>((PACKET_SIZE >> 16) & 0xff),
            static_cast<uint8_t>((PACKET_SIZE >> 24) & 0xff),
            static_cast<uint8_t>((packet_index >> 0) & 0xff),
            static_cast<uint8_t>((packet_index >> 8) & 0xff),
            static_cast<uint8_t>((packet_index >> 16) & 0xff),
            static_cast<uint8_t>((packet_index >> 24) & 0xff)
        }
    };

    uint8_t checksum = 0xef;

    for (size_t i = 0; i < PACKET_SIZE; i++) {
        checksum ^= packet[i];
    }

    request.header.checksum = htole32(checksum);

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 packet,
                 PACKET_SIZE,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    return true;
}

bool CommandExchanger::flash_end(bool reboot,
                                 uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 4;
    static const uint16_t OUTPUT_DATA_SIZE = STATUS_SIZE;

    const struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::FlashEnd,
            .size = htole16(INPUT_DATA_SIZE),
            .checksum = 0
        },
        .data = {
            static_cast<uint8_t>(reboot ? 0 : 1)
        }
    };

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 nullptr,
                 0,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    return true;
}

bool CommandExchanger::spi_flash_md5(size_t flash_offset,
                                     size_t image_size,
                                     uint8_t *md5_buffer,
                                     uint32_t timeout_ms)
{
    static const uint16_t INPUT_DATA_SIZE = 16;
    static const uint16_t OUTPUT_DATA_SIZE = MD5_SIZE + STATUS_SIZE;

    const struct __attribute__((packed))
    {
        RequestHeader header;
        uint8_t data[INPUT_DATA_SIZE];
    } request = {
        .header = {
            .direction = Direction::Request,
            .command = Command::SpiFlashMd5,
            .size = htole16(INPUT_DATA_SIZE),
            .checksum = 0
        },
        .data = {
            static_cast<uint8_t>((flash_offset >> 0) & 0xff),
            static_cast<uint8_t>((flash_offset >> 8) & 0xff),
            static_cast<uint8_t>((flash_offset >> 16) & 0xff),
            static_cast<uint8_t>((flash_offset >> 24) & 0xff),
            static_cast<uint8_t>((image_size >> 0) & 0xff),
            static_cast<uint8_t>((image_size >> 8) & 0xff),
            static_cast<uint8_t>((image_size >> 16) & 0xff),
            static_cast<uint8_t>((image_size >> 24) & 0xff)
        }
    };

    struct __attribute__((packed))
    {
        ResponseHeader header;
        uint8_t md5[MD5_SIZE];
        uint8_t status[STATUS_SIZE];
    } response = {
        .header = {
            .direction = Direction::Unknown,
            .command = Command::Unknown,
            .size = 0,
            .value = 0
        },
        .md5 = {},
        .status = {}
    };

    if (!command(&(request.header),
                 sizeof(request),
                 nullptr,
                 0,
                 &(response.header),
                 sizeof(response),
                 timeout_ms)) {
        return false;
    }

    if (response.header.size != htole16(OUTPUT_DATA_SIZE)) {
        return false;
    }

    if (response.status[0] != 0) {
        return false;
    }

    memcpy(md5_buffer, response.md5, MD5_SIZE);

    return true;
}

bool CommandExchanger::command(const RequestHeader *request_header,
                               size_t request_length,
                               const uint8_t *additional_bytes,
                               size_t additional_length,
                               ResponseHeader *response_header,
                               size_t response_length,
                               uint32_t timeout_ms)
{
    const uint8_t *request_bytes = reinterpret_cast<const uint8_t *>(request_header);

    if (!send(request_bytes,
              request_length,
              additional_bytes,
              additional_length)) {
        return false;
    }

    uint8_t *response_bytes = reinterpret_cast<uint8_t *>(response_header);

    size_t received_length = 0;

    while (true) {
        received_length = receive(response_bytes,
                                  response_length,
                                  timeout_ms);
        if (received_length < sizeof(RequestHeader))
        {
            return false;
        }

        if (response_header->direction != Direction::Response) {
            return false;
        }

        if (response_header->command == request_header->command) {
            break;
        }
    }

    if (received_length < response_length)
    {
        return false;
    }

    return true;
}

bool CommandExchanger::send(const uint8_t *request_bytes,
                            size_t request_length,
                            const uint8_t *additional_bytes,
                            size_t additional_length)
{
    if (!slip_codec_.send_end()) {
        return false;
    }

    for (size_t i = 0; i < request_length; i++) {
        if (!slip_codec_.send_byte(request_bytes[i])) {
            return false;
        }
    }

    for (size_t i = 0; i < additional_length; i++) {
        if (!slip_codec_.send_byte(additional_bytes[i])) {
            return false;
        }
    }

    if (!slip_codec_.send_end()) {
        return false;
    }

    return true;
}

size_t CommandExchanger::receive(uint8_t *response_bytes,
                                 size_t response_length,
                                 uint32_t timeout_ms)
{
    while (true) {
        uint8_t byte = 0;
        bool end = false;

        if (!slip_codec_.recv_byte_or_end(byte, end, timeout_ms)) {
            return 0;
        }

        if (end) {
            break;
        }
    }

    size_t received_length = 0;

    while (received_length <= response_length) {
        uint8_t byte = 0;
        bool end = false;

        if (!slip_codec_.recv_byte_or_end(byte, end, timeout_ms)) {
            return 0;
        }

        if (end) {
            if (received_length > 0) {
                break;
            } else {
                continue;
            }
        }

        if (received_length < response_length) {
            response_bytes[received_length++] = byte;
        } else {
            break;
        }
    }

    return received_length;
}
