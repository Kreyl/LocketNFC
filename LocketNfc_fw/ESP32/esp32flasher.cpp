#include "esp32flasher.h"

#include <cstring>

static const uint32_t INITIAL_BAUD_RATE = 115200;
static const uint32_t FINAL_BAUD_RATE = 921600;

static const size_t FLASH_SIZE = 4 * 1024 * 1024;

static const uint8_t SYNC_ATTEMPTS = 32;

static const uint32_t SYNC_TIMEOUT = 999;
static const uint32_t FLASH_TIMEOUT = 30000;
static const uint32_t MD5_TIMEOUT = 60000;
static const uint32_t OTHER_TIMEOUT = 1000;

Esp32Flasher::Esp32Flasher(UartSetBaudRateCallback set_baud_rate_callback,
                           UartSendByteCallback send_byte_callback,
                           UartRecvByteCallback recv_byte_callback,
                           FileReadCallback read_callback)
    : exchanger_(send_byte_callback, recv_byte_callback),
      set_baud_rate_callback_(set_baud_rate_callback),
      read_callback_(read_callback)
{
}

int Esp32Flasher::flash(void *descriptor,
                        size_t flash_offset,
                        size_t image_size,
                        uint8_t *md5_buffer)
{
    set_baud_rate_callback_(INITIAL_BAUD_RATE);

    uint8_t attempt = 0;

    for (attempt = 0; attempt < SYNC_ATTEMPTS; attempt++) {
        if (exchanger_.sync(SYNC_TIMEOUT)) {
            break;
        }
    }

    if (attempt == SYNC_ATTEMPTS) {
        return -1;
    }

    if (!exchanger_.change_baudrate(FINAL_BAUD_RATE, OTHER_TIMEOUT)) {
        return -2;
    }

    if (!set_baud_rate_callback_(FINAL_BAUD_RATE)) {
        return -3;
    }

    if (!exchanger_.spi_attach(OTHER_TIMEOUT)) {
        return -4;
    }

    if (!exchanger_.spi_set_params(FLASH_SIZE, OTHER_TIMEOUT)) {
        return -5;
    }

    if (!exchanger_.flash_begin(flash_offset, image_size, FLASH_TIMEOUT)) {
        return -6;
    }

    uint8_t packet[CommandExchanger::PACKET_SIZE];
    size_t packet_index = 0;

    size_t remaining_size = image_size;

    while (remaining_size > 0) {
        size_t packet_size = CommandExchanger::PACKET_SIZE;
        if (packet_size > remaining_size) {
            packet_size = remaining_size;
        }

        if (read_callback_(descriptor,
                           packet,
                           packet_size) < packet_size) {
            return -7;
        }

        if (packet_size < CommandExchanger::PACKET_SIZE) {
            memset(packet + packet_size,
                   0xff,
                   CommandExchanger::PACKET_SIZE - packet_size);
        }

        if (!exchanger_.flash_data(packet,
                                   packet_index,
                                   FLASH_TIMEOUT)) {
            return -8;
        }

        remaining_size -= packet_size;
        packet_index++;
    }

    if (md5_buffer != nullptr) {
        if (!exchanger_.spi_flash_md5(flash_offset,
                                      image_size,
                                      md5_buffer,
                                      MD5_TIMEOUT)) {
            return -9;
        }
    }

    if (!exchanger_.flash_end(false, FLASH_TIMEOUT)) {
        return -10;
    }

    return 0;
}
