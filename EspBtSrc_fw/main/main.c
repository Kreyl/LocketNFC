#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/i2s.h"
#include "esp_intr_alloc.h"
#include <string.h>

#include "config.h"
#include "kl_uart.h"
#include "kl_bt.h"

#ifndef I2S_PIN_NO_CHANGE
#define I2S_PIN_NO_CHANGE (-1)
#endif

#define VERSION     1

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

TaskHandle_t TaskI2S_handle;
#define I2S_BUF_LEN     1024L
#define I2S_BUF_SZ      (I2S_BUF_LEN * sizeof(uint16_t))
RingbufHandle_t AuRingBuf;
uint16_t i2sbuf[I2S_BUF_LEN];

void TaskI2S() {
    while(true) {
//        size_t BytesRead;
//        i2s_read(I2S_NUM_0, i2sbuf, I2S_BUF_SZ, &BytesRead, portMAX_DELAY);
//        if(BytesRead > 0) xRingbufferSend(AuRingBuf, (char*)i2sbuf, BytesRead, 0);

        i2s_stop(I2S_NUM_0);
        vTaskDelay(54/portTICK_RATE_MS);
        size_t BytesToRead = xRingbufferGetCurFreeSize(AuRingBuf);
        if(BytesToRead >= I2S_BUF_SZ) {
            i2s_start(I2S_NUM_0);
            while(BytesToRead >= I2S_BUF_SZ) {
                i2s_read(I2S_NUM_0, i2sbuf, I2S_BUF_SZ, &BytesToRead, portMAX_DELAY);
                xRingbufferSend(AuRingBuf, (char*)i2sbuf, BytesToRead, 0);
                BytesToRead = xRingbufferGetCurFreeSize(AuRingBuf);
            }
        }

//        ESP_LOGI("@KL", "btr %u", BytesToRead);
//        vTaskDelay(450/portTICK_RATE_MS);
    }
}

void i2sInit() {
    // I2S config
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
//        .sample_rate = 44100,
        .sample_rate = 50000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 32,
        .dma_buf_len = I2S_BUF_LEN,
        .use_apll = false,
    };

    // Setup pins
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATAIN_PIN,
    };

    // install and start i2s driver
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config,  0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pin_config) );
//    ESP_ERROR_CHECK( i2s_set_clk(I2S_NUM_0, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO) );
//    $EC( i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0) );

    vTaskDelay(1000/portTICK_RATE_MS);  // Delay necessary for reliable start

    // I2S task
    xTaskCreate(TaskI2S, "I2S Task", 8192, NULL, 5, &TaskI2S_handle);
}

int32_t BTOnDataRequest(uint8_t *PData, int32_t ALen) {
//    ESP_LOGI("@KL", "dreq %u", ALen);
    if (ALen < 0 || PData == NULL) return 0;

    // receive data from byte buffer
    size_t BytesReallyRead = 0;
    uint8_t *ptr = (uint8_t*)xRingbufferReceiveUpTo(AuRingBuf, &BytesReallyRead, 0, ALen);

    if (BytesReallyRead == 0 || ptr == NULL) return 0;
//    ESP_LOGI("@KL", "brr %u", BytesReallyRead);
    // copy data
    memcpy(PData, ptr, BytesReallyRead);
    // free up the data retrieved; requred to free space in buffer
    vRingbufferReturnItem(AuRingBuf, (void*)ptr);

    // generate random sequence
//    int val = rand() % (1 << 16);
//    for (int i = 0; i < (ALen >> 1); i++) {
//        PData[(i << 1)] = val & 0xff;
//        PData[(i << 1) + 1] = (val >> 8) & 0xff;
//    }

    return ALen;
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    UartInit();
    // create a bype ring buffer for I2S audio
    AuRingBuf = xRingbufferCreate(8192, RINGBUF_TYPE_BYTEBUF);

    BTInit();
    i2sInit();

    Printf("Ready\r\n");
}

void OnCmd() {
    ESP_LOGI("OnCmd", "Cmd: %s", CmdGetName());
    if(CmdNameIs("Ping")) CmdReplyOk();

    else if(CmdNameIs("Version")) Printf("Version: 1\r\n");

    else if(CmdNameIs("GetState")) Printf("State: %u\r\n", BTGetState());

    else if(CmdNameIs("Discover")) BTStartDiscovery();
    else if(CmdNameIs("StopDiscover")) BTStopDiscovery();

    else if(CmdNameIs("Connect")) {
        uint8_t BTAddr[6];
        if(CmdGetArrUint8(BTAddr, 6) == retvOk) BTConnect(BTAddr);
        else CmdReplyBadParam();
    }
    else if(CmdNameIs("Disconnect")) {
        BTDisconnect();
        CmdReplyOk();
    }
}
