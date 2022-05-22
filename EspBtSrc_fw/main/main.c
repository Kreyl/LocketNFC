#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "config.h"
#include "kl_uart.h"
#include "kl_bt.h"

#define VERSION     1

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
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
    BTInit();
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
