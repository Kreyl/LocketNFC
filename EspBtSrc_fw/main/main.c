#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

//#include "driver/gpio.h"
//#include "driver/uart.h"

#include "config.h"
#include "kl_uart.h"
#include "kl_bt.h"
#include "EvtQMain.h"
#include "EvtMsgIDs.h"

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

//static void MainTask(void *arg) {
//    static const char *TAG = "MAIN_TASK";
//    esp_log_level_set(TAG, ESP_LOG_INFO);
//    EvtMsg_t Msg;
//    while(1) {
//        if(xQueueReceive(MainQ, (void*)&Msg, (TickType_t)portMAX_DELAY)) {
//            ESP_LOGI(TAG, "evt: %d", Msg.ID);
//        }
//    } // while
//}


void app_main(void) {
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Create main Q
//    MainQ = xQueueCreate(MAIN_Q_LEN, sizeof(EvtMsg_t));
    UartInit();
    BTInit();

//    xTaskCreate(MainTask, "MainTask", 1024, NULL, 4, NULL); // Priority=4
//    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
}

void OnCmd() {
    ESP_LOGI("OnCmd", "Cmd: %s", CmdGetName());
    if(CmdNameIs("Ping")) CmdReplyOk();

    else if(CmdNameIs("tst")) {
        uint32_t udw;
        int32_t dw;
        char *S = NULL;
        if(CmdGetNextString(&S) == retvOk) {
            Printf("Str: %S\r", S);
            if(CmdGetNextUint32(&udw) == retvOk) {
                Printf("u: %u\r", udw);
                if(CmdGetNextInt32(&dw) == retvOk) Printf("d: %d\r", dw);
            }
        }
        else CmdReplyBadParam();
    }

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
