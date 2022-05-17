#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
//#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "config.h"
#include "kl_uart.h"
#include "EvtQMain.h"
#include "EvtMsgIDs.h"

esp_err_t event_handler(void *ctx, system_event_t *event) {
    return ESP_OK;
}

//static void tx_task(void *arg) {
//    static const char *TX_TASK_TAG = "TX_TASK";
//    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//    while(1) {
//        uart_write_bytes(UART_NUM_1, "aga\r", 4);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }
//}

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
    // Create main Q
    MainQ = xQueueCreate(MAIN_Q_LEN, sizeof(EvtMsg_t));

    UartInit();
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
}
