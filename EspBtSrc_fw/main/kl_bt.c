/*
 * bt.c
 *
 *  Created on: 15 мая 2022 г.
 *      Author: layst
 */

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_log.h"
#include "bt_app_core.h"

#include "freertos/timers.h"
#include <string.h>

#include "kl_bt.h"
#include "kl_uart.h"

#if 1 // ====================== Defines and declarations ========================
#define $EC                     ESP_ERROR_CHECK
#define BT_AV_TAG               "BT_AV"
#define BT_RC_CT_TAG            "RCCT"

// AVRCP used transaction label
#define APP_RC_CT_TL_GET_CAPS            (0)
#define APP_RC_CT_TL_RN_VOLUME_CHANGE    (1)

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

// A2DP global state
enum {
    APP_AV_STATE_UNCONNECTED, // 0
    APP_AV_STATE_DISCOVERING, // 1
    APP_AV_STATE_CONNECTING,  // 2
    APP_AV_STATE_CONNECTED,   // 3
    APP_AV_STATE_DISCONNECTING, // 4
};

// sub states of APP_AV_STATE_CONNECTED
enum {
    APP_AV_MEDIA_STATE_IDLE,
    APP_AV_MEDIA_STATE_STARTING,
    APP_AV_MEDIA_STATE_STARTED,
    APP_AV_MEDIA_STATE_STOPPING,
};

#define BT_APP_HEART_BEAT_EVT    (0xff00)

bool AutoConnectEn = false;

/// handler for bluetooth stack enabled events
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/// callback function for A2DP source
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/// callback function for A2DP source audio data stream
static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len);

/// callback function for AVRCP controller
static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);

static void a2d_app_heart_beat(void *arg);

/// A2DP application state machine
static void bt_app_av_sm_hdlr(uint16_t event, void *param);

/// avrc CT event handler
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);

/* A2DP application state machine handler for each state */
static void bt_app_av_state_unconnected(uint16_t event, void *param);
static void bt_app_av_state_connecting(uint16_t event, void *param);
static void bt_app_av_state_connected(uint16_t event, void *param);
static void bt_app_av_state_disconnecting(uint16_t event, void *param);

static void ProcessAvMedia(uint16_t event, void *param);

// Device addr & name
static esp_bd_addr_t BtDevAddr = {0};
static uint8_t DevName[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static int A2D_State = APP_AV_STATE_UNCONNECTED;
static int s_media_state = APP_AV_MEDIA_STATE_IDLE;
static int s_intv_cnt = 0;
static int ConnectRetryCnt = 0;
static uint32_t s_pkt_cnt = 0;
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
static TimerHandle_t s_tmr;
#endif

static void get_name_from_eir(uint8_t *eir, uint8_t *bdname) {
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;
    // Get name pointer
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if(!rmt_bdname) rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    // Get length and copy name out
    if(rmt_bdname) {
        if(rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        memcpy(bdname, rmt_bdname, rmt_bdname_len);
        bdname[rmt_bdname_len] = '\0';
    }
}

static void ProcessInquiryScanRslt(esp_bt_gap_cb_param_t *param) {
    uint32_t cod = 0;
    int32_t rssi = -129; // invalid value
    esp_bt_gap_dev_prop_t *p;
    *DevName = 0;
    // Get addr string
    char AddrStr[19] = {0}; // 6 * (2 + 1) + \0
    if(param->disc_res.bda) {
        uint8_t *pa = param->disc_res.bda;
        sprintf(AddrStr, "%02x:%02x:%02x:%02x:%02x:%02x", pa[0],pa[1],pa[2],pa[3],pa[4],pa[5]);
    }

    for(int i=0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch(p->type) {
            case ESP_BT_GAP_DEV_PROP_COD:  cod =  *(uint32_t*)(p->val); break;
            case ESP_BT_GAP_DEV_PROP_RSSI: rssi = *(int8_t *)(p->val); break;
            case ESP_BT_GAP_DEV_PROP_EIR:
                if(p->val) get_name_from_eir((uint8_t*)(p->val), DevName);
                break;
            case ESP_BT_GAP_DEV_PROP_BDNAME: break;
            default: break;
        } // switch
    } // for
    // Report result
    ESP_LOGI(BT_AV_TAG, "Addr: %s, Class: 0x%x, RSSI: %d, Name: \"%s\"", AddrStr, cod, rssi, DevName);
    // Is it valid addr? Search for device with MAJOR service class as "rendering" in COD
    if (esp_bt_gap_is_valid_cod(cod) && (esp_bt_gap_get_cod_srvc(cod) & ESP_BT_COD_SRVC_RENDERING)) {
        Printf("Found %s %d \"%s\"\r\n", AddrStr, rssi, DevName);
    }
}

void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch(event) {
        case ESP_BT_GAP_DISC_RES_EVT: ProcessInquiryScanRslt(param); break;

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            if(param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                A2D_State = APP_AV_STATE_DISCOVERING;
                ESP_LOGI(BT_AV_TAG, "DiscoveryStart");
                Printf("DiscoveryStart\r\n");
            }
            else { // Stopped
                A2D_State = APP_AV_STATE_UNCONNECTED;
                ESP_LOGI(BT_AV_TAG, "DiscoveryStop");
                Printf("DiscoveryStop\r\n");
            }
            break;

        case ESP_BT_GAP_RMT_SRVCS_EVT:    break;
        case ESP_BT_GAP_RMT_SRVC_REC_EVT: break;

        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if(param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
                esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            }
            else ESP_LOGE(BT_AV_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
            if(param->pin_req.min_16_digit) {
                ESP_LOGI(BT_AV_TAG, "Input pin code: 0000 0000 0000 0000");
                esp_bt_pin_code_t pin_code = { 0 };
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            }
            else {
                ESP_LOGI(BT_AV_TAG, "Input pin code: 1234");
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;

#if (CONFIG_BT_SSP_ENABLED == true)
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
            break;
#endif

        case ESP_BT_GAP_MODE_CHG_EVT: ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode); break;
        default: ESP_LOGI(BT_AV_TAG, "bt_app_gap_cb event: %d", event); break;
    }
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param) {
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    if(event == BT_APP_EVT_STACK_UP) {
        esp_bt_dev_set_device_name("LcktNFC"); /* set up device name */
        esp_bt_gap_register_callback(bt_app_gap_cb); /* register GAP callback function */

        // initialize AVRCP controller
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = { 0 };
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        // initialize A2DP source
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_source_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_source_init();

        // set discoverable and connectable mode
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE); // No need to connect or discover us

        // create and start heart beat timer
        int tmr_id = 0;
        s_tmr = xTimerCreate("connTmr", (10000 / portTICK_RATE_MS), pdTRUE, (void*) &tmr_id, a2d_app_heart_beat);
        xTimerStart(s_tmr, portMAX_DELAY);
    }
    else ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    bt_app_work_dispatch(bt_app_av_sm_hdlr, event, param, sizeof(esp_a2d_cb_param_t), NULL);
}

static void a2d_app_heart_beat(void *arg) {
    bt_app_work_dispatch(bt_app_av_sm_hdlr, BT_APP_HEART_BEAT_EVT, NULL, 0, NULL);
}

#if 1 // =========================== State machine ============================
static void bt_app_av_sm_hdlr(uint16_t event, void *param) {
    ESP_LOGI(BT_AV_TAG, "%s state %d, evt 0x%x", __func__, A2D_State, event);
    switch(A2D_State) {
        case APP_AV_STATE_DISCOVERING:   break;
        case APP_AV_STATE_UNCONNECTED:   bt_app_av_state_unconnected  (event, param); break;
        case APP_AV_STATE_CONNECTING:    bt_app_av_state_connecting   (event, param); break;
        case APP_AV_STATE_CONNECTED:     bt_app_av_state_connected    (event, param); break;
        case APP_AV_STATE_DISCONNECTING: bt_app_av_state_disconnecting(event, param); break;
    }
}

// ===== States handlers ====
static void bt_app_av_state_unconnected(uint16_t event, void *param) {
    switch(event) {
        case ESP_A2D_CONNECTION_STATE_EVT: break;
        case ESP_A2D_AUDIO_STATE_EVT:      break;
        case ESP_A2D_AUDIO_CFG_EVT:        break;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT:   break;
        case ESP_A2D_PROF_STATE_EVT: ESP_LOGE(BT_AV_TAG, "a2dp init/deinit done"); break;

        case BT_APP_HEART_BEAT_EVT:  // Connect to device if addr is valid
            if(AutoConnectEn) {
                uint8_t *p = BtDevAddr;
                ESP_LOGI(BT_AV_TAG, "Connecting to: %02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4], p[5]);
                esp_a2d_source_connect(BtDevAddr);
                A2D_State = APP_AV_STATE_CONNECTING;
                ConnectRetryCnt = 0;
            }
            break;

        default: ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event); break;
    }
}

static void bt_app_av_state_connecting(uint16_t event, void *param) {
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);
    switch(event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if(a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(BT_AV_TAG, "a2dp connected");
                A2D_State =  APP_AV_STATE_CONNECTED;
                s_media_state = APP_AV_MEDIA_STATE_IDLE;
                Printf("Connected\r\n");
            }
            else if(a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                A2D_State =  APP_AV_STATE_UNCONNECTED;
                Printf("Unconnected\r\n");
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT: break;
        case ESP_A2D_AUDIO_CFG_EVT:   break;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT: break;

        case BT_APP_HEART_BEAT_EVT:
            if(++ConnectRetryCnt >= 2) A2D_State = APP_AV_STATE_UNCONNECTED;
            break;

        default: ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event); break;
    }
}

static void bt_app_av_state_connected(uint16_t event, void *param) {
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);
    switch(event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if(a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(BT_AV_TAG, "a2dp disconnected");
                A2D_State = APP_AV_STATE_UNCONNECTED;
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) s_pkt_cnt = 0;
            break;

        case ESP_A2D_AUDIO_CFG_EVT: break; // not suppposed to occur for A2DP source

        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        case BT_APP_HEART_BEAT_EVT:
            ProcessAvMedia(event, param);
            break;

        default: ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event); break;
    }
}

static void bt_app_av_state_disconnecting(uint16_t event, void *param) {
    esp_a2d_cb_param_t *a2d = NULL;
    switch(event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            a2d = (esp_a2d_cb_param_t *)(param);
            if(a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(BT_AV_TAG, "a2dp disconnected");
                A2D_State =  APP_AV_STATE_UNCONNECTED;
            }
            break;

        case ESP_A2D_AUDIO_STATE_EVT:    break;
        case ESP_A2D_AUDIO_CFG_EVT:      break;
        case ESP_A2D_MEDIA_CTRL_ACK_EVT: break;
        case BT_APP_HEART_BEAT_EVT:      break;

        default: ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event); break;
    }
}
#endif


static void ProcessAvMedia(uint16_t event, void *param) {
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);
    switch(s_media_state) {
        case APP_AV_MEDIA_STATE_IDLE:
            if(event == BT_APP_HEART_BEAT_EVT) {
                ESP_LOGI(BT_AV_TAG, "a2dp media ready checking...");
                esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
            }
            else if(event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
                if(a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY && a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                    ESP_LOGI(BT_AV_TAG, "a2dp media ready, starting...");
                    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
                    s_media_state = APP_AV_MEDIA_STATE_STARTING;
                }
            }
            break;

        case APP_AV_MEDIA_STATE_STARTING:
            if(event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
                if(a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_START && a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                    ESP_LOGI(BT_AV_TAG, "a2dp media start ok");
                    Printf("MediaOk\r\n"); // report to host
                    s_intv_cnt = 0;
                    s_media_state = APP_AV_MEDIA_STATE_STARTED;
                }
                else { // not started succesfully, switch to idle state
                    ESP_LOGI(BT_AV_TAG, "a2dp media start failed");
                    Printf("ConnectFail\r\n");
                    s_media_state = APP_AV_MEDIA_STATE_IDLE;
                }
            }
            break;

        case APP_AV_MEDIA_STATE_STARTED:
            if(event == BT_APP_HEART_BEAT_EVT) {
                if(++s_intv_cnt >= 10) {
                    ESP_LOGI(BT_AV_TAG, "a2dp media stopping...");
                    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
                    s_media_state = APP_AV_MEDIA_STATE_STOPPING;
                    s_intv_cnt = 0;
                }
            }
            break;

        case APP_AV_MEDIA_STATE_STOPPING:
            if(event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
                if(a2d->media_ctrl_stat.cmd == ESP_A2D_MEDIA_CTRL_STOP && a2d->media_ctrl_stat.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
                    ESP_LOGI(BT_AV_TAG, "a2dp media stopped successfully, disconnecting...");
                    s_media_state = APP_AV_MEDIA_STATE_IDLE;
                    esp_a2d_source_disconnect(BtDevAddr);
                    A2D_State = APP_AV_STATE_DISCONNECTING;
                }
                else {
                    ESP_LOGI(BT_AV_TAG, "a2dp media stopping...");
                    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_STOP);
                }
            }
            break;
    } // switch
}

static void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    if(event == ESP_AVRC_CT_PLAY_STATUS_RSP_EVT) {
        ESP_LOGE(BT_RC_CT_TAG, "Invalid AVRC event: %d", event);
    }
    else bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
}

static void bt_av_volume_changed(void) {
    if(esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap, ESP_AVRC_RN_VOLUME_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_VOLUME_CHANGE, ESP_AVRC_RN_VOLUME_CHANGE, 0);
    }
}

void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter) {
    if(event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
        ESP_LOGI(BT_RC_CT_TAG, "Volume changed: %d", event_parameter->volume);
        ESP_LOGI(BT_RC_CT_TAG, "Set absolute volume: volume %d", event_parameter->volume + 5);
        esp_avrc_ct_send_set_absolute_volume_cmd(APP_RC_CT_TL_RN_VOLUME_CHANGE, event_parameter->volume + 5);
        bt_av_volume_changed();
    }
}

static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param) {
    ESP_LOGD(BT_RC_CT_TAG, "%s evt %d", __func__, event);
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_CT_TAG, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            // get remote supported event_ids of peer AVRCP Target
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        } else {
            // clear peer notification capability record
            s_avrc_peer_rn_cap.bits = 0;
        }
        break;
    }
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d", rc->psth_rsp.key_code, rc->psth_rsp.key_state);
        break;
    }
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
        free(rc->meta_rsp.attr_text);
        break;
    }
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC event notification: %d", rc->change_ntf.event_id);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    }
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC remote features %x, TG features %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
        break;
    }
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;

        bt_av_volume_changed();
        break;
    }
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "Set absolute volume rsp: volume %d", rc->set_volume_rsp.volume);
        break;
    }

    default:
        ESP_LOGE(BT_RC_CT_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}


static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len) {
    if (len < 0 || data == NULL) return 0;

    // generate random sequence
    int val = rand() % (1 << 16);
    for (int i = 0; i < (len >> 1); i++) {
        data[(i << 1)] = val & 0xff;
        data[(i << 1) + 1] = (val >> 8) & 0xff;
    }

    return len;
}
//

#if 1 // ================= High level =================
void BTInit() {
    A2D_State = APP_AV_STATE_UNCONNECTED;
    // release controller memory
    $EC(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    // BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    $EC(esp_bt_controller_init(&bt_cfg));
    $EC(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_sleep_disable();
    // bluedroid
    $EC(esp_bluedroid_init());
    $EC(esp_bluedroid_enable());
    // create application task
    bt_app_task_start_up();
    // Bluetooth device name, connection mode and profile set up
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    // Set default parameters for Legacy Pairing
    // Use variable pin, input pin code when pairing
   esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
   esp_bt_pin_code_t pin_code;
   esp_bt_gap_set_pin(pin_type, 0, pin_code);
}

uint8_t BTGetState() { return (uint8_t)A2D_State; }

void BTStartDiscovery() {
    BTDisconnect();
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0); // Discover for 12.8s
}

void BTStopDiscovery() {
    if(A2D_State == APP_AV_STATE_DISCOVERING) esp_bt_gap_cancel_discovery();
}

void BTConnect(uint8_t *pAddr) {
    BTDisconnect();
    memcpy(BtDevAddr, pAddr, ESP_BD_ADDR_LEN);
    A2D_State = APP_AV_STATE_CONNECTING;
    esp_a2d_source_connect(BtDevAddr);
}

void BTDisconnect() {
    AutoConnectEn = false;
    BTStopDiscovery();
    if(A2D_State != APP_AV_STATE_UNCONNECTED) esp_a2d_source_disconnect(BtDevAddr);
}
#endif
