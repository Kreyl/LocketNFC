/*
 * Esp32.cpp
 *
 *  Created on: 6 ���. 2018 �.
 *      Author: Kreyl
 */

#include <cstring>
#include "Esp32.h"
#include "board.h"
#include "kl_lib.h"
#include "kl_buf.h"
#include "kl_fs_utils.h"
//#include <stdarg.h>
#include "MsgQ.h"
#include "uart.h"

//#include "esp32flasher.h"

//#define DBG_PINS
//#define ESP_VERBOSE     TRUE

#ifdef DBG_PINS
#define DBG_GPIO    GPIOD
#define DBG_PIN1    13
#define DBG1_SET()  PinSetHi(DBG_GPIO, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO, DBG_PIN1)
#define DBG_PIN2    15
#define DBG2_SET()  PinSetHi(DBG_GPIO, DBG_PIN2)
#define DBG2_CLR()  PinSetLo(DBG_GPIO, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#define DBG2_SET()
#define DBG2_CLR()
#endif

#if 1 // ==== Defines ====
#define ESP_BAUDRATE0       115200
//#define ESP_BAUDRATE2       5000000UL
#define ESP_BUF_SZ          256

// ==== Timings ====
#define SIMPLE_CMD_TIMEOUT_ms   27000
#endif

#if 1 // ==== Pins & buffers ====
static PinOutput_t EspEn(ESP_EN, omPushPull);
static PinOutput_t EspPwrEn(ESP_PWR_EN, omPushPull);
static PinOutput_t EspGpio0(ESP_GPIO0, omPushPull);

extern HostUart_t EspUart;
#endif

#if 1 // ==== Q ====
enum EspMsgId_t {espDiscover, espStopDiscover};

union EspMsg_t {
    uint32_t DWord32;
    EspMsgId_t ID;
};

EvtMsgQ_t<EspMsg_t, MAIN_EVT_Q_LEN> EvtQEsp;
#endif

#if 1 // ================================ Task =================================
//enum EspState_t {estaOff, estaReady, estaDiscovering};
//static EspState_t State = estaOff;

static THD_WORKING_AREA(waEspThread, 256);
__noreturn
static void EspThread(void *arg) {
    chRegSetThreadName("Esp");
    while(true) {
        if(EspUart.WaitReply() != retvOk) continue;
        Cmd_t &Reply = EspUart.Reply;
        if(Reply.NameIs("Ready")) {
            Printf("Esp Ready\r");
//                State = estaReady;
        }
        else if(Reply.NameIs("DiscoveryStart")) {
            Printf("DiscoveryStart\r");
        }
        else if(Reply.NameIs("DiscoveryStop")) {
            Printf("DiscoveryStop\r");
        }

        else if(Reply.NameIs("Found")) {
            int Rssi = -180;
            char FAddr[BT_ADDR_STRLEN];
            char* S;
            if((S = Reply.GetNextString()) != nullptr) {
                Printf("%S ", S);
                strncpy(FAddr, S, BT_ADDR_STRLEN);
            }
            if((S = Reply.GetNextString()) != nullptr) {
                Printf("%S ", S);
                char *p;
                int tmp = strtol(S, &p, 0);
                Rssi = (*p == '\0')? tmp : -180;
            }
            if((S = Reply.GetNextString()) != nullptr) Printf("%S ", S);
            PrintfEOL();
            // Is it ok?
            if(Rssi > -99) {
                strcpy(Esp::BtAddr, FAddr);
                Esp::StopDiscover();
                EvtQMain.SendNowOrExit(EvtMsg_t(evtIdBtDevFound));
            }
        }

        /*
        EspMsg_t Msg = EvtQEsp.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case espDiscover:
                EspUart.Print("Discover\n");
                if(EspUart.WaitReply(999) == retvOk and EspUart.Reply.NameIs("DiscoveryStart")) {
                    while(EspUart.WaitReply(18000) == retvOk) {
                        Cmd_t &Reply = EspUart.Reply;
                        if(Reply.NameIs("DiscoveryStop")) break;
                    }
                }
                break;

            case espStopDiscover:
                EspUart.Print("StopDiscover\n");
                break;
        } //switch
        */
    } // while
}
#endif

namespace Esp { // ==== Functions ====

char BtAddr[BT_ADDR_STRLEN] = {0};

void Init() {
    EspEn.Init();
    EspEn.SetLo(); // Disable ESP
    EspPwrEn.Init();
    EspPwrEn.SetLo(); // Disable ESP
    EspGpio0.Init();
    // Q
    EvtQEsp.Init();
    // Thread
    chThdCreateStatic(waEspThread, sizeof(waEspThread), NORMALPRIO, (tfunc_t)EspThread, NULL);
}

uint8_t Start() {
    EspEn.SetLo(); // Disable ESP
    EspPwrEn.SetHi();
    chThdSleepMilliseconds(99);
    // Enter working mode
    EspGpio0.SetHi();   // Working mode
    chThdSleepMilliseconds(180);
    EspEn.SetHi();
    return retvOk;
}

void PwrOff() {
    EspEn.SetLo();
}

void Enable()     { EspEn.SetHi(); }
void Disable()    { EspEn.SetLo(); }
void SetGpio0Hi() { EspGpio0.SetHi(); }
void SetGpio0Lo() { EspGpio0.SetLo(); }

void Discover()     { EspUart.Print("Discover\n"); }
void StopDiscover() { EspUart.Print("StopDiscover\n"); }


} // namespace


//void EspDmaCmdTxIrq(void *p, uint32_t flags) {
//    chSysLockFromISR();
//    dmaStreamDisable(UART3_DMA_TX);
//    chThdResumeI(&ThdRef, MSG_OK);
//    chSysUnlockFromISR();
//}

#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO, DBG_PIN1, omPushPull);
    PinSetupOut(DBG_GPIO, DBG_PIN2, omPushPull);
#endif

#if 0 // ==== Uart0 ====
    PinSetupAlterFunc(ESP_TX0, omPushPull, pudNone, AF7);
    PinSetupAlterFunc(ESP_RX0, omPushPull, pudNone, AF7);
    rccEnableUSART2(FALSE);
    uint32_t tmp = RCC->CCIPR;
    tmp &= ~RCC_CCIPR_USART2SEL_Msk;
    tmp |= RCC_CCIPR_USART2SEL_0; // SYSCLK as USART2 clk
    EspSetBaudrate(ESP_BAUDRATE0);
    ESP_BOOT_UART->CR2 = 0;  // Nothing that interesting there
    ESP_BOOT_UART->CR1 = USART_CR1_TE | USART_CR1_RE;        // TX & RX enable
    // Enable USART
    ESP_BOOT_UART->CR1 |= USART_CR1_UE;
#endif

#if 0 // ==== Uart2 ====
    PinSetupAlterFunc(ESP_TX2, omPushPull, pudNone, AF7);
    PinSetupAlterFunc(ESP_RX2, omPushPull, pudNone, AF7);
    rccEnableUSART3(FALSE);
    tmp = RCC->CCIPR;
    tmp &= ~RCC_CCIPR_USART2SEL_Msk;
    tmp |= RCC_CCIPR_USART3SEL_0; // SYSCLK as USART3 clk
    ESP_CMD_UART->CR2 = 0;  // Nothing that interesting there
    ESP_CMD_UART->CR1 = USART_CR1_TE | USART_CR1_RE; // TX & RX enable
    // Baudrate 5MBit. Will work @ 80MHz only.
    ESP_CMD_UART->BRR = Clk.AHBFreqHz / ESP_BAUDRATE2;

    // DMA
    ESP_CMD_UART->CR3 |= USART_CR3_DMAR;
    dmaStreamAllocate     (UART3_DMA_RX, IRQ_PRIO_LOW, nullptr, nullptr);
    dmaStreamSetPeripheral(UART3_DMA_RX, &ESP_CMD_UART->RDR);
    dmaStreamSetMemory0   (UART3_DMA_RX, IRxBuf);
    dmaStreamSetTransactionSize(UART3_DMA_RX, ESP_RX_BUF_SZ);
    dmaStreamSetMode      (UART3_DMA_RX, UART_DMA_RX_MODE(UART3_DMA_CHNL));
    dmaStreamEnable       (UART3_DMA_RX);

    ESP_CMD_UART->CR3 |= USART_CR3_DMAT;
    dmaStreamAllocate     (UART3_DMA_TX, IRQ_PRIO_MEDIUM, EspDmaCmdTxIrq, nullptr);
    dmaStreamSetPeripheral(UART3_DMA_TX, &ESP_CMD_UART->TDR);
    dmaStreamSetMode      (UART3_DMA_TX, UART_DMA_TX_MODE(UART3_DMA_CHNL));

    // Enable USART
    ESP_CMD_UART->CR1 |= USART_CR1_UE;
#endif

#if 0 // ==== SPI ====
    AlterFunc_t Spi_AF;
    if(ISpi.PSpi == SPI1 or ISpi.PSpi == SPI2) Spi_AF = AF5;
    else Spi_AF = AF6;
    PinSetupAlterFunc(ESP_SCK,  omPushPull, pudNone, Spi_AF);
    PinSetupAlterFunc(ESP_MISO, omPushPull, pudNone, Spi_AF);
    PinSetupAlterFunc(ESP_MOSI, omPushPull, pudNone, Spi_AF);
    EspWrite.Init();
    EspWrite.SetHi();
    // Spi
    ISpi.Setup(boMSB, cpolIdleLow, cphaFirstEdge, 10000000);
//    ISpi.SetupRxIrqCallback(SpiRxIrqHandler);
//    ISpi.EnableRxIrq();
//    ISpi.EnableIrq(IRQ_PRIO_LOW);
    ISpi.Enable();
#endif
//    WMsgQ.Init();
//    chThdCreateStatic(waWiFiThread, sizeof(waWiFiThread), NORMALPRIO, (tfunc_t)WiFiThread, NULL);
//    Printf("WiFi init done\r");
//}

#if 0 // ============================== WiFi ===================================
uint8_t Esp32_t::IPwrOnIfNotYet() {
    if(State == wfstOff) {
        LogIt("LOGS\nWiFi PoweringOn\n\n");
        IStartOrRestartTmrOff();
        // Enter working mode
        EspEn.SetLo();      // Disable ESP
        EspGpio0.SetHi();   // Working mode
        chThdSleepMilliseconds(180);
        EspEn.SetHi();
        // Wait "ready"
        while(true) {
            if(GetReply(SIMPLE_CMD_TIMEOUT_ms) != retvOk) {
                Printf("ESP timeout\r");
                return retvTimeout;
            }
            if(strcasecmp(IStr, "ready") == 0) break;
            chThdSleepMilliseconds(7);
        }

    }
    State = wfstDisconnected;
    LogIt("LOGS\nWiFi PoweredOn\n\n");
    return retvOk;
}

#endif

#if 0 // ============================= App level ===============================
void Esp32_t::ExchangeDataWithServer() {
    // Authorise if not yet
    if(!System.Auth.IsValid()) {
        LogIt("LOGS\nNot Authorised, requesting tokens\n\n");
        if(IAuthorise() != retvOk) {
            LogIt("LOGS\nAuth failed\n\n");
            return;
        }
        LogIt("LOGS\nAuth done\n\n");
    }
    // Authorised
    uint32_t RespLen = 0, RetryCnt = 7;
    while(RetryCnt) { // Several retries
        LogIt("LOGS\nTry %u\n\n", RetryCnt);
        uint8_t Rslt = IGetContentInfo(&RespLen);
        if(Rslt == retvNotAuthorised) {
            LogIt("LOGS\nRefresh required\n\n");
            IRefreshAuthTokens();
            RetryCnt--;
            continue;
        }
        else if(Rslt == retvOk) { // Some content data is received
            LogIt("LOGS\n%u bytes received\n", RespLen);
            // Reconnect if server closes connection
            if(HttpRespHdr.ConnectionOpt == HttpRespHdr_t::connClose) {
                LogIt("Conn closed, reconnecting\n");
                if(ReconnectToServer() != retvOk) continue;
            }
            LogIt("Processing reply\n");
            Rslt = ContentMaster.ProcessContentInfoFromSrv(IStr, RespLen, ESP_STR_SZ);
            ContentMaster.AnalyzeStorage();
            if(Rslt == retvOk) LogIt("Processed\n");
            else if(Rslt == retvEmpty) {
                Printf("Empty list, download completed\r");
                LogIt("Empty list received\nNothing to do, switching WiFi off\n\n");
                ISwitchOff();
                return;
            }
            else {
                LogIt("Processing reply failed\n\n");
                continue;
                RetryCnt--;
            }
            if(ContentMaster.FwUpdateMcuExists) {
                LogIt("FW update received\nSwitching off\n\n");
                ISwitchOff();
                return;
            }
        }
        else RetryCnt--;
    } // while
}

void Esp32_t::AppendStrWithDevInfo() {
    IPrc--; // Proceed printing starting from last \0
    char* SContent = IPrc; // Where content starts
    // Add device info
    IProceedPrintingToIStr(
            "{"
            "\"sn\":\"%S\","
            "\"hardware\":\"1.1\","
            "\"firmware\":\"%S\""
            "}"
            , Keys.SerialID, FwVersion);
    IPutChar(0); // Finish string
    // Put content-length
    uint32_t ContLen = strlen(SContent);
    IPrc = SContent - 12; // Rewind to Content-Length position
    IProceedPrintingToIStr("%u", ContLen);
}

uint8_t Esp32_t::IAuthorise() {
    Printf("Authorise\r");
    IStartOrRestartTmrOff();
    if(IConnectToWifi() != retvOk) return retvFail;
    if(ConnectToServer() != retvOk) return retvFail;
    if(EnterTransparentTransmissionMode() != retvOk) return retvFail;

    // Send POST request
    IPrintfToIStr(
            "POST /api/v1/auth HTTP/1.1\r\n"
            "Host: my.mishka.cloud\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length:         \r\n" // 8 spaces added to contain length
            "\r\n");
    AppendStrWithDevInfo();
//    Printf("http > Len=%u\r%S\r", strlen(IStr), IStr);

    SendIStr();

    // Receive Reply header
    if(HttpRespHdr.Receive() != retvOk) {
        LogIt("LOGS\nNoAnswer\n\n");
        return retvNoAnswer;
    }

    if(HttpRespHdr.StatusCode != 200) return retvFail;
    if(HttpRespHdr.ContentLength == 0 or HttpRespHdr.ContentLength > (ESP_STR_SZ-1)) return retvFail;
    if(HttpGetResponseData(HttpRespHdr.ContentLength) != retvOk) return retvFail;
    // JSON with access and refresh tokens is received
//    Printf("\rRcvd:\r%S\r", IStr);
    uint8_t rslt = System.ProcessAuthReply(IStr, ESP_STR_SZ, ESP_STR_SZ);
    if(rslt == retvOk) LogIt("LOGS\nNew Tokens saved\n\n");
    else if(rslt == retvWriteError) LogIt("LOGS\nNew Tokens saving failed\n\n");
    else LogIt("LOGS\nParsing failed\n\n");
    return rslt;
}

void Esp32_t::IRefreshAuthTokens() {
    Printf("RefreshTokens\r");
    IStartOrRestartTmrOff();
    if(IConnectToWifi() != retvOk) return;
    if(ConnectToServer() != retvOk) return;
    if(EnterTransparentTransmissionMode() != retvOk) return;
    // Send POST request
    IPrintfToIStr(
            "POST /api/v1/auth HTTP/1.1\r\n"
            "Host: my.mishka.cloud\r\n"
            "Content-Type: application/json\r\n"
            "Authorization: Bearer %S\r\n"
            "Content-Length:         \r\n" // 8 spaces added to contain length
            "\r\n"
            , System.Auth.RefreshToken);
    AppendStrWithDevInfo();
//    Printf("http > Len=%u\r%S\r", strlen(IStr), IStr);
    SendIStr();
    // Receive Reply header
    if(HttpRespHdr.Receive() != retvOk) {
        LogIt("LOGS\nNoAnswer\n\n");
        return;
    }

    if(HttpRespHdr.StatusCode != 200) return;
    if(HttpRespHdr.ContentLength == 0 or HttpRespHdr.ContentLength > (ESP_STR_SZ-1)) return;
    if(HttpGetResponseData(HttpRespHdr.ContentLength) != retvOk) return;
    // JSON with access and refresh tokens is received
//    Printf("\rRcvd:\r%S\r", IStr);
    uint8_t rslt = System.ProcessAuthReply(IStr, ESP_STR_SZ, ESP_STR_SZ);
    if(rslt == retvOk) LogIt("LOGS\nNew Tokens saved\n\n");
    else if(rslt == retvWriteError) LogIt("LOGS\nNew Tokens saving failed\n\n");
    else LogIt("LOGS\nParsing failed\n\n");
}

uint8_t Esp32_t::IGetContentInfo(uint32_t *PRespLen) {
    LogIt("LOGS\nGetContentInfo\n\n");
    IStartOrRestartTmrOff();
    if(IConnectToWifi() != retvOk) return retvFail;
    if(ConnectToServer() != retvOk) return retvFail;
    if(EnterTransparentTransmissionMode() != retvOk) return retvFail;
#if 1 // ==== TX ====
    // Construct first part of request. We do not know Content Length for now.
    IPrintfToIStr(
            "POST /api/v1/exch HTTP/1.1\r\n"
            "Host: my.mishka.cloud\r\n"
            "Content-Type: application/json\r\n"
            "Authorization: Bearer %S\r\n"
            "Content-Length:         \r\n" // 8 spaces added to contain length
            "\r\n"
            , System.Auth.AccessToken);
    IPrc--; // Proceed printing starting from last \0
    char* SContent = IPrc; // Where content starts
    // === Add device info and beginning of content data ===
    IProceedPrintingToIStr(
            "{"
            "\"firmware\":\"%S\","
            "\"content\":["
            , FwVersion);
    // === Add content data ===
    ContentMaster.StartIterate();
    char UniqID[45];
    while(ContentMaster.GetNextUniqIDIfReady(UniqID, 45) == retvOk) {
        IProceedPrintingToIStr("\"%S\",", UniqID);
    }
    ContentMaster.EndIterate();
    if(*(IPrc-1) == ',') IPrc--; // Remove last comma
    IProceedPrintingToIStr("],");
    // === Put favorites ===
    IProceedPrintingToIStr("\"favorites\":[");
    ContentMaster.StartIterate();
    while(ContentMaster.GetNextFavoriteUniqIDIfReady(UniqID, 45) == retvOk) {
        IProceedPrintingToIStr("\"%S\",", UniqID);
    }
    ContentMaster.EndIterate();
    if(*(IPrc-1) == ',') IPrc--; // Remove last comma
    IProceedPrintingToIStr("],");
    // ==== Put UserIDs ====
    IProceedPrintingToIStr("\"users\":[");
    for(uint32_t i=0; i<UserIdTokenStorage.Cnt; i++) {
        IProceedPrintingToIStr("\"%S\",", UserIdTokenStorage[i].UserID);
    }
    if(*(IPrc-1) == ',') IPrc--; // Remove last comma
    IProceedPrintingToIStr("]}");
    IPutChar(0); // Finish string
    // === Put content-length ===
    uint32_t ContLen = strlen(SContent);
    IPrc = SContent - 12; // Rewind to Content-Length position
    IProceedPrintingToIStr("%u", ContLen);

//    Printf("http > Len=%u\r%S\r", strlen(IStr), IStr);
    SendIStr();
#endif

#if 1 // ==== RX ====
    // Receive Reply header
    if(HttpRespHdr.Receive() != retvOk) {
        LogIt("LOGS\nNoAnswer\n\n");
        return retvNoAnswer;
    }
    Printf("\rStatus: %u\r", HttpRespHdr.StatusCode);
    if(HttpRespHdr.StatusCode == 401) { // Unauthorised, token refresh required
        Printf("RefreshToken required\r");
        FlushRxBuf();
        return retvNotAuthorised;
    }
    else if(HttpRespHdr.StatusCode == 200) {
        Printf("ContLen: %u\r", HttpRespHdr.ContentLength);
        if(HttpRespHdr.ContentLength == 0) return retvFail;
        uint32_t LenToRead = (HttpRespHdr.ContentLength > (ESP_STR_SZ-1)) ? (ESP_STR_SZ-1) : HttpRespHdr.ContentLength;
        if(HttpGetResponseData(LenToRead) != retvOk) return retvFail;
        // Something is received
//        Printf("Rcvd:\r%S\r", IStr);
        *PRespLen = LenToRead;
        return retvOk;
    }

    LogIt("LOGS\nBad Reply: %u\n\n", HttpRespHdr.StatusCode);
    return retvCmdUnknown;
#endif
}

// Returns: retvDisconnected, retvTimeout, retvNotFound, retvWriteError, retvOk
uint8_t Esp32_t::DownloadFile(FIL *PFile, char* Aurl, int32_t StartingFrom) {
    IStartOrRestartTmrOff();
    if(IConnectToWifi() != retvOk) return retvDisconnected;
    if(ConnectToServer() != retvOk) return retvDisconnected;
    if(EnterTransparentTransmissionMode() != retvOk) return retvDisconnected;

    systime_t Start = chVTGetSystemTimeX();
    uint32_t Len = StartingFrom;
    while(true) {
        IStartOrRestartTmrOff();
        // Prepare GET request
        IPrintfToIStr(
                "GET %S HTTP/1.1\r\n"
                "Host: my.mishka.cloud\r\n"
                "Range: bytes=%u-%u\r\n"
                "\r\n"
                , Aurl
                , Len, (Len + ESP_STR_SZ - 1)
                );
//        Printf("http > Len=%u\r%S\r", strlen(IStr), IStr);
        SendIStr();

        if(HttpRespHdr.Receive() != retvOk) {
            Printf("No reply\r");
            return retvTimeout;
        }
//        Printf("\rStatus: %u\r", HttpRespHdr.StatusCode);
        if(! (HttpRespHdr.StatusCode == 200 or HttpRespHdr.StatusCode == 206)) return retvNotFound;

        uint32_t BytesToRcv = MIN(ESP_STR_SZ, HttpRespHdr.ContentLength);
        char* S = IStr;
        for(uint32_t i=0; i<BytesToRcv; i++) {
            if(GetCmdByte(S++, 45000) != retvOk) {
                Printf("Get fail\r");
                return retvTimeout;
            }
        }
        Len += BytesToRcv;
        Printf(".");

        // Write to file
        if(f_write(PFile, IStr, BytesToRcv, &BytesToRcv) != FR_OK) {
            Printf("write fail\r");
            return retvWriteError;
        }

        // Reconnect if server closes connection
        if(HttpRespHdr.ConnectionOpt == HttpRespHdr_t::connClose) {
            uint8_t Rslt = ReconnectToServer();
            if(Rslt != retvOk) return Rslt;
        }

        // Check if completed
        if(Len >= HttpRespHdr.FullSz) {
            Printf("Done %u in %u s\r", Len, ST2S(chVTTimeElapsedSinceX(Start)));
            return retvOk;
        }
    } // while true
}
#endif

#if 0 // ============================== Commands ===============================
uint8_t Esp32_t::ConnectToServer() {
    if(IsConnectedToServer) return retvOk;
    for(int i=0; i<4; i++) {
        SendCmd("AT+CIPSTART=\"SSL\",\"my.mishka.cloud\",443");
        while(true) {
            if(GetReply(AP_CONNECT_TIMEOUT_ms) != retvOk) return retvFail;
            if(strcasecmp(IStr, "OK") == 0) {
                IsConnectedToServer = true;
                return retvOk;
            }
            else if(strcasecmp(IStr, "ERROR") == 0) break; // repeat cmd
        }
    } // for
    return retvFail;
}

uint8_t Esp32_t::ReconnectToServer() {
    Printf("Reconnecting\r");
    IPrintfToIStr("+++");
    SendIStr();
    chThdSleepMilliseconds(1008); // Datasheet demands to wait at least 1s before sending next cmd
    IsTransparentModeOn = false;
    SendCmdAndWaitOk("AT+CIPCLOSE"); // Close connection
    IsConnectedToServer = false;
    if(ConnectToServer() != retvOk) return retvDisconnected;
    return EnterTransparentTransmissionMode();
}

uint8_t Esp32_t::EnterTransparentTransmissionMode() {
    if(IsTransparentModeOn) return retvOk;
    if(SendCmdAndWaitOk("AT+CIPMODE=1") != retvOk) return retvFail; // Transmission mode = transparent
    SendCmd("AT+CIPSEND");
    // Wait '>'
    char *S = IStr;
    uint32_t RplLen = 0;
    while(true) {
        if(GetCmdByte(S, SIMPLE_CMD_TIMEOUT_ms) != retvOk) return retvTimeout;
        if(*S == '\r' or *S == '\n') { // End of line
            if(RplLen > 0) { // Something non-empty was received
                *S = 0; // End of string
//                Printf("< %S\r", IStr);
                S = IStr;
                RplLen = 0;
            }
        }
        else if(*S == '>') {
            Printf("Ready2Send\r");
            IsTransparentModeOn = true;
            return retvOk;
        }
        else { // Some other char
            RplLen++;
            if(RplLen > ESP_STR_SZ) return retvOverflow;
            S++;
        }
    } // while true
}

uint8_t Esp32_t::HttpRequestAndGetResponse(const char* APost, const char* AHost, const char* ACmd) {
    // Construct POST request
    IPrintfToIStr(
            "POST %S HTTP/1.1\r\n"
            "Host: %S\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\n"
            "\r\n"
            "%S\r\n"
            "\r\n"
            , APost, AHost, strlen(ACmd), ACmd);
    SendIStr();
    Printf("http >\r%S", IStr);

    return HttpRespHdr.Receive(); // Receive Reply header
}

uint8_t Esp32_t::HttpGetResponseData(int32_t ALen) {
    char *S = IStr;
    for(int32_t i=0; i<ALen; i++) {
        if(GetCmdByte(S, 45000) != retvOk) return retvFail;
        S++;
    }
    *S = 0; // end of string
    return retvOk;
}
#endif

#if 00 // === Common cmd related ====
uint8_t Esp32_t::SendCmdAndWaitOk(const char* ACmd) {
    SendCmd(ACmd);
    while(true) {
        if(GetReply(SIMPLE_CMD_TIMEOUT_ms) != retvOk) return retvFail;
        if(strcasecmp(IStr, "OK") == 0) return retvOk;
    }
}

// Do not append Cmd with CR, it is done inside
void Esp32_t::SendCmd(const char* ACmd, ...) {
    IPrc = IStr;
    va_list args;
    va_start(args, ACmd);
    IVsPrintf(ACmd, args);
    va_end(args);
    IPutChar('\r'); // Add CR
    IPutChar(0);    // End string
#if ESP_VERBOSE
    Printf("> %S", IStr);
#endif
    SendIStr();
}

uint8_t Esp32_t::GetReply(uint32_t Timeout_ms) {
    char *S = IStr;
    uint32_t RplLen = 0;
    while(true) {
        if(GetCmdByte(S, Timeout_ms) != retvOk) return retvTimeout;
        if(*S == '\r' or *S == '\n') { // End of line
            if(RplLen > 0) { // Something non-empty was received
                *S = 0; // End of string
#if ESP_VERBOSE
                Printf("< %S\r", IStr);
#endif
                return retvOk;
            }
        }
        else { // Some other char
            RplLen++;
            if(RplLen > ESP_STR_SZ) return retvOverflow;
            S++;
        }
    } // while true
}
#endif

#if 0 // ==== Printf related ====
uint8_t Esp32_t::IPutChar(char c) {
    *IPrc = c;
    if((IPrc - IStr) >= ESP_STR_SZ) return retvOverflow;
    IPrc++;
    return retvOk;
}

void Esp32_t::IPrintfToIStr(const char *format, ...) {
    IPrc = IStr;
    va_list args;
    va_start(args, format);
    IVsPrintf(format, args);
    va_end(args);
    IPutChar(0); // End string
}

// Does not put \0 at end of string
void Esp32_t::IProceedPrintingToIStr(const char *format, ...) {
    va_list args;
    va_start(args, format);
    IVsPrintf(format, args);
    va_end(args);
}
#endif

#if 0 // ============================= Flasher =================================
// Callbacks
bool EspSetBaudrate(uint32_t baud_rate) {
    ESP_BOOT_UART->BRR = Clk.AHBFreqHz / baud_rate;
    chThdSleepMilliseconds(90); // Without this, ESP will react slower. Magic.
    return true;
}

bool EspSendByte(uint8_t b) {
    while(!(ESP_BOOT_UART->ISR & USART_ISR_TXE));
    ESP_BOOT_UART->TDR = b;
//    Printf(">%02X\r", b);
    WrittenBytes++;
    if(WrittenBytes % 10000 == 0)Printf(" %u\r", WrittenBytes);
    return true;
}

static uint8_t GetByte(uint8_t *b) {
    chSysLock();
    static int32_t RIndx = 0;
    int32_t WIndx = ESP_BUF_SZ - ESP_DMA_RX0->channel->CNDTR;
    int32_t BytesCnt = WIndx - RIndx;
    if(BytesCnt < 0) BytesCnt += ESP_BUF_SZ;
    if(BytesCnt == 0) {
        chSysUnlock();
        return retvEmpty;
    }
    *b = IRx0Buf[RIndx++];
    if(RIndx >= ESP_BUF_SZ) RIndx = 0;
    chSysUnlock();
    return retvOk;
}

bool EspRcvByte(uint8_t &b, uint32_t Timeout_ms) {
    while(Timeout_ms-- > 0) {
        if(GetByte(&b) == retvOk) {
//            Printf("<%02X\r", b);
            return true;
        }
        chThdSleepMilliseconds(1);
    }
    return false;
}

size_t EspFileRead(void *descriptor, uint8_t *data, size_t length) {
    uint32_t ReadSz = 0;
    f_read((FIL*)descriptor, data, length, &ReadSz);
    return ReadSz;
}

void Esp32_t::UpdateFW(const char* FName) {
    EspEn.SetLo(); // Disable ESP
    EspGpio0.SetLo();
    chThdSleepMilliseconds(100);
    EspEn.SetHi();
    chThdSleepMilliseconds(100);
    EspGpio0.SetHi();

    uint8_t md5[Esp32Flasher::MD5_SIZE] = {};

    FIL *PFile = new FIL;
    if(!PFile) return;

    if(TryOpenFileRead(FName, PFile) == retvOk) {
        size_t fwsz = f_size(PFile);
        WrittenBytes = 0;
        Printf("Esp FW sz: %u\r", fwsz);
        Esp32Flasher flasher(EspSetBaudrate, EspSendByte, EspRcvByte, EspFileRead);

        ESP_BOOT_UART->CR1 &= ~USART_CR1_UE; // Disable UART
        // DMA
        ESP_BOOT_UART->CR3 |= USART_CR3_DMAR;
        dmaStreamAllocate     (ESP_DMA_RX0, IRQ_PRIO_LOW, nullptr, nullptr);
        dmaStreamSetPeripheral(ESP_DMA_RX0, &ESP_BOOT_UART->RDR);
        dmaStreamSetMemory0   (ESP_DMA_RX0, IRx0Buf);
        dmaStreamSetTransactionSize(ESP_DMA_RX0, ESP_BUF_SZ);
        dmaStreamSetMode      (ESP_DMA_RX0, UART_DMA_RX_MODE(2));
        dmaStreamEnable       (ESP_DMA_RX0);

        ESP_BOOT_UART->CR1 |= USART_CR1_UE; // Enable UART back

        systime_t IStart = chVTGetSystemTimeX();

        int error = flasher.flash(PFile, 0, static_cast<size_t>(fwsz), md5);
        if(error < 0) {
            Printf("EspFlashErr: %d\r", error);
        }
        else {
            Printf("md5: %A\r", md5, sizeof(md5), ' ');
        }
        Printf("Flashing duration: %u\r", ST2MS(chVTTimeElapsedSinceX(IStart)));

        dmaStreamDisable(ESP_DMA_RX0);
        dmaStreamRelease(ESP_DMA_RX0);

        CloseFile(PFile);
    }
    free(PFile);
}
#endif
