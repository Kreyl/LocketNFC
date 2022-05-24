/*
 * Esp32.h
 *
 *  Created on: 6 ���. 2018 �.
 *      Author: Kreyl
 */

#pragma once

#include <inttypes.h>
#include "shell.h"
#include "MsgQ.h"
#include "kl_lib.h"
#include "kl_buf.h"
#include "ff.h"

//class Esp32_t { //: private PrintfHelper_t {
//private:
//    // General commands
//    uint8_t IPwrOnIfNotYet();
//    void ISwitchOff();
//    // Printf to buf
////    char *IPrc;
////    uint8_t IPutChar(char c);
////    void IPrintfToIStr(const char *format, ...);
////    void IProceedPrintingToIStr(const char *format, ...);
////    void IStartTransmissionIfNotYet() {} // dummy
//public:
//    void Init();
//    void UpdateFW(const char* FName);
//
//    // High level
//    Stringlist_t *StrList = nullptr;
//
//    // Inner use
//    void ITask();
////    EvtMsgQ_t<WiFiEvt_t, WIFI_MSGQ_LEN> WMsgQ;
//};
//
//extern Esp32_t Esp32;

#define BT_ADDR_STRLEN  18

namespace Esp {

extern char BtAddr[BT_ADDR_STRLEN];
extern bool BtConnected;

void Init();
uint8_t Start();
void PwrOff();

void Enable();
void Disable();

void SetGpio0Hi();
void SetGpio0Lo();

void Discover();
void StopDiscover();

void Connect(char* S);
void Disconnect();
}
