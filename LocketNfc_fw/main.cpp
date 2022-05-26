
#if 1 // ============================ Includes =================================
#include "hal.h"
#include "MsgQ.h"
#include "shell.h"
#include "led.h"
#include "kl_sd.h"
#include "kl_fs_utils.h"
#include "SAI.h"
#include "AuPlayer.h"
//#include "usb_msd.h"
//#include "buttons.h"
//#include "Charger.h"
#include "Sequences.h"
#include "kl_i2c.h"
#include "radio_lvl1.h"
#include "Esp32.h"
#endif
#if 1 // ======================== Variables & prototypes =======================
// Forever
bool OsIsInitialized = false;
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{CmdUartParams};
static const UartParams_t EspUartParams(115200, ESP_UART_PARAMS);
HostUart_t EspUart{EspUartParams};
void OnCmd(Shell_t *PShell);
void ITask();


//PinInput_t PinUsbDetect(USB_DETECT_PIN, pudPullDown);

#define BATTERY_LOW_mv  3200
#define BATTERY_DEAD_mv 3300

//static void Standby();
//static void Resume();

//static void EnterSleepNow();
//static void EnterSleep();
static TmrKL_t TmrOff {TIME_S2I(18), evtIdPwrOffTimeout, tktOneShot};
static TmrKL_t TmrOneSecond {TIME_MS2I(999), evtIdEverySecond, tktPeriodic}; // Measure battery periodically

//static Charger_t Charger;
LedRGB_t Led { LED_R_PIN, LED_G_PIN, LED_B_PIN };
#endif

#define BUF_SZ_FRAME       1024UL
static uint32_t Buf[BUF_SZ_FRAME];

int main(void) {
#if 0 // ==== Get source of wakeup ====
    rccEnablePWRInterface(FALSE);
    if(Sleep::WasInStandby()) {
        // Is it button?
        PinSetupInput(GPIOA, 0, pudPullDown);
        if(PinIsHi(GPIOA, 0)) {
            // Check if pressed long enough
            for(uint32_t i=0; i<270000; i++) {
                // Go sleep if btn released too fast
                if(PinIsLo(GPIOA, 0)) EnterSleepNow();
            }
            // Btn was not released long enough, proceed with powerOn
        }
        else { // Check if USB is connected
            PinSetupInput(GPIOA, 2, pudPullDown);
            if(PinIsLo(GPIOA, 2)) EnterSleepNow(); // Something strange happened
        }
    }
#endif
    // Start Watchdog. Will reset in main thread by periodic 1 sec events.
//    Iwdg::InitAndStart(4500);
//    Iwdg::DisableInDebug();

#if 1 // ==== Iwdg, Clk, Os, EvtQ, Uart ====
    // Setup clock frequency
    Clk.SetVoltageRange(mvrHiPerf);
    Clk.SetupFlashLatency(48, mvrHiPerf);
    Clk.EnablePrefetch();
    if(Clk.EnableHSE() == retvOk) {
        Clk.SetupPllSrc(pllsrcHse);
        Clk.SetupM(3);
    }
    else { // PLL fed by MSI
        Clk.SetupPllSrc(pllsrcMsi);
        Clk.SetupM(1);
    }
    // SysClock 48MHz
    Clk.SetupPll(24, 2, 2);
    Clk.SetupBusDividers(ahbDiv1, apbDiv1, apbDiv1);
    if(Clk.EnablePLL() == retvOk) {
        Clk.EnablePllROut();
        Clk.SwitchToPLL();
        // Setup PLLQ as 48MHz clock for USB and SDIO
        Clk.EnablePllQOut();
//        Clk.SetupSai1Qas48MhzSrc();
        uint32_t tmp = RCC->CCIPR;
        tmp &= ~RCC_CCIPR_CLK48SEL;
        tmp |= 0b10UL << 26; // PLLQ is source
        // ADC clock = SYSCLK
        tmp &= ~RCC_CCIPR_ADCSEL;
        tmp |= 0b11UL << 28; // SYSCLK is ADC clock
        RCC->CCIPR = tmp;
    }
//    Clk.SetupPllSai1(24, 4, 2, 7); // 4MHz * 24 = 96; R = 96 / 4 = 24, Q = 96 / 2 = 48
//    if(Clk.EnablePllSai1() == retvOk) {
//        // Setup Sai1R as ADC source
//        Clk.EnableSai1ROut();
//        uint32_t tmp = RCC->CCIPR;
//        tmp &= ~RCC_CCIPR_ADCSEL;
//        tmp |= 0b01UL << 28; // SAI1R is ADC clock
//        // Setup Sai1Q as 48MHz source
//        Clk.EnableSai1QOut();
//        tmp &= ~RCC_CCIPR_CLK48SEL;
//        tmp |= 0b01UL << 26;
//        RCC->CCIPR = tmp;
//    }
    Clk.UpdateFreqValues();

    // Init OS
    halInit();
    chSysInit();
    OsIsInitialized = true;

    // ==== Init hardware ====
    EvtQMain.Init();
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk.PrintFreqs();
#endif

    // Check if IWDG frozen in standby
    if(!Flash::IwdgIsFrozenInStandby()) {
        Printf("IWDG not frozen in Standby, frozing\r");
        chThdSleepMilliseconds(45);
        Flash::IwdgFrozeInStandby();
    }

    Led.Init();
//    PinUsbDetect.Init();
//    Buttons::Init();
//    Charger.Init();
//    AuPlayer.Init();


//    Esp::Init();
//    Esp::Start();
//    EspUart.Init();

    TmrOneSecond.StartOrRestart();

    SD.Init();
    Sai.Init();
    Sai.SetupSampleRate(16000);
//    Sai.EnableSAI();

    for(uint32_t i=0; i<BUF_SZ_FRAME; i++) {
        Buf[i] = 0x8000c000;
    }
    Sai.TransmitBuf(Buf, BUF_SZ_FRAME);

    // Init if SD ok
    if(SD.IsReady) {
        Led.StartOrRestart(lsqStart);
//        UsbMsd.Init();
//        AuPlayer.Play("WakeUp.wav", spmSingle);
    } // if SD is ready
    else {
        Led.StartOrRestart(lsqFail);
//        chThdSleepMilliseconds(3600);
//        EnterSleep();
    }

//    Radio.Init();

    // Main cycle
    ITask();
}


void OnDmaSaiTxIrqI() {
    Sai.TransmitBuf(Buf, BUF_SZ_FRAME);
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdShellCmdRcvd:
                while(((CmdUart_t*)Msg.Ptr)->TryParseRxBuff() == retvOk) OnCmd((Shell_t*)((CmdUart_t*)Msg.Ptr));
                break;

            case evtIdPwrOffTimeout:
                Printf("TmrOff timeout\r");
                break;
/*
            case evtIdButtons:
//                Printf("Btn %u %u\r", Msg.BtnInfo.ID, Msg.BtnInfo.Evt);
                if(Msg.BtnInfo.Evt == bePress) {
                    Resume();
                    IsPlayingIntro = false;
                    Songs[Msg.BtnInfo.ID - 1].Play();
                    Radio.ClrToTx = Clrs[Msg.BtnInfo.ID - 1];
                    Radio.BtnIndx = Msg.BtnInfo.ID - 1;
                    Radio.MustTx = true;
                }
                else if(Msg.BtnInfo.Evt == beRelease) {
                    if(IsPlayingIntro) IsPlayingIntro = false;
                    else if(Buttons::AreAllIdle()) AuPlayer.FadeOut();
                    Radio.MustTx = false;
                }
                else if(Msg.BtnInfo.Evt == beCombo) {
                    Resume();
                    IsPlayingIntro = true;
                    MustSleep = true;
                    AuPlayer.Play("Sleep.wav", spmSingle);
                    Radio.MustTx = false;
                }
                break;

            // ==== Sound ====
            case evtIdAudioPlayStop:
//                Printf("Snd Done\r");
                IsPlayingIntro = false;
                if(MustSleep) EnterSleep();
                Standby();
                break;
*/

            case evtIdBtDevFound:
                Printf("BT dev found: %S\r", Esp::BtAddr);
                break;
            case evtIdBtDevConnected:
                Printf("BT dev connected\r");
                break;


            case evtIdEverySecond:
//                Printf("Second\r");
                Iwdg::Reload();
                /*
                // Check SD
                if(!UsbConnected and !SD.IsReady) {
                    if(SD.Reconnect() == retvOk) Led.StartOrRestart(lsqOk);
                    else Led.StartOrContinue(lsqFail);
                }
                // Check USB
                if(PinUsbDetect.IsHi() and !PinUsbIsHigh) {
                    PinUsbIsHigh = true;
                    EvtQMain.SendNowOrExit(EvtMsg_t(evtIdUsbConnect));
                }
                else if(!PinUsbDetect.IsHi() and PinUsbIsHigh) {
                    PinUsbIsHigh = false;
                    EvtQMain.SendNowOrExit(EvtMsg_t(evtIdUsbDisconnect));
                    Led.StartOrContinue(lsqOk);
                }
                // Charger
                Charger.OnSecond(PinUsbIsHigh);

                // Check battery
                if(!IsStandby) {
                    uint32_t Battery_mV = Codec.GetBatteryVmv();
//                    Printf("%u\r", Battery_mV);
                    if(Battery_mV < BATTERY_DEAD_mv) {
                        Printf("Discharged: %u\r", Battery_mV);
//                        EnterSleep();
                    }
                }

                */
                break;

#if 0 // ======= USB =======
            case evtIdUsbConnect:
                Printf("USB connect\r");
                Resume();
                UsbConnected = true;
                UsbMsd.Connect();
                Charger.Enable();
                break;
            case evtIdUsbDisconnect:
                Standby();
                Printf("USB disconnect\r");
                UsbConnected = false;
                UsbMsd.Disconnect();
                break;
            case evtIdUsbReady:
                Printf("USB ready\r");
                break;
#endif
            default: break;
        } // switch
    } // while true
}

/*
void Resume() {
    if(!IsStandby) return;
    Printf("Resume\r");
    // Clock
    Clk.SetCoreClk(cclk48MHz);
    Clk.SetupSai1Qas48MhzSrc();
    Clk.UpdateFreqValues();
    Clk.PrintFreqs();
    // Sound
    Codec.Init();
    Codec.SetSpeakerVolume(-96);    // To remove speaker pop at power on
    Codec.DisableHeadphones();
    Codec.EnableSpeakerMono();
    Codec.SetupMonoStereo(Stereo);  // For wav player
    Codec.SetupSampleRate(22050); // Just default, will be replaced when changed
    Codec.SetMasterVolume(9); // 12 is max
    Codec.SetSpeakerVolume(0); // 0 is max

    IsStandby = false;
}

void Standby() {
    Printf("Standby\r");
    // Sound
    Codec.Deinit();
    // Clock
    Clk.SwitchToMSI();
    Clk.DisablePLL();
    Clk.DisableSai1();

    Clk.UpdateFreqValues();
    Clk.PrintFreqs();
    IsStandby = true;
}

void EnterSleepNow() {
    // Enable inner pull-ups
//    PWR->PUCRC |= PWR_PUCRC_PC13;
    // Enable inner pull-downs
    PWR->PDCRA |= PWR_PDCRA_PA0 | PWR_PDCRA_PA2;
//    PWR->PDCRC |= PWR_PDCRC_PC13;
    // Apply PullUps and PullDowns
    PWR->CR3 |= PWR_CR3_APC;
    // Enable wake-up srcs
    Sleep::EnableWakeup1Pin(rfRising); // Btn1
//    Sleep::EnableWakeup2Pin(rfRising); // Btn2
    Sleep::EnableWakeup4Pin(rfRising); // USB
    Sleep::ClearWUFFlags();
    Sleep::EnterStandby();
}

void EnterSleep() {
    Printf("Entering sleep\r");
    chThdSleepMilliseconds(45);
    chSysLock();
    EnterSleepNow();
    chSysUnlock();
}
*/

#if 1 // ======================= Command processing ============================
void OnCmd(Shell_t *PShell) {
	Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    else if(PCmd->NameIs("mem")) PrintMemoryInfo();

    else if(PCmd->NameIs("espen")) {
        Esp::Enable();
        PShell->Ok();
    }
    else if(PCmd->NameIs("espDis")) {
        Esp::Disable();
        PShell->Ok();
    }
    else if(PCmd->NameIs("espG0Hi")) {
        Esp::SetGpio0Hi();
        PShell->Ok();
    }
    else if(PCmd->NameIs("espG0Lo")) {
        Esp::SetGpio0Lo();
        PShell->Ok();
    }

    else if(PCmd->NameIs("esp")) {
        char* S;
        while((S = PCmd->GetNextString()) != nullptr) {
            EspUart.Print("%S ", S);
        }
        EspUart.Print("\r\n"); // End of cmd

//        while(EspUart.WaitReply(Timeout_ms)
    }

    else if(PCmd->NameIs("espDiscover")) {
        Esp::Discover();
        PShell->Ok();
    }
    else if(PCmd->NameIs("espStopDiscover")) {
        Esp::StopDiscover();
        PShell->Ok();
    }
    else if(PCmd->NameIs("espConnect")) {
        char *S;
        if((S = PCmd->GetNextString()) == nullptr) S = Esp::BtAddr;
        Esp::Connect(S);
        PShell->Ok();
    }
    else if(PCmd->NameIs("espDisconnect")) {
        Esp::Disconnect();
        PShell->Ok();
    }

    else if(PCmd->NameIs("play")) {
//        int32_t v1, v2;
//        if(PCmd->GetNext<int32_t>(&v1) != retvOk) return;
//        if(PCmd->GetNext<int32_t>(&v2) != retvOk) return;
        AuPlayer.PlayAlive();
    }

    else if(PCmd->NameIs("Sampr")) {
        uint32_t sr;
        if(PCmd->GetNext<uint32_t>(&sr) == retvOk) {
            Sai.SetupSampleRate(sr);
//            Sai.EnableSAI();
            Sai.TransmitBuf(Buf, BUF_SZ_FRAME);
            PShell->Ok();
        }
        else PShell->BadParam();
    }

    else PShell->CmdUnknown();
}
#endif
