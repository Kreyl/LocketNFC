/*
 * CS42L52.h
 *
 *  Created on: 15 ����� 2017 �.
 *      Author: Kreyl
 */

#pragma once

#include "kl_lib.h"

union SampleStereo_t {
    uint32_t DWord32;
    struct {
        int16_t Left, Right;
    } __packed;
} __packed;

typedef int16_t SampleMono_t;

enum MonoStereo_t { Stereo, Mono };

enum SaiMode_t : uint32_t {
    saiModeMasterTransmitter = 0b00UL,
    saiModeMasterReceiver    = 0b01UL,
    saiModeSlaveTransmitter  = 0b10UL,
    saiModeSlaveReceiver     = 0b11UL,
};

enum SaiClkEdge_t : uint32_t {
    saiEdgeRising  = (0UL << 9),
    saiEdgeFalling = (1UL << 9),
};

class SAI_t {
public:
//    ftVoidVoid TxDmaCallbackI = nullptr;
    void Init(SaiMode_t AMode, SaiClkEdge_t AEdge);
    void Deinit();
    uint8_t SetupSampleRate(uint32_t SampleRate);
    void SetupMonoStereo(MonoStereo_t MonoStereo);
    void TransmitBuf(volatile void *Buf, uint32_t Sz16);
    bool IsTransmitting();
    void Stop();
    void StartStream();
    void PutSampleI(SampleStereo_t &Sample);
    void Enable();
    void Disable();
};

extern SAI_t Sai;

void OnDmaSaiTxIrqI();

/*
    // Hi-level
    uint8_t SetMasterVolume(int8_t Volume_dB);
    uint8_t SetHeadphoneVolume(int8_t Volume_dB);
    uint8_t SetSpeakerVolume(int8_t Volume_dB);

    void VolumeUp();
    void VolumeDown();
    void SetVolume(int8_t AVolume);
    int8_t GetVolume() { return IVolume; }

    // Rx/Tx
    void SetupMonoStereo(MonoStereo_t MonoStereo);
    void SetupSampleRate(uint32_t SampleRate);
    void TransmitBuf(volatile void *Buf, uint32_t Sz16);
    bool IsTransmitting();
    void Stop();

    void StartStream();
    void PutSampleI(SampleStereo_t &Sample);

void AuOnNewSampleI(SampleStereo_t &Sample);
*/
