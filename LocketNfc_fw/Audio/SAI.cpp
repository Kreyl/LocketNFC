/*
 * CS42L52.cpp
 *
 *  Created on: 15 ����� 2017 �.
 *      Author: Kreyl
 */

#include <SAI.h>
#include "shell.h"
#include "kl_i2c.h"

//static const stm32_dma_stream_t *PDmaTx;
//static const stm32_dma_stream_t *PDmaRx;

//__attribute__((weak))
//void AuOnNewSampleI(SampleStereo_t &Sample) { }

#if 1 // =========================== SAI defins ================================
#define SAI_IRQ_NUMBER          74
#define SAI_IRQ_HANDLER         Vector168

#define SAI_FIFO_THR_EMPTY      0
#define SAI_FIFO_THR_1_4        1
#define SAI_FIFO_THR_1_2        2
#define SAI_FIFO_THR_3_4        3
#define SAI_FIFO_THR_FULL       4
#define SAI_FIFO_THR            SAI_FIFO_THR_1_2

#define SAI_CR1_DATASZ_8BIT     ((uint32_t)(0b010 << 5))
#define SAI_CR1_DATASZ_10BIT    ((uint32_t)(0b011 << 5))
#define SAI_CR1_DATASZ_16BIT    ((uint32_t)(0b100 << 5))
#define SAI_CR1_DATASZ_20BIT    ((uint32_t)(0b101 << 5))
#define SAI_CR1_DATASZ_24BIT    ((uint32_t)(0b110 << 5))
#define SAI_CR1_DATASZ_32BIT    ((uint32_t)(0b111 << 5))

#define SAI_SYNC_ASYNC          ((uint32_t)(0b00 << 10))
#define SAI_SYNC_INTERNAL       ((uint32_t)(0b01 << 10))

#define SAI_RISING_EDGE         ((uint32_t)(0 << 9))
#define SAI_FALLING_EDGE        ((uint32_t)(1 << 9))

// Slots related
#define SAI_SLOT_CNT            2
#define SAI_SlotActive_0        (1 << 16)
#define SAI_SlotActive_1        (1 << 17)
#define SAI_SLOTSZ_EQ_DATASZ    (0b00 << 6)
#define SAI_SLOTSZ_16bit        (0b01 << 6)
#define SAI_SLOTSZ_32bit        (0b10 << 6)

#define SAI_MASTER_TX           ((uint32_t)0x00000000)
#define SAI_MASTER_RX           (SAI_xCR1_MODE_0)
#define SAI_SLAVE_TX            (SAI_xCR1_MODE_1)
#define SAI_SLAVE_RX            (SAI_xCR1_MODE_1 | SAI_xCR1_MODE_0)

#define SAI_DMATX_MONO_MODE  \
                        STM32_DMA_CR_CHSEL(SAI_DMA_CHNL) |   \
                        DMA_PRIORITY_MEDIUM | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P |  /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE       /* Enable Transmission Complete IRQ */

#define SAI_DMATX_STEREO_MODE  \
                        STM32_DMA_CR_CHSEL(SAI_DMA_CHNL) |   \
                        DMA_PRIORITY_MEDIUM | \
                        STM32_DMA_CR_MSIZE_WORD | \
                        STM32_DMA_CR_PSIZE_WORD | \
                        STM32_DMA_CR_MINC |     /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_M2P |  /* Direction is memory to peripheral */ \
                        STM32_DMA_CR_TCIE       /* Enable Transmission Complete IRQ */

#define SAI_DMARX_MODE  STM32_DMA_CR_CHSEL(Chnl) |   \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_BYTE | \
                        STM32_DMA_CR_PSIZE_BYTE | \
                        STM32_DMA_CR_MINC |         /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |      /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_CIRC           /* Circular buffer enable */
//                        STM32_DMA_CR_TCIE           /* Enable Transmission Complete IRQ */
#endif

// DMA Tx Completed IRQ
extern "C"
void DmaSAITxIrq(void *p, uint32_t flags) {
    chSysLockFromISR();
//    if(Codec.SaiDmaCallbackI) Codec.SaiDmaCallbackI();
    chSysUnlockFromISR();
}

#if 0 // ======= Setup SAI =======
    // === Clock ===
    Clk.EnableMCO(mcoHSE, mcoDiv1); // Master clock output
    AU_SAI_RccEn();

    // === GPIOs ===
    PinSetupAlterFunc(AU_LRCK); // Left/Right (Frame sync) clock output
    PinSetupAlterFunc(AU_SCLK); // Bit clock output
    PinSetupAlterFunc(AU_SDIN); // SAI_A is Slave Transmitter

    DisableSAI();   // All settings must be changed when both blocks are disabled
    // Sync setup: SaiA async, SaiB sync
    AU_SAI->GCR = 0;    // No external sync input/output

    // === Setup SAI_A as async Slave Transmitter ===
    // Stereo mode, Async, MSB first, Rising edge, Data Sz = 16bit, Free protocol, Slave Tx
    AU_SAI_A->CR1 = SAI_SYNC_ASYNC | SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_SLAVE_TX;
    // No offset, FS Active Low, FS Active Lvl Len = 1, Frame Len = 32
    AU_SAI_A->FRCR = ((1 - 1) << 8) | (62 - 1);
    // 0 & 1 slots en, N slots = 2, slot size = 16bit, no offset
    AU_SAI_A->SLOTR = SAI_SlotActive_0 | SAI_SlotActive_1 | ((SAI_SLOT_CNT - 1) << 8) | SAI_SLOTSZ_16bit;
    AU_SAI_A->IMR = 0;  // No irq on TX

#if MIC_EN    // === Setup SAI_B as Slave Receiver ===
    PinSetupAlterFunc(AU_SDOUT); // SAI_B is Slave Receiver
    // Stereo mode, sync with sub-block, MSB first, Rising edge, Data Sz = 16bit, Free protocol, Slave Rx
    AU_SAI_B->CR1 = SAI_SYNC_INTERNAL | SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_SLAVE_RX;
    AU_SAI_B->FRCR = AU_SAI_A->FRCR;
    AU_SAI_B->SLOTR = AU_SAI_A->SLOTR;
    AU_SAI_B->IMR = 0;  // No irq on RX
#endif
#endif

#if 0 // ==== DMA ====
    AU_SAI_A->CR1 |= SAI_xCR1_DMAEN;
    PDmaTx = dmaStreamAlloc(SAI_DMA_A, IRQ_PRIO_MEDIUM, DmaSAITxIrq, nullptr);
    dmaStreamSetPeripheral(PDmaTx, &AU_SAI_A->DR);
#endif

    /*
void CS42L52_t::Deinit() {
    if(PDmaTx) {
        dmaStreamDisable(PDmaTx);
        dmaStreamFree(PDmaTx);
        PDmaTx = nullptr;
    }
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH;
    Clk.DisableMCO();
    PinRst.SetLo();
    AU_SAI_RccDis();
    IsOn = false;
}

void CS42L52_t::Standby() {
    WriteReg(CS_R_PWR_CTRL1, 0xFF);
    IsOn = false;
}

void CS42L52_t::Resume() {
    // PwrCtrl 1: Power on codec only
    WriteReg(CS_R_PWR_CTRL1, 0b11111110);
    IsOn = true;
}
*/

#if 0 // ============================= Tx/Rx ===================================
void CS42L52_t::SetupMonoStereo(MonoStereo_t MonoStereo) {
    dmaStreamDisable(PDmaTx);
    DisableSAI();   // All settings must be changed when both blocks are disabled
    // Wait until really disabled
    while(AU_SAI_A->CR1 & SAI_xCR1_SAIEN);
    // Setup mono/stereo
    if(MonoStereo == Stereo) AU_SAI_A->CR1 &= ~SAI_xCR1_MONO;
    else AU_SAI_A->CR1 |= SAI_xCR1_MONO;
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH | SAI_FIFO_THR; // Flush FIFO
}

void CS42L52_t::SetupSampleRate(uint32_t SampleRate) {  // Setup sample rate. No Auto, 32kHz, not27MHz
//    Printf("CS fs: %u\r", SampleRate);
    uint8_t                      v = (0b10 << 5) | (1 << 4) | (0 << 3) | (0b01 << 1);    // 16 kHz
    if     (SampleRate == 22050) v = (0b10 << 5) | (0 << 4) | (0 << 3) | (0b11 << 1);
    else if(SampleRate == 44100) v = (0b01 << 5) | (0 << 4) | (0 << 3) | (0b11 << 1);
    else if(SampleRate == 48000) v = (0b01 << 5) | (0 << 4) | (0 << 3) | (0b01 << 1);
    else if(SampleRate == 96000) v = (0b00 << 5) | (0 << 4) | (0 << 3) | (0b01 << 1);
    WriteReg(0x05, v);
//    Printf("v: %X\r", v);
}

void CS42L52_t::TransmitBuf(volatile void *Buf, uint32_t Sz16) {
    dmaStreamDisable(PDmaTx);
    dmaStreamSetMemory0(PDmaTx, Buf);
    dmaStreamSetMode(PDmaTx, SAI_DMATX_MONO_MODE);
    dmaStreamSetTransactionSize(PDmaTx, Sz16);
    dmaStreamEnable(PDmaTx);
    EnableSAI(); // Start tx
}

bool CS42L52_t::IsTransmitting() {
    return (dmaStreamGetTransactionSize(PDmaTx) != 0);
}

void CS42L52_t::Stop() {
    dmaStreamDisable(PDmaTx);
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH;
}

void CS42L52_t::StartStream() {
    DisableSAI();   // All settings must be changed when both blocks are disabled
    dmaStreamDisable(PDmaTx);
#if MIC_EN
    dmaStreamDisable(SAI_DMA_B);
#endif
    AU_SAI_A->CR1 &= ~(SAI_xCR1_MONO | SAI_xCR1_DMAEN); // Always stereo, no DMA
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH | SAI_FIFO_THR; // Flush FIFO
    AU_SAI_B->CR2 = SAI_xCR2_FFLUSH | SAI_FIFO_THR; // Flush FIFO
    // Setup IRQ
    AU_SAI_B->IMR = SAI_xIMR_FREQIE;
    nvicEnableVector(SAI_IRQ_NUMBER, IRQ_PRIO_MEDIUM);
    EnableSAI();
}

void CS42L52_t::PutSampleI(SampleStereo_t &Sample) {
    AU_SAI_A->DR = Sample.Right;    // }
    AU_SAI_A->DR = Sample.Left;     // } Somehow Left will be sent first if put last
}
#endif


#if 0 // ============================== IRQ ====================================
extern "C"
OSAL_IRQ_HANDLER(SAI_IRQ_HANDLER) {
    OSAL_IRQ_PROLOGUE();
    if(AU_SAI_B->SR & SAI_xSR_FREQ) {
        SampleStereo_t Sample;
        Sample.Left = AU_SAI_B->DR;
        Sample.Right = AU_SAI_B->DR;
        AuOnNewSampleI(Sample);
    }
    OSAL_IRQ_EPILOGUE();
}

#endif
