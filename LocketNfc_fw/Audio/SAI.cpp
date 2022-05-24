/*
 * CS42L52.cpp
 *
 *  Created on: 15 ����� 2017 �.
 *      Author: Kreyl
 */

#include <SAI.h>
#include "shell.h"
#include "kl_i2c.h"

SAI_t Sai;
void OnDmaSaiTxIrqI();

static const stm32_dma_stream_t *PDmaTx;
//static const stm32_dma_stream_t *PDmaRx;

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
    // DO nothing if disabled
    if(AU_SAI_A->CR1 & SAI_xCR1_SAIEN) OnDmaSaiTxIrqI();
    chSysUnlockFromISR();
}

void SAI_t::Init() {
    // === Clock ===
    AU_SAI_RccEn();
    // PLLSAI2
    Clk.EnablePllSai2POut();
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_SAI1SEL) | (0b01UL << 22); // PLL2P is clock for SAI1

    // === GPIOs ===
    PinSetupAlterFunc(AU_LRCK); // Left/Right (Frame sync) clock output
    PinSetupAlterFunc(AU_SCLK); // Bit clock output
    PinSetupAlterFunc(AU_SDIN); // SAI_A is Transmitter

    Disable();   // All settings must be changed when both blocks are disabled
    // Sync setup: SaiA async, SaiB sync
    AU_SAI->GCR = 0;    // No external sync input/output

    // === Setup SAI_A as async Master Transmitter ===
    // Stereo mode, Async, MSB first, Rising edge, Data Sz = 16bit, Free protocol, Slave Tx
//    AU_SAI_A->CR1 = SAI_SYNC_ASYNC | SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_SLAVE_TX;
    // Stereo mode, Async, MSB first, Rising edge, Data Sz = 16bit, Free protocol, Master Tx
    AU_SAI_A->CR1 = SAI_SYNC_ASYNC | SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_MASTER_TX;
    // No offset, FS Active Low, FS is start + ch side (I2S), FS Active Lvl Len = 16, Frame Len = 32
    AU_SAI_A->FRCR = SAI_xFRCR_FSDEF | ((16 - 1) << 8) | (32 - 1);
    // 0 & 1 slots en, N slots = 2, slot size = DataSz in CR1, no offset
    AU_SAI_A->SLOTR = SAI_SlotActive_0 | SAI_SlotActive_1 | ((SAI_SLOT_CNT - 1) << 8) | SAI_SLOTSZ_EQ_DATASZ;
    AU_SAI_A->IMR = 0;  // No irq on TX

#if MIC_EN    // === Setup SAI_B as Slave Receiver ===
    PinSetupAlterFunc(AU_SDOUT); // SAI_B is Slave Receiver
    // Stereo mode, sync with sub-block, MSB first, Rising edge, Data Sz = 16bit, Free protocol, Slave Rx
    AU_SAI_B->CR1 = SAI_SYNC_INTERNAL | SAI_RISING_EDGE | SAI_CR1_DATASZ_16BIT | SAI_SLAVE_RX;
    AU_SAI_B->FRCR = AU_SAI_A->FRCR;
    AU_SAI_B->SLOTR = AU_SAI_A->SLOTR;
    AU_SAI_B->IMR = 0;  // No irq on RX
#endif

#if 1 // ==== DMA ====
    AU_SAI_A->CR1 |= SAI_xCR1_DMAEN;
    PDmaTx = dmaStreamAlloc(SAI_DMA_A, IRQ_PRIO_MEDIUM, DmaSAITxIrq, nullptr);
    dmaStreamSetPeripheral(PDmaTx, &AU_SAI_A->DR);
#endif
}

void SAI_t::Enable() {
    AU_SAI_A->CR1 |= SAI_xCR1_SAIEN;
}
void SAI_t::Disable() {
    AU_SAI_A->CR1 &= ~SAI_xCR1_SAIEN;
    while(AU_SAI_A->CR1 & SAI_xCR1_SAIEN); // Poll until disabled.Will be disabled after current frame transmission is ended.
}

void SAI_t::Deinit() {
    if(PDmaTx) {
        dmaStreamDisable(PDmaTx);
        dmaStreamFree(PDmaTx);
        PDmaTx = nullptr;
    }
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH;
//    Clk.DisableMCO();
    AU_SAI_RccDis();
}


/*
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
*/

// Samplerate: 16000, 22050, 32000, 44100, 48000
uint8_t SAI_t::SetupSampleRate(uint32_t SampleRate) {
    Stop();
    // PLL must be disabled to change its setup
    Clk.DisablePllSai2();
    AU_SAI_A->CR1 &= ~SAI_xCR1_MCKDIV;
    switch(SampleRate) {
        case 16000:
            Clk.SetupPllSai2(43, 2, 7); // (4MHz * 43 / 7) / 256const / 6mclk = 15997
            AU_SAI_A->CR1 |= (3UL << 20); // MCLKDIV=3 => div by 6
            break;
        case 22050:
            Clk.SetupPllSai2(48, 2, 17);
            AU_SAI_A->CR1 |= (1UL << 20); // MCLKDIV=1 => div by 2
            break;
        case 32000:
            Clk.SetupPllSai2(86, 2, 7);
            AU_SAI_A->CR1 |= (3UL << 20); // MCLKDIV=3 => div by 6
            break;
        case 44100:
            Clk.SetupPllSai2(48, 2, 17); // (4MHz * 48 / 17) / 256const / 1mclk = 44117
            AU_SAI_A->CR1 |= (0UL << 20); // MCLKDIV=0 => div by 1
            break;
        case 48000:
            Clk.SetupPllSai2(43, 2, 7); // (4MHz * 43 / 7) / 256const / 2mclk = 47991
            AU_SAI_A->CR1 |= (1UL << 20); // MCLKDIV=1 => div by 2
            break;
        default:
            Printf("Sai: bad sample rate\r\n");
            return retvBadValue;
            break;
    }
    // Start PLL
    if(Clk.EnablePllSai2() == retvOk) return retvOk;
    else {
        Printf("SAI1 fail\r");
        return retvFail;
    }
}


void SAI_t::TransmitBuf(volatile void *Buf, uint32_t Sz16) {
    dmaStreamDisable(PDmaTx);
    dmaStreamSetMemory0(PDmaTx, Buf);
    dmaStreamSetMode(PDmaTx, SAI_DMATX_MONO_MODE);
    dmaStreamSetTransactionSize(PDmaTx, Sz16);
    dmaStreamEnable(PDmaTx);
    Enable(); // Start tx
}

bool SAI_t::IsTransmitting() {
    return (dmaStreamGetTransactionSize(PDmaTx) != 0);
}

void SAI_t::Stop() {
    dmaStreamDisable(PDmaTx);
    Disable();
    AU_SAI_A->CR2 = SAI_xCR2_FFLUSH;
}

void SAI_t::StartStream() {
    Disable();   // All settings must be changed when both blocks are disabled
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
    Enable();
}

void SAI_t::PutSampleI(SampleStereo_t &Sample) {
    AU_SAI_A->DR = Sample.Right;    // }
    AU_SAI_A->DR = Sample.Left;     // } Somehow Left will be sent first if put last
}


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
