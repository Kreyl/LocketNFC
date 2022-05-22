/*
 * board.h
 *
 *  Created on: 05 08 2018
 *      Author: Kreyl
 */

#pragma once

// ==== General ====
#define APP_NAME            "LocketNFC"

// Version of PCB
#define PCB_VER                 1

// MCU type as defined in the ST header.
#define STM32L476xx

// Freq of external crystal if any. Leave it here even if not used.
#define CRYSTAL_FREQ_HZ         12000000

// OS timer settings
#define STM32_ST_IRQ_PRIORITY   2
#define STM32_ST_USE_TIMER      5
#define STM32_TIMCLK1           (Clk.APB1FreqHz)    // Timer 5 is clocked by APB1

//  Periphery
#define I2C1_ENABLED            FALSE
#define I2C2_ENABLED            TRUE
#define I2C3_ENABLED            FALSE

#define ADC_REQUIRED            FALSE
#define STM32_DMA_REQUIRED      TRUE    // Leave this macro name for OS

#if 1 // ========================== GPIO =======================================
// EXTI
#define INDIVIDUAL_EXTI_IRQ_REQUIRED    FALSE

// Buttons
#define BTN1_PIN        GPIOA, 0, pudPullDown
#define BTN2_PIN        GPIOC, 14, pudPullDown
#define BTN3_PIN        GPIOC, 13, pudPullDown

// Charging
#define IS_CHARGING_PIN GPIOD, 12

// UART
#define UART_TX_PIN     GPIOA, 9
#define UART_RX_PIN     GPIOA, 10

// USB
#define USB_DM          GPIOA, 11
#define USB_DP          GPIOA, 12
#define USB_AF          AF10
// USB detect
#define USB_DETECT_PIN  GPIOA, 2

// MFRC52202
#define NFC_RST         GPIOA, 3
#define NFC_IRQ         GPIOC, 5
#define NFC_NSS         GPIOA, 4
#define NFC_SCK         GPIOA, 5
#define NFC_MOSI        GPIOA, 7
#define NFC_MISO        GPIOA, 6

// Vibro
#define VIBRO_SETUP     { GPIOC, 6, TIM3, 1, invNotInverted, omPushPull, 99 }

// SD
#define SD_PWR_PIN      GPIOC, 7
#define SD_AF           AF12
#define SD_DAT0         GPIOC,  8, omPushPull, pudPullUp, SD_AF
#define SD_DAT1         GPIOC,  9, omPushPull, pudPullUp, SD_AF
#define SD_DAT2         GPIOC, 10, omPushPull, pudPullUp, SD_AF
#define SD_DAT3         GPIOC, 11, omPushPull, pudPullUp, SD_AF
#define SD_CLK          GPIOC, 12, omPushPull, pudNone,   SD_AF
#define SD_CMD          GPIOD,  2, omPushPull, pudPullUp, SD_AF

// ESP32
#define ESP_GPIO15      GPIOD, 1
#define ESP_EN          GPIOD, 3
#define ESP_PWR_EN      GPIOD, 7
#define ESP_GPIO0       GPIOD, 4
#define ESP_AUX         GPIOE, 3
#define ESP_TX0         GPIOD, 6
#define ESP_RX0         GPIOD, 5
#define ESP_RX2_MCU_TX  GPIOD, 8
#define ESP_TX2_MCU_RX  GPIOD, 9
// Audio
#define AU_LRCK         GPIOE, 4, omPushPull, pudNone, AF13
#define AU_SCLK         GPIOE, 5, omPushPull, pudNone, AF13
#define AU_SDIN         GPIOE, 6, omPushPull, pudNone, AF13 // MOSI; SAI1_A
#define AU_SAI          SAI1
#define AU_SAI_A        SAI1_Block_A
#define AU_SAI_B        SAI1_Block_B
#define AU_SAI_RccEn()  RCC->APB2ENR |= RCC_APB2ENR_SAI1EN
#define AU_SAI_RccDis() RCC->APB2ENR &= ~RCC_APB2ENR_SAI1EN

// LED
#define LED_R_PIN       { GPIOD, 13, TIM4, 2, invNotInverted, omPushPull, 255 }
#define LED_B_PIN       { GPIOD, 14, TIM4, 3, invNotInverted, omPushPull, 255 }
#define LED_G_PIN       { GPIOD, 15, TIM4, 4, invNotInverted, omPushPull, 255 }

// Acc
#define ACG_IRQ_PIN     GPIOB, 0
#define ACG_PWR_PIN     GPIOE, 15

// Radio: SPI, PGpio, Sck, Miso, Mosi, Cs, Gdo0
#define CC_Setup0       SPI2, GPIOB, 13,14,15, GPIOB,12, GPIOB,1

#define DBG_PIN         GPIOB, 8

// I2C
#define I2C1_GPIO       GPIOB
#define I2C1_SCL        6
#define I2C1_SDA        7
#define I2C2_GPIO       GPIOB
#define I2C2_SCL        10
#define I2C2_SDA        11
#define I2C3_GPIO       GPIOC
#define I2C3_SCL        0
#define I2C3_SDA        1
// I2C Alternate Function
#define I2C_AF          AF4

#endif // GPIO

#if 1 // =========================== I2C =======================================
// i2cclkPCLK1, i2cclkSYSCLK, i2cclkHSI
#define I2C_CLK_SRC     i2cclkHSI
#define I2C_BAUDRATE_HZ 400000
#endif

#if ADC_REQUIRED // ======================= Inner ADC ==========================
// Clock divider: clock is generated from the APB2
#define ADC_CLK_DIVIDER		adcDiv2

// ADC channels
#define ADC_BATTERY_CHNL 	14
// ADC_VREFINT_CHNL
#define ADC_CHANNELS        { ADC_BATTERY_CHNL, ADC_VREFINT_CHNL }
#define ADC_CHANNEL_CNT     2   // Do not use countof(AdcChannels) as preprocessor does not know what is countof => cannot check
#define ADC_SAMPLE_TIME     ast24d5Cycles
#define ADC_OVERSAMPLING_RATIO  64   // 1 (no oversampling), 2, 4, 8, 16, 32, 64, 128, 256
#endif

#if 1 // =========================== DMA =======================================
// ==== Uart ====
// Remap is made automatically if required
#define UART_DMA_TX     STM32_DMA_STREAM_ID(2, 6)
#define UART_DMA_RX     STM32_DMA_STREAM_ID(2, 7)
#define UART_DMA_CHNL   2
#define UART_DMA_TX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_LOW | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_TCIE)
#define UART_DMA_RX_MODE(Chnl) (STM32_DMA_CR_CHSEL(Chnl) | DMA_PRIORITY_MEDIUM | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_CIRC)

// ESP UARTs
#define UART2_DMA_TX     STM32_DMA_STREAM_ID(1, 7)
#define UART2_DMA_RX     STM32_DMA_STREAM_ID(1, 6)
#define UART3_DMA_TX     STM32_DMA_STREAM_ID(1, 2)
#define UART3_DMA_RX     STM32_DMA_STREAM_ID(1, 3)

// ==== SAI ====
#define SAI_DMA_A       STM32_DMA_STREAM_ID(2, 1)
#define SAI_DMA_CHNL    1

// ==== SDMMC ====
#define STM32_SDC_SDMMC1_DMA_STREAM   STM32_DMA_STREAM_ID(2, 5)


// ==== I2C ====
#define I2C1_DMA_TX     STM32_DMA_STREAM_ID(1, 6)
#define I2C1_DMA_RX     STM32_DMA_STREAM_ID(1, 7)
#define I2C1_DMA_CHNL   3
#define I2C2_DMA_TX     STM32_DMA_STREAM_ID(1, 4)
#define I2C2_DMA_RX     STM32_DMA_STREAM_ID(1, 5)
#define I2C2_DMA_CHNL   3
#define I2C3_DMA_TX     STM32_DMA_STREAM_ID(1, 2)
#define I2C3_DMA_RX     STM32_DMA_STREAM_ID(1, 3)
#define I2C3_DMA_CHNL   3

#if ADC_REQUIRED
#define ADC_DMA         STM32_DMA1_STREAM1
#define ADC_DMA_MODE    STM32_DMA_CR_CHSEL(0) |   /* DMA1 Stream1 Channel0 */ \
                        DMA_PRIORITY_LOW | \
                        STM32_DMA_CR_MSIZE_HWORD | \
                        STM32_DMA_CR_PSIZE_HWORD | \
                        STM32_DMA_CR_MINC |       /* Memory pointer increase */ \
                        STM32_DMA_CR_DIR_P2M |    /* Direction is peripheral to memory */ \
                        STM32_DMA_CR_TCIE         /* Enable Transmission Complete IRQ */
#endif // ADC

#endif // DMA

#if 1 // ========================== USART ======================================
#define PRINTF_FLOAT_EN TRUE
#define UART_TXBUF_SZ   4096
#define UART_RXBUF_SZ   256
#define CMD_BUF_SZ      256

#define CMD_UART        USART1
#define ESP_BOOT_UART   USART2
#define ESP_CMD_UART    USART3

#define CMD_UART_PARAMS \
    CMD_UART, UART_TX_PIN, UART_RX_PIN, \
    UART_DMA_TX, UART_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // Use independent clock

#define ESP_UART_PARAMS \
    ESP_CMD_UART, ESP_RX2_MCU_TX, ESP_TX2_MCU_RX, \
    UART3_DMA_TX, UART3_DMA_RX, UART_DMA_TX_MODE(UART_DMA_CHNL), UART_DMA_RX_MODE(UART_DMA_CHNL), \
    uartclkHSI // Use independent clock

#endif
