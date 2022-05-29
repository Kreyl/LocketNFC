/*
 * config.h
 *
 *  Created on: 15 мая 2022 г.
 *      Author: layst
 */

#pragma once

// ==== UART ====
#define UART_TX_PIN     (GPIO_NUM_17)
#define UART_RX_PIN     (GPIO_NUM_16)
#define UART_RXBUF_SZ   128
#define UART_TXBUF_SZ   512
#define UART_PERIPH     UART_NUM_1
#define UART_Q_SZ       18

// ==== I2S ====
#define I2S_BCK_PIN     (GPIO_NUM_26)
#define I2S_WS_PIN      (GPIO_NUM_25)
#define I2S_DATAIN_PIN  (GPIO_NUM_27)
