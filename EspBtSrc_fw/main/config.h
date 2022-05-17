/*
 * config.h
 *
 *  Created on: 15 мая 2022 г.
 *      Author: layst
 */

#pragma once

// ==== System ====
#define MAIN_Q_LEN      18

// ==== UART ====
#define UART_TX_PIN     (GPIO_NUM_14)
#define UART_RX_PIN     (GPIO_NUM_27)
#define UART_RXBUF_SZ   128
#define UART_TXBUF_SZ   256
#define UART_PERIPH     UART_NUM_1
#define UART_Q_SZ       18
