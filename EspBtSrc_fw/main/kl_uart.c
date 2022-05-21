/*
 * uart.c
 *
 *  Created on: 15 мая 2022 г.
 *      Author: layst
 */

#include "config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "kl_uart.h"

#include <string.h>

#define DELIMITERS      " ,\r\n"

static const char *TAG = "uart_events";
static QueueHandle_t UartQ;

extern void OnCmd();

static char CmdBuf[UART_RXBUF_SZ];
static int CmdBufSz = 0;
static char *S = CmdBuf, *Token = NULL;
static char *CmdName = 0;

static void SkipLeadingWhitespaces() {
    while(*S != '\0' && *S <= ' ') S++;
}

void ParseCmd() {
//    ESP_LOGI(TAG, "Cmd: %s, Sz: %d", CmdBuf, CmdBufSz);
    S = CmdBuf;
    SkipLeadingWhitespaces();
    CmdName = strtok_r(S, DELIMITERS, &S);
    if(CmdName && *CmdName != '\0') OnCmd();
}

#if 1 // ========= CMD =========
bool CmdNameIs(const char *SCmd) { return (strcasecmp(CmdName, SCmd) == 0); }

char* CmdGetName() { return CmdName; }

uint8_t CmdGetNextString(char **PStr) {
    Token = strtok_r(S, DELIMITERS, &S);
    if(PStr != NULL) *PStr = Token;
    return (Token != NULL &&  *Token != '\0')? retvOk : retvEmpty;
}

static uint8_t GetNextString() {
    Token = strtok_r(S, DELIMITERS, &S);
    return (Token != NULL &&  *Token != '\0')? retvOk : retvEmpty;
}

uint8_t CmdGetNextUint32(uint32_t *ptr) {
    if(GetNextString() != retvOk) return retvEmpty;
    char *p;
    *ptr = strtoul(Token, &p, 0);
    return (*p == '\0') ? retvOk : retvNotANumber;
}

uint8_t CmdGetNextInt32(int32_t *ptr) {
    if(GetNextString() != retvOk) return retvEmpty;
    char *p;
    *ptr = strtol(Token, &p, 0);
    return (*p == '\0') ? retvOk : retvNotANumber;
}

uint8_t CmdGetArrUint8(uint8_t *pArr, uint32_t ALen) {
    if(GetNextString() != retvOk) return retvEmpty;
    char ByteStrBuf[5] = "0x00";
    char *ArrStr = Token;
    for(uint32_t i=0; i<ALen; i++) {
        ByteStrBuf[2] = *ArrStr++;
        ByteStrBuf[3] = *ArrStr++;
        ArrStr++; // skip delimiter
        char *p;
        *pArr++ = strtoul(ByteStrBuf, &p, 0);
        if(*p != '\0') return retvFail;
    }
    return retvOk;
}


void CmdReplyOk() { Printf("Ok\r\n"); }
void CmdReplyBadParam() { Printf("BadParam\r\n"); }
#endif

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    size_t buffered_size;
    while(true) {
        if(xQueueReceive(UartQ, (void*)&event, (portTickType)portMAX_DELAY)) {
//            ESP_LOGI(TAG, "uart[%d] event:", UART_PERIPH);
            switch(event.type) {
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "! UART fifo ovf");
                    uart_flush_input(UART_PERIPH);
                    xQueueReset(UartQ);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "! UART buffer full");
                    uart_flush_input(UART_PERIPH);
                    xQueueReset(UartQ);
                    break;

                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_PERIPH, &buffered_size);
                    CmdBufSz = uart_pattern_pop_pos(UART_PERIPH);
//                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", CmdBufSz, buffered_size);
                    // Flush q if it is full
                    if (CmdBufSz == -1) uart_flush_input(UART_PERIPH);
                    else {
                        // Read payload
                        uart_read_bytes(UART_PERIPH, CmdBuf, CmdBufSz, 100 / portTICK_PERIOD_MS);
                        CmdBuf[CmdBufSz] = '\0'; // Add end of string
                        ParseCmd();
                    }
                    break;

                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            } // switch
        }
    } // while 1
    vTaskDelete(NULL);
}


void UartInit() {
    const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0, // dummy
            .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_PERIPH, UART_RXBUF_SZ * 2, UART_TXBUF_SZ, UART_Q_SZ, &UartQ, 0);
    uart_param_config(UART_PERIPH, &uart_config);
    uart_set_pin(UART_PERIPH, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(UART_PERIPH, '\n', 1, 9, 0, 0); // rise evt on single \n
    // Reset the pattern queue length to record at most Q_SZ pattern positions.
    uart_pattern_queue_reset(UART_PERIPH, UART_Q_SZ);

    xTaskCreate(uart_event_task, "UartEvtTask", 2048, NULL, 12, NULL);
//    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
}


static char TxBuf[UART_TXBUF_SZ];
static uint32_t TxIndx = 0;
static uint8_t IPutChar(char c) {
    if(TxIndx < UART_TXBUF_SZ) {
        TxBuf[TxIndx++] = c;
        return retvOk;
    }
    else return retvFail;
}

static void IStartTransmissionIfNotYet() {
    uart_tx_chars(UART_PERIPH, TxBuf, TxIndx);
    TxIndx = 0;
}

static uint8_t IPutUint(uint32_t n, uint32_t base, uint32_t width, char filler) {
    char digits[10];
    uint32_t len = 0;
    // Place digits to buffer
    do {
        uint32_t digit = n % base;
        n /= base;
        digits[len++] = (digit < 10)? '0'+digit : 'A'+digit-10;
    } while(n > 0);
    // Add padding
    for(uint32_t i = len; i < width; i++) {
        if(IPutChar(filler) != retvOk) return retvOverflow;
    }
    // Print digits
    while(len > 0) {
        if(IPutChar(digits[--len]) != retvOk) return retvOverflow;
    }
    return retvOk;
} // IPutUint

static void IVsPrintf(const char *format, va_list args) {
    const char *fmt = format;
    uint32_t width = 0, precision;
    char c, filler;
    while(true) {
        c = *fmt++;
        if(c == 0) goto End;
        if(c != '%') {  // Not %
            if(IPutChar(c) != retvOk) goto End;
            else continue;
        }

        // Here goes optional width specification.
        // If it starts with zero (zero_padded is true), it means we use '0' instead of ' ' as a filler.
        filler = ' ';
        if(*fmt == '0') {
            fmt++;
            filler = '0';
        }

        width = 0;
        while(true) {
            c = *fmt++;
            if(c >= '0' && c <= '9') c -= '0';
            else if (c == '*') c = va_arg(args, int);
            else break;
            width = width * 10 + c;
        }

        precision = 0;
        if(c == '.') {
            while(true) {
                c = *fmt++;
                if(c >= '0' && c <= '9') c -= '0';
                else if(c == '*') c = va_arg(args, int);
                else break;
                precision = precision * 10 + c;
            }
        }

        // Command decoding
        switch(c) {
            case 'c':
                if(IPutChar(va_arg(args, int)) != retvOk) goto End;
                break;

            case 's':
            case 'S': {
                char *s = va_arg(args, char*);
                while(*s != 0) {
                    if(IPutChar(*s++) != retvOk) goto End;
                }
            }
            break;

            case 'x':
            case 'X':
                if(IPutUint(va_arg(args, uint32_t), 16, width, filler) != retvOk) goto End;
                break;
            case 'u':
                if(IPutUint(va_arg(args, uint32_t), 10, width, filler) != retvOk) goto End;
                break;

            case 'd':
            case 'i':
            {
                int32_t n = va_arg(args, int32_t);
                if(n < 0) {
                    if(IPutChar('-') != retvOk) goto End;
                    n = -n;
                }
                if(IPutUint(n, 10, width, filler) != retvOk) goto End;
            }
            break;

#if PRINTF_FLOAT_EN
            case 'f': {
                float f = (float)va_arg(args, double);
                if (f < 0) {
                    if(IPutChar('-') != retvOk) goto End;
                    f = -f;
                }
                int32_t n;
                if((precision == 0) || (precision > FLOAT_PRECISION)) precision = FLOAT_PRECISION;
                n = (int32_t)f;
                if(IPutUint(n, 10, width, filler) != retvOk) goto End;
                if(IPutChar('.') != retvOk) goto End;
                filler = '0';
                width = precision;
                n = (long)((f - n) * power10Table[precision - 1]);
                if(IPutUint(n, 10, width, filler) != retvOk) goto End;
            } break;
#endif

            case 'A': {
                uint8_t *arr = va_arg(args, uint8_t*);
                int32_t n = va_arg(args, int32_t);
                int32_t Delimiter = va_arg(args, int32_t);
                filler = '0';       // }
                width = 2;          // } 01 02 0A etc.; not 1 2 A
                for(int32_t i = 0; i < n; i++) {
                    if((i > 0) && (Delimiter != 0)) { // do not place delimiter before or after array
                        if(IPutChar((char)Delimiter) != retvOk) goto End;
                    }
                    if(IPutUint(arr[i], 16, width, filler) != retvOk) goto End;
                }
            } break;

            case '%':
                if(IPutChar('%') != retvOk) goto End;
                break;
        } // switch
    } // while
    End:
    IStartTransmissionIfNotYet();
}


void Printf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    IVsPrintf(format, args);
    va_end(args);
}
