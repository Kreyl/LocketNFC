/*
 * EvtQMain.h
 *
 *  Created on: 16 мая 2022 г.
 *      Author: layst
 */

#pragma once

QueueHandle_t MainQ;

typedef union {
    uint32_t DWord[2];
    struct {
        union {
            void* Ptr;
            struct {
                int32_t Value;
                uint8_t ValueID;
            } __attribute__((__packed__));
        } __attribute__((__packed__));
        uint8_t ID;
    } __attribute__((__packed__));
} __attribute__((__packed__)) EvtMsg_t;
