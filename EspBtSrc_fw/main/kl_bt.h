/*
 * bt.h
 *
 *  Created on: 15 мая 2022 г.
 *      Author: layst
 */

#pragma once

void BTInit();

uint8_t BTGetState();

void BTStartDiscovery();
void BTStopDiscovery();

void BTConnect(uint8_t *pAddr);
void BTDisconnect();

int32_t BTOnDataRequest(uint8_t *PData, int32_t ALen);
