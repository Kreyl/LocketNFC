/*
 * bt.h
 *
 *  Created on: 15 ��� 2022 �.
 *      Author: layst
 */

#pragma once

void BTInit();

uint8_t BTGetState();

void BTStartDiscovery();
void BTStopDiscovery();

void BTConnect(uint8_t *pAddr);
void BTDisconnect();
