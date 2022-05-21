/*
 * uart.h
 *
 *  Created on: 15 мая 2022 г.
 *      Author: layst
 */

#pragma once

#include "RetVal.h"

void UartInit();

void Printf(const char *format, ...);

bool CmdNameIs(const char *SCmd);
char* CmdGetName();
uint8_t CmdGetNextString(char **PStr);
uint8_t CmdGetNextUint32(uint32_t *ptr);
uint8_t CmdGetNextInt32(int32_t *ptr);
uint8_t CmdGetArrUint8(uint8_t *pArr, uint32_t ALen);


void CmdReplyOk();
void CmdReplyBadParam();

