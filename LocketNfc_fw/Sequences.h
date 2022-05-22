/*
 * Sequences.h
 *
 *  Created on: 09 ���. 2015 �.
 *      Author: Kreyl
 */

#pragma once

#include "ChunkTypes.h"

const LedRGBChunk_t lsqStart[] = {
        {csSetup, 0, clRed},
        {csWait, 207},
        {csSetup, 0, clGreen},
        {csWait, 207},
        {csSetup, 0, clBlue},
        {csWait, 207},
//        {csSetup, 0, clBlack},
        {csSetup, 0, {0,4,0}},
        {csEnd},
};

const LedRGBChunk_t lsqFail[] = {
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csRepeat, 3},
        {csSetup, 0, {4,0,0}},
        {csEnd},
};
