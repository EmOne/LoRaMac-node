/*!
 * \file      encoder-board.h
 *
 * \brief     Target board encoder driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______          _______
 *               / _____)        /       \
 *              ( (____   ____   |   _   | __  ___   _____
 *               |____ | |     | |  (_)  | | |/_  | (  ___)
 *              ( (____  | | | | |       | |  / | | (  ___)
 *              (______) |_|_| | \_______/ |_|  | | (_____)
 *              (C)2015-2019 EmOne
 *
 * \endcode
 *
 * \author    Anol Paisal ( EmOne )
 *
 */
#ifndef __ENCODER_BOARD_H__
#define __ENCODER_BOARD_H__

#include "encoder.h"

typedef enum {
    RATE_01_MIN = 0,
    RATE_15_MIN,
    RATE_30_MIN,
    RATE_60_MIN,
    RATE_90_MIN
    /* 5 to 15 reserved for default RATE_01_MIN */
} DUTYCYCLE_RATE ;

// An encoder.c file has to be implmented under system directory.
void OnTamperingIrq( void* context );
void OnAlarmIrq( void* context );
#ifdef USE_GPIO
void OnPulseDetected( void* context );
#endif /* USE_GPIO */
#endif // __ENCODER_BOARD_H__
