/* USER CODE BEGIN Header */
/*
 ******************************************************************************
  * File Name          : App/tsl_user.h.h
  * Description        : Touch-Sensing user configuration.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TSL_USER_H
#define __TSL_USER_H

#include "tsl.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#include "stm32l1xx_hal.h"

/* Select if you use STMStudio software (0=No, 1=Yes) */
#define USE_STMSTUDIO (0)

#if USE_STMSTUDIO > 0
#include "stmCriticalSection.h"
#define STMSTUDIO_LOCK {enterLock();}
#define STMSTUDIO_UNLOCK {exitLock();}
#else
#define STMSTUDIO_LOCK
#define STMSTUDIO_UNLOCK
#endif

typedef enum
{
  TSL_USER_STATUS_BUSY       = 0, /**< The bank acquisition is on-going */
  TSL_USER_STATUS_OK_NO_ECS  = 1, /**< The bank acquisition is ok, no time for ECS */
  TSL_USER_STATUS_OK_ECS_ON  = 2, /**< The bank acquisition is ok, ECS finished */
  TSL_USER_STATUS_OK_ECS_OFF = 3  /**< The bank acquisition is ok, ECS not executed */
} tsl_user_status_t;

/* Channel IOs definition */
#define CHANNEL_0_SRC       ((uint32_t) (GR7))
#define CHANNEL_0_DEST      (0)
#define CHANNEL_0_SAMPLE    (TSL_GROUP7_IO1)
#define CHANNEL_0_CHANNEL   (TSL_GROUP7_IO2)
#define CHANNEL_1_SRC       ((uint32_t) (GR7))
#define CHANNEL_1_DEST      (1)
#define CHANNEL_1_SAMPLE    (TSL_GROUP7_IO1)
#define CHANNEL_1_CHANNEL   (TSL_GROUP7_IO3)
#define CHANNEL_2_SRC       ((uint32_t) (GR7))
#define CHANNEL_2_DEST      (2)
#define CHANNEL_2_SAMPLE    (TSL_GROUP7_IO1)
#define CHANNEL_2_CHANNEL   (TSL_GROUP7_IO4)
#define CHANNEL_3_SRC       ((uint32_t) (GR1))
#define CHANNEL_3_DEST      (3)
#define CHANNEL_3_SAMPLE    (TSL_GROUP1_IO1)
#define CHANNEL_3_CHANNEL   (TSL_GROUP1_IO2)
#define CHANNEL_4_SRC       ((uint32_t) (GR1))
#define CHANNEL_4_DEST      (4)
#define CHANNEL_4_SAMPLE    (TSL_GROUP1_IO1)
#define CHANNEL_4_CHANNEL   (TSL_GROUP1_IO4)
/* Bank(s) definition */
/* TOUCHKEYS bank(s) definition*/
#define BANK_0_NBCHANNELS (1)
#define BANK_1_NBCHANNELS (1)
#define BANK_2_NBCHANNELS (2)
#define BANK_3_NBCHANNELS (1)

/* User Parameters */
extern CONST TSL_Bank_T MyBanks[];
extern CONST TSL_TouchKey_T MyTKeys[];
extern CONST TSL_Object_T MyObjects[];
extern TSL_ObjectGroup_T MyObjGroup;

void tsl_user_Init(void);
tsl_user_status_t tsl_user_Exec(void);
void tsl_user_SetThresholds(void);

#endif /* __TSL_USER_H */
