/*!
 * \file      encoder-board.c
 *
 * \brief     Target board Encoder driver implementation
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
 */
#include "stm32l1xx.h"
#include "stm32l1xx_hal_tim.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "encoder-board.h"
#include "math.h"
#include "lpm-board.h"
#include "stdlib.h"
// #include "timer.h"

// #define USE_GPIO

#define TAMPER_FLAG    0x1
#define ALARM_FLAG     0x2
#define DIR_FLAG       0x4

EncoderIrqHandler *EncoderIrq[] = { OnTamperingIrq , OnAlarmIrq, 
#if (USE_GPIO )
OnPulseDetected, 
#endif /* USE_GPIO */
NULL};

Encoder_t Encoder;
flow_t flow;
flow_t last_flow;
flow_config_t config;
#ifndef USE_GPIO
static TIM_HandleTypeDef TimHandle;
#endif
extern Gpio_t Led3;
static uint8_t EncoderInitialized = 0;
uint32_t uwDirection = 0;
int16_t round_msb = 0;
int32_t max_lsb = 160;
int32_t counter = 0, last_counter = 0;
int position = 0, speed = 0;
static int indx = 0;

/*!
 * Timer to handle the application data transmission duty cycle
 */
// static TimerEvent_t StorePacketTimer;
static void update_encoder(flow_t *enc, TIM_HandleTypeDef *htim);
static void update_encoder(flow_t *enc, TIM_HandleTypeDef *htim) {
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t first_time = 0;
	if (!first_time) {
		enc->rate = 0;
		first_time = 1;
	} else {
		if (temp_counter == enc->last) {
			enc->rate = 0;
		} else if (temp_counter > enc->last) {
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				enc->rate = -enc->last
						- (__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter);
                enc->status &= ~DIR_FLAG;
			} else {
				enc->rate = temp_counter - enc->last;
                enc->status |= DIR_FLAG;
			}
		} else {
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) {
				enc->rate = temp_counter - enc->last;
                enc->status &= ~DIR_FLAG;
			} else {
				enc->rate = temp_counter
						+ (__HAL_TIM_GET_AUTORELOAD(htim) - enc->last);
                enc->status |= DIR_FLAG;
			}
		}
	}
    
    if(abs(enc->rate) > 120) 
    {
        enc->rate = enc->last_rate;
    } 

    if (enc->rate < 0)
    {
        enc->rev_cnt += enc->rate;
    }
    counter = temp_counter;
	enc->fwd_cnt +=  enc->rate;
	enc->last = temp_counter;
    enc->last_rate = enc->rate;
	enc->position = (temp_counter) * 360.0f / max_lsb;
}

void EncoderInit(Encoder_t *obj, EncoderId_t timId, PinNames pulse, PinNames dir, PinNames tampering, PinNames alarm, PinNames en)
{
    CRITICAL_SECTION_BEGIN( );

    if( EncoderInitialized == 0 ){
        obj->EncoderId = timId;

        if( timId == TIM_2 )
        {
            GpioInit( &obj->Tampering, tampering, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
            GpioSetInterrupt( &obj->Tampering, IRQ_RISING_FALLING_EDGE, IRQ_MEDIUM_PRIORITY, EncoderIrq[0]);
            // GpioSetContext(&obj->Tampering, &obj);
            GpioInit( &obj->Alarm, alarm, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
            GpioSetInterrupt( &obj->Alarm, IRQ_FALLING_EDGE, IRQ_LOW_PRIORITY, EncoderIrq[1]);
            // GpioSetContext(&obj->Alarm, &obj);
            
            GpioInit( &obj->En, en, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );

    #ifdef USE_GPIO
            GpioInit( &obj->Pulse, pulse, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
            GpioSetInterrupt( &obj->Pulse, IRQ_FALLING_EDGE, IRQ_HIGH_PRIORITY, EncoderIrq[2]);
            GpioInit( &obj->Direction, dir, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    #else
            __HAL_RCC_TIM2_CLK_ENABLE();

            // TimHandle.Instance = ( TIM_TypeDef* )TIM2_BASE;
            
            GpioInit( &obj->Pulse, pulse, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF1_TIM2 );
            GpioInit( &obj->Direction, dir, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF1_TIM2 );

            TIM_Encoder_InitTypeDef sConfig = {0};
            TIM_MasterConfigTypeDef sMasterConfig = {0};

            TimHandle.Instance = TIM2;
            TimHandle.Init.Prescaler = 0;
            TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
            TimHandle.Init.Period = max_lsb - 1;
            TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
            sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
            sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
            sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
            sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
            sConfig.IC1Filter = 15;
            sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
            sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
            sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
            sConfig.IC2Filter = 15;
            HAL_TIM_Encoder_Init(&TimHandle, &sConfig);

            sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
            sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
            HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);

            HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
            HAL_NVIC_EnableIRQ(TIM2_IRQn);

            GpioWrite(&obj->En, GPIO_PIN_SET);
            HAL_Delay(500);
            //HAL_TIM_Encoder_Start(&TimHandle, TIM_CHANNEL_ALL);
    #endif
            Encoder.FlowData = &flow;
            Encoder.LastFlowData = &last_flow;
            Encoder.ConfigData = &config;
            // TimerInit( &StorePacketTimer, OnStorePacketTimerEvent );
            // TimerSetValue( &StorePacketTimer, 60000 );
            EncoderInitialized = 1;
        }
    } 
    else 
    {
        // if ((last_flow.fwd_cnt == flow.fwd_cnt) && (last_flow.rev_cnt == flow.rev_cnt))
        // {
        //     Encoder.ConfigData->isActiveMode = 0;   
        // } else {
        //     // if (Encoder.OnShowPulseDetect != NULL)
        //     // {
        //     //     Encoder.OnShowPulseDetect();
        //     // }

        //     // Encoder.ConfigData->isActiveMode = 1;
        //     // if (Encoder.OnSendOneshot != NULL)
        //     // {
        //     //     Encoder.OnSendOneshot();
        //     // }

        // }
    }
    CRITICAL_SECTION_END( );
}

void EncoderDeInit( Encoder_t *obj )
{
#ifndef USE_GPIO
    // HAL_TIM_Encoder_DeInit(&TimHandle);
#endif
}

void OnTamperingIrq( void* context )
{
    CRITICAL_SECTION_BEGIN();
    // Encoder_t *obj = (Encoder_t *) context;
#if 1
    if (GpioRead(&Encoder.Tampering) == GPIO_PIN_RESET) 
#else
    if (GpioRead(&Encoder.En) == GPIO_PIN_SET) 
#endif /* USE_IREN01A0 */
    {
        	flow.status |= TAMPER_FLAG;
            printf( "\r\n###### ===== Tampering attached ==== ######\r\n\r\n" );
#ifndef USE_GPIO
		    HAL_TIM_Encoder_Start_IT(&TimHandle, TIM_CHANNEL_ALL);
            LpmSetStopMode(LPM_APPLI_ID, LPM_DISABLE);
            if (Encoder.OnShowPulseDetect != NULL)
            {
                Encoder.OnShowPulseDetect();
            }
#endif            
	} else {
            flow.status &= ~TAMPER_FLAG;
		    printf( "\r\n###### ===== Tampering released ==== ######\r\n\r\n" );
#ifndef USE_GPIO
            HAL_TIM_Encoder_Stop_IT(&TimHandle, TIM_CHANNEL_ALL);
            LpmSetStopMode(LPM_APPLI_ID, LPM_ENABLE);
             if (Encoder.OnArcRelease != NULL)
            {
                Encoder.OnArcRelease();
            }
#endif
        		
    }	
    
    if (Encoder.OnSendOneshot != NULL)
    {
        Encoder.OnSendOneshot(  );  
    }
    
    CRITICAL_SECTION_END();
}

void OnAlarmIrq( void* context )
{
    CRITICAL_SECTION_BEGIN();
    // Encoder_t *obj = (Encoder_t *) context;
    if (GpioRead(&Encoder.Alarm) == GPIO_PIN_RESET) {
            flow.status |= ALARM_FLAG;
            printf( "\r\n###### ===== Alarm ==== ######\r\n\r\n" );
	} else {
            flow.status &= ~ALARM_FLAG;
            printf( "\r\n###### ===== Silent ==== ######\r\n\r\n" );
	}
    
    if (Encoder.OnSendOneshot != NULL && config.digital_alarm > 0) 
    {
        Encoder.OnSendOneshot(  );
    }

    CRITICAL_SECTION_END();  
}

#ifdef USE_GPIO

void OnPulseDetected( void* context )
{
    CRITICAL_SECTION_BEGIN();

    if (GpioRead(&Encoder.Direction) != GPIO_PIN_RESET)
    {
        flow.status |= DIR_FLAG;
        flow.fwd_cnt++;
        printf( "\r\nForward cnt : %ld\r\n\r\n",  flow.fwd_cnt);
    } else {
        flow.status &= ~DIR_FLAG;
        flow.rev_cnt++;
        printf( "\r\nBackward cnt : %ld\r\n\r\n",  flow.rev_cnt);
    }
    flow.rate++;

    if (Encoder.OnShowPulseDetect != NULL)
    {
        Encoder.OnShowPulseDetect();
    }

    if(Encoder.ConfigData->isActiveMode == 0) {
        Encoder.ConfigData->isActiveMode = 1;
        if (Encoder.OnSendOneshot != NULL)
        {
            Encoder.OnSendOneshot(  );  
        }
    }  
    
    CRITICAL_SECTION_END();
}

#else
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	CRITICAL_SECTION_BEGIN();
	if (htim->Instance == TIM2) {
        // uint32_t current = 0;
// #if ( USE_GPIO )        
//         if (GpioRead(&Encoder.Direction) != GPIO_PIN_RESET)
// #else
//         // uwDirection = __HAL_TIM_DIRECTION_STATUS(&TimHandle);
//         // counter = (int32_t) __HAL_TIM_GET_COUNTER(htim);
//         // if (uwDirection != GPIO_PIN_RESET)
// #endif /* USE_GPIO */
// 		{
//             flow.status |= DIR_FLAG;
//             // printf( "\r\nForward cnt : %ld\r\n\r\n",  flow.fwd_cnt);
//             if (counter == 0 && last_counter == max_lsb ) {
// 				++round_msb;
// 			}
// 		} else {
//             flow.status &= ~DIR_FLAG;
//             // printf( "\r\nBackward cnt : %ld\r\n\r\n",  flow.rev_cnt);
//             if (counter == max_lsb && last_counter == 0) {
// 				--round_msb;
// 			}
// 		}
//         // flow.fwd_cnt = (round_msb << 6 | counter);
// 		// last_counter = counter;
//         // flow.position = counter / max_lsb;
//         // current = HAL_GetTick(); //HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//         // float delta = (float) abs(current - flow.last);
// 		// flow.rate = (uint32_t) ((1.0f / delta) * 1000.0f);
// 		// flow.last = HAL_GetTick();
//         // __NOP();
	}
    
    if (Encoder.OnShowPulseDetect != NULL)
    {
        Encoder.OnShowPulseDetect();
    }

    // if(Encoder.ConfigData->isActiveMode == 0) {
    //     Encoder.ConfigData->isActiveMode = 1;
    //     if (Encoder.OnSendOneshot != NULL)
    //     {
    //         Encoder.OnSendOneshot(  );  
    //     }
    // }
    CRITICAL_SECTION_END();
}

void OnPeriodElapsedCallback( void ) {
    /* code */
    // indx++;
    // if (indx == 100)
    // {
        // uwDirection = __HAL_TIM_DIRECTION_STATUS(&TimHandle);
        // if (uwDirection != GPIO_PIN_RESET)
		// {
        //     flow.status |= DIR_FLAG;
         
		// } else {
        //     flow.status &= ~DIR_FLAG;
         
		// }
        update_encoder(&flow, &TimHandle);
        
        position = flow.position;
        speed = flow.rate;
    //     indx = 0;
    // }
}

void TIM2_IRQHandler( void )
{
    HAL_TIM_IRQHandler( &TimHandle );
}

#endif /* USE_GPIO */

/*!
 * \brief Function executed on Led 4 Timeout event
 */
// static void OnStorePacketTimerEvent( void* context )
// {
//     // TimerStop( &StorePacketTimer );
//     // NvmCtxMgmtStore();
// }
void EncoderUpdateStatus ( void ) 
{
    OnTamperingIrq( NULL );
    OnAlarmIrq( NULL );
}
