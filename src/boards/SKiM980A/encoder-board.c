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
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "encoder-board.h"
#include "math.h"
// #include "timer.h"

#define TAMPER_FLAG    0x1
#define ALARM_FLAG     0x2
#define DIR_FLAG       0x4

EncoderIrqHandler *EncoderIrq[] = { OnTamperingIrq , OnAlarmIrq, NULL};

Encoder_t Encoder;
flow_t flow;
flow_t last_flow;
flow_config_t config;
static TIM_HandleTypeDef TimHandle;
extern Gpio_t Led3;

/*!
 * Timer to handle the application data transmission duty cycle
 */
// static TimerEvent_t StorePacketTimer;

void EncoderInit( Encoder_t *obj, EncoderId_t timId, PinNames pulse, PinNames dir, PinNames tampering, PinNames alarm)
{
    CRITICAL_SECTION_BEGIN( );

     obj->EncoderId = timId;

     if( timId == TIM_2 )
     {
        __HAL_RCC_TIM2_CLK_ENABLE();

        TimHandle.Instance = ( TIM_TypeDef* )TIM2_BASE;
        
        GpioInit( &obj->Pulse, pulse, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF1_TIM2 );
        GpioInit( &obj->Direction, dir, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, GPIO_AF1_TIM2 );

        TIM_Encoder_InitTypeDef sConfig = {0};
        TIM_MasterConfigTypeDef sMasterConfig = {0};

        TimHandle.Instance = TIM2;
        TimHandle.Init.Prescaler = 1;
        TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
        TimHandle.Init.Period = 0xffff;
        TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        sConfig.IC1Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
        sConfig.IC1Filter = 0;
        // sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
        // sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        // sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
        // sConfig.IC2Filter = 0;
        HAL_TIM_Encoder_Init(&TimHandle, &sConfig);

        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);

        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);

        GpioInit( &obj->Tampering, tampering, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
        GpioSetInterrupt( &obj->Tampering, IRQ_RISING_FALLING_EDGE, IRQ_VERY_HIGH_PRIORITY, EncoderIrq[0]);
        // GpioSetContext(&obj->Tampering, &obj);
        GpioInit( &obj->Alarm, alarm, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
        GpioSetInterrupt( &obj->Alarm, IRQ_FALLING_EDGE, IRQ_VERY_HIGH_PRIORITY, EncoderIrq[1]);
        // GpioSetContext(&obj->Alarm, &obj);

        Encoder.FlowData = &flow;
        Encoder.LastFlowData = &last_flow;
        Encoder.ConfigData = &config;
        // TimerInit( &StorePacketTimer, OnStorePacketTimerEvent );
        // TimerSetValue( &StorePacketTimer, 60000 );
        HAL_TIM_Encoder_Start_IT(&TimHandle, TIM_CHANNEL_1);
    }

    CRITICAL_SECTION_END( );
}

void EncoderDeInit( Encoder_t *obj )
{
    HAL_TIM_Encoder_DeInit(&TimHandle);
}

void OnTamperingIrq( void* context )
{
    CRITICAL_SECTION_BEGIN();
    // Encoder_t *obj = (Encoder_t *) context;
    if (GpioRead(&Encoder.Tampering) == GPIO_PIN_RESET) {
        if (!(flow.status && TAMPER_FLAG)) {
        	flow.status |= TAMPER_FLAG;
            printf( "\r\n###### ===== Tampering attached ==== ######\r\n\r\n" );
		    // HAL_TIM_Encoder_Start_IT(&TimHandle, TIM_CHANNEL_1);
        }
	} else {
        if ((flow.status && TAMPER_FLAG))
        {
            flow.status &= ~TAMPER_FLAG;
		    printf( "\r\n###### ===== Tampering released ==== ######\r\n\r\n" );
            // HAL_TIM_Encoder_Stop_IT(&TimHandle, TIM_CHANNEL_1);
        }		
    }	
    CRITICAL_SECTION_END();
    if (Encoder.OnSendOneshot != NULL)
    {
        Encoder.OnSendOneshot(  );  
    }
}

void OnAlarmIrq( void* context )
{
    CRITICAL_SECTION_BEGIN();
    // Encoder_t *obj = (Encoder_t *) context;
    if (GpioRead(&Encoder.Alarm) == GPIO_PIN_RESET) {
        if (!(flow.status && ALARM_FLAG))
        {
            flow.status |= ALARM_FLAG;
            printf( "\r\n###### ===== Alarm ==== ######\r\n\r\n" );
        }
	} else {
        if ((flow.status && ALARM_FLAG)) 
        {
            flow.status &= ~ALARM_FLAG;
            printf( "\r\n###### ===== Silent ==== ######\r\n\r\n" );
        }
	}
    CRITICAL_SECTION_END();  
    if (Encoder.OnSendOneshot != NULL && config.digital_alarm > 0) 
    {
        Encoder.OnSendOneshot(  );
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	CRITICAL_SECTION_BEGIN();
	if (htim->Instance == TIM2) {
        // uint32_t current = 0;
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
        // current = HAL_GetTick(); //HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        // float delta = (float) abs(current - flow.last);
		// flow.rate = (uint32_t) ((1.0f / delta) * 1000.0f);
		// flow.last = HAL_GetTick();
        // __NOP();
	}
    

	CRITICAL_SECTION_END();

    if (Encoder.OnPulseDetect != NULL)
    {
        Encoder.OnPulseDetect();
    }

    if(Encoder.ConfigData->isActiveMode == 0) {
        Encoder.ConfigData->isActiveMode = 1;
        if (Encoder.OnSendOneshot != NULL)
        {
            Encoder.OnSendOneshot(  );  
        }
    }

}

void TIM2_IRQHandler( void )
{
    HAL_TIM_IRQHandler( &TimHandle );
}

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
