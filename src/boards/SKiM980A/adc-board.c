/*!
 * \file      adc-board.c
 *
 * \brief     Target board ADC driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "stm32l1xx.h"
#include "board-config.h"
#include "adc-board.h"

ADC_HandleTypeDef AdcHandle;

/*!
 * Function executed on IO2 event
 */
extern void OnIO2Event(void *context);

void AdcMcuInit( Adc_t *obj, PinNames adcInput )
{
    AdcHandle.Instance = ( ADC_TypeDef* )ADC1_BASE;

    __HAL_RCC_ADC1_CLK_ENABLE( );

    HAL_ADC_DeInit( &AdcHandle );

    if( adcInput != NC )
    {
        GpioInit( &obj->AdcInput, adcInput, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        HAL_NVIC_SetPriority( ADC1_IRQn , 0, 0 );
        HAL_NVIC_EnableIRQ( ADC1_IRQn );
    }
}

void AdcMcuConfig( void )
{
    ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
    ADC_ChannelConfTypeDef adcConf = { 0 };
    
    // Configure ADC
    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    AdcHandle.Init.ContinuousConvMode    = DISABLE;
    AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    AdcHandle.Init.DMAContinuousRequests = DISABLE;
    AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    AdcHandle.Init.NbrOfConversion       = 1;
    AdcHandle.Init.LowPowerAutoWait      = DISABLE;
    AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
    AdcHandle.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
    AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    // AdcHandle.Init.ScanConvMode = ADC_SCAN_DISABLE;

    if (HAL_ADC_Init( &AdcHandle )!= HAL_OK)
    {
        
    }

    // /** Configure the analog watchdog
    //  */
    // AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
    // AnalogWDGConfig.Channel = ADC_CHANNEL_1;
    // AnalogWDGConfig.ITMode = ENABLE;
    // AnalogWDGConfig.HighThreshold = 0x800;
    // AnalogWDGConfig.LowThreshold = 0x200;
    // if (HAL_ADC_AnalogWDGConfig(&AdcHandle, &AnalogWDGConfig) != HAL_OK)
    // {
        
    // }
    
    // adcConf.Channel = ADC_CHANNEL_1;
    // adcConf.Rank = ADC_REGULAR_RANK_1;
    // adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;
    // if (HAL_ADC_ConfigChannel( &AdcHandle, &adcConf )!= HAL_OK)
    // {
        
    // }



}

uint16_t AdcMcuReadChannel( Adc_t *obj, uint32_t channel )
{
    ADC_ChannelConfTypeDef adcConf = { 0 };
    uint16_t adcData = 0;

    // Enable HSI
    __HAL_RCC_HSI_ENABLE( );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    __HAL_RCC_ADC1_CLK_ENABLE( );

    adcConf.Channel = channel;
    adcConf.Rank = ADC_REGULAR_RANK_1;
    adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;

    HAL_ADC_ConfigChannel( &AdcHandle, &adcConf );

    // Enable ADC1
    if( ADC_Enable( &AdcHandle ) == HAL_OK )
    {
        // Start ADC Software Conversion
        HAL_ADC_Start( &AdcHandle );

        HAL_ADC_PollForConversion( &AdcHandle, HAL_MAX_DELAY );

        adcData = HAL_ADC_GetValue( &AdcHandle );
    }

    ADC_ConversionStop_Disable( &AdcHandle );

    if( ( adcConf.Channel == ADC_CHANNEL_TEMPSENSOR ) || ( adcConf.Channel == ADC_CHANNEL_VREFINT ) )
    {
        HAL_ADC_DeInit( &AdcHandle );
    }
    __HAL_RCC_ADC1_CLK_DISABLE( );

    // Disable HSI
    __HAL_RCC_HSI_DISABLE( );

    return adcData;
}

/**
  * @brief This function handles ADC global interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&AdcHandle);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  OnIO2Event(NULL);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_LevelOutOfWindowCallback must be implemented in the user file.
  */
}
