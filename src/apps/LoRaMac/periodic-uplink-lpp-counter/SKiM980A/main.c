/*!
 * \file      main.c
 *
 * \brief     Performs a periodic uplink
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
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 */

/*! \file periodic-uplink/SKiM980A/main.c */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../firmwareVersion.h"
#include "../../common/githubVersion.h"
#include "utilities.h"
#include "board.h"
#include "lpm-board.h"
#include "gpio.h"
#include "uart.h"
#include "RegionCommon.h"
#include "eeprom-board.h"
#include "nvmm.h"  

#include "cli.h"
#include "Commissioning.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "CayenneLpp.h"
#include "LmHandlerMsgDisplay.h"

static uint32_t         in_cnt          = 0U;
static uint32_t         in_out_cnt      = 0U;
const volatile uint32_t stop_mode_timer = 8000U; /* Test at 8 sec */
static uint8_t uplink_buffer[9U]    = { 0 };
static uint8_t test_read_buffer[8U] = { 0 };

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            1000 * 60 * 5

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are reserved.
 */
#define LORAWAN_APP_PORT                            3

/*!
 *
 */
typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
}LmHandlerTxEvents_t;

/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData =
{
    .Buffer = AppDataBuffer,
    .BufferSize = 0,
    .Port = 0,
};

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

/*!
 * Timer to handle the state of LED4
 */
static TimerEvent_t Led4Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED beacon indicator
 */
static TimerEvent_t LedBeaconTimer;

static void OnMacProcessNotify( void );
static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size );
static void OnNetworkParametersChange( CommissioningParams_t* params );
static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn );
static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn );
static void OnJoinRequest( LmHandlerJoinParams_t* params );
static void OnTxData( LmHandlerTxParams_t* params );
static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params );
static void OnClassChange( DeviceClass_t deviceClass );
static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params );
#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection );
#else
static void OnSysTimeUpdate( void );
#endif
static void PrepareTxFrame( void );
static void StartTxProcess( LmHandlerTxEvents_t txEvent );
static void UplinkProcess( void );

static void OnTxPeriodicityChanged( uint32_t periodicity );
static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed );
static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity );

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context );

/*!
 * Function executed on Led 4 Timeout event
 */
static void OnLed4TimerEvent( void* context );

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void* context );

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent( void* context );

// /*!
//  * Timer to handle the state of WKUP indicator
//  */
// static TimerEvent_t WKUPTimer;

/*!
 * Function executed on Wakeup event
 */
void OnWKUPEvent(void *context);

/*!
 * Function executed on IO1 event
 */
void OnIO1Event(void *context);

/*!
 * Function executed on IO2 event
 */
void OnIO2Event(void *context);

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = NULL,
    .GetRandomSeed = BoardGetRandomSeed,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams =
{
    .Region = ACTIVE_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = AppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = FIRMWARE_VERSION,
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

/*!
 * Indicates if LoRaMacProcess call is pending.
 * 
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

static volatile uint8_t IsStopMode = 0;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led4; // Tx
// extern Gpio_t Led2; // Rx and blinks every 5 seconds when beacon is acquired
// extern Gpio_t Led3; // App
extern Gpio_t Wkup;
extern Gpio_t Io1;
extern Gpio_t Io2;
/*!
 * UART object used for command line interface handling
 */
extern Uart_t Uart1;

/*!
 * Main application entry point.
 */
int main( void )
{
    // uint16_t size = 0U;
    // uint16_t i    = 0U;

    BoardInitMcu( );
    BoardInitPeriph( );

    NvmmRead(&test_read_buffer[0], 8U, 0x0FF8);
    in_cnt |= test_read_buffer[0U] << 24;
    in_cnt |= test_read_buffer[1U] << 16;
    in_cnt |= test_read_buffer[2U] << 8;
    in_cnt |= test_read_buffer[3U] << 0;
    in_out_cnt |= test_read_buffer[4U] << 24;
    in_out_cnt |= test_read_buffer[5U] << 16;
    in_out_cnt |= test_read_buffer[6U] << 8;
    in_out_cnt |= test_read_buffer[7U] << 0;
    
    TimerInit( &Led4Timer, OnLed4TimerEvent );
    TimerSetValue( &Led4Timer, 25 );

    TimerInit( &Led2Timer, OnLed2TimerEvent );
    TimerSetValue( &Led2Timer, 500 );
    TimerStart(&Led2Timer);

    TimerInit( &LedBeaconTimer, OnLedBeaconTimerEvent );
    TimerSetValue( &LedBeaconTimer, 5000 );

    // TimerInit(&WKUPTimer, OnWKUPTimerEvent);
    // TimerSetValue(&WKUPTimer, stop_mode_timer);
    // TimerStart(&WKUPTimer);
    // IsStopMode = 3U;

    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );

    const Version_t appVersion = { .Value = FIRMWARE_VERSION };
    const Version_t gitHubVersion = { .Value = GITHUB_VERSION };
    DisplayAppInfo( "periodic-uplink-lpp", 
                    &appVersion,
                    &gitHubVersion );

    if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
    {
        printf( "LoRaMac wasn't properly initialized\n" );
        // Fatal error, endless loop.
        while ( 1 )
        {
        }
    }

    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError( 20 );

    // The LoRa-Alliance Compliance protocol package should always be
    // initialized and activated.
    LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );

    LmHandlerJoin( );

    StartTxProcess( LORAMAC_HANDLER_TX_ON_TIMER );

    while( 1 )
    {
        // Process characters sent over the command line interface
        CliProcess( &Uart1 );

        // Processes the LoRaMac events
        LmHandlerProcess( );

        // Process application uplinks management
        UplinkProcess( );

        CRITICAL_SECTION_BEGIN( );
        if( IsMacProcessPending == 1 )
        {
            // Clear flag and prevent MCU to go into low power modes.
            IsMacProcessPending = 0;            
        }
        else
        {
            // The MCU wakes up through events
            BoardLowPowerHandler( );

        }
        
        
        CRITICAL_SECTION_END( );
    }
}

static void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
    DisplayNvmDataChange( state, size );
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
    DisplayNetworkParametersUpdate( params );
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
    DisplayMacMcpsRequestUpdate( status, mcpsReq, nextTxIn );
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
    DisplayMacMlmeRequestUpdate( status, mlmeReq, nextTxIn );
}

static void OnJoinRequest( LmHandlerJoinParams_t* params )
{
    DisplayJoinRequestUpdate( params );
    if( params->Status == LORAMAC_HANDLER_ERROR )
    {
        LmHandlerJoin( );
    }
    else
    {
        LmHandlerRequestClass( LORAWAN_DEFAULT_CLASS );
    }
}

static void OnTxData( LmHandlerTxParams_t* params )
{
    DisplayTxUpdate( params );
}

static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params )
{
    DisplayRxUpdate( appData, params );

    switch( appData->Port )
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case LORAWAN_APP_PORT:
        {
            AppLedStateOn = appData->Buffer[0] & 0x01;
            // GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 1 : 0 );
        }
        break;
    default:
        break;
    }

    // Switch LED 2 ON for each received downlink
    // GpioWrite( &Led2, 1 );
    // TimerStart( &Led2Timer );
}

static void OnClassChange( DeviceClass_t deviceClass )
{
    DisplayClassUpdate( deviceClass );

    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Buffer = NULL,
        .BufferSize = 0,
        .Port = 0,
    };
    LmHandlerSend( &appData, LORAMAC_HANDLER_UNCONFIRMED_MSG );
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{
    switch( params->State )
    {
        case LORAMAC_HANDLER_BEACON_RX:
        {
            TimerStart( &LedBeaconTimer );
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            TimerStop( &LedBeaconTimer );
            break;
        }
        default:
        {
            break;
        }
    }

    DisplayBeaconUpdate( params );
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{

}
#else
static void OnSysTimeUpdate( void )
{

}
#endif

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame( void )
{
    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    // uint8_t channel = 0;

    AppData.Port = LORAWAN_APP_PORT;

    // CayenneLppReset( );

    uint8_t potiPercentage = 0;
    // uint16_t vdd = 0;

    // // Read the current potentiometer setting in percent
    // potiPercentage = BoardGetPotiLevel( );

    // // Read the current voltage level
    potiPercentage = BoardGetBatteryLevel( ); // Updates the value returned by BoardGetBatteryVoltage( ) function.
    // vdd = BoardGetBatteryVoltage( );

    // CayenneLppAddDigitalInput( channel++, AppLedStateOn );
    // CayenneLppAddAnalogInput( channel++, BoardGetBatteryLevel( ) * 100 / 254 );
    // CayenneLppAddAnalogInput( channel++, potiPercentage );
    // CayenneLppAddAnalogInput( channel++, vdd );

    // CayenneLppCopy( AppData.Buffer );
    // AppData.BufferSize = CayenneLppGetSize( );

   /* Test uplink WKUP and IO2 */

    uplink_buffer[0U]            = in_cnt << 24U;
    uplink_buffer[1U]            = in_cnt << 16U;
    uplink_buffer[2U]            = in_cnt << 8U;
    uplink_buffer[3U]            = in_cnt & 0x00FF;
    uplink_buffer[4U]            = in_out_cnt << 24U;
    uplink_buffer[5U]            = in_out_cnt << 16U;
    uplink_buffer[6U]            = in_out_cnt << 8U;
    uplink_buffer[7U]            = in_out_cnt & 0x00FF;
    uplink_buffer[8U]            = potiPercentage;

    memcpy1(AppData.Buffer, uplink_buffer, 9U);
    
    AppData.BufferSize = 9U;

    if( LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed ) == LORAMAC_HANDLER_SUCCESS )
    {
        // Switch LED 4 ON
        GpioWrite( &Led4, 1 );
        TimerStart( &Led4Timer );
        NvmmRead(&test_read_buffer[0], 8U, 0x0FF8); 
        if(memcmp(test_read_buffer, uplink_buffer, 8) < 0)
        {
            NvmmWrite(uplink_buffer, 8U, 0x0FF8);
        }
        
    }
}

static void StartTxProcess( LmHandlerTxEvents_t txEvent )
{
    switch( txEvent )
    {
    default:
        // Intentional fall through
    case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit( &TxTimer, OnTxTimerEvent );
            TimerSetValue( &TxTimer, TxPeriodicity );
            OnTxTimerEvent( NULL );
        }
        break;
    case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
        break;
    }
}

static void UplinkProcess( void )
{
    uint8_t isPending = 0;
    CRITICAL_SECTION_BEGIN( );
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    CRITICAL_SECTION_END( );
    if( isPending == 1 )
    {
        PrepareTxFrame( );
    }
}

static void OnTxPeriodicityChanged( uint32_t periodicity )
{
    TxPeriodicity = periodicity;

    if( TxPeriodicity == 0 )
    { // Revert to application default periodicity
        TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
    }
}

static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity )
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
    TimerStop( &TxTimer );

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

/*!
 * Function executed on Led 4 Timeout event
 */
static void OnLed4TimerEvent( void* context )
{
    TimerStop( &Led4Timer );
    // Switch LED 4 OFF
    GpioWrite( &Led4, 0 );
}

/*!
 * Function executed on Led 2 Timeout event
 */
bool IsArc = false;
static void OnLed2TimerEvent( void* context )
{
    TimerStop( &Led2Timer );
    // // Switch LED 2 OFF
    // GpioWrite( &Led2, 0 );

    uint8_t potiLevel = BoardGetPotiLevel();
    
    if(potiLevel < 50 && !IsArc)
    {
        IsArc = true;
        OnIO2Event(NULL);
        printf("Arc lvl: %d\n", potiLevel);
    } else if (potiLevel > 50 && IsArc)
    {
        IsArc = false;
        printf("Arc lvl: %d\n", potiLevel);
    }


    TimerStart( &Led2Timer );
}

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent( void* context )
{
    // GpioWrite( &Led2, 1 );
    // TimerStart( &Led2Timer );

    TimerStart( &LedBeaconTimer );
}

// /*!
//  * Function executed on WKUP Timeout event
//  */
// static void OnWKUPTimerEvent(void *context)
// {
//     IsStopMode = 0U;
//     TimerStop(&WKUPTimer);
// }

/*!
 * Function executed on WKUP event
 */
void OnWKUPEvent(void *context)
{
    uint32_t timeout = 1000000;
    
    CRITICAL_SECTION_BEGIN( );
    
    TimerStop( &TxTimer );
    IsStopMode = 1U;

    while (--timeout > 0x0);
    
    in_cnt++;

    IsTxFramePending = 1;

    CRITICAL_SECTION_END( ) ;
    
    printf("In : %ld\n", in_cnt);
    
    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

/*!
 * Function executed on IO1 event
 */
void OnIO1Event(void *context)
{
    /* Do nothing */
}

/*!
 * Function executed on IO2 event
 */
void OnIO2Event(void *context)
{
    // uint32_t timeout = 1000000;
    
    CRITICAL_SECTION_BEGIN( );
    
    TimerStop( &TxTimer );
    IsStopMode = 2U;
    
    // while (--timeout > 0);

    in_out_cnt++;

    // read/write input/output
    IsTxFramePending = 1;

    CRITICAL_SECTION_END( ) ;

    printf("In Out: %ld\n", in_out_cnt);

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}   