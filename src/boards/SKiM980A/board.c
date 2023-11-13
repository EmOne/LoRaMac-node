/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
 *
 * \author    Andreas Pella ( IMST GmbH )
 */
#include "stm32l1xx.h"
#include "utilities.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "sysIrqHandlers.h"
#include "board-config.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "sx1272-board.h"
#include "board.h"
#include "delay.h"

#if ( USE_TS == 1 )
#include "touchsensing.h"
#endif /* SENSOR */

#if ( USE_IMU == 1 )
#include "math.h"
// #if defined ( USE_C6DOFIMU9 )
#include "c6dofimu9.h"
// #elif defined ( USE_C6DOFIMU17 )
// #include "c6dofimu9.h"
// #elif defined ( USE_INCLINOMETER )
// #include "inclinometer.h"
// #else

// #endif /* */

Gpio_t imu_sync;
Gpio_t imu_interupt;
Gpio_t imu_chipselect;
c6dofimu9_t c6dofimu9;
c6dofimu9_cfg_t cfg;

#endif /* IMU */

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * LED GPIO pins objects
 */
#if ( USE_POTENTIOMETER == 0 )
Gpio_t Led1;
#endif
Gpio_t Led2;
Gpio_t Led3;
Gpio_t Led4;

/*
 * MCU objects
 */
Adc_t Adc;
I2c_t I2c;
Uart_t Uart1;

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;
static bool PeripheralInitialized = false;

/*!
 * UART2 FIFO buffers size
 */
#define UART1_FIFO_TX_SIZE                                1024
#define UART1_FIFO_RX_SIZE                                1024

uint8_t Uart1TxBuffer[UART1_FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[UART1_FIFO_RX_SIZE];

void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
}

void BoardInitPeriph( void )
{
    if (PeripheralInitialized == false)
    {

#if (USE_TS == 1)
    MX_TOUCHSENSING_Init();
#elif (USE_IMU == 1)

    c6dofimu9_cfg_setup( &cfg );
//    C6DOFIMU9_MAP_MIKROBUS( cfg, MIKROBUS_1 );
    c6dofimu9_init(&c6dofimu9, &cfg);

    if( c6dofimu9_get_device_id(&c6dofimu9) != C6DOFIMU9_DEVICE_ID)
    {
    	return;
    }

    c6dofimu9_reset_default(&c6dofimu9);

    c6dofimu9_hw_selftest(&c6dofimu9);
    
    c6dofimu9_set_gyro_config_lp_mode(&c6dofimu9, C6DOFIMU9_GYRO_AVERAGE_1x);
	c6dofimu9_set_gyro_measurement_range(&c6dofimu9,
			C6DOFIMU9_GYRO_FULL_SCALE_250dps);
	c6dofimu9_set_accel_measurement_range(&c6dofimu9,
			C6DOFIMU9_ACCEL_FULL_SCALE_4g);
	c6dofimu9_set_accel_avg_filter_mode(&c6dofimu9,
			C6DOFIMU9_ACCEL_AVERAGE_4_SAMPLES);
        
    // c6dofimu9_get_interrupt_status(&c6dofimu9);
    c6dofimu9_enable_accel_wake_on_motion(&c6dofimu9);
    // c6dofimu9_enable_accel_interrupt(&c6dofimu9);
    c6dofimu9_enable_gyro_interrupt(&c6dofimu9);
    c6dofimu9_enable_data_ready_interrupt(&c6dofimu9);
    // c6dofimu9_enable_fifo(&c6dofimu9, C6DOFIMU9_BIT_ACCEL_FIFO_EN | C6DOFIMU9_BIT_GYRO_FIFO_EN);

#endif

        PeripheralInitialized = true;
    }
}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
        HAL_Init( );

        // LEDs
#if ( USE_TS == 1)
        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );     
#elif (USE_POTENTIOMETER == 1)
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioInit( &Led4, LED_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
#elif ( USE_IMU == 1 )
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
#endif

        SystemClockConfig( );

        FifoInit( &Uart1.FifoTx, Uart1TxBuffer, UART1_FIFO_TX_SIZE );
        FifoInit( &Uart1.FifoRx, Uart1RxBuffer, UART1_FIFO_RX_SIZE );
        // Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
        UartInit( &Uart1, UART_1, UART_TX, UART_RX );
        UartConfig( &Uart1, RX_TX, 921600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        RtcInit( );

        // Switch LED 1, 2, 3, 4 OFF
#if ( USE_TS == 1 )
        GpioWrite( &Led1, 0 );
#elif (USE_POTENTIOMETER == 1)
        GpioWrite( &Led2, 0 );
        GpioWrite( &Led3, 0 );
        GpioWrite( &Led4, 0 );
#elif ( USE_IMU == 1 )
        GpioWrite( &Led2, 0 );
#endif

        BoardUnusedIoInit( );
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            // Disables OFF mode - Enables lowest power mode (STOP)
            LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
        }
    }
    else
    {
        SystemClockReConfig( );
    }

#ifndef USE_TS
    AdcInit( &Adc, POTI );
#else
    AdcInit( &Adc, NC );
#endif

    SpiInit( &SX1272.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX1272IoInit( );

    if( McuInitialized == false )
    {
        McuInitialized = true;
        SX1272IoDbgInit( );
        SX1272IoTcxoInit( );
    }
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

    //Restart system
    NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
    Gpio_t ioPin;

    AdcDeInit( &Adc );

    SpiDeInit( &SX1272.Spi );
    SX1272IoDeInit( );

    GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
    GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

/*!
 * Potentiometer max and min levels definition
 */
#define POTI_MAX_LEVEL 900
#define POTI_MIN_LEVEL 10

#if ( USE_TS == 1)
// For debug purpose with STMStudio
TSL_StateId_enum_T DS[TSLPRM_TOTAL_TKEYS]; // To store the States (one per object)
TSL_tDelta_T DD[TSLPRM_TOTAL_TKEYS]; // To store the Deltas (one per channel)
TSL_tMeas_T DM[TSLPRM_TOTAL_TKEYS];// To store the Measures (one per object)
TSL_tMeas_T DR[TSLPRM_TOTAL_TKEYS];// To store the references (one per object)

tsl_user_status_t status;

void ProcessSensors(void)
{
	uint32_t idx = 0;
	uint32_t idx_ds = 0;
   uint32_t idx_dd = 0;
   uint32_t idx_dm = 0;
   uint32_t idx_dr = 0;

   // STMStudio debug
   for (idx = 0; idx < TSLPRM_TOTAL_TKEYS; idx++)
   {
     DS[idx_ds++] = MyTKeys[idx].p_Data->StateId;
     DD[idx_dd++] = MyTKeys[idx].p_ChD->Delta;
     DM[idx_dm++] = MyTKeys[idx].p_ChD->Meas;
     DR[idx_dr++] = MyTKeys[idx].p_ChD->Ref;
   }

	// // TKEY 0
	// if (TEST_TKEY(0)) {
	// 	LED_BLINK(GREEN, 4);
	// } else {
	// 	LED_GREEN_OFF
	// 	;
	// }

	// // TKEY 1
	// if (TEST_TKEY(1)) {
	// 	LED_BLINK(GREEN, 4);
	// } else {
	// 	LED_GREEN_OFF
	// 	;
	// }

	// if (TEST_TKEY(2)) {
	// 	LED_BLINK(GREEN, 4);
	// } else {
	// 	LED_GREEN_OFF
	// 	;
	// }

	// if (TEST_TKEY(3)) {
	// 	LED_BLINK(GREEN, 4);
	// } else {
	// 	LED_GREEN_OFF
	// 	;
	// }

	// if (TEST_TKEY(4)) {
	// 	LED_BLINK(GREEN, 4);
	// } else {
	// 	LED_GREEN_OFF
	// 	;
	// }
}
#endif

uint8_t potiLevel = 0;

uint8_t BoardGetPotiLevel( void )
{
    
#ifndef USE_TS
    uint16_t vpoti = 0;
    // Read the current potentiometer setting
    vpoti = AdcReadChannel( &Adc , ADC_CHANNEL_3 );

    // check the limits
    if( vpoti >= POTI_MAX_LEVEL )
    {
        potiLevel = 100;
    }
    else if( vpoti <= POTI_MIN_LEVEL )
    {
        potiLevel = 0;
    }
    else
    {
        // if the value is in the area, calculate the percentage value
        potiLevel = ( ( vpoti - POTI_MIN_LEVEL ) * 100 ) / POTI_MAX_LEVEL;
    }
#elif (USE_TS == 1)
     int ch_idx = 0, delta[TSLPRM_TOTAL_TOUCHKEYS];
     int gain = 2;
     status =  tsl_user_Exec();
	 switch (status) {
		case TSL_USER_STATUS_OK_ECS_ON:
        {
			GpioToggle(&Led2);
            for (size_t i = 0; i < TSLPRM_TOTAL_TOUCHKEYS; i++)
            {
                delta[i] = MyTKeys[i].p_ChD->Delta;
                if (delta[i] < 0) DD[i] = delta[i] = 0;                
            }
            for (size_t idx = 1; idx < TSLPRM_TOTAL_TOUCHKEYS-1; idx++)
            {
                if ((delta[ch_idx] * gain) < (delta[idx] * gain))
                    ch_idx = idx;
            }
            potiLevel = ch_idx + 1;
        }
			break;
		case TSL_USER_STATUS_OK_ECS_OFF:
		case TSL_USER_STATUS_OK_NO_ECS:
		case TSL_USER_STATUS_BUSY:
		default:
			GpioWrite(&Led2, 0);
            // potiLevel = 0xff;
			break;
		}
        ProcessSensors();
#elif (USE_IMU == 1)

#endif

    return potiLevel;
}

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY                        3300 // mV

/*!
 * VREF calibration value
 */
#ifdef STM32L151xBA
#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF80078)
/* Internal temperature sensor: constants data used for indicative values in  */
/* this example. Refer to device datasheet for min/typ/max values.            */

/* Internal temperature sensor, parameter TS_CAL1: TS ADC raw data acquired at
 *a temperature of 110 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP30_CAL_ADDR   ((uint16_t*) ((uint32_t) 0x1FF8007A))

/* Internal temperature sensor, parameter TS_CAL2: TS ADC raw data acquired at
 *a temperature of  30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP110_CAL_ADDR  ((uint16_t*) ((uint32_t) 0x1FF8007E))
/* Vdda value with which temperature sensor has been calibrated in production
   (+-10 mV). */
#define VDDA_TEMP_CAL                  ((uint32_t) 3000)
#define VDDA_VREFINT_CAL               ((uint32_t) 3000)

#define COMPUTE_TEMPERATURE(TS_ADC_DATA, VDDA_APPLI)                           \
  ((((( ((int32_t)((TS_ADC_DATA * VDDA_APPLI) / VDDA_TEMP_CAL)                  \
         - (int32_t) *TEMP30_CAL_ADDR)                                          \
      ) * (int32_t)(110 - 30)                                                   \
     )<<8) / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR)                        \
   ) + (30<<8)                                                                      \
  )


#else
#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF800F8U )
#endif
/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE                               4095

/*!
 * VREF bandgap value
 */
#define ADC_VREF_BANDGAP                            1224 // mV

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           3000 // mV
#define BATTERY_MIN_LEVEL                           2400 // mV
#define BATTERY_SHUTDOWN_LEVEL                      2300 // mV

static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;

uint16_t BoardBatteryMeasureVoltage( void )
{
    uint16_t vref = 0;
    uint32_t batteryVoltage = 0;

    // Read the current Voltage
    vref = AdcReadChannel( &Adc , ADC_CHANNEL_VREFINT );

    // We don't use the VREF from calibValues here.
    // calculate the Voltage in millivolt
    batteryVoltage = ( uint32_t )ADC_VREF_BANDGAP * ( uint32_t )ADC_MAX_VALUE;
    batteryVoltage = batteryVoltage / ( uint32_t )vref;

    return batteryVoltage;
}

int16_t BoardGetTemperatureLevel(void)
{
  /* USER CODE BEGIN SYS_GetTemperatureLevel_1 */

  /* USER CODE END SYS_GetTemperatureLevel_1 */
  int16_t temperatureDegreeC = 0;
  uint32_t measuredLevel = 0;
  uint16_t batteryLevelmV = BoardBatteryMeasureVoltage();

  measuredLevel = AdcReadChannel( &Adc, ADC_CHANNEL_TEMPSENSOR);

  temperatureDegreeC = COMPUTE_TEMPERATURE(measuredLevel, batteryLevelmV);

//   APP_LOG(TS_ON, VLEVEL_L, "temp= %d\n\r", temperatureDegreeC >> 8);

  return (int16_t) temperatureDegreeC;
  /* USER CODE BEGIN SYS_GetTemperatureLevel_2 */

  /* USER CODE END SYS_GetTemperatureLevel_2 */
}

#if (Use_ts == 1)
int BoardGetCapSenseStatus( uint8_t* cap )
{
    
   for (size_t i = 0; i < TSLPRM_TOTAL_TKEYS; i++)
   {
        *cap = (uint8_t) DD[i];
        cap++;
   }
   return TSLPRM_TOTAL_TKEYS;
}
#endif

#if (USE_IMU == 1)
float KalmanAngleRoll = 0, KalmanUncertanintyAngleRoll=2*2;
float KalmanAnglePitch = 0, KalmanUncertanintyAnglePitch=2*2;
float Kalman1DOutput[] = {0,0};

void kalman_1d(float *kS, float *kU, const float kI, const float kM){
    float S = *kS;
    float U = *kU;
    float I = kI;
    float M = kM;
    S = S+(0.004*I);
    U = U+(0.004*0.004*4*4);
    float G = U*(1/(1*U+3*3));
    S = S + (G * (M - S));
    U = (1 - G) * U;
    *kS = Kalman1DOutput[0] = S;
    *kU = Kalman1DOutput[1] = U;
}

void BoardGetAngle(float *Roll, float *Pitch)
{
    *Roll = KalmanAngleRoll;
    *Pitch = KalmanAnglePitch;
}

void BoardGetIMU( float* gyro_x ,float* gyro_y, float* gyro_z, float* accel_x ,float* accel_y, float* accel_z)
{
    float gyro_result[3];
    float accel_result[3];
    float rad = M_PI*180.0f;
    double accZInertial, temp;
 
#if 1 //defined(SENSOR_DATA_FROM_REGISTERS)
    // do {
	// 	c6dofimu9_get_interrupt_status(&c6dofimu9);
	// 	DelayMs(DEF_GYRO_WAIT_TIME);
	// } while ((c6dofimu9.irq_status & 0x01) == 0x00);

    c6dofimu9_get_accel_data(&c6dofimu9, &accel_result[0], &accel_result[1],
                             &accel_result[2]);
    DelayMs(10);
    c6dofimu9_get_gyro_data(&c6dofimu9, &gyro_result[0], &gyro_result[1], &gyro_result[2]);
    DelayMs(10);
#else
       
    c6dofimu9_get_imu ( &c6dofimu9, &gyro_result, &accel_result, false );

#endif

	c6dofimu9.a.AngleRoll = atan(c6dofimu9.a.ay/sqrt((c6dofimu9.a.ax*c6dofimu9.a.ax)+(c6dofimu9.a.az*c6dofimu9.a.az))) * 1 / rad;

  	c6dofimu9.a.AnglePitch = -atan(c6dofimu9.a.ax/sqrt((c6dofimu9.a.ay*c6dofimu9.a.ay)+(c6dofimu9.a.az*c6dofimu9.a.az))) * 1 / rad;
    

    kalman_1d(&KalmanAngleRoll,&KalmanUncertanintyAngleRoll,gyro_result[0], c6dofimu9.a.AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertanintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(&KalmanAnglePitch,&KalmanUncertanintyAnglePitch,gyro_result[1], c6dofimu9.a.AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertanintyAnglePitch = Kalman1DOutput[1];

    accZInertial = -sin(c6dofimu9.a.AnglePitch * rad)
							* (c6dofimu9.a.ax + cos(c6dofimu9.a.AnglePitch * rad))
							* sin(c6dofimu9.a.AngleRoll * rad) * (c6dofimu9.a.ay+cos(c6dofimu9.a.AnglePitch * rad))
							* cos(c6dofimu9.a.AngleRoll * rad) * c6dofimu9.a.az;   
  	temp = accZInertial - 1;
	accZInertial = temp * 9.81;
	c6dofimu9.a.AccZInertial = (float) accZInertial * 100;

    *gyro_x = gyro_result[0];
    *gyro_y = gyro_result[1];
    *gyro_z = gyro_result[2];
    *accel_x = c6dofimu9.a.ax;
    *accel_y = c6dofimu9.a.ay;
    *accel_z = c6dofimu9.a.az;
}

void BoardGetIMUTemperature(int32_t * temperature)
{
    c6dofimu9_get_temperature_data(&c6dofimu9, temperature);
}
#endif


float BoardGetTemperature( void )
{
    return BoardGetTemperatureLevel();
}

uint32_t BoardGetBatteryVoltage( void )
{
    return BatteryVoltage;
}

uint8_t BoardGetBatteryLevel( void )
{
    uint8_t batteryLevel = 0;

    BatteryVoltage = BoardBatteryMeasureVoltage( );

    if( GetBoardPowerSource( ) == USB_POWER )
    {
        batteryLevel = 0;
    }
    else
    {
        if( BatteryVoltage >= BATTERY_MAX_LEVEL )
        {
            batteryLevel = 254;
        }
        else if( ( BatteryVoltage > BATTERY_MIN_LEVEL ) && ( BatteryVoltage < BATTERY_MAX_LEVEL ) )
        {
            batteryLevel = ( ( 253 * ( BatteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
        }
        else if( ( BatteryVoltage > BATTERY_SHUTDOWN_LEVEL ) && ( BatteryVoltage <= BATTERY_MIN_LEVEL ) )
        {
            batteryLevel = 1;
        }
        else //if( BatteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
        {
            batteryLevel = 255;
        }
    }
    return batteryLevel;
}

static void BoardUnusedIoInit( void )
{
    Gpio_t ioPin;

    if( GetBoardPowerSource( ) == BATTERY_POWER )
    {
        GpioInit( &ioPin, USB_DM, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, USB_DP, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }

#if defined( USE_DEBUGGER )
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );

    GpioInit( &ioPin, JTAG_TMS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TCK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TDI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_TDO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, JTAG_NRST, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    __HAL_RCC_PWR_CLK_ENABLE( );

    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV     = RCC_PLL_DIV3;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        assert_param( LMN_STATUS_ERROR );
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        assert_param( LMN_STATUS_ERROR );
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        assert_param( LMN_STATUS_ERROR );
    }

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    // Enable HSE
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    // Wait till HSE is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
    {
    }

    // Enable PLL
    __HAL_RCC_PLL_ENABLE( );

    // Wait till PLL is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    // Select PLL as system clock source
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    // Wait till PLL is used as system clock source
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
#if (USE_TS == 1)
    TSL_tim_ProcessIT();
#endif
}

uint8_t GetBoardPowerSource( void )
{
    return USB_POWER;
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exists the function when waking up
  */
void LpmEnterStopMode( void)
{
    CRITICAL_SECTION_BEGIN( );

    BoardDeInitMcu( );

    // Disable the Power Voltage Detector
    HAL_PWR_DisablePVD( );

    // Clear wake up flag
    SET_BIT( PWR->CR, PWR_CR_CWUF );

    // Enable Ultra low power mode
    HAL_PWREx_EnableUltraLowPower( );

    // Enable the fast wake up from Ultra low power mode
    HAL_PWREx_EnableFastWakeUp( );

    CRITICAL_SECTION_END( );

    // Enter Stop Mode
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
}

/*!
 * \brief Exists Low Power Stop Mode
 */
void LpmExitStopMode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );

    // Initilizes the peripherals
    BoardInitMcu( );

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode( void)
{
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void BoardLowPowerHandler( void )
{
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending 
     * and cortex will not enter low power anyway
     */

    LpmEnterLowPower( );

    __enable_irq( );
}

#if !defined ( __CC_ARM )

/*
 * Function to be used by stdout for printf etc
 */
int _write( int fd, const void *buf, size_t count )
{
    while( UartPutBuffer( &Uart1, ( uint8_t* )buf, ( uint16_t )count ) != 0 ){ };
    return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read( int fd, const void *buf, size_t count )
{
    size_t bytesRead = 0;
    while( UartGetBuffer( &Uart1, ( uint8_t* )buf, count, ( uint16_t* )&bytesRead ) != 0 ){ };
    // Echo back the character
    while( UartPutBuffer( &Uart1, ( uint8_t* )buf, ( uint16_t )bytesRead ) != 0 ){ };
    return bytesRead;
}

#else

#include <stdio.h>

// Keil compiler
int fputc( int c, FILE *stream )
{
    while( UartPutChar( &Uart1, ( uint8_t )c ) != 0 );
    return c;
}

int fgetc( FILE *stream )
{
    uint8_t c = 0;
    while( UartGetChar( &Uart1, &c ) != 0 );
    // Echo back the character
    while( UartPutChar( &Uart1, c ) != 0 );
    return ( int )c;
}

#endif

#ifdef USE_FULL_ASSERT

#include <stdio.h>

/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
