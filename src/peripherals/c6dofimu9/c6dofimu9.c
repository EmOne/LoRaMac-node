/*
 * MikroSDK - MikroE Software Development Kit
 * Copyright© 2020 MikroElektronika d.o.o.
 * 
 * Permission is hereby granted, free of charge, to any person 
 * obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
 * OR OTHER DEALINGS IN THE SOFTWARE. 
 */
/*!
 * \file
 *
 */
#include <stdbool.h>
#include "utilities.h"
#include "i2c-board.h"
#include "gpio-board.h"
#include "board-config.h"
#include "math.h"
#include "delay.h"
#include "c6dofimu9.h"

#include "tinyekf_config.h"
#include "tiny_ekf.h"
#include "inttypes.h"
#include "motion_gc_manager.h"

#define SEED 1

static MGC_knobs_t Knobs;
ekf_t ekf;
// Make a place to store the data from the file and the output of the EKF
double SV_Pos[Nsta][Mobs];
double SV_Rho[Mobs];
double Pos_KF[SEED][Mobs];
double mean_Pos_KF[Mobs] = { 0 };
int j, k;
static void ekf_local_init(ekf_t * ekf);
static void ekf_model(ekf_t * ekf);

// ------------------------------------------------------------- PRIVATE MACROS 
extern I2c_t I2c;
extern Gpio_t imu_sync;
extern Gpio_t imu_interupt;
extern Gpio_t imu_chipselect;
extern c6dofimu9_t c6dofimu9;
extern c6dofimu9_cfg_t cfg;

// static uint8_t I2cDeviceAddr = 0;
static bool IMUInitialized = false;

#define NELEM(x) (sizeof(x)/ sizeof((x)[0]))

#define C6DOFIMU9_DUMMY 0

#define CS_PORT		GPIOB
#define CS_PIN		GPIO_PIN_12
#define SCK_PORT 	GPIOB
#define SCK_PIN 	GPIO_PIN_13
#define MISO_PORT	GPIOB
#define MISO_PIN 	GPIO_PIN_14
#define MOSI_PORT	GPIOB
#define MOSI_PIN 	GPIO_PIN_15
#define SCL_PORT	GPIOB
#define SCL_PIN		GPIO_PIN_8
#define SDA_PORT	GPIOB
#define SDA_PIN		GPIO_PIN_9
#if 0
#define SYNC_PORT	GPIOB
#define SYNC_PIN	GPIO_PIN_4
#else
#define SYNC_PORT	GPIOA
#define SYNC_PIN	GPIO_PIN_12
#endif
#define INT_PORT	GPIOA
#define INT_PIN		GPIO_PIN_8


// ---------------------------------------------- PRIVATE FUNCTION DECLARATIONS 

static int c6dofimu9_i2c_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

static int c6dofimu9_i2c_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

static void c6dofimu9_spi_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

static void c6dofimu9_spi_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );
/*!
 * \brief INT PIN IRQ callback
 */
static void c6dofimu9_irq_handler( void* context );


/**
  * @brief  Initialize DWT register for counting clock cycles purpose
  * @param  None
  * @retval None
  */
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
}

/**
  * @brief  Start counting clock cycles
  * @param  None
  * @retval None
  */
static void DWT_Start(void)
{
  DWT->CYCCNT = 0; /* Clear count of clock cycles */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Enable counter */
}

/**
  * @brief  Stop counting clock cycles and calculate elapsed time in [us]
  * @param  None
  * @retval Elapsed time in [us]
  */
static uint32_t DWT_Stop(void)
{
  volatile uint32_t cycles_count = 0U;
  uint32_t system_core_clock_mhz = 0U;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; /* Disable counter */
  cycles_count = DWT->CYCCNT; /* Read count of clock cycles */

  /* Calculate elapsed time in [us] */
  system_core_clock_mhz = SystemCoreClock / 1000000U;
  return cycles_count / system_core_clock_mhz;
}



// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

void c6dofimu9_cfg_setup ( c6dofimu9_cfg_t *cfg )
{
    // Communication gpio pins

//    cfg->scl = NULL; //HAL_PIN_NC;
//    cfg->sda = NULL; //HAL_PIN_NC;
//    cfg->sck = NULL; //HAL_PIN_NC;
//    cfg->miso = NULL; //HAL_PIN_NC;
//    cfg->mosi = NULL; //HAL_PIN_NC;
//    cfg->cs = NULL; //HAL_PIN_NC;

    // Additional gpio pins

//    cfg->syn = NULL; //HAL_PIN_NC;
//    cfg->int_pin = NULL; //HAL_PIN_NC;

    cfg->i2c_speed = 100000; //I2C_MASTER_SPEED_STANDARD;
    cfg->i2c_address = C6DOFIMU9_I2C_SLAVE_ADDRESS_1;
    cfg->spi_speed = 100000;
    cfg->spi_mode = 0; //SPI_MASTER_MODE_0;
    cfg->sel = C6DOFIMU9_MASTER_I2C;
    cfg->cs_polarity = 0; //SPI_MASTER_CHIP_SELECT_POLARITY_ACTIVE_LOW;
}

//C6DOFIMU9_RETVAL c6dofimu9_init ( c6dofimu9_t *ctx, c6dofimu9_cfg_t *cfg )
C6DOFIMU9_RETVAL c6dofimu9_init ( c6dofimu9_t *ctx, c6dofimu9_cfg_t *cfg )
{
    if (IMUInitialized == 0)
    {
        // Only in case it is necessary to check somewhere which communication is set
        ctx->master_sel = cfg->sel;

        if ( ctx->master_sel == C6DOFIMU9_MASTER_I2C )
        {
    //        i2c_master_config_t i2c_cfg;
    //
    //        i2c_master_configure_default( &i2c_cfg );
    //        i2c_cfg.speed    = cfg->i2c_speed;
    //        i2c_cfg.scl = cfg->scl;
    //        i2c_cfg.sda = cfg->sda;

    //         ctx->i2c.Instance = I2C1;
    //         ctx->i2c.Init.ClockSpeed = cfg->i2c_speed;
    //         ctx->i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    //         ctx->i2c.Init.OwnAddress1 = 0;
    //         ctx->i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    //         ctx->i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    //         ctx->i2c.Init.OwnAddress2 = 0;
    //         ctx->i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    //         ctx->i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    //         __HAL_RCC_GPIOB_CLK_ENABLE();

    //         cfg->scl.Pin = SCL_PIN;
    //         cfg->scl.Mode = GPIO_MODE_AF_OD;
    //         cfg->scl.Alternate = GPIO_AF4_I2C1;
    //         cfg->scl.Pull = GPIO_NOPULL;
    //         cfg->scl.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //         HAL_GPIO_Init(SCL_PORT, &cfg->scl);

    //         cfg->sda.Pin = SDA_PIN;
    //         cfg->sda.Mode = GPIO_MODE_AF_OD;
    //         cfg->sda.Alternate = GPIO_AF4_I2C1;
    //         cfg->sda.Pull = GPIO_NOPULL;
    //         cfg->sda.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //         HAL_GPIO_Init(SDA_PORT, &cfg->sda);

    //         __HAL_RCC_I2C1_CLK_ENABLE();

    //         __HAL_RCC_I2C1_FORCE_RESET();

    //         __HAL_RCC_I2C1_RELEASE_RESET();

    //         if (HAL_I2C_Init(&ctx->i2c) != HAL_OK)
    // //        if (  i2c_master_open( &ctx->i2c, &i2c_cfg ) == I2C_MASTER_ERROR )
    //         {
    //             return C6DOFIMU9_INIT_ERROR;
    //         }

			I2cMcuInit( &I2c, I2C_1, I2C_SCL, I2C_SDA );
			I2cMcuFormat( &I2c, MODE_I2C, I2C_DUTYCYCLE_2, true, I2C_ACK_ADD_7_BIT, cfg->i2c_speed );
    //        i2c_master_set_slave_address( &ctx->i2c, ctx->slave_address );
            ctx->slave_address = cfg->i2c_address;

    //        i2c_master_set_speed( &ctx->i2c, cfg->i2c_speed );
    //
            ctx->read_f = c6dofimu9_i2c_read;
            ctx->write_f = c6dofimu9_i2c_write;
    //
    //        spi_master_deselect_device( ctx->chip_select );
            ctx->cs = CS_PORT;

            cfg->cs.Pin = CS_PIN;
            cfg->cs.Mode = GPIO_MODE_OUTPUT_PP;
            cfg->cs.Pull = GPIO_PULLUP;
            cfg->cs.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
            // HAL_GPIO_Init(ctx->cs, &cfg->cs);
            // HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
			GpioInit( &imu_chipselect, IMU_CS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
        }
        else
        {
    //        spi_master_config_t spi_cfg;
    //
    //        spi_master_configure_default( &spi_cfg );
    //        spi_cfg.mode        = cfg->spi_mode;
    //        spi_cfg.speed  = cfg->spi_speed;
    //        spi_cfg.sck    = cfg->sck;
    //        spi_cfg.miso   = cfg->miso;
    //        spi_cfg.mosi   = cfg->mosi;

    //         ctx->spi.Instance = SPI2;
    //         ctx->spi.Init.Mode = SPI_MODE_MASTER;
    //         ctx->spi.Init.Direction = SPI_DIRECTION_2LINES;
    //         ctx->spi.Init.DataSize = SPI_DATASIZE_8BIT;
    //         ctx->spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    //         ctx->spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    //         ctx->spi.Init.NSS = SPI_NSS_SOFT;
    //         ctx->spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    //         ctx->spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    //         ctx->spi.Init.TIMode = SPI_TIMODE_DISABLE;
    //         ctx->spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    //         ctx->spi.Init.CRCPolynomial = 10;

    //         ctx->cs = CS_PORT;

    //         cfg->cs.Pin = CS_PIN;
    //         cfg->cs.Mode = GPIO_MODE_OUTPUT_PP;
    //         cfg->cs.Pull = GPIO_NOPULL;
    //         cfg->cs.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //         HAL_GPIO_Init(ctx->cs, &cfg->cs);
    // //        digital_out_init( &ctx->cs, cfg->cs );
    //         ctx->chip_select = cfg->cs;
    // //
    //         __HAL_RCC_GPIOB_CLK_ENABLE();

    //         cfg->sck.Pin = SCK_PIN;
    //         cfg->sck.Mode = GPIO_MODE_AF_PP;
    //         cfg->sck.Alternate = GPIO_AF5_SPI2;
    //         cfg->sck.Pull = GPIO_PULLDOWN;
    //         cfg->sck.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //         HAL_GPIO_Init(SCK_PORT, &cfg->sck);

    //         cfg->miso.Pin = MISO_PIN;
    //         cfg->miso.Mode = GPIO_MODE_AF_PP;
    //         cfg->miso.Alternate = GPIO_AF5_SPI2;
    //         cfg->miso.Pull = GPIO_PULLDOWN;
    //         cfg->miso.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //         HAL_GPIO_Init(MISO_PORT, &cfg->miso);

    //         cfg->mosi.Pin = MOSI_PIN;
    //         cfg->mosi.Mode = GPIO_MODE_AF_PP;
    //         cfg->mosi.Alternate = GPIO_AF5_SPI2;
    //         cfg->mosi.Pull = GPIO_PULLDOWN;
    //         cfg->mosi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //         HAL_GPIO_Init(MOSI_PORT, &cfg->mosi);

    //         __HAL_RCC_SPI2_CLK_ENABLE();

    // //        if (  spi_master_open( &ctx->spi, &spi_cfg ) == SPI_MASTER_ERROR )
    //         if (  HAL_SPI_Init(&ctx->spi) != HAL_OK)
    //         {
    //             return  C6DOFIMU9_INIT_ERROR;
    //         }

    // //
    // //        spi_master_set_default_write_data( &ctx->spi, C6DOFIMU9_DUMMY );
    // //        spi_master_set_speed( &ctx->spi, spi_cfg.speed );
    // //        spi_master_set_mode( &ctx->spi, spi_cfg.mode );
    // //        spi_master_set_chip_select_polarity( cfg->cs_polarity );

    //         ctx->read_f = c6dofimu9_spi_read;
    //         ctx->write_f = c6dofimu9_spi_write;

    // //        spi_master_select_device( ctx->chip_select );
    //         ctx->cs = CS_PORT;

    //         cfg->cs.Pin = CS_PIN;
    //         cfg->cs.Mode = GPIO_MODE_OUTPUT_PP;
    //         cfg->cs.Pull = GPIO_PULLUP;
    //         cfg->cs.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    //         HAL_GPIO_Init(ctx->cs, &cfg->cs);
    //         HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
	// 		GpioInit( &imu_sync, IMU_SYNC, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        }
        
        // Output pins 
    //    digital_out_init( &ctx->syn, cfg->syn );
        // __HAL_RCC_GPIOB_CLK_ENABLE();
        ctx->syn = SYNC_PORT;
        cfg->syn.Pin = SYNC_PIN;
        cfg->syn.Mode = GPIO_MODE_OUTPUT_PP;
        cfg->syn.Pull = GPIO_NOPULL;
        cfg->syn.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        // HAL_GPIO_Init(ctx->syn, &cfg->syn);

        // Input pins
        
    //    digital_in_init( &ctx->int_pin, cfg->int_pin );
        // __HAL_RCC_GPIOA_CLK_ENABLE();
        ctx->int_pin = INT_PORT;
        cfg->int_pin.Pin = INT_PIN;
        cfg->int_pin.Mode = GPIO_MODE_INPUT;
        cfg->int_pin.Pull = GPIO_NOPULL;
        cfg->int_pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        // HAL_GPIO_Init(ctx->int_pin, &cfg->int_pin);

        GpioInit( &imu_sync, IMU_SYNC, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &imu_interupt, IMU_INT, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioSetInterrupt( &imu_interupt, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, c6dofimu9_irq_handler );

		ekf_init(&ekf, Nsta, Mobs);
		ekf_local_init(&ekf);
		/* Get current settings and set desired ones */
		MotionGC_GetKnobs(&Knobs);
		Knobs.AccThr = 0.004f;
		Knobs.GyroThr = 0.001f;
		Knobs.MaxGyro = 250;
		Knobs.MaxAcc = 12;
		(void)MotionGC_SetKnobs(&Knobs);

		/* Set sample frequency */
  
		MotionGC_manager_set_frequency(1000.0f / 10);

		DWT_Init();

        IMUInitialized = 1;
    }
    return C6DOFIMU9_OK;
}

int c6dofimu9_generic_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    return ctx->write_f( ctx, reg, data_buf, len ); 
    
}

int c6dofimu9_generic_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    return ctx->read_f( ctx, reg, data_buf, len );
}

uint8_t c6dofimu9_check_interrupt ( c6dofimu9_t *ctx )
{
    return HAL_GPIO_ReadPin(ctx->int_pin, GPIO_PIN_8);// digital_in_read( &ctx->int_pin );
}

void c6dofimu9_set_config ( c6dofimu9_t *ctx, uint8_t config_data )
{
    config_data &= 0x7F;
    
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_CONFIG, &config_data, 1 );
}

void c6dofimu9_set_gyro_config ( c6dofimu9_t *ctx, uint8_t gyro_config_data )
{
    gyro_config_data &= 0xFB;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_GYRO_CONFIG, &gyro_config_data, 1 );
}

void c6dofimu9_set_gyro_measurement_range ( c6dofimu9_t *ctx, uint16_t gyro_full_scale_range )
{
    uint8_t temp;
    
    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_GYRO_CONFIG, &temp, 1 );
    temp &= 0xE7;
    
    switch ( gyro_full_scale_range )
    {
        case C6DOFIMU9_GYRO_FULL_SCALE_250dps:
        {
            temp |= 0x00;
            break;
        }
        case C6DOFIMU9_GYRO_FULL_SCALE_500dps:
        {
            temp |= 0x08;
            break;
        }
        case C6DOFIMU9_GYRO_FULL_SCALE_1000dps:
        {
            temp |= 0x10;
            break;
        }
        case C6DOFIMU9_GYRO_FULL_SCALE_2000dps:
        {
            temp |= 0x18;
            break;
        }
        default:
        {
            temp |= 0x00;
            break;
        }
    }
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_GYRO_CONFIG, &temp, 1 );
}

void c6dofimu9_set_accel_config ( c6dofimu9_t *ctx, uint16_t accel_config_data )
{
    uint8_t temp;
    
    accel_config_data &= 0xFC3F;
    
    temp = 0x00;
    temp = ( uint8_t ) ( accel_config_data >> 8 );
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_ACCEL_CONFIG, &temp, 1 );
    
    temp = 0x00;
    temp = ( uint8_t ) accel_config_data;
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_ACCEL_CONFIG_2, &temp, 1 );
}

void c6dofimu9_set_accel_measurement_range ( c6dofimu9_t *ctx, uint8_t accel_full_scale_range )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_ACCEL_CONFIG, &temp, 1 );
    temp &= 0xE7;

    switch ( accel_full_scale_range )
    {
        case C6DOFIMU9_ACCEL_FULL_SCALE_2g:
        {
            temp |= 0x00;
            break;
        }
        case C6DOFIMU9_ACCEL_FULL_SCALE_4g:
        {
            temp |= 0x08;
            break;
        }
        case C6DOFIMU9_ACCEL_FULL_SCALE_8g:
        {
            temp |= 0x10;
            break;
        }
        case C6DOFIMU9_ACCEL_FULL_SCALE_16g:
        {
            temp |= 0x18;
            break;
        }
        default:
        {
            temp |= 0x00;
            break;
        }
    }
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_ACCEL_CONFIG, &temp, 1 );
}

void c6dofimu9_set_accel_avg_filter_mode ( c6dofimu9_t *ctx, uint8_t accel_avg_samples )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_ACCEL_CONFIG_2, &temp, 1 );
    temp &= 0xCF;

    switch ( accel_avg_samples )
    {
        case C6DOFIMU9_ACCEL_AVERAGE_4_SAMPLES:
        {
            temp |= 0x00;
            break;
        }
        case C6DOFIMU9_ACCEL_AVERAGE_8_SAMPLES:
        {
            temp |= 0x10;
            break;
        }
        case C6DOFIMU9_ACCEL_AVERAGE_16_SAMPLES:
        {
            temp |= 0x20;
            break;
        }
        case C6DOFIMU9_ACCEL_AVERAGE_32_SAMPLES:
        {
            temp |= 0x30;
            break;
        }
        default:
        {
            temp |= 0x00;
            break;
        }
    }
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_ACCEL_CONFIG_2, &temp, 1 );
}

void c6dofimu9_set_gyro_config_lp_mode ( c6dofimu9_t *ctx, uint8_t gyro_averages )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_LP_MODE_CFG, &temp, 1 );
    temp &= 0x8F;
    temp |= 0x80;

    switch ( gyro_averages )
    {
        case C6DOFIMU9_GYRO_AVERAGE_1x:
        {
            temp |= 0x00;
            break;
        }
        case C6DOFIMU9_GYRO_AVERAGE_2x:
        {
            temp |= 0x10;
            break;
        }
        case C6DOFIMU9_GYRO_AVERAGE_4x:
        {
            temp |= 0x20;
            break;
        }
        case C6DOFIMU9_GYRO_AVERAGE_8x:
        {
            temp |= 0x30;
            break;
        }
        case C6DOFIMU9_GYRO_AVERAGE_16x:
        {
            temp |= 0x40;
            break;
        }
        case C6DOFIMU9_GYRO_AVERAGE_32x:
        {
            temp |= 0x50;
            break;
        }
        case C6DOFIMU9_GYRO_AVERAGE_64x:
        {
            temp |= 0x60;
            break;
        }
        case C6DOFIMU9_GYRO_AVERAGE_128x:
        {
            temp |= 0x70;
            break;
        }
        default:
        {
            temp |= 0x00;
            break;
        }
    }
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_LP_MODE_CFG, &temp, 1 );
}

uint8_t c6dofimu9_get_accel_wake_on_motion_trsh ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_ACCEL_WOM_THR, &temp, 1 );
    
    return temp;
}

void c6dofimu9_enable_fifo ( c6dofimu9_t *ctx, uint8_t en_fifo_data )
{
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_FIFO_EN, &en_fifo_data, 1 );
}

void c6dofimu9_enable_accel_interrupt( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read ( ctx, C6DOFIMU9_REG_INT_ENABLE, &temp, 1 );

    temp |= 0xE0;
    
    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_INT_ENABLE, &temp, 1 );
}

void c6dofimu9_enable_gyro_interrupt ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_INT_ENABLE, &temp, 1 );

    temp |= 0x04;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_INT_ENABLE, &temp, 1 );
}

void c6dofimu9_enable_data_ready_interrupt ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_INT_ENABLE, &temp, 1 );

    temp |= 0x01;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_INT_ENABLE, &temp, 1 );
}

uint8_t c6dofimu9_get_interrupt_status ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_INT_STATUS, &temp, 1 );
	ctx->irq_status = temp;
    return temp;
}

void c6dofimu9_reset_accel_signal ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_SIGNAL_PATH_RESET, &temp, 1 );

    temp &= 0x00;
    temp |= 0x02;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_SIGNAL_PATH_RESET, &temp, 1 );
}

void c6dofimu9_reset_gyro_signal ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_SIGNAL_PATH_RESET, &temp, 1 );

    temp &= 0x00;
    temp |= 0x01;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_SIGNAL_PATH_RESET, &temp, 1 );
}

void c6dofimu9_enable_accel_wake_on_motion ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_ACCEL_INTEL_CTRL, &temp, 1 );

    temp |= 0x80;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_ACCEL_INTEL_CTRL, &temp, 1 );
}

void c6dofimu9_reset_default ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_PWR_MGMT_1, &temp, 1 );

    temp |= 0x80;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_PWR_MGMT_1, &temp, 1 );
}

void c6dofimu9_set_sleep_mode ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_PWR_MGMT_1, &temp, 1 );

    temp |= 0x40;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_PWR_MGMT_1, &temp, 1 );
}

void c6dofimu9_set_standby_mode ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_PWR_MGMT_1, &temp, 1 );

    temp &= 0xBF;

    c6dofimu9_generic_write( ctx, C6DOFIMU9_REG_PWR_MGMT_1, &temp, 1 );
}

uint8_t c6dofimu9_get_device_id ( c6dofimu9_t *ctx )
{
    uint8_t temp;

    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_WHO_AM_I, &temp, 1 );

    return temp;
}

int16_t c6dofimu9_get_axis ( c6dofimu9_t *ctx, uint8_t addr_reg_msb, uint8_t addr_reg_lsb )
{
    int16_t result;
    uint16_t temp;
    uint8_t read_buffer[ 2 ];

    c6dofimu9_generic_read( ctx, addr_reg_msb, read_buffer, 2 );

    temp = read_buffer[ 0 ];
    temp <<= 8;
    temp |= read_buffer[ 1 ];
    
    result = ( int16_t ) temp;

    return result;
}

void c6dofimu9_get_accel_data ( c6dofimu9_t *ctx, float *p_accel_x, float *p_accel_y, float *p_accel_z )
{

    ctx->a.ax = (double) (c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_ACCEL_XOUT_H, C6DOFIMU9_REG_ACCEL_XOUT_L ) ) * IAM20680_SENSITIVITY_0_12G;
    ctx->a.ay = (double) (c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_ACCEL_YOUT_H, C6DOFIMU9_REG_ACCEL_YOUT_L ) ) * IAM20680_SENSITIVITY_0_12G;
    ctx->a.az = (double) (c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_ACCEL_ZOUT_H, C6DOFIMU9_REG_ACCEL_ZOUT_L ) ) * IAM20680_SENSITIVITY_0_12G;

	ctx->a.ax -= ctx->accel_st_bias[0];
	ctx->a.ay -= ctx->accel_st_bias[1];
	ctx->a.az -= ctx->accel_st_bias[2];

	ctx->a.ax /= 1000;
	ctx->a.ay /= 1000;
	ctx->a.az /= 1000;

	//Please adjust Accel X, Y, Z here
	ctx->a.ax += 0.008;
	ctx->a.ay += 0.012;
	ctx->a.az += 0.023;

	*p_accel_x = (float) ctx->a.ax;
	*p_accel_y = (float) ctx->a.ay;
	*p_accel_z = (float) ctx->a.az;
}

void c6dofimu9_get_gyro_data ( c6dofimu9_t *ctx, float *p_gyro_x, float *p_gyro_y, float *p_gyro_z )
{
   ctx->g.gx = (double) (c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_GYRO_XOUT_H, C6DOFIMU9_REG_GYRO_XOUT_L ) ) * IAM20680_GYRO_SENSITIVITY_250;
   ctx->g.gy = (double) (c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_GYRO_YOUT_H, C6DOFIMU9_REG_GYRO_YOUT_L ) ) * IAM20680_GYRO_SENSITIVITY_250;
   ctx->g.gz = (double) (c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_GYRO_ZOUT_H, C6DOFIMU9_REG_GYRO_ZOUT_L ) ) * IAM20680_GYRO_SENSITIVITY_250;

	ctx->g.gx -= ctx->gyro_st_bias[0];
	ctx->g.gy -= ctx->gyro_st_bias[1];
	ctx->g.gz -= ctx->gyro_st_bias[2];

	//Please adjust GYRO X, Y, Z here
	// ctx->g.gx += 0.008;
	// ctx->g.gy += 0.012;
	// ctx->g.gz += 0.023;

	*p_gyro_x = (float) (ctx->g.gx);
	*p_gyro_y = (float) (ctx->g.gy);
	*p_gyro_z = (float) (ctx->g.gz);

#if 1
//   	MGC_input_t data_in;
//   	MGC_output_t data_out;
// 	uint32_t elapsed_time_us = 0U;
//   	int bias_update = 0;

//   /* Convert acceleration [g] */
//       data_in.Acc[0] = (float)ctx->a.ax;
//       data_in.Acc[1] = (float)ctx->a.ay;
//       data_in.Acc[2] = (float)ctx->a.az;

//       /* Convert angular velocity [dps] */
//       data_in.Gyro[0] = (float)ctx->g.gx ;
//       data_in.Gyro[1] = (float)ctx->g.gy ;
//       data_in.Gyro[2] = (float)ctx->g.gz ;

// 	  DWT_Start();

//       MotionGC_manager_update(&data_in, &data_out, &bias_update);

// 	  MotionGC_manager_compensate(&ctx->g, &ctx->delta_g);

// 	*p_gyro_x = data_out.GyroBiasX; //(ctx->delta_g.gx);
// 	*p_gyro_y = data_out.GyroBiasY;
// 	*p_gyro_z = data_out.GyroBiasZ;

// 	  SV_Rho[0] = (ctx->g.gx);
// 	  SV_Rho[1] = (ctx->g.gy);
// 	  SV_Rho[2] = (ctx->g.gz);
// // 	  //			SV_Rho[3] = (double) flow.temp / 10.0f;

// 	  /* Update autoreload and capture compare registers value */
// 	  ctx->abs_g.gx = fabs(ctx->g.gx);
// 	  ctx->abs_g.gy = fabs(ctx->g.gy);
// 	  ctx->abs_g.gz = fabs(ctx->g.gz);
	  			
// // // //			sprintf(txt, "$E:%lf %lf %lf\r\n", flow.e.roll, flow.e.pitch, flow.e.yaw);
// // // //			len = strlen(txt);
// // // //			HAL_UART_Transmit(&huart1, (uint8_t*) txt, len, 500);
// // // //			CDC_Transmit_HS((uint8_t*) txt, len);

// 		ekf_model(&ekf);
// //
// 		ekf_step(&ekf, (double*) SV_Rho);
// // //
// // //	// estimate
// 		for (k = 0; k < Nsta; ++k) {
// 			Pos_KF[j][k] = ekf.x[k];
// 			*(((double *)&ctx->ekf_g) + k) = ekf.x[k];
// 		}

// 		// *p_gyro_x = (ctx->ekf_g.gx);
// 		// *p_gyro_y = (ctx->ekf_g.gy);
// 		// *p_gyro_z = (ctx->ekf_g.gz);

// 		for (k = 0; k < Mobs; ++k)
// 			mean_Pos_KF[k] = 0;
// 		for (j = 0; j < SEED; ++j)
// 			for (k = 0; k < Nsta; ++k)
// 				mean_Pos_KF[k] += Pos_KF[j][k];
// 		for (k = 0; k < Mobs; ++k)
// 			mean_Pos_KF[k] /= SEED;

// 		// Dump filtered positions minus their means
// 		ctx->delta_g.gx = 0;
// 		ctx->delta_g.gy = 0;
// 		ctx->delta_g.gz = 0;
// 		for (j = 0; j < SEED; ++j) {

// // 			printf("$M:%ld %ld %ld\r\n", (int16_t) mean_Pos_KF[0], (int16_t) mean_Pos_KF[1],
// // 					(int16_t) mean_Pos_KF[2]);
// // //			len = strlen(txt);
// // //			HAL_UART_Transmit(&huart1, (uint8_t*) txt, len, 500);
// // //			CDC_Transmit_HS((uint8_t*) txt, len);

// 			ctx->delta_g.gx += (ctx->ekf_g.gx - mean_Pos_KF[0]);
// 			ctx->delta_g.gy += (ctx->ekf_g.gy - mean_Pos_KF[1]);
// 			ctx->delta_g.gz += (ctx->ekf_g.gz - mean_Pos_KF[2]);
// 		}
// 		ctx->delta_g.gx /= SEED;
// 		ctx->delta_g.gy /= SEED;
// 		ctx->delta_g.gz /= SEED;

// 		*p_gyro_x = (int16_t) (ctx->delta_g.gx);
// 		*p_gyro_y = (int16_t) (ctx->delta_g.gy);
// 		*p_gyro_z = (int16_t) (ctx->delta_g.gz);

// 		printf("$D:%ld %ld %ld\r\n", (int16_t) (ctx->delta_g.gx* DEF_ST_PRECISION), (int16_t) (ctx->delta_g.gy* DEF_ST_PRECISION),
// 				(int16_t) (ctx->delta_g.gz* DEF_ST_PRECISION));

// 		ctx->q = ToQuaternion(ctx->ekf_e);

// 		printf("$Q:%ld %ld %ld %ld\r\n", (int32_t) (ctx->q.x * 1000), (int32_t) (ctx->q.y * 1000),
// 				(int32_t) (ctx->q.z * 1000), (int32_t) (ctx->q.w * 1000));
#endif
}

void c6dofimu9_get_temperature_data ( c6dofimu9_t *ctx, int32_t *temp )
{
    uint8_t data[2];
    int16_t temp_out;
    c6dofimu9_generic_read( ctx, C6DOFIMU9_REG_TEMP_OUT_H, data, 2 );
    temp_out = (data[0] << 8 | data[1]) * 10000;
    temp_out = (temp_out / TEMP_SENSITIVITY) + TEMP_OFFSET;
    *temp = (int32_t) temp_out;
}

int c6dofimu9_get_imu ( c6dofimu9_t *ctx, int *gyro_result, int *accel_result, int lp_mode )
{
	int result, i, j, packet_size, total_bytes;
	uint8_t data[BYTES_PER_SENSOR * 2], d, dd;
	int fifo_count, packet_count, ind;
    uint8_t reset = 0;
	packet_size = BYTES_PER_SENSOR * 2;

	/* disable interrupt */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_INT_ENABLE, &reset, 1);
	if (result)
		return result;
	/* disable the sensor output to FIFO */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_FIFO_EN, &reset, 1);
	if (result)
		return result;
	/* disable fifo reading */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &reset, 1);
	if (result)
		return result;
	/* clear FIFO */
    d = C6DOFIMU9_BIT_FIFO_RST;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &d, 1);
	if (result)
		return result;
	/* setup parameters */
    d = INV_FILTER_98HZ;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_CONFIG, &d, 1);
	if (result)
		return result;

	/* gyro lp mode */
	if (lp_mode == 1)
		d = C6DOFIMU9_BIT_GYRO_CYCLE_EN;
	else if (lp_mode == 2)
		d = DEF_SELFTEST_SAMPLE_RATE_ACC_LP;
	else
		d = 0;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_LP_MODE_CFG, &d, 1);
	if (result)
		return result;

	/* config accel LPF register */
	if (lp_mode == 2)
		d = C6DOFIMU9_BIT_ACCEL_FCHOCIE_B;
	else
		d = DEF_ST_IAM20680_ACCEL_LPF;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_20680_ACCEL_CONFIG2, &d, 1);
	if (result)
		return result;

	if (lp_mode) {
        d = DEF_SELFTEST_SAMPLE_RATE_LP;
	} else {
        d = DEF_SELFTEST_SAMPLE_RATE;
	}
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_SMPLRT_DIV,
				&d, 1);
	if (result)
		return result;

	for (i = 0; i < THREE_AXIS; i++) {
		gyro_result[i] = 0;
		accel_result[i] = 0;
	}

	/* enable FIFO reading */
	dd = C6DOFIMU9_BIT_FIFO_EN;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &dd, 1);
	if (result)
		return result;

	/* accel lp mode */
	dd = C6DOFIMU9_BIT_CLK_PLL;
	dd &= ~C6DOFIMU9_BIT_LP_EN;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_1, &dd, 1);
	if (result)
		return result;
	
	dd = C6DOFIMU9_BIT_GYRO_FIFO_EN	| C6DOFIMU9_BIT_ACCEL_FIFO_EN;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_FIFO_EN, &dd, 1);
	
	if (result)
		return result;
	
	do {
		c6dofimu9_get_interrupt_status(&c6dofimu9);
		DelayMs(DEF_GYRO_WAIT_TIME);
	} while ((c6dofimu9.irq_status & 0x01) == 0x00);
	
	/* disable the sensor output to FIFO */
	reset = 0;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_FIFO_EN, &reset, 1);
	if (result)
		return result;

	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_FIFO_COUNTH,
				data, FIFO_COUNT_BYTE);

	fifo_count = data[0] << 8 | data[1];
	// pr_debug("%s self_test fifo_count - %d\n",
	// 	 st->hw->name, fifo_count);
	packet_count = fifo_count / packet_size;
	total_bytes = fifo_count;
	i = 0;
	while ((i < packet_count))
	{
		short vals[3];

		result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_FIFO_R_W,
										data, packet_size);
		if (result)
			return result;
		ind = 0;

		for (j = 0; j < THREE_AXIS; j++)
		{
			// vals[j] = (short) be16_to_cpup(
			// 	(__be16 *)(&data[ind + 2 * j]));
			vals[j] = data[ind + 2 * j] << 8 | data[(ind + 2 * j) + 1];
			accel_result[j] += vals[j];
		}
		ind += BYTES_PER_SENSOR;
		// pr_debug(
		// 	"%s self_test accel data - %d %+d %+d %+d\n",
		// 	st->hw->name, s, vals[0], vals[1], vals[2]);

		for (j = 0; j < THREE_AXIS; j++)
		{
			// vals[j] = (short)be16_to_cpup(
			// 	(__be16 *)(&data[ind + 2 * j]));
			vals[j] = data[ind + 2 * j] << 8 | data[(ind + 2 * j) + 1];
			gyro_result[j] += vals[j];
		}
		// pr_debug("%s self_test gyro data - %d %+d %+d %+d\n",
		// 	st->hw->name, s, vals[0], vals[1], vals[2]);
		i++;
	}

	// for (j = 0; j < THREE_AXIS; j++) {
	// 	accel_result[j] = accel_result[j] / packet_count;
	// 	accel_result[j] *= DEF_ST_PRECISION;
	// }
	// for (j = 0; j < THREE_AXIS; j++) {
	// 	gyro_result[j] = gyro_result[j] / packet_count;
	// 	gyro_result[j] *= DEF_ST_PRECISION;
	// }

// /* Update autoreload and capture compare registers value */
// 			ctx->abs_g.gx = fabs((double) gyro_result[0]);
// 			ctx->abs_g.gy = fabs((double) gyro_result[1]);
// 			ctx->abs_g.gz = fabs((double) gyro_result[2]);
// //			SV_Rho[3] = (double) flow.temp / 10.0f;
// 			SV_Rho[0] = ctx->g.gx = (double) gyro_result[0];
// 			SV_Rho[1] = ctx->g.gy = (double) gyro_result[1];
// 			SV_Rho[2] = ctx->g.gz = (double) gyro_result[2];
// //			sprintf(txt, "$E:%lf %lf %lf\r\n", flow.e.roll, flow.e.pitch, flow.e.yaw);
// //			len = strlen(txt);
// //			HAL_UART_Transmit(&huart1, (uint8_t*) txt, len, 500);
// //			CDC_Transmit_HS((uint8_t*) txt, len);

// 			ekf_model(&ekf);
// //
// 			ekf_step(&ekf, (double*) SV_Rho);
// //
// //			// estimate
// 			for (k = 0; k < Nsta; ++k)
// 				*(((double *)&ctx->ekf_g) + k) = Pos_KF[j][k] = ekf.x[k];

// 			gyro_result[0] = (int16_t) (ctx->ekf_g.gx);
// 			gyro_result[1] = (int16_t) (ctx->ekf_g.gy);
// 			gyro_result[2] = (int16_t) (ctx->ekf_g.gz);

}
// ----------------------------------------------- PRIVATE FUNCTION DEFINITIONS

static int c6dofimu9_i2c_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
//    uint8_t tx_buf[ 256 ];
//    uint8_t cnt;
//
//    tx_buf[ 0 ] = reg;
//
//    for ( cnt = 1; cnt <= len; cnt++ )
//    {
//        tx_buf[ cnt ] = data_buf[ cnt - 1 ];
//    }

//    i2c_master_write( &ctx->i2c, tx_buf, len + 1 );
//    HAL_I2C_Master_Transmit(&ctx->i2c, ctx->slave_address << 1, tx_buf, len+1, 1000);
	return I2cMcuWriteMemBuffer( &I2c, (ctx->slave_address << 1), (uint16_t) reg, data_buf, (uint16_t) len );
    // return HAL_I2C_Mem_Write(&ctx->i2c, (uint16_t) (ctx->slave_address << 1), (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, data_buf, len, 1000);
}

static int c6dofimu9_i2c_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
//    i2c_master_write_then_read( &ctx->i2c, &reg, 1, data_buf, len );
	return I2cMcuReadMemBuffer( &I2c, (ctx->slave_address << 1), (uint16_t) reg, data_buf, (uint16_t) len );
	// return HAL_I2C_Mem_Read(&ctx->i2c, (uint16_t) (ctx->slave_address << 1), (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, data_buf, len, 1000);
}

static void c6dofimu9_spi_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    uint8_t tx_buf[ 265 ];
    uint8_t cnt;

    tx_buf[ 0 ] = reg;
    for ( cnt = 1; cnt <= len; cnt++ )
    {
        tx_buf[ cnt ] = data_buf[ cnt - 1 ]; 
    }
    
//    spi_master_select_device( ctx->chip_select );
//    spi_master_write( &ctx->spi, tx_buf, len + 1 );
//    spi_master_deselect_device( ctx->chip_select );

    HAL_GPIO_WritePin(ctx->cs, ctx->chip_select.Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&ctx->spi, tx_buf, len + 1, 1000);

    HAL_GPIO_WritePin(ctx->cs, ctx->chip_select.Pin, GPIO_PIN_SET);
}

static void c6dofimu9_spi_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    uint8_t tx_buf[ 1 ];
    uint8_t rx_buf[ 265 ];
    uint8_t cnt;

    tx_buf[ 0 ] = reg | 0x80;
    
//    spi_master_select_device( ctx->chip_select );
//    spi_master_write_then_read( &ctx->spi, tx_buf, 1, rx_buf, len + 1 );
//    spi_master_deselect_device( ctx->chip_select );

    HAL_GPIO_WritePin(ctx->cs, ctx->chip_select.Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&ctx->spi, tx_buf, 1, 1000);
    HAL_SPI_Receive(&ctx->spi, rx_buf, len + 1, 1000);

    HAL_GPIO_WritePin(ctx->cs, ctx->chip_select.Pin, GPIO_PIN_SET);

    for ( cnt = 0; cnt < len; cnt++ )
    {
        data_buf[ cnt ] = rx_buf [ cnt + 1];
    }
}

static void c6dofimu9_irq_handler( void* context )
{
    uint8_t temp = c6dofimu9_get_interrupt_status ( &c6dofimu9 );
    if (temp & (C6DOFIMU9_BIT_WOM_X_INT|C6DOFIMU9_BIT_WOM_Y_INT|C6DOFIMU9_BIT_WOM_Z_INT))
    {
        /* WOM */
    }
    if (temp & C6DOFIMU9_BIT_FIFO_OVF_INT)
    {
        /* FIFO OVF */
    }
    if (temp & C6DOFIMU9_BIT_GDRIVE_INT)
    {
        /* GDIVE */
    }
    if (temp & C6DOFIMU9_BIT_DRY_INT)
    {
        /* DRY */
    }
}

static void inv_show_saved_setting(c6dofimu9_t *st)
{
	// pr_debug(" REG_INT_ENABLE      : 0x%02X\n", saved_regs.int_enable);
	// pr_debug(" REG_FIFO_EN         : 0x%02X\n", saved_regs.fifo_en);
	// pr_debug(" REG_USER_CTRL       : 0x%02X\n", saved_regs.user_ctrl);
	// pr_debug(" REG_CONFIG          : 0x%02X\n", saved_regs.config);
	// pr_debug(" REG_GYRO_CONFIG     : 0x%02X\n", saved_regs.gyro_config);
	// pr_debug(" REG_ACCEL_CONFIG    : 0x%02X\n", saved_regs.accel_config);
	// pr_debug(" REG_ACCEL_CONFIG_2  : 0x%02X\n", saved_regs.accel_config_2);
	// pr_debug(" REG_SAMPLE_RATE_DIV : 0x%02X\n", saved_regs.smplrt_div);
	// pr_debug(" REG_LP_MODE_CTRL    : 0x%02X\n", saved_regs.lp_mode);
	// pr_debug(" REG_PWR_MGMT_1      : 0x%02X\n", saved_regs.pwr_mgmt_1);
	// pr_debug(" REG_PWR_MGMT_2      : 0x%02X\n", saved_regs.pwr_mgmt_2);
}

static int inv_save_setting(c6dofimu9_t *ctx)
{
	int result;

	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_PWR_MGMT_1,
			&saved_regs.pwr_mgmt_1, 1);
	if (result)
		return result;

	/* wake up */
    saved_regs.pwr_mgmt_1 &= ~C6DOFIMU9_BIT_SLEEP;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_1,
			&saved_regs.pwr_mgmt_1, 1);
	if (result)
		return result;

	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_INT_ENABLE,
			&saved_regs.int_enable, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_FIFO_EN,
			&saved_regs.fifo_en, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_USER_CTRL,
			&saved_regs.user_ctrl, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_CONFIG,
			&saved_regs.config, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_GYRO_CONFIG,
			&saved_regs.gyro_config, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_ACCEL_CONFIG,
			&saved_regs.accel_config, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_ACCEL_CONFIG_2,
			&saved_regs.accel_config_2, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_SMPLRT_DIV,
			&saved_regs.smplrt_div, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_LP_MODE_CFG,
			&saved_regs.lp_mode, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_PWR_MGMT_2,
			&saved_regs.pwr_mgmt_2, 1);
	if (result)
		return result;

	inv_show_saved_setting(ctx);

	return result;
}

static int inv_recover_setting( c6dofimu9_t *ctx)
{
	int result;
	/* Stop sensors */
    uint8_t pwr_mgmt_2 = C6DOFIMU9_BIT_PWR_ACCEL_STBY | C6DOFIMU9_BIT_PWR_GYRO_STBY;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_2,
			&pwr_mgmt_2, 1);
	if (result)
		return result;

	/* Restore sensor configurations */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_INT_ENABLE,
			&saved_regs.int_enable, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_FIFO_EN,
			&saved_regs.fifo_en, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL,
			&saved_regs.user_ctrl, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_CONFIG,
			&saved_regs.config, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_GYRO_CONFIG,
			&saved_regs.gyro_config, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_ACCEL_CONFIG,
			&saved_regs.accel_config, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_ACCEL_CONFIG_2,
			&saved_regs.accel_config_2, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_SMPLRT_DIV,
			&saved_regs.smplrt_div, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_LP_MODE_CFG,
			&saved_regs.lp_mode, 1);
	if (result)
		return result;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_1,
			&saved_regs.pwr_mgmt_1, 1);
	if (result)
		return result;

	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_2,
			&saved_regs.pwr_mgmt_2, 1);
	if (result)
		return result;

	return result;
}

static int inv_switch_engine(c6dofimu9_t *ctx, bool en, uint32_t mask)
{
	uint8_t data, mgmt_1;
	int result;

	if (mask == C6DOFIMU9_BIT_PWR_GYRO_STBY) {
		result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_PWR_MGMT_1, &mgmt_1, 1);
		if (result)
			return result;
		mgmt_1 &= ~C6DOFIMU9_BIT_CLK_MASK;
	}

	if ((mask == C6DOFIMU9_BIT_PWR_GYRO_STBY) && (!en)) {
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_1, &mgmt_1, 1);
		if (result)
			return result;
	}

	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_PWR_MGMT_2, &data, 1);
	if (result)
		return result;
	if (en)
		data &= (~mask);
	else
		data |= mask;
	data |= C6DOFIMU9_BIT_FIFO_LP_EN;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_2, &data, 1);
	if (result)
		return result;

	if ((mask == C6DOFIMU9_BIT_PWR_GYRO_STBY) && en) {
		/* only gyro on needs sensor up time */
		DelayMs(SENSOR_UP_TIME);
		/* after gyro is on & stable, switch internal clock to PLL */
		mgmt_1 |= C6DOFIMU9_BIT_CLK_PLL;
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_1, &mgmt_1, 1);
		if (result)
			return result;
	}
	if ((mask == C6DOFIMU9_BIT_PWR_ACCEL_STBY) && en)
		DelayMs(REG_UP_TIME);

	return 0;
}

/**
 * inv_check_gyro_self_test() - check gyro self test. this function
 *                                   returns zero as success. A non-zero return
 *                                   value indicates failure in self test.
 *  @*st: main data structure.
 *  @*reg_avg: average value of normal test.
 *  @*st_avg:  average value of self test
 */
static int inv_check_gyro_self_test(c6dofimu9_t *ctx,
						int *reg_avg, int *st_avg)
{
	uint8_t regs[3];
	int ret_val, result;
	int otp_value_zero = 0;
	int st_shift_prod[3], st_shift_cust[3], i;

	ret_val = 0;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_20680_XG_ST_DATA, regs, 3);
	if (result)
		return result;
	// pr_debug("%s self_test gyro shift_code - %02x %02x %02x\n",
	// 	st->hw->name, regs[0], regs[1], regs[2]);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = mpu_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	// pr_debug("%s self_test gyro st_shift_prod - %+d %+d %+d\n",
	// 	st->hw->name, st_shift_prod[0], st_shift_prod[1],
	// 	st_shift_prod[2]);

	for (i = 0; i < 3; i++) {
		st_shift_cust[i] = st_avg[i] - reg_avg[i];
		if (!otp_value_zero) {
			/* Self Test Pass/Fail Criteria A */
			if (st_shift_cust[i] <= DEF_GYRO_CT_SHIFT_DELTA *
					st_shift_prod[i])
				ret_val = 1;
		} else {
			/* Self Test Pass/Fail Criteria B */
			if (fabs(st_shift_cust[i]) < DEF_GYRO_ST_AL *
						DEF_SELFTEST_GYRO_SENS *
						DEF_ST_PRECISION)
				ret_val = 1;
		}
	}
	// pr_debug("%s self_test gyro st_shift_cust - %+d %+d %+d\n",
	// 	st->hw->name, st_shift_cust[0], st_shift_cust[1],
	// 	st_shift_cust[2]);

	if (ret_val == 0) {
		/* Self Test Pass/Fail Criteria C */
		for (i = 0; i < 3; i++)
			if (fabs(reg_avg[i]) > DEF_GYRO_OFFSET_MAX *
						DEF_SELFTEST_GYRO_SENS *
						DEF_ST_PRECISION)
				ret_val = 1;
	}

	return ret_val;
}

/**
 * inv_check_accel_self_test() - check 20680 accel self test. this function
 *                                   returns zero as success. A non-zero return
 *                                   value indicates failure in self test.
 *  @*st: main data structure.
 *  @*reg_avg: average value of normal test.
 *  @*st_avg:  average value of self test
 */
static int inv_check_accel_self_test(c6dofimu9_t *ctx,
						int *reg_avg, int *st_avg)
{
	int ret_val, result;
	int st_shift_prod[3], st_shift_cust[3], i;
	uint8_t regs[3];
	int otp_value_zero = 0;

	ret_val = 0;
	result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_20680_XA_ST_DATA, regs, 3);
	if (result)
		return result;
	// pr_debug("%s self_test accel shift_code - %02x %02x %02x\n",
	// 	st->hw->name, regs[0], regs[1], regs[2]);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = mpu_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	// pr_debug("%s self_test accel st_shift_prod - %+d %+d %+d\n",
	// 	st->hw->name, st_shift_prod[0], st_shift_prod[1],
	// 	st_shift_prod[2]);

	for (i = 0; i < 3; i++) {
		st_shift_cust[i] = st_avg[i] - reg_avg[i];
		if (!otp_value_zero) {
			/* Self Test Pass/Fail Criteria A */
			if (st_shift_cust[i] <= DEF_ACCEL_ST_SHIFT_DELTA_MIN * st_shift_prod[i] ||
					st_shift_cust[i] >= DEF_ACCEL_ST_SHIFT_DELTA_MAX * st_shift_prod[i])
				ret_val = 1;
		} else {
			/* Self Test Pass/Fail Criteria B */
			if (fabs(st_shift_cust[i]) < ACCEL_ST_AL_MIN ||
					fabs(st_shift_cust[i]) > ACCEL_ST_AL_MAX)
				ret_val = 1;
		}
	}
	// pr_debug("%s self_test accel st_shift_cust - %+d %+d %+d\n",
	// 	st->hw->name, st_shift_cust[0], st_shift_cust[1],
	// 	st_shift_cust[2]);

	return ret_val;
}

/*
 *  inv_do_test() - do the actual test of self testing
 */
static int inv_do_test(c6dofimu9_t *ctx, int self_test_flag,
		int *gyro_result, int *accel_result, int lp_mode)
{
	int result, i, j, packet_size;
	uint8_t data[BYTES_PER_SENSOR * 2], d, dd;
	int fifo_count, packet_count, ind, s;
    uint8_t reset = 0;
	packet_size = BYTES_PER_SENSOR * 2;

	/* disable interrupt */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_INT_ENABLE, &reset, 1);
	if (result)
		return result;
	/* disable the sensor output to FIFO */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_FIFO_EN, &reset, 1);
	if (result)
		return result;
	/* disable fifo reading */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &reset, 1);
	if (result)
		return result;
	/* clear FIFO */
    d = C6DOFIMU9_BIT_FIFO_RST;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &d, 1);
	if (result)
		return result;
	/* setup parameters */
    d = INV_FILTER_98HZ;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_CONFIG, &d, 1);
	if (result)
		return result;

	/* gyro lp mode */
	if (lp_mode == 1)
		d = C6DOFIMU9_BIT_GYRO_CYCLE_EN;
	else if (lp_mode == 2)
		d = DEF_SELFTEST_SAMPLE_RATE_ACC_LP;
	else
		d = 0;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_LP_MODE_CFG, &d, 1);
	if (result)
		return result;

	/* config accel LPF register */
	if (lp_mode == 2)
		d = C6DOFIMU9_BIT_ACCEL_FCHOCIE_B;
	else
		d = DEF_ST_IAM20680_ACCEL_LPF;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_20680_ACCEL_CONFIG2, &d, 1);
	if (result)
		return result;

	if (lp_mode) {
        d = DEF_SELFTEST_SAMPLE_RATE_LP;
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_SMPLRT_DIV,
				&d, 1);
	} else {
        d = DEF_SELFTEST_SAMPLE_RATE;
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_SMPLRT_DIV,
				&d, 1);
	}
	if (result)
		return result;
	/* wait for the sampling rate change to stabilize */
	DelayMs(SAMPLE_RATE_CHANGE_STABLE);
    d = self_test_flag | DEF_SELFTEST_GYRO_FS;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_GYRO_CONFIG,
		&d, 1);
	if (result)
		return result;

	d = DEF_SELFTEST_20680_ACCEL_FS;
	d |= self_test_flag;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_ACCEL_CONFIG, &d, 1);
	if (result)
		return result;

	/* wait for the output to get stable */
	DelayMs(DEF_ST_20680_STABLE_TIME);

	/* enable FIFO reading */
    d = C6DOFIMU9_BIT_FIFO_EN;
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &d, 1);
	if (result)
		return result;
	/* enable sensor output to FIFO */
	d = BITS_GYRO_OUT | BIT_ACCEL_OUT;
	for (i = 0; i < THREE_AXIS; i++) {
		gyro_result[i] = 0;
		accel_result[i] = 0;
	}
	s = 0;
	while (s < 200 /*st->self_test.samples*/) {
		/* Stop FIFO */
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &reset, 1);
		if (result)
			return result;
		/* clear FIFO */
        dd = C6DOFIMU9_BIT_FIFO_RST;
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &dd, 1);
		if (result)
			return result;
		/* enable FIFO reading */
        dd = C6DOFIMU9_BIT_FIFO_EN;
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_USER_CTRL, &dd, 1);
		if (result)
			return result;

		/* accel lp mode */
		dd = C6DOFIMU9_BIT_CLK_PLL;
		if (lp_mode == 2)
			dd |= C6DOFIMU9_BIT_LP_EN;
		else
			dd &= ~C6DOFIMU9_BIT_LP_EN;
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_1, &dd, 1);
		if (result)
			return result;
		
		dd = C6DOFIMU9_BIT_GYRO_FIFO_EN	| C6DOFIMU9_BIT_ACCEL_FIFO_EN;
		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_FIFO_EN, &dd, 1);

		if (result)
			return result;
		if (lp_mode)
			DelayMs(DEF_GYRO_WAIT_TIME_LP);
		else
			DelayMs(DEF_GYRO_WAIT_TIME);

		result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_FIFO_EN, &reset, 1);
		if (result)
			return result;

		result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_FIFO_COUNTH,
					data, FIFO_COUNT_BYTE);
		if (result)
			return result;
		// fifo_count = be16_to_cpup((__be16 *)(&data[0]));
        fifo_count = data[0] << 8 | data[1];
		// pr_debug("%s self_test fifo_count - %d\n",
		// 	 st->hw->name, fifo_count);
		packet_count = fifo_count / packet_size;
		i = 0;
		while ((i < packet_count) && (s < 200)) {
			short vals[3];

			result = c6dofimu9_generic_read(ctx, C6DOFIMU9_REG_FIFO_R_W,
				 data, packet_size);
			if (result)
				return result;
			ind = 0;

			for (j = 0; j < THREE_AXIS; j++) {
				// vals[j] = (short) be16_to_cpup(
				// 	(__be16 *)(&data[ind + 2 * j]));
                vals[j] = data[ind + 2 * j] << 8 | data[(ind + 2 * j) + 1];
				accel_result[j] += vals[j];
			}
			ind += BYTES_PER_SENSOR;
			// pr_debug(
			// 	"%s self_test accel data - %d %+d %+d %+d\n",
			// 	st->hw->name, s, vals[0], vals[1], vals[2]);

			for (j = 0; j < THREE_AXIS; j++) {
				// vals[j] = (short)be16_to_cpup(
				// 	(__be16 *)(&data[ind + 2 * j]));
                vals[j] = data[ind + 2 * j] << 8 | data[(ind + 2 * j) + 1];
				gyro_result[j] += vals[j];
			}
			// pr_debug("%s self_test gyro data - %d %+d %+d %+d\n",
			// 	st->hw->name, s, vals[0], vals[1], vals[2]);

			s++;
			i++;
		}
	}

	for (j = 0; j < THREE_AXIS; j++) {
		accel_result[j] = accel_result[j] / s;
		accel_result[j] *= DEF_ST_PRECISION;
	}
	for (j = 0; j < THREE_AXIS; j++) {
		gyro_result[j] = gyro_result[j] / s;
		gyro_result[j] *= DEF_ST_PRECISION;
	}

	return 0;
}

int inv_set_accel_config2(c6dofimu9_t *ctx, bool cycle_mode)
{
	int cycle_freq[] = {275, 192, 111, 59};
	int cont_freq[] = {219, 219, 99, 45, 22, 11, 6};
	int r, rate;
	uint8_t i, v, d;

	v = 0;
#ifdef CONFIG_INV_MPU_IIO_ICM20690
	v |= BIT_FIFO_SIZE_1K;
#endif
	if (cycle_mode) {
		rate = (BASE_SAMPLE_RATE << 1);
		i = NELEM(cycle_freq) - 1;
		while (i > 0) {
			if (rate < cycle_freq[i])
				break;
			i--;
		}
        d = v | (i << 4) | 7;
		r = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_ACCEL_CONFIG_2, 
								&d, 1);
		if (r)
			return r;
		ctx->accel_lp_mode = 1;
	} else {
		rate = (BASE_SAMPLE_RATE >> 1);
		for (i = 1; i < NELEM(cont_freq); i++) {
			if (rate >= cont_freq[i])
				break;
		}
		if (i > 6)
			i = 6;
        d = v | i;
		r = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_ACCEL_CONFIG_2, &d, 1);
		if (r)
			return r;
		ctx->accel_lp_mode = 0;
	}

	return 0;
}

static int inv_lp_en_off_mode(c6dofimu9_t *ctx, bool on)
{
    int r = 0;
    uint8_t d;

	if (!ctx->is_asleep)
		return 0;

// #if !defined(CONFIG_INV_MPU_IIO_ICM42600) && !defined(CONFIG_INV_MPU_IIO_ICM43600) && !defined(CONFIG_INV_MPU_IIO_ICM45600)
    d = C6DOFIMU9_BIT_CLK_PLL;
	r = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_PWR_MGMT_1, &d, 1);
	// usleep_range(REG_UP_TIME_USEC, REG_UP_TIME_USEC + 1);
// #endif
	ctx->is_asleep = 0;
// #if defined(CONFIG_INV_MPU_IIO_ICM20602) || \
// 	defined(CONFIG_INV_MPU_IIO_ICM20690) || \
// 	defined(CONFIG_INV_MPU_IIO_IAM20680) || \
// 	defined(CONFIG_INV_MPU_IIO_ICM20608D)
	inv_set_accel_config2(ctx, false);
// #endif
	return r;
}

static int inv_power_up_self_test(c6dofimu9_t *ctx)
{
	int result;
    uint8_t d = 0;
	result = inv_lp_en_off_mode(ctx, true);

	/* make sure no interrupts */
	result = c6dofimu9_generic_write(ctx, C6DOFIMU9_REG_INT_ENABLE, &d, 1);
	if (result)
		return result;

	if (result)
		return result;
	result = inv_switch_engine(ctx, true, C6DOFIMU9_BIT_PWR_ACCEL_STBY);
	if (result)
		return result;
	result = inv_switch_engine(ctx, true, C6DOFIMU9_BIT_PWR_GYRO_STBY);
	if (result)
		return result;

	return 0;
}

/*
 *  inv_hw_self_test() - main function to do hardware self test
 */
int c6dofimu9_hw_selftest ( c6dofimu9_t *ctx )
{
int result;
	int gyro_bias_st[THREE_AXIS], gyro_bias_regular[THREE_AXIS];
	int accel_bias_st[THREE_AXIS], accel_bias_regular[THREE_AXIS];
#ifdef ENABLE_GYRO_HW_SELFTEST
	int gyro_bias_regular_lp[THREE_AXIS];
	int accel_bias_regular_lp[THREE_AXIS];
	int dummy_bias_regular[THREE_AXIS];
#endif
	int test_times, i;
	char accel_result, gyro_result;

	result = inv_save_setting(ctx);
	if (result)
		return result;

	result = inv_power_up_self_test(ctx);
	if (result)
		return result;
	accel_result = 0;
	gyro_result = 0;
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(ctx, 0, gyro_bias_regular,
			accel_bias_regular, 0);
		if (result == -1)
			test_times--;
		else
			test_times = 0;
	}
	if (result)
		goto test_fail;
	// pr_debug("%s self_test accel bias_regular - %+d %+d %+d\n",
	// 	st->hw->name, accel_bias_regular[0],
	// 	accel_bias_regular[1], accel_bias_regular[2]);
	// pr_debug("%s self_test gyro bias_regular - %+d %+d %+d\n",
	// 	st->hw->name, gyro_bias_regular[0], gyro_bias_regular[1],
	// 	gyro_bias_regular[2]);

	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(ctx, C6DOFIMU9_BITS_SELF_TEST_EN, gyro_bias_st,
					accel_bias_st, 0);
		if (result == -1)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;
	// pr_debug("%s self_test accel bias_st - %+d %+d %+d\n",
	// 	st->hw->name, accel_bias_st[0], accel_bias_st[1],
	// 	accel_bias_st[2]);
	// pr_debug("%s self_test gyro bias_st - %+d %+d %+d\n",
	// 	st->hw->name, gyro_bias_st[0], gyro_bias_st[1],
	// 	gyro_bias_st[2]);

#ifdef ENABLE_GYRO_HW_SELFTEST
	/* lp gyro mode */
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(ctx, 0, gyro_bias_regular_lp,
			dummy_bias_regular, 1);
		if (result == -1)
			test_times--;
		else
			test_times = 0;
	}
	if (result)
		goto test_fail;
	// pr_debug("%s self_test gyro bias_regular lp - %+d %+d %+d\n",
	// 	 st->hw->name, gyro_bias_regular_lp[0], gyro_bias_regular_lp[1],
	// 	 gyro_bias_regular_lp[2]);

	/* lp accel mode */
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_do_test(ctx, 0, dummy_bias_regular,
			accel_bias_regular_lp, 2);
		if (result == -1)
			test_times--;
		else
			test_times = 0;
	}
	if (result)
		goto test_fail;
	// pr_debug("%s self_test accel bias_regular lp - %+d %+d %+d\n",
	// 	 st->hw->name, accel_bias_regular_lp[0],
	// 	 accel_bias_regular_lp[1], accel_bias_regular_lp[2]);
#endif

	/* copy bias */
	for (i = 0; i < 3; i++) {
		/* gyro : LN bias as LN is default mode */
		ctx->gyro_st_bias[i] = gyro_bias_regular[i] / DEF_ST_PRECISION;
		/* accel : LN bias as LN is default mode */
		ctx->accel_st_bias[i] = accel_bias_regular[i] / DEF_ST_PRECISION;
	}

	/* Check is done on continuous mode data */
	accel_result = !inv_check_accel_self_test(ctx,
		accel_bias_regular, accel_bias_st);
	gyro_result = !inv_check_gyro_self_test(ctx,
		gyro_bias_regular, gyro_bias_st);

test_fail:
	inv_recover_setting(ctx);
	return (accel_result << DEF_ST_ACCEL_RESULT_SHIFT) | gyro_result;
}

static void ekf_local_init(ekf_t * ekf)
{
    // Set Q, see [1]
    const double Sf    = 0.00001;
    const double Sg    = 0.00001;
//    const double sigma = 0.001;         // state transition variance
//    const double Qb[4] = {Sg*T}; //{Sf*T+Sg*T*T*T/3, Sg*T*T/2, Sg*T*T/2, Sg*T};
//    const double Qxyz[4] = {sigma*sigma*T}; //{sigma*sigma*T*T*T/3, sigma*sigma*T*T/2, sigma*sigma*T*T/2, sigma*sigma*T};

//    ekf_blkfill(ekf, Qxyz, 0);
//    ekf_blkfill(ekf, Qxyz, 1);
//    ekf_blkfill(ekf, Qxyz, 2);
//    ekf_blkfill(ekf, Qb,   3);


    // initial covariances of state noise, measurement noise
    double P0 = 0.00001;
    double R0 = Sf;

    int i;

    for (i=0; i<Nsta; ++i)
        ekf->Q[i][i] = Sg;

    for (i=0; i<Nsta; ++i)
        ekf->P[i][i] = P0;

    for (i=0; i<Mobs; ++i)
        ekf->R[i][i] = R0;

    // position euler angle
    ekf->x[0] = 0;
    ekf->x[1] = 0;
    ekf->x[2] = 0;

    // euler angle //velocity
//    ekf->x[4] = 0;
//    ekf->x[5] = 0;
//    ekf->x[6] = 0;
//    ekf->x[7] = 0;

//    // clock bias
//    ekf->x[6] = 3.575261153706439e+006;
//
//    // clock drift
//    ekf->x[7] = 4.549246345845814e+001;
}

static void ekf_model(ekf_t * ekf)
{
    int i, j;
//
	 // Process model is f(x) = x
	for (i = 0; i < Nsta; ++i) {
		ekf->fx[i] = ekf->x[i];
	}
//    for (j=0; j<4; j+=2) {
//        ekf->fx[j] = ekf->x[j] + T * ekf->x[j+1];
//        ekf->fx[j+1] = ekf->x[j+1];
//    }
	//// So process model Jacobian is identity matrix
//		F[0][0] = 1;
//		F[1][1] = 1;
    for (j=0; j<Nsta; ++j)
        ekf->F[j][j] = 1;
//
//    for (j=0; j<4; ++j)
//        ekf->F[2*j][2*j+1] = T;
//
//    double dx[Mobs][Nsta];
//
	// Measurement function simplifies the relationship between state and sensor readings for convenience.
		// A more realistic measurement function would distinguish between state value and measured value; e.g.:
		//   hx[0] = pow(this->x[0], 1.03);
		//   hx[1] = 1.005 * this->x[1];
		//   hx[2] = .9987 * this->x[1] + .001;

//	hx[0] = this->x[0]; // Barometric pressure from previous state
//	hx[1] = this->x[1]; // Baro temperature from previous state
//	hx[2] = this->x[1]; // LM35 temperature from previous state
    ekf->hx[0] = ekf->x[0]; // GX from previous state
    ekf->hx[1] = ekf->x[1]; // GY from previous state
    ekf->hx[2] = ekf->x[2]; // GZ from previous state
//    ekf->hx[3] = ekf->x[3]; // Temperature from previous state

//    for (i = 0; i < Mobs; ++i) {
//    	ekf->hx[i] = 0;
//		for (j = 0; j < Nsta; ++j) {
//			double d = ekf->fx[j];
//			dx[i][j] = d;
//			ekf->hx[i] += d * d;
//		}
//		ekf->hx[i] = pow(ekf->hx[i], 0.5);
//	}
//    for (i=0; i<4; ++i) {
//        ekf->hx[i] = 0;
//        for (j=0; j<4; ++j) {
//            double d = ekf->fx[j*2] - SV[i][j];
//            dx[i][j] = d;
//            ekf->hx[i] += d*d;
//        }
//        ekf->hx[i] = pow(ekf->hx[i], 0.5) + ekf->fx[6];
//    }

	// Jacobian of measurement function
//	H[0][0] = 1;        // Barometric pressure from previous state
//	H[1][1] = 1;       // Baro temperature from previous state
//	H[2][1] = 1;       // LM35 temperature from previous state
    ekf->H[0][0] = 1;
    ekf->H[1][1] = 1;
    ekf->H[2][2] = 1;
//    ekf->H[3][0] = 1;
//    ekf->H[3][1] = 1;
//    ekf->H[3][2] = 1;
    //	for (i = 0; i < Mobs; ++i) {
//		for (j = 0; j < Nsta; ++j)
//			ekf->H[i][j] = dx[i][j] / ekf->hx[i];
////		ekf->H[i][3] = 1;
//	}
//    for (i=0; i<4; ++i) {
//        for (j=0; j<4; ++j)
//            ekf->H[i][j*2]  = dx[i][j] / ekf->hx[i];
//        ekf->H[i][6] = 1;
//    }
}

Quaternion ToQuaternion(EulerAngles e)
{

	Quaternion q;
	// Convert degrees to radians
	e.roll = e.roll * M_PI / 180.0;
	e.pitch = e.pitch * M_PI / 180.0;
	e.yaw = e.yaw * M_PI / 180.0;

	// Calculate trigonometric values
	float cy = cos(e.yaw * 0.5);
	float sy = sin(e.yaw * 0.5);
	float cp = cos(e.pitch * 0.5);
	float sp = sin(e.pitch * 0.5);
	float cr = cos(e.roll * 0.5);
	float sr = sin(e.roll * 0.5);

	// Calculate quaternion values
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;
	return q;
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
EulerAngles ToEulerAngles(Quaternion q)
{
	EulerAngles angles;

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
	angles.roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
	double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
	angles.pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
	angles.yaw = atan2(siny_cosp, cosy_cosp);

	return angles;
}

// static int inv_process_20680_data(c6dofimu9_t *st)
// {
	
// 	int total_bytes, tmp, res, fifo_count, pk_size, i;
// 	uint8_t *dptr, *d;
// 	uint8_t data[14];
// 	bool done_flag;
// 	uint8_t v;
// #ifdef SENSOR_DATA_FROM_REGISTERS
// 	u8 reg;
// 	int len;
// #endif

// 	if (ctx->gesture_only_on && (!st->batch.timeout)) {
// 		res = inv_plat_read(ctx, REG_INT_STATUS, 1, data);
// 		if (res)
// 			return res;
// 		pr_debug("ges cnt=%d, statu=%x\n",
// 						st->gesture_int_count, data[0]);
// 		if (data[0] & (BIT_WOM_ALL_INT_EN)) {
// 			if (!st->gesture_int_count) {
// 				inv_switch_power_in_lp(ctx, true);
// 				res = inv_plat_single_write(ctx, REG_INT_ENABLE,
// 					BIT_WOM_ALL_INT_EN | BIT_DATA_RDY_EN);
// 				if (res)
// 					return res;
// 				v = 0;
// 				// if (ctx->chip_config.gyro_enable)
// 					v |= BITS_GYRO_FIFO_EN;

// 				// if (ctx->chip_config.accel_enable)
// 					v |= BIT_ACCEL_FIFO_EN;
// 				res = inv_plat_single_write(ctx, REG_FIFO_EN, v);
// 				if (res)
// 					return res;
// 				/* First time wake up from WOM.
// 				 * We don't need data in the FIFO
// 				 */
// 				res = inv_reset_fifo(ctx, true);
// 				if (res)
// 					return res;
// 				res = inv_switch_power_in_lp(ctx, false);
// 				st->gesture_int_count = WOM_DELAY_THRESHOLD;

// 				return res;
// 			}
// 			st->gesture_int_count = WOM_DELAY_THRESHOLD;
// 		} else {
// 			if (!st->gesture_int_count) {
// 				inv_switch_power_in_lp(ctx, true);
// 				res = inv_plat_single_write(ctx, REG_FIFO_EN, 0);
// 				res = inv_plat_single_write(ctx, REG_INT_ENABLE,
// 					BIT_WOM_ALL_INT_EN);
// 				inv_switch_power_in_lp(ctx, false);

// 				return res;
// 			}
// 			st->gesture_int_count--;
// 		}
// 	}

// 	fifo_count = inv_get_last_run_time_non_dmp_record_mode(ctx);
// 	pr_debug("fifc= %d\n", fifo_count);
// 	if (!fifo_count) {
// 		pr_debug("REG_FIFO_COUNT_H size is 0\n");
// 		return 0;
// 	}
// 	pk_size = st->batch.pk_size;
// 	if (!pk_size)
// 		return -EINVAL;

// 	if (fifo_count >= (HARDWARE_FIFO_SIZE / st->batch.pk_size)) {
// 		pr_warn("fifo overflow pkt count=%d pkt sz=%d\n",
// 				fifo_count, st->batch.pk_size);
// 		return -EOVERFLOW;
// 	}

// 	fifo_count *= st->batch.pk_size;
// 	st->fifo_count = fifo_count;
// 	d = st->fifo_data_store;
// 	dptr = d;
// 	total_bytes = fifo_count;

// #ifdef SENSOR_DATA_FROM_REGISTERS
// 	len = 0;
// 	if (ctx->sensor[SENSOR_GYRO].on) {
// 		reg = REG_RAW_GYRO;
// 		len += BYTES_PER_SENSOR;
// 		if (ctx->sensor[SENSOR_ACCEL].on && !st->sensor[SENSOR_TEMP].on)
// 			len += BYTES_FOR_TEMP;
// 	}
// 	if (ctx->sensor[SENSOR_TEMP].on) {
// 		reg = REG_RAW_TEMP;
// 		len += BYTES_FOR_TEMP;
// 	}
// 	if (ctx->sensor[SENSOR_ACCEL].on) {
// 		reg = REG_RAW_ACCEL;
// 		len += BYTES_PER_SENSOR;
// 	}

// 	if (len == 0) {
// 		pr_debug("No sensor is enabled\n");
// 		return 0;
// 	}

// 	/* read data registers */
// 	res = inv_plat_read(ctx, reg, len, data);
// 	if (res < 0) {
// 		pr_err("read data registers is failed\n");
// 		return res;
// 	}

// 	/* copy sensor data to buffer as FIFO data format */
// 	tmp = 0;
// 	if (ctx->sensor[SENSOR_ACCEL].on) {
// 		for (i = 0; i < BYTES_PER_SENSOR; i++)
// 			dptr[i] = data[tmp + i];
// 		dptr += BYTES_PER_SENSOR;
// 		tmp += BYTES_PER_SENSOR;
// 	}

// 	if (ctx->sensor[SENSOR_TEMP].on) {
// 		for (i = 0; i < BYTES_FOR_TEMP; i++)
// 			dptr[i] = data[tmp + i];
// 		dptr += BYTES_FOR_TEMP;
// 		tmp += BYTES_FOR_TEMP;
// 	}

// 	if (ctx->sensor[SENSOR_GYRO].on) {
// 		if (ctx->sensor[SENSOR_ACCEL].on && !st->sensor[SENSOR_TEMP].on)
// 			tmp += BYTES_FOR_TEMP;
// 		for (i = 0; i < BYTES_PER_SENSOR; i++)
// 			dptr[i] = data[tmp + i];
// 	}
// #else
// 	while (total_bytes > 0) {
// 		if (total_bytes < pk_size * MAX_FIFO_PACKET_READ)
// 			tmp = total_bytes;
// 		else
// 			tmp = pk_size * MAX_FIFO_PACKET_READ;
// 		res = inv_plat_read(ctx, REG_FIFO_R_W, tmp, dptr);
// 		if (res < 0) {
// 			pr_err("read REG_FIFO_R_W is failed\n");
// 			return res;
// 		}
// 		pr_debug("inside: %x, %x, %x, %x, %x, %x, %x, %x\n",
// 				dptr[0], dptr[1], dptr[2], dptr[3],
// 				dptr[4], dptr[5], dptr[6], dptr[7]);
// 		pr_debug("insid2: %x, %x, %x, %x, %x, %x, %x, %x\n",
// 				dptr[8], dptr[9], dptr[10], dptr[11],
// 				dptr[12], dptr[13], dptr[14], dptr[15]);

// 		dptr += tmp;
// 		total_bytes -= tmp;
// 	}
// #endif /* SENSOR_DATA_FROM_REGISTERS */
// 	dptr = d;
// 	pr_debug("dd: %x, %x, %x, %x, %x, %x, %x, %x\n",
// 			d[0], d[1], d[2], d[3],
// 			d[4], d[5], d[6], d[7]);
// 	pr_debug("dd2: %x, %x, %x, %x, %x, %x, %x, %x\n",
// 			d[8], d[9], d[10], d[11],
// 			d[12], d[13], d[14], d[15]);
// 	total_bytes = fifo_count;

// 	for (i = 0; i < SENSOR_NUM_MAX; i++) {
// 		if (ctx->sensor[i].on)
// 			st->sensor[i].count =  total_bytes / pk_size;
// 	}
// 	st->header_count = 0;
// 	for (i = 0; i < SENSOR_NUM_MAX; i++) {
// 		if (ctx->sensor[i].on)
// 			st->header_count = max(ctx->header_count,
// 							st->sensor[i].count);
// 	}

// 	st->ts_algo.calib_counter++;
// 	inv_bound_timestamp(ctx);

// 	dptr = d;
// 	done_flag = false;

// 	while (!done_flag) {
// 		pr_debug("total%d, pk=%d\n", total_bytes, pk_size);
// 		if (total_bytes >= pk_size) {
// 			res = inv_push_20680_data(indio_dev, dptr);
// 			if (res)
// 				return res;
// 			total_bytes -= pk_size;
// 			dptr += pk_size;
// 		} else {
// 			done_flag = true;
// 		}
// 	}

// 	return 0;
// }

// ------------------------------------------------------------------------- END

