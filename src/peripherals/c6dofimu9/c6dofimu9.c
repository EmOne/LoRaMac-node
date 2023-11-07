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
#include "i2c.h"
#include "gpio.h"
#include "c6dofimu9.h"

// ------------------------------------------------------------- PRIVATE MACROS 
extern I2c_t I2c;
extern Gpio_t imu_sync;
extern Gpio_t imu_interupt;

static uint8_t I2cDeviceAddr = 0;
static bool IMUInitialized = false;

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

static void c6dofimu9_i2c_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

static void c6dofimu9_i2c_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

static void c6dofimu9_spi_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

static void c6dofimu9_spi_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

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

		ctx->i2c.Instance = I2C1;
		ctx->i2c.Init.ClockSpeed = 100000;
        ctx->i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
        ctx->i2c.Init.OwnAddress1 = 0;
        ctx->i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        ctx->i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        ctx->i2c.Init.OwnAddress2 = 0;
        ctx->i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        ctx->i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        __HAL_RCC_GPIOB_CLK_ENABLE();

		cfg->scl.Pin = SCL_PIN;
		cfg->scl.Mode = GPIO_MODE_AF_OD;
		cfg->scl.Alternate = GPIO_AF4_I2C1;
		cfg->scl.Pull = GPIO_NOPULL;
		cfg->scl.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(SCL_PORT, &cfg->scl);

		cfg->sda.Pin = SDA_PIN;
		cfg->sda.Mode = GPIO_MODE_AF_OD;
		cfg->sda.Alternate = GPIO_AF4_I2C1;
		cfg->sda.Pull = GPIO_NOPULL;
		cfg->sda.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(SDA_PORT, &cfg->sda);

		__HAL_RCC_I2C1_CLK_ENABLE();

		__HAL_RCC_I2C1_FORCE_RESET();

		__HAL_RCC_I2C1_RELEASE_RESET();

		if (HAL_I2C_Init(&ctx->i2c) != HAL_OK)
//        if (  i2c_master_open( &ctx->i2c, &i2c_cfg ) == I2C_MASTER_ERROR )
        {
            return C6DOFIMU9_INIT_ERROR;
        }

//
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
		HAL_GPIO_Init(ctx->cs, &cfg->cs);
        HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);

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

    	ctx->spi.Instance = SPI2;
    	ctx->spi.Init.Mode = SPI_MODE_MASTER;
    	ctx->spi.Init.Direction = SPI_DIRECTION_2LINES;
    	ctx->spi.Init.DataSize = SPI_DATASIZE_8BIT;
    	ctx->spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    	ctx->spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    	ctx->spi.Init.NSS = SPI_NSS_SOFT;
    	ctx->spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    	ctx->spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    	ctx->spi.Init.TIMode = SPI_TIMODE_DISABLE;
    	ctx->spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    	ctx->spi.Init.CRCPolynomial = 10;

    	ctx->cs = CS_PORT;

		cfg->cs.Pin = CS_PIN;
		cfg->cs.Mode = GPIO_MODE_OUTPUT_PP;
		cfg->cs.Pull = GPIO_NOPULL;
		cfg->cs.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(ctx->cs, &cfg->cs);
//        digital_out_init( &ctx->cs, cfg->cs );
        ctx->chip_select = cfg->cs;
//
		__HAL_RCC_GPIOB_CLK_ENABLE();

		cfg->sck.Pin = SCK_PIN;
		cfg->sck.Mode = GPIO_MODE_AF_PP;
		cfg->sck.Alternate = GPIO_AF5_SPI2;
		cfg->sck.Pull = GPIO_PULLDOWN;
		cfg->sck.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(SCK_PORT, &cfg->sck);

		cfg->miso.Pin = MISO_PIN;
		cfg->miso.Mode = GPIO_MODE_AF_PP;
		cfg->miso.Alternate = GPIO_AF5_SPI2;
		cfg->miso.Pull = GPIO_PULLDOWN;
		cfg->miso.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(MISO_PORT, &cfg->miso);

		cfg->mosi.Pin = MOSI_PIN;
		cfg->mosi.Mode = GPIO_MODE_AF_PP;
		cfg->mosi.Alternate = GPIO_AF5_SPI2;
		cfg->mosi.Pull = GPIO_PULLDOWN;
		cfg->mosi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(MOSI_PORT, &cfg->mosi);

        __HAL_RCC_SPI2_CLK_ENABLE();

//        if (  spi_master_open( &ctx->spi, &spi_cfg ) == SPI_MASTER_ERROR )
        if (  HAL_SPI_Init(&ctx->spi) != HAL_OK)
        {
            return  C6DOFIMU9_INIT_ERROR;
		}

//
//        spi_master_set_default_write_data( &ctx->spi, C6DOFIMU9_DUMMY );
//        spi_master_set_speed( &ctx->spi, spi_cfg.speed );
//        spi_master_set_mode( &ctx->spi, spi_cfg.mode );
//        spi_master_set_chip_select_polarity( cfg->cs_polarity );

    	ctx->read_f = c6dofimu9_spi_read;
        ctx->write_f = c6dofimu9_spi_write;

//        spi_master_select_device( ctx->chip_select );
		ctx->cs = CS_PORT;

		cfg->cs.Pin = CS_PIN;
		cfg->cs.Mode = GPIO_MODE_OUTPUT_PP;
		cfg->cs.Pull = GPIO_PULLUP;
		cfg->cs.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(ctx->cs, &cfg->cs);
		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);
    }
    
    // Output pins 

//    digital_out_init( &ctx->syn, cfg->syn );
    __HAL_RCC_GPIOB_CLK_ENABLE();
    ctx->syn = SYNC_PORT;
	cfg->syn.Pin = SYNC_PIN;
	cfg->syn.Mode = GPIO_MODE_OUTPUT_PP;
	cfg->syn.Pull = GPIO_NOPULL;
	cfg->syn.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ctx->syn, &cfg->syn);

    // Input pins

//    digital_in_init( &ctx->int_pin, cfg->int_pin );
	__HAL_RCC_GPIOA_CLK_ENABLE();
	ctx->int_pin = INT_PORT;
	cfg->int_pin.Pin = INT_PIN;
	cfg->int_pin.Mode = GPIO_MODE_INPUT;
	cfg->int_pin.Pull = GPIO_NOPULL;
	cfg->int_pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ctx->int_pin, &cfg->int_pin);

    return C6DOFIMU9_OK;
}

void c6dofimu9_generic_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    ctx->write_f( ctx, reg, data_buf, len ); 
}

void c6dofimu9_generic_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    ctx->read_f( ctx, reg, data_buf, len );
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

void c6dofimu9_get_accel_data ( c6dofimu9_t *ctx, int16_t *p_accel_x, int16_t *p_accel_y, int16_t *p_accel_z )
{
    *p_accel_x = c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_ACCEL_XOUT_H, C6DOFIMU9_REG_ACCEL_XOUT_L );
    *p_accel_y = c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_ACCEL_YOUT_H, C6DOFIMU9_REG_ACCEL_YOUT_L );
    *p_accel_z = c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_ACCEL_ZOUT_H, C6DOFIMU9_REG_ACCEL_ZOUT_L );
}

void c6dofimu9_get_gyro_data ( c6dofimu9_t *ctx, int16_t *p_gyro_x, int16_t *p_gyro_y, int16_t *p_gyro_z )
{
    *p_gyro_x = c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_GYRO_XOUT_H, C6DOFIMU9_REG_GYRO_XOUT_L );
    *p_gyro_y = c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_GYRO_YOUT_H, C6DOFIMU9_REG_GYRO_YOUT_L );
    *p_gyro_z = c6dofimu9_get_axis( ctx, C6DOFIMU9_REG_GYRO_ZOUT_H, C6DOFIMU9_REG_GYRO_ZOUT_L );
}
// ----------------------------------------------- PRIVATE FUNCTION DEFINITIONS

static void c6dofimu9_i2c_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
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
    HAL_I2C_Mem_Write(&ctx->i2c, (uint16_t) (ctx->slave_address << 1), (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, data_buf, len, 1000);
}

static void c6dofimu9_i2c_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
//    i2c_master_write_then_read( &ctx->i2c, &reg, 1, data_buf, len );
	HAL_I2C_Mem_Read(&ctx->i2c, (uint16_t) (ctx->slave_address << 1), (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, data_buf, len, 1000);
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

// ------------------------------------------------------------------------- END

