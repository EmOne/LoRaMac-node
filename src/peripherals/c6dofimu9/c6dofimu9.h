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
 * \brief This file contains API for 6DOF IMU 9 Click driver.
 *
 * \addtogroup c6dofimu9 6DOF IMU 9 Click Driver
 * @{
 */
// ----------------------------------------------------------------------------

#ifndef C6DOFIMU9_H
#define C6DOFIMU9_H

//#include "drv_digital_out.h"
//#include "drv_digital_in.h"
//#include "drv_i2c_master.h"
//#include "drv_spi_master.h"
#include "stm32l1xx_hal.h"
//#include "motion_sensor.h"
// -------------------------------------------------------------- PUBLIC MACROS 
/**
 * \defgroup macros Macros
 * \{
 */

/**
 * \defgroup map_mikrobus MikroBUS
 * \{
 */

////#define C6DOFIMU9_MAP_MIKROBUS( cfg, mikrobus ) \
//   cfg.scl   = MIKROBUS( mikrobus, MIKROBUS_SCL ); \
//   cfg.sda   = MIKROBUS( mikrobus, MIKROBUS_SDA ); \
//   cfg.miso  = MIKROBUS( mikrobus, MIKROBUS_MISO ); \
//   cfg.mosi  = MIKROBUS( mikrobus, MIKROBUS_MOSI ); \
//   cfg.sck   = MIKROBUS( mikrobus, MIKROBUS_SCK ); \
//   cfg.cs    = MIKROBUS( mikrobus, MIKROBUS_CS ); \
//   cfg.syn = MIKROBUS( mikrobus, MIKROBUS_PWM ); \
//   cfg.int_pin = MIKROBUS( mikrobus, MIKROBUS_INT );
/** \} */
typedef struct Quaternion
{
	double w, x, y, z;
} Quaternion;

typedef struct EulerAngles
{
	double roll, pitch, yaw;
} EulerAngles;

typedef struct Gyroscope
{
	double gy, gx, gz;
} Gyroscope;

/**
 * \defgroup communication Select communication
 * \{
 */
#define C6DOFIMU9_MASTER_I2C 0
#define C6DOFIMU9_MASTER_SPI 1
/** \} */

/**
 * \defgroup error_code Error Code
 * \{
 */
#define C6DOFIMU9_RETVAL  uint8_t


#define C6DOFIMU9_OK           0x00
#define C6DOFIMU9_INIT_ERROR   0xFF
/** \} */

/**
 * \defgroup slave_address Slave Address
 * \{
 */
#define C6DOFIMU9_I2C_SLAVE_ADDRESS_0                               0x68
#define C6DOFIMU9_I2C_SLAVE_ADDRESS_1                               0x69
/** \} */

/**
 * \defgroup spi_command Spi command
 * \{
 */
#define C6DOFIMU9_SPI_WRITE_CMD                                     0x00
#define C6DOFIMU9_SPI_READ_CMD                                      0x80
/** \} */

/**
 * \defgroup Register Map
 * \{
 */
#define C6DOFIMU9_REG_SELF_TEST_X_GYRO                              0x00
#define C6DOFIMU9_REG_SELF_TEST_Y_GYRO                              0x01
#define C6DOFIMU9_REG_SELF_TEST_Z_GYRO                              0x02
#define C6DOFIMU9_REG_SELF_TEST_X_ACCEL                             0x0D
#define C6DOFIMU9_REG_SELF_TEST_Y_ACCEL                             0x0E
#define C6DOFIMU9_REG_SELF_TEST_Z_ACCEL                             0x0F
#define C6DOFIMU9_REG_XG_OFFS_USRH                                  0x13
#define C6DOFIMU9_REG_XG_OFFS_USRL                                  0x14
#define C6DOFIMU9_REG_YG_OFFS_USRH                                  0x15
#define C6DOFIMU9_REG_YG_OFFS_USRL                                  0x16
#define C6DOFIMU9_REG_ZG_OFFS_USRH                                  0x17
#define C6DOFIMU9_REG_ZG_OFFS_USRL                                  0x18
#define C6DOFIMU9_REG_SMPLRT_DIV                                    0x19

#define C6DOFIMU9_REG_CONFIG                                        0x1A
#define C6DOFIMU9_EXT_SYNC_SET		0x08

#define C6DOFIMU9_REG_GYRO_CONFIG                                   0x1B
#define C6DOFIMU9_BITS_SELF_TEST_EN	0xE0
#define C6DOFIMU9_SHIFT_GYRO_FS_SEL	0x03

#define C6DOFIMU9_REG_ACCEL_CONFIG                                  0x1C
#define C6DOFIMU9_SHIFT_ACCEL_FS		0x03

#define C6DOFIMU9_REG_ACCEL_CONFIG_2                                0x1D
#define C6DOFIMU9_BIT_ACCEL_FCHOCIE_B	0x08

#define C6DOFIMU9_REG_LP_MODE_CFG                                   0x1E
#define C6DOFIMU9_BIT_GYRO_CYCLE_EN	0x80

#define C6DOFIMU9_REG_ACCEL_WOM_THR                                 0x1F
#define C6DOFIMU9_REG_ACCEL_WOM_X_THR	0x20
#define C6DOFIMU9_REG_ACCEL_WOM_Y_THR	0x21
#define C6DOFIMU9_REG_ACCEL_WOM_Z_THR	0x22

#define C6DOFIMU9_REG_FIFO_EN                                       0x23
#define C6DOFIMU9_BITS_GYRO_FIFO_EN	0x70
#define C6DOFIMU9_BIT_ACCEL_FIFO_EN	0x08

#define C6DOFIMU9_REG_FSYNC_INT                                     0x36
#define C6DOFIMU9_BIT_FSYNC_INT		0x80

#define C6DOFIMU9_REG_INT_PIN_CFG                                   0x37

#define C6DOFIMU9_REG_INT_ENABLE                                    0x38
#define C6DOFIMU9_BIT_WOM_X_INT_EN	0x80
#define C6DOFIMU9_BIT_WOM_Y_INT_EN	0x40
#define C6DOFIMU9_BIT_WOM_Z_INT_EN	0x20
#define C6DOFIMU9_BIT_WOM_ALL_INT_EN	0xE0
#define C6DOFIMU9_BIT_FSYNC_INT_EN	0x8
#define C6DOFIMU9_BIT_DATA_RDY_EN		0x1

#define C6DOFIMU9_REG_INT_STATUS                                    0x3A
#define C6DOFIMU9_BIT_WOM_X_INT		0x80
#define C6DOFIMU9_BIT_WOM_Y_INT		0x40
#define C6DOFIMU9_BIT_WOM_Z_INT		0x20
#define C6DOFIMU9_BIT_FIFO_OVF_INT	0x10
#define C6DOFIMU9_BIT_GDRIVE_INT	0x04
#define C6DOFIMU9_BIT_DRY_INT   	0x01

#define C6DOFIMU9_REG_ACCEL_XOUT_H                                  0x3B
#define C6DOFIMU9_REG_ACCEL_XOUT_L                                  0x3C
#define C6DOFIMU9_REG_ACCEL_YOUT_H                                  0x3D
#define C6DOFIMU9_REG_ACCEL_YOUT_L                                  0x3E
#define C6DOFIMU9_REG_ACCEL_ZOUT_H                                  0x3F
#define C6DOFIMU9_REG_ACCEL_ZOUT_L                                  0x40
#define C6DOFIMU9_REG_TEMP_OUT_H                                    0x41
#define C6DOFIMU9_REG_TEMP_OUT_L                                    0x42
#define C6DOFIMU9_REG_GYRO_XOUT_H                                   0x43
#define C6DOFIMU9_REG_GYRO_XOUT_L                                   0x44
#define C6DOFIMU9_REG_GYRO_YOUT_H                                   0x45
#define C6DOFIMU9_REG_GYRO_YOUT_L                                   0x46
#define C6DOFIMU9_REG_GYRO_ZOUT_H                                   0x47
#define C6DOFIMU9_REG_GYRO_ZOUT_L                                   0x48
#define C6DOFIMU9_REG_SIGNAL_PATH_RESET                             0x68

#define C6DOFIMU9_REG_ACCEL_INTEL_CTRL                              0x69
#define C6DOFIMU9_BIT_ACCEL_INTEL_EN	0x80
#define C6DOFIMU9_BIT_ACCEL_INTEL_MODE	0x40

#define C6DOFIMU9_REG_USER_CTRL                                     0x6A
#define C6DOFIMU9_BIT_COND_RST		0x01
#define C6DOFIMU9_BIT_FIFO_RST		0x04
#define C6DOFIMU9_BIT_FIFO_EN		0x40

#define C6DOFIMU9_REG_PWR_MGMT_1                                    0x6B
#define C6DOFIMU9_BIT_H_RESET		0x80
#define C6DOFIMU9_BIT_SLEEP		0x40
#define C6DOFIMU9_BIT_LP_EN		0x20
#define C6DOFIMU9_BIT_CLK_PLL		0x01
#define C6DOFIMU9_BIT_CLK_MASK		0x07

#define C6DOFIMU9_REG_PWR_MGMT_2                                    0x6C
#define C6DOFIMU9_BIT_PWR_ACCEL_STBY	0x38
#define C6DOFIMU9_BIT_PWR_GYRO_STBY	0x07
#define C6DOFIMU9_BIT_PWR_ALL_OFF		0x3F
#define C6DOFIMU9_BIT_FIFO_LP_EN		0x80

#define C6DOFIMU9_REG_FIFO_COUNTH                                   0x72
#define C6DOFIMU9_REG_FIFO_COUNTL                                   0x73
#define C6DOFIMU9_REG_FIFO_R_W                                      0x74
#define C6DOFIMU9_REG_WHO_AM_I                                      0x75

#define C6DOFIMU9_REG_20680_XG_ST_DATA	0x00
#define C6DOFIMU9_REG_20680_XA_ST_DATA	0xD

#define C6DOFIMU9_REG_XA_OFFSET_H                                   0x77
#define C6DOFIMU9_REG_XA_OFFSET_L                                   0x78
#define C6DOFIMU9_REG_YA_OFFSET_H                                   0x7A
#define C6DOFIMU9_REG_YA_OFFSET_L                                   0x7B
#define C6DOFIMU9_REG_ZA_OFFSET_H                                   0x7D
#define C6DOFIMU9_REG_ZA_OFFSET_L                                   0x7E

#define C6DOFIMU9_REG_20680_ACCEL_CONFIG2	0x1D
#define C6DOFIMU9_BIT_ACCEL_FCHOCIE_B	0x08
#define C6DOFIMU9_BIT_FIFO_SIZE_1K	0x40

#define C6DOFIMU9_REG_LP_MODE_CFG		0x1E

#define C6DOFIMU9_REG_20680_LP_ACCEL_ODR	0x1E
#define C6DOFIMU9_REG_20680_ACCEL_WOM_THR	0x1F

/* data output control reg 2 */
#define C6DOFIMU9_ACCEL_ACCURACY_SET	0x4000
#define C6DOFIMU9_GYRO_ACCURACY_SET		0x2000
#define C6DOFIMU9_CPASS_ACCURACY_SET	0x1000

/** \} */

/**
 * \defgroup configuration Configuration
 * \{
 */
#define C6DOFIMU9_BM_FIFO_MODE_FULL_REPLACE_OLD_DATA                0x00
#define C6DOFIMU9_BM_FIFO_MODE_FULL_NO_WRITE_DATA                   0x40
#define C6DOFIMU9_BM_DISABLE_FSYNC_PIN                              0x00
#define C6DOFIMU9_BM_ENABLE_FSYNC_PIN_TEMP_OUT_L                    0x08
#define C6DOFIMU9_BM_ENABLE_FSYNC_PIN_GYRO_XOUT_L                   0x10
#define C6DOFIMU9_BM_ENABLE_FSYNC_PIN_GYRO_YOUT_L                   0x18
#define C6DOFIMU9_BM_ENABLE_FSYNC_PIN_GYRO_ZOUT_L                   0x20
#define C6DOFIMU9_BM_ENABLE_FSYNC_PIN_ACCEL_XOUT_L                  0x28
#define C6DOFIMU9_BM_ENABLE_FSYNC_PIN_ACCEL_YOUT_L                  0x30
#define C6DOFIMU9_BM_ENABLE_FSYNC_PIN_ACCEL_ZOUT_L                  0x38
/** \} */

/**
 * \defgroup gyro_configuration  Gyro configuration
 * \{
 */
/* Gyro configuration */
#define C6DOFIMU9_GYRO_FULL_SCALE_250dps                           0x00FA
#define C6DOFIMU9_GYRO_FULL_SCALE_500dps                           0x01F4
#define C6DOFIMU9_GYRO_FULL_SCALE_1000dps                          0x03E8
#define C6DOFIMU9_GYRO_FULL_SCALE_2000dps                          0x07D0
/** \} */

/**
 * \defgroup accel_configuration  Accel configuration
 * \{
 */
#define C6DOFIMU9_ACCEL_FULL_SCALE_2g                               0x02
#define C6DOFIMU9_ACCEL_FULL_SCALE_4g                               0x04
#define C6DOFIMU9_ACCEL_FULL_SCALE_8g                               0x08
#define C6DOFIMU9_ACCEL_FULL_SCALE_16g                              0x10
#define C6DOFIMU9_ACCEL_AVERAGE_4_SAMPLES                           0x04
#define C6DOFIMU9_ACCEL_AVERAGE_8_SAMPLES                           0x08
#define C6DOFIMU9_ACCEL_AVERAGE_16_SAMPLES                          0x10
#define C6DOFIMU9_ACCEL_AVERAGE_32_SAMPLES                          0x20
/** \} */

/**
 * \defgroup low_power_mode_configuration  Low power mode configuration
 * \{
 */
#define C6DOFIMU9_GYRO_AVERAGE_1x                                   0x01
#define C6DOFIMU9_GYRO_AVERAGE_2x                                   0x02
#define C6DOFIMU9_GYRO_AVERAGE_4x                                   0x04
#define C6DOFIMU9_GYRO_AVERAGE_8x                                   0x08
#define C6DOFIMU9_GYRO_AVERAGE_16x                                  0x10
#define C6DOFIMU9_GYRO_AVERAGE_32x                                  0x20
#define C6DOFIMU9_GYRO_AVERAGE_64x                                  0x40
#define C6DOFIMU9_GYRO_AVERAGE_128x                                 0x80
/** \} */

/**
 * \defgroup bit_mask_fifo_enable  Bit mask FIFO enable
 * \{
 */
#define C6DOFIMU9_BM_DISABLE_TEMP_FIFO                              0x00
#define C6DOFIMU9_BM_ENABLE_TEMP_FIFO                               0x80
#define C6DOFIMU9_BM_DISABLE_XGYRO_FIFO                             0x00
#define C6DOFIMU9_BM_ENABLE_XGYRO_FIFO                              0x40
#define C6DOFIMU9_BM_DISABLE_YGYRO_FIFO                             0x00
#define C6DOFIMU9_BM_ENABLE_YGYRO_FIFO                              0x20
#define C6DOFIMU9_BM_DISABLE_ZGYRO_FIFO                             0x00
#define C6DOFIMU9_BM_ENABLE_ZGYRO_FIFO                              0x10
#define C6DOFIMU9_BM_DISABLE_ACCEL_FIFO                             0x00
#define C6DOFIMU9_BM_ENABLE_ACCEL_FIFO                              0x08
/** \} */

#define MAX_FIFO_READ_SIZE       128
#define MAX_DMP_READ_SIZE        16

#define ACCEL_COVARIANCE	0
#define BYTES_PER_SENSOR	6
#define BYTES_FOR_TEMP		2
#define FIFO_COUNT_BYTE		2
#define HARDWARE_FIFO_SIZE	512
#define FIFO_SIZE		(HARDWARE_FIFO_SIZE * 7 / 10)
#define POWER_UP_TIME		100
#define REG_UP_TIME_USEC	100
#define LEFT_OVER_BYTES		128
#define IIO_BUFFER_BYTES	8
#define BASE_SAMPLE_RATE	1000
#define DRY_RUN_TIME		50
#define INV_IAM20680_GYRO_START_TIME	35
#define INV_IAM20680_ACCEL_START_TIME	30
#define MODE_1K_INIT_SAMPLE	5
#define FIRST_SAMPLE_BUF_MS	30

/* temperature */
#define TEMP_SENSITIVITY	32680   // 326.8 LSB/degC * 100
#define TEMP_OFFSET		2500    // 25 degC * 100

#define MPU_DEFAULT_DMP_FREQ	200
#define PEDOMETER_FREQ		(MPU_DEFAULT_DMP_FREQ >> 2)
#define SENSOR_FUSION_MIN_RATE	100
#define GESTURE_ACCEL_RATE	50
#define ESI_GYRO_RATE		1000
#define MAX_FIFO_PACKET_READ	6
#define MAX_BATCH_FIFO_SIZE	FIFO_SIZE

#define MIN_MST_ODR_CONFIG	4
#define MAX_MST_ODR_CONFIG	5
/* initial rate is important. For non-DMP mode, it is set as 4 1000/256*/
#define MPU_INIT_SENSOR_RATE	4
#define MAX_MST_NON_COMPASS_ODR_CONFIG	7
#define THREE_AXES		3
#define NINE_ELEM		(THREE_AXES * THREE_AXES)
#define MPU_TEMP_SHIFT		16

#define DMP_DIVIDER		(BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ)
#define DEFAULT_BATCH_RATE	400
#define DEFAULT_BATCH_TIME	(MSEC_PER_SEC / DEFAULT_BATCH_RATE)

#define TEMPERATURE_SCALE	3340827L
#define TEMPERATURE_OFFSET	1376256L
#define SECONDARY_INIT_WAIT	100

/* data limit definitions */
#define MIN_FIFO_RATE		4
#define MAX_FIFO_RATE		MPU_DEFAULT_DMP_FREQ

#define MAX_MPU_MEM		8192
#define MAX_PRS_RATE		281

#define DEF_SELFTEST_GYRO_SENS		(32768 / 250)
/* wait time before collecting data */
#define MAX_PACKETS			20
#define SELFTEST_WAIT_TIME		(MAX_PACKETS * 10)
#define DEF_ST_STABLE_TIME		20
#define DEF_GYRO_SCALE			131
#define DEF_ST_PRECISION		1000
#define DEF_ST_ACCEL_FS_MG		2000UL
#define DEF_ST_SCALE			32768
#define DEF_ST_TRY_TIMES		2
#define DEF_ST_ACCEL_RESULT_SHIFT	1
#define DEF_ST_SAMPLES			200

#define DEF_ACCEL_ST_SHIFT_DELTA_MIN	500
#define DEF_ACCEL_ST_SHIFT_DELTA_MAX	1500
#define DEF_GYRO_CT_SHIFT_DELTA		500

#define SENSOR_UP_TIME			30
#define REG_UP_TIME			2

#define DEF_ST_ACCEL_FS_MG		2000UL
#define DEF_ACCEL_ST_SHIFT_DELTA	500
#define ACCEL_ST_AL_MIN ((DEF_ACCEL_ST_AL_MIN * DEF_ST_SCALE \
				 / DEF_ST_ACCEL_FS_MG) * DEF_ST_PRECISION)
#define ACCEL_ST_AL_MAX ((DEF_ACCEL_ST_AL_MAX * DEF_ST_SCALE \
				 / DEF_ST_ACCEL_FS_MG) * DEF_ST_PRECISION)

#define THREE_AXIS			        3
#define DEF_ST_IAM20680_ACCEL_LPF	2
#define DEF_SELFTEST_SAMPLE_RATE	0  /* 1000Hz */
#define DEF_SELFTEST_SAMPLE_RATE_LP	3  /*  250Hz */
#define DEF_SELFTEST_SAMPLE_RATE_ACC_LP	10 /*  250Hz LPOSC_CLKSEL */
#define SAMPLE_RATE_CHANGE_STABLE	50
#define DEF_SELFTEST_20680_ACCEL_FS	(0 << 3)
#define DEF_SELFTEST_GYRO_FS		(0 << 3)
#define DEF_ST_20680_STABLE_TIME	20
#define BIT_ACCEL_OUT			    0x08
#define BITS_GYRO_OUT			    0x70

#define DEF_GYRO_WAIT_TIME		10
#define DEF_GYRO_WAIT_TIME_LP		50

/* Gyro Offset Max Value (dps) */
#define DEF_GYRO_OFFSET_MAX		20
/* Gyro Self Test Absolute Limits ST_AL (dps) */
#define DEF_GYRO_ST_AL			60
/* Accel Self Test Absolute Limits ST_AL (mg) */
#define DEF_ACCEL_ST_AL_MIN		225
#define DEF_ACCEL_ST_AL_MAX		675


/** @defgroup Sensitivity_selection               
  * @{
  */
#define IAM20680_SENSITIVITY_0_06G            0.06  /* 0.06 mg/digit*/
#define IAM20680_SENSITIVITY_0_12G            0.12  /* 0.12 mg/digit*/
#define IAM20680_SENSITIVITY_0_18G            0.18  /* 0.18 mg/digit*/
#define IAM20680_SENSITIVITY_0_24G            0.24  /* 0.24 mg/digit*/
#define IAM20680_SENSITIVITY_0_73G            0.73  /* 0.73 mg/digit*/

#define IAM20680_GYRO_SENSITIVITY_250            0.008  /* 0.008 dps/digit*/
#define IAM20680_GYRO_SENSITIVITY_500            0.004  /* 0.004 dps/digit*/
#define IAM20680_GYRO_SENSITIVITY_1000           0.002  /* 0.002 dps/digit*/
#define IAM20680_GYRO_SENSITIVITY_2000           0.001  /* 0.001 dps/digit*/

enum inv_filter_e {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};
/**
 * \defgroup device_id  Device ID
 * \{
 */
#define C6DOFIMU9_DEVICE_ID                                         0xA9
/** \} */

/** \} */ // End group macro 
// --------------------------------------------------------------- PUBLIC TYPES
/**
 * \defgroup type Types
 * \{
 */

struct recover_regs {
	uint8_t int_enable;		/* REG_INT_ENABLE */
	uint8_t fifo_en;		/* REG_FIFO_EN */
	uint8_t user_ctrl;		/* REG_USER_CTRL */
	uint8_t config;		/* REG_CONFIG */
	uint8_t gyro_config;		/* REG_GYRO_CONFIG */
	uint8_t accel_config;	/* REG_ACCEL_CONFIG */
	uint8_t accel_config_2;	/* REG_ACCEL_CONFIG_2 */
	uint8_t smplrt_div;		/* REG_SAMPLE_RATE_DIV */
	uint8_t lp_mode;		/* REG_LP_MODE_CTRL */
	uint8_t pwr_mgmt_1;		/* REG_PWR_MGMT_1 */
	uint8_t pwr_mgmt_2;		/* REG_PWR_MGMT_2 */
};

static struct recover_regs saved_regs;

static const uint16_t mpu_st_tb[256] = {
	2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
	2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
	3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
	3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
	3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
	3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
	4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
	4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
	4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
	5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
	5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
	6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
	6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
	7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
	7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
	8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
	9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
	10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
	10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
	11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
	12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
	13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
	15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
	16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
	17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
	19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
	22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
	24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
	26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
	28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
	30903, 31212, 31524, 31839, 32157, 32479, 32804
};
/**
 * @brief Communication type.
 */
typedef uint8_t  c6dofimu9_select_t;
typedef uint8_t  spi_master_chip_select_polarity_t;

/**
 * @brief Master Input/Output type.
 */
typedef int ( *c6dofimu9_master_io_t )( struct c6dofimu9_s*, uint8_t, uint8_t*, uint8_t );

/**
 * @brief Click ctx object definition.
 */
typedef struct c6dofimu9_s
{
    // Output pins 

    GPIO_TypeDef* syn;
    GPIO_TypeDef* cs;

    // Input pins

    GPIO_TypeDef* int_pin;

    // Modules

	I2C_HandleTypeDef i2c;
	SPI_HandleTypeDef spi;

    // ctx variable 

    uint8_t slave_address;
    GPIO_InitTypeDef chip_select;
    c6dofimu9_master_io_t  write_f;
    c6dofimu9_master_io_t  read_f;
    c6dofimu9_select_t master_sel;
    
    int accel_st_bias[3];
    int gyro_st_bias[3];
	uint8_t is_asleep;
	uint8_t accel_lp_mode;

	Quaternion q;

	union {
		Gyroscope g;
		EulerAngles e;
	};
	union {
		Gyroscope ekf_g;
		EulerAngles ekf_e;
	};

	union {
		Gyroscope delta_g;
		EulerAngles delta_e;
	};

	union {
		Gyroscope abs_g;
		EulerAngles abs_e;
	};
} c6dofimu9_t;

/**
 * @brief Click configuration structure definition.
 */
typedef struct
{
    // Communication gpio pins 

    GPIO_InitTypeDef scl;
    GPIO_InitTypeDef sda;
    GPIO_InitTypeDef miso;
    GPIO_InitTypeDef mosi;
    GPIO_InitTypeDef sck;
    GPIO_InitTypeDef cs;
//
//    // Additional gpio pins
//
    GPIO_InitTypeDef syn;
    GPIO_InitTypeDef int_pin;

    // static variable 

    uint32_t i2c_speed;
    uint8_t i2c_address;
    uint32_t spi_speed;
    uint8_t spi_mode;
    spi_master_chip_select_polarity_t cs_polarity;

    c6dofimu9_select_t sel;

} c6dofimu9_cfg_t;

/** \} */ // End types group
// ----------------------------------------------- PUBLIC FUNCTION DECLARATIONS
/**
 * \defgroup public_function Public function
 * \{
 */

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief Config Object Initialization function.
 *
 * @param cfg  Click configuration structure.
 *
 * @description This function initializes click configuration structure to init state.
 * @note All used pins will be set to unconnected state.
 */
void c6dofimu9_cfg_setup ( c6dofimu9_cfg_t *cfg );

/**
 * @brief Initialization function.
 * @param c6dofimu9 Click object.
 * @param cfg Click configuration structure.
 * 
 * @description This function initializes all necessary pins and peripherals used for this click.
 */
C6DOFIMU9_RETVAL c6dofimu9_init ( c6dofimu9_t *ctx, c6dofimu9_cfg_t *cfg );

/**
 * @brief Click Default Configuration function.
 *
 * @param ctx  Click object.
 *
 * @description This function executes default configuration for 6DOF IMU 9 click.
 */
void c6dofimu9_default_cfg ( c6dofimu9_t *ctx );

/**
 * @brief Generic write function.
 *
 * @param ctx        Click object.
 * @param reg        Register address.
 * @param data_buf   Output data buf
 * @param len        Number of the bytes to be read
 *
 * @description This function writes data to the desired register.
 */
int c6dofimu9_generic_write ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

/**
 * @brief Generic read function.
 *
 * @param ctx      Click object.
 * @param reg          Register address.
 * @param data_buf  Data buf to be written.
 * @param len          Number of the bytes in data buf.
 *
 * @description This function reads data from the desired register.
 */
int c6dofimu9_generic_read ( c6dofimu9_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len );

/**
 * @brief Check interrupt state function
 *
 * @param ctx      Click object.
 *
 * @return
 * Interrupt state:
 * - 0 : No active;
 * - 1 : Active; 
 * @description  Function check interrupt state by return state of the INT pin of 6DOF IMU 9 Click board.
 */
uint8_t c6dofimu9_check_interrupt ( c6dofimu9_t *ctx );

/**
 * @brief Set configuration function
 *
 * @param ctx      Click object.
 * @param config_data          Register address.
 *
 * @note
 *<pre>
 * - [ bit 7 ]     : Always set to 0;
 * - [ bit 6 ]     : FIFO is full:
 *                   - 0 : Additional writes will be written to the FIFO, replacing the oldest data;
 *                   - 1 : Additional writes will not be written to FIFO;
 * - [ bit 5 : 3 ] : Enables the FSYNC pin data to be sampled:
 *                   - 0 : Function disabled;
 *                   - 1 : TEMP_OUT_L;
 *                   - 2 : GYRO_XOUT_L;
 *                   - 3 : GYRO_YOUT_L;
 *                   - 4 : GYRO_ZOUT_L;
 *                   - 5 : ACCEL_XOUT_L;
 *                   - 6 : ACCEL_YOUT_L;
 *                   - 7 : ACCEL_ZOUT_L;
 * - [ bit 2 : 0 ] : For the DLPF to be used;
 *</pre>
 *
 * @description Function set configuration data to the targeted C6DOFIMU9_REG_CONFIG register address of
 * IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_config ( c6dofimu9_t *ctx, uint8_t config_data );

/**
 * @brief Set Gyro configuration function
 *
 * @param ctx      Click object.
 * @param gyro_config_data
 *
 * @note
 *<pre>
 * - [ bit 7 ]     : X Gyro self-test;
 * - [ bit 6 ]     : Y Gyro self-test;
 * - [ bit 5 ]     : Z Gyro self-test;
 * - [ bit 4 : 3 ] : Gyro Full Scale Select:
 *                   - 0 : � 250 dps;
 *                   - 1 : � 500 dps;
 *                   - 2 : �1000 dps;
 *                   - 3 : �2000 dps;
 * - [ bit 2 ]     : Reserved;
 * - [ bit 1 : 0 ] : Used to bypass DLPF as shown in Table 16 above;
 *</pre>
 *
 * @description Function set Gyro configuration data to the targeted C6DOFIMU9_REG_GYRO_CONFIG register address of
 * IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_gyro_config ( c6dofimu9_t *ctx, uint8_t gyro_config_data );

/**
 * @brief Set Gyro measurement range configuration function
 *
 * @param ctx      Click object.
 * @param gyro_full_scale_range
 * @note
 *<pre>
 * -  250 : Gyro Full Scale Select � 250 dps;
 * -  500 : Gyro Full Scale Select � 500 dps;
 * - 1000 : Gyro Full Scale Select �1000 dps;
 * - 2000 : Gyro Full Scale Select �2000 dps;
 *</pre>
 *
 * @description Function set Gyro configuration data to the targeted C6DOFIMU9_REG_GYRO_CONFIG register address of
 * IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_gyro_measurement_range ( c6dofimu9_t *ctx, uint16_t gyro_full_scale_range );

/**
 * @brief Set Accel configuration function
 *
 * @param ctx      Click object.
 * @param accel_config_data          
 *
 * @note
 *<pre>
 * - [ bit 15 ]      : X Accel self-test;
 * - [ bit 14 ]      : Y Accel self-test;
 * - [ bit 13 ]      : Z Accel self-test;
 * - [ bit 12 : 11 ] : Accel Full Scale Select:
 *                     - 0 : � 2g;
 *                     - 1 : � 4g;
 *                     - 2 : � 8g;
 *                     - 3 : �16g;
 * - [ bit 10 : 6 ]  : Reserved;
 * - [ bit  5 : 4 ]  : Averaging filter settings for Low Power Accelerometer mode:
 *                     - 0 : Average  4 samples;
 *                     - 1 : Average  8 samples;
 *                     - 2 : Average 16 samples;
 *                     - 3 : Average 32 samples;
 * - [ bit  3 ]      : Used to bypass DLPF as shown;
 * - [ bit  2 : 0 ]  : Accelerometer 3-dB BW:
 *                     - 0 : 1046.0 Hz;
 *                     - 1 :  218.1 Hz;
 *                     - 2 :  218.1 Hz;
 *                     - 3 :   99.0 Hz;
 *                     - 4 :   44.8 Hz;
 *                     - 5 :   21.2 Hz;
 *                     - 6 :   10.2 Hz;
 *                     - 7 :  420.0 Hz;
 *</pre>
 *
 * @description Function set Accel configuration data to the targeted C6DOFIMU9_REG_ACCEL_CONFIG and C6DOFIMU9_REG_ACCEL_CONFIG_2
 * register address of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_accel_config ( c6dofimu9_t *ctx, uint16_t accel_config_data );

/**
 * @brief Set Accel measurement range configuration function
 *
 * @param ctx      Click object.
 * @param accel_full_scale_range          
 *
 * @note 
 *<pre>
 * -  2 : Accel Full Scale Select � 2g;
 * -  4 : Accel Full Scale Select � 4g;
 * -  8 : Accel Full Scale Select � 8g;
 * - 16 : Accel Full Scale Select �16g;
 *</pre>
 * @description Function set Accel measurement range configuration data to the targeted C6DOFIMU9_REG_ACCEL_CONFIG
 * register address of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_accel_measurement_range ( c6dofimu9_t *ctx, uint8_t accel_full_scale_range );

/**
 * @brief Set Accel averaging filter settings for Low Power mode function
 *
 * @param ctx      Click object.
 * @param accel_avg_samples          Register address.
 * @note
 *<pre>
 * -  4 : Average  4 samples;
 * -  8 : Average  8 samples;
 * - 16 : Average 16 samples;
 * - 32 : Average 32 samples;
 *</pre>
 * @description Function set Accel averaging filter settings for Low Power mode data to the targeted C6DOFIMU9_REG_ACCEL_CONFIG_2
 * register address of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_accel_avg_filter_mode ( c6dofimu9_t *ctx, uint8_t accel_avg_samples );

/**
 * @brief Generic read function.
 *
 * @param ctx      Click object.
 * @param gyro_averages          Register address.
 * @note
 *<pre> 
 * - [ bit 7 ]     : Low-power gyroscope mode:
 *                   - 0 : Disabled ( Default );
 *                   - 1 : Enabled;
 * - [ bit 6 : 4 ] : Averaging filter configuration for low-power gyroscope mode:
 *                   - 0 : Averages   1x; Ton (ms)  1.73; Noise BW (Hz) 650.8;
 *                   - 1 : Averages   2x; Ton (ms)  2.23; Noise BW (Hz) 407.1;
 *                   - 2 : Averages   4x; Ton (ms)  3.23; Noise BW (Hz) 224.2;
 *                   - 3 : Averages   8x; Ton (ms)  5.23; Noise BW (Hz) 117.4;
 *                   - 4 : Averages  16x; Ton (ms)  9.23; Noise BW (Hz)  60.2;
 *                   - 5 : Averages  32x; Ton (ms) 17.23; Noise BW (Hz)  30.6;
 *                   - 6 : Averages  64x; Ton (ms) 33.23; Noise BW (Hz)  15.6;
 *                   - 7 : Averages 128x; Ton (ms) 65.23; Noise BW (Hz)   8.0;
 * - [ bit 3 : 0 ] : Reserved;
 *</pre>
 *
 * @description Function set Gyro Low-Power mode configuration data to the targeted C6DOFIMU9_REG_LP_MODE_CFG
 * register address of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_gyro_config_lp_mode ( c6dofimu9_t *ctx, uint8_t gyro_averages );

/**
 * @brief Get Accel wake-on motion threshold function
 *
 * @param ctx      Click object.
 *
 * @description Function get Accel wake-on motion threshold data from the targeted C6DOFIMU9_REG_ACCEL_WOM_THR
 * register address of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
uint8_t c6dofimu9_get_accel_wake_on_motion_trsh ( c6dofimu9_t *ctx );

/**
 * @brief Enable FIFO configuration function
 *
 * @param ctx      Click object.
 * @param en_fifo_data          
 * @note
 * <pre>
 * - [ bit 7 ]     : Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate:
 *                   - 0 : Disabled;
 *                   - 1 : Enabled;
 * - [ bit 6 ]     : Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate:
 *                   - 0 : Disabled;
 *                   - 1 : Enabled;
 * - [ bit 5 ]     : Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample rate:
 *                   - 0 : Disabled;
 *                   - 1 : Enabled;
 * - [ bit 4 ]     : Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample rate:
 *                   - 0 : Disabled;
 *                   - 1 : Enabled;
 * - [ bit 3 ]     : Write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H and ACCEL_ZOUT_L to the FIFO at the sample rate:
 *                   - 0 : Disabled;
 *                   - 1 : Enabled;
 * - [ bit 2 : 0 ] : Reserved;
 * </pre>
 * @description Function enable FIFO configuration by written the byte of data to the targeted C6DOFIMU9_REG_FIFO_EN
 * register address of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_enable_fifo ( c6dofimu9_t *ctx, uint8_t en_fifo_data );

/**
 * @brief Enable Accel interrupt function
 *
 * @param ctx      Click object.
 *
 * @description Function enable Accel interrupt of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device
 * on 6DOF IMU 9 Click board.
 */
void c6dofimu9_enable_accel_interrupt ( c6dofimu9_t *ctx );

/**
 * @brief Enable Gyro interrupt function
 *
 * @param ctx      Click object.
 *
 * @description Function enable Gyro interrupt of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device
 * on 6DOF IMU 9 Click board.
 */
void c6dofimu9_enable_gyro_interrupt ( c6dofimu9_t *ctx );

/**
 * @brief Enable data ready interrupt function
 *
 * @param ctx      Click object.
 *
 * @description Function enable data ready interrupt of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device
 * on 6DOF IMU 9 Click board.
 */
void c6dofimu9_enable_data_ready_interrupt ( c6dofimu9_t *ctx );

/**
 * @brief Get interrupt status function
 *
 * @param ctx      Click object.
 * 
 * @return interrupt status
 * @note
 *<pre>
 * - [ bit 7 : 5 ] : Accelerometer WoM interrupt status. Cleared on Read;
 * - [ bit 4 ]     : This bit automatically sets to 1 when a FIFO buffer overflow has been generated. The bit clears to 0 after the register has been read;
 * - [ bit 3 ]     : Reserved;
 * - [ bit 2 ]     : Gyroscope Drive System Ready interrupt;
 * - [ bit 1 ]     : Reserved;
 * - [ bit 0 ]     : This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read;
 *</pre>
 *
 * @description Function Get interrupt status of the targeted C6DOFIMU9_REG_INT_STATUS
 * register address of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
uint8_t c6dofimu9_get_interrupt_status ( c6dofimu9_t *ctx );

/**
 * @brief Reset Accel signal path function
 *
 * @param ctx      Click object.
 *
 * @description Function reset Accel signal path of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device
 * on 6DOF IMU 9 Click board.
 */
void c6dofimu9_reset_accel_signal ( c6dofimu9_t *ctx );

/**
 * @brief Reset Gyro signal path function
 *
 * @param ctx      Click object.
 *
 * @description Function reset Gyro signal path of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device
 * on 6DOF IMU 9 Click board.
 */
void c6dofimu9_reset_gyro_signal ( c6dofimu9_t *ctx );

/**
 * @brief Enable the Wake-on-Motion detection function
 *
 * @param ctx      Click object.
 *
 * @description Function enable the Wake-on-Motion detection of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device
 * on 6DOF IMU 9 Click board.
 */
void c6dofimu9_enable_accel_wake_on_motion ( c6dofimu9_t *ctx );

/**
 * @brief Reset the internal registers and restores the default settings function
 *
 * @param ctx      Click object.
 *
 * @description Function reset the internal registers and restores the default settings of
 * IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_reset_default ( c6dofimu9_t *ctx );

/**
 * @brief Set the device to sleep mode function
 *
 * @param ctx      Click object.
 *
 * @description Function set the device to sleep mode of
 * IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_sleep_mode ( c6dofimu9_t *ctx );

/**
 * @brief Set the device to standby mode function
 *
 * @param ctx      Click object.
 *
 * @description Function set the device to standby mode of
 * IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_set_standby_mode ( c6dofimu9_t *ctx );

/**
 * @brief Get device ID function
 *
 * @param ctx      Click object.
 *
 * @return
 * 8-bit device ID value ( default 0xA9 ); 
 * @description This function reads data from the desired register.
 */
uint8_t c6dofimu9_get_device_id ( c6dofimu9_t *ctx );

/**
 * @brief Get axis data function
 *
 * @param ctx           Click object.
 * @param addr_reg_msb  least significant bit register address
 * @param addr_reg_lsb  most significant bit register address
 *
 * @description Function get axis data by read data from the two targeted register address
 * of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
int16_t c6dofimu9_get_axis ( c6dofimu9_t *ctx, uint8_t addr_reg_msb, uint8_t addr_reg_lsb );

/**
 * @brief Read Accel X-axis, Y-axis and Z-axis axis function
 *
 * @param ctx           Click object.
 * @param p_accel_x     pointer to memory location where Accel X-axis data be stored
 * @param p_accel_y     pointer to memory location where Accel Y-axis data be stored
 * @param p_accel_z     pointer to memory location where Accel Z-axis data be stored
 *
 * @description Function reads 16-bit ( signed ) Accel X-axis, Y-axis data and Z-axis data from the
 * targeted starts from C6DOFIMU9_REG_ACCEL_XOUT_H to the C6DOFIMU9_REG_ACCEL_ZOUT_L register address
 * of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_get_accel_data ( c6dofimu9_t *ctx, int16_t *p_accel_x, int16_t *p_accel_y, int16_t *p_accel_z );

/**
 * @brief Read Gyro X-axis, Y-axis and Z-axis axis function
 *
 * @param ctx          Click object.
 * @param p_gyro_x     pointer to memory location where Accel X-axis data be stored
 * @param p_gyro_y     pointer to memory location where Accel Y-axis data be stored
 * @param p_gyro_z     pointer to memory location where Accel Z-axis data be stored
 *
 * @description Function reads 16-bit ( signed ) Gyro X-axis, Y-axis data and Z-axis data from the
 * targeted starts from C6DOFIMU9_REG_GYRO_XOUT_H to the C6DOFIMU9_REG_GYRO_ZOUT_L register address
 * of IAM-20680 High Performance Automotive 6-Axis MotionTracking Device on 6DOF IMU 9 Click board.
 */
void c6dofimu9_get_gyro_data ( c6dofimu9_t *ctx, int16_t *p_gyro_x, int16_t *p_gyro_y, int16_t *p_gyro_z );

void c6dofimu9_get_temperature_data ( c6dofimu9_t *ctx, int32_t *temp );

int c6dofimu9_hw_selftest ( c6dofimu9_t *ctx );


Quaternion ToQuaternion(EulerAngles e);
EulerAngles ToEulerAngles(Quaternion q);

#ifdef __cplusplus
}
#endif
#endif  // _C6DOFIMU9_H_

/** \} */ // End public_function group
/// \}    // End click Driver group  
/*! @} */
// ------------------------------------------------------------------------- END
