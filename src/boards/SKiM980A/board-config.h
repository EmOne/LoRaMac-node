/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

/*!
 * Enables the choice between Led1 and Potentiometer.
 * LED1 and Potentiometer are exclusive.
 * \remark When using Potentiometer don't forget  that the connection between
 *         ADC input pin of iM980A and the Demoboard Poti requires a connection
 *         between X5:11 - X5:18.
 *         Remove the original jumpers for that.
 *         On SKiM980A X5 is the 20 pin header close to the DIP SW and Buttons
 */
#define USE_POTENTIOMETER                           1
#define USE_SPI                                     0
#define USE_ENCODER                                 1
#define USE_GPS                                     1
#define USE_USB_CDC                                 1
/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PA_2

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5
#define RADIO_NSS                                   PB_0

#define RADIO_DIO_0                                 PB_1
#define RADIO_DIO_1                                 PB_10
#define RADIO_DIO_2                                 PB_11
#define RADIO_DIO_3                                 PB_7
#define RADIO_DIO_4                                 PB_5
#define RADIO_DIO_5                                 PB_4

#define RADIO_ANT_SWITCH_RX                         PC_13
#define RADIO_ANT_SWITCH_TX                         PA_4

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define USB_DM                                      PA_11
#define USB_DP                                      PA_12

#define JTAG_TMS                                    PA_13
#define JTAG_TCK                                    PA_14
#define JTAG_TDI                                    PA_15
#define JTAG_TDO                                    PB_3
#define JTAG_NRST                                   PB_4

#define I2C_SCL                                     PB_8
#define I2C_SDA                                     PB_9

#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#if ( USE_POTENTIOMETER == 1 )
#define POTI                                        PA_3
#else
#define LED_1                                       PA_3
#endif
#if ( USE_ENCODER )
#define PULSE                                       PA_0
#define DIR                                         PA_1
#define TAMPERING                                   PA_8
#define ALARM                                       PB_12
#define LED_1                                       JTAG_TDO
#define LED_2                                       PB_14
#define LED_3                                       PB_13
#define LED_4                                       PB_15
#define USER_BUTTON                                 JTAG_TDI
#else
#define LED_2                                       PA_0
#define LED_3                                       PA_1
#define LED_4                                       PA_8
#endif

#if (USE_GPS == 1)
#define GPS_UART_TX                                 UART_TX
#define GPS_UART_RX                                 UART_RX
#define GPS_POWER_ON                                ALARM
#define GPS_PPS                                     NC
#endif

#if (USE_SPI == 1)
#define SPI_MOSI                                    PB_15
#define SPI_MISO                                    PB_14
#define SPI_SCK                                     PB_13
#define SPI_NSS                                     PB_12
#endif

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            NC
#define RADIO_DBG_PIN_RX                            NC

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
