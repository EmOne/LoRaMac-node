/*!
 * \file      encoder-board.h
 *
 * \brief     Encoder driver implementation
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
 *
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "gpio.h"
#ifndef USE_ENCODER
#define USE_ENCODER 1
#endif
/*!
 * ENCODER peripheral ID
 */
typedef enum
{
    TIM_2,
} EncoderId_t;

/*!
 * TIM object type definition
 */
typedef struct {
	int32_t fwd_cnt;
	int32_t rev_cnt;
	int32_t rate;	//Speed degree per second
    uint8_t status; //[0:tampering, 1:alarm, 2:dir]
	volatile uint16_t adc_lvl[3];
	uint32_t adc_acc;
	uint32_t adc_avg;
	uint32_t adc_sample;
	uint32_t last;
	uint8_t batt;
	int16_t temp;
	uint32_t position;
} flow_t;

typedef struct 
{
	uint8_t sampling;
	uint8_t digital_alarm;
	uint16_t analog_alarm;
	uint8_t isActiveMode;
} flow_config_t;

typedef struct Encoder_s
{
    EncoderId_t EncoderId;
    Gpio_t Pulse;
    Gpio_t Direction;
    Gpio_t Tampering;
    Gpio_t Alarm;
	Gpio_t En;
	void ( *OnSendOneshot) ( void );
	void ( *OnShowPulseDetect ) (  void );
	void ( *OnArcRelease ) ( void );
	flow_t * FlowData;
	flow_t * LastFlowData;
	flow_config_t * ConfigData;
} Encoder_t;

extern Encoder_t Encoder;
extern flow_t flow;
extern flow_t last_flow;
extern flow_config_t config;

/*!
 * Hardware IO IRQ callback function definition
 */
/*!
 * GPIO IRQ handler function prototype
 */
typedef void( EncoderIrqHandler )( void* context );

/*!
 * \brief Initializes the ENCODER object and MCU peripheral
 *
 * \remark 
 *
 * \param [IN] obj  ENCODER object
 */
void EncoderInit( Encoder_t *obj, EncoderId_t timId, PinNames pulse, PinNames dir, PinNames tampering, PinNames alarm, PinNames en);

/*!
 * \brief De-initializes the ENCODER object and MCU peripheral
 *
 * \param [IN] obj ENCODER object
 */
void EncoderDeInit( Encoder_t *obj );


void EncoderUpdateStatus ( void ) ;

#endif // __SPI_H__
