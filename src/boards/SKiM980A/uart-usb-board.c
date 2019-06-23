/*!
 * \file      uart-usb-board.c
 *
 * \brief     Target board UART USB driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *              (C)2015-2019 EmOne
 *
 * \endcode
 *
 * \author    Anol Paisal (anol.p@emone.co.th)
 *
 */

#include "uart-usb-board.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern PCD_HandleTypeDef hpcd_USB_FS;
/*!
 * \brief Initializes the UART object and MCU peripheral
 *
 * \param [IN] obj  UART object
 * \param [IN] tx   UART Tx pin name to be used
 * \param [IN] rx   UART Rx pin name to be used
 */
void UartUsbInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
    MX_USB_DEVICE_Init();
}

/*!
 * \brief Initializes the UART object and USB peripheral
 *
 * \param [IN] obj          UART object
 * \param [IN] mode         Mode of operation for the UART
 * \param [IN] baudrate     UART baudrate
 * \param [IN] wordLength   packet length
 * \param [IN] stopBits     stop bits setup
 * \param [IN] parity       packet parity
 * \param [IN] flowCtrl     UART flow control
 */
void UartUsbConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    
}

/*!
 * \brief DeInitializes the UART object and USB peripheral
 *
 * \param [IN] obj  UART object
 */
void UartUsbDeInit( Uart_t *obj )
{

}

/*!
 * \brief Checks if the cable is connected or not
 *
 * \retval connected [0: Not connected, 1: Connected]
 */
uint8_t UartUsbIsUsbCableConnected( void )
{
    return 0;
}

/*!
 * \brief Sends a buffer to the UART
 *
 * \param [IN] obj    UART object
 * \param [IN] buffer Buffer to be sent
 * \param [IN] size   Buffer size
 * \retval status     [0: OK, 1: Busy, 2: Fail]
 */
uint8_t UartUsbPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size ) {
    return CDC_Transmit_FS(buffer, size);
}

/*!
 * \brief Sends a character to the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Character to be sent
 * \retval status    [0: OK, 1: Busy, 2: Fail]
 */
uint8_t UartUsbPutChar( Uart_t *obj, uint8_t data ) {
    return CDC_Transmit_FS(&data, 1);
}

/*!
 * \brief Gets a character from the UART
 *
 * \param [IN] obj   UART object
 * \param [IN] data  Received character
 * \retval status    [0: OK, 1: Busy, 2: Fail]
 */
uint8_t UartUsbGetChar( Uart_t *obj, uint8_t *data ) {
    return 0;
}

/**
  * @brief This function handles USB low priority interrupt.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}