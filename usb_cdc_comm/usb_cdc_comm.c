/** @file usb_cdc_comm.c
 *  @brief Provides an interface to the USB-CDC-class device
 *
 *  The USB-CDC-Class device is used for the main data transfer
 *  between PC and SPV (music control data).
 *
 *  @author Josef Heel
	@date March 20th, 2019
 */

#include "main.h"
#include "usb_cdc_comm.h"
#include "usbd_cdc_if.h"

int USB_CDC_TransmitBuffer(uint8_t* buffer, uint32_t length)
{

	if (CDC_Transmit_FS(buffer, length))
		return SUCCESS;
	else
		return ERROR;
}

