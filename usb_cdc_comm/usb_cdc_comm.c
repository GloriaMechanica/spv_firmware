/** @file usb_cdc_comm.c
 *  @brief Provides an interface to the USB-CDC-class device
 *
 *  The USB-CDC-Class device is used for the main data transfer
 *  between PC and SPV (music control data).
 *
 *  @author Josef Heel
	@date March 20th, 2019
 */


#include "settings.h"
#include "main.h"
#include "usb_cdc_comm.h"
#include "usbd_cdc_if.h"
#include "debug_tools.h"


// Struct for accessing the usb rx-buffer
typedef struct
{
	uint8_t data[USB_CDC_RX_BUFFER_SIZE]; 		// space for keeping received data
	uint32_t top; 								// Offset to the first empty byte of the buffer
}T_USB_CDC_RX_BUFFER;


// GLOBAL VARIABLES
T_USB_CDC_RX_BUFFER usb_cdc_rx_buffer; 			// Global data structure for keeping received data


/** @brief When using USB CDC stuff, call this fx first
 *
 *  @param (none)
 *  @return SUCCESS
 */
int USB_CDC_Init(void)
{
	USB_CDC_clearRxBuffer();
	return SUCCESS;
}


/** @brief Transmits bytes out of a buffer to the PC via
 * 			USB CDC device.
 *
 *  @param buffer - byte array containing bytes to transmit
 *  @param length - number of bytes to transmit out of buffer
 *  @return (none)
 */
int USB_CDC_TransmitBuffer(uint8_t* buffer, uint32_t length)
{

	if (CDC_Transmit_FS(buffer, length))
		return SUCCESS;
	else
		return ERROR;
}

/** @brief This function is called by the CDC driver!
 * 			It adds the freshly received bytes on the buffer.
 * 			If long sequences are received, this function may
 * 			be called multiple times because the max number of bytes
 * 			from USB at one point is 64 bytes (endpoint size).
 *
 * 			BE CAREFUL: This function must not be blocked for too long, otherwise
 * 			packets will be lost and USB will not work properly!
 *
 *  @param buffer - USB driver passes over its internal buffer here
 *  @param length - USB driver tells us how many bytes it currently has
 *  @return (none)
 */
void USB_CDC_addDataToRxBuffer(uint8_t* buffer, uint32_t length)
{
	if (usb_cdc_rx_buffer.top + length > USB_CDC_RX_BUFFER_SIZE)
		return;

	memcpy(&usb_cdc_rx_buffer.data[usb_cdc_rx_buffer.top], buffer, length);
	usb_cdc_rx_buffer.top += length;

	//dbgprintf("Buffer status:");
	//dbgprintbuf(usb_cdc_rx_buffer.data, usb_cdc_rx_buffer.top);
}

/** @brief Clears the receive buffer
 *  @param (none)
 *  @return (none)
 */
void USB_CDC_clearRxBuffer(void)
{
	usb_cdc_rx_buffer.top = 0;
}







