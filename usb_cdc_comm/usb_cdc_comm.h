/** @file usb_cdc_comm.h
 *  @brief Provides an interface to the USB-CDC-class device
 *
 *  @author Josef Heel
	@date March 20th, 2019
 */

// PROTOTYPES
int USB_CDC_Init(void);
int USB_CDC_TransmitBuffer(uint8_t* buffer, uint32_t length);
void USB_CDC_addDataToRxBuffer(uint8_t* buffer, uint32_t length); // Called by driver! Do not call yourself!
void USB_CDC_clearRxBuffer(void);



