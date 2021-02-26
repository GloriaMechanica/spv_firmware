/** @file usb_cdc_comm.h
 *  @brief Provides an interface to the USB-CDC-class device
 *
 *  @author Josef Heel
	@date March 20th, 2019
 */


#define 	USB_CDC_RX_BUFFER_SIZE		1024		// Receive buffer (Data from PC is put here)

// Struct for accessing the usb rx-buffer
typedef struct
{
	uint8_t data[USB_CDC_RX_BUFFER_SIZE]; 		// space for keeping received data
	uint32_t top; 								// Offset to the first empty byte of the buffer
	int32_t packet_in_buffer; 					// Flag used to indicate when something is in the input buffer

}T_USB_CDC_RX_BUFFER;

// GLOBAL VARIABLES
T_USB_CDC_RX_BUFFER usb_cdc_rx_buffer; 			// Global data structure for keeping received data


// PROTOTYPES
int USB_CDC_Init(void);
int USB_CDC_TransmitBuffer(uint8_t* buffer, uint32_t length);
void USB_CDC_addDataToRxBuffer(uint8_t* buffer, uint32_t length); // Called by driver! Do not call yourself!
void USB_CDC_clearRxBuffer(void);



