/** @file settings.h
 *  @brief All sorts of general settings that should be easy to access
 *
 *  @author Josef Heel
	@date March 21th, 2019
 */



/*
 * USB CDC Communication with PC for music data
 */
#define 	USB_CDC_RX_BUFFER_SIZE		1024		// Receive buffer (Data from PC is put here)
#define 	DBG_TIM_ISR_LOAD_PIN		0			// switch to 1-> duration in timer-isr will pull the ISR_LOAD Pin high.
