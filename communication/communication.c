/*
 * communication.c
 *
 * This software module handles all the communication with the PC, i.e. the SPVplayer
 *
 *  Created on: Nov 24, 2020
 *      Author: josef
 */

#include "communication.h"
#include "motor_control.h"
#include "usb_cdc_comm.h"
#include "debug_tools.h"

/** @brief  Initializes communication stuff.
 *
 *  @param (none)
 *  @return (none)
 */
void COM_init(void)
{
	comm.timeout = 0;
	comm.packet_in_buffer = 0;
}

/** @brief 	Is periodically called from the main loop and checks if new
 * 			commands have entered via the USB-CDC interface.
 *
 *  @param (none)
 *  @return (none)
 */
void COM_update (void)
{
	if (comm.packet_in_buffer==1)
	{
		comm.packet_in_buffer = 0;

		// DEBUG
		SM_restart_testcylce();

		// Clear the buffer at the end, so that new commands are correctly interpreted
		USB_CDC_clearRxBuffer();
	}

}

/** @brief 	Called from the 1ms-timer. Used to update the timeout
 *
 *  @param (none)
 *  @return (none)
 */
void COM_updateTimeout (void)
{
	if (comm.timeout > 1)
	{
		comm.timeout--;
	}
	else if (comm.timeout == 1)
	{
		comm.timeout = 0;
		USB_CDC_clearRxBuffer();
	}
}

/** @brief 	Called from UART Rx handler. Sets the timeout because
 * 			fresh bytes have been received. If nothing comes to
 * 			complete the packet within the timeout, it will be tossed away.
 *
 *  @param (none)
 *  @return (none)
 */
void COM_restartTimeout (void)
{
	comm.timeout = COM_PACKET_TIMEOUT;
}

/** @brief 	Stops the timeout if the buffer content should not be tossed out because
 *
 *  @param (none)
 *  @return (none)
 */
void COM_stopTimeout (void)
{
	comm.timeout = COM_PACKET_TIMEOUT;
}

/** @brief 	Is periodically called from the main loop and checks if new
 * 			commands have entered via the USB-CDC interface.
 *
 *  @param 	*buf - pointer to buffer where data is
 *  		len - length the function is allowed to read
 *  @return (none)
 */
E_COM_PACKET_STATUS COM_checkIfPacketValid(uint8_t *buf, int32_t len)
{
	uint16_t packet_len = 0;

	if (len < COM_MIN_PACKET_LEN || len < 2)
		return COM_PACKET_SMALLER_MINIMAL_LENGTH;

	if (buf[0] != COM_SPV_UID_0 && buf[1] != COM_SPV_UID_1)
		return COM_PACKET_UID_ERROR;

	packet_len = buf[2] << 8 | buf[3];

	dbgprintf("len: %d", packet_len);

	if (packet_len > len)
		return COM_PACKET_TOO_SHORT;
	else if (packet_len < len)
		return COM_PACKET_TOO_LONG;

	// TODO: Implement CRC Check here.
	// If no error, its vaid

	return COM_PACKET_VALID;
}
