/*
 * communication.c
 *
 * This software module handles all the communication with the PC, i.e. the SPVplayer
 *
 *  Created on: Nov 24, 2020
 *      Author: josef
 */

#include "main.h"
#include <string.h>
#include "communication.h"
#include "motor_control.h"
#include "usb_cdc_comm.h"
#include "debug_tools.h"
#include "command_def.h"
#include "settings.h"

// PROTOTYPES
unsigned short crc16(const unsigned char* data_p, unsigned char length);

/** @brief  Initializes communication stuff.
 *
 *  @param (none)
 *  @return (none)
 */
void COM_init(void)
{
	comm.timeout = 0;
	comm.packet_counter = 0;
}

/** @brief 	Is periodically called from the main loop and checks if new
 * 			commands have entered via the USB-CDC interface.
 *
 * 			From here, all the package decoding is done.
 *
 *  @param (none)
 *  @return (none)
 */
void COM_update (void)
{
	if (usb_cdc_rx_buffer.packet_in_buffer==1)
	{
		// There is a good packet in the buffer -> decode and execute it
		COM_decodePackage(&(usb_cdc_rx_buffer.data[COMM_COMMAND_POSITION]), usb_cdc_rx_buffer.top - (COM_MIN_PACKET_LEN));

		// Clear the buffer at the end, so that new commands are correctly interpreted
		USB_CDC_clearRxBuffer();
	}
	else if (usb_cdc_rx_buffer.packet_in_buffer == (-1))
	{
		// There is a bad packet in the buffer, and no more is going to come.
		USB_CDC_clearRxBuffer(); // We don't need the bad packet anymore
		COM_sendResponse(NACK, NULL, 0); // Indicate by a NACK that something is wrong.
	}

}

/** @brief 	Decodes the package and gets the require stuff going!
 *			It needs to be passed a stripped package containing
 *			only the command and the data bytes!
 *
 *  @param *buf - pointer to buffer, where buf[0] is CMD, the rest is data (CRC is excluded already)
 *  @param len - lenth of data field. len = number of data bytes + 1 command byte
 *  @return (none)
 */
void COM_decodePackage(uint8_t *buf, int32_t len)
{
#if DEBUG_ENABLE_UART_LOGGING
	dbgprintf("Command: %01X", buf[0]);
	dbgprintf("Data length: %d", len);
#endif
	uint8_t command = buf[0];

	// -----------------------------------------------------
	if (command == COMM_GETSTATUS)
	{
		// PC requested the status of the SPV
		uint8_t data[COMM_STATUS_FIELD_SIZE];
		uint8_t *ptr = &(data[0]);

		*(ptr++) = COMM_STAT_ID_TAG;
		*(ptr++) = COMM_STAT_ID_LEN;
		*(ptr++) = SOFTWARE_ID_0;
		*(ptr++) = SOFTWARE_ID_1;

		uint32_t timestamp = CHA_getChannelTime();
		*(ptr++) = COMM_STAT_TIME_TAG;
		*(ptr++) = COMM_STAT_TIME_LEN;
		memcpy(ptr, &timestamp, COMM_STAT_TIME_LEN);
		ptr += COMM_STAT_TIME_LEN;

		*(ptr++) = COMM_STAT_RUNNING_TAG;
		*(ptr++) = COMM_STAT_RUNNING_LEN;
		*(ptr++) = CHA_getIfTimeActive() & 0x000000FF;

		// DO NOT FORGET to adapt COMM_STATUS_FIELD_SIZE when adding new status fields here.

		COM_sendResponse(ACK, data, sizeof(data));
	}
	// -----------------------------------------------------
	else if (command == COMM_GETMACHINESTATUS)
	{
		// PC requested the machine status (position and movement of all axis)
		uint8_t data[COMM_STAT_AXISSTATUS_FIELD_SIZE];
		uint8_t *ptr = &(data[0]);

		T_MOTOR_CONTROL *axis[COMM_AXISSTATUS_AXIS];
		axis[0] = &x_dae_motor;
		axis[1] = &y_dae_motor;
		axis[2] = &z_dae_motor;

		T_CHANNEL *channels[COMM_AXISSTATUS_AXIS];
		channels[0] = &cha_posx_dae;
		channels[1] = &cha_posy_dae;
		channels[2] = &cha_str_dae;

		int16_t pos;

		int i;
		for (i = 0; i < 3; i++)
		{
			*(ptr++) = COMM_STAT_AXISSTATUS_TAG;
			*(ptr++) = COMM_STAT_AXISSTATUS_LEN;
			*(ptr++) = channels[i]->channel_number;
			*(ptr++) = axis[i]->motor.scheduled_pos == axis[i]->motor.pos ? 0x00 : 0x01;
			pos = (axis[i]->motor.pos);
			*(ptr++) =  pos & 0x00FF;
			*(ptr++) = (pos >> 8) & 0x00FF;
		}

		// DO NOT FORGET to adapt COMM_STAT_AXISSTATUS_FIELD_SIZE when adding new status fields here.

		COM_sendResponse(ACK, data, sizeof(data));
	}
	// -----------------------------------------------------
	else if (command == COMM_REQUESTCHANNELFILL)
	{
		// PC wants to know how many points are missing (empty) in each channel.
		uint8_t data[COMM_CHANNELFILL_CHANNELS*COMM_CHANNELFILL_FIELD_LEN];
		uint8_t *ptr = &(data[0]);
		uint8_t missing = 0;

		T_CHANNEL *channels[COMM_CHANNELFILL_CHANNELS];
		channels[0] = &cha_e_note;
		channels[1] = &cha_posx_dae;
		channels[2] = &cha_posy_dae;
		channels[3] = &cha_str_dae;
		// DO NOT FORGET to adapt COMM_CHANNELFILL_CHANNELS when adding more channels here!

		int32_t i;
		for (i = 0; i < COMM_CHANNELFILL_CHANNELS; i++)
		{
			missing = channels[i]->buffer_length - CHA_getNumberDatapoint(channels[i]);
			if (missing > 0)
			{
				*(ptr++) = COMM_STAT_CHANNELFILL_TAG;
				*(ptr++) = COMM_STAT_CHANNELFILL_LEN;
				*(ptr++) = channels[i]->channel_number;
				*(ptr++) = missing;
			}
		}
		COM_sendResponse(ACK, data, (int)(ptr - &(data[0])));
	}
	// -----------------------------------------------------
	else if (command == COMM_SENDDATAPOINTS)
	{
		dbgprintf("Got some datapoints...");
		int i = 1;
		int channel_nr = 0;
		int number_points = 0;
		// I admit, not so much overflow checking is done here.
		// But we do not expect hackers to do something bad between the PC and the SPV
		// and the python program should provide the correct field lengths etc.
		while(i < len)
		{
			channel_nr = buf[i++];
			number_points = buf[i++];
			if (channel_nr >= 0 && channel_nr < CHA_NUMBER_CHANNELS_TOTAL)
			{
				CHA_pushDatapoints(cha_list[channel_nr], &(buf[i]), number_points);
				i += (cha_list[channel_nr])->ellen * number_points;
				dbgprintf("Added %d points of length %d to channel %d", number_points, (cha_list[channel_nr])->ellen, channel_nr);
			}

		}
		COM_sendResponse(ACK, NULL, 0);
	}
	// -----------------------------------------------------
	else if (command == COMM_STARTPLAYING)
	{
		dbgprintf("Start Playing command!");
		CHA_startPlaying();
		COM_sendResponse(ACK, NULL, 0);
	}
	// -----------------------------------------------------
	else if (command == COMM_STOPPLAYING)
	{
		dbgprintf("Stop Playing command!");
		CHA_stopPlaying();
		SM_softstop();
		COM_sendResponse(ACK, NULL, 0);
	}
	// -----------------------------------------------------
	else if (command == COMM_CLEARCHANNELS)
	{
		dbgprintf("Clear all channels");
		CHA_Init();
		COM_sendResponse(ACK, NULL, 0);
	}
	// -----------------------------------------------------
	else if (command == COMM_MOVECHANNELTO)
	{
		dbgprintf("Move Channel to");
		uint8_t channel_nr = buf[1];
		int32_t position = buf[3] << 8 | buf[2];
		real speed = (double) buf[4];

		dbgprintf("Should move to pos=%d at speed=%d", position, (int)speed);

		if (channel_nr == CHA_E_NOTE_NR)
		{
			// TODO: Not implemented yet
		}
		else if (channel_nr == CHA_POSX_DAE_NR)
		{
			SM_moveMotorToLocation(&x_dae_motor, (int32_t) position, speed);
		}
		else if (channel_nr == CHA_POSY_DAE_NR)
		{
			SM_moveMotorToLocation(&y_dae_motor, (int32_t) position, speed);
		}
		else if (channel_nr == CHA_STR_DAE_NR)
		{
			SM_moveMotorToLocation(&z_dae_motor, (int32_t) position, speed);
		}

		COM_sendResponse(ACK, NULL, 0);
	}
	// -----------------------------------------------------
	else if (command == COMM_MOVECHANNELRELATIVE)
	{
		dbgprintf("Move Channel relative");
		uint8_t channel_nr = buf[1];
		int16_t pos_diff = buf[3] << 8 | buf[2];
		real speed = (double) buf[4];
		uint8_t acknowledge = ACK;

		dbgprintf("Should move by %d steps at speed=%d", pos_diff, (int)speed);

		if (channel_nr == CHA_POSX_DAE_NR)
		{
			SM_moveMotorRelative(&x_dae_motor, pos_diff, speed);
		}
		else if (channel_nr == CHA_POSY_DAE_NR)
		{
			SM_moveMotorRelative(&y_dae_motor, pos_diff, speed);
		}
		else if (channel_nr == CHA_STR_DAE_NR)
		{
			SM_moveMotorRelative(&z_dae_motor, pos_diff, speed);
		}
		else
		{
			// This command is not supported for other channels
			acknowledge = NACK;
		}
		COM_sendResponse(acknowledge, NULL, 0);
	}
	// -----------------------------------------------------
	else if (command == COMM_REFERENCECHANNEL)
	{
		dbgprintf("Reference Channel");
		uint8_t channel_nr = buf[1];
		real speed = (double) buf[2];
		uint8_t acknowledge = ACK;

		if (channel_nr == CHA_POSX_DAE_NR)
		{
			SM_referenceMotor(&x_dae_motor, speed);
		}
		else if (channel_nr == CHA_POSY_DAE_NR)
		{
			SM_referenceMotor(&y_dae_motor, speed);
		}
		else if (channel_nr == CHA_STR_DAE_NR)
		{
			SM_referenceMotor(&z_dae_motor, speed);
		}
		else
		{
			// This command is not supported for other channels
			acknowledge = NACK;
		}
		COM_sendResponse(acknowledge, NULL, 0);
	}
	// -----------------------------------------------------
	else
	{
		dbgprintf("Unknown command.");
		COM_sendResponse(NACK, NULL, 0);
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
		dbgprintf("Receive Timeout expired. Tossed away packet!");
	}
}

/** @brief 	Called from UART Rx handler. Sets the timeout because
 * 			fresh bytes have been received. If nothing comes to
 * 			complete the packet within the timeout, it will be tossed away.
 *
 *  @param (none)
 *  @return (none)
 */
void COM_startTimeout (void)
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
	comm.timeout = 0;
}

/** @brief 	Is periodically called from the main loop and checks if new
 * 			commands have entered via the USB-CDC interface.
 *
 * 			so far it checks:
 * 				o	not too short
 * 				o	UID correct
 * 				o	CRC correct
 * 				o	package has the length it says
 *
 *  @param 	*buf - pointer to buffer where data is
 *  		len - length the function is allowed to read
 *  @return (none)
 */
E_COM_PACKET_STATUS COM_checkIfPacketValid(uint8_t *buf, int32_t len)
{
	uint16_t packet_len = 0;
	uint16_t crc_calc = 0, crc_send = 0;

	if (len < COM_MIN_PACKET_LEN || len < 2) // < 2 is for safety that the next if does not produce a segfault
		return COM_PACKET_SMALLER_MINIMAL_LENGTH;

	if (buf[0] != COM_SPV_UID_0 && buf[1] != COM_SPV_UID_1)
		return COM_PACKET_UID_ERROR;

	packet_len = buf[2] << 8 | buf[3];
	crc_send = buf[len-2] << 8 | buf[len-1];
	crc_calc = crc16(buf, len-2); // len-2 because CRC bytes are not included in calculation
#if DEBUG_ENABLE_UART_LOGGING
	dbgprintf("crc send: %02X vs. calculated %02X", crc_send, crc_calc);
#endif

	if (packet_len > len)
		return COM_PACKET_TOO_SHORT;
	else if (packet_len < len)
		return COM_PACKET_TOO_LONG;
	else if (crc_send != crc_calc)
		return COM_PACKET_CRC_ERROR;
#if DEBUG_ENABLE_UART_LOGGING
	dbgprintf("packet says len= %d", packet_len);
#endif

	return COM_PACKET_VALID;
}

/** @brief 	Sends a response packet to the PC.
 *
 *  @param status - 0 means ACK, 1 means NACK, other values can be used to indicate errors (later)
 *  @param *data - buffer containing len data bytes to be attached as data to the packet
 *  @param len - length of data bytes to send
 *  @return COM_PACKET_VALID if success, different errors otherwise.
 */
E_COM_PACKET_STATUS COM_sendResponse(uint8_t status, uint8_t *data, int32_t len)
{
	uint16_t crc_calc = 0;

	if (len > 0xFFFF)
		return COM_PACKET_TOO_LONG;
	else if (len < 0)
		return COM_PACKET_TOO_SHORT;
	else if (len != 0 && data == NULL)
		return COM_PACKET_GENERAL_ERROR;

	// Assemble packet in temporary buffer
	uint8_t buf[len + COM_MIN_PACKET_LEN];
	buf[0] = COM_SPV_UID_0;
	buf[1] = COM_SPV_UID_1;
	buf[2] = ((len+8) & 0x0000FF00) >> 8;
	buf[3] = ((len+8) & 0x000000FF);
	buf[4] = comm.packet_counter++;
	buf[5] = status;
	if (len != 0)
		memcpy(&(buf[6]), data, len);
	crc_calc = crc16(buf, 6+len);
	buf[6+len] = (crc_calc & 0xFF00) >> 8;
	buf[6+len+1] = crc_calc & 0x00FF;

	if (USB_CDC_TransmitBuffer(buf, len+8) == SUCCESS)
		return COM_PACKET_VALID;
	return COM_PACKET_GENERAL_ERROR;
}


/** @brief 	CRC16 calculation function. Code is from Stackexchange:
 * 			https://stackoverflow.com/questions/10564491/function-to-calculate-a-crc16-checksum
 *
 *  @param 	*data_p - pointer to data bytes
 *  		length - length of buffer
 *  @return 16 bit CRC of given data field.
 */
unsigned short crc16(const unsigned char* data_p, unsigned char length){
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
}
