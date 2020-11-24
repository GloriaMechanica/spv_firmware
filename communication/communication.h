/*
 * communication.h
 *
 *  Created on: Nov 24, 2020
 *      Author: josef
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "main.h"

// The good old CAFE is used as a unique ID for UART communication
#define COM_SPV_UID_0		0xCA
#define COM_SPV_UID_1		0xFE
#define COM_MIN_PACKET_LEN	8
#define COM_PACKET_TIMEOUT	50		// If not a complete packet is received within this timeout in [ms], it will be tossed away

typedef struct
{
	int32_t packet_in_buffer; 		// Flag used to indicate when something is in the input buffer
	uint32_t timeout; 				// Used to toss away too small packets after a certian time
}T_COMMUNICATION;

typedef enum
{
	COM_PACKET_VALID,
	COM_PACKET_UID_ERROR,
	COM_PACKET_TOO_LONG,
	COM_PACKET_TOO_SHORT,
	COM_PACKET_SMALLER_MINIMAL_LENGTH,
	COM_PACKET_CRC_ERROR
}E_COM_PACKET_STATUS;

T_COMMUNICATION comm;

void COM_update (void);
void COM_updateTimeout (void);
void COM_restartTimeout (void);
E_COM_PACKET_STATUS COM_checkIfPacketValid(uint8_t *buf, int32_t len);


#endif /* COMMUNICATION_H_ */
