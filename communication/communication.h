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
#define COM_BUFFER_SIZE		1024	// Size of buffer holding one command plus data .

typedef struct
{
	uint32_t timeout; 				// Used to toss away too small packets after a certian time
	uint8_t packet_counter; 		// Counter used to enumerate the outgoing packets
	uint8_t buffer[COM_BUFFER_SIZE];// Holds one command plus data bytes.
	int32_t len; 					// length of buffer (including command and all data bytes). No CRC, UID etc.
}T_COMMUNICATION;

typedef enum
{
	COM_PACKET_VALID,
	COM_PACKET_UID_ERROR,
	COM_PACKET_TOO_LONG,
	COM_PACKET_TOO_SHORT,
	COM_PACKET_SMALLER_MINIMAL_LENGTH,
	COM_PACKET_CRC_ERROR,
	COM_PACKET_GENERAL_ERROR
}E_COM_PACKET_STATUS;

T_COMMUNICATION comm;

void COM_update (void);
void COM_updateTimeout (void);
void COM_startTimeout (void);
E_COM_PACKET_STATUS COM_checkIfPacketValid(uint8_t *buf, int32_t len);
E_COM_PACKET_STATUS COM_sendResponse(uint8_t status, uint8_t *data, int32_t len);
void COM_decodePackage(uint8_t *buf, int32_t len);

#endif /* COMMUNICATION_H_ */
