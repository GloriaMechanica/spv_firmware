/*
 * command_def.h
 *
 *  Created on: Feb 25, 2021
 *      Author: josef
 */

#ifndef COMMAND_DEF_H_
#define COMMAND_DEF_H_

// commands that the PC can send to the SPV
#define 	COMM_GETSTATUS 				0x00
#define		COMM_GETMACHINESTATUS 		0x01
#define 	COMM_REQUESTCHANNELFILL		0x02
#define		COMM_SENDDATAPOINTS			0x03
#define		COMM_STARTPLAYING			0x04
#define		COMM_STOPPLAYING			0x05
#define		COMM_CLEARCHANNELS			0x06
#define		COMM_INITCHANNELSTODATA		0x07
#define		COMM_GETCHANNELSREADY		0x08
#define 	COMM_MOVECHANNELTO			0x09

// Tags which SPV returns upon request
#define		COMM_STAT_ID_TAG			0x00
#define		COMM_STAT_ID_LEN			2
#define 	COMM_STAT_TIME_TAG			0x01
#define		COMM_STAT_TIME_LEN			4
#define 	COMM_STAT_RUNNING_TAG		0x02
#define		COMM_STAT_RUNNING_LEN		1
#define		COMM_STAT_CHANNELFILL_TAG	0x03
#define		COMM_STAT_CHANNELFILL_LEN	2
#define		COMM_STAT_CHANNELREADY_TAG	0x04
#define		COMM_STAT_CHANNELREADY_LEN	1

#define 	COMM_STATUS_FIELD_SIZE 	(COMM_STAT_ID_LEN + 2 + COMM_STAT_TIME_LEN + 2 + COMM_STAT_RUNNING_LEN + 2)

// Number of channels which are included in the channelfill - report
#define		COMM_CHANNELFILL_CHANNELS	4
#define		COMM_CHANNELFILL_FIELD_LEN	4

#define 	COMM_COMMAND_POSITION	5

#endif /* COMMAND_DEF_H_ */
