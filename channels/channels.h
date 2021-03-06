/*
 * channels.h
 *
 *  Created on: Nov 21, 2020
 *      Author: josef
 */
#include "datapoint_def.h"

#ifndef CHANNELS_H_
#define CHANNELS_H_

#define 	CHA_NUMBER_CHANNELS_TOTAL 	14

// Numbers for channels according to specification
// Be careful: when changing something here, you also need to change the order in cha_list[] in channels.c
#define		CHA_E_NOTE_NR		3
#define		CHA_POSX_DAE_NR		4
#define		CHA_POSY_DAE_NR		5
#define		CHA_STR_DAE_NR		6

// Number of datapoints in each channel (can be adjusted individually to have more buffer for more active channels such as STR_DAE)
#define CHA_G_NOTE_LENGTH 		50
#define CHA_D_NOTE_LENGTH 		50
#define CHA_A_NOTE_LENGTH 		50
#define CHA_E_NOTE_LENGTH 		50
#define CHA_POSX_DAE_LENGTH 	50
#define CHA_POSY_DAE_LENGTH 	50
#define CHA_STR_DAE_LENGTH 		50
#define CHA_POSX_GDA_LENGTH 	50
#define CHA_POSY_GDA_LENGTH 	50
#define CHA_STR_GDA_LENGTH 		50

/*
 * Main handle for one channel
 * It basically is the handle for a ringbuffer
 */
typedef struct
{
	uint8_t	channel_number; // characteristical number (spec). Just set to const and read from there
	int32_t buffer_length; // Number of datapoints in this buffer
	int32_t ellen; // element length: because void pointers are used, to increment them
	void*	base; // pointer to incoming element
	int32_t in; // index of incoming element which is empty and ready to write on (array-like)
	int32_t out; // index of outgoing element which is filled and ready to be read (array-like numeration)
	uint32_t last_point_time; // used to keep the time stamp of the last event to be able to check when the relative time has elapsed
}T_CHANNEL;

/*
 * Time structure for channel time
 * The main music time for all motor and note channels
 * is stored here. It can be stopped.
 *
 * in milliseconds from start of song
// 32 bit allow over 1000h of music, so no worries about overflow ;-)
 */
typedef struct
{
	volatile int32_t	time_running; 	// if 0, the channel time is not incremented AND the channels are not checked
	volatile uint32_t 	time;	// main music time [ms]

}T_CHANNEL_TIME;

// Allocation of channel buffer handles
T_CHANNEL cha_g_note;
T_CHANNEL cha_d_note;
T_CHANNEL cha_a_note;
T_CHANNEL cha_e_note;
T_CHANNEL cha_posx_dae;
T_CHANNEL cha_posy_dae;
T_CHANNEL cha_str_dae;
T_CHANNEL cha_posx_gda;
T_CHANNEL cha_posy_gda;
T_CHANNEL cha_str_gda;
T_CHANNEL cha_g_vib;
T_CHANNEL cha_d_vib;
T_CHANNEL cha_a_vib;
T_CHANNEL cha_e_vib;

// an array with all the channel pointers (for easy selection by channel number)
extern T_CHANNEL *cha_list[CHA_NUMBER_CHANNELS_TOTAL];

// Prototypes
void CHA_Init(void);
int32_t CHA_pushDatapoints(T_CHANNEL *cha, void *in, int32_t count);
int32_t CHA_popDatapoints(T_CHANNEL *cha, void *out, int32_t count);
int32_t CHA_readDatapoints(T_CHANNEL *cha, void *out, int32_t count);
int32_t CHA_getNumberDatapoint(T_CHANNEL *cha);
void* CHA_peekFirstDatapoint(T_CHANNEL *cha);
void CHA_clearBuffer(T_CHANNEL *cha);
void CHA_updateChannels (void);
void CHA_setChannelTime(uint32_t time);
void CHA_incrementChannelTime(void);
uint32_t CHA_getChannelTime(void);
int32_t CHA_getIfTimeActive (void);
void CHA_startTime (void);
void CHA_stopTime (void);
void CHA_startPlaying (void);
void CHA_stopPlaying (void);
void CHA_setRelativeExecutionTime(uint32_t time);





#endif /* CHANNELS_H_ */
