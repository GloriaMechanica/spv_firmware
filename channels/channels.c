/*
 * channels.c
 *
 *  Created on: Nov 21, 2020
 *      Author: josef
 *
 *	This file contains the buffer management (pop, push, read...) for the
 *	channel buffers.
 *
 *	To init the pointers and so on, CHA_init() has to be called at first.
 *	Then, elements can be pushed on a channel, popped from a channel etc.
 *
 */


#include "main.h"
#include "debug_tools.h"
#include "motor_control.h"
#include "channels.h"
#include <string.h>

// Memory allocation for channel buffers
T_DTP_NOTE e_note_buffer[CHA_E_NOTE_LENGTH];
T_DTP_MOTOR posx_dae_buffer[CHA_POSX_DAE_LENGTH];
T_DTP_MOTOR posy_dae_buffer[CHA_POSX_DAE_LENGTH];
T_DTP_MOTOR str_dae_buffer[CHA_POSX_DAE_LENGTH];

// Main structure where channel time is accessed.
T_CHANNEL_TIME channel_time;

/** @brief 	Initializes all the channels presently in use
 *
 *  @param (none)
 *  @return (none)
 */
void CHA_Init(void)
{
	// E_NOTE channel
	cha_e_note.channel_number = CHA_E_NOTE_NR;
	cha_e_note.base = (void*) e_note_buffer;
	cha_e_note.ellen = sizeof(e_note_buffer[0]);
	cha_e_note.buffer_length = CHA_E_NOTE_LENGTH;
	cha_e_note.in = 0; // empty
	cha_e_note.out = 0;
	cha_e_note.last_point_time = 0;

	// POSX_DAE channel
	cha_posx_dae.channel_number = CHA_POSX_DAE_NR;
	cha_posx_dae.base = (void*) posx_dae_buffer;
	cha_posx_dae.ellen = sizeof(posx_dae_buffer[0]);
	cha_posx_dae.buffer_length = CHA_POSX_DAE_LENGTH;
	cha_posx_dae.in = 0;
	cha_posx_dae.out = 0;
	cha_posx_dae.last_point_time = 0;

	// POSY_DAE channel
	cha_posy_dae.channel_number = CHA_POSY_DAE_NR;
	cha_posy_dae.base = (void*) posy_dae_buffer;
	cha_posy_dae.ellen = sizeof(posy_dae_buffer[0]);
	cha_posy_dae.buffer_length = CHA_POSY_DAE_LENGTH;
	cha_posy_dae.in = 0;
	cha_posy_dae.out = 0;
	cha_posy_dae.last_point_time = 0;

	// STR_Z_DAE channel
	cha_str_dae.channel_number = CHA_STR_DAE_NR;
	cha_str_dae.base = (void*) str_dae_buffer;
	cha_str_dae.ellen = sizeof(str_dae_buffer[0]);
	cha_str_dae.buffer_length = CHA_STR_DAE_LENGTH;
	cha_str_dae.in = 0;
	cha_str_dae.out = 0;
	cha_str_dae.last_point_time = 0;
}

/** @brief 	Pushes count elements that can be found in *in on the channel buffer
 * 			with handle *cha
 *
 * 			Be careful! it expects the elements in *in to have length
 * 			cha->ellen each! otherwise strange things will happen.
 *
 *  @param *cha - data structure of channel that should be accessed
 *  @param *in - pointer to input data points (needs to be casted to void)
 *  @param count - number of elements in *in to be pushed
 *  @return 0 if success, -1 if elements do not fit or *in is null pointer
 */
int32_t CHA_pushDatapoints(T_CHANNEL *cha, void *in, int32_t count)
{
	int32_t i;
	// Check if they still fit in
	if (cha->buffer_length - CHA_getNumberDatapoint(cha) < count)
		return -1;

	if (in == NULL)
		return -1;

	for (i = 0; i < count; i++)
	{
		memcpy(cha->base + cha->in*cha->ellen, in + i*cha->ellen, cha->ellen);
		cha->in = (cha->in + 1) % cha->buffer_length;
	}
	return 0;
}

/** @brief 	Popps count elements from channel *cha out on the given pointre
 * 			*out. Popping means they are output and deleted on the buffer.
 *
 *  @param *cha - data structure of channel that should be accessed
 *  @param *out - pointer to memory area where output datapoints can be stored.
 *  			You can pass NULL pointer, it will pop and discard the element.
 *  @param count - number of elements which are popped from the buffer
 *  @return 0 if succes, -1 if there were not enough elements in buffer
 */
int32_t CHA_popDatapoints(T_CHANNEL *cha, void *out, int32_t count)
{
	int32_t i;

	// Check if they still fit in
	if (CHA_getNumberDatapoint(cha) < count)
		return -1;

	for (i = 0; i < count; i++)
	{
		if (out != NULL)
			memcpy(out + i*cha->ellen, cha->base + cha->out*cha->ellen, cha->ellen);
		cha->out = (cha->out + 1) % cha->buffer_length;
	}
	return 0;
}

/** @brief 	Reads count elements from channel *cha out on the given pointer
 * 			*out. Reading means they are only copied to *out but not deleted
 * 			on the input buffer.
 *
 *  @param *cha - data structure of channel that should be accessed
 *  @param *out - pointer to memory area where output datapoints can be stored
 *  @param count - number of elements which are read from the buffer
 *  @return 0 if success, -1 if not enough elements there
 */
int32_t CHA_readDatapoints(T_CHANNEL *cha, void *out, int32_t count)
{
	int32_t i;
	// Check if they still fit in
	if (CHA_getNumberDatapoint(cha) < count)
		return -1;
	int32_t temp_out = cha->out;
	for (i = 0; i < count; i++)
	{
		if (out != NULL)
			memcpy(out + i*cha->ellen, cha->base + temp_out*cha->ellen, cha->ellen);
		temp_out = (temp_out + 1) % cha->buffer_length;
	}
	return 0;
}


/** @brief 	Only returns the pointer to the first (most urgent) element
 * 			in the buffer. It does not write or copy anything, just returns
 * 			the pointer.
 *
 * 			This is intended for the millisecond periodic checking if
 * 			this channel contains a datapoint that nees to be executed now.
 * 			That way, not every millisecond stuff has to be copied.
 *
 * 			Be careful! If the buffer is empty, it will return a pointer to an
 * 			old or uninitialized datapoint.
 *
 *  @param *cha - data structure of channel that should be accessed
 *  @return (none)
 */
void* CHA_peekFirstDatapoint(T_CHANNEL *cha)
{
	return (cha->base + (cha->ellen * cha->out));
}


/** @brief 	Returns how many elements are in buffer
 *
 *  @param *cha - data structure of channel that should be accessed
 *  @return number of elements in buffer
 */
int32_t CHA_getNumberDatapoint(T_CHANNEL *cha)
{
	int32_t diff = cha->in - cha->out;
	if (diff < 0)
		diff += cha->buffer_length;
	return diff;
}

/** @brief 	Clears out the whole buffer
 *
 *  @param *cha - data structure of channel that should be accessed
 *  @return (none)
 */
void CHA_clearBuffer(T_CHANNEL *cha)
{
	cha->in = 0;
	cha->out = 0;
}

/** @brief 	Forces the system time to a certain time
 *
 *  @param time - system time in ms which should be set
 *  @return (none)
 */
void CHA_setChannelTime(uint32_t time)
{
	channel_time.time = time;
}

/** @brief 	Increments the channel time by 1.
 * 			Should be called only by the timer with
 * 			1ms period.
 *
 *  @param (none)
 *  @return (none)
 */
void CHA_incrementChannelTime(void)
{
	channel_time.time += 1;
}

/** @brief 	Returns the current system time
 *
 *  @param (none)
 *  @return the current system time in ms.
 */
uint32_t CHA_getChannelTime(void)
{
	return channel_time.time;
}

/** @brief 	Returns the current system time
 *
 *  @param (none)
 *  @return 0 if the time is stopped, 1 if the time is running
 */
int32_t CHA_getIfTimeActive (void)
{
	return channel_time.time_running;
}

/** @brief 	Sets the SPV channels into action!
 * 			The channel time gets incremented, and interesting
 * 			datapoints will be executed ;-)
 *
 *  @param (none)
 *  @return (none)
 */
void CHA_startTime (void)
{
	channel_time.time_running = 1;
}

/** @brief 	 Stops the channel time (which is music time)
 *
 * 			Be careful! When a motor self-follws a trajectory,
 * 			it will continue to do so even without the channel time
 * 			incrementing. It will, however, not continue when it
 * 			encounters a zero-cycle.
 *
 *  @param (none)
 *  @return (none)
 */
void CHA_stopTime (void)
{
	channel_time.time_running = 0;
}

/** @brief 	This function checks if any of the channels contains a datapoint
 * 			that is due at just this millisecond. If so, it initiates action.
 *
 * 			It gets called once per ms from the timekeeper.
 *
 *  @param (none)
 *  @return (none)
 */
void CHA_updateChannels (void)
{
	if (CHA_getNumberDatapoint(&cha_posx_dae) > 0)
	{
		if(((T_DTP_MOTOR*) CHA_peekFirstDatapoint(&cha_posx_dae))->timediff ==
				CHA_getChannelTime() - cha_posx_dae.last_point_time)
		{
			if (x_dae_motor.status == STG_IDLE)
				SM_setMotorReady(&x_dae_motor);
		}
	}

	if (CHA_getNumberDatapoint(&cha_posy_dae) > 0)
	{
		if(((T_DTP_MOTOR*) CHA_peekFirstDatapoint(&cha_posy_dae))->timediff ==
				CHA_getChannelTime() - cha_posy_dae.last_point_time)
		{
			if (y_dae_motor.status == STG_IDLE)
				SM_setMotorReady(&y_dae_motor);
		}
	}

	if (CHA_getNumberDatapoint(&cha_str_dae) > 0)
	{
		if(((T_DTP_MOTOR*) CHA_peekFirstDatapoint(&cha_str_dae))->timediff ==
				CHA_getChannelTime() - cha_str_dae.last_point_time)
		{
			if (z_dae_motor.status == STG_IDLE)
				SM_setMotorReady(&z_dae_motor);
		}
	}
}
















