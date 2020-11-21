/*
 * timekeeper.c
 *
 *  Created on: Nov 21, 2020
 *      Author: josef
 */

#include "main.h"
#include "debug_tools.h"
#include "device_handles.h"
#include "motor_control.h"
#include "channels.h"
#include <string.h>

// Global system time (only local here, accessed through setter and getter functions)
// in milliseconds from start of song
// 32 bit allow over 1000h of music, so no worries about overflow ;-)
volatile uint32_t systime;

/** @brief 	Forces the system time to a certain time
 *
 *  @param time - system time in ms which should be set
 *  @return (none)
 */
void TK_setSystime(uint32_t time)
{
	systime = time;
}

/** @brief 	Returns the current system time
 *
 *  @param (none)
 *  @return the current system time in ms.
 */
uint32_t TK_getSystime(void)
{
	return systime;
}

/** @brief 	Enables the timer and causes the system time to self-increment
 *
 *  @param (none)
 *  @return (none)
 */
void TK_startTimer (void)
{
	HAL_TIM_Base_Start_IT(&htim10);
}

/** @brief  Stops the timer (no interrupts anymore). The system time is frozen
 * 			No new channel events will get executed.
 *
 * 			Be careful: When a motor is self-following a trajectory,
 * 			it will not stop because of stopping the timer. Only if
 * 			it encounters a zero-cycle, it will remain forever in it.
 * 			(Which could be after many seconds).
 *
 *  @param (none)
 *  @return (none)
 */
void TK_stopTimer (void)
{
	HAL_TIM_Base_Stop_IT(&htim10);
}

/** @brief 	ISR callback which gets executed every millisecond if TK timer
 * 			is running.
 *
 *  @param (none)
 *  @return (none)
 */
void isr_tk_millisecond (void)
{
	// DEBUG
	// Check if time advanced to match a datapoint.

	if (CHA_getNumberDatapoint(&cha_posx_dae) > 0)
	{
		if(((T_DTP_MOTOR*) CHA_peekFirstDatapoint(&cha_posx_dae))->timestamp == TK_getSystime())
		{
			if (x_dae_motor.status == STG_IDLE)
				SM_setMotorReady(&x_dae_motor);
		}
	}

	if (CHA_getNumberDatapoint(&cha_posy_dae) > 0)
	{
		if(((T_DTP_MOTOR*) CHA_peekFirstDatapoint(&cha_posy_dae))->timestamp == TK_getSystime())
		{
			if (y_dae_motor.status == STG_IDLE)
				SM_setMotorReady(&y_dae_motor);
		}
	}

	if (CHA_getNumberDatapoint(&cha_str_dae) > 0)
	{
		if(((T_DTP_MOTOR*) CHA_peekFirstDatapoint(&cha_str_dae))->timestamp == TK_getSystime())
		{
			if (z_dae_motor.status == STG_IDLE)
				SM_setMotorReady(&z_dae_motor);
		}
	}


	// Increment the time...
	systime += 1;
}



















