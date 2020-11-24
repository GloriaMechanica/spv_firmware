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
#include "communication.h"


/** @brief  Starts the main system timer. Timeouts and other things will
 * 			get updated.
 *
 *  @param (none)
 *  @return (none)
 */
void TK_startTimer(void)
{
	HAL_TIM_Base_Start_IT(&htim10);
}

/** @brief  Stops the timer (no interrupts anymore). The system time and
 * 			all timeouts are frozen.
 *
 * 			Be careful! Some important timeouts like the magnet safety timeout
 * 			depends on this timer. So better not stop it without thinking!
 *
 *  @param (none)
 *  @return (none)
 */
void TK_stopTimer(void)
{
	HAL_TIM_Base_Start_IT(&htim10);
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
	if (CHA_getIfTimeActive())
	{
		// Check if any of the channels has a datapoint that needs to be executed now
		CHA_updateChannels();

		// And increment the channel time.
		CHA_incrementChannelTime();
	}

	COM_updateTimeout();
}



















