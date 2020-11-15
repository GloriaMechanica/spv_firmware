/** @file step_generation.c
 *  @brief Contains the low level stepper motor stuff (recalculation of timer-preloads, ISR control swapping etc.)
 *  		The functions here run automatically and take their data from the stepper control swap.
 *  		The swap needs to be checked and updated by other routines
 *
 *  @author Josef Heel
	@date March 26th, 2019

	@usage
		o	Call STG_Init first, otherwise some stuff is uninitialized
		o	Use motor_control.h functions to put new cycle settings in the timer ISR control struct
		o	Use STG_StartCycle to push the timer to immediately start
 */

#include "main.h"
#include "device_handles.h"
#include "stm32f7xx_it.h"
#include "debug_tools.h"
#include "step_generation.h"
#include "motor_parameters.h"
#include <math.h>

int32_t 	c_table_xy[C_TABLE_SIZE]; 		// contains timer preload values for each acceleration index n for all x and y axis
int32_t 	c_table_z[C_TABLE_SIZE]; 		// Contains timer preload values for each acceleration index n for both z-axis

// Width of step pulse. Has basically no effect on CPU load, but should not be too short
// in order for the ISR to finish before the next interrupt comes. currently set to 40us.
#define STEP_PULSE_WIDTH 	F_TIMER/25000

// PROTOTYPES
uint16_t step_calculations(T_ISR_CONTROL *isr);
void xy_type_init(T_MOTOR_CONTROL *ctl);
void z_type_init(T_MOTOR_CONTROL *ctl);
void check_cycle_status(T_MOTOR_CONTROL *ctl);
void accel_table_init(int32_t *array, uint32_t length, double acceleration, double alpha);
static int32_t absolute(int32_t arg);

// FUNCTIONS

/** @brief 	Inits the step generation algorithm
 *
 *  @param (none)
 *  @return (none)
 */
void STG_Init (void)
{
	// Calculate acceleration table which is held in RAM for making the ISR as short as possible
	// needs to be done before initializing the individual motors
	accel_table_init(c_table_z, C_TABLE_SIZE, Z_ACCEL_MAX, Z_ALPHA);
	accel_table_init(c_table_xy, C_TABLE_SIZE, XY_ACCEL_MAX, XY_ALPHA);

	// Init X_DAE motor
	x_dae_motor.motor.hw.flip_dir = X_DAE_HW_FLIP_DIR;
	x_dae_motor.motor.hw.dir_port = X_DAE_HW_DIR_PORT;
	x_dae_motor.motor.hw.dir_pin = X_DAE_HW_DIR_PIN;
	x_dae_motor.motor.hw.timer = X_DAE_HW_TIMER;
	x_dae_motor.motor.hw.CCMR = X_DAE_HW_CCMR;
	x_dae_motor.motor.hw.channel = X_DAE_HW_CHANNEL;
	x_dae_motor.motor.hw.oc_mask = X_DAE_HW_OC_MASK;
	x_dae_motor.motor.hw.oc_active_mask = X_DAE_HW_OC_ACTIVE_MASK;
	x_dae_motor.motor.hw.oc_inactive_mask = X_DAE_HW_OC_INACTIVE_MASK;
	x_dae_motor.motor.hw.oc_forced_inactive_mask = X_DAE_HW_OC_FORCED_INACTIVE_MASK;
	xy_type_init(&x_dae_motor);
	HAL_TIM_OC_Start_IT(X_DAE_HW_TIMER, X_DAE_HW_CHANNEL); // Start timer channel for this motor

	// Init Y_DAE motor
	y_dae_motor.motor.hw.flip_dir = Y_DAE_HW_FLIP_DIR;
	y_dae_motor.motor.hw.dir_port = Y_DAE_HW_DIR_PORT;
	y_dae_motor.motor.hw.dir_pin = Y_DAE_HW_DIR_PIN;
	y_dae_motor.motor.hw.timer = Y_DAE_HW_TIMER;
	y_dae_motor.motor.hw.CCMR = Y_DAE_HW_CCMR;
	y_dae_motor.motor.hw.channel = Y_DAE_HW_CHANNEL;
	y_dae_motor.motor.hw.oc_mask = Y_DAE_HW_OC_MASK;
	y_dae_motor.motor.hw.oc_active_mask = Y_DAE_HW_OC_ACTIVE_MASK;
	y_dae_motor.motor.hw.oc_inactive_mask = Y_DAE_HW_OC_INACTIVE_MASK;
	y_dae_motor.motor.hw.oc_forced_inactive_mask = Y_DAE_HW_OC_FORCED_INACTIVE_MASK;
	xy_type_init(&y_dae_motor);
	HAL_TIM_OC_Start_IT(Y_DAE_HW_TIMER, Y_DAE_HW_CHANNEL); // Start timer channel for this motor

	// Init Z_DAE motor
	z_dae_motor.motor.hw.flip_dir = Z_DAE_HW_FLIP_DIR;
	z_dae_motor.motor.hw.dir_port = Z_DAE_HW_DIR_PORT;
	z_dae_motor.motor.hw.dir_pin = Z_DAE_HW_DIR_PIN;
	z_dae_motor.motor.hw.timer = Z_DAE_HW_TIMER;
	z_dae_motor.motor.hw.CCMR = Z_DAE_HW_CCMR;
	z_dae_motor.motor.hw.channel = Z_DAE_HW_CHANNEL;
	z_dae_motor.motor.hw.oc_mask = Z_DAE_HW_OC_MASK;
	z_dae_motor.motor.hw.oc_active_mask = Z_DAE_HW_OC_ACTIVE_MASK;
	z_dae_motor.motor.hw.oc_inactive_mask = Z_DAE_HW_OC_INACTIVE_MASK;
	z_dae_motor.motor.hw.oc_forced_inactive_mask = Z_DAE_HW_OC_FORCED_INACTIVE_MASK;
	z_type_init(&z_dae_motor);
	HAL_TIM_OC_Start_IT(Z_DAE_HW_TIMER, Z_DAE_HW_CHANNEL); // Start timer channel for this motor

}

/** @brief 	Initializes a acceleration ramp table according to the desired acceleration, timer parameters and so on.
 *
 *  @param array - pointer to array for all preload values. Has to be of of lengh "length"
 *  @param length - length of acceleration array.
 *  @param acceleration - that motor will accel/decel with, in [rad/sec^2]
 *  @param alpha - angle which the rotor moves when making one step pulse [rad/step]
 *
 *  @return (none)
 */
void accel_table_init(int32_t *array, uint32_t length, real acceleration, real alpha)
{
	uint32_t i;
	real c_temp = 0.0F;

	// New more exact way to calculate steptable by proper mechanical approach rather than taylor series.
	for (i = 0; i < length-1; i++)
	{
		c_temp = (real) F_TIMER * (real) FACTOR * sqrt(2*alpha/acceleration) * (sqrt(i+1) - sqrt(i));
		array[i] = c_temp;
	}

	/*dbgprintf(" Step table output \n");
	for (i = 0; i < length-1; i++)
	{
		dbgprintf("%d: %d", i, array[i]);
	}*/
}

/** @brief 	Initialisation function that is used for the x/y-axis.
 *
 * 			We need different functions for the z and the x/y axis
 * 			because they have different accelerations.
 *
 *  @param  *ctl - pointer to motor control structure that should be initialized
 *  @return (none)
 */
void xy_type_init(T_MOTOR_CONTROL *ctl)
{
	ctl->motor.pos = 0;
	ctl->motor.acc = XY_ACCEL_MAX;
	ctl->motor.w_max = XY_SPEED_MAX;
	ctl->motor.alpha = XY_ALPHA;
	ctl->motor.c_err = 0;
	ctl->motor.overshoot_on = 0;
	ctl->motor.overshoot_off = 0;

	// Put pointers for c_table in place
	ctl->ctl_swap[0].c_table = c_table_xy;
	ctl->ctl_swap[1].c_table = c_table_xy;

	// Initialize ISR control swap stuff
	ctl->active = &stepper_shutoff; // Motor is stopped at the beginning
	ctl->waiting = &(ctl->ctl_swap[0]); // swap[0] is waiting to be filled.
	ctl->available = 0;
}

/** @brief 	Initialisation function that is used for the z-axis.
 * 			So it will be called once for the z-dae motor and once
 * 			for the z-gda motor.
 *
 * 			We need different functions for the z and the x/y axis
 * 			because they have different accelerations.
 *
 *  @param  *ctl - pointer to motor control structure that should be initialized
 *  @return (none)
 */
void z_type_init(T_MOTOR_CONTROL *ctl)
{
	ctl->motor.pos = 0;
	ctl->motor.acc = Z_ACCEL_MAX;
	ctl->motor.w_max = Z_SPEED_MAX;
	ctl->motor.alpha = Z_ALPHA;
	ctl->motor.c_err = 0;
	ctl->motor.overshoot_on = 0;
	ctl->motor.overshoot_off = 0;

	// Put pointers for c_table in place
	ctl->ctl_swap[0].c_table = c_table_z;
	ctl->ctl_swap[1].c_table = c_table_z;

	// Initialize ISR control swap stuff
	ctl->active = &stepper_shutoff; // Motor is stopped at the beginning
	ctl->waiting = &(ctl->ctl_swap[0]); // swap[0] is waiting to be filled.
	ctl->available = 0;
}

/** @brief 	Usually, the timer swaps its control struct by itself. But if it is the first command or
 * 			a stop cycle was executed, it has to be started again precisely. Therefore it is necessary
 * 			to generate a timer interrupt without (!) generating an output pulse. This is done by this function.
 * 			This function can also be called when a cycle is already in progress and should be restarted for some
 * 			reason. But be careful, it may perform an instant change in speed and therefore loose steps!
 *
 *  @param  *ctl - motor control structure for which the cycle should be started
 *  @return (none)
 */
void STG_StartCycle(T_MOTOR_CONTROL *ctl)
{
	uint16_t tim_preload;

	// First mute the output (The ISR activates it again by itself immediately)
	*(ctl->motor.hw.CCMR) &= ctl->motor.hw.oc_mask;
	*(ctl->motor.hw.CCMR) |= ctl->motor.hw.oc_forced_inactive_mask;

	// Initialize out_state so that it first waits the calculated time and does not generate an interrupt after pulsewidth
	ctl->active->out_state = 0;

	// in case the cycle was already running, we reset it (should usually not be necessary)
	ctl->active->s = 0;
	ctl->active->running = 1;
	ctl->motor.c_err = 0;

	// And finally offset the timer by 1, so that it starts at the next tick (which is statistically 0.5 intervals away).
	tim_preload = __HAL_TIM_GET_COUNTER(ctl->motor.hw.timer) + 1;
	__HAL_TIM_SetCompare(ctl->motor.hw.timer, ctl->motor.hw.channel, tim_preload);

}

/** @brief 	Perfrom an immediate hard stop.
 *
 *  @param *ctl - Motor control struct to operate on.
 *  @return (none)
 */
void STG_hardstop (T_MOTOR_CONTROL *ctl)
{
	ctl->active = &stepper_shutoff;
}


/** @brief 	Makes the waiting ISR control struct active and puts the other of the two
 * 			control structs in waiting. Whatever was in "active" before is thrown away.
 * 			(if e.g. stepper_shutoff was put here). It also sets shutoff to 0, which
 * 			is always set to 1 in a fresh cycle
 *
 *  @param *ctl - Motor control struct to operate on.
 *  @return (none)
 */
void STG_swapISRcontrol (T_MOTOR_CONTROL *ctl)
{
	if (ctl->available == 1)
	{

		ctl->active = ctl->waiting;
		if (ctl->active == &(ctl->ctl_swap[0]))
		{
			ctl->waiting = &(ctl->ctl_swap[1]);
		}
		else
		{
			// Also if active is shutoff, the zero is waiting.
			ctl->waiting = &(ctl->ctl_swap[0]);
		}

		if (ctl->active->shutoff == 1)
		{
			// Maybe something special to do with c_err here?
			// This would be the place to do something for catchup.
		}

		ctl->active->running = 1;
		debug_indicate_cycle_start(ctl->active->s_total, ctl->active->c_ideal/(F_TIMER/1000));
		ctl->available = 0;	// Mark that the now waiting one does not contain valid information.
		toggle_debug_led();
	}
	else
	{
		// for some reason the waiting control sturct has not been updated. As we cannot do anything
		// sensible now, we stop the motor immediately (for now).
		ctl->active = &stepper_shutoff;
		dbgprintf("Swap ISR control: no new struct available");
	}
}



/** @brief This function calculates the new timer preload value according to the current ISR-
 * 			(and hence motor-) state. It must not be called when the step number s_total is exceeded.
 * 			This is made sure in the timer ISR.
 *
 * 			Simple table-based constant acceleration algorithm implemeted. No jerk-limitation yet.
 *
 *  @param  *ctl - 	control structure containing all the low level ISR setup paramters (c, n, s...)
 *  						some of those parameters are modified by this routine (e.g. n, c)
 *  @return timer preload (without FACTOR).
 */
uint16_t step_calculations(T_ISR_CONTROL *isr)
{
	int32_t c_temp;
	if (isr->s < isr->s_on)
	{
		if (isr->s == 0)
		{
			isr->n = isr->neq_on;
		}

		if (isr->no_accel == 1)
		{
			c_temp = isr->c_t;
		}
		else
		{
			c_temp = isr->c_table[absolute(isr->n)];
		}

		isr->n++;

		// Overshoot protection
		if ((c_temp - isr->c_t)*(isr->d_on) >= 0)
		{
			isr->c = c_temp;
		}
		else
		{
			// Count overshoot here for debug purposes
			isr->c = isr->c_t;
			isr->overshoot_on++;
		}
	}
	else if (isr->s >= isr->s_off)
	{
		if (isr->s == isr->s_off)
		{
			isr->n = isr->neq_off;
		}

		if (isr->no_accel == 1)
		{
			c_temp = isr->c_t;
		}
		else
		{
			c_temp = isr->c_table[absolute(isr->n)];
		}

		isr->n++;

		if ((isr->c_t - c_temp)*(isr->d_off) >= 0)
		{
			isr->c = isr->c_t;
			isr->c = c_temp;
		}
		else
		{
			// Count overshoot here for debug purposes
			isr->overshoot_off++;
		}
	}
	else
	{
		// If neighter accel nor decel, just run along at target speed.
		isr->c = isr->c_t;
	}

	// set and return real hardware_count
	isr->c_hw = isr->c / FACTOR;

	// if c_hw is so large that it includes multiple rounds of the timer, it needs to be split in round count and rest.
	isr->c_hwr = (isr->c_hw - STEP_PULSE_WIDTH) / C_MAX;
	isr->c_hwi = (isr->c_hw - STEP_PULSE_WIDTH) % C_MAX;

	// count the used up time to determine error
	isr->c_real += isr->c_hw;

	return isr->c_hw;

}

/** @brief This function checks if the cycle is finished already. If so, it saves the timing error (c_err) and
 * 			swaps out the structs
 *  @param [in/out] *ctl - 	swap structure with both control structures
 *  @return (none)
 */
void check_cycle_status(T_MOTOR_CONTROL *ctl)
{
	// Did we finish this cycle?
	if (ctl->active->s == ctl->active->s_total)
	{
		// Save the difference in timer ticks this cycle produced for information and possibly correction at some point.
		ctl->motor.c_err += ctl->active->c_real - ctl->active->c_ideal;
		ctl->motor.overshoot_on = ctl->active->overshoot_on;
		ctl->motor.overshoot_off = ctl->active->overshoot_off;

		// Swap the buffers and start a new cycle
		STG_swapISRcontrol(ctl);
	}

}

/** @brief Updates...
 *
 * 			timer
 * 			channel
 * 			output pi
 *  @param [in/out] *ctl - 	swap structure with both control structures
 *  @return (none)
 */
void isr_update_stg (T_MOTOR_CONTROL *ctl, uint16_t tim_cnt)
{
	uint16_t 	preload = tim_cnt+1;
	uint16_t	c_hw;

	// First check if the cycle is finished already. This is done only on the falling edge of the step pulse (save interrupt time)
	if (ctl->active->out_state == 0 && ctl->active->c_hwr == 0  && ctl->active->running == 1
			&& ctl->active->shutoff == 0)
	{
		// If yes, the swap is performed here, so from here on all ctl->active values changed!
		check_cycle_status(ctl);
	}

	if (ctl->active->running == 1 && ctl->active->shutoff == 0)
	{
		if (ctl->active->out_state == 1)
		{
			// We've just generated a positive edge and moved the motor.
			// So in a couple of us, we will reset the step line to 0. (second part of this if)
			// Configure timer so the pin goes low at the next overflow
			ctl->active->out_state = 0;
			*(ctl->motor.hw.CCMR) &= ~(ctl->motor.hw.oc_mask);
			*(ctl->motor.hw.CCMR) |= ctl->motor.hw.oc_inactive_mask;
			// Intermediate step to generate small pulse
			preload = tim_cnt + STEP_PULSE_WIDTH;
			// now a step has been done
			ctl->motor.pos += ctl->active->dir_abs;
			// relative step counter is always positive
			ctl->active->s++;
		}
		else if (ctl->active->out_state == 0)
		{
			// The step line is at 0 again. We need to calculate how long it takes to the next step.
			if (ctl->active->c_hwr == 0)
			{
				// We've completed all subsequent full rounds of the timer and process the next step.
				// Recalculate timer preload out of old or new control struct
				c_hw = step_calculations(ctl->active);
				debug_push_preload(c_hw); // we need to push the full preload, not every individual round.

				preload = tim_cnt + ctl->active->c_hwi; // the PULSE_WIDTH is included in the step calculation already.

				// generate an edge at the next match if no rounds left
				if (ctl->active->c_hwr == 0)
				{
					ctl->active->out_state = 1;
					*(ctl->motor.hw.CCMR) &= ~(ctl->motor.hw.oc_mask);
					*(ctl->motor.hw.CCMR) |= ctl->motor.hw.oc_active_mask;
				}
			}
			else if (ctl->active->c_hwr > 0)
			{
				// We've already waited the fraction of a round, but need to wait more full rounds.
				preload = tim_cnt + C_MAX;
				ctl->active->c_hwr--;

				// generate a tick at the next match if no rounds left
				if (ctl->active->c_hwr == 0)
				{
					ctl->active->out_state = 1;
					*(ctl->motor.hw.CCMR) &= ~(ctl->motor.hw.oc_mask);
					*(ctl->motor.hw.CCMR) |= ctl->motor.hw.oc_active_mask;
				}
			}
			else
			{
				// should never come here.
				ctl->active->c_hwr = 0;
			}

			// Take care of direction pin.
			switch(ctl->active->dir_abs * ctl->motor.hw.flip_dir)
			{
			case  1: HAL_GPIO_WritePin(ctl->motor.hw.dir_port, ctl->motor.hw.dir_pin, GPIO_PIN_SET); break;
			case -1: HAL_GPIO_WritePin(ctl->motor.hw.dir_port, ctl->motor.hw.dir_pin, GPIO_PIN_RESET); break;
			}
		}

		// Only preset compare reg if neccesary
		__HAL_TIM_SetCompare(ctl->motor.hw.timer, ctl->motor.hw.channel, preload);

	}
	else
	{
		// Force off output, so that it does not randomly tick along
		*(ctl->motor.hw.CCMR) &= ~(ctl->motor.hw.oc_mask);
		*(ctl->motor.hw.CCMR) |= ctl->motor.hw.oc_forced_inactive_mask;
	}

}

/** @brief If argument is negative, this function returns -argument. Absolute.
 *  @param [in/out] *ctl - 	swap structure with both control structures
 *  @return (none)
 */
static int32_t absolute(int32_t arg)
{
	if (arg < 0)
		return -arg;

	return arg;
}


