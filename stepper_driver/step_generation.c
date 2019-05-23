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

// Is used in ISR. Contains the current CNT-register state of timer1
uint16_t 	tim1_cnt;
uint16_t 	tim1_c_hw;
uint16_t 	tim1_preload;
int32_t 	c_table_z[C_TABLE_SIZE]; 		// Contains timer preload values for each acceleration index n for both z-axis
int32_t 	c_table_xy[C_TABLE_SIZE]; 		// contains timer preload values for each acceleration index n for all x and y axis

// DAE - Z - Axis
uint8_t z_dae_output_state;

#define STEP_PULSE_WIDTH 	10

// PROTOTYPES
uint16_t step_calculations(T_ISR_CONTROL *ctl);
void z_dae_init(T_STEPPER_STATE* stat, T_ISR_CONTROL_SWAP *swap);
void check_cycle_status(T_ISR_CONTROL_SWAP *ctl, T_STEPPER_STATE *state);
void accel_table_init(int32_t *array, uint32_t length, double acceleration, double alpha);

// FUNCTIONS

/** @brief 	Inits the step generation algorithm
 *
 *  @param (none)
 *  @return (none)
 */
void STG_Init (void)
{
	// Init the individual motors
	z_dae_init(&z_dae_motor, &z_dae_swap);

	// Calculate acceleration table which is held in RAM for making the ISR as short as possible
	accel_table_init(c_table_z, C_TABLE_SIZE, Z_ACCEL_MAX, Z_ALPHA);

	// Start timers
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
}

/** @brief 	Initializes a acceleration ramp table according to the desired acceleration, timer parameters and so on.
 *
 *  @param array - pointer to array for all preload values. Has to be of of lengh "length"
 *  @param length - length of acceleration array.
 *  @param acceleration - well, just that ... [rad/sec^2]
 *  @param alpha - angle which the rotor moves when making one step pulse [rad/step]
 *
 *  @return (none)
 */
void accel_table_init(int32_t *array, uint32_t length, real acceleration, real alpha)
{
	int32_t c_0;
	uint32_t i;

	// Calculate start preload
	c_0 = F_TIMER * sqrt(2*alpha/acceleration) * CORR0; // * FACTOR / FACTOR, because CORR0 contains it already.

	array[0] = c_0;
	for (i = 0; i < length; i++)
	{
		// Const acceleration calculation according to D. Austin
		array[i + 1] = array[i] - 2 * array[i] / (4*i + 1);
	}
}

/** @brief 	Initialisation of motor state and swap buffer for Z-DAE-Axis
 *
 *  @param (none)
 *  @return (none)
 */
void z_dae_init(T_STEPPER_STATE* stat, T_ISR_CONTROL_SWAP *swap)
{
	// Z Axis on DAE strings
	stat->pos = 0;
	stat->acc = Z_ACCEL_MAX;
	stat->w_max = Z_SPEED_MAX;
	stat->alpha = Z_ALPHA;

	// Initialize isr control swap stuff
	swap->active = &stepper_shutoff;
	swap->waiting = &(z_dae_swap.z_dae_control[0]);
	swap->available = 0;
}

/** @brief 	Usually, the timer swaps its control struct by itself. But if it is the first command or
 * 			a stop cycle was executed, it has to be started again precisely. Therefore it is necessary
 * 			to generate a timer interrupt without (!) generating an output pulse. This is done by this function.
 * 			This function can also be called when a cycle is already in progress and should be restarted for some
 * 			reason. But be careful, it may perform an instant change in speed and therefore loose steps!
 *
 *  @param [in/out] *ctl - 	swap structure with both control structures
 *  @return (none)
 */
void STG_StartCycle(T_ISR_CONTROL_SWAP *ctl)
{
	uint16_t tim_preload;

	// Z-DAE Axis
	if(ctl == &z_dae_swap)
	{
		// First mute the output (The ISR activates it again by itself immediately)
		htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
		htim1.Instance->CCMR1 |= TIM_OCMODE_FORCED_INACTIVE;

		// Initialize out_state so that it first waits the calculated time and does not generate an interrupt after pulsewidth
		ctl->active->out_state = 0;

		// in case the cycle was already running, we reset it (should ususally not be necessary)
		ctl->active->s = 0;
		ctl->active->running = 1;

		// And finally offset the timer by 1, so that it starts at the next tick (which is statistically 0.5 intervals away).
		tim_preload = __HAL_TIM_GET_COUNTER(&htim1) + 1;
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, tim_preload);
	}
}


/** @brief 	Makes the waiting ISR control struct active and puts the other of the two
 * 			control structs in waiting. Whatever was in "active" before is thrown away.
 * 			(if e.g. stepper_shutoff was put here). It also sets shutoff to 0, which
 * 			is always set to 1 in a fre
 *
 *  @param (none)
 *  @return (none)
 */
void STG_swapISRcontrol (T_ISR_CONTROL_SWAP * ctl)
{
	if (ctl->available == 1)
	{
		ctl->active = ctl->waiting;
		if (ctl->active->shutoff == 1)
		{
			dbgprintf("Swapped in stop-struct. c_err = %d", z_dae_motor.c_err);
		}
		if (ctl->active == &(ctl->z_dae_control[0]))
		{
			ctl->waiting = &(ctl->z_dae_control[1]);
			  toggle_debug_led();
		}
		else
		{
			ctl->waiting = &(ctl->z_dae_control[0]);
		}

		ctl->active->running = 1;
		ctl->available = 0;	// Mark that the now waiting one does not contain valid information.
	}
	else
	{
		// for some reason the waiting control sturct has not been updated. As we cannot do anything
		// sensible now, we stop the motor immediately (for now).
		ctl->active = &stepper_shutoff;
	}
}



/** @brief This function calculates the new timer preload value according to the current ISR-
 * 			(and hence motor-) state. It must not be called when the step number s_total is exceeded.
 * 			This is made sure in the timer ISR.
 *
 * 			Simple table-based constant acceleration algorithm implemeted. No jerk-limitation yet.
 *
 *  @param [in/out] *ctl - 	control structure containing all the low level ISR setup paramters (c, n, s...)
 *  						some of those parameters are modified by this routine (e.g. n, c)
 *  @return timer preload (without FACTOR).
 */
uint16_t step_calculations(T_ISR_CONTROL *ctl)
{
	uint16_t c_temp;
	if (ctl->s < ctl->s_on)
	{
		if (ctl->s == 0)
		{
			ctl->n = ctl->neq_on;
		}

		if (ctl->no_accel == 1)
		{
			c_temp = ctl->c_t;
		}
		else
		{
			c_temp = ctl->c_table[ctl->n];
		}

		ctl->n++;

		// Overshoot protection
		if ((c_temp - ctl->c_t)*ctl->d_on >= 0)
		{
			ctl->c = c_temp;
		}
		else
		{
			// Count overshoot here for debug purposes
		}
	}
	else if (ctl->s >= ctl->s_off)
	{
		if (ctl->s == ctl->s_off)
		{
			ctl->n = ctl->neq_off;
		}

		if (ctl->no_accel == 1)
		{
			c_temp = ctl->c_t;
		}
		else
		{
			c_temp = ctl->c_table[ctl->n];
		}

		ctl->n++;

		if ((ctl->c_t - c_temp)*ctl->d_off >= 0)
		{
			ctl->c = c_temp;
		}
		else
		{
			// Count overshoot here for debug purposes
		}
	}
	else
	{
		// If neighter accel nor decel, just run along at target speed.
		ctl->c = ctl->c_t;
	}

	// set and return real hardware_count
	ctl->c_hw = ctl->c / FACTOR;

	// count the used up time to determine error
	ctl->c_real += ctl->c_hw;

	return ctl->c_hw;

}

/** @brief This function checks if the cycle is finished already. If so, it saves the timing error (c_err) and
 * 			swaps out the structs
 *  @param [in/out] *ctl - 	swap structure with both control structures
 *  @return (none)
 */
void check_cycle_status(T_ISR_CONTROL_SWAP *ctl, T_STEPPER_STATE *state)
{
	// Did we finish this cycle?
	if (ctl->active->s == ctl->active->s_total)
	{
		// Save the difference in timer ticks this cycle produced for information and possibly correction at some point.
		state->c_err += ctl->active->c_real - ctl->active->c_ideal;

		// Swap the buffers and start a new cycle
		STG_swapISRcontrol(ctl);
	}
}

void tim1_cc_irq_handler (void)
{
	// We make a local pointer for copy and paste purposes of the other axis.
	T_ISR_CONTROL_SWAP *ctl;
	T_STEPPER_STATE *state;

	// Read out current timer state
	tim1_cnt = __HAL_TIM_GET_COUNTER(&htim1);

	// Check if the interrupt was created by CC1, then its for Z_DAE axis
	// ---------------------------------------------------------------------------------------
	  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1) != RESET)
	  {
	    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC1) != RESET)
	    {
	    	// Clear interrupt source, otherwise interrupt is permanently executed
	    	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
	    	htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_1;

	    	// CCINT1 is used for Z_DAE_axis, so we switch the control and state structs to it
	    	ctl = &z_dae_swap;
	    	state = &z_dae_motor;

	    	if (ctl->active->running == 1 && ctl->active->shutoff == 0)
	    	{
	    		if (ctl->active->out_state == 1)
	    		{
	    			ctl->active->out_state = 0;
	    			htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	    			htim1.Instance->CCMR1 |= TIM_OCMODE_INACTIVE;
	    			// Intermediate step to generate small pulse
	    			tim1_preload = tim1_cnt + STEP_PULSE_WIDTH;
	    			// now a step has been done
	    			state->pos += ctl->active->dir_abs;
	    			// relative step counter is always positive
	    			ctl->active->s++;
	    		}
	    		else if (ctl->active->out_state == 0)
	    		{
	    			ctl->active->out_state = 1;
	    			htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	    			htim1.Instance->CCMR1 |= TIM_OCMODE_ACTIVE;

	    			// Check if cycle is finished by now
	    			check_cycle_status(ctl, state);

	    			// Recalculate timer preload out of old or new control struct
	    			tim1_c_hw = step_calculations(ctl->active);
	    			tim1_preload = tim1_cnt + tim1_c_hw - STEP_PULSE_WIDTH;

	    			// Take care of direction pin.
	    			switch(ctl->active->dir_abs * Z_DAE_FLIP_DIR)
	    			{
	    			case  1: HAL_GPIO_WritePin(Z_DAE_DIR_GPIO_Port, Z_DAE_DIR_Pin, GPIO_PIN_SET); break;
	    			case -1: HAL_GPIO_WritePin(Z_DAE_DIR_GPIO_Port, Z_DAE_DIR_Pin, GPIO_PIN_RESET); break;
	    			}
	    		}

	    		// Only preset compare reg if neccesary
	    		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, tim1_preload);

	    	}
	    	else
	    	{
	    		// Force off output, so that it does not randomly tick along
	    		htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	    		htim1.Instance->CCMR1 |= TIM_OCMODE_FORCED_INACTIVE;
	    	}
	    }
	  }

	  // Check if the interrupt was created by CC2
	  // ---------------------------------------------------------------------------------------
	  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC2) != RESET)
	  {
		  if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC2) != RESET)
		  {
		        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
		        htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_2;

		  }
	  }

	  // Check if the interrupt was created by CC3
	  // ---------------------------------------------------------------------------------------
	  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC3) != RESET)
	  {
		  if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC3) != RESET)
		  {
		        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC3);
		        htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_3;

		  }
	  }
}


