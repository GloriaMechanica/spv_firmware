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

// Is used in ISR. Contains the current CNT-register state of timer1
uint16_t tim1_cnt;
uint16_t tim1_c_hw;
uint16_t tim1_preload;

// DAE - Z - Axis


uint8_t z_dae_output_state;
int32_t ticks_per_step;
int32_t f_ticks_per_step;
int32_t n;
uint8_t mode;

#define STEP_PULSE_WIDTH 	10

// PROTOTYPES
uint16_t step_calculations(T_ISR_CONTROL *ctl);
void check_cycle_status(T_ISR_CONTROL_SWAP *ctl, T_STEPPER_STATE *state);

/** @brief 	Inits the step generation algorithm
 *
 *  @param (none)
 *  @return (none)
 */
void STG_Init (void)
{
	// Init the individual motors
	// Z Axis on DAE strings
	z_dae_state.pos = 0;
	z_dae_state.acc = Z_DAE_ACCEL_MAX;
	z_dae_state.w_max = Z_DAE_SPEED_MAX;
	z_dae_state.alpha = 2 * PI / (Z_DAE_STEPS_PER_REV * Z_DAE_STEP_MODE);

	// Initialize isr control swap stuff
	z_dae_swap.active = &stepper_shutoff;
	z_dae_swap.waiting = &(z_dae_swap.z_dae_control[0]);
	z_dae_swap.available = 0;

	// Start timers
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
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
 *  @param [in/out] *ctl - 	control structure containing all the low level ISR setup paramters (c, n, s...)
 *  						some of those parameters are modified by this routine (n, c
 *  @return timer preload (without FACTOR).
 */
uint16_t step_calculations(T_ISR_CONTROL *ctl)
{
	uint16_t c_temp;
	if (ctl->s == 0)
	{
		ctl->n = ctl->neq_on;
		if (ctl->n == 0)
		{	// If neq is 0, there can be two reasons for that:
			n = n + 1;
			if (ctl->no_accel == 1)
			{	// Const slow speed -> no acceleration is done in the whole cycle
				ctl->c = ctl->c_t;
			}
			else
			{	// Or we just started from 0, so we need initial counter preload
				ctl->c = ctl->c_0;
			}
		}
	}
	if (ctl->s <= ctl->s_on && ctl->n != 0)
	{
		if (ctl->n == 1)
		{ 	// First step acceleration correction
			c_temp = ctl->c * CORR;
			c_temp /= 1000;
		}

		else
		{	// Normal acceleration calculation according to D. Austin
			c_temp = ctl->c - 2*(ctl->c)/(4*(ctl->n) + 1);
		}

		// Overshoot protection
		if ((c_temp - ctl->c)*(ctl->d_on) > 0)
		{
			ctl->c = c_temp;
			ctl->n++;
		}
	}
	else if (ctl->s == ctl->s_off)
	{
		ctl->n = ctl->neq_off;
		if (ctl->n == 0)
		{
			ctl->c = ctl->c_0;
		}
	}
	else if (ctl->s > ctl->s_off && n != 0)
	{
		if (ctl->n == 1)
		{ 	// First step acceleration correction
			ctl->c = ctl->c * CORR;
			ctl->c /= 1000; // CORR Factor is x1000 to avoid rounding errors, so we need to divide afterwards.
		}
		else
		{	// Normal acceleration calculation according to D. Austin
			ctl->c = ctl->c - 2*(ctl->c)/(4*(ctl->n) + 1);
		}
		ctl->n++;
	}
	else
	{
		// We already reached target speed, so we just go with constant target speed.
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
	    	state = &z_dae_state;

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


