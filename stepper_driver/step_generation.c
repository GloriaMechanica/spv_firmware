/** @file step_generation.c
 *  @brief Contains the low level stepper motor stuff (recalculation of timer-preloads, ISR control swapping etc.)
 *  		The functions here run automatically and take their data from the stepper control swap.
 *  		The swap needs to be checked and updated by other routines.
 *
 *  @author Josef Heel
	@date March 26th, 2019
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
uint8_t pulse_generation;
int32_t ticks_per_step;
int32_t f_ticks_per_step;
int32_t n;
uint8_t mode;


#define STEP_PULSE_WIDTH				10		// Witdh of step pulse in Timer-ticks (10us)
#define FIX_POINT_OFFSET				2048
#define C_START							500 * FIX_POINT_OFFSET
#define C_END							100 * FIX_POINT_OFFSET

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

	// Sets up stepper_shutoff ISR control struct. If this struct
	// is passed to the ISR, it will not do anything but wait forever.
	stepper_shutoff.c = C_MAX * FACTOR;
	stepper_shutoff.c_hw = C_MAX;
	stepper_shutoff.c_0 = C_MAX;
	stepper_shutoff.c_t = C_MAX;
	stepper_shutoff.c_ideal = C_MAX;
	stepper_shutoff.c_real = C_MAX;
	stepper_shutoff.n = 0;
	stepper_shutoff.neq_on = 0;
	stepper_shutoff.neq_off = 0;
	stepper_shutoff.s = 0;
	stepper_shutoff.s_on = 0;
	stepper_shutoff.s_off = 0;
	stepper_shutoff.shutoff = 1; // <- thats the important one
	stepper_shutoff.no_accel = 1;

	// Init the individual motors
	// Z Axis on DAE strings
	z_dae_state.pos = 0;
	z_dae_state.acc = Z_DAE_ACCEL_MAX;
	z_dae_state.w_max = Z_DAE_SPEED_MAX;
	z_dae_state.alpha = 2 * PI / (Z_DAE_STEPS_PER_REV * Z_DAE_STEP_MODE);

	// ISRs need to be shut off here.
}

/** @brief 	Exchanges active and passive ISR control structure. The active one is the one currently
 * 			executed by the ISR, while the waiting one is set up for the next cycle.
 *
 *  @param (none)
 *  @return (none)
 */
void STG_swapISRcontrol (T_ISR_CONTROL_SWAP * isr_control)
{
	T_ISR_CONTROL * temp;
	if (isr_control->available == 1)
	{
		temp = isr_control->active;
		isr_control->active = isr_control->waiting;
		isr_control->waiting = temp;
		isr_control->available = 0;	// Mark that the now waiting one does not contain valid information.
	}
	else
	{
		// for some reason the waiting control sturct has not been updated. As we cannot do anything
		// sensible now, we stop the motor immediately (for now).
		isr_control->active = &stepper_shutoff;
	}
}

/** @brief This function calculates the new timer preload value according to the current ISR-
 * 			(and hence motor-) state. It must not be called when the step number s_total is exceeded.
 * 			This is made sure in the timer ISR.
 *
 *  @param [in/out] *ctl - 	control structure containing all the low level ISR setup paramters (c, n, s...)
 *  						some of those parameters are modified by this routine (n, c
 *  @return timer preload x FACTOR.
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

	// count the used up time to determine error
	ctl->c_real += ctl->c;

	return ctl->c;

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

/* Not needed anymore
void kick_timer(void)
{
	uint16_t count = __HAL_TIM_GET_COUNTER(&htim1);
	count ++;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, count);
}
*/

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

	    	ctl = &z_dae_swap;
	    	state = &z_dae_state;

	    	// Pulse generation is done here for name reasons
	    	if (pulse_generation)
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
	    			case  1: HAL_GPIO_WritePin(Z_DAE_DIR_GPIO_Port, Z_DAE_DIR_Pin, GPIO_PIN_SET);
	    			case -1: HAL_GPIO_WritePin(Z_DAE_DIR_GPIO_Port, Z_DAE_DIR_Pin, GPIO_PIN_RESET);
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
	    		ctl->active->out_state = 1;
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


