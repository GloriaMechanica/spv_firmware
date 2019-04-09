/** @file motor_control.c
 *  @brief 	Contains the hight level stepper control functions (steps-per-delta_t interface). From the steps-per-time
 *  		information, the optimal passover speed is calculated and the isr control structs are prepared.
 *
 *  @author Josef Heel
	@date April 9th, 2019
 */

#include "main.h"
#include "debug_tools.h"
#include "step_generation.h"
#include "motor_control.h"

// PROTOTYPES
void calculate_motor_control(T_ISR_CONTROL_SWAP *ctl);


/** @brief  Needs to be called periodically by the main program.
 *  @param 	(none)
 *  @return (none)
 */
void SM_updateMotorControl(void)
{
	if (z_dae_swap.available == 0)
	{
		calculate_motor_control(&z_dae_swap);
	}
}

/** @brief 	If the stepper ISR is done with one cycle and switched to the waiting one, this function has
 * 			to calculate the next waiting one.
 *
 *  @param (none)
 *  @return (none)
 */
void calculate_motor_control (T_ISR_CONTROL_SWAP *ctl)
{

	// Just load a fixed configuration in for now
	ctl->waiting->c = C_MAX;
	ctl->waiting->c_hw = C_MAX;
	ctl->waiting->c_t = 145723; // Factor included
	ctl->waiting->c_ideal = 125000;
	ctl->waiting->c_real = 0;
	ctl->waiting->s = 0;
	ctl->waiting->s_total = 800;
	ctl->waiting->s_on = 28;
	ctl->waiting->s_off = 771;
	ctl->waiting->n = 0;
	ctl->waiting->neq_on = 0;
	ctl->waiting->neq_off = -29;
	ctl->waiting->shutoff = 0;
	ctl->waiting->no_accel = 0;
	ctl->waiting->out_state = 0;
	ctl->waiting->dir_abs = 1;  // maybe toggle here at some point.
	ctl->waiting->d_on = 1;
	ctl->waiting->d_off = -1;

	// Mark that new values have been put in place.
	ctl->available = 1;
}








