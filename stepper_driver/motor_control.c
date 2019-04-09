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

static int32_t toggledir;

// PROTOTYPES
void calculate_motor_control(T_ISR_CONTROL_SWAP *ctl);

/** @brief  Initializes all the stepper controller stuff.
 *  @param 	(none)
 *  @return (none)
 */
void SM_Init (void)
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
	stepper_shutoff.running = 0; // Really does not matter in this case
	stepper_shutoff.no_accel = 1;

	// Step generation setup (activates timers etc.)
	STG_Init();

	toggledir = 1;

	// Just for test purposes kick it off
	SM_updateMotorControl();
	STG_swapISRcontrol(&z_dae_swap);
	STG_StartCycle(&z_dae_swap);
}



/** @brief  Needs to be called periodically by the main program.
 *  @param 	(none)
 *  @return (none)
 */
void SM_updateMotorControl(void)
{
	if (z_dae_swap.available == 0)
	{
		calculate_motor_control(&z_dae_swap);
		// Mark that new values have been put in place.
		z_dae_swap.available = 1;
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
	ctl->waiting->c = C_MAX*FACTOR;
	ctl->waiting->c_hw = C_MAX;
	ctl->waiting->c_0 = 1279158;
	ctl->waiting->c_t = 61751; // Factor included
	ctl->waiting->c_ideal = 125000;
	ctl->waiting->c_real = 0;
	ctl->waiting->s = 0;
	ctl->waiting->s_total = 1000;
	ctl->waiting->s_on = 107;
	ctl->waiting->s_off = 892;
	ctl->waiting->n = 0;
	ctl->waiting->neq_on = 0;
	ctl->waiting->neq_off = -108;
	ctl->waiting->shutoff = 0;
	ctl->waiting->running = 0; // cycle is not activated yet
	ctl->waiting->no_accel = 0;
	ctl->waiting->out_state = 0;
	ctl->waiting->dir_abs = toggledir;  // maybe toggle here at some point.
	ctl->waiting->d_on = 1;
	ctl->waiting->d_off = -1;

	toggledir = toggledir * -1;

}








