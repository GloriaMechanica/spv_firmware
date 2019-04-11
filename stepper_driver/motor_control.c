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
#include <math.h>

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



/** @brief  Needs to be called periodically by the main program. (ideally less than 1ms rhythm)
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
 *  @param (many)
 *  @return calculated passover speed (which is start speed of the next cycle's calculation
 */
real calculate_motor_control (T_SPT_SETUP *setup,  T_STEPPER_STATE *motor, T_ISR_CONTROL_SWAP *ctl)
{
	// General variables
	real	t_t;				// Timer tick period (in seconds)
	real 	alpha = motor->alpha; // Just for shorter writing in formulas
	real 	acc = motor->acc; 	// Same here ^
	int32_t i, j; 				// Loop counters
	int32_t runs = N_approx;	// Number of iteration steps. This values is altered if a slow cycle is detected.

	// Flags for all sorts of decisions
	int32_t	slow0 = 0; 			// set to 1 when this cycle is "slow", meaning it can accelerate to target speed with one step
	int32_t	slow1 = 0; 			// set to 1 when the future cycle is "slow"

	// Total angle to move in this cycle, in radiant
	real delta_theta0;
	real delta_theta1;

	// Angular speeds
	real 	w_mean0;			// Mean speed of this cycle
	real 	w_mean1;			// Mean speed of the next cycle in the future
	real 	w_s = setup->w_s;	// start speed of this cycle (end of last cycle)
	real 	w_m[N_APPROX]; 		// Array of all interated passover speeds
	real 	w_diff[N_APPROX]; 	// Difference between target speeds in this and the future cycle (should be as small as possible)
	real 	w_m_f; 				// The optimal passoverspeed that was selected
	real 	w_e; 				// End speed of future cycle
	real 	w_base; 			// The smaller one of w_mean0 and w_mean1
	real 	w_top; 				// The bigger one of w_mean0 and w_mean1
	real 	w_stepsize; 		// Stepsize for iteration of w_m

	// Accelerations
	real 	dw_s; 				// Start acceleration
	real	dw_m; 				// Mid (passover) acceleration
	real	dw_e; 				// Stop acceleration

	// Equivalent acceleration indices
	int32_t	neq_mean0;
	int32_t neq_mean1;


	// ------------ start calculations ------------------------------------
	// All calculations of speeds etc. are done in radiant. So we need to convert steps to radiant
	delta_theta0 = delta_s0 * motor->alpha;
	delta_theta1 = delta_s1 * motor->alpha;

	// Mean speeds of cycles are base for some decisions
	w_mean0 = (real) delta_theta0 / delta_t0;
	w_mean1 = (real) delta_theta1 / delta_t1;

	// End speed of future cycle is set to its mean speed, because it can always do that
	// Other speeds might not even be possible if only very few steps are available
	w_e = w_mean1;

	// Calculate range and stepsize for w_m iteration so that it fills N_approx steps between w_mean0 and w_mean1
	w_base = min(w_mean0, w_mean1);
	w_stepsize = (max(w_mean0, w_mean1) - w_base) / (N_approx - 1);
	w_m[0] = w_base;

	// equivalent acceleration indices are needed to decide wheter this cycle is "slow" or not.
	neq_mean0 = w_mean0 * w_mean0 / (2 * alpha * acc);
	neq_mean1 = w_mean1 * w_mean1 / (2 * alpha * acc);

	// Decide if cycles are "slow" and therefore do not need any acceleration ramps
	if (neq_mean1 == 0)
	{
		w_m[0] = w_mean1;
		// w_e is already w_mean1, which it is always.
		slow1 = 1;
		runs = 1;
	}

	if (neq_mean0 == 0)
	{
		w_s = w_mean0; 	// this looks crude, what if the motor is turning faster and will be forced slow here?
						// But this never happens because w_m was already chosen right the cycle before,
						// so this statement is not even really necessary here, but it can't hurt to make sure.
		w_m[0] = w_mean0;
		slow0 = 1; 		// Mark flag
		runs = 1; 		// No need for burning power if w_m is already defined by the slow cycle
	}


	for (i = 0; i < runs; i++)
	{
		if (runs > 1)
		{
			// neither of the cycles is slow and more than one iteration shall be done
			w_m[i] = w_stepsize * i + w_base;
		}

		if (w_s > w_mean0)
		{

		}

	}





}

/*	// A demo configuration.
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
 */

/** @brief  Returns the smaller one of two values
 *
 *  @param a, b - values to compare
 *  @return the smaller one of a and b
 */
real min (real a, real b)
{
	if (a < b)
		return a;
	return b;
}

/** @brief  Returns the bigger one of two values
 *
 *  @param a, b - values to compare
 *  @return the bigger one of a and b
 */
real max (real a, real b)
{
	if (a > b)
		return a;
	return b;
}





