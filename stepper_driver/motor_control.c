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
real calculate_motor_control (T_SPT_SETUP *setup,  T_STEPPER_STATE *motor, T_ISR_CONTROL_SWAP *ctl);
real min (real a, real b);
real max (real a, real b);

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
		T_SPT_SETUP setup;
		setup.delta_s0 = 1200; // steps
		setup.delta_t0 = 400; // ms
		setup.delta_s1 = 300;
		setup.delta_t1 = 400;
		calculate_motor_control(&setup, &z_dae_motor ,  &z_dae_swap);
		// Mark that new values have been put in place.
		z_dae_swap.available = 1;
	}
}

/** @brief 	If the stepper ISR is done with one cycle and switched to the waiting one, this function has
 * 			to calculate the next waiting one.
 *
 * 			It so far only accepts positive steps.
 *
 * 			What to do if delta_s0 = 0 ? -> shutdown mode
 *
 * 			chatch t=0 exceptions
 *
 *  @param (many)
 *  @return calculated passover speed (which is start speed of the next cycle's calculation
 */
real calculate_motor_control (T_SPT_SETUP *setup,  T_STEPPER_STATE *motor, T_ISR_CONTROL_SWAP *ctl)
{
	// General variables
	//real	t_t;				// Timer tick period (in seconds)
	real 	alpha = motor->alpha; // Just for shorter writing in formulas
	real 	acc = motor->acc; 	// Same here ^
	int32_t i; 					// Loop counters
	int32_t runs = N_APPROX;	// Number of iteration steps. This values is altered if a slow cycle is detected.

	// Flags for all sorts of decisions
	int32_t	slow0 = 0; 			// set to 1 when this cycle is "slow", meaning it can accelerate to target speed with one step

	// Total angle to move in this cycle, in radiant
	real 	delta_theta0;
	real 	delta_theta1;
	int32_t delta_s0 = setup->delta_s0;
	int32_t delta_s1 = setup->delta_s1;
	real	delta_t0 = (real) setup->delta_t0 / 1000;
	real	delta_t1 = (real) setup->delta_t1 / 1000;

	// Angular speeds
	real 	w_mean0;			// Mean speed of this cycle
	real 	w_mean1;			// Mean speed of the next cycle in the future
	real 	w_s = setup->w_s;	// start speed of this cycle (end of last cycle)
	real 	w_m_f; 				// The optimal passoverspeed that was selected
	real 	w_e; 				// End speed of future cycle
	real 	w_t0_f; 			// The final target speed 0 we picked from the iteration
	real 	w_t1_f;				// The final target speed 1 we picked from the iteration
	real 	w_base; 			// The smaller one of w_mean0 and w_mean1
	real 	w_stepsize; 		// Stepsize for iteration of w_m

	// Accelerations
	real 	dw_s; 				// Start acceleration
	real	dw_m; 				// Mid (passover) acceleration
	real	dw_e; 				// Stop acceleration

	// Approximation of ideal passover speed
	real	w_min; 				// is overwritten when a smaller w_m is found
	real 	w_m[N_APPROX]; 		// Array of all iterated passover speeds
	real 	w_t0[N_APPROX]; 	// Array of all iterated target speeds of this cycle
	real	w_t1[N_APPROX]; 	// Array of all iterated target speeds of the next cycle
	real 	w_diff[N_APPROX]; 	// Difference between target speeds in this and the future cycle (should be as small as possible)
	int32_t	d_s[N_APPROX]; 		// Start acceleration direction for each iterated passover speed
	int32_t	d_m[N_APPROX]; 		// Mid acceleration direction for each iterated passover speed
	real	a0, b0, c0; 		// Polynomial coefficients for this cycle
	real 	a1, b1, c1; 		// Polynomial coefficients for next cycle
	real 	disk0, disk1; 		// square root discriminants (for checking if possible)
	int32_t	d_s_f; 				// Final start acceleration direction (-1 or 1)
	int32_t d_m_f; 				// Final mid acceleration direction (-1 or 1)
	//int32_t d_e_f; 				// Final end acceleration direction (-1 or 1)

	// Equivalent acceleration indices
	int32_t	neq_mean0;
	int32_t neq_mean1;


	// ------------ start calculations ------------------------------------
	dbgprintf(" --------- Start motor control calculations -------");
	// All calculations of speeds etc. are done in radiant. So we need to convert steps to radiant

	delta_theta0 = delta_s0 * motor->alpha;
	delta_theta1 = delta_s1 * motor->alpha;

	if (delta_s0 == 0)
	{
		// This is a zero-cycle. No need for further calculation of anything.


	}
	else if (delta_s0 < 0)
	{
		// TODO: Whatnot.
	}

	// Mean speeds of cycles are base for some decisions
	w_mean0 = (real) delta_theta0 / delta_t0;
	w_mean1 = (real) delta_theta1 / delta_t1;

	// End speed of future cycle is set to its mean speed, because it can always do that
	// Other speeds might not even be possible if only very few steps are available
	w_e = w_mean1;

	// Calculate range and stepsize for w_m iteration so that it fills N_approx steps between w_mean0 and w_mean1
	w_base = min(w_mean0, w_mean1);
	w_stepsize = (max(w_mean0, w_mean1) - w_base) / (N_APPROX - 1);
	w_m[0] = w_base;

	// equivalent acceleration indices are needed to decide wheter this cycle is "slow" or not.
	neq_mean0 = w_mean0 * w_mean0 / (2 * alpha * acc);
	neq_mean1 = w_mean1 * w_mean1 / (2 * alpha * acc);

	// Decide if cycles are "slow" and therefore do not need any acceleration ramps
	if (neq_mean1 == 0)
	{
		w_m[0] = w_mean1;
		// w_e is already w_mean1, which it is always.
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

		// chose right direction for start acceleration
		if (w_s > w_mean0)
		{
			dw_s = -acc;
			d_s[i] = -1;
			dbgprintf("Start down");
		}
		else
		{
			dw_s = acc;
			d_s[i] = 1;
			dbgprintf("Start up");
		}

		// chose right direction for mid acceleration (passover)
		if (w_mean0 > w_mean1)
		{
			dw_m = -acc;
			d_m[i] = -1;
			dbgprintf("Middle down");
		}
		else
		{
			dw_m = acc;
			d_m[i] = 1;
			dbgprintf("Middle up");
		}

		// Its always end up for now, because it is not used later anyways.
		dw_e = acc;


		// Polynomial coefficients
		a0 = 1/(2*dw_m) - 1/(2*dw_s);
		b0 = delta_t0 + w_s/dw_s - w_m[i]/dw_m;
		c0 = w_m[i]*w_m[i]/(2*dw_m) - w_s*w_s/(2*dw_s) - delta_theta0;

		a1 = 1/(2*dw_e) - 1/(2*dw_m);
		b1 = delta_t1 + w_m[i]/dw_m - w_e/dw_e;
		c1 = w_e*w_e/(2*dw_e) - w_m[i]*w_m[i]/(2*dw_m) - delta_theta1;

		// Calculate target speeds for this w_m[i]
		// Target speed 0
		if (-R_ERR < a0 && a0 < R_ERR ) // Means a0 == 0
		{
			if (-R_ERR < b0 && b0 < R_ERR) // Means b0 == 0
			{
				// Speed cannot be calculated
				dbgprintf("ERROR: Linear 0 (loop %d)", i);
				return W_ERR;
			}
			else
			{
				dbgprintf("Linear 0 ok");
				w_t0[i] = -c0/b0;
			}
		}
		else
		{
			disk0 = b0*b0-4*a0*c0;
			if (disk0 > 0)
			{
				dbgprintf("Root 0 ok");
				w_t0[i] = (-b0 + sqrt(disk0))/(2*a0);
			}
			else
			{
				dbgprintf("ERROR: Root 0 (loop %d)", i);
				return W_ERR;
			}
		}

		// Target speed 1
		if (-R_ERR < a0 && a0 < R_ERR ) // Means a0 == 0
		{
			if (-R_ERR < b0 && b0 < R_ERR) // Means b0 == 0
			{
				// Speed cannot be calculated
				dbgprintf("ERROR: Linear 1 (loop %d)", i);
				return W_ERR;
			}
			else
			{
				dbgprintf("Linear 1 ok");
				w_t1[i] = -c1/b1;
			}
		}
		else
		{
			disk1 = b1*b1-4*a1*c1;
			if (disk1 > 0)
			{
				dbgprintf("Root 1 ok");
				w_t1[i] = (-b1 + sqrt(disk1))/(2*a1);
			}
			else
			{
				dbgprintf("ERROR: Root 1 (loop %d)", i);
				return W_ERR;
			}
		}

		// Difference between target speed is the thing to be minimized
		w_diff[i] = fabs(w_t0[i] - w_t1[i]);
	}

	// And now choose the best option
	w_min = W_ERR;
	w_m_f = W_ERR;
	w_t0_f = W_ERR;
	w_t1_f = W_ERR;

	for (i = 0; i < runs; i++)
	{
		if (w_t0[i] >= 0 && w_t0[i] < motor->w_max && w_t1[i] >= 0 && w_t1[i] < motor->w_max && w_diff[i] < w_min)
		{
			// Its better than the previous, so we take it
			w_min = w_diff[i];
			w_m_f = w_m[i];
			w_t0_f = w_t0[i];
			w_t1_f = w_t1[i];
			d_s_f = d_s[i];
			d_m_f = d_m[i];
			//d_e_f = d_e[i];
		}
	}

	// Correction of acceleration, if it changed during calculation. This can happen sometimes, and dw_m needs to be updated.
	// It should not be neccessary to update d_m_f here, but by rewriting it again we assure that they always match in direction
	if (w_t0_f > w_t1_f)
	{
		dw_m = -acc;
		d_m_f = -1;
	}
	else
	{
		dw_m = acc;
		d_m_f = 1;
	}

	// In some cases, laying the target speed under w_t0 and w_t1 and not between them actually gives
	// a faster acceleration (don't ask me why, but the equations show it), but we no not want that, so
	// we use the smallest possible in the range between w_t0 and w_t1, which is w_t0  (w_t1 is always
	// "further away", because we accelerate towards it).
	if ((w_t0_f - w_m_f)*d_m_f > 0)
	{
		w_m_f = w_t0_f;
	}

	if (w_t0_f < motor->w_max)
	{
		dbgprintf("Found ideal w_m = %f, w_t0 = %f, w_t1 = %f", w_m_f, w_t0_f, w_t1_f);
	}
	else
	{
		dbgprintf("ERROR: Found nothing possible!");
	}

	// Post processing all values for setting up the ISR struct
	ctl->waiting->s = 0;
	ctl->waiting->s_total = delta_s0;
	ctl->waiting->s_on = (w_t0_f*w_t0_f - w_s*w_s)/(2*motor->alpha*dw_s) + S_EXTRA;
	ctl->waiting->s_off = (delta_s0) - (w_m_f*w_m_f - w_t0_f*w_t0_f)/(2*motor->alpha*dw_m);
	ctl->waiting->neq_on = w_s*w_s/(2*motor->alpha*dw_s);
	ctl->waiting->neq_off = w_t0_f*w_t0_f/(2*motor->alpha*dw_m);
	ctl->waiting->c_0 = F_TIMER * sqrt(2*motor->alpha/motor->acc) * CORR0; // * FACTOR / FACTOR, because CORR0 already conta
	ctl->waiting->c_t = motor->alpha/(w_t0_f / F_TIMER) * FACTOR;
	ctl->waiting->d_on = d_s_f;
	ctl->waiting->d_off = d_m_f;

	// Special treatment for slow speeds (the above calculations may be off by 1, this might make problems in the ISR)
	if (slow0 == 1)
	{
		ctl->waiting->neq_on = 0;
		ctl->waiting->neq_off = 0;
		ctl->waiting->s_on = 0;
		ctl->waiting->s_off = delta_s0;
	}

	ctl->waiting->c_ideal = delta_t0 * F_TIMER;
	ctl->waiting->c_real = 0;
	ctl->waiting->shutoff = 0; // unless its a 0-cycle
	ctl->waiting->running = 0; // Cycle is not activated yet
	ctl->waiting->no_accel = slow0;
	ctl->waiting->dir_abs = 1; // TODO: Change to sensible value



	// And thats it. Wow.
	return w_m_f;
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





