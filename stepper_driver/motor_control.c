/** @file motor_control.c
 *  @brief 	Contains the hight level stepper control functions (steps-per-delta_t interface). From the steps-per-time
 *  		information, the optimal passover speed is calculated and the isr control structs are prepared.
 *
 *  @author Josef Heel
	@date April 9th, 2019
 */

#include "main.h"
#include "debug_tools.h"
#include "motor_control.h"
#include "timekeeper.h"
#include <math.h>

// Debug-only stuff
int32_t test_positions_xy[TEST_POINTS] = {0, 	500, 	500, 	1550, 	250, 	2000, 	0, 		100, 		0};
int32_t test_times_xy[TEST_POINTS] = 	  {0, 	300, 	400, 	500, 	600, 	500, 	500, 	100, 	100};
int32_t test_positions_z[TEST_POINTS] = {0, 	1000, 	2000, 	3100, 	500, 	10000, 	0, 		400, 		0};
int32_t test_times_z[TEST_POINTS] = 	  {0, 	300, 	400, 	500, 	600, 	1000, 	1000, 	200, 	500};
real w_old_x, w_old_y, w_old_z;
int32_t cycle_number_x, cycle_number_y, cycle_number_z;


// PROTOTYPES
real calculate_motor_control (T_SPT_CYCLESPEC *setup, T_MOTOR_CONTROL *ctl);
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
	stepper_shutoff.c_hwr = 0;
	stepper_shutoff.c_hwi = C_MAX;
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
	stepper_shutoff.overshoot_on = 0;
	stepper_shutoff.overshoot_off = 0;
	stepper_shutoff.out_state = 0;

	// Step generation setup (activates timers etc.)
	STG_Init();
}

/*
 * Just for debugging
 */
void SM_restart_testcylce (void)
{
	int i;
	T_DTP_MOTOR datapoint;

	TK_stopTimer();
	CHA_Init(); // Clears all channel buffers and sets all the last executed times to 0

	// Push some events in the queue
	for (i = 0; i < TEST_POINTS; i++)
	{
		datapoint.steps = test_positions_xy[i];
		datapoint.timediff = test_times_xy[i];
		CHA_pushDatapoints(&cha_posx_dae, (void*) &datapoint, 1);
		CHA_pushDatapoints(&cha_posy_dae, (void*) &datapoint, 1);
	}


	// Push some events in the queue
	for (i = 0; i < TEST_POINTS; i++)
	{
		datapoint.steps = test_positions_z[i];
		datapoint.timediff = test_times_z[i];
		CHA_pushDatapoints(&cha_str_dae, (void*) &datapoint, 1);
	}

	CHA_setChannelTime(0);
	CHA_startTime();
	dbgprintf(" RESTART testcycle at t=%d", CHA_getChannelTime());
}

/** @brief  Immediately shuts the motor off.
 *  @param 	(none)
 *  @return (none)
 */
void SM_hardstop (void)
{

}

/** @brief  Call when time reached the timestamp of the first element in the channel buffer.
 *  @param 	(none)
 *  @return (none)
 */
void SM_setMotorReady (T_MOTOR_CONTROL *ctl)
{
	// The motor controller always calculates the difference to the last scheduled position
	// As the motor is not moving, the last scheduled position must have been the one we are currently at.
	ctl->motor.scheduled_pos = ctl->motor.pos;

	ctl->status = STG_READY;
}


/** @brief	Should be called periodically in the main loop for each motor.
 *
 * 			Executes the difference in time/position between the first two
 * 			elements in a channel if the status of the motor indicates to
 * 			do so.
 *
 * 			When the time reaches the timestamp of the execution, status
 * 			STG_READY should be set and this thing will start the execution
 *
 * 			When the step generator requires a new cycle to be calculated in
 * 			an ongoing trajectory, it will set STG_NOT_PREPARED and this
 * 			function will calculate the next waiting cycle.
 *
 * 			It
 *  @param 	*ctl - pointer to motor control structure
 *  @param 	*cha - pointer to channel handle assigned to this motor.
 *  @return returns -2 if channel is empty, -1 if this was the last entry in the channel.
 */
int32_t SM_updateMotor(T_MOTOR_CONTROL *ctl, T_CHANNEL *cha)
{
	T_SPT_CYCLESPEC setup;
	T_DTP_MOTOR datapoint[2];
	int32_t points_available;
	int32_t ret = 0;
	real w_ret = 0.0;

	// Only do something if a cycle is currently executed and needs refilling or if a new trajectory should be started
	if (ctl->status == STG_READY || ctl->status == STG_NOT_PREPARED)
	{
		// Depending on how many datapoints are available, we pop one and read two more, or we
		// pop one, read, whats there and fill the rest up with zero-cycles (you always need something
		// pass to the motor_calculations function.
		points_available = CHA_getNumberDatapoint(cha);
		if (points_available >=2)
		{
			CHA_popDatapoints(cha, (void*) &(datapoint[0]),1);
			CHA_readDatapoints(cha, (void*) &(datapoint[1]), 1); // get one new datapoint without deleting.
		}
		else if (points_available == 1)
		{
			// Add one zero-cylce at the end
			CHA_popDatapoints(cha, (void*) &(datapoint[0]),1);
			datapoint[1].timediff = 100;
			datapoint[1].steps = datapoint[0].steps;
			dbgprintf("Last point for %s", ctl->name);
			ret = -1;
		}
		else if (points_available == 0)
		{
			// Add two zero-cycles at the end
			datapoint[0].timediff = 100;
			datapoint[0].steps = ctl->motor.scheduled_pos;
			datapoint[1].timediff = 100;
			datapoint[1].steps = datapoint[0].steps;
			dbgprintf("No points for %s", ctl->name);
		}

		// And extract the difference between datapoints and pass them over to the motor calculator
		setup.delta_s0 = datapoint[0].steps - ctl->motor.scheduled_pos; // where we need to be minus where we are
		setup.delta_t0 = datapoint[0].timediff;
		setup.delta_s1 = datapoint[1].steps - datapoint[0].steps;
		setup.delta_t1 = datapoint[1].timediff;

		// As the setup for the next cycle is done, we just scheduled a next position
		// so we need to update this variable. Additionally, the last executed time point needs to be incremented.
		ctl->motor.scheduled_pos = datapoint[0].steps;
		cha->last_point_time = cha->last_point_time + datapoint[0].timediff;

		if (ctl->status == STG_READY)
		{
			// This is a new self-following trajectory to start. The motor has not been moving previously
			setup.w_s = 0; // this is the start of a new trajectory, so start speed is 0
			dbgprintf("%s Start trajectory of at t=%d: ", ctl->name, CHA_getChannelTime());
			w_ret = calculate_motor_control(&setup, ctl);
			if (w_ret == W_ERR)
			{
				// Could not fit the motion request
				ctl->status = STG_ERROR;
				dbgprintf("%s CALCULATION ERROR!", ctl->name);
			}
			else
			{
				ctl->status = STG_PREPARED;
				STG_StartCycle(ctl);
			}

		}
		else if (ctl->status == STG_NOT_PREPARED)
		{
			// This is a point in a trajectory and not a new one.
			setup.w_s = ctl->active->w_finish; // set the start of this waiting cycle to the finishing speed of the currently active one
			dbgprintf("%s continue trajectory at t=%d: ", ctl->name, CHA_getChannelTime());
			w_ret = calculate_motor_control(&setup, ctl);
			if (w_ret == W_ERR)
			{
				// Could not fit the motion request
				ctl->status = STG_ERROR;
				dbgprintf("%s CALCULATION ERROR!", ctl->name);
			}
			else
			{
				ctl->status = STG_PREPARED;
			}
			// Does not have to be started because the ISR will swap the waiting struct in at the right time itself
		}
	}

	return ret;
}

/** @brief 	Prepares the next waiting struct according to the cycle setup
 * 			must not be given cycles with a cycle with t=0 which is impossible
 *
 *  @param *setup - steps over time for this and for the next cycle, plus start speed
 *  @param *ctl - pointer to motor handle. Here it finds the waitin struct.
 *  @return calculated passover speed (which is start speed of the next cycle's calculation
 */
real calculate_motor_control (T_SPT_CYCLESPEC *setup, T_MOTOR_CONTROL *ctl)
{
	// General variables
	//real	t_t;				// Timer tick period (in seconds)
	real 	alpha = ctl->motor.alpha; // Just for shorter writing in formulas
	real 	acc = ctl->motor.acc; 	// Same here ^
	real 	w_max = ctl->motor.w_max;
	int32_t i; 					// Loop counters
	int32_t runs = N_APPROX;	// Number of iteration steps. This values is altered if a slow cycle is detected.

	// Flags for all sorts of decisions
	int32_t	slow0 = 0; 			// set to 1 when this cycle is "slow", meaning it can accelerate to target speed with one step

	// Total angle to move in this cycle, in radiant
	real 	delta_theta0;
	real 	delta_theta1;
	int32_t delta_s0 = setup->delta_s0;
	int32_t delta_s1 = setup->delta_s1;
	real	delta_t0 = (real) setup->delta_t0 / 1000; // milliseconds to seconds
	real	delta_t1 = (real) setup->delta_t1 / 1000;
	int32_t dir_abs;

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

	// TODO: do this properly
	int dbp=0;
	if (ctl == &x_dae_motor)
		dbp = 0;

	// Equivalent acceleration indices
	int32_t	neq_mean0;
	int32_t neq_mean1;

	// Information of last cycle (how it performed during execution)
	dbgprintfc(dbp, " --------- Information from last completed -----------------");
	dbgprintfc(dbp, "Timing error: %f ms (%d ticks)", (real) ctl->motor.c_err * 1000 / F_TIMER, ctl->motor.c_err);
	dbgprintfc(dbp, "Overshoot on: %d Overshoot off: %d", ctl->motor.overshoot_on, ctl->motor.overshoot_off);


	// ------------ start calculations ------------------------------------
	dbgprintfc(dbp, " --------- Start motor control calculations for %s -------", ctl->name);


	// Print out input parameters for test purposes
	dbgprintfc(dbp, "delta_s0: %d steps  in   delta_t0: %d ms", setup->delta_s0, setup->delta_t0);
	dbgprintfc(dbp, "delta_s1: %d steps  in   delta_t1: %d ms", setup->delta_s1, setup->delta_t1);
	dbgprintfc(dbp, "start speed: %f rad/s", setup->w_s);

	// First, decide some important things. Are all inputs valid?
	if (delta_t0 < 0 || delta_t1 < 0)
	{
		// negative times are crap. Stop stepper and report error
		dbgprintfc(dbp, "Input error: negative times");
		ctl->waiting = &stepper_shutoff;
		return 0;
	}

	// Is the current cycle a zero-cycle?
	if (delta_s0 == 0)
	{
		// This is a zero-cycle. No need for further calculation of anything.
		dbgprintfc(1, "%s Detected zero-cycle. Motor-control stop.", ctl->name);
		ctl->waiting = &stepper_shutoff;
		return 0; // Passover speed after a stop-cycle is 0

	}

	// Does the next cycle go in the other direction as this one?
	if (delta_s0 * delta_s1 < 0)
	{
		// If it does so, passover speed needs to be 0.
		// This is achived by setting delta_s1 to 0. The algorithm itself chooses w_m_f to be 0, which is correct.
		// The fact that we wrote 0 to delta_s1 is not relevant, because when it comes to this cycle, it will be reloaded
		// as delta_s0 and executed correctly
		delta_s1 = 0;
	}

	// Is the direction of this cycle backwards?
	if (delta_s0 < 0)
	{
		// If so, the sign bits both of delta_s0 and delta_s1 need to be flipped, but the hardware direction is also flipped
		// If the signs are different, delta_s1 has already been set to 0, so this does not matter.
		delta_s0 = -delta_s0;
		delta_s1 = -delta_s1;
		dir_abs = -1;
	}
	else
	{
		dir_abs = 1;
	}

	// All calculations of speeds etc. are done in radiant. So we need to convert steps to radiant
	delta_theta0 = delta_s0 * alpha;
	delta_theta1 = delta_s1 * alpha;

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

	// equivalent acceleration indices are needed to decide whether this cycle is "slow" or not.
	neq_mean0 = w_mean0 * w_mean0 / (2 * alpha * acc);
	neq_mean1 = w_mean1 * w_mean1 / (2 * alpha * acc);

	// Decide if cycles are "slow" and therefore do not need any acceleration ramps
	// It is important that first neq_mean1 is checked and then possibly overwritten by neq_mean0
	if (neq_mean1 == 0)
	{
		w_m[0] = w_mean1;
		// w_e is already w_mean1, which it is always. (because we cannot look in the future forever, so its a good compromise)
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
			dbgprintfc(dbp, "Start down");
		}
		else
		{
			dw_s = acc;
			d_s[i] = 1;
			dbgprintfc(dbp, "Start up");
		}

		// chose right direction for mid acceleration (passover)
		if (w_mean0 > w_mean1)
		{
			dw_m = -acc;
			d_m[i] = -1;
			dbgprintfc(dbp, "Middle down");
		}
		else
		{
			dw_m = acc;
			d_m[i] = 1;
			dbgprintfc(dbp, "Middle up");
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
				dbgprintfc(dbp, "ERROR: Linear 0 (loop %d)", i);
				return W_ERR;
			}
			else
			{
				dbgprintfc(dbp, "Linear 0 ok");
				w_t0[i] = -c0/b0;
			}
		}
		else
		{
			disk0 = b0*b0-4*a0*c0;
			if (disk0 > 0)
			{
				dbgprintfc(dbp, "Root 0 ok");
				w_t0[i] = (-b0 + sqrt(disk0))/(2*a0);
			}
			else
			{
				dbgprintfc(dbp, "ERROR: Root 0 (loop %d)", i);
				return W_ERR;
			}
		}

		// Target speed 1
		if (-R_ERR < a1 && a1 < R_ERR ) // Means a0 == 0
		{
			if (-R_ERR < b1 && b1 < R_ERR) // Means b0 == 0
			{
				// Speed cannot be calculated
				dbgprintfc(dbp, "ERROR: Linear 1 (loop %d)", i);
				return W_ERR;
			}
			else
			{
				dbgprintfc(dbp, "Linear 1 ok");
				w_t1[i] = -c1/b1;
			}
		}
		else
		{
			disk1 = b1*b1-4*a1*c1;
			if (disk1 > 0)
			{
				dbgprintfc(dbp, "Root 1 ok");
				w_t1[i] = (-b1 + sqrt(disk1))/(2*a1);
			}
			else
			{
				dbgprintfc(dbp, "ERROR: Root 1 (loop %d)", i);
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
	d_s_f = 0; // Initialize, just so the warning goes away

	for (i = 0; i < runs; i++)
	{
		if (((w_t0[i] >= 0) && (w_t0[i] < w_max) && (w_t1[i] >= 0) && (w_t1[i] < w_max) && (w_diff[i] < w_min)) || slow0)
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
	// It should not be necessary to update d_m_f here, but by rewriting it again we assure that they always match in direction
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

	if (w_t0_f < w_max)
	{
		dbgprintfc(dbp, "Found ideal w_m = %f, w_t0 = %f, w_t1 = %f", w_m_f, w_t0_f, w_t1_f);
	}
	else
	{
		dbgprintfc(dbp, "ERROR: Found nothing possible! (w_m = %f)", w_m_f);
		ctl->waiting = &stepper_shutoff;
		return 0;
	}

	// Post processing all values for setting up the ISR struct
	// c is set by ISR
	ctl->waiting->c_t = alpha/(w_t0_f / F_TIMER) * FACTOR;
	// c_hw is set by ISR
	ctl->waiting->c_ideal = delta_t0 * F_TIMER;
	ctl->waiting->c_real = 0;
	ctl->waiting->c_hwr = 0; // That needs to be initialized for the ISR to calculate the first step. Then it is overwritten in the ISR.

	ctl->waiting->s = 0;
	ctl->waiting->s_total = delta_s0;
	ctl->waiting->s_on = (w_t0_f*w_t0_f - w_s*w_s)/(2*alpha*dw_s) + S_EXTRA;
	ctl->waiting->s_off = (delta_s0) - (w_m_f*w_m_f - w_t0_f*w_t0_f)/(2*alpha*dw_m);
	// n is set by ISR
	ctl->waiting->neq_on = w_s*w_s/(2*alpha*dw_s);
	ctl->waiting->neq_off = w_t0_f*w_t0_f/(2*alpha*dw_m);
	ctl->waiting->shutoff = 0; // unless its a 0-cycle
	ctl->waiting->running = 0; // Cycle is not activated yet
	ctl->waiting->no_accel = slow0;
	ctl->waiting->out_state = 0; // Always start with a low and wait first (DO NOT CHANGE!)
	ctl->waiting->dir_abs = dir_abs;
	ctl->waiting->d_on = d_s_f;
	ctl->waiting->d_off = d_m_f;
	ctl->waiting->w_finish = w_m_f; // important to indicate the speed at which this cycle will finish (for calc of next cycle)

	// Special treatment for slow speeds (the above calculations may be off by 1, this might make problems in the ISR)
	if (slow0 == 1)
	{
		ctl->waiting->neq_on = 0;
		ctl->waiting->neq_off = 0;
		ctl->waiting->s_on = 0;
		ctl->waiting->s_off = delta_s0;
	}

	dbgprintfc(dbp, "s_total: %d s_on: %d s_off: %d", ctl->waiting->s_total, ctl->waiting->s_on, ctl->waiting->s_off);
	dbgprintfc(dbp, "neq_on: %d neq_off: %d ", ctl->waiting->neq_on, ctl->waiting->neq_off);
	dbgprintfc(dbp, "c_t: %d", ctl->waiting->c_t);
	dbgprintfc(dbp, "d_on: %d d_off: %d", ctl->waiting->d_on, ctl->waiting->d_off);
	dbgprintfc(dbp, "dir_abs: %d slow: %d", ctl->waiting->dir_abs, ctl->waiting->no_accel);

	dbgprintfc(dbp, "-------- Finished motor control calculations -------------");
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





