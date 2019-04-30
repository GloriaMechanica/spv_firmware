/** @file motor_control.h
 *  @brief Interface to high level stepper control routines.
 *
 *  @author Josef Heel
	@date April 9th, 2019
 */

#define N_APPROX		4 					// Number of Interations used to find optimum passover speed
#define R_ERR			1e-6 				// For zero detection on floats
#define S_EXTRA			2					// acceleration is allowed to be S_EXTRA steps longer (overshoot protection catches it normally) to avoid big speed jumps if accel is a little to small
#define W_ERR			100.0F				// To indicate something is wrong.

typedef struct
{
	int32_t 			delta_s0;
	int32_t				delta_t0;
	int32_t 			delta_s1;
	int32_t				delta_t1;
	real				w_s;
} T_SPT_SETUP; // meaning steps per time setup

// PROTOTYPES
void SM_Init(void);
void SM_updateMotorControl(void);

