/** @file motor_control.h
 *  @brief Interface to high level stepper control routines.
 *
 *  @author Josef Heel
	@date April 9th, 2019
 */

# ifndef MOTOR_CONTROL_H_
# define MOTOR_CONTROL_H_

#define N_APPROX		4 					// Number of Interations used to find optimum passover speed
#define R_ERR			1e-6 				// For zero detection on floats
#define S_EXTRA			2					// acceleration is allowed to be S_EXTRA steps longer (overshoot protection catches it normally) to avoid big speed jumps if accel is a little to small
#define W_ERR			100.0F				// To indicate something is wrong.

#include "channels.h"
#include "step_generation.h"

typedef struct
{
	int32_t 			delta_s0;
	int32_t				delta_t0;
	int32_t 			delta_s1;
	int32_t				delta_t1;
	real				w_s;
} T_SPT_CYCLESPEC; // meaning steps per time setup

// Struct containing test motor data
#define TEST_POINTS			9


// PROTOTYPES
void SM_Init(void);
void SM_setMotorReady (T_MOTOR_CONTROL *ctl);
int32_t SM_updateMotor(T_MOTOR_CONTROL *ctl, T_CHANNEL *cha);
void SM_restart_testcylce (void);

# endif // MOTOR_CONTROL_H_
