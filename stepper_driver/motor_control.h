/** @file motor_control.h
 *  @brief Interface to high level stepper control routines.
 *
 *  @author Josef Heel
	@date April 9th, 2019
 */

#define N_APPROX		4 					// Number of Interations used to find optimum passover speed

typedef float real;

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

