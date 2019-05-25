/** @file interrupt_handler.h
 *  @brief Contains functions for stepper control. They are called by various timer interrupts
 *
 *  @author Josef Heel
	@date March 26th, 2019
 */
#include "main.h"
// A sort of fixed-point arithmetic is used
#define FACTOR			1000
#define PI				(3.141592654F)
#define CORR0			676					// Internally devided by 1000, so it should really mean 0.676
#define C_TABLE_SIZE	1200				// Size of acceleration table. That is the maximum number of accelerating steps starting at zero speed.

// Timer setup
#define F_TIMER			250000				// TODO: Calculate from project settings later.
#define C_MAX			65535				// Biggest possible timer preload in one cycle

// Contains motor parameters which are not dependent on the current cycle
typedef struct
{
	// General motor data
	int32_t 	pos; 			// Absolute motor position, relative to end stop [in steps]
	float		acc; 			// Maximum allowed acceleration/deceleration [rad^2/sec]
	float 		w_max; 			// maximal allowed motor speed [rad/sec]
	float 		alpha; 			// Rotor angle per step [rad]
	int32_t		c_err; 			// Difference in actual and relative timer ticks (for correction)
	int32_t 	overshoot_on; 	// Keeps overshoot-ticks from last cycle to be extracted when calculating the next cycle
	int32_t		overshoot_off; 	// same.
} T_STEPPER_STATE;

// Contains information for ISR Setup of one cycle
typedef struct
{	int32_t		c;				// ISR Timer preload (contains FACTOR!) [in timer ticks * FACTOR]
	int32_t 	c_t; 			// Target speed preload value [in timer ticks * FACTOR]
	int32_t 	c_0; 			// Timer preload for cold start  [in timer ticks * FACTOR]
	int32_t 	c_hw; 			// Hardware timer preload. This value is actually put in the timer. [in timer ticks]
	int32_t		c_ideal; 		// Theoretical number of timer ticks in this cycle
	int32_t		c_real; 		// Actual number of timer ticks this cycle took. Used to keep track of timing error accumulation.
	int32_t		s; 				// Current relative step position in this cycle
	int32_t 	s_total; 		// Relative amount of steps to do in this cycle
	int32_t 	s_on; 			// relative step position when on-phase is completed
	int32_t 	s_off; 			// relative step position when off-phase starts
	int32_t		n; 				// acceleration index (corresponds to the number of steps needed to get to this speed from 0)
	int32_t		neq_on; 		// acceleration index preload at the start of cycle
	int32_t 	neq_off; 		// acceleration index preload at the end of cycle
	int32_t		shutoff; 		// When set to 1, the motor does not move at all and it does not automatically start the next cycle
	int32_t		running; 		// Timer only executes this control struct, when running is 1. Otherwise it does nothing.
	int32_t		no_accel;		// When set to 1, the motor does not accelerate at all and just moves at target speed
	int32_t 	out_state; 		// For pulse generation. See ISR.
	int32_t 	dir_abs; 		// Direction. 1 means forward, -1 means backwards. DO NOT PUT 0 in here!
	int32_t 	d_on; 			// direction of acceleration at the beginning of cycle. 1 means faster, -1 means slower
	int32_t		d_off; 			// direction of acceleration at the end of cycle. 1 means faster, -1 means slower
	int32_t		*c_table; 		// Acceleration table. This pointer points to the right table somewhere in the ram to be used with this motor.
	int32_t		overshoot_on; 	// Counter for how much overshoot was done when starting up
	int32_t		overshoot_off; 	// Counter for how much overshoot was done when approaching passover speed
} T_ISR_CONTROL;

typedef struct
{
	T_ISR_CONTROL 	z_dae_control[2]; // This is the memory allocation for active and waiting structs. In active and waiting, there are the pointers of those two
	T_ISR_CONTROL* 	active;
	T_ISR_CONTROL* 	waiting;
	int32_t			available; 	// Gets set to 1 if the waiting control struct has been updated and is ready to use in the next cycle
}T_ISR_CONTROL_SWAP;

// Global stepper state variables
T_ISR_CONTROL stepper_shutoff;
T_STEPPER_STATE	z_dae_motor;
T_ISR_CONTROL_SWAP z_dae_swap;

// PROTOTYPES
void tim1_cc_irq_handler (void);
void STG_Init (void);
void STG_swapISRcontrol (T_ISR_CONTROL_SWAP * isr_control);
void STG_StartCycle(T_ISR_CONTROL_SWAP *ctl);




