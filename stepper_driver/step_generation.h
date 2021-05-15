/** @file interrupt_handler.h
 *  @brief Contains functions for stepper control. They are called by various timer interrupts
 *
 *  @author Josef Heel
	@date March 26th, 2019
 */

#ifndef STEP_GENERATION_H_
#define STEP_GENERATION_H_

#include "main.h"
// A sort of fixed-point arithmetic is used
#define FACTOR			1000
#define PI				(3.141592654F)
#define C_TABLE_SIZE	1200				// Size of acceleration table. That is the maximum number of accelerating steps starting at zero speed.

// Timer setup
#define F_TIMER			8000000				// Motor timer frequency. currently 1MHz.
#define C_MAX			65536				// 16 bit timer -> one revolution is 2^16 = 65536 ticks.

typedef enum
{
	STG_IDLE, 		// Meaning the axis is resting because it previously encountered a zero-cycle or has not been started yet
	STG_READY,		// Meaning the axis can start executing a new cycle because the timekeeper decided it needs to start now
	STG_PREPARED,	// It has an active cycle running and the next one already prepared
	STG_NOT_PREPARED,// It has an active cycle running, but the next one still needs to be calculated
	STG_MANUAL,		// Motor is in manual moving mode. This means after each move, it goes to stop automatically
	STG_ERROR,		// When a calculation error has been made, this motor stops.
}E_STG_EXECUTION_STATUS;

typedef enum
{
	STG_NOT_HOME,				// directly after powerup, no home position was found
	STG_HOME,					// this motor's position is accurate to the absolute machine position, it referenced successfully on the limits
	STG_WAITING_FIRST_CONTACT,	// Currently moving towards the first contact with the limit switch
	STG_AT_FIRST_CONTACT,		// This status is set when the motor hits the limit switch the first time.
	STG_RETRACTING, 			// Moving back from first contact a bit
	STG_WAITING_SECOND_CONTACT  // Currently moving towards the second contact with the limit switch
}E_STG_HOME_STATUS;

typedef struct
{
	int32_t			flip_dir;		// A final switch to reverse motor direction
	GPIO_TypeDef* 	dir_port; 		// GPIO port where dir pin is (e.g. Z_DAE_DIR_GPIO_Port)
	uint16_t 		dir_pin;  		// GPIO number of dir pin (e.g. Z_DAE_DIR_Pin)
	TIM_HandleTypeDef *timer; 		// timer which this motor uses
	uint32_t		channel;		// to save the mask of the channel that this motor is connected to
	uint32_t		*CCMR; 			// Capture control register (CCMR1 or CCMR2)
	uint32_t		oc_mask; 		// to save the output compare mask for this channel
	uint32_t		oc_active_mask; // The three masks needed to set the compare mode of the step pin.
	uint32_t		oc_inactive_mask;
	uint32_t 		oc_forced_inactive_mask;
}T_MOTOR_HW;

// Contains motor parameters which are not dependent on the current cycle
typedef struct
{
	// General motor data
	int32_t 		pos; 			// Absolute motor position, relative to end stop [in steps]
	int32_t			scheduled_pos;	// Next scheduled position. After completion of the ongoing cycle, the motor will be there. scheduled_pos is identical to pos if the motor stands still
	float			acc; 			// Maximum allowed acceleration/deceleration [rad^2/sec]
	float 			w_max; 			// maximal allowed motor speed [rad/sec]
	float 			alpha; 			// Rotor angle per step [rad]
	int32_t			c_err; 			// Difference in actual and relative timer ticks (for correction)
	int32_t 		overshoot_on; 	// Tracks how many ticks the overshoot-protection was active (for debug)
	int32_t			overshoot_off; 	// same, but not for the acceleration overshoot, but for deceleration overshoot
	int32_t 		max_travel; 	// Holds the number of steps of the whole range the motor is able to cover. Used for homing the axis
	T_MOTOR_HW		hw;				// contains the mapping of this motor to the hardware (pins, timer registers ...)
	E_STG_HOME_STATUS home_status;	// Holds the information whether this motor has found its home position or not, or if homing is ongoing.
} T_STEPPER_STATE;

// Contains information for ISR Setup of one cycle
// Each motor has two of those, one is actively executed in the ISR, while the other one is being prepared
typedef struct
{	int32_t		c;				// ISR Timer preload (contains FACTOR!) [in timer ticks * FACTOR]
	int32_t 	c_t; 			// Target speed preload value [in timer ticks * FACTOR]
	int32_t 	c_hw; 			// Hardware timer preload. This value is actually put in the timer. [in timer ticks]
	int32_t		c_hwi; 			// timer preload increment. c_hw = c_hwr * 65536 + c_hwi. Both together allow for about 4s between steps with 1MHz and FACTOR is 1000
	int32_t 	c_hwr;			// timer preload rounds. If c_hw cannot be obtained by one full timer revolution, this is the round counter
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
	float		w_finish;		// finishing speed, when this cycle is done. Not used for calculations, but to correctly update the motor status after cycle execution.
} T_ISR_CONTROL;

// One of these for every motor. Contains all the information for this particular motor
typedef struct
{
	char* 			name; 			// Pointer to string where name of motor is stored. Used to print out which motor it was in routines where only the handle is given.
	T_STEPPER_STATE	motor;
	T_ISR_CONTROL 	ctl_swap[2]; 	// This is the memory allocation for active and waiting structs. In active and waiting, there are the pointers of those two
	T_ISR_CONTROL* 	active;
	T_ISR_CONTROL* 	waiting;
	E_STG_EXECUTION_STATUS	status; // State machine status. Running, Idle, prepared, error... see definition
	int32_t slow_decel_at_limit; 	// Should usually be set to 0. If set to non-zero, the motor makes a soft stop when running in the limit. This is used for referencing as it is assumed that at a hardstop, it looses steps.
}T_MOTOR_CONTROL;

// Global stepper state variables
T_ISR_CONTROL stepper_shutoff; // to map into other motor control structs to turn it off.
T_MOTOR_CONTROL x_dae_motor;
T_MOTOR_CONTROL y_dae_motor;
T_MOTOR_CONTROL z_dae_motor;

// PROTOTYPES
void isr_update_stg (T_MOTOR_CONTROL *ctl, uint16_t tim_cnt);
void STG_Init (void);
void STG_swapISRcontrol (T_MOTOR_CONTROL *ctl);
void STG_StartCycle(T_MOTOR_CONTROL *ctl);
void STG_hardstop (T_MOTOR_CONTROL *ctl);
void STG_softstop (T_MOTOR_CONTROL *ctl);

#endif // STEP_GENERATION_H_


