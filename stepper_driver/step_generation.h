/** @file interrupt_handler.h
 *  @brief Contains functions for stepper control. They are called by various timer interrupts
 *
 *  @author Josef Heel
	@date March 26th, 2019
 */

// A sort of fixed-point arithmetic is used
#define FACTOR		1024

#define PI			3.141592654F;

// Timers intervals (can be calculated somehow, but define for now)
#define TIM1_TICKS_PER_SEC		250000 // runs at 250kHz

typedef enum
{
	ACCEL,
	RUN,
	DECEL,
	STOP
} E_STEPPER_MODE;

typedef struct
{
	// General motor data
	int32_t pos; 				// Contains absolute motor position relativ to end switch (in steps).

	// ISR const dw/dt algorithm stuff
	E_STEPPER_MODE mode; 	// The current state of the stepper engine
	uint8_t compare_out; 	// Toggle flag to generate output pulses
	int32_t c_F; 			// Contains the time between two steps (in timer-ticks x FACTOR)
	int32_t c_new_F; 		// new calculated time between two steps (in timer-ticks x FACTOR)
	int32_t n; 				// counter used to time acceleration and deceleration

	// Other motor parameters used to calculate acceleration etc.
	float alpha; 			// angle which the motor moves when generating one step pulse
	float w_start; 			// speed
	float dw_accel; 		// acceleration [rad/sec^2]
	float dw_decel; 		// deceleration [rad/sec^2] is also positive, because its interpreted as deceleration


} T_STEPPER_STATE;

void slow_down (void);
void speed_up (void);
void tim1_cc_irq_handler (void);

