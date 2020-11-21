/** @file motor_parameters.c
 *  @brief contains all the static defines for the motors
 *
 *  Maybe some of this stuff will be set up by a GUI via a PC, but for now, its just defines here
 *
 *  @author Josef Heel
	@date March 30th, 2019
 */


// General Z-axis-related parameters
#define Z_STEPS_PER_REV		400			// For example 200 or 400 Steps per revolution
#define Z_STEP_MODE			4			// 1 for full step, 2 for half step, 4 for quater step ...
#define Z_ACCEL_MAX			(600.0F)		// maximal acceleration on this axis, [rad/sec^2]
#define Z_SPEED_MAX			(50.0F)		// maximal speed on this axis [rad/sec]
#define Z_ALPHA				((double) 2 * PI / (Z_STEPS_PER_REV * Z_STEP_MODE))	// This thing is used sometimes, easier that way

// General X/Y-axis-related parameters
#define XY_STEPS_PER_REV		200			// For example 200 or 400 Steps per revolution
#define XY_STEP_MODE			4			// 1 for full step, 2 for half step, 4 for quater step ...
#define XY_ACCEL_MAX			(600.0F)	// maximal acceleration on this axis, [rad/sec^2]
#define XY_SPEED_MAX			(50.0F)		// maximal speed on this axis [rad/sec]
#define XY_ALPHA				((double) 2 * PI / (XY_STEPS_PER_REV * XY_STEP_MODE))	// This thing is used sometimes, easier that way

// -------- DAE apparatus -------------------------
#define TIMER1_CHANNEL1_MOTOR				(&x_dae_motor)
#define TIMER1_CHANNEL2_MOTOR				(&y_dae_motor)
#define TIMER1_CHANNEL3_MOTOR				(&z_dae_motor)

// hardware mapping of X_DAE-motor
#define X_DAE_HW_FLIP_DIR					1 // if -1, it changes direction
#define X_DAE_HW_DIR_PORT  					X_DAE_DIR_GPIO_Port
#define X_DAE_HW_DIR_PIN					X_DAE_DIR_Pin
#define X_DAE_HW_TIMER						(&htim1)
#define X_DAE_HW_CCMR						((uint32_t*) &(htim1.Instance->CCMR1))
#define X_DAE_HW_CHANNEL					TIM_CHANNEL_1
#define X_DAE_HW_OC_MASK					TIM_CCMR1_OC1M_Msk
#define X_DAE_HW_OC_ACTIVE_MASK				TIM_CCMR1_OC1M_0
#define X_DAE_HW_OC_INACTIVE_MASK		 	TIM_CCMR1_OC1M_1
#define X_DAE_HW_OC_FORCED_INACTIVE_MASK	TIM_CCMR1_OC1M_2

// hardware mapping of Y_DAE-motor
#define Y_DAE_HW_FLIP_DIR					1 // if -1, it changes direction
#define Y_DAE_HW_DIR_PORT  					Y_DAE_DIR_GPIO_Port
#define Y_DAE_HW_DIR_PIN					Y_DAE_DIR_Pin
#define Y_DAE_HW_TIMER						(&htim1)
#define Y_DAE_HW_CCMR						((uint32_t*) &(htim1.Instance->CCMR1))
#define Y_DAE_HW_CHANNEL					TIM_CHANNEL_2
#define Y_DAE_HW_OC_MASK					TIM_CCMR1_OC2M_Msk
#define Y_DAE_HW_OC_ACTIVE_MASK				TIM_CCMR1_OC2M_0
#define Y_DAE_HW_OC_INACTIVE_MASK		 	TIM_CCMR1_OC2M_1
#define Y_DAE_HW_OC_FORCED_INACTIVE_MASK	TIM_CCMR1_OC2M_2

// hardware mapping of Z_DAE-motor
#define Z_DAE_HW_FLIP_DIR					1 	// If set to -1, the direction is flipped, it runs backwards.
#define Z_DAE_HW_DIR_PORT  					Z_DAE_DIR_GPIO_Port
#define Z_DAE_HW_DIR_PIN					Z_DAE_DIR_Pin
#define Z_DAE_HW_TIMER						(&htim1)
#define Z_DAE_HW_CCMR						((uint32_t*) &(htim1.Instance->CCMR2))
#define Z_DAE_HW_CHANNEL					TIM_CHANNEL_3
#define Z_DAE_HW_OC_MASK					TIM_CCMR2_OC3M_Msk
#define Z_DAE_HW_OC_ACTIVE_MASK				TIM_CCMR2_OC3M_0
#define Z_DAE_HW_OC_INACTIVE_MASK		 	TIM_CCMR2_OC3M_1
#define Z_DAE_HW_OC_FORCED_INACTIVE_MASK	TIM_CCMR2_OC3M_2


// -------- GDA apparatus -------------------------
// hardware mapping of X_GDA-motor
#define X_GDA_FLIP_DIR						1 // if -1, it changes direction

// hardware mapping of Y_GDA-motor
#define Y_GDA_FLIP_DIR						1 // if -1, it changes direction

// hardware mapping of Z_GDA-motor
#define Z_GDA_FLIP_DIR						1	// If set to -1, the direction is flipped, it runs backwards.


