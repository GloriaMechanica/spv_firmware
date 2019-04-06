/** @file motor_parameters.c
 *  @brief contains all the static defines for the motors
 *
 *  Maybe some of this stuff will be set up by a GUI via a PC, but for now, its just defines here
 *
 *  @author Josef Heel
	@date March 30th, 2019
 */


// Z-AXIS on "DAE Bogenzug"
#define Z_DAE_STEPS_PER_REV		400			// For example 200 or 400 Steps per revolution
#define Z_DAE_STEP_MODE			4			// 1 for full step, 2 for half step, 4 for quater step ...
#define Z_DAE_ACCEL_MAX			(500.0F)	// maximal acceleration on this axis, [rad/sec^2]
#define Z_DAE_SPEED_MAX			(50.0F)		// maximal speed on this axis [rad/sec]
#define Z_DAE_FLIP_DIR			1			// If set to -1, the direction is flipped, it runs backwards.
