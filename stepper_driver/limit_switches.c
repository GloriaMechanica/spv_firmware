/*
 * limit_switches.c
 *
 *  Created on: May 14, 2021
 *      Author: josef
 */
#include "main.h"
#include "debug_tools.h"
#include "limit_switches.h"
#include "step_generation.h"


// PROTOTYPES
void check_stop(T_MOTOR_CONTROL *ctl);
void check_referencing(T_MOTOR_CONTROL *ctl);

/** @brief 	Callback for the X-DAE limit switch. If it is triggered
 * 			meaning the moving part is touching, this interrupt is
 * 			called. Only upon trigger, not upon release so far.
 * 			(could be reconfigured in CubeMX)
 *
 *  @param (none)
 *  @return (none)
 */
void LIM_x_dae_callback(void)
{
	check_stop(&x_dae_motor);
	check_referencing(&x_dae_motor);
}

/** @brief 	Callback for the Y-DAE limit switch. If it is triggered
 * 			meaning the moving part is touching, this interrupt is
 * 			called. Only upon trigger, not upon release so far.
 * 			(could be reconfigured in CubeMX)
 *
 *  @param (none)
 *  @return (none)
 */
void LIM_y_dae_callback(void)
{
	check_stop(&y_dae_motor);
	check_referencing(&y_dae_motor);
}

/** @brief 	Callback for the Z-DAE limit switch. If it is triggered
 * 			meaning the moving part is touching, this interrupt is
 * 			called. Only upon trigger, not upon release so far.
 * 			(could be reconfigured in CubeMX)
 *
 *  @param (none)
 *  @return (none)
 */
void LIM_z_dae_callback(void)
{
	check_stop(&z_dae_motor);
	check_referencing(&z_dae_motor);
}

void check_stop(T_MOTOR_CONTROL *ctl)
{
	if (ctl->slow_decel_at_limit == 0)
		STG_hardstop(ctl);
	else
		STG_softstop(ctl);
}

void check_referencing(T_MOTOR_CONTROL *ctl)
{
	if (ctl->motor.home_status == STG_WAITING_FIRST_CONTACT)
	{
		// First contact made -> retract for the second one
		ctl->slow_decel_at_limit = 1;
		ctl->motor.home_status = STG_AT_FIRST_CONTACT;

	}
	else if (ctl->motor.home_status == STG_WAITING_SECOND_CONTACT)
	{
		// Second contact made -> this is now the final motor position zero
		ctl->slow_decel_at_limit = 0;
		ctl->motor.home_status = STG_HOME;
		ctl->motor.pos = 0;
	}
}


/** @brief 	This is the callback from the HAL interrupt handler. There is a weak
 * 			definition of this callback that does nothing in case a callback is
 * 			not needed in the HAL files. This function is the override of this
 * 			weak definition that should be called.
 *
 * 			Use the GPIO_PIN_10 defines to compare GPIO_Pin against.
 *
 *  @param[in] GPIO_Pin - number of Pin which called this interrupt.
 *  @return (none)
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == LIMIT_X_DAE_Pin)
		LIM_x_dae_callback();
	else if (GPIO_Pin == LIMIT_Y_DAE_Pin)
		LIM_y_dae_callback();
	else if (GPIO_Pin == LIMIT_Z_DAE_Pin)
		LIM_z_dae_callback();
}

