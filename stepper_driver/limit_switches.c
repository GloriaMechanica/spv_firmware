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
	if (x_dae_motor.slow_decel_at_limit == 0)
		STG_hardstop(&x_dae_motor);
	else
		STG_softstop(&x_dae_motor);

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
	if (y_dae_motor.slow_decel_at_limit == 0)
		STG_hardstop(&y_dae_motor);
	else
		STG_softstop(&y_dae_motor);
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
	if (z_dae_motor.slow_decel_at_limit == 0)
		STG_hardstop(&z_dae_motor);
	else
		STG_softstop(&z_dae_motor);

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

