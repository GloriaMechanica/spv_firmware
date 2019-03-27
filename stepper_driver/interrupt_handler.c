/** @file interrupt_handler.c
 *  @brief Contains functions for stepper control. They are called by various timer interrupts.
 *
 *  @author Josef Heel
	@date March 26th, 2019
 */

#include "main.h"
#include "device_handles.h"
#include "stm32f7xx_it.h"
#include "debug_tools.h"
#include "interrupt_handler.h"

uint8_t output_state;
uint32_t step_duration;

#define STEP_PULSE_WIDTH				1		// Witdh of step pulse in Timer-ticks (10us)




void tim1_cc_irq_handler (void)
{
	// Check if the interrupt was created by CC1
	  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1) != RESET)
	  {
	    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC1) != RESET)
	    {
	    	//dbgprintf("TIM1 CC1 interrupt!");

	    	uint16_t count;
	    	count = __HAL_TIM_GET_COUNTER(&htim1);

	    	if (output_state == 0)
	    	{
	    		count += STEP_PULSE_WIDTH;
	    		output_state = 1; // Generate a pulse
	    		htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	    		htim1.Instance->CCMR1 |= TIM_OCMODE_INACTIVE;


	    	}
	    	else
	    	{
	    		count +=50000;
	    		output_state = 0;
	    		htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	    		htim1.Instance->CCMR1 |= TIM_OCMODE_ACTIVE;
	    	}


	    	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, count);
	    }
	  }

}
