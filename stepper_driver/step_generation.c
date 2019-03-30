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

// Is used in ISR. Contains the current CNT-register state of timer1
uint16_t tim1_cnt;

// DAE - Z - Axis


uint8_t output_state;
uint8_t pulse_generation;
int32_t ticks_per_step;
int32_t f_ticks_per_step;
int32_t n;
uint8_t mode;

#define ACC				0
#define RUN				1
#define DEC				2
#define STOP			3



#define STEP_PULSE_WIDTH				10		// Witdh of step pulse in Timer-ticks (10us)
#define FIX_POINT_OFFSET				2048
#define C_START							500 * FIX_POINT_OFFSET
#define C_END							100 * FIX_POINT_OFFSET




void kick_timer(void);



void speed_up (void)
{

	mode = ACC;
	f_ticks_per_step = 400 * FIX_POINT_OFFSET;
	n = 1;
	pulse_generation = 1;
	kick_timer();
}

void slow_down (void)
{
	mode = DEC;
	n = -400 ;
	kick_timer();
}

void recalculate_preload(void)
{
	if (mode == ACC)
	{

		f_ticks_per_step = f_ticks_per_step - (2*f_ticks_per_step)/(4*n+1);
		n++;

		if (f_ticks_per_step <= C_END)
			mode = RUN;
	}
	else if (mode == RUN)
	{
		// Do nothing and just run along
	}
	else if (mode == DEC)
	{
		f_ticks_per_step = f_ticks_per_step - (2*f_ticks_per_step)/(4*n+1);
		n++;
		if (n > 0 || f_ticks_per_step > C_START)

			mode = STOP;
	}
	else if (mode == STOP)
	{
		pulse_generation = 0;
	}

	ticks_per_step = (uint16_t)(f_ticks_per_step/FIX_POINT_OFFSET);

}
void kick_timer(void)
{
	uint16_t count = __HAL_TIM_GET_COUNTER(&htim1);
	count ++;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, count);
}

void tim1_cc_irq_handler (void)
{
	tim1_cnt = __HAL_TIM_GET_COUNTER(&htim1);

	// Check if the interrupt was created by CC1
	  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC1) != RESET)
	  {
	    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC1) != RESET)
	    {
	    	//dbgprintf("TIM1 CC1 interrupt!");
	        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
	        htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_1;

			recalculate_preload();


	    	if (pulse_generation)
	    	{
	    		if (output_state == 0 && pulse_generation)
				{
	    			tim1_cnt += STEP_PULSE_WIDTH;
					output_state = 1; // Generate a pulse
					htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
					htim1.Instance->CCMR1 |= TIM_OCMODE_INACTIVE;


				}
				else if (output_state == 1 && pulse_generation)
				{

					tim1_cnt += ticks_per_step - STEP_PULSE_WIDTH;
					output_state = 0;
					htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
					htim1.Instance->CCMR1 |= TIM_OCMODE_ACTIVE;
				}

	    	}
	    	else
	    	{
	    		// Switch off output, so that it does not randomly tick along
	    		htim1.Instance->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
	    		htim1.Instance->CCMR1 |= TIM_OCMODE_FORCED_INACTIVE;
	    		output_state = 1;
	    	}

	    		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, tim1_cnt);

	    }
	  }

	  // Check if the interrupt was created by CC2
	  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC2) != RESET)
	  {
		  if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC2) != RESET)
		  {
		        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
		        htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_2;

		  }
	  }

	  // Check if the interrupt was created by CC3
	  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC3) != RESET)
	  {
		  if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC3) != RESET)
		  {
		        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC3);
		        htim1.Channel = HAL_TIM_ACTIVE_CHANNEL_3;

		  }
	  }
}


