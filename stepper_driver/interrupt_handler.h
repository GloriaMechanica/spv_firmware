/** @file interrupt_handler.h
 *  @brief Contains functions for stepper control. They are called by various timer interrupts
 *
 *  @author Josef Heel
	@date March 26th, 2019
 */

void slow_down (void);
void speed_up (void);
void tim1_cc_irq_handler (void);

