/*
 * timekeeper.h
 *
 *  Created on: Nov 21, 2020
 *      Author: josef
 */

#ifndef TIMEKEEPER_H_
#define TIMEKEEPER_H_

void TK_setSystime(uint32_t time);
uint32_t TK_getSystime(void);
void TK_startTimer (void);
void TK_stopTimer (void);
void isr_tk_millisecond (void);

#endif /* TIMEKEEPER_H_ */
