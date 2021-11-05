/**
 * @file HW_delay.h
 * @author Keith Graham
 * @date 2020-04-19
 * @brief Contains function that enables a time delay
 *
 */

#ifndef SRC_HW_DELAY_H_
#define SRC_HW_DELAY_H_

#include "em_timer.h"
#include "em_cmu.h"

void timer_delay(uint32_t ms_delay);

#endif /* SRC_HW_DELAY_H_ */
