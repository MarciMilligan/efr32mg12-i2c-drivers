/**
 * @file scheduler.h
 * @author Mason Milligan
 * @date 2021-09-25
 * @brief Contains functions that manipulate and report the event schedule
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef SCHEDULER_HG
#define	SCHEDULER_HG

/* System include statements */
#include <stdint.h>

/* Silicon Labs include statements */
#include "em_assert.h"

/* The developer's include statements */
#include "sleep_routines.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************
void scheduler_open(void);
void add_scheduled_event(uint32_t event);
void remove_scheduled_event(uint32_t event);
uint32_t get_scheduled_events(void);

#endif
