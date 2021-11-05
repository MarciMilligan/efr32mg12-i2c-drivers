/**
 * @file scheduler.c
 * @author Mason Milligan
 * @date 2021-09-25
 * @brief Contains functions that manipulate and report the event schedule
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "scheduler.h"
#include "em_assert.h"
#include "em_core.h"
#include "em_emu.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// Private variables
//***********************************************************************************
static unsigned int event_scheduled;

//***********************************************************************************
// Private functions
//***********************************************************************************

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Function to initialize schedule with no events
 *
 * @details
 *  This function sets the static int event_scheduled to 0, ensuring that the
 *  scheduler starts with no events scheduled.
 *
 * @note
 *  This function is intended to be called once before using the scheduler.
 *  Running this function at any other time will clear all scheduled events.
 *
 ******************************************************************************/
void scheduler_open(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    event_scheduled = 0;
    CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *  Function to add an event to the schedule
 *
 * @details
 *  This function sets the associated bit for an event to 1, adding the event
 *  to the scheduler.
 *
 * @note
 *  This function is typically, but not necessarily always, called to schedule
 *  an event handler after an interrupt occurs.
 *
 * @param[in] event
 *  The each bit in this variable represents a different callback event. Bits
 *  set to 1 are scheduled.
 *
 ******************************************************************************/
void add_scheduled_event(uint32_t event)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    event_scheduled |= event;
    CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *  Function to remove an event from the schedule
 *
 * @details
 *  This function sets the associated bit for an event to 0, removing the event
 *  from the scheduler.
 *
 * @note
 *  This function is typically, but not necessarily always, called to remove an
 *  an event from the schedule before an event handler is run.
 *
 * @param[in] event
 *  The each bit in this variable represents a different callback event. Bits
 *  set to 1 are removed from the schedule.
 *
 ******************************************************************************/
void remove_scheduled_event(uint32_t event)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    event_scheduled &= ~event;
    CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *  Function to report currently scheduled events
 *
 * @details
 *  This function returns the current schedule.
 *
 * @note
 *  Note that in some instances, the schedule may be accessible and writable by
 *  interrupt handlers. This means the state of the schedule can change immediately
 *  after returning.
 *
 * @param[out] event_scheduled
 *  This variable has specific events associated with each bit. A 1 indicates
 *  that an event is scheduled, a 0 indicates that an event is not scheduled.
 *
 ******************************************************************************/
uint32_t get_scheduled_events(void)
{
    return event_scheduled;
}
