/***************************************************************************//**
 * @file sleep_routines.c
 * @brief Contains functions that control the system's energy mode
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 **************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************
#include "sleep_routines.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// Private variables
//***********************************************************************************
static int lowest_energy_mode[MAX_ENERGY_MODES];

//***********************************************************************************
// Private functions
//***********************************************************************************

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Function to initialize the lowest_energy_mode array with all energy modes
 *  unblocked
 * 
 * @details
 *  This function sets all array elements to 0, unblocking all energy modes.
 * 
 * @note
 *  This function is intended to be run once before configuring most peripherals.
 *  Running this function more than once or after peripherals are configured may
 *  result in sleep states being inadvertently being unblocked.
 * 
 ******************************************************************************/
void sleep_open(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    int i;
    for(i = 0; i < MAX_ENERGY_MODES; i++)
    {
        lowest_energy_mode[i] = 0;
    }
    CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *  Function to block the system from entering a given energy mode
 * 
 * @details
 *  This function increases the given energy state's value in lowest_energy_mode
 *  by 1, preventing that energy state from being entered.
 * 
 * @note
 *  Multiple peripherals may block the same energy mode. If an energy mode's value
 *  is greater than 0, that energy mode and all deeper sleep states are blocked.
 * 
 * @param[in] EM
 *  Parameter to select which energy mode to block
 * 
 ******************************************************************************/
void sleep_block_mode(uint32_t EM)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    lowest_energy_mode[EM]++;
    EFM_ASSERT(lowest_energy_mode[EM] < 5);
    CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *  Function to unblock the system from entering a given energy mode
 * 
 * @details
 *  This function decreases the given energy state's value in lowest_energy_mode
 *  by 1.
 * 
 * @note
 *  Multiple peripherals may block the same energy mode. If an energy mode's value
 *  is greater than 0, that energy mode and all deeper sleep states are blocked.
 * 
 * @param[in] EM
 *  Parameter to select which energy mode to unblock
 * 
 ******************************************************************************/
void sleep_unblock_mode(uint32_t EM)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    if(lowest_energy_mode[EM] > 0)
    {
        lowest_energy_mode[EM]--;
    }
    EFM_ASSERT(lowest_energy_mode[EM] >= 0);
    CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *  Function to enter the deepest allowed sleep state
 *
 * @details
 *  This function checks lowest_energy_mode for the deepest allowable sleep state,
 *  then enters it.
 *
 * @note
 *  If an energy mode has a value greater than 0, it and all deeper sleep states
 *  will not be entered.
 *
 ******************************************************************************/
void enter_sleep(void)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();
    if(lowest_energy_mode[EM0] > 0)
    {
        // do nothing
    }
    else if(lowest_energy_mode[EM1] > 0)
    {
        // do nothing
    }
    else if(lowest_energy_mode[EM2] > 0)
    {
        EMU_EnterEM1();
    }
    else if(lowest_energy_mode[EM3] > 0)
    {
        EMU_EnterEM2(true);
    }
    else
    {
        EMU_EnterEM3(true);
    }
    CORE_EXIT_CRITICAL();
}
