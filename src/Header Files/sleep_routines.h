/***************************************************************************//**
 * @file sleep_routines.h
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
#ifndef HEADER_FILES_SLEEP_ROUTINES_H_
#define HEADER_FILES_SLEEP_ROUTINES_H_

/* System include statements */

/* Silicon Labs include statements */
#include "em_emu.h"
#include "em_core.h"
#include "em_assert.h"

/* The developer's include statements */

//***********************************************************************************
// defined files
//***********************************************************************************
#define EM0               0
#define EM1               1
#define EM2               2
#define EM3               3
#define EM4               4
#define MAX_ENERGY_MODES  5

//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************
void sleep_open(void);
void sleep_block_mode(uint32_t EM);
void sleep_unblock_mode(uint32_t EM);
void enter_sleep(void);

#endif /* HEADER_FILES_SLEEP_ROUTINES_H_ */
