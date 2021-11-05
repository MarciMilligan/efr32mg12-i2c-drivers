/**
 * @file gpio.h
 * @author Mason Milligan
 * @date 2021-09-25
 * @brief Contains functions that configure GPIO peripherals
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef GPIO_HG
#define GPIO_HG

/* System include statements */

/* Silicon Labs include statements */
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_assert.h"

/* The developer's include statements */
#include "brd_config.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_open(void);

#endif
