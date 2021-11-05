/**
 * @file LEDs_thunderboard.h
 * @date 2021-09-12
 * @brief Contains functions that configure and control the RGB LEDs
 *
 */

#ifndef LED_thunderboard_HG
#define LED_thunderboard_HG

//***********************************************************************************
// Include files
//***********************************************************************************
/* System include statements */

/* Silicon Labs include statements */
#include "stdbool.h"
#include "stdint.h"
#include "em_gpio.h"

/* The developer's include statements */
#include "brd_config.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define	COLOR_RED		(0x01 << 0)
#define	COLOR_GREEN (0x01 << 1)
#define	COLOR_BLUE	(0x01 << 2)
#define NO_COLOR		(0x00 << 0)

#define RGB_LED_0		(0x01 << 0)
#define RGB_LED_1		(0x01 << 1)
#define RGB_LED_2		(0x01 << 2)
#define RGB_LED_3		(0x01 << 3)
#define NO_LEDS			(0x00 << 0)

#define	RGB_PWM_PERIOD	20
#define RGB_PWM_ACTIVE	1

//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************
void rgb_init(void);
void leds_enabled(uint32_t leds, uint32_t color, bool enable);

#endif
