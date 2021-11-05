/**
 * @file gpio.c
 * @author Mason Milligan
 * @date 2021-09-25
 * @brief Contains functions that configure GPIO peripherals
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************

//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Function to configure LED pins for use
 *
 * @details
 *  This function enables the red and green LED pins and sets the appropriate
 *  drive strength.
 *
 * @note
 *  This function is typically run once to prepare the LEDs as an output.
 *
 ******************************************************************************/
void gpio_open(void)
{

    CMU_ClockEnable(cmuClock_GPIO, true);

    /* red LED pin configuration */
    GPIO_DriveStrengthSet(LED_RED_PORT, LED_RED_DRIVE_STRENGTH);
    GPIO_PinModeSet(LED_RED_PORT, LED_RED_PIN, LED_RED_GPIOMODE, LED_RED_DEFAULT);

    /* green LED pin configuration */
    GPIO_DriveStrengthSet(LED_GREEN_PORT, LED_GREEN_DRIVE_STRENGTH);
    GPIO_PinModeSet(LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_GPIOMODE, LED_GREEN_DEFAULT);

    /* RGB LED pin configuration */
    GPIO_PinModeSet(RGB_ENABLE_PORT, RGB_ENABLE_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
    GPIO_PinModeSet(RGB0_PORT, RGB0_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
    GPIO_PinModeSet(RGB1_PORT, RGB1_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
    GPIO_PinModeSet(RGB2_PORT, RGB2_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
    GPIO_PinModeSet(RGB3_PORT, RGB3_PIN, gpioModePushPull, RGB_DEFAULT_OFF);
    GPIO_PinModeSet(RGB_RED_PORT, RGB_RED_PIN, gpioModePushPull, COLOR_DEFAULT_OFF);
    GPIO_PinModeSet(RGB_GREEN_PORT, RGB_GREEN_PIN, gpioModePushPull, COLOR_DEFAULT_OFF);
    GPIO_PinModeSet(RGB_BLUE_PORT, RGB_BLUE_PIN, gpioModePushPull, COLOR_DEFAULT_OFF);

    /* Si1133 enable pin configuration */
    GPIO_DriveStrengthSet(SI1133_SENSOR_EN_PORT, SI1133_SENSOR_EN_DRIVE_STRENGTH);
    GPIO_PinModeSet(SI1133_SENSOR_EN_PORT, SI1133_SENSOR_EN_PIN, SI1133_SENSOR_EN_GPIOMODE, SI1133_SENSOR_EN_DEFAULT);

    /* Si1133 SCL pin configuration */
    GPIO_PinModeSet(SI1133_SCL_PORT, SI1133_SCL_PIN, SI1133_SCL_GPIOMODE, SI1133_SCL_DEFAULT);

    /* Si1133 SDA pin configuration */
    GPIO_PinModeSet(SI1133_SDA_PORT, SI1133_SDA_PIN, SI1133_SDA_GPIOMODE, SI1133_SDA_DEFAULT);
}
