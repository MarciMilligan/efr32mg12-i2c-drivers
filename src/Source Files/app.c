/**
 * @file app.c
 * @author Mason Milligan
 * @date 2021-10-24
 * @brief Contains high-level functions that drive the application
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "app.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// Private variables
//***********************************************************************************

//***********************************************************************************
// Private functions
//***********************************************************************************
static void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route);

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Function that calls several functions that open and initialize all necessary
 *  peripherals
 *
 * @details
 *  This routine calls the setup processes for the clock and gpio peripherals,
 *  gathers PWM period and routing settings, then starts the low energy timer.
 *
 * @note
 *  This function is typically run once. It calls all necessary functions to
 *  prepare peripherals and begin operation.
 *
 ******************************************************************************/
void app_peripheral_setup(void)
{
    sleep_open();
    scheduler_open();
    cmu_open();
    gpio_open();
    rgb_init();
    app_letimer_pwm_open(PWM_PER, PWM_ACT_PER, PWM_ROUTE_0, PWM_ROUTE_1);
    letimer_start(LETIMER0, true);  // initiate the start of the LETIMER0
    si1133_i2c_open();
}

/***************************************************************************//**
 * @brief
 *  Function that prepares a struct with necessary information to set LETIMER0 to
 *  interact with LED 1
 *
 * @details
 *  This function builds a struct with information needed to set the LETIMER
 *  to blink LED 1. This includes the timer period, the timer active period,
 *  routing information, and pin enables.
 *
 * @note
 *  This function typically runs once to provide the LETIMER peripheral driver
 *  with the desired operating settings.
 *
 * @param[in] period
 *  Period, in seconds, to run the LETIMER
 *
 * @param[in] act_period
 *  Time, in seconds, that the output of the LETIMER is active
 *
 * @param[in] out0_route
 *  Route from LETIMER0 to a peripheral, see EFR32MG12 datasheet table 6.8
 *
 * @param[in] out1_route
 *  Used to route LETIMER0 to another peripheral
 *
 ******************************************************************************/
void app_letimer_pwm_open(float period, float act_period, uint32_t out0_route, uint32_t out1_route)
{
    /* initialize LETIMER0 for PWM operation by creating letimer_pwm_struct and initializing elements */
    /* APP_LETIMER_PWM_TypeDef is defined in letimer.h */
    APP_LETIMER_PWM_TypeDef letimer_pwm_struct;
    letimer_pwm_struct.debugRun = false;
    letimer_pwm_struct.enable = false;
    letimer_pwm_struct.period = period;
    letimer_pwm_struct.active_period = act_period;
    letimer_pwm_struct.out_pin_route0 = out0_route;
    letimer_pwm_struct.out_pin_route1 = out1_route;
    letimer_pwm_struct.out_pin_0_en = false;
    letimer_pwm_struct.out_pin_1_en = false;
    letimer_pwm_struct.comp0_irq_enable = false;
    letimer_pwm_struct.comp1_irq_enable = true;
    letimer_pwm_struct.uf_irq_enable = true;
    letimer_pwm_struct.comp0_cb = LETIMER0_COMP0_CB;
    letimer_pwm_struct.comp1_cb = LETIMER0_COMP1_CB;
    letimer_pwm_struct.uf_cb = LETIMER0_UF_CB;
    letimer_pwm_open(LETIMER0, &letimer_pwm_struct);
}

/***************************************************************************//**
 * @brief
 *  Event handler that responds to a LETIMER0_UF_CB event
 *
 * @details
 *  This function contains an assert that is set to always fail.
 *
 * @note
 *  LETIMER0_UF_CB events should not occur in this application. If one does occur,
 *  the assert statement may be used to trace the cause.
 *
 ******************************************************************************/
void scheduled_letimer0_uf_cb(void)
{
    EFM_ASSERT(!(get_scheduled_events() & LETIMER0_UF_CB));

    /* read measurements from Si1133 */
    si1133_read(I2C1, SI1133_HOSTOUT0_REG, SI1133_HOSTOUT0_REG_BYTES, SI1133_REG_READ_CB);
}

/***************************************************************************//**
 * @brief
 *  Event handler that responds to a LETIMER0_COMP0_CB event
 *
 * @details
 *  This function contains an assert that is set to always fail.
 *
 * @note
 *  LETIMER0_COMP0_CB events should not occur in this application. If one does
 *  occur, the assert statement may be used to trace the cause.
 *
 ******************************************************************************/
void scheduled_letimer0_comp0_cb(void)
{
    EFM_ASSERT(false);
}

/***************************************************************************//**
 * @brief
 *  Event handler that invokes an I2C read in response to a LETIMER0_COMP1_CB event
 *
 * @details
 *  This function calls @ref si1133_read to begin I2C communication with Si1133.
 *
 * @note
 *  Typically, this function should only be called when a LETIMER0_COMP1_CB event
 *  occurs.
 *
 ******************************************************************************/
void scheduled_letimer0_comp1_cb(void)
{
    EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP1_CB));

    /* send command for FORCE measurement */
    si1133_force_command(I2C1);
}

/***************************************************************************//**
 * @brief
 *  Event handler that turns RGB LED red or green depending on result of I2C
 *  interaction in response to a SI1133_REG_READ_CB event
 *
 * @details
 *  This function calls @ref si1133_get_result to collect the data retrieved via
 *  I2C for comparison with the Si1133 part ID. If the values match, a green LED
 *  is enabled. If they do not match, a red LED is enabled.
 *
 * @note
 *  Typically, this function should only be called when a SI1133_REG_READ_CB event
 *  occurs.
 *
 ******************************************************************************/
void scheduled_si1133_reg_read_cb(void)
{
    EFM_ASSERT(!(get_scheduled_events() & SI1133_REG_READ_CB));

    /* check if Si1133 part number was correctly received via I2C  */
    if(si1133_get_result() < 20)
    {
        /* turn on green LED */
        leds_enabled(RGB_LED_1, COLOR_BLUE, true);
    }
    else
    {
        /* turn on red LED */
        leds_enabled(RGB_LED_1, COLOR_BLUE, false);
    }
}
