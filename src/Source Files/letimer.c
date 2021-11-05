/**
 * @file letimer.c
 * @author Mason Milligan
 * @date 2021-09-25
 * @brief Contains functions that configure and use LETIMER peripherals
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "letimer.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// Private variables
//***********************************************************************************
static uint32_t scheduled_comp0_cb;
static uint32_t scheduled_comp1_cb;
static uint32_t scheduled_uf_cb;

//***********************************************************************************
// Private functions
//***********************************************************************************

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Driver to open and set an LETIMER peripheral in PWM mode
 *
 * @details
 *  This routine is a low level driver.  The application code calls this function
 *  to open one of the LETIMER peripherals for PWM operation to directly drive
 *  GPIO output pins of the device and/or create interrupts that can be used as
 *  a system "heart beat" or by a scheduler to determine whether any system
 *  functions need to be serviced.
 *
 * @note
 *  This function is normally called once to initialize the peripheral and the
 *  function @ref letimer_start() is called to turn-on or turn-off the LETIMER PWM
 *  operation.
 *
 * @param[in] letimer
 *  Pointer to the base peripheral address of the LETIMER peripheral being opened
 *
 * @param[in] app_letimer_struct
 *  Is the STRUCT that the calling routine will use to set the parameters for PWM
 *  operation
 *
 ******************************************************************************/
void letimer_pwm_open(LETIMER_TypeDef *letimer, APP_LETIMER_PWM_TypeDef *app_letimer_struct)
{
    LETIMER_Init_TypeDef letimer_pwm_values;

    unsigned int period_cnt;
    unsigned int period_active_cnt;

    /*  initialize LETIMER for PWM mode */
    /*  enable routed clock to the LETIMER0 peripheral */
    if(letimer == LETIMER0)
    {
        CMU_ClockEnable(cmuClock_LETIMER0, true);
    }

    /* verify that LETIMER clock tree is properly configured and enabled */
    letimer->CMD = LETIMER_CMD_START;
    while(letimer->SYNCBUSY);
    EFM_ASSERT(letimer->STATUS & LETIMER_STATUS_RUNNING);
    letimer->CMD = LETIMER_CMD_STOP;
    while(letimer->SYNCBUSY);

    /* reset counter to 0 so that value is known */
    letimer->CNT = 0;

    /* initialize letimer for PWM operation */
    letimer_pwm_values.bufTop = false; // COMP1 will not be used to load COMP0, but used to create an on-time/duty cycle
    letimer_pwm_values.comp0Top = true; // load COMP0 into cnt register when count register underflows enabling continuous looping
    letimer_pwm_values.debugRun = app_letimer_struct->debugRun;
    letimer_pwm_values.enable = app_letimer_struct->enable;
    letimer_pwm_values.out0Pol = 0; // while PWM is not active out, idle is DEASSERTED, 0
    letimer_pwm_values.out1Pol = 0; // while PWM is not active out, idle is DEASSERTED, 0
    letimer_pwm_values.repMode = letimerRepeatFree; // setup letimer for free running for continuous looping
    letimer_pwm_values.ufoa0 = letimerUFOAPwm; // using the HAL documentation, set to PWM mode
    letimer_pwm_values.ufoa1 = letimerUFOAPwm; // using the HAL documentation, set to PWM mode

    LETIMER_Init(letimer, &letimer_pwm_values); // initialize letimer

    /* calculate value of COMP0 and COMP1 and load control registers with calculated values */
    period_cnt = app_letimer_struct->period * LETIMER_HZ;
    period_active_cnt = app_letimer_struct->active_period * LETIMER_HZ;

    LETIMER_CompareSet(letimer, 0, period_cnt);  // comp0 register is PWM period
    LETIMER_CompareSet(letimer, 1, period_active_cnt); // comp1 register is PWM active period

    /* set REP0 and REP1 mode bits for PWM operation */
    letimer->REP0 |= 1;
    letimer->REP1 |= 1;

    /* Use values from app_letimer_struct for OUT0LOC and OUT1LOC fields within ROUTELOC0 register */
    letimer->ROUTELOC0 = app_letimer_struct->out_pin_route0;
    letimer->ROUTELOC0 |= app_letimer_struct->out_pin_route1;

    /* Use values from app_letimer_struct to program the ROUTEPEN register for OUT 0 Pin Enable (OUT0PEN) and
     * OUT 1 Pin Enable (OUT1PEN) in combination with the enumeration of these pins utilizing boolean multiplication */
    letimer->ROUTEPEN = (LETIMER_ROUTEPEN_OUT0PEN * app_letimer_struct->out_pin_0_en);
    letimer->ROUTEPEN |= (LETIMER_ROUTEPEN_OUT1PEN * app_letimer_struct->out_pin_1_en);

    /* ISR callbacks */
    scheduled_comp0_cb = app_letimer_struct->comp0_cb;
    scheduled_comp1_cb = app_letimer_struct->comp1_cb;
    scheduled_uf_cb = app_letimer_struct->uf_cb;

    /* clear LETIMER0 interrupt flags */
    letimer->IFC |= LETIMER_IFC_COMP0;
    letimer->IFC |= LETIMER_IFC_COMP1;
    letimer->IFC |= LETIMER_IFC_UF;

    /* set interrupt enable bits for LETIMER0 */
    uint32_t IEN_SET;
    IEN_SET = (app_letimer_struct->comp0_irq_enable * LETIMER_IEN_COMP0);
    IEN_SET |= (app_letimer_struct->comp1_irq_enable * LETIMER_IEN_COMP1);
    IEN_SET |= (app_letimer_struct->uf_irq_enable * LETIMER_IEN_UF);
    letimer->IEN = IEN_SET;

    /* Enable interrupts globally */
    NVIC_EnableIRQ(LETIMER0_IRQn);

    /* block from entering EM4 while LETIMER is running */
    if(letimer->STATUS & LETIMER_STATUS_RUNNING)
    {
        sleep_block_mode(LETIMER_EM);
    }
}

/***************************************************************************//**
 * @brief
 *  Function to enable/turn-on or disable/turn-off the LETIMER specified
 *
 * @details
 *  letimer_start uses the lower level API interface of the EM libraries to
 *  directly interface to the LETIMER peripheral to turn-on or off its counter
 *
 * @note
 *  This function should only be called to enable/turn-on the LETIMER once the
 *  LETIMER peripheral has been completely configured via its open driver
 *
 * @param[in] letimer
 *  Pointer to the base peripheral address of the LETIMER peripheral being opened
 *
 * @param[in] enable
 *  Variable to turn-on the LETIMER if boolean value = true and turn-off the LETIMER
 *  if the boolean value = false
 *
 ******************************************************************************/
void letimer_start(LETIMER_TypeDef *letimer, bool enable)
{
    if((letimer->STATUS & LETIMER_STATUS_RUNNING) ^ enable) // check if running bit is different from enable
    {
        LETIMER_Enable(letimer, enable);          // if different, change status
        while(letimer->SYNCBUSY);            // stall until LETIMER synchronizes
        if(enable)
        {
            sleep_block_mode(LETIMER_EM); // block from entering EM4 while LETIMER is running
        }
        else
        {
            sleep_unblock_mode(LETIMER_EM); // unblock EM4 when LETIMER is stopped
        }
    }
}

/***************************************************************************//**
 * @brief
 *  Interrupt handler to schedule events when an interrupt occurs
 *
 * @details
 *  This function determines the type of interrupt that has occurred and schedules
 *  the relevant event for each type of interrupt flag.
 *
 * @note
 *  This function should not be called manually, as it is intended to only be called
 *  by the interrupt controller when an interrupt occurs.
 *
 ******************************************************************************/
void LETIMER0_IRQHandler(void)
{
    uint32_t int_flag;
    int_flag = LETIMER0->IF & LETIMER0->IEN; // AND interrupt flag with enable register to preserve only relevant bits
    LETIMER0->IFC = int_flag;                   // clear interrupt flag register

    if(int_flag & LETIMER_IF_COMP0)                // check for COMP0 interrupt
    {
        add_scheduled_event(scheduled_comp0_cb);    // schedule COMP0 event
        EFM_ASSERT(!(LETIMER0->IF & LETIMER_IF_COMP0));
    }
    if(int_flag & LETIMER_IF_COMP1)                // check for COMP1 interrupt
    {
        add_scheduled_event(scheduled_comp1_cb);    // schedule COMP1 event
        EFM_ASSERT(!(LETIMER0->IF & LETIMER_IF_COMP1));
    }
    if(int_flag & LETIMER_IF_UF)                   // check for UF interrupt
    {
        add_scheduled_event(scheduled_uf_cb);       // schedule UF event
        EFM_ASSERT(!(LETIMER0->IF & LETIMER_IF_UF));
    }
}
