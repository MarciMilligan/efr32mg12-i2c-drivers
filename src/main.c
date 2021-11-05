/***************************************************************************//**
 * @file main.c
 * @brief Application exploring SLTB004A energy modes through controlling on-board
 * RGB LEDs
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "main.h"

int main(void)
{
    EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
    CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;

    /* Chip errata */
    CHIP_Init();

    /* Init DCDC regulator and HFXO with kit specific parameters */
    /* Initialize DCDC. Always start in low-noise mode. */
    CMU_HFRCOBandSet(MCU_HFXO_FREQ); // set main CPU oscillator frequency
    EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
    EMU_DCDCInit(&dcdcInit);
    em23Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;
    EMU_EM23Init(&em23Init);
    CMU_HFXOInit(&hfxoInit);

    /* Switch HFCLK to HFRCO and disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

    /* Call application program to open / initialize all required peripheral */
    app_peripheral_setup();

    /* infinite blink loop */
    while(1)
    {
        CORE_DECLARE_IRQ_STATE;
        CORE_ENTER_CRITICAL();
        /* enter deepest unblocked sleep state if no events are scheduled */
        if(!get_scheduled_events())
        {
            enter_sleep();
        }
        CORE_EXIT_CRITICAL();

        /* check if any LETIMER0_UF_CB events need to be handled */
        if(get_scheduled_events() & LETIMER0_UF_CB)
        {
            remove_scheduled_event(LETIMER0_UF_CB);
            scheduled_letimer0_uf_cb();
        }

        /* check if any LETIMER0_COMP0_CB events need to be handled */
        if(get_scheduled_events() & LETIMER0_COMP0_CB)
        {
            remove_scheduled_event(LETIMER0_COMP0_CB);
            scheduled_letimer0_comp0_cb();
        }

        /* check if any LETIMER0_COMP1_CB events need to be handled */
        if(get_scheduled_events() & LETIMER0_COMP1_CB)
        {
            remove_scheduled_event(LETIMER0_COMP1_CB);
            scheduled_letimer0_comp1_cb();
        }

        /* check if any SI1133_REG_READ_CB events need to be handled */
        if(get_scheduled_events() & SI1133_REG_READ_CB)
        {
            remove_scheduled_event(SI1133_REG_READ_CB);
            scheduled_si1133_reg_read_cb();
        }
    }
}
