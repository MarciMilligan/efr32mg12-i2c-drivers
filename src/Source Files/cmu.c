/**
 * @file cmu.c
 * @author Mason Milligan
 * @date 2021-09-12
 * @brief Contains functions that configure clocks and clock routing
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "cmu.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// Private variables
//***********************************************************************************

//***********************************************************************************
// Private functions
//***********************************************************************************

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Function to disable unused clocks and route ULFRCO.
 *
 * @details
 *  Since LFRCO and LFXO are not needed, they are disabled here. Additionally,
 *  ULFRCO is routed through CMU_LFACLKSEL.LFA and CMU_LFACLKEN0.LETIMER0.
 *
 * @note
 *  This function is typically called once to disable unused clocks and prepare
 *  ULFRCO to be used.
 *
 ******************************************************************************/
void cmu_open(void)
{
    /* by default, LFRCO is enabled */
    /* disable the LFRCO oscillator */
    CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);

    /* disable the LFXO oscillator */
    CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

    /* no requirement to enable the ULFRCO oscillator, it is always enabled in EM0-4H1 */

    /* Route LF clock to the LF clock tree */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

    /* enable clock tree onto the LE clock branches */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* select HFRCO frequency */
    CMU_HFRCOBandSet(MCU_HFXO_FREQ);

    /* enable HFRCO oscillator */
    CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);

    /* select HFRCO for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

    /* disable HFXO */
    CMU_OscillatorEnable(cmuOsc_HFXO, false, false);
}
