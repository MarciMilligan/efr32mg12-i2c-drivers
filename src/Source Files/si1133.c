/**
 * @file si1133.c
 * @author Mason Milligan
 * @date 2021-10-24
 * @brief Contains functions that enable interaction with the Si1133 sensor
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "si1133.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// Private variables
//***********************************************************************************
static uint32_t si1133_read_result;
static uint32_t si1133_write_data;

//***********************************************************************************
// Private functions
//***********************************************************************************
void si1133_configure(I2C_TypeDef *i2c);

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Function that configures one of the Mighty Gecko's I2C peripherals to interact
 *  with the Si1133 sensor
 *
 * @details
 *  This function configures an I2C peripheral to properly communicate with an
 *  Si1133 sensor and prepares the I2C state machine for interaction with the
 *  sensor.
 *
 * @note
 *  This function is typically called once to perform initial configurations for
 *  interaction with the sensor.
 *
 ******************************************************************************/
void si1133_i2c_open(void)
{
    I2C_OPEN_STRUCT i2c_si1133_struct;
    timer_delay(SI1133_STARTUP_DELAY);          // wait 25 ms for Si1133 startup
    i2c_si1133_struct.enable = true;
    i2c_si1133_struct.master = true;
    i2c_si1133_struct.refFreq = 0;   // use currently configured reference clock
    i2c_si1133_struct.freq = I2C_FREQ_FAST_MAX;
    i2c_si1133_struct.clhr = i2cClockHLRAsymetric;  // 6:3 is closest to 13:6
    i2c_si1133_struct.out_pin_route_scl = SI1133_SCL_LOC;
    i2c_si1133_struct.out_pin_route_sda = SI1133_SDA_LOC;
    i2c_si1133_struct.out_pin_enable_scl = true;
    i2c_si1133_struct.out_pin_enable_sda = true;
    i2c_open(I2C1, &i2c_si1133_struct);
    si1133_configure(I2C1);
}

/***************************************************************************//**
 * @brief
 *  Function that invokes I2C communication with the Si1133 sensor to collect data
 *  from the sensor
 *
 * @details
 *  This function begins I2C state machine operation to gather the part ID from
 *  the Si1133 sensor via I2C.
 *
 * @note
 *  This function results in @ref si1133_read_result being overwritten.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being used
 *
 * @param[in] callback
 *  Callback value for scheduler so that an event may be triggered upon
 *  completion of the I2C interaction
 *
 ******************************************************************************/
void si1133_read(I2C_TypeDef *i2c, uint32_t reg, uint32_t bytes, uint32_t callback)
{
    i2c_start(i2c, SI1133_I2C_ADDRESS, reg, true, bytes, &si1133_read_result, 0, callback);
}

/***************************************************************************//**
 * @brief
 *  Function that invokes I2C communication with the Si1133 sensor to write data
 *  to the sensor
 *
 * @details
 *  This function begins I2C state machine operation write data to the Si1133.
 *
 * @note
 *  This function reads @ref si1133_write_data.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being used
 *
 * @param[in] callback
 *  Callback value for scheduler so that an event may be triggered upon
 *  completion of the I2C interaction
 *
 ******************************************************************************/
void si1133_write(I2C_TypeDef *i2c, uint32_t reg, uint32_t bytes, uint32_t callback)
{
    i2c_start(i2c, SI1133_I2C_ADDRESS, reg, false, bytes, 0, si1133_write_data, callback);
}

/***************************************************************************//**
 * @brief
 *  Function to configure the Si1133 sensor to be used by the I2C state machine
 *
 * @details
 *  This function sets the Si1133 sensor's parameters to measure white ambient
 *  light and function with the I2C state machine.
 *
 * @note
 *  This function is typically called once by @ref si1133_i2c_open to prepare the
 *  sensor for proper I2C communication.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being used
 *
 ******************************************************************************/
void si1133_configure(I2C_TypeDef *i2c)
{
    uint32_t cmd_ctr_result;

    /* read RESPONSE0 register */
    si1133_read(i2c, SI1133_RESPONSE0_REG, SI1133_RESPONSE0_REG_BYTES, NULL_CALLBACK);
    while(i2c_busy(i2c));   // wait for read to finish

    /* store command count */
    cmd_ctr_result = si1133_read_result & SI1133_CMD_CTR_MASK;

    /* write value for ADCMUX bit field to INPUT0 register */
    si1133_write_data = SI1133_ADCMUX_WHITE;
    si1133_write(i2c, SI1133_INPUT0_REG, SI1133_INPUT0_REG_BYTES, NULL_CALLBACK);
    while(i2c_busy(i2c));   // wait for write to finish

    /* send command to update ADCMUX bit field */
    si1133_write_data = SI11333_PARAM_WRITE_MASK | SI1133_ADCCONFIG0_ADDRESS;
    si1133_write(i2c, SI1133_COMMAND_REG, SI1133_COMMAND_REG_BYTES, NULL_CALLBACK);
    while(i2c_busy(i2c));

    /* check RESPONSE0 error indicator */
    si1133_read(i2c, SI1133_RESPONSE0_REG, SI1133_RESPONSE0_REG_BYTES, NULL_CALLBACK);
    while(i2c_busy(i2c));
    EFM_ASSERT(!(si1133_read_result & SI1133_RESPONSE0_ERROR_MASK));

    /* verify that command counter has increased by 1 */
    EFM_ASSERT(++cmd_ctr_result == (si1133_read_result & SI1133_CMD_CTR_MASK));

    /* prepare to set Si1133 channel 0 as active */
    si1133_write_data = SI1133_CHANNEL_0_ACTIVE;
    si1133_write(i2c, SI1133_INPUT0_REG, SI1133_INPUT0_REG_BYTES, NULL_CALLBACK);
    while(i2c_busy(i2c));

    /* send command to update CHAN_LIST */
    si1133_write_data = SI11333_PARAM_WRITE_MASK | Si1133_CHAN_LIST_ADDRESS;
    si1133_write(i2c, SI1133_COMMAND_REG, SI1133_COMMAND_REG_BYTES, NULL_CALLBACK);
    while(i2c_busy(i2c));

    /* check RESPONSE0 error indicator */
    si1133_read(i2c, SI1133_RESPONSE0_REG, SI1133_RESPONSE0_REG_BYTES, NULL_CALLBACK);
    while(i2c_busy(i2c));
    EFM_ASSERT(!(si1133_read_result & SI1133_RESPONSE0_ERROR_MASK));

    /* verify that command counter has increased by 1 */
    EFM_ASSERT(++cmd_ctr_result == (si1133_read_result & SI1133_CMD_CTR_MASK));
}

/***************************************************************************//**
 * @brief
 *  Function that returns @ref si1133_read_result, which is where data received
 *  via I2C is stored
 *
 * @details
 *  This function simply returns the value stored in @ref si1133_read_result.
 *
 * @note
 *  The static variable being returned by this function is subject to being updated
 *  at any time.
 *
 * @param[out] si1133_read_result
 *  Variable containing data received via I2C
 *
 ******************************************************************************/
uint32_t si1133_get_result(void)
{
    return si1133_read_result;
}

/***************************************************************************//**
 * @brief
 *  Function that invokes a FORCE measurement on the Si1133 sensor.
 *
 * @details
 *  This function sends a command to the Si1133 sensor to initiate measurements
 *  and output them as specified by CHAN_LIST.
 *
 * @note
 *  This function should not be run until the CHAN_LIST parameter has been
 *  configured on the Si1133 sensor.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being used
 *
 ******************************************************************************/
void si1133_force_command(I2C_TypeDef *i2c)
{
    si1133_write_data = SI1133_COMMAND_FORCE;
    si1133_write(i2c, SI1133_COMMAND_REG, SI1133_COMMAND_REG_BYTES, NULL_CALLBACK);
}
