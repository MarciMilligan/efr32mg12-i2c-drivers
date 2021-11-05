/**
 * @file si1133.h
 * @author Mason Milligan
 * @date 2021-10-24
 * @brief Contains functions that enable interaction with the Si1133 sensor
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_SI1133_H_
#define HEADER_FILES_SI1133_H_

/* System include statements */

/* Silicon Labs include statements */

/* The developer's include statements */
#include "i2c.h"
#include "brd_config.h"
#include "HW_delay.h"

//***********************************************************************************
// defined files
//***********************************************************************************
/* empty scheduler callback */
#define NULL_CALLBACK                  0

/* time to wait between startup and interaction with Si1133 sensor */
#define SI1133_STARTUP_DELAY             25      // startup time in ms

/* Si1133 I2C definitions */
#define SI1133_I2C_ADDRESS              0x55
#define SI1133_PART_ID_REG              0x00
#define SI1133_PART_ID_REG_BYTES        1

#define SI1133_RESPONSE0_REG            0x11
#define SI1133_RESPONSE0_REG_BYTES      1
#define SI1133_CMD_CTR_MASK             0xF
#define SI1133_RESPONSE0_ERROR_MASK     0x10

#define SI1133_INPUT0_REG               0x0A
#define SI1133_INPUT0_REG_BYTES         1
#define SI1133_ADCMUX_WHITE             0b01011
#define SI1133_CHANNEL_0_ACTIVE         1

#define SI1133_COMMAND_REG              0x0B
#define SI1133_COMMAND_REG_BYTES        1
#define SI1133_COMMAND_FORCE            0x11

#define SI1133_HOSTOUT0_REG             0x13
#define SI1133_HOSTOUT0_REG_BYTES       2

#define SI11333_PARAM_WRITE_MASK        0x80
#define SI11333_PARAM_READ_MASK         0x40
#define SI1133_ADCCONFIG0_ADDRESS       0x02
#define Si1133_CHAN_LIST_ADDRESS        0x01

//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************
void si1133_i2c_open(void);
void si1133_read(I2C_TypeDef *i2c, uint32_t reg, uint32_t bytes, uint32_t callback);
void si1133_write(I2C_TypeDef *i2c, uint32_t reg, uint32_t bytes, uint32_t callback);
uint32_t si1133_get_result(void);
void si1133_force_command(I2C_TypeDef *i2c);

#endif /* HEADER_FILES_SI1133_H_ */
