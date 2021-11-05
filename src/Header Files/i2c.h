/**
 * @file i2c.h
 * @author Mason Milligan
 * @date 2021-10-24
 * @brief Contains functions that enable I2C state machine functionality
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#ifndef HEADER_FILES_I2C_H_
#define HEADER_FILES_I2C_H_

/* System include statements */

/* Silicon Labs include statements */
#include "em_cmu.h"
#include "em_i2c.h"

/* The developer's include statements */
#include "scheduler.h"
#include "sleep_routines.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define I2C_EM          EM2 // block from entering energy mode 2
#define I2C_READ_BIT    1
#define I2C_WRITE_BIT   0
#define BYTE            8

//***********************************************************************************
// global variables
//***********************************************************************************
typedef struct
{
    bool enable;                // enable I2C on completion of i2c_open
    bool master;                // behave as I2C master
    uint32_t refFreq;           // I2C reference frequency
    uint32_t freq;              // I2C frequency
    I2C_ClockHLR_TypeDef clhr;               // clock low:high ratio
    uint32_t out_pin_route_scl; // SCL route to GPIO port/pin
    uint32_t out_pin_route_sda; // SDA route to GPIO port/pin
    bool out_pin_enable_scl;    // enable SCL route
    bool out_pin_enable_sda;    // enable SDA route
} I2C_OPEN_STRUCT;

//***********************************************************************************
// function prototypes
//***********************************************************************************
void i2c_open(I2C_TypeDef *i2c, I2C_OPEN_STRUCT *i2c_setup);
void i2c_start(I2C_TypeDef *i2c, uint32_t slave_addr, uint32_t slave_reg, bool read, uint32_t bytes_expected, uint32_t *rx_data, uint32_t tx_data, uint32_t callback);
bool i2c_busy(I2C_TypeDef *i2c);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);

#endif /* HEADER_FILES_I2C_H_ */
