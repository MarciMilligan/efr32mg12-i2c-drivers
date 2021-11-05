/**
 * @file i2c.c
 * @author Mason Milligan
 * @date 2021-10-24
 * @brief Contains functions that enable I2C state machine functionality
 *
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "i2c.h"

//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// Private variables
//***********************************************************************************
enum DEFINED_STATES
{
    idle,       // idle state
    init,       // initialize i2c communication
    reg_sel,    // select register
    rep_start,  // repeated start
    get_data,   // get byte via i2c
    send_data,  // send byte via i2c
    stop        // send stop command
};

typedef struct
{
    enum DEFINED_STATES state;  // state of I2C state machine
    I2C_TypeDef *i2c;           // pointer to base address of I2C peripheral
    uint32_t slave_addr;        // I2C address of slave
    uint32_t slave_reg;         // slave register being accessed
    bool read;                  // 1 = read data; 0 = write data
    uint32_t bytes_expected;    // total number of bytes in transmission
    uint32_t bytes_received;    // variable to track number of bytes transmitted
    uint32_t *rx_data;          // pointer to store received data
    uint32_t tx_data;           // data to transmit
    uint32_t callback;          // callback function to schedule when transmission is completed
    volatile bool busy;         // busy status if I2C state machine
} I2C_STATE_MACHINE;

static I2C_STATE_MACHINE i2c_sm_0;
static I2C_STATE_MACHINE i2c_sm_1;

//***********************************************************************************
// Private functions
//***********************************************************************************
static void i2c_bus_reset(I2C_TypeDef *i2c);
static void i2c_send(I2C_STATE_MACHINE *i2c_sm);
static void i2c_receive(I2C_STATE_MACHINE *i2c_sm);
static void i2c_ACK(I2C_STATE_MACHINE *i2c_sm);
static void i2c_NACK(I2C_STATE_MACHINE *i2c_sm);
static void i2c_end(I2C_STATE_MACHINE *i2c_sm);

//***********************************************************************************
// Global functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 *  Driver to initialize and configure an I2C peripheral
 *
 * @details
 *  This routine is a low-level driver that enables usage of an I2C peripheral,
 *  enables interrupts from the peripheral, and prepares the I2C state machine.
 *
 * @note
 *  This function is typically called once per I2C peripheral, as it is used for
 *  initial setup.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being opened
 *
 * @param[in] i2c_setup
 *  Struct that the calling routine will use to set the parameters for I2C
 *  operation
 *
 ******************************************************************************/
void i2c_open(I2C_TypeDef *i2c, I2C_OPEN_STRUCT *i2c_setup)
{
    I2C_Init_TypeDef i2c_values;

    /* enable routed clock to I2C peripheral */
    if(i2c == I2C0)
    {
        CMU_ClockEnable(cmuClock_I2C0, true);
    }
    else if(i2c == I2C1)
    {
        CMU_ClockEnable(cmuClock_I2C1, true);
    }

    /* verify proper I2C clock operation */
    if((i2c->IF & 0x01) == 0)
    {
        i2c->IFS = 0x01;
        EFM_ASSERT(i2c->IF & 0x01);
        i2c->IFC = 0x01;
    }
    else
    {
        i2c->IFC = 0x01;
        EFM_ASSERT(!(i2c->IF & 0x01));
    }

    /* prepare I2C_Init_TypeDef values */
    i2c_values.enable = i2c_setup->enable;
    i2c_values.master = i2c_setup->master;
    i2c_values.refFreq = i2c_setup->refFreq;
    i2c_values.freq = i2c_setup->freq;
    i2c_values.clhr = i2c_setup->clhr;

    /* initialize i2c peripheral */
    I2C_Init(i2c, &i2c_values);

    /* configure i2c pin routes based on input STRUCT */
    i2c->ROUTELOC0 = i2c_setup->out_pin_route_scl;
    i2c->ROUTELOC0 |= i2c_setup->out_pin_route_sda;

    /* configure i2c pin enables based on input STRUCT */
    i2c->ROUTEPEN = I2C_ROUTEPEN_SCLPEN * i2c_setup->out_pin_enable_scl;
    i2c->ROUTEPEN |= I2C_ROUTEPEN_SDAPEN * i2c_setup->out_pin_enable_sda;

    /* clear I2C interrupt flags */
    i2c->IFC |= I2C_IEN_ACK;
    i2c->IFC |= I2C_IEN_NACK;
    i2c->IFC |= I2C_IEN_RXDATAV;
    i2c->IFC |= I2C_IEN_MSTOP;

    /* set interrupt enable bits for I2C */
    uint32_t IEN_SET;
    IEN_SET = I2C_IEN_ACK;
    IEN_SET |= I2C_IEN_NACK;
    IEN_SET |= I2C_IEN_RXDATAV;
    IEN_SET |= I2C_IEN_MSTOP;
    i2c->IEN = IEN_SET;

    /* enable interrupts globally */
    if(i2c == I2C0)
    {
        NVIC_EnableIRQ(I2C0_IRQn);
    }
    else if(i2c == I2C1)
    {
        NVIC_EnableIRQ(I2C1_IRQn);
    }

    /* initialize state machine as not busy */
    if(i2c == I2C0)
    {
        i2c_sm_0.busy = 0;
    }
    else if(i2c == I2C1)
    {
        i2c_sm_1.busy = 0;
    }

    /* perform bus reset */
    i2c_bus_reset(i2c);
}

/***************************************************************************//**
 * @brief
 *  Function to reset I2C state machine of given I2C peripheral and all connected
 *  external I2C devices
 *
 * @details
 *  This function resets the Mighty Gecko's internal I2C state machine and the
 *  I2C state machine of any external peripheral connected via I2C.
 *
 * @note
 *  This function is typically called at the beginning of any I2C interaction.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being reset
 *
 ******************************************************************************/
void i2c_bus_reset(I2C_TypeDef *i2c)
{
    uint32_t ien_state;
    i2c->CMD = I2C_CMD_ABORT;                   // reset internal state machine
    ien_state = i2c->IEN;                       // store state of IEN register
    i2c->IEN = 0;                               // disable i2c interrupts
    i2c->IFC = i2c->IF;                         // clear raised flags
    i2c->CMD = I2C_CMD_CLEARTX;                 // clear transmit buffer
    i2c->CMD = I2C_CMD_START | I2C_CMD_STOP; // simultaneously set START and STOP commands
    while(!(i2c->IF & I2C_IF_MSTOP));          // stall until STOP completed
    i2c->IFC = i2c->IF;                         // clear all raised flags
    i2c->CMD = I2C_CMD_ABORT;                   // reset internal state machine
    i2c->IEN = ien_state;                       // restore IEN register state
}

/***************************************************************************//**
 * @brief
 *  Function that invokes I2C communication with an external device
 *
 * @details
 *  This function begins an exchange via I2C with an external device and invokes
 *  the I2C state machine behavior.
 *
 * @note
 *  A given I2C state machine may only have one active I2C exchange at any time.
 *  If a previous exchange is incomplete, a new one may not yet be started.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being used
 *
 * @param[in] slave_addr
 *  I2C address of external device that will be communicated with
 *
 * @param[in] slave_reg
 *  Register within external device that will be read or written to via I2C
 *
 * @param[in] bytes_expected
 *  Number of bytes expected to be sent or received via I2C
 *
 * @param[in] data
 *  Pointer to data that will be sent or where received data will be stored
 *
 * @param[in] callback
 *  Event callback for scheduler
 *
 ******************************************************************************/
void i2c_start(I2C_TypeDef *i2c, uint32_t slave_addr, uint32_t slave_reg, bool read, uint32_t bytes_expected, uint32_t *rx_data, uint32_t tx_data, uint32_t callback)
{
    I2C_STATE_MACHINE *i2c_sm;

    /* vary active state machine struct by active I2C peripheral */
    if(i2c == I2C0)
    {
        i2c_sm = &i2c_sm_0;
    }
    else
    {
        i2c_sm = &i2c_sm_1;
    }

    /* wait for state machine to not be busy */
    while(i2c_sm->busy);

    /* verify that I2C peripheral is idle */
    EFM_ASSERT((i2c->STATE & _I2C_STATE_STATE_MASK) == I2C_STATE_STATE_IDLE);

    /* block from entering EM2 while I2C is active */
    sleep_block_mode(I2C_EM);

    /* prepare state machine struct */
    i2c_sm->state = init;
    i2c_sm->i2c = i2c;
    i2c_sm->slave_addr = slave_addr;
    i2c_sm->slave_reg = slave_reg;
    i2c_sm->read = read;
    i2c_sm->bytes_expected = bytes_expected;
    i2c_sm->bytes_received = 0;
    i2c_sm->callback = callback;
    i2c_sm->busy = true;

    /* reset output data if reading and use proper type for read/write */
    if(read)
    {
        i2c_sm->rx_data = rx_data;
        *i2c_sm->rx_data = 0;
    }
    else
    {
        i2c_sm->tx_data = tx_data;
    }

    /* begin state machine operation */
    i2c_send(i2c_sm);
}

/***************************************************************************//**
 * @brief
 *  Function run during I2C state machine functionality to perform a state's
 *  respective sending behavior
 *
 * @details
 *  This function sends data via I2C that varies based on the current status of
 *  the I2C state machine.
 *
 * @note
 *  This function is typically run when initiating an exchange via I2C or when
 *  responding to a message via I2C.
 *
 * @param[in] i2c_sm
 *  Struct containing I2C state machine information
 *
 ******************************************************************************/
void i2c_send(I2C_STATE_MACHINE *i2c_sm)
{
    switch(i2c_sm->state)
    {
        case init:
            i2c_sm->i2c->CMD = I2C_CMD_START;
            i2c_sm->i2c->TXDATA = (i2c_sm->slave_addr << 1) | I2C_WRITE_BIT;
            break;
        case reg_sel:
            i2c_sm->i2c->TXDATA = i2c_sm->slave_reg;
            break;
        case rep_start:
            i2c_sm->i2c->CMD = I2C_CMD_START;
            i2c_sm->i2c->TXDATA = (i2c_sm->slave_addr << 1) | i2c_sm->read;
            break;
        case get_data:
            i2c_sm->i2c->CMD = I2C_CMD_ACK;
            break;
        case send_data:
            i2c_sm->i2c->TXDATA = i2c_sm->tx_data >> (BYTE * (i2c_sm->bytes_expected - i2c_sm->bytes_received - 1));
            break;
        case stop:
            /* send NACK first if reading */
            if(i2c_sm->read)
            {
                i2c_sm->i2c->CMD = I2C_CMD_NACK;
            }
            i2c_sm->i2c->CMD = I2C_CMD_STOP;
            break;
        default:
            EFM_ASSERT(false);
            break;
    }
}

/***************************************************************************//**
 * @brief
 *  Function run during I2C communication when data is received
 *
 * @details
 *  This function is run when data has been received via I2C and is in the receive
 *  buffer.
 *
 * @note
 *  This function is typically only called in response to an RXDATAV interrupt.
 *
 * @param[in] i2c_sm
 *  Struct containing I2C state machine information
 *
 ******************************************************************************/
void i2c_receive(I2C_STATE_MACHINE *i2c_sm)
{
    switch(i2c_sm->state)
    {
        case get_data:
            /* collect byte from receive buffer */
            *i2c_sm->rx_data <<= BYTE; // bit shift current data left by one byte
            *i2c_sm->rx_data |= i2c_sm->i2c->RXDATA;   // OR in new byte
            i2c_sm->bytes_received++;               // new byte received

            /* check whether expected number of bytes have been received */
            if(i2c_sm->bytes_received == i2c_sm->bytes_expected)
            {
                i2c_sm->state = stop;
            }

            /* send response depending on current state */
            i2c_send(i2c_sm);
            break;
        default:
            EFM_ASSERT(false);
            break;
    }
}

/***************************************************************************//**
 * @brief
 *  Function run during I2C communication when an ACK is received
 *
 * @details
 *  This function is run when an ACK has been received via I2C. The following
 *  behavior is determined based on the current state of the I2C state machine.
 *
 * @note
 *  This function is typically only called in response to an ACK interrupt.
 *
 * @param[in] i2c_sm
 *  Struct containing I2C state machine information
 *
 ******************************************************************************/
void i2c_ACK(I2C_STATE_MACHINE *i2c_sm)
{
    switch(i2c_sm->state)
    {
        case init:
            /* change state and send response */
            i2c_sm->state = reg_sel;
            i2c_send(i2c_sm);
            break;
        case reg_sel:
            /* determine next state based on whether we are reading or writing */
            i2c_sm->state = i2c_sm->read ? rep_start : send_data;
            i2c_send(i2c_sm);
            break;
        case rep_start:
            /* change state to begin waiting for data */
            i2c_sm->state = get_data;
            break;
        case send_data:
            /* update bytes received and check if transmission is done */
            i2c_sm->bytes_received++;
            if(i2c_sm->bytes_received == i2c_sm->bytes_expected)
            {
                i2c_sm->state = stop;
                i2c_send(i2c_sm);
            }
            break;
        default:
            EFM_ASSERT(false);
            break;
    }
}

/***************************************************************************//**
 * @brief
 *  Function run during I2C communication when a NACK is received
 *
 * @details
 *  This function is run when a NACK has been received via I2C. The following
 *  behavior is determined based on the current state of the I2C state machine.
 *
 * @note
 *  This function is typically only called in response to a NACK interrupt.
 *
 * @param[in] i2c_sm
 *  Struct containing I2C state machine information
 *
 ******************************************************************************/
void i2c_NACK(I2C_STATE_MACHINE *i2c_sm)
{
    switch(i2c_sm->state)
    {
        case rep_start:
            /* retry repeated start if NACK received */
            i2c_send(i2c_sm);
            break;
        default:
            EFM_ASSERT(false);
            break;
    }
}

/***************************************************************************//**
 * @brief
 *  Function to end an I2C interaction
 *
 * @details
 *  This function ends an I2C interaction by resetting the I2C state machine and
 *  unblocking sleep modes that were restricted by I2C.
 *
 * @note
 *  This function is typically only called in response to an MSTOP interrupt.
 *
 * @param[in] i2c_sm
 *  Struct containing I2C state machine information
 *
 ******************************************************************************/
void i2c_end(I2C_STATE_MACHINE *i2c_sm)
{
    i2c_sm->busy = false;
    i2c_sm->state = idle;
    sleep_unblock_mode(I2C_EM);
    add_scheduled_event(i2c_sm->callback);
}

/***************************************************************************//**
 * @brief
 *  Function to report the busy status of the I2C state machine associated with
 *  the given I2C peripheral
 *
 * @details
 *  This function returns the busy status variable of the selected I2C state
 *  machine variable.
 *
 * @note
 *  This function may be called by higher-level routines to check the status of
 *  the private I2C state machine variable.
 *
 * @param[in] i2c
 *  Pointer to the base peripheral address of the I2C peripheral being used
 *
 * @param[out] busy
 *  Boolean indicating the busy status of the selected I2C state machine. True
 *  indicates busy; false indicates not busy.
 *
 ******************************************************************************/
bool i2c_busy(I2C_TypeDef *i2c)
{
    I2C_STATE_MACHINE *i2c_sm;

    /* select state machine struct by given I2C peripheral */
    if(i2c == I2C0)
    {
        i2c_sm = &i2c_sm_0;
    }
    else
    {
        i2c_sm = &i2c_sm_1;
    }

    /* return busy status of active state machine struct */
    return i2c_sm->busy;
}

/***************************************************************************//**
 * @brief
 *  Interrupt handler to respond to I2C-related interrupts
 *
 * @details
 *  This function determines the nature of the I2C interrupt that occurred and
 *  calls the relevant function for the event that occurred.
 *
 * @note
 *  This function should not be called manually, as it is intended to only be
 *  called by the interrupt controller when an interrupt occurs.
 *
 ******************************************************************************/
void I2C0_IRQHandler(void)
{
    uint32_t int_flag;
    int_flag = I2C0->IF & I2C0->IEN; // AND interrupt flag with enable register to preserve only relevant bits
    I2C0->IFC = int_flag;               // clear interrupt flag register

    /* check for ACK interrupt */
    if(int_flag & I2C_IF_ACK)
    {
        i2c_ACK(&i2c_sm_0);
    }

    /* check for NACK interrupt */
    if(int_flag & I2C_IF_NACK)
    {
        i2c_NACK(&i2c_sm_0);
    }

    /* check for RXDATAV interrupt */
    if(int_flag & I2C_IF_RXDATAV)
    {
        i2c_receive(&i2c_sm_0);
    }

    /* check for MSTOP interrupt */
    if(int_flag & I2C_IF_MSTOP)
    {
        i2c_end(&i2c_sm_0);
    }
}

/***************************************************************************//**
 * @brief
 *  Interrupt handler to respond to I2C-related interrupts
 *
 * @details
 *  This function determines the nature of the I2C interrupt that occurred and
 *  calls the relevant function for the event that occurred.
 *
 * @note
 *  This function should not be called manually, as it is intended to only be
 *  called by the interrupt controller when an interrupt occurs.
 *
 ******************************************************************************/
void I2C1_IRQHandler(void)
{
    uint32_t int_flag;
    int_flag = I2C1->IF & I2C1->IEN; // AND interrupt flag with enable register to preserve only relevant bits
    I2C1->IFC = int_flag;               // clear interrupt flag register

    /* check for ACK interrupt */
    if(int_flag & I2C_IF_ACK)
    {
        i2c_ACK(&i2c_sm_1);
    }

    /* check for NACK interrupt */
    if(int_flag & I2C_IF_NACK)
    {
        i2c_NACK(&i2c_sm_1);
    }

    /* check for RXDATAV interrupt */
    if(int_flag & I2C_IF_RXDATAV)
    {
        i2c_receive(&i2c_sm_1);
    }

    /* check for MSTOP interrupt */
    if(int_flag & I2C_IF_MSTOP)
    {
        i2c_end(&i2c_sm_1);
    }
}
