#include "sl_i2c.h"
#include "Board.h"
#include "uart_term.h"


static I2C_Params params;
static I2C_Handle handle=NULL;
static I2C_Transaction transaction;

/*
 * 按官方API的文档描述，该驱动不支持多主机仲裁，就是想在FreeRTOS内，使用多线程来分别读取不同的I2C设备是不可行，经实践事实上也是如此。
 * Multi-master arbitration is not supported; therefore, this driver assumes it is the only I2C master on the bus.
 * This I2C driver's API set provides the ability to transmit and receive data over an I2C bus between the I2C master and I2C slave(s).
 */

#define DEBUG_ERROR
#undef DEBUG_ERROR

/*
 *  ======== i2cErrorHandler ========
 */
static void i2cErrorHandler(I2C_Transaction *transaction)
{
#ifdef DEBUG_ERROR
    switch (transaction->status) {
    case I2C_STATUS_TIMEOUT:
        UART_PRINT("I2C transaction timed out!\n\r");
        break;
    case I2C_STATUS_CLOCK_TIMEOUT:
        UART_PRINT("I2C serial clock line timed out!\n\r");
        break;
    case I2C_STATUS_ADDR_NACK:
        UART_PRINT("I2C slave address 0x%x not acknowledged!\n\r", transaction->slaveAddress);
        break;
    case I2C_STATUS_DATA_NACK:
        UART_PRINT("I2C data byte not acknowledged!\n\r");
        break;
    case I2C_STATUS_ARB_LOST:
        UART_PRINT("I2C arbitration to another master!\n\r");
        break;
    case I2C_STATUS_INCOMPLETE:
        UART_PRINT("I2C transaction returned before completion!\n\r");
        break;
    case I2C_STATUS_BUS_BUSY:
        UART_PRINT( "I2C bus is already in use!\n\r");
        break;
    case I2C_STATUS_CANCEL:
        UART_PRINT( "I2C transaction cancelled!\n\r");
        break;
    case I2C_STATUS_INVALID_TRANS:
        UART_PRINT("I2C transaction invalid!\n\r");
        break;
    case I2C_STATUS_ERROR:
        UART_PRINT( "I2C generic error!\n\r");
        break;
    default:
        UART_PRINT("I2C undefined error case!\n\r");
        break;
    }
#endif

}


void open_i2c(uint32_t i2c)
{
    /*
     * 按照官方的I2C Driver API描述 如下:
     * The I2C driver is designed to operate as an I2C master and will not function as an I2C slave.
     * Multi-master arbitration is not supported; therefore, this driver assumes it is the only I2C master on the bus.
     * This I2C driver's API set provides the ability to transmit and receive data over an I2C bus between the I2C master and I2C slave(s).
     *  The application is responsible for manipulating and interpreting the data.
     *  但这里使用 sem_wait,sem_post 是可以为Multi-master使用的。
     */
    I2C_Params_init(&params);
    params.bitRate= I2C_400kHz;
    handle = I2C_open(i2c,&params);

    if(handle == NULL){
        UART_PRINT("open I2C addr %x Error\r\n",i2c);
        while(1){}
    }
}

void close_i2c(void)
{
    I2C_close(handle);
}

bool i2c_write_bytes(uint32_t i2cbase ,uint8_t addr,uint8_t *buf,uint8_t len){
    bool status = 0;
    open_i2c(i2cbase);
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = addr;
    transaction.writeBuf = buf;
    transaction.writeCount = len;
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    status = I2C_transfer(handle, &transaction);
    if (!status) {
      i2cErrorHandler(&transaction);
    }
    I2C_close(handle);
    return status;
}



bool i2c_read_bytes(uint32_t i2cbase,uint8_t addr,uint8_t *buf,uint8_t len){
    bool status;
    open_i2c(i2cbase);
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = addr;
    transaction.writeBuf = NULL;
    transaction.writeCount = 0;
    transaction.readBuf = buf;
    transaction.readCount = len;
    status = I2C_transfer(handle, &transaction);
    if (!status) {
        i2cErrorHandler(&transaction);
    }
    I2C_close(handle);
    return status;
}

bool i2c_rw_cmd(uint32_t i2cbase,uint8_t addr,uint8_t *wbuf,uint8_t wlen,uint8_t *rbuf,uint8_t rlen)
{
    bool status;
    open_i2c(i2cbase);
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = addr;
    transaction.writeBuf = wbuf;
    transaction.writeCount = wlen;
    transaction.readBuf = rbuf;
    transaction.readCount = rlen;
    status = I2C_transfer(handle, &transaction);
    if (!status) {
        i2cErrorHandler(&transaction);
    }
    I2C_close(handle);
    return status;
}

