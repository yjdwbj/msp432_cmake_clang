#ifndef I2C_H_
#define I2C_H_

#include <ti/devices/msp432e4/driverlib/driverlib.h>

#include <stdint.h>
#include <stdbool.h>
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

/* Defines for I2C bus parameters */
#define I2C_NUM_DATA    32


/* Defines for I2C State Machine */
#define I2C_MASTER_IDLE 0x0
#define I2C_MASTER_TX   0x1
#define I2C_MASTER_RX   0x2
#define I2C_MASTER_ANAK 0x3
#define I2C_MASTER_DNAK 0x4
#define I2C_MASTER_ALST 0x5
#define I2C_MASTER_UNKN 0x6

/* Enumerated Data Types for I2C State Machine */
enum I2C_MASTER_STATE
{
    I2C_OP_IDLE = 0,
    I2C_OP_FIFO,
    I2C_OP_TXDATA,
    I2C_OP_RXDATA,
    I2C_OP_STOP,
    I2C_ERR_STATE
};

/* Variables for I2C data and state machine */
volatile uint8_t  sendMasterTxData[I2C_NUM_DATA];
volatile uint8_t  bufLen;
uint8_t           getMasterRxData[I2C_NUM_DATA];
volatile uint8_t  setMasterCurrState;
volatile uint8_t  setMasterPrevState;
volatile bool     setI2CDirection;
//volatile uint8_t  setMasterBytes       = I2C_NUM_DATA;
//const    uint8_t  setMasterBytesLength = I2C_NUM_DATA;
volatile uint8_t  setReadPointerStart;


/* The control table used by the uDMA controller.  This table must be aligned
 * to a 1024 byte boundary. */
#if defined(__ICCARM__)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(__TI_ARM__)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif



void I2C2_Init();
void I2C1_Init();

uint32_t I2C_ReadByte(uint8_t slave_address);
void I2C_ReadData(uint32_t base,uint8_t slave_address,uint8_t reg,uint8_t *data,uint8_t len);
void I2C_WriteData(uint32_t base,uint8_t slave_address,uint8_t reg,uint8_t *data,uint8_t len);
uint32_t I2C_WriteByte(uint8_t slave_address,uint8_t byte);
void DelayUs(uint32_t num);
void I2C_SendData(u32 data);
#endif
