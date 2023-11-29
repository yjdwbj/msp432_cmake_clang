#ifndef SIMPLELINKE_I2C_H_
#define SIMPLELINKE_I2C_H_
// simple link driver API

#include <ti/drivers/I2C.h>
#include <stdint.h>
#include <stdbool.h>
#include "Board.h"

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t


// 从设备地址，这里是mpu6050的地址。
#define SLAVE_ADDR 0x68



void close_i2c(void);
void open_i2c(uint32_t i2cbase);
bool i2c_write_bytes(uint32_t i2cbase,uint8_t addr,uint8_t *buf,uint8_t len);
bool i2c_rw_cmd(uint32_t i2cbase,uint8_t addr,uint8_t *wbuf,uint8_t wlen,uint8_t *rbuf,uint8_t rlen);
bool i2c_read_bytes(uint32_t i2cbase,uint8_t addr,uint8_t *buf,uint8_t len);


#endif
