#ifndef HMC5883L_H_
#define HMC5883L_H_
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>

#define USE_I2C_DRV

#define CFGA_REG  0x00   // config register A  (r/w)
#define CFGB_REG 0x01    // config register B  (r/w)
#define MODE_REG  0x02   // mode register   (r/w)

#define DATA_X_MSB_REG 0x03
#define DATA_X_LSB_REG 0x04

#define DATA_Z_MSB_REG 0x05
#define DATA_Z_LSB_REG 0x06

#define DATA_Y_MSB_REG 0x07
#define DATA_Y_LSB_REG 0x08

#define STATUS_REG   0x09  // status register (r)

#define IDTFA_REG 0x10    // identify register A (r)
#define IDTFB_REG 0x11    // identify register B (r)
#define IDTFC_REG 0x12    // identify register C (r)

#define HMC5883L_ADDR 0x1e
#define CONTINUE_CMD 0x00

void read_data(uint8_t *data, uint8_t len);
void init_hmc(void);

#endif
