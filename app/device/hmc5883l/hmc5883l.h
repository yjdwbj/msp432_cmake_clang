#ifndef HMC5883L_H_
#define HMC5883L_H_
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>

#define USE_I2C_DRV

#define CFGA_REG  0x00   // 配置寄存器 A  (r/w)
#define CFGB_REG  0x01   // 配置寄存器 B  (r/w)
#define MODE_REG  0x02   // 模式寄存器   (r/w)

#define DATA_X_MSB_REG 0x03  // 数据输出 X MSB 寄存器 （r)
#define DATA_X_LSB_REG 0x04  // 数据输出 X LSB 寄存器 （r)

#define DATA_Z_MSB_REG 0x05  // 数据输出 X MSB 寄存器 （r)
#define DATA_Z_LSB_REG 0x06  // 数据输出 X LSB 寄存器 （r)

#define DATA_Y_MSB_REG 0x07  // 数据输出 X MSB 寄存器 （r)
#define DATA_Y_LSB_REG 0x08  // 数据输出 X LSB 寄存器 （r)

#define STATUS_REG   0x09  // 状态寄存器 (r)


#define IDTFA_REG   0x10  // 识别寄存器A (r)
#define IDTFB_REG   0x11  // 识别寄存器B (r)
#define IDTFC_REG   0x12  // 识别寄存器C (r)


#define HMC5883L_ADDR 0x1e  // HMC5883L I2C 地址。
#define CONTINUE_CMD 0x00   //

void read_data(uint8_t *data, uint8_t len);
void init_hmc(void);

#endif
