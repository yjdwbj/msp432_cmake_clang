#include "hmc5883l.h"
#include "../../driver/i2c/sl_i2c.h"
#include "../../driver/i2c/i2c.h"
#include "uart_term.h"


void read_data(uint8_t *data, uint8_t len){

    uint8_t writeBuffer[2];
    writeBuffer[0] = MODE_REG; /*  mode register */
    writeBuffer[1] = 0x00;     /*  0x1单一模式 ,0x00 连续。*/

    i2c_write_bytes(Board_I2C_HMC5883L,HMC5883L_ADDR,writeBuffer,2);

    writeBuffer[0] = 0x3; /* begin XB  */
    i2c_rw_cmd(Board_I2C_HMC5883L,HMC5883L_ADDR,writeBuffer,1,data,len);

}
