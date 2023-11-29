#include "mpu6050.h"
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>

extern uint32_t tickPeriod;
extern uint32_t systemClock;

void MPU_DelayMs(uint32_t num) {
    vTaskDelay(num);
}

u8 MPU_WriteByte(uint8_t device_address, u8 reg, u8 data) {
    uint8_t writeBuffer[2];
    writeBuffer[0] = reg;
    writeBuffer[1] = data;
    write_bytes(device_address, writeBuffer, 2);
    return 0;
}

u8 MPU_ReadByte(uint8_t device_address, u8 reg) {
    uint8_t res = 0;
    write_read_cmd(device_address, &reg, 1, &res, 1);
    return res;
}

u8 MPU_Init(void) {
    printf("MPU_init starting.....\r\n");

    u8 res;

    MPU_WriteByte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0x80);
    DelayUs(200);
    printf("wake up mpu6050");
    MPU_WriteByte(MPU_ADDR, MPU_PWR_MGMT2_REG, 0X00);

    MPU_SetGyroFsr(3);
    MPU_SetAccelFsr(0);
    MPU_SetRate(50);
    MPU_WriteByte(MPU_ADDR, MPU_INT_EN_REG, 0X00);
    MPU_WriteByte(MPU_ADDR, MPU_USER_CTRL_REG, 0X00);
    MPU_WriteByte(MPU_ADDR, MPU_FIFO_EN_REG, 0X00);
    MPU_WriteByte(MPU_ADDR, MPU_INTBP_CFG_REG, 0X80);
    res = MPU_ReadByte(MPU_ADDR, MPU_DEVICE_ID_REG);
    printf("read MPU_DEVICE_ID_REG %x\r\n", res);
    if (res == MPU_ADDR) {
        MPU_WriteByte(MPU_ADDR, MPU_PWR_MGMT1_REG, 0X01);
        MPU_WriteByte(MPU_ADDR, MPU_PWR_MGMT2_REG, 0X00);
        MPU_SetRate(50);
    } else
        return 1;
    return 0;
}

u8 MPU_SetGyroFsr(u8 fsr) {
    return MPU_WriteByte(MPU_ADDR, MPU_GYRO_CFG_REG, fsr << 3);
}

u8 MPU_SetAccelFsr(u8 fsr) {
    return MPU_WriteByte(MPU_ADDR, MPU_ACCEL_CFG_REG, fsr << 3);
}

u8 MPU_SetLPF(u16 lpf) {
    u8 data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_WriteByte(MPU_ADDR, MPU_CFG_REG, data);
}

u8 MPU_SetRate(u16 rate) {
    u8 data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_WriteByte(MPU_ADDR, MPU_SAMPLE_RATE_REG, data);
    return MPU_SetLPF(rate / 2);
}

short MPU_GetTemperature(void) {
    u8 buf[2];
    short raw;
    float temp;
    MPU_ReadData(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
    ;
}

u8 MPU_GetGyroscope(short *gx, short *gy, short *gz) {
    u8 buf[6], res;
    res = MPU_ReadData(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0) {
        *gx = ((u16)buf[0] << 8) | buf[1];
        *gy = ((u16)buf[2] << 8) | buf[3];
        *gz = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}

u8 MPU_GetAccelerometer(short *ax, short *ay, short *az) {
    u8 buf[6], res;
    res = MPU_ReadData(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0) {
        *ax = ((u16)buf[0] << 8) | buf[1];
        *ay = ((u16)buf[2] << 8) | buf[3];
        *az = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}

u8 MPU_WriteData(u8 slave_address, u8 reg, u8 len, u8 *buf) {
    write_bytes(slave_address, &reg, 1);
    write_bytes(slave_address, buf, len);
    return 0;
}

u8 MPU_ReadData(u8 slave_address, u8 reg, u8 len, u8 *buf) {
    write_read_cmd(slave_address, &reg, 1, buf, len);
    return 0;
}
