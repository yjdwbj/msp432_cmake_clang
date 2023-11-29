#ifndef BMP180_H_
#define BMP180_H_
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>
#define USE_I2C_DRV


// 0xEE='0b11101110'  0x77='0b1110111' 7-bit , 后一位 1，0 代表读，写。
#define BMP180_SlaveAddress 0x77 // 7-bit address 定义器件在IIC总线中的从地址,这里文档写的是0xEE,I2C应该只写7-bit地址才有效的。

#define BMP_AC1_ADDR        0xAA // 定义校准寄存器的地址
#define BMP_AC2_ADDR        0xAC
#define BMP_AC3_ADDR        0xAE
#define BMP_AC4_ADDR        0xB0
#define BMP_AC5_ADDR        0xB2
#define BMP_AC6_ADDR        0xB4
#define BMP_B1_ADDR         0xB6
#define BMP_B2_ADDR         0xB8
#define BMP_MB_ADDR         0xBA
#define BMP_MC_ADDR         0xBC
#define BMP_MD_ADDR         0xBE

#define CONTROL_REG_ADDR    0xF4 // 控制寄存器，在这个寄存器中设置不同的值可以设置不同转换时间,同时不同的值可以确认转换大气压或者温度
#define BMP_COVERT_TEMP     0x2E // 转换温度 4.5MS
#define BMP_COVERT_PRES_0   0x34 // 转换大气压 4.5ms
#define BMP_COVERT_PRES_1   0x74 // 转换大气压 7.5ms
#define BMP_COVERT_PRES_2   0xB4 // 转换大气压 13.5ms
#define BMP_COVERT_PRES_3   0xF4 // 转换大气压 25.5ms
#define BMP_OUT_MSB                 0xF6 // ADC输出高8位
#define BMP_OUT_LSB                 0xF7 // ADC输出低8位
#define BMP_OUT_XLSB                0xF8 // 19位测量时，ADC输出最低3位

#define OSS_TIME_MS 26 // 时间间隔
#define BMP_COVERT_PRES         BMP_COVERT_PRES_3    // 大气压转换用时
#define OSS  3  // 大气压转换时间
#define PRESSURE_OF_SEA         101325.0f   // 参考海平面压强

bool BMP180_Multiple_Read(uint16_t REG_Address , uint16_t *ReadData);
bool BMP180_ReadTemp(uint16_t * temp);
bool BMP180_ReadPressure(uint32_t * pressure,uint16_t *temp);

void BMP180_Init(void);
void BMP180Convert(double *temperature, long *pressure);
void BMP180_Altitude(float *temperature, long *pressure, float *altitide);


//double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;

#endif
