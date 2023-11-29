#ifndef BMP180_H_
#define BMP180_H_
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>
#define USE_I2C_DRV


// 0xEE='0b11101110'  0x77='0b1110111' 7-bit
#define BMP180_SlaveAddress 0x77 /* 7-bit address Define the slave address of the device in the IIC bus.
                                  * The document here writes 0xEE. I2C should only write 7-bit addresses to be valid.
                                  */

#define BMP_AC1_ADDR        0xAA
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

#define CONTROL_REG_ADDR 0xF4    /* Control register, setting different values in this register can set different conversion times,
                                  * and different values can confirm the conversion of atmospheric pressure or temperature.
                                  */

#define BMP_COVERT_TEMP     0x2E     // conversion temperature 4.5MS
#define BMP_COVERT_PRES_0   0x34   // Convert atmospheric pressure 4.5ms
#define BMP_COVERT_PRES_1   0x74   // Convert atmospheric pressure 7.5ms
#define BMP_COVERT_PRES_2   0xB4   // Convert atmospheric pressure 13.5ms
#define BMP_COVERT_PRES_3   0xF4   // Convert atmospheric pressure 25.5ms
#define BMP_OUT_MSB                 0xF6
#define BMP_OUT_LSB                 0xF7
#define BMP_OUT_XLSB                0xF8

#define OSS_TIME_MS 26
#define BMP_COVERT_PRES         BMP_COVERT_PRES_3
#define OSS  3
#define PRESSURE_OF_SEA         101325.0f

bool BMP180_Multiple_Read(uint16_t REG_Address , uint16_t *ReadData);
bool BMP180_ReadTemp(uint16_t * temp);
bool BMP180_ReadPressure(uint32_t * pressure,uint16_t *temp);

void BMP180_Init(void);
void BMP180Convert(double *temperature, long *pressure);
void BMP180_Altitude(float *temperature, long *pressure, float *altitide);


//double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;

#endif
