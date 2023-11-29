#include "bmp180.h"
#include "../../driver/i2c/i2c.h"
#include "../../driver/i2c/sl_i2c.h"
#include "uart_term.h"

// https://github.com/sparkfun/BMP180_Breakout     BMP180_Breakout/Libraries/Teensy/src/Teensy_BMP180.cpp
uint32_t systemClock;
uint32_t tickPeriod;

static int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;

void BMP180_I2C_Init() {

    systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                     120000000);
    tickPeriod = SysTickPeriodGet();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    /* Enable clocks to GPIO Port G and configure pins as I2C */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))) {
    }

    GPIOPinConfigure(GPIO_PG0_I2C1SCL);
    GPIOPinConfigure(GPIO_PG1_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);

    /* Since there are no board pull up's we shall enable the weak internal
     * pull up */
    GPIOG->PUR |= (GPIO_PIN_1 | GPIO_PIN_0);

    /* Enable the clock to I2C-1 module and configure the I2C Master */
    //    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))) {
    }

    /* Configure the I2C Master in standard mode and enable interrupt for Data
     * completion, NAK and Stop condition on the bus */
    I2CMasterInitExpClk(I2C1_BASE, systemClock, true);

    // Clear I2C FIFOs
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;

    /* Initialize the state of the I2C Master */
    //    setI2CState = I2C_MASTER_IDLE;

    /* Enable the interrupt generation from I2C-2 */
    //    IntEnable(INT_I2C2);
    while (I2CMasterBusy(I2C1_BASE))
        ;
}

bool BMP180_Multiple_Read(uint16_t reg, uint16_t *ReadData) {
    uint8_t REG_Data[2];
    uint8_t status;
#ifdef USE_I2C_DRV
    status = i2c_rw_cmd(Board_I2C_BMP180, BMP180_SlaveAddress, &reg, 1, REG_Data, 2);
#else
    I2C_ReadData(I2C2_BASE, BMP180_SlaveAddress, reg, REG_Data, 2);
#endif
    *ReadData = (uint16_t)REG_Data[0] << 8 | (uint16_t)REG_Data[1];
    return status;
}

bool BMP180_ReadTemp(uint16_t *temp) {
    uint8_t status;
    uint8_t writeBuffer[2];
    uint16_t ut = 0;
    writeBuffer[0] = CONTROL_REG_ADDR;
    writeBuffer[1] = BMP_COVERT_TEMP;
#ifdef USE_I2C_DRV
    i2c_write_bytes(Board_I2C_BMP180, BMP180_SlaveAddress, writeBuffer, 2);
#else
    uint8_t data = BMP_COVERT_TEMP;
    I2C_WriteData(I2C2_BASE, BMP180_SlaveAddress, CONTROL_REG_ADDR, &data, 1);
#endif
    usleep(4500);
    status = BMP180_Multiple_Read(BMP_OUT_MSB, &ut);
    *temp = ut;
    return status;
}

bool BMP180_ReadPressure(uint32_t *pressure, uint16_t *temp) {
    uint8_t status;
    uint8_t data[3] = {0};
    uint8_t reg = BMP_OUT_MSB;

    uint8_t writeBuffer[2];
    writeBuffer[0] = CONTROL_REG_ADDR;
    writeBuffer[1] = 0x34 + (OSS << 6);

#ifdef USE_I2C_DRV
    i2c_write_bytes(Board_I2C_BMP180, BMP180_SlaveAddress, &writeBuffer[0], 2);
#else
    uint8_t wd = 0x34 + (OSS << 6);
    I2C_WriteData(I2C2_BASE, BMP180_SlaveAddress, CONTROL_REG_ADDR, &wd, 1);
#endif
    vTaskDelay(OSS_TIME_MS);

#ifdef USE_I2C_DRV
    status = i2c_rw_cmd(Board_I2C_BMP180, BMP180_SlaveAddress, &reg, 1, &data[0], 3);
#else
    I2C_ReadData(I2C2_BASE, BMP180_SlaveAddress, reg, &data[0], 3);
#endif
    *pressure = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2]) >> (8 - OSS);
    return status;
}

void BMP180_Init(void) {

    BMP180_Multiple_Read(BMP_AC1_ADDR, (uint16_t *)&AC1);
    BMP180_Multiple_Read(BMP_AC2_ADDR, (uint16_t *)&AC2);
    BMP180_Multiple_Read(BMP_AC3_ADDR, (uint16_t *)&AC3);
    BMP180_Multiple_Read(BMP_AC4_ADDR, (uint16_t *)&AC4);
    BMP180_Multiple_Read(BMP_AC5_ADDR, (uint16_t *)&AC5);
    BMP180_Multiple_Read(BMP_AC6_ADDR, (uint16_t *)&AC6);
    BMP180_Multiple_Read(BMP_B1_ADDR, (uint16_t *)&B1);
    BMP180_Multiple_Read(BMP_B2_ADDR, (uint16_t *)&B2);
    BMP180_Multiple_Read(BMP_MB_ADDR, (uint16_t *)&MB);
    BMP180_Multiple_Read(BMP_MC_ADDR, (uint16_t *)&MC);
    BMP180_Multiple_Read(BMP_MD_ADDR, (uint16_t *)&MD);
}

void BMP180Convert(double *temperature, long *pressure) {
    uint16_t ut;
    uint32_t up;

    bool temp_state, pressure_state;
    long x1, x2, b5, b6, x3, b3;
    long p;
    unsigned long b4, b7;

    temp_state = BMP180_ReadTemp(&ut);
    pressure_state = BMP180_ReadPressure(&up, &ut);

    if (ut == 0)
        return;

    x1 = ((long)ut - AC6) * AC5 >> 15;
    x2 = ((long)MC << 11) / (x1 + MD);
    b5 = x1 + x2;
    *temperature = ((b5 + 8) >> 4) * 0.1;

    b6 = b5 - 4000;
    x1 = (B2 * (b6 * b6 >> 12)) >> 11;
    x2 = (AC2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((long)AC1 * 4 + x3) << OSS) + 2) >> 2;
    x1 = (AC3 * b6) >> 13;
    x2 = (B1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (AC4 * (unsigned long)(x3 + 32768)) >> 15;
    b7 = ((unsigned long)up - b3) * (50000 >> OSS);
    if (b7 < 0x80000000)
        p = (b7 * 2) / b4;
    else
        p = (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    *pressure = p + ((x1 + x2 + 3791) >> 4);
}

void BMP180_Altitude(float *temperature, long *pressure, float *altitide) {
    BMP180Convert(temperature, pressure);
    *altitide = 44330 * (1 - pow((*pressure) / PRESSURE_OF_SEA, 1.0f / 5.255f));
}
