#include "bmp180.h"
#include "../../driver/i2c/sl_i2c.h"
#include "../../driver/i2c/i2c.h"
#include "uart_term.h"

// https://github.com/sparkfun/BMP180_Breakout     BMP180_Breakout/Libraries/Teensy/src/Teensy_BMP180.cpp
// https://blog.csdn.net/qq_41422043/article/details/89205990
// 定义存取EEPROM校准值的变量
uint32_t systemClock;
uint32_t tickPeriod;

static int16_t AC1,AC2,AC3,B1,B2,MB,MC,MD;
static uint16_t AC4,AC5,AC6;




void BMP180_I2C_Init()
{

    /* 这里的初始化与SDK里定义I2C_Config再使用I2C_init() 有什么区别？ */
    systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                     120000000);
//    printf("The main clock is : %d\r\n", systemClock);
    tickPeriod = SysTickPeriodGet();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);


    /* Enable clocks to GPIO Port G and configure pins as I2C */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)))
    {
    }

    // 这里初始化的GPIO要与MSP_EXP432E401.c内的定义是一致的。

    GPIOPinConfigure(GPIO_PG0_I2C1SCL);
    GPIOPinConfigure(GPIO_PG1_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);

    /* Since there are no board pull up's we shall enable the weak internal
     * pull up */
    GPIOG->PUR |= (GPIO_PIN_1 | GPIO_PIN_0);

    // 使能回送模式，用于调试。 The controller in a test mode loopback configuration.
    //    HWREG(I2C2_BASE + I2C_O_MCR) |= 0x1;

//    SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
//    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    /* Enable the clock to I2C-1 module and configure the I2C Master */
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)))
    {
    }

    /* Configure the I2C Master in standard mode and enable interrupt for Data
     * completion, NAK and Stop condition on the bus */
    I2CMasterInitExpClk(I2C1_BASE, systemClock, true);

    // 初始化为Master模式,设置中断源。
    //    I2CMasterIntEnableEx(I2C1_BASE, I2C_MASTER_INT_NACK |
    //                                    I2C_MASTER_INT_STOP |
    //                                    I2C_MASTER_INT_DATA);
    //Clear I2C FIFOs
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;

    /* Initialize the state of the I2C Master */
//    setI2CState = I2C_MASTER_IDLE;

    /* Enable the interrupt generation from I2C-2 */
    //    IntEnable(INT_I2C2);
    while (I2CMasterBusy(I2C1_BASE))
        ;
//    printf("after I2C1_BASE init\r\n");
    // I2C_init();
}


// 从气压传感器读取值
bool BMP180_Multiple_Read(uint16_t reg, uint16_t *ReadData)
{
    uint8_t REG_Data[2];
    uint8_t status;
#ifdef USE_I2C_DRV
    status = i2c_rw_cmd(Board_I2C_BMP180, BMP180_SlaveAddress, &reg, 1, REG_Data, 2);
#else
    I2C_ReadData(I2C2_BASE,BMP180_SlaveAddress,reg,REG_Data,2);
#endif
    *ReadData = (uint16_t)REG_Data[0] << 8 | (uint16_t)REG_Data[1];
    return status;
}

// 读取温度值
bool BMP180_ReadTemp(uint16_t *temp)
{
    uint8_t status;
    uint8_t writeBuffer[2];
    uint16_t ut = 0;
//    double x1,x2,b5;
    writeBuffer[0] = CONTROL_REG_ADDR;
    writeBuffer[1] = BMP_COVERT_TEMP;
#ifdef USE_I2C_DRV
    i2c_write_bytes(Board_I2C_BMP180, BMP180_SlaveAddress, writeBuffer, 2);
#else
    uint8_t data = BMP_COVERT_TEMP;
    I2C_WriteData(I2C2_BASE,BMP180_SlaveAddress,CONTROL_REG_ADDR,&data,1);
#endif
    usleep(4500);
//    vTaskDelay(6);
    status = BMP180_Multiple_Read(BMP_OUT_MSB, &ut);
    *temp = ut;
//    x1 = (ut - AC6) * AC5 / pow(2,15);
//    x2 = MC * pow(2,11) / (x1 + MD);
//    b5 = x1 + x2;
//    *temp = (b5 + 8)/pow(2,4);
    return status;
}

// 读取气压值,需要有温度数据。
bool BMP180_ReadPressure(uint32_t *pressure,uint16_t *temp)
{
//    double pu,s,x,y,z;
    uint8_t status;
    uint8_t data[3]={0};
    uint8_t reg =BMP_OUT_MSB;
//    double X1,X2,X3,B3,B4,B5,B6,B7,p;


    uint8_t writeBuffer[2];
    writeBuffer[0] = CONTROL_REG_ADDR;
    writeBuffer[1] = 0x34 + (OSS << 6 );

#ifdef USE_I2C_DRV
    i2c_write_bytes(Board_I2C_BMP180, BMP180_SlaveAddress, &writeBuffer[0], 2);
#else
    uint8_t wd = 0x34 + (OSS << 6 );
    I2C_WriteData(I2C2_BASE,BMP180_SlaveAddress,CONTROL_REG_ADDR,&wd,1);
#endif
    //    usleep(OSS_TIME_MS * 1000);
    vTaskDelay(OSS_TIME_MS);


    // 测量完成  读取传感器测出的值
#ifdef USE_I2C_DRV
    status = i2c_rw_cmd(Board_I2C_BMP180, BMP180_SlaveAddress, &reg, 1, &data[0], 3);
#else
    I2C_ReadData(I2C2_BASE,BMP180_SlaveAddress,reg,&data[0],3);
#endif
    *pressure = ((uint32_t)data[0] << 16 | (uint32_t)data[1] << 8 | (uint32_t)data[2]) >> (8 - OSS);
    // 计算真实的气压
//    B6 = B5-4000;
//    X1 = (B2 * (B6 *B6 / pow(2,16)))/pow(2,11);
//    X2 = AC2 * B6/ pow(2,11);
//    X3 = X1+X2;
//    B3 = ((unsigned long)(AC1 * 4 + X3) << OSS + 2) /4;
//    X1 = AC3 * B6 / pow(2,13);
//    X2 = (B1 *(B6*B6/ pow(2,12)))/pow(2,16);
//    X3 = ((X1+X2) + 2)/pow(2,2);
//    B4 = AC4 * ((unsigned long)(X3 + 32768))/pow(2,15);
//    B7 = ((unsigned long)up - B3) * (50000 >> OSS);
//    if(B7 < 0x80000000){
//        p = (B7 * 2 ) / B4;
//    }else {
//        p = (B7 /B4) * 2;
//    }
//
//    X1 = (p / pow(2,8))* (p/pow(2,8));
//    X1 = (X1 * 3038) / pow(2,16);
//    X2 = (-7357 * p)/ pow(2,16);
//    *pressure = p+(X1+X2 +3791)/pow(2,4);
    return status;
}

// 传感器初始化并读取EEPROM值
void BMP180_Init(void)
{

    double c3,c4,b1;
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
//    UART_PRINT("AC1 %d, AC2 %d, AC3 %d,AC4 %d, AC5 %d, \n\r AC6 %d,B1 %d,B2 %d,MB %d,MC %d, MD %d\n\r",
//               AC1,AC2,AC3,AC4,AC5,AC6,B1,B2,MB,MC,MD);

    // Compute floating-point polynominals:
//    c3 = 160.0 * pow(2,-15) * AC3;
//    c4 = pow(10,-3) * pow(2,-15) * AC4;
//    b1 = pow(160,2) * pow(2,-30) * VB1;
//    c5 = (pow(2,-15) / 160) * AC5;
//    c6 = AC6;
//    mc = (pow(2,11) / pow(160,2)) * MC;
//    md = MD / 160.0;
//    x0 = AC1;
//    x1 = 160.0 * pow(2,-13) * AC2;
//    x2 = pow(160,2) * pow(2,-25) * VB2;
//    y0 = c4 * pow(2,15);
//    y1 = c4 * c3;
//    y2 = c4 * b1;
//    p0 = (3791.0 - 8.0) / 1600.0;
//    p1 = 1.0 - 7357.0 * pow(2,-20);
//    p2 = 3038.0 * 100.0 * pow(2,-36);


//    BMP180_Multiple_Read(BMP_MD_ADDR, (uint16_t *)&md);
//    short AC1, AC2, AC3, B1, B2, MB, MC, MD; //calibration vars
//    unsigned short AC4, AC5, AC6; //same
//    uint8_t reg_addr = BMP_AC1_ADDR;
//    uint8_t rxBufferLong[22]={0};
//    uint8_t status;
//
//
//    status = i2c_rw_cmd(Board_I2C_BMP180, BMP180_SlaveAddress, &reg_addr, 1, &rxBufferLong[0], sizeof(rxBufferLong));
//
//
//    AC1 = rxBufferLong[0]<<8 | rxBufferLong[1];
//    AC2 = rxBufferLong[2]<<8 | rxBufferLong[3];
//    AC3 = rxBufferLong[4]<<8 | rxBufferLong[5];
//    AC4 = rxBufferLong[6]<<8 | rxBufferLong[7];
//    AC5 = rxBufferLong[8]<<8 | rxBufferLong[9];
//    AC6 = rxBufferLong[10]<<8 | rxBufferLong[11];
//    B1 = rxBufferLong[12]<<8 | rxBufferLong[13];
//    B2 = rxBufferLong[14]<<8 | rxBufferLong[15];
//    MB = rxBufferLong[16]<<8 | rxBufferLong[17];
//    MC = rxBufferLong[18]<<8 | rxBufferLong[19];
//    MD = rxBufferLong[20]<<8 | rxBufferLong[21];

}

// 校准温度和气压
void BMP180Convert(double *temperature, long *pressure)
{
    uint16_t ut;
    uint32_t up;

    bool temp_state, pressure_state;
    long x1, x2, b5, b6, x3, b3;
    long p;
    unsigned long b4, b7;

    temp_state = BMP180_ReadTemp(&ut);
    pressure_state = BMP180_ReadPressure(&up,&ut);


    if(ut == 0 )
        return ;


    // 计算温度
    x1 = ((long)ut - AC6) * AC5 >> 15;
    x2 = ((long)MC << 11) / (x1 + MD);
    b5 = x1 + x2;
    *temperature = ((b5 + 8) >> 4) * 0.1;



    // 计算气压
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

// 计算海拔
void BMP180_Altitude(float *temperature, long *pressure, float *altitide)
{
    BMP180Convert(temperature, pressure); // 计算出压强
    *altitide = 44330 * (1 - pow((*pressure) / PRESSURE_OF_SEA, 1.0f / 5.255f));
}
