#include "i2c.h"

#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_i2c.h>
//#include "../../uart_term.h"

// refer to https://github.com/mahengunawardena/Tiva_I2C_Nokia_ADXL345
// https://processors.wiki.ti.com/index.php/I2C_Tips 关于后面提到的多线程访问I2C总线问题非常有用。

uint8_t setI2CState;
uint8_t dataIndex;

static uint8_t SlaveAddr = 0;
uint32_t systemClock;
uint32_t tickPeriod;

void DelayUs(uint32_t us)
{
    float f = 1000000 / (float)us;
    f = (float)systemClock / (3.0 * f);
    // SysCtlDelay((uint32_t)f);
    // 实时系统用 vTaskDelay,无系统使用 SysCtlDelay;
    vTaskDelay(us);
}

void I2C2_Init()
{

    /* 这里的初始化与SDK里定义I2C_Config再使用I2C_init() 有什么区别？ */
    systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                     120000000);
    printf("The main clock is : %d\r\n", systemClock);
    tickPeriod = SysTickPeriodGet();
    /* Enable clocks to GPIO Port N and configure pins as Output */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)))
    {
    }

    GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    GPIOPinConfigure(GPIO_PN4_I2C2SDA);
    GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
    GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);

    /* Since there are no board pull up's we shall enable the weak internal
     * pull up */
    GPION->PUR |= (GPIO_PIN_4 | GPIO_PIN_5);
    // 使能回送模式，用于调试。 The controller in a test mode loopback configuration.
    //    HWREG(I2C2_BASE + I2C_O_MCR) |= 0x1;

    /* Enable the clock to I2C-1 module and configure the I2C Master */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2)))
    {
    }

    /* Configure the I2C Master in standard mode and enable interrupt for Data
     * completion, NAK and Stop condition on the bus */
    I2CMasterInitExpClk(I2C2_BASE, systemClock, true);

    // 初始化为Master模式,设置中断源。
    //    I2CMasterIntEnableEx(I2C2_BASE, I2C_MASTER_INT_NACK |
    //                                    I2C_MASTER_INT_STOP |
    //                                    I2C_MASTER_INT_DATA);
    //Clear I2C FIFOs
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;

    /* Initialize the state of the I2C Master */
    setI2CState = I2C_MASTER_IDLE;

    /* Enable the interrupt generation from I2C-2 */
    //    IntEnable(INT_I2C2);
    while (I2CMasterBusy(I2C2_BASE))
        ;
    printf("after I2C2_Init\r\n");
    // I2C_init();
}

void I2C1_Init()
{
    uint32_t systemClock;
    /* 这里的初始化与SDK里定义I2C_Config再使用I2C_init() 有什么区别？ */
    systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                     120000000);
    tickPeriod = SysTickPeriodGet();
    /* Enable clocks to GPIO Port N and configure pins as Output */

    /* Enable clocks to GPIO Port G and configure pins as I2C */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)))
    {
    }

    GPIOPinConfigure(GPIO_PG0_I2C1SCL);
    GPIOPinConfigure(GPIO_PG1_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);

    /* Since there are no board pull up's we shall enable the weak internal
     * pull up */
    GPIOG->PUR |= (GPIO_PIN_1 | GPIO_PIN_0);

    SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    /* Enable the clock to I2C-1 module and configure the I2C Master */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1)))
    {
    }

    /* Configure the I2C Master in standard mode and enable interrupt for Data
     * completion, NAK and Stop condition on the bus */
    I2CMasterInitExpClk(I2C1_BASE, systemClock, false);

    // 初始化为Master模式,设置中断源。这里不使用要中断处理。
    //    I2CMasterIntEnableEx(I2C1_BASE, I2C_MASTER_INT_NACK |
    //                                        I2C_MASTER_INT_STOP |
    //                                        I2C_MASTER_INT_DATA);

    /* Initialize the state of the I2C Master */
    setI2CState = I2C_MASTER_IDLE;

    /* Enable the interrupt generation from I2C-2 */
    //    IntEnable(INT_I2C1);
    printf("after I2C1_Init\r\n");
}

//uint32_t I2C_ReadByte(uint8_t slave_address)
//{
//    /* Put the Slave Address on the bus for Read */
//    I2CMasterSlaveAddrSet(I2CX_BASE, slave_address, true); // 这里等同于 hex(SlaveAddr << 1 | 1) = 0xd1 .
//    while (I2CMasterBusy(I2CX_BASE));
//    I2CMasterControl(I2CX_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
//    while (I2CMasterBusy(I2CX_BASE));
//    I2CMasterErr(I2CX_BASE);
//    return I2CMasterDataGet(I2CX_BASE);
//}

//uint32_t I2C_WriteByte(uint8_t slave_address, uint8_t byte)
//{
//    // 发送单字节命令时把中断关闭
//    //    I2CMasterIntDisable(I2CX_BASE);
//    /* Put the Slave Address on the bus for Write */
//    /*  I2C的从机地址是7bit，加上RW位为0(写), 比如：从机地址是0x68,写置位，用逻辑分析仪抓包会显示： Setup Write to [0xD0]+ACK */
//    I2CMasterSlaveAddrSet(I2CX_BASE, slave_address, false);
//    /* Write the first data to the bus */
//    I2CMasterDataPut(I2CX_BASE, byte);
//    I2CMasterControl(I2CX_BASE, I2C_MASTER_CMD_SINGLE_SEND);
//
//    //     如果I2C总线上有多个主机是，必须先调用I2CMasterBusBusy.
//    while (I2CMasterBusy(I2CX_BASE))
//        ;
//    I2CMasterErr(I2CX_BASE);
//}

void I2C_WriteData(uint32_t base,uint8_t slave_address, uint8_t reg, uint8_t *data, uint8_t len)
{

    // 写入字串时使用中断。
    //    I2CMasterIntEnable(I2CX_BASE);
    //    /* 为何此处开启中断时，会卡在这里？ */
    //    I2CMasterIntEnableEx(I2CX_BASE, I2C_MASTER_INT_NACK |
    //                                    I2C_MASTER_INT_STOP |
    //                                    I2C_MASTER_INT_DATA);
    //    // 使用中断发送数据。
    //    setI2CState = I2C_MASTER_TX;

    if(len == 1){
        I2CMasterSlaveAddrSet(base, slave_address, false);
           /* Write the first data to the bus */
        I2CMasterDataPut(base, reg);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);
        //     如果I2C总线上有多个主机是，必须先调用I2CMasterBusBusy.
//        while (I2CMasterBusy(base));
//        I2CMasterSlaveAddrSet(base, slave_address, false);
        I2CMasterDataPut(base, *data);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(I2CMasterBusy(base));
        return ;
    }

    /* Put the Slave Address on the bus for Write */
    I2CMasterSlaveAddrSet(base, slave_address, false);
    // 写入寄存器地址。
    I2CMasterDataPut(base, reg);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(base));
    volatile uint8_t index;

    for (index = 0; index < len - 1; index++)
    {
        I2CMasterDataPut(base, data[index]);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_CONT);
        while (I2CMasterBusy(base))
            ;
        I2CMasterErr(base);
    }
    I2CMasterDataPut(base, data[len - 1]);
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(base));
}

void I2C_ReadData(uint32_t base,uint8_t slave_address, uint8_t reg, uint8_t *data, uint8_t len)
{
    //    setI2CState = I2C_MASTER_RX;
    volatile uint8_t index = 0;
    I2CMasterSlaveAddrSet(base, slave_address, false);
    // 写入寄存器地址。
    I2CMasterDataPut(base, reg);
    I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(base));

    /* Put the Slave Address on the bus for Read */
    I2CMasterSlaveAddrSet(base, slave_address, true);
    /* Start the Read transaction */
    if(len == 1){
       while (I2CMasterBusy(base));
       I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);
       while (I2CMasterBusy(base));
       I2CMasterErr(base);
       *data = I2CMasterDataGet(base);
       return;
    }

    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(base));
    data[index] = I2CMasterDataGet(base);
    I2CMasterErr(base);

    for (index = 1; index < len - 2; index++)
    {
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while (I2CMasterBusy(base)){}
        data[index] = I2CMasterDataGet(base);
    }
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(base)){}
    data[index] = I2CMasterDataGet(base);
    I2CMasterErr(base);

}

// I2C2的中断函数。
//void I2C2_IRQHandler(void)
//{
//    uint32_t getIntStatus;
//    uint32_t getERRStatus;
//
//    /* Get the interrupt status and clear the same */
//    getIntStatus = I2CMasterIntStatusEx(I2C2_BASE, true);
//    // 先清理中断。
//    I2CMasterIntClearEx(I2C2_BASE, getIntStatus);
//    printf("I2C2_IRQHandler getIntStatus %x \r\n", getIntStatus);
//
//    /* Check if we have a Data Request */
//    if ((getIntStatus & I2C_MASTER_INT_DATA) == I2C_MASTER_INT_DATA)
//    {
//        /* Process data interrupt  for the Transmit Path */
//        if ((setI2CState == I2C_MASTER_TX) && (dataIndex < bufLen - 1))
//        {
//            I2CMasterDataPut(I2C2_BASE, sendMasterTxData[dataIndex++]);
//            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
//        }
//        else if ((setI2CState == I2C_MASTER_TX) && (dataIndex == bufLen - 1))
//        {
//            // 数据已经发送完成。
//            I2CMasterDataPut(I2C2_BASE, sendMasterTxData[dataIndex++]);
//            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
//            setI2CState = I2C_MASTER_IDLE;
//        }
//
//        /* Process data interrupt  for the Receive Path */
//        if ((setI2CState == I2C_MASTER_RX) && (dataIndex < bufLen - 2))
//        {
//            getMasterRxData[dataIndex++] = I2CMasterDataGet(I2C2_BASE);
//            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//        }
//        else if ((setI2CState == I2C_MASTER_RX) && (dataIndex == bufLen - 2))
//        {
//            getMasterRxData[dataIndex++] = I2CMasterDataGet(I2C2_BASE);
//            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
//            // 数据已经接收完成。
//            setI2CState = I2C_MASTER_IDLE;
//            bufLen = dataIndex;
//        }
//    }
//
//    /* Check if we have a Stop condition on the bus */
//    if ((getIntStatus & I2C_MASTER_INT_STOP) == I2C_MASTER_INT_STOP)
//    {
//        if (setI2CState == I2C_MASTER_TX)
//        {
//            setI2CState = I2C_MASTER_IDLE;
//        }
//        else if (setI2CState == I2C_MASTER_RX)
//        {
//            getMasterRxData[dataIndex] = I2CMasterDataGet(I2CX_BASE);
//            setI2CState = I2C_MASTER_IDLE;
//        }
//    }
//}
