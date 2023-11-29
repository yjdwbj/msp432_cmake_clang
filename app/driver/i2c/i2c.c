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
    vTaskDelay(us);
}

void I2C2_Init()
{

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
    // The controller in a test mode loopback configuration.
    //    HWREG(I2C2_BASE + I2C_O_MCR) |= 0x1;

    /* Enable the clock to I2C-1 module and configure the I2C Master */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2)))
    {
    }

    /* Configure the I2C Master in standard mode and enable interrupt for Data
     * completion, NAK and Stop condition on the bus */
    I2CMasterInitExpClk(I2C2_BASE, systemClock, true);

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

    //    I2CMasterIntEnableEx(I2C1_BASE, I2C_MASTER_INT_NACK |
    //                                        I2C_MASTER_INT_STOP |
    //                                        I2C_MASTER_INT_DATA);

    /* Initialize the state of the I2C Master */
    setI2CState = I2C_MASTER_IDLE;

    /* Enable the interrupt generation from I2C-2 */
    //    IntEnable(INT_I2C1);
    printf("after I2C1_Init\r\n");
}

void I2C_WriteData(uint32_t base,uint8_t slave_address, uint8_t reg, uint8_t *data, uint8_t len)
{

    if(len == 1){
        I2CMasterSlaveAddrSet(base, slave_address, false);
           /* Write the first data to the bus */
        I2CMasterDataPut(base, reg);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);
//        while (I2CMasterBusy(base));
//        I2CMasterSlaveAddrSet(base, slave_address, false);
        I2CMasterDataPut(base, *data);
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
        while(I2CMasterBusy(base));
        return ;
    }

    /* Put the Slave Address on the bus for Write */
    I2CMasterSlaveAddrSet(base, slave_address, false);
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