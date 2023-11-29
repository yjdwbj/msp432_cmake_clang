


#include "dht11.h"
/* for usage usleep() */
#include <unistd.h>

#include <ti/drivers/GPIO.h>
#include "ti/devices/msp432e4/inc/msp432e401y.h"
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/devices/msp432e4/driverlib/gpio.h>

static uint32_t  tickPeriod;
volatile uint8_t checksum = 0;

/*
 *  DHT11的数据格式  <湿度数据H8>:<湿度数据L8>:<温度数据H8>:<温度数据L8>:<校验和-8bit> 。
 *  用户MCU发送一次开始信号后,DHT11从低功耗模式转换到高速模式,等待主机开始信号结束后,DHT11发送响应信号,送出40bit的数据,并触发一次信号采集,用户可选择读取部分数据.
 *  采样步骤：总线空闲状态为高电平,主机把总线拉低等待DHT11响应,主机把总线拉低必须大于18毫秒,保证DHT11能检测到起始信号。DHT11接收到主机的开始信号后,等待主机开始信号结束,
 *  然后发送80us低电平响应信号，再把总线拉高80us,准备发送数据,每一bit数据都以50us低电平时隙开始,这里的典型值是26~28us高电平为数字0，70us高电平是数字1，间隙是50us。
 *  釆样周不能低于1秒钟。
 *
 * */
void DHT11_Reset() {
    tickPeriod = SysTickPeriodGet();


       // 设置PE0 为输出模式
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

    /* 拉低PE0的电平 */
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0);
    /* 微秒级延时可以使用C标准库里的 usleep */
     usleep(20000);
    /* vTaskDelay 是FreeRTOS里的延时函数, 拉低控制器总线PE0 至少18ms */
//    vTaskDelay(20);

    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,1); /* 拉高控制器总线PE0 */
    /* SysCtlDelay的延时是根据系统时钟主频去计算，如下：
     * systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                                  SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                                  120000000);
     * 这里设置时钟频率是120M, 它的时间周期就是 1/120 us，如果要延时1us,SysCtlDelay(40)就可以得到1us,如果是16M的主频，SysCtlDelay(5)是1us。
     * 因为是刚入门学，所以用了一个很笨的方法求得它的延时参数，使用一个逻辑分析仪（8路 24M，某宝在35元左右），抓它一个GPIO的电平，拉高GPIO的电平，SysCtlDelay(40)，拉低GPIO的电平，
     * 打开逻辑分析仪的软件，就能很精确的知道这的延时参数。
     * */
//    SysCtlDelay((tickPeriod / 3000 )*40);  // 拉高控制器总线PE0 保持40us 的延时
    usleep(20);
//    SysCtlDelay(800);
//    usleep(20);

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);   // 设置控制器的总线为输入模式

}

uint8_t DHT11_Read_Byte(void){

    uint8_t i,dat;
    dat = 0;
    for(i = 0 ; i < 8;i++)
    {
        while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)!= GPIO_PIN_0);
        //        https://blog.csdn.net/Attack_on_cc/article/details/86668182
        SysCtlDelay((tickPeriod / 3000 )*35);
        dat <<=1;
        if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) ==  GPIO_PIN_0){
            dat = dat | 0x01;
        }
        while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == GPIO_PIN_0);
    }
    return dat;
}

uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi){
    uint8_t buf[5];
    uint8_t i;


    DHT11_Reset();

    // 如果控制器检测到DHT11的低电平信号,这里在初学时，GPIOPinRead会返回的值，不是如常规的我们认为高电平是1，低电平是0，
    // 而是返回的高电平的PIN值，如： 读取GPIO_PIN_0是高电平是返回是 0x1,低电平是0,读取GPIO_PIN_3是高电平是返回是0x8低电平是0.
    if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)!= GPIO_PIN_0) {
       // DHT11拉低总线时程序在此等待
       while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) != GPIO_PIN_0);
       // DHT11释放总线时程序在此等待
       while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == GPIO_PIN_0);
       for(i = 0 ; i < 5;i++){
            buf[i]=DHT11_Read_Byte();  // 读取DHT11的40bit数据。
       }
       // 如果检验和与上次一样，就不用反馈数据。
       if(checksum == buf[4])
           return 1;
       checksum = buf[4];
       if( (buf[0]+buf[1]+buf[2]+buf[3]) == buf[4]){
           *humi = buf[0];
           *temp = buf[2];
       }
       GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
    }
    else return 1;
    return 0;
}


