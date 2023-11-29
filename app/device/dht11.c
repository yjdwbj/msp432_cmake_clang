


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
 *  Single-bus data format is used for communication and synchronization between MCU and
 * DHT11 sensor. One communication process is about 4ms.
 * Data consists of decimal and integral parts. A complete data transmission is 40bit, and the
 * sensor sends higher data bit first.
 * Data format: 8bit integral RH data + 8bit decimal RH data + 8bit integral T data + 8bit decimal T
 * data + 8bit check sum. If the data transmission is right, the check-sum should be the last 8bit of
 * "8bit integral RH data + 8bit decimal RH data + 8bit integral T data + 8bit decimal T data".
 *
 * */
void DHT11_Reset() {
    tickPeriod = SysTickPeriodGet();


    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);

    /* set gpioe pin0 output low */
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0);
    usleep(20000);
    /* set gpioe pin0 output high */
    GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,1);
    /* The delay of SysCtlDelay is calculated based on the system clock frequency, as follows:
     * systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                                  SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                                  120000000);
     * The clock frequency is set here to 120M, and its time period is 1/120 us.
     * If you want to delay 1us, SysCtlDelay(40) can get 1us. If the main frequency is 16M, SysCtlDelay(5) is 1us.
     * */
    usleep(20);
    /* set gpioe pin0 to input direct */
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);

}

uint8_t DHT11_Read_Byte(void){

    uint8_t i,dat;
    dat = 0;
    for(i = 0 ; i < 8;i++)
    {
        while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)!= GPIO_PIN_0);
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

    if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0)!= GPIO_PIN_0) {
       while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) != GPIO_PIN_0);
       while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == GPIO_PIN_0);
       for(i = 0 ; i < 5;i++){
            buf[i]=DHT11_Read_Byte();
       }
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


