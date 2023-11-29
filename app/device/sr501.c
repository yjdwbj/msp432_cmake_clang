#include "sr501.h"

/* 使用中断方式去处理 PE0的电平变化 */
void HCSR501_InterruptHandler2(uint_least8_t index)
{
    static uint8_t status = 'a';

    struct msgQueue queueElement;
    queueElement.msgPtr = &status;
    /* Disable the SW2 interrupt */
    //    GPIO_disableInt(Board_GPIO_BUTTON0); // SW2

    /*
     * 单次检测模式传感器检测到移动，输出高电平后，延迟时间段一结束，输出自动从高电平变成低电平。
     * 连续检测模式
     * 传感器检测到移动，输出高电平后，如果人体继续在检测范围内移动，传感器一直保持高电平，知道人离开后才延迟将高电平变为低电平。
     * 区别两种检测模式的区别，就在检测移动触发后，人体若继续移动，是否持续输出高电平。
     *
     * */

    queueElement.event = PUSLISH_SR501_INT;
    status = 'a';
    printf("HCSR501_InterruptHandler2 \r\n");
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 1);
    /* write message indicating publish message                             */
    if (MQTT_SendMsgToQueue(&queueElement))
    {
        printf("High Queue is full\n\n\r");
    }

    // 如果不清中断，下次中断不能重入
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_1);

    //    if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == GPIO_PIN_1){
    //        printf("GPIOPinRead  is GPIO_PIN_1\r\n  ");
    //    }
    printf("restart interrupt !!!\r\n");
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1);
    //    while(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == GPIO_PIN_1);//等待电平下降再发一次消信。
    //    status = 'b';
    //    if(MQTT_SendMsgToQueue(&queueElement))
    //    {
    //        printf("Low Queue is full\n\n\r");
    //    }
}

const char *pcTextForTask1 = "Task 1 is running\r\n";

/* 使用一个任务去 检测PE0的电平变化。 */
static void vTaskSR501Function(void *pvParameters)
{
    vTaskDelay(5000);
    struct msgQueue queueElement;
    // 开启GPIOE使能
    //        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    //        while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)))
    //        {
    //
    //        }
    // 设置PE1 为输入模式
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    printf("start vTaskSR501Function !\r\n");
    queueElement.event = PUSLISH_SR501_INT;
    //        vTaskDelay(5000);
    // 检查现在电平。
    if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == GPIO_PIN_1)
    {
        *(char *)queueElement.msgPtr = 31;
    }
    else
    {
        *(char *)queueElement.msgPtr = 32;
    }
    printf("GPIOPinRead  !\r\n");
    /* write message indicating publish message                             */
    if (MQTT_SendMsgToQueue(&queueElement))
    {
        printf("High Queue is full\n\n\r");
    }
    printf(" main loop \r\n");
    for (;;)
    {

        while (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == GPIO_PIN_1)
            ;
        printf("read low level!!\r\n");
        *(char *)queueElement.msgPtr = 31;
        if (MQTT_SendMsgToQueue(&queueElement))
        {
            printf("High Queue is full\n\n\r");
        }
        while (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) != GPIO_PIN_1)
            ;
        printf("read HIGH level!!\r\n");
        *(char *)queueElement.msgPtr = 32;
        if (MQTT_SendMsgToQueue(&queueElement))
        {
            printf("High Queue is full\n\n\r");
        }
    }
}
