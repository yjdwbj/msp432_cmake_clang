/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*****************************************************************************

   Application Name     -   MQTT Client
   Application Overview -   The device is running a MQTT client which is
                           connected to the online broker. Three LEDs on the
                           device can be controlled from a web client by
                           publishing msg on appropriate topics. Similarly,
                           message can be published on pre-configured topics
                           by pressing the switch buttons on the device.

   Application Details  - Refer to 'MQTT Client' README.html

*****************************************************************************/
//*****************************************************************************
//
//! \addtogroup mqtt_server
//! @{
//
//*****************************************************************************
/* Standard includes                                                         */
#include <stdlib.h>
#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include <unistd.h>


/* TI-Driver includes                                                        */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>

/* Simplelink includes                                                       */
#include <ti/drivers/net/wifi/simplelink.h>

/* SlNetSock includes
 * 注意这里是使用simplelink-sdk-wifi-plugin 里的文件。                                                */
#include <ti/drivers/net/wifi/slnetifwifi.h>
//#include <ti/net/slnetif.h>
//#include <ti/net/slnetsock.h>
//#include <ti/net/slnetutils.h>

// ndk socket
#include <ti/ndk/inc/socketndk.h>
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/stkmain.h>

/* MQTT Library includes                                                     */
#include <ti/net/mqtt/mqttclient.h>

/* Common interface includes                                                 */
#include "network_if.h"
#include "uart_term.h"

/* Application includes                                                      */
#include "Board.h"
#include "client_cbs.h"
//#include <ti/drivers/gpio/GPIOMSP432E4.h>

// 使用Sysconfig 配置生产的硬件定义文件。
//#include "ti_drivers_config.h"

#include "ti/devices/msp432e4/inc/msp432e401y.h"

// 使用下面函数，把数据发送到BLE的特征服务里去。
#define DATA_HMC5883L_CHAR2 0x02
#define DATA_DHT11_CHAR3 0x03
#define DATA_BMP180_CHAR4 0x04

#define PROFILE_VALUE 0x00
#define PROFILE_ID_CREATE(C, H) ((C << 2) | H)
#define DS_HMC5883L_ID PROFILE_ID_CREATE(DATA_HMC5883L_CHAR2, PROFILE_VALUE)
#define DS_DHT11_ID PROFILE_ID_CREATE(DATA_DHT11_CHAR3, PROFILE_VALUE)
#define DS_BMP180_ID PROFILE_ID_CREATE(DATA_BMP180_CHAR4, PROFILE_VALUE)

uint8_t DataService_setParameter(uint8_t param, uint16_t len, void *value);

// 这里需要重写SDK内的函数指针，让它支持IPv6
extern MQTT_DeviceNetServices_t MQTTClient_net;

#define IPv6_DEVICE_INDEX 1

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
/* enables secured client                                                    */
//#define SECURE_CLIENT
// 参考示例 https://e2e.ti.com/support/wireless-connectivity/wifi/f/968/t/862067?tisearch=e2e-sitesearch&keymatch=mbedtls%252525252520ssl%252525252520client

/* enables client authentication by the server                               */
#define CLNT_USR_PWD

#define CLIENT_INIT_STATE (0x01)
#define MQTT_INIT_STATE (0x04)

#define APPLICATION_VERSION "1.1.1"
#define APPLICATION_NAME "MQTT client"

#define SLNET_IF_WIFI_PRIO (5)

/* Operate Lib in MQTT 3.1 mode.                                             */
#define MQTT_3_1_1 false
#define MQTT_3_1 true

#undef SECURE_CLIENT

#define WILL_TOPIC "Client"
#define WILL_MSG "Client Stopped"
#define WILL_QOS MQTT_QOS_0
#define WILL_RETAIN true

/* Defining Broker IP address and port Number                                */
//#define SERVER_ADDRESS           "messagesight.demos.ibm.com"
//#define SERVER_ADDRESS           "m2m.eclipse.org"
//#define SERVER_ADDRESS           "test.mosquitto.org"
//#define  SERVER_ADDRESS          "broker.hivemq.com"
//#define SERVER_ADDRESS             "173.255.223.213"   // USA  vps
//#define SERVER_ADDRESS             "192.168.1.10"
#define SERVER_ADDRESS              "iot.lcycc.click"
//#define SERVER_ADDRESS           "2409:8a55:2412:5bc0:ea4e:6ff:fe5b:fa98"
//#define SERVER_ADDRESS             "fe80::ea4e:6ff:fe5b:fa98"
//#define SERVER_IP_ADDRESS        "192.168.178.67"
#define SERVER_IP_ADDRESS           "23.94.201.33"
#define PORT_NUMBER 1883
#define SECURED_PORT_NUMBER 8883
#define LOOPBACK_PORT 1882

/* Clean session flag                                                        */
#define CLEAN_SESSION true

/* Retain Flag. Used in publish message.                                     */
#define RETAIN_ENABLE 0

/* Defining Number of subscription topics                                    */
#define SUBSCRIPTION_TOPIC_COUNT 4

/* Defining Subscription Topic Values                                        */
#define SUBSCRIPTION_TOPIC0 "/lcy/To/cc3120"
#define SUBSCRIPTION_TOPIC1 "/lcy/cc3120/LED1"
#define SUBSCRIPTION_TOPIC2 "/lcy/cc3120/LED2"
#define SUBSCRIPTION_TOPIC3 "/lcy/cc3120/report" // 的接收到report时，就通sys发送自已的ip与LED的状态。

/* Defining Publish Topic Values                                             */
#define PUBLISH_TOPIC0 "/lcy/cc3120/sw2"
#define PUBLISH_DHT11 "/lcy/cc3120/dht11"
#define PUBLISH_HMC5883L "/lcy/cc3120/hmc5883l"
#define PUBLISH_BMP180 "/lcy/cc3120/bmp180"
#define PUBLISH_SR501 "/lcy/cc3120/sr501"
#define PUBLISH_SYSINFO "/lcy/cc3120/sys"
#define PUBLISH_BLE_STR "/lcy/cc3120/ble_str" // 接收从APP的蓝牙接到的字符。
#define PUBLISH_BLE_RGB "/lcy/cc3120/ble_rgb" // 接收从APP的蓝牙接到的RGB值。

/* Spawn task priority and Task and Thread Stack Size                        */
#define TASKSTACKSIZE 2048
#define RXTASKSIZE 4096
#define MQTTTHREADSIZE 2048
#define SUBTHREADSIZE 2048
#define SMALLTHREADSIZE 1024
#define SPAWN_TASK_PRIORITY 9

/* secured client requires time configuration, in order to verify server     */
/* certificate validity (date).                                              */

/* Day of month (DD format) range 1-31                                       */
#define DAY 1
/* Month (MM format) in the range of 1-12                                    */
#define MONTH 5
/* Year (YYYY format)                                                        */
#define YEAR 2017
/* Hours in the range of 0-23                                                */
#define HOUR 12
/* Minutes in the range of 0-59                                              */
#define MINUTES 33
/* Seconds in the range of 0-59                                              */
#define SEC 21

/* Number of files used for secure connection                                */
#define CLIENT_NUM_SECURE_FILES 1

/* Expiration value for the timer that is being used to toggle the Led.      */
#define TIMER_EXPIRATION_VALUE 100 * 1000000

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
void pushButtonInterruptHandler2(uint_least8_t index);
//void HCSR501_InterruptHandler2(uint_least8_t index);
void pushButtonInterruptHandler3(uint_least8_t index);
void TimerPeriodicIntHandler(sigval val);
void LedTimerConfigNStart();
void LedTimerDeinitStop();
static void DisplayBanner(char *AppName);
void *MqttClient(void *pvParameters);
void Mqtt_ClientStop(uint8_t disconnect);
void Mqtt_ServerStop();
void Mqtt_Stop();
void Mqtt_start();
int32_t Mqtt_IF_Connect();
int32_t MqttServer_start();
int32_t MqttClient_start();
int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement);

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

/* Connection state: (0) - connected, (negative) - disconnected              */
int32_t gApConnectionState = -1;
uint32_t gInitState = 0;
uint32_t memPtrCounterfree = 0;
bool gResetApplication = false;
static MQTTClient_Handle gMqttClient;
MQTTClient_Params MqttClientExmple_params;
unsigned short g_usTimerInts;

/* Receive task handle                                                       */
pthread_t g_rx_task_hndl = (pthread_t)NULL;
uint32_t gUiConnFlag = 0;

/* AP Security Parameters                                                    */
SlWlanSecParams_t SecurityParams = {0};

/* Client ID                                                                 */
/* If ClientId isn't set, the MAC address of the device will be copied into  */
/* the ClientID parameter.                                                   */
char ClientId[13] = {'\0'};

/* Client User Name and Password                                             */
const char *ClientUsername = "cc3200";
const char *ClientPassword = "e401y";

/* Subscription topics and qos values                                        */
char *topic[SUBSCRIPTION_TOPIC_COUNT] =
    {SUBSCRIPTION_TOPIC0, SUBSCRIPTION_TOPIC1,
     SUBSCRIPTION_TOPIC2, SUBSCRIPTION_TOPIC3};

unsigned char qos[SUBSCRIPTION_TOPIC_COUNT] =
    {MQTT_QOS_0, MQTT_QOS_0, MQTT_QOS_0, MQTT_QOS_0};

/* Publishing topics and messages                                            */
const char *publish_topic = {PUBLISH_TOPIC0};
const char *publish_dht11_topic = {PUBLISH_DHT11};
const char *publish_SR501_topic = {PUBLISH_SR501};
const char *publish_hmc5883l_topic = {PUBLISH_HMC5883L};
const char *publish_ble_str_topic = {PUBLISH_BLE_STR};
const char *publish_ble_rgb_topic = {PUBLISH_BLE_RGB};
const char *publish_bmp180_topic = {PUBLISH_BMP180};

/* Message Queue                                                             */
mqd_t g_PBQueue;

pthread_t mqttThread = (pthread_t)NULL;
pthread_t appThread = (pthread_t)NULL;

#define SLNETUTIL_ADDRINFO_MAX_DNS_NODES 10
#define SLNETUTIL_DNSBUFSIZE ((SLNETUTIL_ADDRINFO_MAX_DNS_NODES) * \
                              (sizeof(uint32_t)))
#define LL_PREFIX 0xFE80

timer_t g_timer;

/* Printing new line                                                         */
char lineBreak[] = "\r\n";

// 用于获取本的外网IP。
extern const void *httpsTask(void *arg0);
static SlNetSock_InAddr_t addr;

/* 读取要HMC5583L的线程 */
pthread_t hmcThread = (pthread_t)NULL;

// 检测HMC线程的变量
volatile bool hmcQueueFull = false;
volatile int  hmcCount=0;

//*****************************************************************************
//                 Banner VARIABLES
//*****************************************************************************
#ifdef SECURE_CLIENT

char *Mqtt_Client_secure_files[CLIENT_NUM_SECURE_FILES] = {"ca-cert.pem"};

/*Initialization structure to be used with sl_ExtMqtt_Init API. In order to  */
/*use secured socket method, the flag MQTTCLIENT_NETCONN_SEC, cipher,        */
/*n_files and secure_files must be configured.                               */
/*certificates also must be programmed  ("ca-cert.pem").                     */
/*The first parameter is a bit mask which configures server address type and */
/*security mode.                                                             */
/*Server address type: IPv4, IPv6 and URL must be declared with The          */
/*corresponding flag.                                                        */
/*Security mode: The flag MQTTCLIENT_NETCONN_SEC enables the security (TLS)  */
/*which includes domain name verification and certificate catalog            */
/*verification, those verifications can be disabled by adding to the bit mask*/
/*MQTTCLIENT_NETCONN_SKIP_DOMAIN_NAME_VERIFICATION and                       */
/*MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION flags             */
/*Example: MQTTCLIENT_NETCONN_IP6 | MQTTCLIENT_NETCONN_SEC |                 */
/*MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION                   */
/*For this bit mask, the IPv6 address type will be in use, the security      */
/*feature will be enable and the certificate catalog verification will be    */
/*skipped.                                                                   */
/*Note: The domain name verification requires URL Server address type        */
/*      otherwise, this verification will be disabled.                       */
MQTTClient_ConnParams Mqtt_ClientCtx =
    {
        MQTTCLIENT_NETCONN_IP6,
        SERVER_ADDRESS,
        PORT_NUMBER, //  PORT_NUMBER
        SLNETSOCK_SEC_METHOD_SSLv3_TLSV1_2,
        SLNETSOCK_SEC_CIPHER_FULL_LIST,
        CLIENT_NUM_SECURE_FILES,
        Mqtt_Client_secure_files};

void setTime()
{
    SlDateTime_t dateTime = {0};
    dateTime.tm_day = (uint32_t)DAY;
    dateTime.tm_mon = (uint32_t)MONTH;
    dateTime.tm_year = (uint32_t)YEAR;
    dateTime.tm_hour = (uint32_t)HOUR;
    dateTime.tm_min = (uint32_t)MINUTES;
    dateTime.tm_sec = (uint32_t)SEC;
    sl_DeviceSet(SL_DEVICE_GENERAL, SL_DEVICE_GENERAL_DATE_TIME,
                 sizeof(SlDateTime_t), (uint8_t *)(&dateTime));
}

#else
MQTTClient_ConnParams Mqtt_ClientCtx =
    {
        MQTTCLIENT_NETCONN_URL,
//        MQTTCLIENT_NETCONN_IP6,
        SERVER_ADDRESS,
        PORT_NUMBER, 0, 0, 0,
        NULL};
#endif

/* Initialize the will_param structure to the default will parameters        */
MQTTClient_Will will_param =
    {
        WILL_TOPIC,
        WILL_MSG,
        WILL_QOS,
        WILL_RETAIN};

//*****************************************************************************
//
//! MQTT_SendMsgToQueue - Utility function that receive msgQueue parameter and
//! tries to push it the queue with minimal time for timeout of 0.
//! If the queue isn't full the parameter will be stored and the function
//! will return 0.
//! If the queue is full and the timeout expired (because the timeout parameter
//! is 0 it will expire immediately), the parameter is thrown away and the
//! function will return -1 as an error for full queue.
//!
//! \param[in] struct msgQueue *queueElement
//!
//! \return 0 on success, -1 on error
//
//*****************************************************************************
int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement)
{
    struct timespec abstime = {0};

    clock_gettime(CLOCK_REALTIME, &abstime);

    if (g_PBQueue)
    {
        /* send message to the queue                                        */
        if (mq_timedsend(g_PBQueue, (char *)queueElement,
                         sizeof(struct msgQueue), 0, &abstime) == 0)
        {
            return (0);
        }
    }
    return (-1);
}

//*****************************************************************************
//
//! Push Button Handler1(GPIOSW2). Press push button1 (GPIOSW2) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//! event publish messages
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void pushButtonInterruptHandler2(uint_least8_t index)
{
    struct msgQueue queueElement;

    /* Disable the SW2 interrupt */
    //    GPIO_disableInt(Board_GPIO_BUTTON0); // SW2

    queueElement.event = PUBLISH_PUSH_BUTTON_PRESSED;
    queueElement.msgPtr = NULL;

    /* write message indicating publish message                             */
    if (MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT("\r\nQueue is full\\r\n");
    }
}

//**
// 这里是 HC-SR501为中断源的，中断回调函数。
//void HCSR501_InterruptHandler2(uint_least8_t index)
//{
//    struct msgQueue queueElement;
//
//    /* Disable the SW2 interrupt */
////    GPIO_disableInt(Board_GPIO_BUTTON0); // SW2
//    printf("HCSR501_InterruptHandler2 \r\n");
////    GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
//
////    GPIO_disableInt(Board_GPIO_PE1); // 关中断；
//
//    /*
//     * 单次检测模式传感器检测到移动，输出高电平后，延迟时间段一结束，输出自动从高电平变成低电平。
//     * 连续检测模式
//     * 传感器检测到移动，输出高电平后，如果人体继续在检测范围内移动，传感器一直保持高电平，知道人离开后才延迟将高电平变为低电平。
//     * 区别两种检测模式的区别，就在检测移动触发后，人体若继续移动，是否持续输出高电平。
//     *
//     * */
//
//    queueElement.event = PUSLISH_SR501_INT;
//    *(char *)queueElement.msgPtr = 31;
//
//    /* write message indicating publish message                             */
//    if(MQTT_SendMsgToQueue(&queueElement))
//    {
//       UART_PRINT("Queue is full\\r\n");
//    }
//
//    while(!GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1));//等待电平下降再发一次消信。
//    *(char *)queueElement.msgPtr = 30;
//    if(MQTT_SendMsgToQueue(&queueElement))
//    {
//       UART_PRINT("Queue is full\\r\n");
//    }
//}

//*****************************************************************************
//
//! Push Button Handler2(GPIOSW3). Press push button3 Whenever user wants to
//! disconnect from the remote broker. Write message into message queue
//! indicating disconnect from broker.
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void pushButtonInterruptHandler3(uint_least8_t index)
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

    queueElement.event = DISC_PUSH_BUTTON_PRESSED;
    queueElement.msgPtr = NULL;

    /* write message indicating disconnect push button pressed message      */
    if (MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "Queue is full, throw first msg and send the new one\\r\n");
        mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }
}

//*****************************************************************************
//
//! Periodic Timer Interrupt Handler
//!
//! \param None
//!
//! \return None
//
//*****************************************************************************
void TimerPeriodicIntHandler(sigval val)
{
    /* Increment our interrupt counter.                                      */
    g_usTimerInts++;

    if (!(g_usTimerInts & 0x1))
    {
        /* Turn Led Off                                                      */
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
    }
    else
    {
        /* Turn Led On                                                       */
        GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    }
}

//*****************************************************************************
//
//! Function to configure and start timer to blink the LED while device is
//! trying to connect to an AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerConfigNStart()
{
    struct itimerspec value;
    sigevent sev;

    /* Create Timer                                                          */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_notify_function = &TimerPeriodicIntHandler;
    timer_create(2, &sev, &g_timer);

    /* start timer                                                           */
    value.it_interval.tv_sec = 0;
    value.it_interval.tv_nsec = TIMER_EXPIRATION_VALUE;
    value.it_value.tv_sec = 0;
    value.it_value.tv_nsec = TIMER_EXPIRATION_VALUE;

    timer_settime(g_timer, 0, &value, NULL);
}

//*****************************************************************************
//
//! Disable the LED blinking Timer as Device is connected to AP
//!
//! \param none
//!
//! return none
//
//*****************************************************************************
void LedTimerDeinitStop()
{
    /* Disable the LED blinking Timer as Device is connected to AP.          */
    timer_delete(g_timer);
}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void DisplayBanner(char *AppName)
{
    UART_PRINT("\r\n");
    UART_PRINT("\t\t *************************************************\r\n");
    UART_PRINT("\t\t    CC32xx %s Application       \r\n", AppName);
    UART_PRINT("\t\t *************************************************\r\n");
    UART_PRINT("\r\n");
}

void *MqttClientThread(void *pvParameters)
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

    MQTTClient_run((MQTTClient_Handle)pvParameters);

    queueElement.event = LOCAL_CLIENT_DISCONNECTION;
    queueElement.msgPtr = NULL;

    /*write message indicating disconnect Broker message.                   */
    if (MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "Queue is full, throw first msg and send the new one\\r\n");
        mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }

    pthread_exit(0);

    return (NULL);
}

static void vTaskFunction(void *pvParameters)
{
    uint8_t temp = 0;
    uint8_t humi = 0;
    struct msgQueue queueElemRecv;
    uint8_t btdata[2] = {0, 0};

    uint16_t data = 0;
    uint16_t oldtemp = 0;
    sleep(2);
    UART_PRINT("MQTT Client dht11 initialization Task\r\n");

    struct msgQueue queueElement;
    queueElement.event = PUBLISH_EVT_DHT11_DATA;
    queueElement.msgPtr = NULL;
    queueElement.msgPtr = &data;
    /**
        *  开启GPIOE使能 ,dht11 通信,这里设置DHT11的控制器的总线管脚是PE0,设置为输出模式 ,
        *  并且只能一次 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE)? 因为还有其它Pin脚有其它定义。
        */
    //    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    //    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))){}

    while (gResetApplication == false)
    {

        data = 0;
        /* dht11 通信*/
        temp = 0;
        humi = 0;
        if (!DHT11_Read_Data(&temp, &humi))
        {
            data = temp << 8 | humi;
            if (oldtemp != data)
            {
                btdata[0] = temp;
                btdata[1] = humi;
                DataService_setParameter(DS_DHT11_ID, sizeof(btdata), btdata);
                if (MQTT_SendMsgToQueue(&queueElement))
                {
                    UART_PRINT("DHT11 Queue is full\\r\n");
                    mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                               NULL);
                    usleep(50000); // 50ms
                    MQTT_SendMsgToQueue(&queueElement);
                }
            }
            oldtemp = data;
            UART_PRINT("DHT11 update new data \r\n");
        }
        sleep(3);
    }
    UART_PRINT("dht11 Thread Exit!!!!\r\n");
    pthread_exit(0);
}

/*
 * 这里的POSIX多线程参照的 SimpleLink MCU SDK User's Guide里的示例工程。
 * 位置于 <SDK_INSTALL_DIR>\examples\rtos\<board>\demos\portable directory.
 */

static void postSem(union sigval val)
{
    sem_t *sem = (sem_t *)(val.sival_ptr);
    sem_post(sem);
}

/*
 *  ======== setupTimer ========
 *  Create a timer that will expire at the period specified by the
 *  time arguments. When the timer expires, the passed in semaphore
 *  will be posted by the postSem function.
 *
 *  A non-zero return indicates a failure.
 */
static int setupTimer(sem_t *sem, timer_t *timerid, time_t sec, long nsec)
{
    struct sigevent sev;
    struct itimerspec its;
    int retc;
    //  initially unlocked
    retc = sem_init(sem, 0, 0);
    if (retc != 0)
    {
        return (retc);
    }

    /* Create the timer that wakes up the thread that will pend on the sem. */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_value.sival_ptr = sem;
    sev.sigev_notify_function = &postSem;
    sev.sigev_notify_attributes = NULL;
    retc = timer_create(CLOCK_MONOTONIC, &sev, timerid);
    if (retc != 0)
    {
        return (retc);
    }

    /* Set the timer to go off at the specified period */
    its.it_interval.tv_sec = sec;
    its.it_interval.tv_nsec = nsec;
    its.it_value.tv_sec = sec;
    its.it_value.tv_nsec = nsec;
    retc = timer_settime(*timerid, 0, &its, NULL);
    if (retc != 0)
    {
        timer_delete(*timerid);
        return (retc);
    }

    return (0);
}

void *vTaskBMP180(void *pvParameters)
{
    char data[32] = {0};
    double ut = 0.0;
    long up = 0;
    struct msgQueue queueElemRecv;

    UART_PRINT("MQTT Client vTaskBMP180 initialization Task\r\n");
    struct msgQueue queueElement;
    queueElement.event = PUBLISH_EVT_BMP180;

    while (gResetApplication == false)
    {
        // 测试读取BMP180
        if(hmcCount == 6)
        {
            BMP180_Init();
            BMP180Convert(&ut, &up);
            memset(data, 0, sizeof(data));

            sprintf((char *)&data[0], "%.2f,", ut);
            sprintf((char *)((char *)&data[0] + strlen(data)), "%ld", up);
            queueElement.msgPtr = data;
            queueElement.topLen = strlen(data);
            DataService_setParameter(DS_BMP180_ID, strlen(data), data);
//            UART_PRINT("vTaskBMP180 new DATA %s,len %d\r\n",data,strlen(data));
            if (MQTT_SendMsgToQueue(&queueElement))
            {
                UART_PRINT("vTaskBMP180 Queue is full\r\n");
                mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                           NULL);
            }
            hmcCount=0;
        }
    }
    UART_PRINT("vTaskBMP180 Thread Exit!!!!\r\n");
    pthread_exit(0);
}

void *vTaskHMC5883L(void *pvParameters)
{

    // https://e2e.ti.com/blogs_/b/analogwire/archive/2015/10/15/how-to-simplify-i2c-tree-when-connecting-multiple-slaves-to-an-i2c-master
    // How to simplify I2C tree when connecting multiple slaves to an I2C master
    uint8_t data[6] = {0};
    uint32_t x, y, z, angle;
    struct msgQueue queueElemRecv;

    UART_PRINT("MQTT Client vTaskHMC5883L initialization Task\r\n");

    struct msgQueue queueElement;
    queueElement.event = PUBLISH_EVT_HMC_DATA;
    queueElement.msgPtr = &data[0];
    queueElement.topLen = 6;

    while (false == gResetApplication)
    {
        /* 测试读取 三轴 HMC5883L,因为两个器件读取I2C总线的时间间隔是不一样，所以这里采用信号量来同步，满6次触发一次读BMP。*/

        if(hmcCount != 6)
        {
            read_data(data, 6);
            x = data[0] << 8 | data[1]; //Combine MSB and LSB of X Data output register
            z = data[2] << 8 | data[3]; //Combine MSB and LSB of Z Data output register
            y = data[4] << 8 | data[5]; //Combine MSB and LSB of Y Data output register

            angle = atan2((double)y, (double)x) * (180 / 3.14159265) + 180; // angle in degrees
            DataService_setParameter(DS_HMC5883L_ID, 6, data);
            /* UART_PRINT("Read HMC5883L data %x%x%x \r\n",data[0],data[1],data[2]); */
            if (MQTT_SendMsgToQueue(&queueElement))
            {
                UART_PRINT("HMC5883L Queue is full\r\n");
                mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                           NULL);
            }
            hmcCount++;
        }

        usleep(400 * 1000);
    }
    UART_PRINT("HMC5883L Thread Exit!!!!\r\n");
    pthread_exit(0);
}

void start_sub_threads()
{
    pthread_t thread;
    uint8_t retc = 0;
    pthread_attr_t attrs;
    struct sched_param priParam;

    /*
     * 初始化I2C总线的互斥量。
     * https://processors.wiki.ti.com/index.php/I2C_Tips 关于后面提到的多线程访问I2C总线问题非常有用。
     * 这里是使用要互斥量来获取I2C总线的使用。不知为何value设置0时，sem_wait会一直冻结。
     */
    UART_PRINT("start_sub_threads >>>  \r\n");
//    retc = pthread_mutex_init(&I2CMutex, NULL);
    if (retc != 0)
    {
        UART_PRINT(" pthread_mutex_init is failed\r\n");
        /* pthread_mutex_init() failed */
        while (1)
        {
        }
    }

    pthread_attr_init(&attrs);

    /* Set priority, detach state, and stack size attributes */
    priParam.sched_priority = 2; /* 数字越大，优先级越高。 */
    retc = pthread_attr_setschedparam(&attrs, &priParam);
    retc |= pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    retc |= pthread_attr_setstacksize(&attrs, SUBTHREADSIZE);
    if (retc != 0)
    {
        UART_PRINT(" pthread_attr_setstacksize is failed\r\n");
        /* failed to set attributes */
        while (1)
        {
        }
    }

    /* 读取HMC5883L数据线程 */
    retc = pthread_create(&thread, &attrs, vTaskHMC5883L, NULL);
    if (retc != 0)
    {
        UART_PRINT(" hread_create(&thread, &attrs, vTaskHMC5883L, NULL); is failed\r\n");
        /* pthread_create() failed */
        while (1)
        {
        }
    }


    priParam.sched_priority = 1;
    retc = pthread_attr_setschedparam(&attrs, &priParam);
    if (retc != 0) {
        /* failed to set priority */
        while (1) {}
    }

    /* 读取BMP180数据线程 */

    retc = pthread_create(&thread, &attrs, vTaskBMP180, NULL);
    if (retc != 0)
    {
        UART_PRINT("create(&thread, &attrs, vTaskBMP180, NULL); is failed\r\n");
        /* pthread_create() failed */
        while (1)
        {
        }
    }

    /*
     *  Let's make the temperature thread a higher priority .
     *  Higher number means higher priority in FreeRTOS.
     */

}

//*****************************************************************************
//
//! Task implementing MQTT Server plus client bridge
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt client ans server libraries and set up MQTT
//!       with the remote broker.
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void *MqttClient(void *pvParameters)
{
    struct msgQueue queueElemRecv;
    long lRetVal = -1;
    char *tmpBuff;
    uint32_t retc;
    // 后面5B是时间戳
    char ipAddr[SLNETSOCK_INET_ADDRSTRLEN + 5];
    uint8_t addrLen = 0;
    char btnMsg[8] = {'S', 'W', '2', ':', 0, 0, 0, 0};
    struct timespec ts;

    memset(ipAddr, 0, SLNETSOCK_INET_ADDRSTRLEN + 5);

    /*Initializing Client and Subscribing to the Broker.                     */
    if (gApConnectionState >= 0)
    {
        lRetVal = MqttClient_start();
        if (0 > lRetVal)
        {
            UART_PRINT("MQTT Client lib initialization failed\r\n");
            pthread_exit(0);
            return (NULL);
        }

        start_sub_threads();
    }

    /*handling the signals from various callbacks including the push button  */
    /*prompting the client to publish a msg on PUB_TOPIC OR msg received by  */
    /*the server on enrolled topic(for which the on-board client ha enrolled)*/
    /*from a local client(will be published to the remote broker by the      */
    /*client) OR msg received by the client from the remote broker (need to  */
    /*be sent to the server to see if any local client has subscribed on the */
    /*same topic).                                                           */
    for (;;)
    {

        /*waiting for signals                                                */
        mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                   NULL);

        switch (queueElemRecv.event)
        {
        case PUBLISH_PUSH_BUTTON_PRESSED:
            /* get current time (relative to POSIX Epoch) */
            //            timestamp =time(&ts);
            clock_gettime(CLOCK_REALTIME, &ts);

            btnMsg[4] = (ts.tv_sec >> 24) & 0xff;
            btnMsg[5] = (ts.tv_sec >> 16) & 0xff;
            btnMsg[6] = (ts.tv_sec >> 8) & 0xff;
            btnMsg[7] = ts.tv_sec & 0xff;
            /*send publish message                                       */
            lRetVal =
                MQTTClient_publish(gMqttClient, (char *)publish_topic, strlen((char *)publish_topic), (char *)btnMsg,
                                   8, MQTT_QOS_0);
            time(&ts);
            UART_PRINT("Current date time : %s\r\n", ctime(&ts));
            UART_PRINT("Topic: %s\r\n", publish_topic);
            //            UART_PRINT("timestamp %u \r\n", ts.tv_sec);

            /* Clear and enable again the SW2 interrupt */
            //            GPIO_clearInt(Board_GPIO_BUTTON0);  // SW2
            //            GPIO_enableInt(Board_GPIO_BUTTON0); // SW2

            break;
        case PUBLISH_EVT_SR501_INT:

            /*send publish message                                       */
            lRetVal =
                MQTTClient_publish(gMqttClient, (char *)publish_SR501_topic, strlen((char *)publish_SR501_topic),
                                   (char *)queueElemRecv.msgPtr,
                                   1, MQTT_QOS_2 | ((RETAIN_ENABLE) ? MQTT_PUBLISH_RETAIN : 0));

            UART_PRINT("CC3200 Publishes the SR501 message \r\n");
            UART_PRINT("Topic: %s\r\n", publish_SR501_topic);
            UART_PRINT("Data: %c\r\n", *(char *)queueElemRecv.msgPtr);

            break;
        case PUBLISH_EVT_DHT11_DATA:
            lRetVal =
                MQTTClient_publish(gMqttClient, (char *)publish_dht11_topic, strlen((char *)publish_dht11_topic),
                                   (char *)queueElemRecv.msgPtr,
                                   2, MQTT_QOS_0);
            //            UART_PRINT("Data: %d,%d\r\n", temp, humi);
            break;
        case PUBLISH_EVT_HMC_DATA:
            lRetVal =
                MQTTClient_publish(gMqttClient, (char *)publish_hmc5883l_topic, strlen((char *)publish_hmc5883l_topic),
                                   (char *)queueElemRecv.msgPtr,
                                   queueElemRecv.topLen, MQTT_QOS_0);
            break;
        case PUBLISH_EVT_BMP180:
            lRetVal =
                MQTTClient_publish(gMqttClient, (char *)publish_bmp180_topic, strlen((char *)publish_bmp180_topic),
                                   (char *)queueElemRecv.msgPtr,
                                   queueElemRecv.topLen, MQTT_QOS_0);
            break;
        /*msg received by client from remote broker (on a topic      */
        /*subscribed by local client)                                */
        case MSG_RECV_BY_CLIENT:
            tmpBuff = (char *)((char *)queueElemRecv.msgPtr + 12);
            if (strncmp(tmpBuff, SUBSCRIPTION_TOPIC0, queueElemRecv.topLen) == 0)
            {
                // pushlich to sys
                lRetVal =
                    MQTTClient_publish(gMqttClient, (char *)publish_topic, strlen((char *)publish_topic),
                                       (char *)"Hello",
                                       5, MQTT_QOS_0);

                UART_PRINT("CC3200 Publishes the following message \r\n");
                UART_PRINT("Topic: %s\r\n", publish_topic);
            }
            else if (strncmp(tmpBuff, SUBSCRIPTION_TOPIC1, queueElemRecv.topLen) == 0)
            {
                //                GPIO_toggle(Board_GPIO_LED0);
                // 为什么要偏移29，因为在MqttClientCallback它就是这样封装的。
                GPIO_write(Board_LED0, *(char *)((char *)queueElemRecv.msgPtr + 29) == 0x31 ? Board_LED_ON : Board_LED_OFF);
            }
            else if (strncmp(tmpBuff, SUBSCRIPTION_TOPIC2,
                             queueElemRecv.topLen) == 0)
            {
                //                GPIO_toggle(Board_GPIO_LED1);
                // 为什么要偏移29，因为在MqttClientCallback它就是这样封装的。
                GPIO_write(Board_LED1, *(char *)((char *)queueElemRecv.msgPtr + 29) == 0x31 ? Board_LED_ON : Board_LED_OFF);
            }
            else if (strncmp(tmpBuff, SUBSCRIPTION_TOPIC3,
                             queueElemRecv.topLen) == 0)
            {
                /* 发送IP */
                SlNetUtil_inetNtop(SLNETSOCK_AF_INET, (SlNetSock_InAddr_t *)pvParameters, (char *)ipAddr, SLNETSOCK_INET_ADDRSTRLEN);
                //                UART_PRINT("\n\r self ip %s \r\n", ipAddr);
                clock_gettime(CLOCK_REALTIME, &ts);
                addrLen = strlen((char *)ipAddr);
                ipAddr[addrLen] = ':';
                ipAddr[addrLen + 1] = (ts.tv_sec >> 24) & 0xff;
                ipAddr[addrLen + 2] = (ts.tv_sec >> 16) & 0xff;
                ipAddr[addrLen + 3] = (ts.tv_sec >> 8) & 0xff;
                ipAddr[addrLen + 4] = ts.tv_sec & 0xff;
                lRetVal =
                    MQTTClient_publish(gMqttClient, (char *)PUBLISH_SYSINFO, strlen((char *)PUBLISH_SYSINFO),
                                       (char *)ipAddr, strlen((char *)ipAddr), MQTT_QOS_0);

                //                UART_PRINT("Topic: %s\r\n", PUBLISH_SYSINFO);
            }

            free(queueElemRecv.msgPtr);
            break;

        /* 接收到来自蓝牙端的字符消息。 */
        case PUBLISH_EVT_BLE_STR:
            lRetVal =
                MQTTClient_publish(gMqttClient, (char *)publish_ble_str_topic, strlen((char *)publish_ble_str_topic),
                                   (char *)queueElemRecv.msgPtr,
                                   queueElemRecv.topLen, MQTT_QOS_0);
            //            UART_PRINT("Ble STR data: %s\r\n", queueElemRecv.msgPtr);
            break;
            /* 接收到来自蓝牙端的RGB消息。 */
        case PUBLISH_EVT_BLE_RGB:
            lRetVal =
                MQTTClient_publish(gMqttClient, (char *)publish_ble_rgb_topic, strlen((char *)publish_ble_rgb_topic),
                                   (char *)queueElemRecv.msgPtr,
                                   queueElemRecv.topLen, MQTT_QOS_0);
            break;
        /*On-board client disconnected from remote broker, only      */
        /*local MQTT network will work                               */
        case LOCAL_CLIENT_DISCONNECTION:
            UART_PRINT("On-board Client Disconnected\r\n");
            gUiConnFlag = 0;
            break;

        /*Push button for full restart check                         */
        case DISC_PUSH_BUTTON_PRESSED:
            gResetApplication = true;
            break;

        case THREAD_TERMINATE_REQ:
            gUiConnFlag = 0;
            pthread_exit(0);
            return (NULL);

        default:
            sleep(1);
            break;
        }
    }
    UART_PRINT("MqttClient exit!!!!!!!!\r\n");
    pthread_exit(0);
    return (NULL);
}

//*****************************************************************************
//
//! This function connect the MQTT device to an AP with the SSID which was
//! configured in SSID_NAME definition which can be found in Network_if.h file,
//! if the device can't connect to to this AP a request from the user for other
//! SSID will appear.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
int32_t Mqtt_IF_Connect()
{
    int32_t lRetVal, retc;
    char SSID_Remote_Name[32];
    int8_t Str_Length;

    pthread_t http_thread = (pthread_t)NULL;
    pthread_attr_t pAttrs_http;
    struct sched_param httpParam;

    memset(SSID_Remote_Name, '\0', sizeof(SSID_Remote_Name));
    Str_Length = strlen(SSID_NAME);

    if (Str_Length)
    {
        /*Copy the Default SSID to the local variable                        */
        strncpy(SSID_Remote_Name, SSID_NAME, Str_Length);
    }

    /*Display Application Banner                                             */
    DisplayBanner(APPLICATION_NAME);

    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);

    /*Reset The state of the machine                                         */
    Network_IF_ResetMCUStateMachine();

    /*Start the driver                                                       */
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device\r\n", lRetVal);
        return (-1);
    }

    /*switch on Board_GPIO_LED2 to indicate Simplelink is properly up.       */

    /*Start Timer to blink Board_GPIO_LED0 till AP connection                */
    LedTimerConfigNStart();

    /*Initialize AP security params                                          */
    SecurityParams.Key = (signed char *)SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /*Connect to the Access Point                                            */
    lRetVal = Network_IF_ConnectAP(SSID_Remote_Name, SecurityParams);
    if (lRetVal < 0)
    {
        UART_PRINT("Connection to an AP failed\r\n");
        return (-1);
    }
    //        sleep(10);
    httpsTask(&addr);
    //    UART_PRINT("SlNetUtil_inetPton addr %x\r\n",((SlNetSock_InAddr_t *)&addr)->s_addr);

    /*Disable the LED blinking Timer as Device is connected to AP.           */
    LedTimerDeinitStop();

    /*Switch ON Board_GPIO_LED0 to indicate that Device acquired an IP.      */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    sleep(5);

    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);

    return (0);
}

//*****************************************************************************
//!
//! MQTT Start - Initialize and create all the items required to run the MQTT
//! protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void Mqtt_start()
{
    int32_t threadArg = 100;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int32_t retc = 0;
    mq_attr attr;
    unsigned mode = 0;

    /*sync object for inter thread communication                             */
    attr.mq_maxmsg = 100;
    attr.mq_curmsgs = 0;
    attr.mq_msgsize = sizeof(struct msgQueue);
    g_PBQueue = mq_open("g_PBQueue", O_CREAT, mode, &attr);

    if (g_PBQueue == NULL)
    {
        UART_PRINT("MQTT Message Queue create fail\r\n");
        gInitState &= ~MQTT_INIT_STATE;
        return;
    }

    /*Set priority and stack size attributes                                 */
    pthread_attr_init(&pAttrs);
    // 数字越大，优先级越高。这里的MqttClient线程与蓝牙的线程的优先不能低于其它的，否则可能运行不正常。
    priParam.sched_priority = 3;
    retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, MQTTTHREADSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);

    if (retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
        UART_PRINT("MQTT thread create fail\r\n");
        return;
    }

    retc = pthread_create(&mqttThread, &pAttrs, MqttClient, (void *)&addr);
    if (retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
        UART_PRINT("MQTT thread create fail\r\n");
        return;
    }


    gInitState &= ~MQTT_INIT_STATE;
}

//*****************************************************************************
//!
//! MQTT Stop - Close the client instance and free all the items required to
//! run the MQTT protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_Stop()
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

    if (gApConnectionState >= 0)
    {
        Mqtt_ClientStop(1);
    }

    queueElement.event = THREAD_TERMINATE_REQ;
    queueElement.msgPtr = NULL;

    /*write message indicating publish message                               */
    if (MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "Queue is full, throw first msg and send the new one\r\n");
        mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }

    sleep(2);

    mq_close(g_PBQueue);
    g_PBQueue = NULL;

    sl_Stop(SL_STOP_TIMEOUT);
    UART_PRINT("Client Stop completed\r\n");

    /*Disable the SW2 and SW3 interrupt */
    //    GPIO_disableInt(Board_GPIO_BUTTON0); // SW2
    //    GPIO_disableInt(Board_GPIO_BUTTON1); // SW3
}

//*****************************************************************************
//
//! \brief Open a TCP socket and modify its properties i.e security options if req.
//! Socket properties modified in this function are based on the options set
//! outside the scope of this function.
//! Returns a valid handle on success, otherwise a negative number.
// Mqtt_createSocket 是来自官方MQTT库里的  createSocket 源码，此处重写它是为了支持IPv6.
//*****************************************************************************
static int32_t Mqtt_createSocket(uint32_t nwconnOpts,
                                 MQTT_SecureConn_t *nwSecurityOpts,
                                 const char *serverAddr,
                                 bool isServer)
{
    int32_t socketFd;
    int32_t status;
    uint32_t dummyVal = 1;
    SlNetSockSecAttrib_t *secAttrib;
    SlNetSockSecAttrib_e attribName;
    //local variables for creating secure socket
    uint8_t SecurityMethod;
    uint32_t SecurityCypher;

    int8_t i;

    //If TLS is required, 这里TLS安全连接，需要独立的TLS协议栈支持。
    if ((nwconnOpts & MQTT_DEV_NETCONN_OPT_SEC) != 0)
    {
        /* Create security attribute */
        secAttrib = SlNetSock_secAttribCreate();

        /* Check if the function failed */
        if (NULL == secAttrib)
        {
            return (MQTT_PACKET_ERR_ALLOC_FAILED);
        }

        socketFd =
            SlNetSock_create(SLNETSOCK_AF_INET, SLNETSOCK_SOCK_STREAM,
                             SLNETSOCK_PROTO_TCP,
                             0,
                             0);
        if (socketFd < 0)
        {
            SlNetSock_secAttribDelete(secAttrib);
            return (socketFd);
        }

        SecurityMethod = *((uint8_t *)(nwSecurityOpts->method));
        SecurityCypher = *((uint32_t *)(nwSecurityOpts->cipher));

        /* Domain name verification only works with URL address
           Check if Skip domain name verification is disabled
           and if the address input is url address.             */
        if (((nwconnOpts &
              MQTT_DEV_NETCONN_OPT_SKIP_DOMAIN_NAME_VERIFICATION) == 0) &&
            ((nwconnOpts & MQTT_DEV_NETCONN_OPT_URL) != 0))
        {
            status =
                SlNetSock_secAttribSet(secAttrib,
                                       SLNETSOCK_SEC_ATTRIB_DOMAIN_NAME,
                                       (void *)serverAddr, strlen(serverAddr));
            if (status < 0)
            {
                SlNetSock_secAttribDelete(secAttrib);
                SlNetSock_close(socketFd);
                return (status);
            }
        }

        /* Check if Skip certificate catalog verification is
           enabled.                                             */
        if ((nwconnOpts &
             MQTT_DEV_NETCONN_OPT_SKIP_CERTIFICATE_CATALOG_VERIFICATION) != 0)
        {
            status = SlNetSock_secAttribSet(
                secAttrib, SLNETSOCK_SEC_ATTRIB_DISABLE_CERT_STORE,
                (void *)&dummyVal, sizeof(dummyVal));
            if (status < 0)
            {
                SlNetSock_secAttribDelete(secAttrib);
                SlNetSock_close(socketFd);
                return (status);
            }
        }

        if (nwSecurityOpts->nFile < 1 || nwSecurityOpts->nFile > 4)
        {
            SlNetSock_secAttribDelete(secAttrib);
            SlNetSock_close(socketFd);
            /* security files missing or wrong number of security files
               Did not create socket */
            return (MQTT_PACKET_ERR_FNPARAM);
        }

        //Set Socket Options that were just defined
        status =
            SlNetSock_secAttribSet(secAttrib, SLNETSOCK_SEC_ATTRIB_METHOD,
                                   (void *)&(SecurityMethod),
                                   sizeof(SecurityMethod));
        if (status < 0)
        {
            SlNetSock_secAttribDelete(secAttrib);
            SlNetSock_close(socketFd);
            return (status);
        }

        status =
            SlNetSock_secAttribSet(secAttrib, SLNETSOCK_SEC_ATTRIB_CIPHERS,
                                   (void *)&(SecurityCypher),
                                   sizeof(SecurityCypher));
        if (status < 0)
        {
            SlNetSock_secAttribDelete(secAttrib);
            SlNetSock_close(socketFd);
            return (status);
        }

        if (nwSecurityOpts->nFile == 1)
        {
            status =
                SlNetSock_secAttribSet(secAttrib,
                                       SLNETSOCK_SEC_ATTRIB_PEER_ROOT_CA,
                                       (void *)nwSecurityOpts->files[0],
                                       strlen(
                                           nwSecurityOpts->files[0]));
            if (status < 0)
            {
                SlNetSock_secAttribDelete(secAttrib);
                SlNetSock_close(socketFd);
                return (status);
            }
        }
        else
        {
            for (i = 0; i < nwSecurityOpts->nFile; i++)
            {
                if (NULL != nwSecurityOpts->files[i])
                {
                    attribName =
                        (SlNetSockSecAttrib_e)(SLNETSOCK_SEC_ATTRIB_PRIVATE_KEY + i);
                    status = SlNetSock_secAttribSet(
                        secAttrib, attribName,
                        (void *)nwSecurityOpts->files[i], strlen(nwSecurityOpts->files[i]));
                    if (status < 0)
                    {
                        SlNetSock_secAttribDelete(secAttrib);
                        SlNetSock_close(socketFd);
                        return (status);
                    }
                }
            }
        }
        status = SlNetSock_startSec(socketFd, secAttrib, isServer ? (SLNETSOCK_SEC_BIND_CONTEXT_ONLY | SLNETSOCK_SEC_IS_SERVER) : SLNETSOCK_SEC_BIND_CONTEXT_ONLY);
        if (status < 0)
        {
            SlNetSock_secAttribDelete(secAttrib);
            SlNetSock_close(socketFd);
            return (status);
        }
        SlNetSock_secAttribDelete(secAttrib);
    }
    // If no TLS required
    else
    {
        // check to create a udp or tcp socket
        if ((nwconnOpts & MQTT_DEV_NETCONN_OPT_UDP) != 0)
        {
            socketFd =
                SlNetSock_create(SLNETSOCK_AF_INET, SLNETSOCK_SOCK_DGRAM,
                                 SLNETSOCK_PROTO_UDP,
                                 0,
                                 0);
        }
        else if ((nwconnOpts & MQTT_DEV_NETCONN_OPT_IP6) != 0)
        {
            // 重写让它支持IPv6
            socketFd =
                SlNetSock_create(SLNETSOCK_AF_INET6, SLNETSOCK_SOCK_STREAM,
                                 SLNETSOCK_PROTO_TCP, 0,
                                 0); // consider putting 0 in place of SLNETSOCK_PROTO_TCP
        }
        else // socket for tcp
        {
            socketFd =
                SlNetSock_create(SLNETSOCK_AF_INET, SLNETSOCK_SOCK_STREAM,
                                 SLNETSOCK_PROTO_TCP, 0,
                                 0); // consider putting 0 in place of SLNETSOCK_PROTO_TCP
        }
    }

    return (socketFd);
}

/*
 *  ======== getAddrByName ========
 *  Retrieve host IP address corresponding to a host name
 *  来源于 ti/net/sntp/sntp.c
 */
static int32_t getAddrByName(const char *name,
                             uint32_t *addr,
                             uint16_t *family)
{

    int32_t ifID;
    int32_t i;
    uint16_t addrLen = SLNETUTIL_DNSBUFSIZE / sizeof(uint32_t);
    SlNetSock_AddrIn6_t *sin6 = addr;

    /* Query DNS for IPv4 address. */
    ifID = SlNetUtil_getHostByName(0, (char *)name, strlen(name), addr, &addrLen, SLNETSOCK_AF_INET);
    if (ifID < 0)
    {
        addrLen = SLNETUTIL_DNSBUFSIZE / sizeof(SlNetSock_In6Addr_t);

                ifID = SlNetUtil_getHostByName(0, (char *)name, strlen(name), addr, &addrLen, SLNETSOCK_AF_INET6);
//        ifID = sl_NetAppDnsGetHostByName((char *)name, strlen(name), addr, SL_AF_INET6);

        if (ifID < 0)
        {
            /* return an error */
            UART_PRINT("sl_NetAppDnsGetHostByName error %d\r\n", ifID);
            return (-1);
        }
        *family = SLNETSOCK_AF_INET6;
    }
    else
    {
        *family = SLNETSOCK_AF_INET;
    }

    /* Return the interface ID */
    return (ifID);
}

//*****************************************************************************
//
//! \brief  Open a TCP socket with required properties
//! Also connect to the server.
//! Returns a valid handle on success, NULL on failure.
// MQTTNet46_commOpen 是来自官方MQTT库里的源码 ti/net/mqtt/platform/mqtt_net_func.c 里的MQTTNet_commOpen，此处重写它是为了支持IPv6.
// SINetIfWiFi 网络栈目前版本是不支持IPv6.只有SINetIfNDK(Ethernet)是支持IPv6
//
//*****************************************************************************
static int32_t MQTTNet46_commOpen(uint32_t nwconnOpts,
                                  const char *serverStr,
                                  uint16_t portNumber,
                                  const MQTT_SecureConn_t *nwSecurity)
{
    int32_t socketFd, status, ifID;
    SlNetSock_AddrIn6_t serverAddr;
    int32_t srvAddrSize;
    uint32_t uiIP[4];
    uint16_t family;
    uint32_t nOpts = nwconnOpts;

    // 此处代码参考 SDK ti/net/sntp/sntp.c
    if ((nwconnOpts & MQTT_DEV_NETCONN_OPT_URL) != 0)
    {
        memset(&uiIP, 0, sizeof(uiIP));
        ifID = getAddrByName(serverStr, uiIP, &family);
    }

    // create socket
    socketFd =
        Mqtt_createSocket(family == SLNETSOCK_AF_INET6 ? nOpts | MQTT_DEV_NETCONN_OPT_IP6 : nOpts,
                          (MQTT_SecureConn_t *)nwSecurity, serverStr,
                          false);
    if (socketFd < 0)
    {
        /* ERROR: Could not create a socket */
        UART_PRINT("RROR: Could not create a socket failed\r\n");
        return (socketFd);
    }

    if ((nwconnOpts & MQTT_DEV_NETCONN_OPT_UDP) != 0)
    {
        //filling the UDP server socket address
        ((SlNetSock_AddrIn_t *)&serverAddr)->sin_addr.s_addr = 0;

        status = SlNetSock_bind(socketFd, (SlNetSock_Addr_t *)&serverAddr,
                                srvAddrSize);
        if (status < 0)
        {
            // error
            SlNetSock_close(socketFd);
            return (status);
        }
    }
    else
    {

        // do tcp connect map SlNetIfWifi_connect --> sl_Connect
        // 通过仔细查看SDK的文档，调用关系是 SlNetSock_connect --> SlNetIfWifi_connect --> sl_Connect。

        serverAddr.sin6_family = family;
        serverAddr.sin6_port = SlNetUtil_htons(portNumber);
        srvAddrSize = sizeof(SlNetSock_AddrIn6_t);

        //        SlNetUtil_inetPton(SLNETSOCK_AF_INET6, serverStr, uiIP);
        serverAddr.sin6_addr._S6_un._S6_u32[0] = SlNetUtil_htonl(uiIP[0]);
        serverAddr.sin6_addr._S6_un._S6_u32[1] = SlNetUtil_htonl(uiIP[1]);
        serverAddr.sin6_addr._S6_un._S6_u32[2] = SlNetUtil_htonl(uiIP[2]);
        serverAddr.sin6_addr._S6_un._S6_u32[3] = SlNetUtil_htonl(uiIP[3]);
        UART_PRINT("Ipv6 Server Address : %4x:%4x:%4x:%4x:%4x:%4x:%4x:%4x\r\n",
                   serverAddr.sin6_addr._S6_un._S6_u16[0],
                   serverAddr.sin6_addr._S6_un._S6_u16[1],
                   serverAddr.sin6_addr._S6_un._S6_u16[2],
                   serverAddr.sin6_addr._S6_un._S6_u16[3],
                   serverAddr.sin6_addr._S6_un._S6_u16[4],
                   serverAddr.sin6_addr._S6_un._S6_u16[5],
                   serverAddr.sin6_addr._S6_un._S6_u16[6],
                   serverAddr.sin6_addr._S6_un._S6_u16[7]);
        status = SlNetSock_connect(socketFd, (SlNetSock_Addr_t *)&serverAddr,
                                   srvAddrSize);

        if (status < 0)
        {
            /* ERROR: SlNetSock_connect failed */

            if ((SLNETERR_ESEC_SNO_VERIFY != status) ||
                ((SLNETERR_ESEC_DATE_ERROR != status) &&
                 (nwconnOpts & MQTT_DEV_NETCONN_OPT_SKIP_DATE_VERIFICATION)))
            {
                UART_PRINT("Ipv6 Server ERROR: Could not establish connection to server, Closing the socket\r\n");
                /* ERROR: Could not establish connection to server, Closing the socket */
                SlNetSock_close(socketFd);
                return (status);
            }

            if (status == SLNETERR_BSD_ECONNREFUSED)
            {
                /* 注意：检查服务器是否可以其它客户端连接，是否启动？中间是否有防墙。 */
            }
            // else - SLNETERR_ESEC_SNO_VERIFY == status or SLNETERR_ESEC_DATE_ERROR == status
            /* ERROR: Could not establish secure connection to server,
               Continuing with unsecured connection to server */
        }

        if ((nwconnOpts & MQTT_DEV_NETCONN_OPT_SEC) != 0)
        {
            status = SlNetSock_startSec(
                socketFd, NULL, SLNETSOCK_SEC_START_SECURITY_SESSION_ONLY);
            if (status < 0)
            {
                SlNetSock_close(socketFd);
                return (status);
            }
        }

        // Success - Connected to server
    } // end of doing binding port to udp socket or doing tcp connect

    return (socketFd);
}

int32_t MqttClient_start()
{
    int32_t lRetVal = -1;
    int32_t iCount = 0;

    int32_t threadArg = 100;
    pthread_attr_t pAttrs;
    struct sched_param priParam;

    MqttClientExmple_params.clientId = ClientId;
    MqttClientExmple_params.connParams = &Mqtt_ClientCtx;
    MqttClientExmple_params.mqttMode31 = MQTT_3_1;
    MqttClientExmple_params.blockingSend = true;

    gInitState |= CLIENT_INIT_STATE;
    // 重新指定一个函数,让它支持IPv6.
    MQTTClient_net.open = MQTTNet46_commOpen;
    /*Initialize MQTT client lib                                             */
    gMqttClient = MQTTClient_create(MqttClientCallback,
                                    &MqttClientExmple_params);

    if (gMqttClient == NULL)
    {
        /*lib initialization failed                                          */
        gInitState &= ~CLIENT_INIT_STATE;
        UART_PRINT("lib initialization failed\r\n");
        return (-1);
    }

    /*Open Client Receive Thread start the receive task. Set priority and    */
    /*stack size attributes                                                  */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;
    lRetVal = pthread_attr_setschedparam(&pAttrs, &priParam);
    lRetVal |= pthread_attr_setstacksize(&pAttrs, RXTASKSIZE);
    lRetVal |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    lRetVal |=
        pthread_create(&g_rx_task_hndl, &pAttrs, MqttClientThread,
                       (void *)&threadArg);
    if (lRetVal != 0)
    {
        UART_PRINT("Client Thread Create Failed failed\r\n");
        gInitState &= ~CLIENT_INIT_STATE;
        return (-1);
    }
#ifdef SECURE_CLIENT
    setTime();
#endif

    /*setting will parameters                                                */
    MQTTClient_set(gMqttClient, MQTTClient_WILL_PARAM, &will_param,
                   sizeof(will_param));

#ifdef CLNT_USR_PWD
    /*Set user name for client connection                                    */
    MQTTClient_set(gMqttClient, MQTTClient_USER_NAME, (void *)ClientUsername,
                   strlen(
                       (char *)ClientUsername));

    /*Set password                                                           */
    MQTTClient_set(gMqttClient, MQTTClient_PASSWORD, (void *)ClientPassword,
                   strlen(
                       (char *)ClientPassword));
#endif
    /*Initiate MQTT Connect                                                  */
    if (gApConnectionState >= 0)
    {
#if CLEAN_SESSION == false
        bool clean = CLEAN_SESSION;
        MQTTClient_set(gMqttClient, MQTTClient_CLEAN_CONNECT, (void *)&clean,
                       sizeof(bool));
#endif
        /*The return code of MQTTClient_connect is the ConnACK value that
           returns from the server */
        lRetVal = MQTTClient_connect(gMqttClient);

        /*negative lRetVal means error,
           0 means connection successful without session stored by the server,
           greater than 0 means successful connection with session stored by
           the server */
        if (0 > lRetVal)
        {
            /*lib initialization failed                                      */
            UART_PRINT("Connection to broker failed, Error code: %d\r\n",
                       lRetVal);

            gUiConnFlag = 0;
        }
        else
        {
            gUiConnFlag = 1;
        }
        /*Subscribe to topics when session is not stored by the server       */
        if ((gUiConnFlag == 1) && (0 == lRetVal))
        {
            uint8_t subIndex;
            MQTTClient_SubscribeParams subscriptionInfo[SUBSCRIPTION_TOPIC_COUNT];

            for (subIndex = 0; subIndex < SUBSCRIPTION_TOPIC_COUNT; subIndex++)
            {
                subscriptionInfo[subIndex].topic = topic[subIndex];
                subscriptionInfo[subIndex].qos = qos[subIndex];
            }

            if (MQTTClient_subscribe(gMqttClient, subscriptionInfo,
                                     SUBSCRIPTION_TOPIC_COUNT) < 0)
            {
                UART_PRINT("Subscription Error \r\n");
                MQTTClient_disconnect(gMqttClient);
                gUiConnFlag = 0;
            }
            else
            {
                for (iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
                {
                    UART_PRINT("Client subscribed on %s\r\n,", topic[iCount]);
                }
            }
        }
    }

    gInitState &= ~CLIENT_INIT_STATE;

    return (0);
}

//*****************************************************************************
//!
//! MQTT Client stop - Unsubscribe from the subscription topics and exit the
//! MQTT client lib.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_ClientStop(uint8_t disconnect)
{
    uint32_t iCount;

    MQTTClient_UnsubscribeParams subscriptionInfo[SUBSCRIPTION_TOPIC_COUNT];

    for (iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
        subscriptionInfo[iCount].topic = topic[iCount];
    }

    MQTTClient_unsubscribe(gMqttClient, subscriptionInfo,
                           SUBSCRIPTION_TOPIC_COUNT);
    for (iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
        UART_PRINT("Unsubscribed from the topic %s\r\n", topic[iCount]);
    }
    gUiConnFlag = 0;

    /*exiting the Client library                                             */
    MQTTClient_delete(gMqttClient);
}

//*****************************************************************************
//!
//! Utility function which prints the borders
//!
//! \param[in] ch  -  hold the charater for the border.
//! \param[in] n   -  hold the size of the border.
//!
//! \return none.
//!
//*****************************************************************************

void printBorder(char ch,
                 int n)
{
    int i = 0;

    for (i = 0; i < n; i++)
    {
        putch(ch);
    }
}

//*****************************************************************************
//!
//! Set the ClientId with its own mac address
//! This routine converts the mac address which is given
//! by an integer type variable in hexadecimal base to ASCII
//! representation, and copies it into the ClientId parameter.
//!
//! \param  macAddress  -   Points to string Hex.
//!
//! \return void.
//!
//*****************************************************************************
void SetClientIdNamefromMacAddress(uint8_t *macAddress)
{
    uint8_t Client_Mac_Name[2];
    uint8_t Index;

    /*When ClientID isn't set, use the mac address as ClientID               */
    if (ClientId[0] == '\0')
    {
        /*6 bytes is the length of the mac address                           */
        for (Index = 0; Index < SL_MAC_ADDR_LEN; Index++)
        {
            /*Each mac address byte contains two hexadecimal characters      */
            /*Copy the 4 MSB - the most significant character                */
            Client_Mac_Name[0] = (macAddress[Index] >> 4) & 0xf;
            /*Copy the 4 LSB - the least significant character               */
            Client_Mac_Name[1] = macAddress[Index] & 0xf;

            if (Client_Mac_Name[0] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index] = Client_Mac_Name[0] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index] = Client_Mac_Name[0] + '0';
            }
            if (Client_Mac_Name[1] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + '0';
            }
        }
    }
}

//*****************************************************************************
//!
//! Utility function which Display the app banner
//!
//! \param[in] appName     -  holds the application name.
//! \param[in] appVersion  -  holds the application version.
//!
//! \return none.
//!
//*****************************************************************************

int32_t DisplayAppBanner(char *appName,
                         char *appVersion)
{
    int32_t ret = 0;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t ConfigSize = 0;
    uint8_t ConfigOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    ConfigSize = sizeof(SlDeviceVersion_t);

    /*Print device version info. */
    ret =
        sl_DeviceGet(SL_DEVICE_GENERAL, &ConfigOpt, &ConfigSize,
                     (uint8_t *)(&ver));

    /*Print device Mac address */
    ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);

    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT("\t   %s Example Ver: %s", appName, appVersion);
    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);
    UART_PRINT("\t CHIP: 0x%x", ver.ChipId);
    UART_PRINT(lineBreak);
    UART_PRINT("\t MAC:  %d.%d.%d.%d", ver.FwVersion[0], ver.FwVersion[1],
               ver.FwVersion[2],
               ver.FwVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t PHY:  %d.%d.%d.%d", ver.PhyVersion[0], ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t NWP:  %d.%d.%d.%d", ver.NwpVersion[0], ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t ROM:  %d", ver.RomVersion);
    UART_PRINT(lineBreak);
    UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
    UART_PRINT(lineBreak);
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0],
               macAddress[1], macAddress[2], macAddress[3], macAddress[4],
               macAddress[5]);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);

    SetClientIdNamefromMacAddress(macAddress);

    return (ret);
}

void mainThread(void *args)
{

    uint32_t count = 0;
    pthread_t spawn_thread = (pthread_t)NULL;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;
    UART_Handle tUartHndl;

    int32_t retc = 0;
    /*Initialize SlNetSock layer with CC31xx/CC32xx interface,
     * 这里如果是Eth网卡会是SlNetIfConfigNDK，具体详情请参考 msp432e4_sdk 源码中的NDK部分。  */
    SlNetIf_init(0);
    SlNetIf_add(SLNETIF_ID_1, "CC3120",
                (const SlNetIf_Config_t *)&SlNetIfConfigWifi,
                SLNET_IF_WIFI_PRIO);

    SlNetSock_init(0);
    SlNetUtil_init(0);

    //    GPIO_init();
    //    I2C_init();
    //    SPI_init();

    /*Create the sl_Task                                                     */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASKSTACKSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs_spawn, PTHREAD_CREATE_DETACHED);

    retc = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if (retc != 0)
    {
        UART_PRINT("could not create simplelink task\r\n");
        while (1)
        {
            ;
        }
    }

    retc = sl_Start(0, 0, 0);
    if (retc < 0)
    {
        /*Handle Error */
        UART_PRINT("sl_Start failed\r\n");
        while (1)
        {
            ;
        }
    }

    /*Output device information to the UART terminal */
    retc = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);

    retc = sl_Stop(SL_STOP_TIMEOUT);
    if (retc < 0)
    {
        /*Handle Error */
        UART_PRINT("sl_Stop failed\r\n");
        while (1)
        {
            ;
        }
    }

    if (retc < 0)
    {
        /*Handle Error */
        UART_PRINT("mqtt_client - Unable to retrieve device information \r\n");
        while (1)
        {
            ;
        }
    }

    while (1)
    {
        gResetApplication = false;
        topic[0] = SUBSCRIPTION_TOPIC0;
        topic[1] = SUBSCRIPTION_TOPIC1;
        topic[2] = SUBSCRIPTION_TOPIC2;
        topic[3] = SUBSCRIPTION_TOPIC3;
        gInitState = 0;

        /*Connect to AP                                                      */
        gApConnectionState = Mqtt_IF_Connect();

        gInitState |= MQTT_INIT_STATE;
        /*Run MQTT Main Thread (it will open the Client and Server)          */
        Mqtt_start();

        /*Wait for init to be completed!!!                                   */
        while (gInitState != 0)
        {
            UART_PRINT(".");
            sleep(1);
        }
        UART_PRINT(".\r\n");

        while (gResetApplication == false)
        {
            ;
        }

        UART_PRINT("TO Complete - Closing all threads and resources\r\n");

        /*Stop the MQTT Process                                              */
        Mqtt_Stop();

        UART_PRINT("reopen MQTT # %d  \r\n", ++count);
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
