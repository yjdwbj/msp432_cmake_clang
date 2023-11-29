/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
/* Standard Defines */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include <unistd.h>

/* TI Defines */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART.h>
#include <ti/sap/sap.h>
#include <ti/sbl/sbl.h>
#include <ti/sbl/sbl_image.h>
#include <ti/devices/msp432e4/driverlib/driverlib.h>

/* Local Defines */
#include "project_zero.h"
#include "platform.h"
#include "Board.h"
#include "Profile/led_service.h"
#include "Profile/button_service.h"
#include "Profile/data_service.h"

#include "uart_term.h"

/*******************************************************************************
 *                             LOCAL VARIABLES
 ******************************************************************************/

/* Used to block SNP calls during a synchronous transaction. */
static mqd_t pzQueueRec;
static mqd_t pzQueueSend;
static mqd_t buttonQueueRec;
static mqd_t buttonQueueSend;

/* Project Zero State Machine */
ProjectZero_States_t projectZeroState;

/* Variable used to start the SNP update process */
static bool startSBLTimer = false;

/* Timer objects for debouncing the buttons */
static Timer_Handle button0DebounceTimer;
static Timer_Handle button1DebounceTimer;

/* Timer object for SNP update */
static Timer_Handle updateSNPTimer;

/* Used for log messages */
extern Display_Handle displayOut;

/* State of the buttons */
static volatile uint8_t button0State = 0;
static volatile uint8_t button1State = 0;

/* Task configuration */
static pthread_t pzTask;
static pthread_t snpTask;
static pthread_t buttonTask;
static sem_t updateSNPSemHandle;

/* SAP Parameters for opening serial port to SNP */
static SAP_Params sapParams;

/* GAP - SCAN RSP data (max size = 31 bytes) */
static uint8_t scanRspData[] = {
    /* Complete Name */
    0x0D, /* Length */
    SAP_GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'P', 'r',
    'o', 'j', 'e', 'c', 't', ' ', 'Z', 'e', 'r',
    'o',

    /* Connection interval range */
    0x05, /* Length */
    0x12, /* SAP_GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE */
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    /* TX power level */
    0x02, /* Length */
    0x0A, /* SAP_GAP_ADTYPE_POWER_LEVEL */
    0     /* 0dBm */
};

/* SNP Device Name */
static uint8_t updateSNPDevName[] = {
    'P',
    'r',
    'o',
    'j',
    'e',
    'c',
    't',
    ' ',
    'Z',
    'e',
    'r',
    'o',
};

/* GAP - Advertisement data (max size = 31 bytes, though this is
 best kept short to conserve power while advertising) */
static uint8_t advertData[] = {
    0x02, /* Length */
    SAP_GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | SAP_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    /* Manufacturer specific advertising data */
    0x06,
    0xFF, /* SAP_GAP_ADTYPE_MANUFACTURER_SPECIFIC */
    LO_UINT16(TI_COMPANY_ID), HI_UINT16(TI_COMPANY_ID),
    TI_ST_DEVICE_ID,
    TI_ST_KEY_DATA_ID, 0x00 /* Key state */
};

/* Connection Handle - only one device currently allowed to connect to SNP */
static uint16_t connHandle = PROJECT_ZERO_DEFAULT_CONN_HANDLE;

/* BD Addr of the NWP */
static char nwpstr[] = "NWP:  0xFFFFFFFFFFFF";
#define nwpstrIDX 8

/* BD Addr of peer device in connection */
static char peerstr[] = "Peer: 0xFFFFFFFFFFFF";
#define peerstrIDX 8

/* Placeholder variable for characteristic initialization */
static uint8_t initVal[40] = {0};
static uint8_t initString[] = "This is a pretty long string, isn't it!\r\n";

/* PWM Instances used for LED Manipulation */
static PWM_Handle pwmGreen;

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void ProjectZero_init(void);
static void *ProjectZero_taskFxn(void *arg0);
static void *ProjectZero_updateSNP_taskFxn(void *arg0);
static void *ProjectZero_buttonTaskFxn(void *arg0);
static void ProjectZero_initServices(void);
static void ProjectZero_asyncCB(uint8_t cmd1, void *pParams);
static void ProjectZero_processSNPEventCB(uint16_t event,
                                          snpEventParam_t *param);
static void buttonDebounceCallbackFxn(Timer_Handle myHandle);
static void user_handleButtonPress(button_state_t *pState);
static void buttonCallbackFxnBUTTON0(uint_least8_t index);
static void buttonCallbackFxnBUTTON1(uint_least8_t index);
static void updateSNPFxn(Timer_Handle myHandle);
static void ProjectZero_processLEDServiceCB(uint8_t charID);
static void ProjectZero_processLEDServicecccdCB(uint8_t charID, uint16_t value);
static void ProjectZero_processButtonServiceCB(uint8_t charID);
static void ProjectZero_processButtonServicecccdCB(uint8_t charID,
                                                   uint16_t value);
static void ProjectZero_processDataServiceCB(uint8_t charID);
static void ProjectZero_processDataServicecccdCB(uint8_t charID, uint16_t value);

/*******************************************************************************
 *                                 PROFILE CALLBACKS
 ******************************************************************************/
/*
 * LED Characteristic value change callback
 */
BLEProfileCallbacks_t ProjectZero_LEDServiceCBs = {
    ProjectZero_processLEDServiceCB, ProjectZero_processLEDServicecccdCB};

/*
 * Button Characteristic value change callback
 */
BLEProfileCallbacks_t ProjectZero_ButtonServiceCBs = {
    ProjectZero_processButtonServiceCB,
    ProjectZero_processButtonServicecccdCB};

/*
 * Data Characteristic value change callback
 */
BLEProfileCallbacks_t ProjectZero_DataServiceCBs = {
    ProjectZero_processDataServiceCB, ProjectZero_processDataServicecccdCB};

/*******************************************************************************
 *                                 PUBLIC FUNCTIONS
 ******************************************************************************/
/*******************************************************************************
 * @fn      ProjectZero_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
void ProjectZero_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = PROJECT_ZERO_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while (1)
            ;
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, PROJECT_ZERO_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while (1)
            ;
    }

    retc = pthread_create(&pzTask, &pAttrs, ProjectZero_taskFxn, NULL);

    if (retc != 0)
    {
        while (1)
            ;
    }
}

/*******************************************************************************
 * @fn      updateSNP_createTask
 *
 * @brief   Task creation function to update the SNP.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
void updateSNP_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = PROJECT_ZERO_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while (1)
            ;
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, PROJECT_ZERO_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while (1)
            ;
    }

    retc = pthread_create(&snpTask, &pAttrs, ProjectZero_updateSNP_taskFxn,
                          NULL);

    if (retc != 0)
    {
        while (1)
            ;
    }

    /* Semaphore used to start the SNP update */
    sem_init(&updateSNPSemHandle, 1, 0);
}

/*******************************************************************************
 * @fn      buttonTask_createTask
 *
 * @brief   Task creation function to notify a button press.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
void buttonTask_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;
    struct mq_attr attr;

    /* Create RTOS Queue */
    attr.mq_flags = 0;
    attr.mq_maxmsg = 64;
    attr.mq_msgsize = sizeof(button_state_t);
    attr.mq_curmsgs = 0;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = PROJECT_ZERO_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while (1)
            ;
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, PROJECT_ZERO_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while (1)
            ;
    }

    buttonQueueSend = mq_open("ButtonQueue", O_RDWR | O_CREAT | O_NONBLOCK,
                              0664, &attr);
    buttonQueueRec = mq_open("ButtonQueue", O_RDWR | O_CREAT, 0664, &attr);

    retc = pthread_create(&buttonTask, &pAttrs, ProjectZero_buttonTaskFxn,
                          NULL);

    if (retc != 0)
    {
        while (1)
            ;
    }

    /* Semaphore used to start the SNP update */
    sem_init(&updateSNPSemHandle, 1, 0);
}

/*******************************************************************************
 * @fn      ProjectZero_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void ProjectZero_init(void)
{
    Timer_Params timerParams;
    PWM_Params pwmParams;
    struct mq_attr attr;

    /* Create RTOS Queue */
    attr.mq_flags = 0;
    attr.mq_maxmsg = 64;
    attr.mq_msgsize = sizeof(uint32_t);
    attr.mq_curmsgs = 0;

    pzQueueRec = mq_open("ProjectZero", O_RDWR | O_CREAT, 0664, &attr);
    pzQueueSend = mq_open("ProjectZero", O_RDWR | O_CREAT | O_NONBLOCK, 0664,
                          &attr);

    /* Zeroing out initial value array */
    memset(initVal, 0x00, 40);

    /* Register Key Handler */
    GPIO_setCallback(Board_BUTTON0, buttonCallbackFxnBUTTON0);
    GPIO_enableInt(Board_BUTTON0);
    GPIO_setCallback(Board_BUTTON1, buttonCallbackFxnBUTTON1);
    GPIO_enableInt(Board_BUTTON1);

    /* Write to the UART. */
    UART_PRINT("--------- Project Zero Example ---------\r\n");
    UART_PRINT("Initializing the user task, hardware, BLE stack and services.\r\n");

    /* Register to receive notifications from LED service if characteristics 
     have been written to */
    LEDService_registerAppCBs(&ProjectZero_LEDServiceCBs);

    /* Register to receive notifications from Button service if characteristics
     have been written to */
    ButtonService_registerAppCBs(&ProjectZero_ButtonServiceCBs);

    /* Register to receive notifications from Data service if characteristics
     have been written to */
    DataService_registerAppCBs(&ProjectZero_DataServiceCBs);

    /* Create the debounce clock objects for Button 0 and Button 1 */
    Timer_Params_init(&timerParams);

    /* Setting up clock parameters */
    timerParams.period = 50000;
    timerParams.periodUnits = Timer_PERIOD_US;
    timerParams.timerCallback = buttonDebounceCallbackFxn;
    timerParams.timerMode = Timer_ONESHOT_CALLBACK;
    GPIO_setConfig(Board_BUTTON0,
                   (GPIO_PinConfig)(GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING));
    GPIO_setConfig(Board_BUTTON1,
                   (GPIO_PinConfig)(GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING));

    /* Initialize to 50 ms timeout when it is started */
    button0DebounceTimer = Timer_open(Board_Timer1, &timerParams);
    button1DebounceTimer = Timer_open(Board_Timer2, &timerParams);

    if (button0DebounceTimer == NULL)
    {
        UART_PRINT("Failed to initialize button0DebounceTimer!\r\n");
    }
    if (button1DebounceTimer == NULL)
    {
        UART_PRINT("Failed to initialize button1DebounceTimer!\r\n");
    }

    /* Second button */
    timerParams.period = 3000000;
    timerParams.timerCallback = updateSNPFxn;

    /* Initialize to 3 seconds timeout when timer is started. */
    updateSNPTimer = Timer_open(Board_Timer0, &timerParams);
    if (updateSNPTimer == NULL)
    {
        UART_PRINT("Failed to initialize updateSNPTimer!\r\n");
    }

    /* Creating the PWM Instances */
    PWM_Params_init(&pwmParams);
    pwmParams.periodUnits = PWM_PERIOD_COUNTS;
    pwmParams.idleLevel = PWM_IDLE_LOW;
    pwmParams.dutyUnits = PWM_DUTY_COUNTS;
    pwmParams.periodValue = PWM_PERIOD_MAX;

    /* Make the LED green on by default */
    pwmParams.dutyValue = PWM_PERIOD_MAX;
    pwmGreen = PWM_open(Board_PWM0, &pwmParams);

    PWM_start(pwmGreen);
}

/*******************************************************************************
 * @fn      ProjectZero_updateSNP_taskFxn
 *
 * @brief   Application task entry point fto update the SNP.
 *
 ******************************************************************************/
static void *ProjectZero_updateSNP_taskFxn(void *arg0)
{
    SBL_Params params;
    SBL_Image image;
    uint8_t sblStatus;

    while (1)
    {
        if (sem_wait(&updateSNPSemHandle) == 0)
        {
            /* Deactivate Application task */
            pthread_cancel(pzTask);
            pthread_cancel(buttonTask);

            /* Close NP so SBL can use serial port */
            SAP_close();

            /* Initialize SBL parameters and open port to target device */
            SBL_initParams(&params);
            params.targetInterface = SBL_DEV_INTERFACE_UART;
            params.localInterfaceID = Board_UART1;
            params.resetPinID = Board_RESET;
            params.blPinID = Board_MRDY;

            /* If SBL cannot be opened the process cannot proceed */
            if ((sblStatus = SBL_open(&params)) == SBL_SUCCESS)
            {
                /* Reset target and force into SBL code */
                SBL_openTarget();

                UART_PRINT("Programming the CC26xx... \r\n");

                /* Notify user that we are updating the SNP */
                GPIO_write(Board_LED0, Board_LED_OFF);

                /* Turn ON PWM LED */
                PWM_setDuty(pwmGreen, PWM_PERIOD_MAX);

                /* Flash new image to target */
                image.imgType = SBL_IMAGE_TYPE_INT;
                image.imgInfoLocAddr = (uint32_t)&SNP_code[0];
                image.imgLocAddr = (uint32_t)&SNP_code[0];
                image.imgTargetAddr = SNP_IMAGE_START;
                sblStatus = SBL_writeImage(&image);

                /* Reset target and exit SBL code */
                SBL_closeTarget();

                /* Close SBL port to target device */
                SBL_close();
            }

            if (sblStatus != SBL_SUCCESS)
            {
                UART_PRINT("Programming failed!\r\n");
            }
            else
            {
                UART_PRINT("Programming passed!\r\n");
            }

            UART_PRINT("Resetting device.\r\n");

            usleep(100000);

            /* SNP force reset */
            MCU_rebootDevice();
        }
    }
}

static void *ProjectZero_buttonTaskFxn(void *arg0)
{
    uint32_t prio = 0;
    button_state_t curMessage;

    while (1)
    {
        mq_receive(buttonQueueRec, (void *)&curMessage, sizeof(button_state_t),
                   (unsigned int *)&prio);

        user_handleButtonPress(&curMessage);
    }
}

/*******************************************************************************
 * @fn      ProjectZero_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 ******************************************************************************/
static void *ProjectZero_taskFxn(void *arg0)
{
    uint32_t pzEvent = 0;
    struct timespec ts;
    uint8_t enableAdv = 1;
    uint8_t disableAdv = 0;
    uint32_t prio = 0;

    /* Initialize State Machine */
    projectZeroState = PROJECT_ZERO_RESET;

    /* Initialize application */
    ProjectZero_init();

    /* Application main loop */
    while (1)
    {
        switch (projectZeroState)
        {
        case PROJECT_ZERO_RESET:
        {
            /* Make sure CC26xx is not in BSL */
            GPIO_write(Board_RESET, Board_LED_OFF);
            GPIO_write(Board_MRDY, Board_LED_ON);

            usleep(10000);

            GPIO_write(Board_RESET, Board_LED_ON);

            /* Initialize UART port parameters within SAP parameters */
            SAP_initParams(SAP_PORT_REMOTE_UART, &sapParams);

            sapParams.port.remote.mrdyPinID = Board_MRDY;
            sapParams.port.remote.srdyPinID = Board_SRDY;
            sapParams.port.remote.boardID = Board_UART1;

            /* Setup NP module */
            SAP_open(&sapParams);

            /* Register Application thread's callback to receive asynchronous 
             requests from the NP. */
            SAP_setAsyncCB(ProjectZero_asyncCB);

            /* Reset the NP, and await a powerup indication.
             Clear any pending power indications received prior to this reset
             call */
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 5;

            mq_timedreceive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                            (unsigned int *)&prio, &ts);

            SAP_reset();

            do
            {
                pzEvent = 0;
                mq_receive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                           &prio);

                if (pzEvent != PROJECT_ZERO_EVT_PUI)
                {
                    UART_PRINT("[bleThread] Warning! Unexpected Event %lu\r\n",
                               pzEvent);
                }
            } while (pzEvent != PROJECT_ZERO_EVT_PUI);

            /* Read BD ADDR */
            SAP_setParam(SAP_PARAM_HCI, SNP_HCI_OPCODE_READ_BDADDR, 0,
                         NULL);

            /* Setup Services - Service creation is blocking so no need
             to pend */
            ProjectZero_initServices();
            projectZeroState = PROJECT_ZERO_START_ADV;
        }

        break;

        case PROJECT_ZERO_START_ADV:
        {
            /* Turn on user LED0 to indicate advertising */
            GPIO_write(Board_LED0, Board_LED_ON);
            /* Turn off user LED1 */
            GPIO_write(Board_LED1, Board_LED_OFF);

            /* Set advertising data. */
            SAP_setServiceParam(SNP_GGS_SERV_ID, SNP_GGS_DEVICE_NAME_ATT,
                                sizeof(updateSNPDevName), updateSNPDevName);

            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_NOTCONN,
                         sizeof(advertData), advertData);

            /* Set Scan Response data. */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_SCANRSP,
                         sizeof(scanRspData), scanRspData);

            /* Enable Advertising and await NP response */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &enableAdv);

            do
            {
                pzEvent = 0;
                mq_receive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                           &prio);

                if (pzEvent != PROJECT_ZERO_EVT_ADV_ENB)
                {
                    UART_PRINT("[bleThread] Warning! Unexpected Event %lu\r\n",
                               pzEvent);
                }
            } while (pzEvent != PROJECT_ZERO_EVT_ADV_ENB);

            /* Wait for connection or button press to cancel advertisement */
            do
            {
                pzEvent = 0;
                mq_receive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                           &prio);

                if ((pzEvent != PROJECT_ZERO_EVT_ADV_END) && (pzEvent != PROJECT_ZERO_EVT_CONN_EST))
                {
                    UART_PRINT("[bleThread] Warning! Unexpected Event %lu\r\n",
                               pzEvent);
                }
            } while ((pzEvent != PROJECT_ZERO_EVT_ADV_END) && (pzEvent != PROJECT_ZERO_EVT_CONN_EST));

            if (pzEvent == PROJECT_ZERO_EVT_ADV_END)
            {
                projectZeroState = PROJECT_ZERO_START_ADV;
            }
            else if (pzEvent == PROJECT_ZERO_EVT_CONN_EST)
            {
                projectZeroState = PROJECT_ZERO_CONNECTED;
            }
        }

        break;

        case PROJECT_ZERO_CONNECTED:
            /* Turn off user LED0 once connection is established */
            GPIO_write(Board_LED0, Board_LED_OFF);

            /* Before connecting, NP will send the stop ADV message */
            do
            {
                pzEvent = 0;
                mq_receive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                           &prio);

                if (pzEvent != PROJECT_ZERO_EVT_ADV_END)
                {
                    UART_PRINT("[bleThread] Warning! Unexpected Event %lu\r\n",
                               pzEvent);
                }
            } while (pzEvent != PROJECT_ZERO_EVT_ADV_END);

            /* Events that can happen during connection - Client Disconnection
             - AP Disconnection */
            do
            {
                pzEvent = 0;
                mq_receive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                           &prio);

                if (pzEvent != PROJECT_ZERO_EVT_CONN_TERM)
                {
                    UART_PRINT("[bleThread] Warning! Unexpected Event %lu\r\n",
                               pzEvent);
                }
            } while (pzEvent != PROJECT_ZERO_EVT_CONN_TERM);

            /* Client has disconnected from server */
            SAP_setParam(SAP_PARAM_CONN, SAP_CONN_STATE, sizeof(connHandle),
                         (uint8_t *)&connHandle);

            do
            {
                pzEvent = 0;
                mq_receive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                           &prio);

                if ((pzEvent != PROJECT_ZERO_EVT_CONN_TERM) && (pzEvent != PROJECT_ZERO_EVT_ADV_ENB))
                {
                    UART_PRINT("[bleThread] Warning! Unexpected Event %lu\r\n",
                               pzEvent);
                }
            } while (pzEvent != PROJECT_ZERO_EVT_CONN_TERM);
            projectZeroState = PROJECT_ZERO_CANCEL_ADV;

            break;

        case PROJECT_ZERO_CANCEL_ADV:
            /* Cancel Advertisement */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &disableAdv);

            do
            {
                pzEvent = 0;
                mq_receive(pzQueueRec, (void *)&pzEvent, sizeof(uint32_t),
                           &prio);

                if (pzEvent != PROJECT_ZERO_EVT_ADV_END)
                {
                    UART_PRINT("[bleThread] Warning! Unexpected Event %lu\r\n",
                               pzEvent);
                }
            } while (pzEvent != PROJECT_ZERO_EVT_ADV_END);

            projectZeroState = PROJECT_ZERO_IDLE;
            break;

        case PROJECT_ZERO_IDLE:
            /* Turn off user LED0 to indicate stop advertising */
            GPIO_write(Board_LED0, Board_LED_OFF);
            projectZeroState = PROJECT_ZERO_START_ADV;
            break;

        default:
            break;
        }
    }
}

/*******************************************************************************
 * @fn      user_handleButtonPress
 *
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @param   pState  pointer to button_state_t message sent from debounce Swi.
 *
 * @return  None.
 ******************************************************************************/
static uint8_t oldvalue = 0;
static void user_handleButtonPress(button_state_t *pState)
{
    struct msgQueue queueElemRecv;
    struct msgQueue queueElement;
    // 这里把蓝牙里的button profile 移到这里。
    /* Update the service with the new value.
     Will automatically send notification/indication if enabled. */
    switch (pState->pinId)
    {
    case Board_BUTTON0:
        ButtonService_setParameter(BS_BUTTON0_ID, sizeof(pState->state),
                                   &pState->state);
        if (oldvalue != pState->state)
        {
            queueElement.event = 1; // PUBLISH_PUSH_BUTTON_PRESSED;
            queueElement.msgPtr = NULL;
            queueElement.topLen = pState->state;
            /* 把按键值发到mqtt的接收队列里去。                             */
            if (MQTT_SendMsgToQueue(&queueElement))
            {
                UART_PRINT("Queue is full\r\n");
                mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                           NULL);
                MQTT_SendMsgToQueue(&queueElement);
            }
        }
        oldvalue = pState->state;

        break;
    case Board_BUTTON1:
        ButtonService_setParameter(BS_BUTTON1_ID, sizeof(pState->state),
                                   &pState->state);
        break;
    }
}

/*******************************************************************************
 * @fn      ProjectZero_initServices
 *
 * @brief   Configure SNP and register services.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void ProjectZero_initServices(void)
{
    static uint8_t ds_dht11Val[DS_DHT11_LEN] = {0,0};
    static uint8_t ds_bmp180Val[DS_BMP180_LEN] = {0};

    /* Initialization of characteristics in LED_Service that can provide data */
    LEDService_setParameter(LS_LED0_ID, LS_LED0_LEN, initVal);
    LEDService_addService();

    /* Initialization of characteristics in Button_Service that can
     * provide data */
    ButtonService_setParameter(BS_BUTTON0_ID, BS_BUTTON0_LEN, initVal);
    ButtonService_setParameter(BS_BUTTON1_ID, BS_BUTTON1_LEN, initVal);
    ButtonService_addService();

    /* Initialization of characteristics in Button_Service that can
     * provide data */
//    DataService_setParameter(DS_STRING_ID, sizeof(initString), initString);
//    DataService_setParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);
//    DataService_setParameter(DS_HMC5883L_ID, DS_HMC5883L_LEN, initVal);
//    DataService_setParameter(DS_DHT11_ID, DS_DHT11_LEN, ds_dht11Val);
//    DataService_setParameter(DS_BMP180_ID, DS_BMP180_LEN, ds_bmp180Val);
    DataService_addService();

    SAP_registerEventCB(ProjectZero_processSNPEventCB, 0xFFFF);
}

/*
 * This is a callback operating in the NPI task.
 * These are events this application has registered for.
 */
static void ProjectZero_processSNPEventCB(uint16_t event,
                                          snpEventParam_t *param)
{
    uint32_t eventPend;

    switch (event)
    {
    case SNP_CONN_EST_EVT:
    {
        snpConnEstEvt_t *connEstEvt = (snpConnEstEvt_t *)param;

        /* Update peer address string */
        connHandle = connEstEvt->connHandle;
        ProfileUtil_convertBdAddr2Str(&peerstr[peerstrIDX], connEstEvt->pAddr);

        /* Notify state machine of established connection */
        eventPend = PROJECT_ZERO_EVT_CONN_EST;
        mq_send(pzQueueSend, (void *)&eventPend, sizeof(uint32_t), 1);
    }
    break;

    case SNP_CONN_TERM_EVT:
    {
        connHandle = PROJECT_ZERO_DEFAULT_CONN_HANDLE;
        /* Notify state machine of disconnection event */
        eventPend = PROJECT_ZERO_EVT_CONN_TERM;
        mq_send(pzQueueSend, (void *)&eventPend, sizeof(uint32_t), 1);
    }
    break;

    case SNP_ADV_STARTED_EVT:
    {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *)param;
        if (advEvt->status == SNP_SUCCESS)
        {
            /* Notify state machine of Advertisement Enabled */
            eventPend = PROJECT_ZERO_EVT_ADV_ENB;
            mq_send(pzQueueSend, (void *)&eventPend, sizeof(uint32_t), 1);
        }
        else
        {
            eventPend = PROJECT_ZERO_ERROR;
            mq_send(pzQueueSend, (void *)&eventPend, sizeof(uint32_t), 1);
        }
    }
    break;

    case SNP_ADV_ENDED_EVT:
    {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *)param;
        if (advEvt->status == SNP_SUCCESS)
        {
            /* Notify state machine of Advertisement Disabled */
            eventPend = PROJECT_ZERO_EVT_ADV_END;
            mq_send(pzQueueSend, (void *)&eventPend, sizeof(uint32_t), 1);
        }
    }
    break;

    default:
        break;
    }
}

/*
 * This is a callback operating in the NPI task. These are asynchronous
 * indications.
 */
static void ProjectZero_asyncCB(uint8_t cmd1, void *pParams)
{
    uint32_t eventPend;

    switch (SNP_GET_OPCODE_HDR_CMD1(cmd1))
    {
    case SNP_DEVICE_GRP:
    {
        switch (cmd1)
        {
        case SNP_POWER_UP_IND:
            eventPend = PROJECT_ZERO_EVT_PUI;
            mq_send(pzQueueSend, (void *)&eventPend, sizeof(uint32_t), 1);
            break;

        case SNP_HCI_CMD_RSP:
        {
            snpHciCmdRsp_t *hciRsp = (snpHciCmdRsp_t *)pParams;
            switch (hciRsp->opcode)
            {
            case SNP_HCI_OPCODE_READ_BDADDR:
                ProfileUtil_convertBdAddr2Str(&nwpstr[nwpstrIDX],
                                              hciRsp->pData);
                break;
            default:
                break;
            }
        }
        break;

        case SNP_EVENT_IND:
            break;

        default:
            break;
        }
    }
    break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn      buttonDebounceSwiFxn
 *
 * @brief   Callback from timer module on timeout, determines new state
 *          after debouncing
 *
 * @param   buttonId    The pin being debounced
 *
 * @return  none
 ******************************************************************************/
static void buttonDebounceCallbackFxn(Timer_Handle myHandle)
{
    /* Used to send message to applications */
    uint8_t buttonPinVal;
    uint8_t buttonId;
    button_state_t buttonMsg;
    uint8_t sendMsg;

    if (myHandle == button0DebounceTimer)
    {
        buttonId = Board_BUTTON0;
    }
    else
    {
        buttonId = Board_BUTTON1;
    }

    buttonMsg.pinId = buttonId;
    sendMsg = false;

    /* Get current value of the button pin after the clock timeout */
    buttonPinVal = GPIO_read(buttonId);

    /* Set interrupt direction to opposite of debounced state
     if button is now released (button is active low, so release is high) */
    if (buttonPinVal)
    {
        /* Enable negative edge interrupts to wait for press */
        GPIO_setConfig(
            buttonId,
            (GPIO_PinConfig)(GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING));
    }
    else
    {
        /* Enable positive edge interrupts to wait for release */
        GPIO_setConfig(
            buttonId,
            (GPIO_PinConfig)(GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING));
    }

    switch (buttonId)
    {
    case Board_BUTTON0:
        /* If button is now released (buttonPinVal is active low, so release
         * is 1) and button state was pressed (button state is active high so
         * press is 1)
         */
        if (buttonPinVal && button0State)
        {
            /* Button was released */
            buttonMsg.state = 0;
            button0State = 0;
            sendMsg = true;
        }
        else if (!buttonPinVal && !button0State)
        {
            /* Button was pressed */
            buttonMsg.state = 1;
            button0State = 1;
            sendMsg = true;
        }
        break;

    case Board_BUTTON1:
        /* If button is now released (buttonPinVal is active low, so release
         * is 1) and button state was pressed (button state is active high so
         * press is 1)
         */
        if (buttonPinVal && button1State)
        {
            /* Button was released */
            buttonMsg.state = 0;
            button1State = 0;
            sendMsg = true;
        }
        else if (!buttonPinVal && !button1State)
        {
            /* Button was pressed */
            buttonMsg.state = 1;
            button1State = 1;
            sendMsg = true;
        }
        break;
    }

    if ((projectZeroState != PROJECT_ZERO_CONNECTED) && (button0State && button1State))
    {
        if (!startSBLTimer)
        {
            startSBLTimer = true;

            /* Start 6 seconds timer */
            TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
            Timer_start(updateSNPTimer);
        }
    }
    else
    {
        /* Stop timer */
        startSBLTimer = false;
        Timer_stop(updateSNPTimer);
    }

    /* Enable interrupt on that pin */
    GPIO_enableInt(buttonId);

    if (sendMsg == true)
    {
        mq_send(buttonQueueSend, (void *)&buttonMsg, sizeof(button_state_t), 1);
    }
}

/*******************************************************************************
 * @fn      updateSNPFxn
 *
 * @brief   Callback from Timer module on timeout, starts the SNP Update
 *
 * @param   paramId - not used.
 *
 * @return  none
 ******************************************************************************/
static void updateSNPFxn(Timer_Handle myHandle)
{
    /* Change to UPDATE_SNP */
    projectZeroState = PROJECT_ZERO_UPDATE_SNP;

    /* Disable buttons interrupts while we update the SNP */
    GPIO_disableInt(Board_BUTTON0);
    GPIO_disableInt(Board_BUTTON1);

    /* Post to start the update task */
    sem_post(&updateSNPSemHandle);
}

/*******************************************************************************
 * @fn      buttonCallbackFxnBUTTON0
 *
 * @brief   Callback from GPIO driver on interrupt
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void buttonCallbackFxnBUTTON0(uint_least8_t index)
{
    /*  Disable interrupt on the pin */
    GPIO_disableInt(Board_BUTTON0);
    Timer_start(button0DebounceTimer);
}

/*******************************************************************************
 * @fn      buttonCallbackFxnBUTTON1
 *
 * @brief    Callback from GPIO driver on interrupt
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void buttonCallbackFxnBUTTON1(uint_least8_t index)
{
    /*  Disable interrupt on the pin */
    GPIO_disableInt(Board_BUTTON1);
    Timer_start(button1DebounceTimer);
}

/*
 * Callbacks for the LED Service
 */
static void ProjectZero_processLEDServiceCB(uint8_t charID)
{
    uint32_t value = 0;
    uint8_t redColor, greenColor, blueColor;

    switch (PROFILE_ID_CHAR(charID))
    {
    case LED_CHAR0:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            /* Parsing out the RGB data and adjusting the resulting PWM */
            LEDService_getParameter(charID, &value);
            redColor = (value & 0xFF);
            greenColor = (value & 0xFF00) >> 8;
            blueColor = (value & 0xFF0000) >> 16;
            if (redColor > 125)
            {
                /* Turn on user LED0 when redColor is greater than 125 */
                GPIO_write(Board_LED0, Board_LED_ON);
            }
            else
            {
                GPIO_write(Board_LED0, Board_LED_OFF);
            }

            /* Adjust brightness of PWM with respect to greenColor */
            PWM_setDuty(pwmGreen, PWM_PERIOD_MAX * (greenColor / 255.0f));

            if (blueColor > 125)
            {
                /* Turn on user LED1 when blueColor is greater than 125 */
                GPIO_write(Board_LED1, Board_LED_ON);
            }
            else
            {
                GPIO_write(Board_LED1, Board_LED_OFF);
            }
            break;

        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void ProjectZero_processLEDServicecccdCB(uint8_t charID, uint16_t value)
{
}

/*
 * Callbacks for the Button Service
 */
static void ProjectZero_processButtonServiceCB(uint8_t charID)
{
}

static void ProjectZero_processButtonServicecccdCB(uint8_t charID,
                                                   uint16_t value)
{
}

/*
 * Callbacks for the Data Service
 */
static void ProjectZero_processDataServiceCB(uint8_t charID)
{
}

static void ProjectZero_processDataServicecccdCB(uint8_t charID, uint16_t value)
{
}
