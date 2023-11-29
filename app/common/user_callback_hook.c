#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <pthread.h>

#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/stkmain.h>

#include <ti/ndk/slnetif/slnetifndk.h>
#include <ti/net/slnet.h>
#include <ti/net/slnetif.h>
#include <ti/net/slnetutils.h>

#include "Board.h"
#include "uart_term.h"

#define IPv6_DEVICE_INDEX 1

#define THREADSTACKSIZE 4096

extern void *mainThread(void *arg0);

/* Refer to the NDK documentation to enable IPv6 support.
 **/

/*
 *  ======== netIPAddrHook ========
 *  user defined network IP address hook
 */
void netIPAddrHook(uint32_t IPAddr, unsigned int IfIdx, unsigned int fAdd) {
    pthread_t thread;
    pthread_attr_t attrs;
    struct sched_param priParam;
    int retc;
    int detachState;
    uint32_t hostByteAddr;
    static bool createTask = true;
    int32_t status = 0;

    pthread_attr_t pAttrs;

    if (fAdd) {
        UART_PRINT("Network Added: \n\r");
    } else {
        UART_PRINT("Network Removed: \n\r");
    }

    /* print the IP address that was added/removed */
    hostByteAddr = NDK_ntohl(IPAddr);
    UART_PRINT("If-%d:%d.%d.%d.%d\n\r", IfIdx,
               (uint8_t)(hostByteAddr >> 24) & 0xFF, (uint8_t)(hostByteAddr >> 16) & 0xFF,
               (uint8_t)(hostByteAddr >> 8) & 0xFF, (uint8_t)hostByteAddr & 0xFF);

    /* initialize SlNet interface(s)
     * The ti_net_SlNet_initConfig here is automatically generated when adding a network card through SysConfig,
     * and is replaced with CC3120 initialization here.
     * */
    status = ti_net_SlNet_initConfig();
    if (status < 0) {
        UART_PRINT("Failed to initialize SlNet interface(s)"
                   "- status (%d)\n\r",
                   status);
        while (1)
            ;
    }

    if (fAdd && createTask) {
        /*
         *  From here, start a main thread for CC3120 Booster initialization and start MQTT related
         */

        /* Set priority and stack size attributes */
        pthread_attr_init(&pAttrs);
        priParam.sched_priority = 1;

        detachState = PTHREAD_CREATE_DETACHED;
        retc = pthread_attr_setdetachstate(&pAttrs, detachState);
        if (retc != 0) {
            /* pthread_attr_setdetachstate() failed */
            while (1) {
                ;
            }
        }

        pthread_attr_setschedparam(&pAttrs, &priParam);

        retc |= pthread_attr_setstacksize(&pAttrs, THREADSTACKSIZE);
        if (retc != 0) {
            /* pthread_attr_setstacksize() failed */
            while (1) {
                ;
            }
        }

        retc = pthread_create(&thread, &pAttrs, mainThread, NULL);
        if (retc != 0) {
            /* pthread_create() failed */
            while (1) {
                ;
            }
        }

        createTask = false;
    }
}

/*
 *  ======== IPv6DADStatus ========
 *  IPv6 initialization callback function
 */
static void IPv6DADStatus(IP6N Address, unsigned short dev_index, unsigned char Status) {
    char strIPAddress[40];

    /* Convert the IP Address to String Format. */
    IPv6IPAddressToString(Address, strIPAddress);

    /* Print the status of the address. */
    UART_PRINT("IPv6 address: %s on device %d is %s\n\r",
               strIPAddress, dev_index, (Status == 1) ? "UNIQUE" : "DUPLICATE");

    return;
}

/*
 *  ======== serviceReport ========
 *  NDK service report.  Initially, just reports common system issues.
 */
void serviceReport(uint32_t item, uint32_t status, uint32_t report, void *h) {
    static char *taskName[] = {"Telnet", "", "NAT", "DHCPS", "DHCPC", "DNS"};
    static char *reportStr[] = {"", "Running", "Updated", "Complete", "Fault"};
    static char *statusStr[] =
        {"Disabled", "Waiting", "IPTerm", "Failed", "Enabled"};

    UART_PRINT("Service Status: %-9s: %-9s: %-9s: %03d\n\r",
               taskName[item - 1], statusStr[status], reportStr[report / 256],
               report & 0xFF);

    /* report common system issues */
    if ((item == CFGITEM_SERVICE_DHCPCLIENT) &&
        (status == CIS_SRV_STATUS_ENABLED) &&
        (report == (NETTOOLS_STAT_RUNNING | DHCPCODE_IPADD) ||
         report == (NETTOOLS_STAT_RUNNING | DHCPCODE_IPRENEW))) {
        UART_PRINT("DHCP Client initialization failed; check your network.\n\r");
        uint32_t IPTmp;
        IPTmp = inet_addr("8.8.8.8");

        if (IPTmp)
            CfgAddEntry(0, CFGTAG_SYSINFO, CFGITEM_DHCP_DOMAINNAMESERVER,
                        0, sizeof(IPTmp), (unsigned char *)&IPTmp, 0);
        UART_PRINT("Set Master DNS Server \n\r");
        //        while (1);
    }
}

/*
 *  ======== netOpenHook ========
 *  NDK network open hook used to initialize IPv6
 */
void netOpenHook() {
    int status;
    pthread_attr_t attrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_t thread;
    pthread_attr_t pAttrs;

#ifdef _INCLUDE_IPv6_CODE
    /* Initialize IPv6 */
    llEnter();
    status = IPv6InterfaceInit(IPv6_DEVICE_INDEX, IPv6DADStatus);
    llExit();

    if (status < 0) {
        UART_PRINT("Error %d: Unable to initialize the IPv6 stack on device %d\n\r",
                   status, IPv6_DEVICE_INDEX);
    } else {
        UART_PRINT("IPv6 stack has been initialized on %d\n\r",
                   IPv6_DEVICE_INDEX);
    }
#endif
    //    status = ti_net_SlNet_initConfig();
    //    if (status < 0)
    //    {
    //        UART_PRINT("Failed to initialize SlNet interface(s)"
    //                       "- status (%d)\n\r", status);
    //        while (1);
    //    }
    //
    //    GPIO_init();
    //    SPI_init();
    //
    //
    //    pthread_attr_init(&pAttrs);
    //    priParam.sched_priority = 1;
    //
    //    detachState = PTHREAD_CREATE_DETACHED;
    //    retc = pthread_attr_setdetachstate(&pAttrs, detachState);
    //    if (retc != 0)
    //    {
    //        /* pthread_attr_setdetachstate() failed */
    //        while (1)
    //        {
    //            ;
    //        }
    //    }
    //
    //    pthread_attr_setschedparam(&pAttrs, &priParam);
    //
    //    retc |= pthread_attr_setstacksize(&pAttrs, THREADSTACKSIZE);
    //    if (retc != 0)
    //    {
    //        /* pthread_attr_setstacksize() failed */
    //        while (1)
    //        {
    //            ;
    //        }
    //    }
    //
    //    // mqtt_client_app.c 里的 mainThread
    //    retc = pthread_create(&thread, &pAttrs, mainThread, NULL);
    //    if (retc != 0)
    //    {
    //        /* pthread_create() failed */
    //        while (1)
    //        {
    //            ;
    //        }
    //    }
}

/*
 *  ======== netCloseHook ========
 *  NDK network close hook used to de-initialize IPv6
 */
void netCloseHook() {
    int status = 0;

#ifdef _INCLUDE_IPv6_CODE
    /* Enter the kernel Mode. */
    llEnter();
    status = IPv6InterfaceDeInit(IPv6_DEVICE_INDEX);
    llExit();

    /* Were we able to deinitialize the stack? */
    if (status < 0) {

        UART_PRINT("Error: Unable to de-initialize the IPv6 stack on device %d\n\r",
                   IPv6_DEVICE_INDEX);
    } else {
        UART_PRINT("IPv6 stack has been deinitialized on %d\n\r",
                   IPv6_DEVICE_INDEX);
    }
#endif
}
