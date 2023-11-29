/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
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

/*
 *  ======== httpsget.c ========
 *  HTTPS Client GET example application
 */

/* BSD support */
#include "string.h"
#include <ti/display/Display.h>

#include <ti/net/http/httpclient.h>
#include <ti/net/slneterr.h>
#include <ti/net/slnetif.h>
#include <ti/net/slnetsock.h>

#include "uart_term.h"
// http协议需要用到 NDK

#define HOSTNAME "http://api.ipify.org/" //  Get self public ip address。
#define REQUEST_URI "/"

#define USER_AGENT "lcy MSP432E40Y (ARM)."

/*
 * Secure object names.
 * Reflects the "Name" field for a Secure Object in SysConfig
 */
#define ROOT_CA_CERT "RootCA"

/*
 * The size of the 'data' array must be set to at least HTTPClient_BUF_LEN
 * defined in 'ti/net/http/httpclient_internal.h'
 */
#define DATA_SIZE 256

// extern Display_Handle display;
#define APP_PRINT Report

// https://e2e.ti.com/support/processors/f/791/t/715672?RTOS-TI-RTOS-PROC-TaskSelf-and-fdOpenSession-TaskSelf-
// https://e2e.ti.com/support/legacy_forums/embedded/tirtos/f/355/t/413759?Why-I-need-to-use-fdOpenSession-TaskSelf-in-sendto-from-another-thread-
// https://software-dl.ti.com/simplelink/esd/simplelink_msp432e4_sdk/2.30.00.14/docs/ndk/NDK_API_Reference.html
extern void fdOpenSession();
extern void fdCloseSession();
extern void *TaskSelf();
extern void startSNTP(void);

// #define HOSTNAME        "https://ipapi.co/"
// #define REQUEST_URI           "/json/"
// uint8_t ti_net_certificate0[] =
//         "-----BEGIN CERTIFICATE-----\r\n"
//         "MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\r\n"
//         "RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\r\n"
//         "VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\r\n"
//         "DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\r\n"
//         "ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\r\n"
//         "VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\r\n"
//         "mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\r\n"
//         "IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\r\n"
//         "mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\r\n"
//         "XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\r\n"
//         "dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\r\n"
//         "jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\r\n"
//         "BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\r\n"
//         "DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\r\n"
//         "9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\r\n"
//         "jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\r\n"
//         "Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\r\n"
//         "ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\r\n"
//         "R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\r\n"
//         "-----END CERTIFICATE-----";

// uint16_t ti_net_certificate0Len = sizeof(ti_net_certificate0);

/*
 *  ======== printError ========
 */
void printError(char *errString, int code) {
    APP_PRINT("Error! code = %d, desc = %s\n", code,
              errString);
    while (1)
        ;
}

/*
 *  ======== httpsTask ========
 *  Makes a HTTP GET request
 */

const void *httpsTask(void *addr) {
    uint16_t status = 0;
    bool moreDataFlag = false;
    char data[DATA_SIZE];
    int ret = 0;
    int16_t len = 0;
    HTTPClient_Handle httpClientHandle;
    int16_t statusCode;
    memset(data, 0, DATA_SIZE);
    //    HTTPClient_extSecParams esParams = {NULL, NULL, ROOT_CA_CERT};

    //    fdOpenSession(TaskSelf());

    /*  Use SNTP to get the current time, as needed for TLS authentication */
    startSNTP();
    //    status = SlNetIf_loadSecObj(SLNETIF_SEC_OBJ_TYPE_CERTIFICATE, ROOT_CA_CERT,
    //                                 strlen(ROOT_CA_CERT), ti_net_certificate0, ti_net_certificate0Len, SLNETIF_ID_1);
    //    printf("SlNetIf_loadSecObj %d\r\n",status);

    do {
        httpClientHandle = HTTPClient_create(&statusCode, 0);
        if (statusCode < 0) {
            printf("httpsTask: creation of http client handle failed %d\r\n", ret);
        }
        ret = HTTPClient_setHeader(httpClientHandle,
                                   HTTPClient_HFIELD_REQ_USER_AGENT, USER_AGENT,
                                   strlen(USER_AGENT) + 1, HTTPClient_HFIELD_PERSISTENT);
        ret = HTTPClient_setHeader(httpClientHandle,
                                   HTTPClient_HFIELD_REQ_ACCEPT, "text/html",
                                   strlen("text/html") + 1, HTTPClient_HFIELD_PERSISTENT);
        ret = HTTPClient_setHeader(httpClientHandle,
                                   HTTPClient_HFIELD_REQ_CONNECTION, "keep-alive",
                                   strlen("keep-alive") + 1, HTTPClient_HFIELD_PERSISTENT);

        ret = HTTPClient_setHeader(httpClientHandle,
                                   HTTPClient_HFIELD_REQ_ACCEPT, "text/html",
                                   strlen("text/html") + 1, HTTPClient_HFIELD_PERSISTENT);
        if (ret < 0) {
            printf("httpsTask: setting request header failed %d\r\n", ret);
        }

        ret = HTTPClient_connect(httpClientHandle, HOSTNAME, 0, 0);
        if (ret < 0) {
            printf("httpsTask: connect failed %d\r\n", ret);
        }

        /* Get the time using the built in NTP server list: */
        ret = HTTPClient_sendRequest(httpClientHandle, HTTP_METHOD_GET,
                                     REQUEST_URI, NULL, 0, 0);
        if (ret < 0) {
            UART_PRINT("httpsTask: send failed %d,sleep 5 secs\r\n", ret);
            sleep(5);
            HTTPClient_disconnect(httpClientHandle);
            HTTPClient_destroy(httpClientHandle);
        }

    } while (ret != HTTP_SC_OK);

    len = 0;

    do {
        ret = HTTPClient_readResponseBody(httpClientHandle, data, sizeof(data),
                                          &moreDataFlag);
        if (ret < 0) {
            UART_PRINT("httpsTask: response body processing failed", ret);
        }
        //        printf("read data %s\r\n",data);
        len += ret;
    } while (moreDataFlag);
    status = SlNetUtil_inetPton(SLNETSOCK_AF_INET, &data, addr);

    //    printf("SlNetUtil_inetPton ret %d,%x\r\n",status,((SlNetSock_InAddr_t *)addr)->s_addr);

    ret = HTTPClient_disconnect(httpClientHandle);
    if (ret < 0) {
        UART_PRINT("httpsTask: disconnect failed\r\n", ret);
    }

    HTTPClient_destroy(httpClientHandle);

    //    fdCloseSession(TaskSelf());
    //    status = SlNetIf_loadSecObj(SLNETIF_SEC_OBJ_TYPE_CERTIFICATE, "RootCA",
    //                                     strlen("RootCA"), NULL, 0, SLNETIF_ID_1);
    //    pthread_exit(0);
    return addr;
}
