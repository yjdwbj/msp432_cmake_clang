/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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

#include <time.h>
#include <unistd.h>

#include <ti/display/Display.h>
#include <ti/net/sntp/sntp.h>

/*
 * Difference between NTP Epoch (seconds since January 1, 1900 GMT) and UNIX
 * Epoch (seconds since January 1, 1970 GMT)
 */
#define TIME_BASEDIFF ((((uint32_t)70 * 365 + 17) * 24 * 3600))

#define TIME_NTP_TO_UNIX(t) ((t)-TIME_BASEDIFF)

#define NTP_SERVERS 1
#define NTP_SERVER_PORT 123

/*  Time to wait for reply from server (seconds) */
#define NTP_REPLY_WAIT_TIME 5

/* Must wait at least 15 sec to retry NTP server (RFC 4330) */
#define NTP_POLL_TIME 15

/*
 *  ======== startSNTP ========
 */
void startSNTP(void) {
    uint64_t ntpTimeStamp;
    uint32_t currentTimeNtp = 0;
    uint32_t currentTimeUnix = 0;
    int32_t retval;
    time_t ts;
    SlNetSock_Timeval_t timeval;
    struct timespec tspec;

    static const char *ntpList[] = {
        "cn.pool.ntp.org",
        "ntp1.aliyun.com",
        "ntp.ntsc.ac.cn"};

    /* Set timeout value for NTP server reply */
    timeval.tv_sec = NTP_REPLY_WAIT_TIME;
    timeval.tv_usec = 0;
    do {
        /* Get the time using the built in NTP server list: */

        retval = SNTP_getTime(ntpList, (sizeof(ntpList) / sizeof(const char *)), &timeval, &ntpTimeStamp);
        if (retval != 0) {
            //            Display_printf(display, 0, 0,
            printf("startSNTP: couldn't get time (%d), will retry in %d secs ...",
                   retval, NTP_POLL_TIME);
            sleep(NTP_POLL_TIME);
            //            Display_printf(display, 0, 0, "startSNTP: retrying ...");
        }

        /* Save the current (NTP Epoch based) time */
        currentTimeNtp = ntpTimeStamp >> 32;

    } while (retval < 0);

    /* Set the time. Always pass a time value based on the UNIX Epoch */
    currentTimeUnix = TIME_NTP_TO_UNIX(currentTimeNtp);
    tspec.tv_nsec = 0;
    tspec.tv_sec = currentTimeUnix;
    if (clock_settime(CLOCK_REALTIME, &tspec) != 0) {
        //        Display_printf(display, 0, 0,
        //                "startSNTP: Failed to set current time\n");
        while (1)
            ;
    }

    /*
     * Use time.h APIs to get the time and display it on the console.
     * Must call time.h APIs using the appropriate (toolchain dependent) Epoch
     * time base
     */
#if defined(__TI_COMPILER_VERSION__)
    /* For the TI toolchain, time APIs expect times based on the NTP Epoch */
    ts = currentTimeNtp;
#else
    /* Time APIs for GCC and IAR expect times based on the UNIX Epoch */
    ts = currentTimeUnix;
#endif
    //    printf("startSNTP: Current time: %s\n", ctime(&ts));
}
