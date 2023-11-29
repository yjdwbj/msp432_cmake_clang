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
#ifndef ProjectZero_H
#define ProjectZero_H

#ifdef __cplusplus
extern "C"
{
#endif

   /*******************************************************************************
 *                                FUNCTIONS
 ******************************************************************************/
   /*
 * Task creation function for the Application Processor.
 */
   extern void ProjectZero_createTask(void);
   extern void updateSNP_createTask(void);
   extern void buttonTask_createTask(void);

   extern const unsigned short SNP_code[];

/*******************************************************************************
 *                            CONSTANTS
 ******************************************************************************/
/* Advertising interval when device is discoverable (units 625us, 160=100ms)) */
#define DEFAULT_ADVERTISING_INTERVAL 160

/* Limited discoverable mode advertises for 30.72s, and then stops
   General discoverable mode advertises indefinitely */
#define DEFAULT_DISCOVERABLE_MODE SAP_GAP_ADTYPE_FLAGS_GENERAL

/* Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
   parameter update request is enabled */
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL 80

/* Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
   parameter update request is enabled */
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL 800

   /**
 * Project Zero States.
 */
   typedef enum
   {
      PROJECT_ZERO_RESET,
      PROJECT_ZERO_IDLE,
      PROJECT_ZERO_START_ADV,
      PROJECT_ZERO_CONNECTED,
      PROJECT_ZERO_CANCEL_ADV,
      PROJECT_ZERO_UPDATE_SNP
   } ProjectZero_States_t;

/* Task configuration */
#define PROJECT_ZERO_TASK_PRIORITY      3
#define PROJECT_ZERO_TASK_STACK_SIZE    2048

/* Application Events */
#define PROJECT_ZERO_NONE           0x00000001
#define PROJECT_ZERO_EVT_PUI        0x00000002
#define PROJECT_ZERO_EVT_ADV_ENB    0x00000004
#define PROJECT_ZERO_EVT_ADV_END    0x00000008
#define PROJECT_ZERO_EVT_CONN_EST   0x00000010
#define PROJECT_ZERO_EVT_CONN_TERM  0x00000020
#define PROJECT_ZERO_EVT_BSL_BUTTON 0x00000040
#define PROJECT_ZERO_EVT_BUTTON_RESET 0x00000080
#define PROJECT_ZERO_EVT_BUTTON_RIGHT 0x00000100
#define PROJECT_ZERO_ERROR 0x00000200

/* How often to perform periodic event (in msec) */
#define PROJECT_ZERO_PERIODIC_EVT_PERIOD 5000
#define PROJECT_ZERO_DEFAULT_CONN_HANDLE 0xFFFF

/* Company Identifier: Texas Instruments Inc. (13) */
#define TI_COMPANY_ID      0x000D
#define TI_ST_DEVICE_ID    0x03
#define TI_ST_KEY_DATA_ID  0x00

   /* Struct for message about button state */
   typedef struct
   {
      uint8_t pinId;
      uint8_t state;
   } button_state_t;

/* Period Units for PWM */
#define PWM_PERIOD_MAX 2400

#ifdef __cplusplus
}
#endif

#endif /* ProjectZero_H */
