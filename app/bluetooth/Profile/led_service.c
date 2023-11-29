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
 *                                INCLUDES
 ******************************************************************************/
#include <string.h>
#include <ti/display/Display.h>
#include <ti/sap/snp.h>
#include <ti/sap/snp_rpc.h>
#include <ti/sap/snp_rpc_synchro.h>
#include <ti/sap/sap.h>
#include "led_service.h"

#include "uart_term.h"
/*******************************************************************************
 *                                   MACROS
 ******************************************************************************/
#define LED_NUM_ATTR_SUPPORTED 1

/*******************************************************************************
 *                                  GLOBAL TYPEDEFS
 ******************************************************************************/
/* LED_Service Service UUID */
static uint8_t LEDServiceUUID[SNP_128BIT_UUID_SIZE] =
{ TI_BASE_UUID_128(LED_SERVICE_UUID) };

/* LED0 UUID */
static uint8_t ls_LED0UUID[SNP_128BIT_UUID_SIZE] =
{ TI_BASE_UUID_128(LS_LED0_UUID) };

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
static BLEProfileCallbacks_t *ledServiceCallbacks;

/* Used for log messages */
extern Display_Handle displayOut;

/*******************************************************************************
 *                              Profile Attributes - TYPEDEFS
 ******************************************************************************/
static SAP_Service_t    LEDService;
static SAP_CharHandle_t LEDServiceCharHandles[LED_NUM_ATTR_SUPPORTED];

/* Profile Service attribute */
static UUIDType_t LEDServiceDecl =
{ SNP_128BIT_UUID_SIZE, LEDServiceUUID };

/* Characteristic "LED0" Value variable */
static uint8_t ls_LED0Val[LS_LED0_LEN] = {0};

/* Length of data in characteristic "LED0" Value variable, initialized
 * to minimal size. */
static uint16_t ls_LED0ValLen = LS_LED0_LEN_MIN;

/*******************************************************************************
 *                              Profile Attributes - TABLE
 ******************************************************************************/
static SAP_Char_t LEDAttrTable[LED_NUM_ATTR_SUPPORTED] =
{
    /* LED0 Characteristic */
    {
        { SNP_128BIT_UUID_SIZE, ls_LED0UUID },        /* UUID */
        SNP_GATT_PROP_READ | SNP_GATT_PROP_WRITE | SNP_GATT_PROP_WRITE_NORSP,
                                                      /* Properties */
        SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE, /* Permissions */
    },
};

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void LED_processSNPEventCB(uint16_t event, snpEventParam_t *param);
static uint8_t LEDService_ReadAttrCB(void *context, uint16_t conn,
        uint16_t charHdl, uint16_t offset, uint16_t size, uint16_t * pLen,
        uint8_t *pData);
static uint8_t LEDService_WriteAttrCB(void *context, uint16_t conn,
        uint16_t charHdl, uint16_t len, uint8_t *pData);

/*******************************************************************************
 *                                 PUBLIC FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      LEDService_addService
 *
 * @brief   Initializes the LedService service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 ******************************************************************************/
uint8_t LEDService_addService(void)
{
    /* Register to receive Connection Established Events */
    SAP_registerEventCB(LED_processSNPEventCB,
            SNP_CONN_EST_EVT | SNP_CONN_TERM_EVT);

    /* Build Service to register with NP */
    LEDService.serviceUUID = LEDServiceDecl;
    LEDService.serviceType = SNP_PRIMARY_SERVICE;
    LEDService.charTableLen = LED_NUM_ATTR_SUPPORTED;
    LEDService.charTable = LEDAttrTable;
    LEDService.context = NULL;
    LEDService.charReadCallback = LEDService_ReadAttrCB;
    LEDService.charWriteCallback = LEDService_WriteAttrCB;
    LEDService.cccdIndCallback = NULL;
    LEDService.charAttrHandles = LEDServiceCharHandles;

    UART_PRINT("LEDService: Registered service, %d attributes\n\r",
                    LED_NUM_ATTR_SUPPORTED);

    /* Service is set up, register with GATT server on the SNP. */
    return SAP_registerService(&LEDService);
}

/*******************************************************************************
 * @fn      LEDService_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  BLE_PROFILE_SUCCESS or BLE_PROFILE_ALREADY_IN_REQ_MODE
 ******************************************************************************/
uint8_t LEDService_registerAppCBs(BLEProfileCallbacks_t *appCallbacks)
{
    if (ledServiceCallbacks == NULL)
    {
        if (appCallbacks->cccdUpdateCB != NULL
                && appCallbacks->charChangeCB != NULL)
        {
            ledServiceCallbacks = appCallbacks;
            UART_PRINT("LEDService: Registered callbacks to application.\n\r");
        }
        else
        {
            UART_PRINT("LEDService: nullptr given for application callbacks.\n\r");
            return BLE_PROFILE_FAILURE;
        }
        return BLE_PROFILE_SUCCESS;
    }

    return BLE_PROFILE_ALREADY_IN_REQ_MODE;
}

/*******************************************************************************
 * @fn      LEDService_setParameter
 *
 * @brief   Set a LedService parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  uint8_t
 ******************************************************************************/
uint8_t LEDService_setParameter(uint8_t param, uint16_t len, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;
    uint8_t *pAttrVal;
    uint16_t valMinLen;
    uint16_t valMaxLen;

    switch (PROFILE_ID_CHAR(param))
    {
    case LED_CHAR0:
        pAttrVal = ls_LED0Val;
        valMinLen = LS_LED0_LEN_MIN;
        valMaxLen = LS_LED0_LEN;
        break;

    default:
        ret = BLE_PROFILE_INVALIDPARAMETER;
        break;
    }

    /* Check bounds, update value and send notification or indication
     * if possible. */
    if (len <= valMaxLen && len >= valMinLen)
    {
        memcpy(pAttrVal, value, len);
    }
    else
    {
        ret = BLE_PROFILE_INVALID_RANGE;
    }

    return ret;
}

/*******************************************************************************
 * @fn      LEDService_getParameter
 *
 * @brief   Get a LedService parameter
 *
 * @param   param - Profile parameter IDz
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  uint8_t
 ******************************************************************************/
uint8_t LEDService_getParameter(uint8_t param, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;
    switch (PROFILE_ID_CHAR(param))
    {
    case LED_CHAR0:
        memcpy(value, ls_LED0Val, ls_LED0ValLen);
        break;

    default:
        ret = BLE_PROFILE_INVALIDPARAMETER;
        break;
    }

    return ret;
}

/*******************************************************************************
 * @fn      LEDService_ReadAttrCB
 *
 * @brief   Read an attribute.
 *
 * @param   context - context used when registering service
 * @param   conn    - connection handle ID
 * @param   charHdl - characteristic value handle
 * @param   offset  - offset of data to be read
 * @param   size    - maximum size of data bytes to be read
 * @param   pLen    - amount of bytes copied into pData
 * @param   pData   - pointer to copy read data
 *
 * @return  BLE_PROFILE_SUCCESS, blePending or Failure
 ******************************************************************************/
static uint8_t LEDService_ReadAttrCB(void *context, uint16_t conn,
        uint16_t charHdl, uint16_t offset, uint16_t size, uint16_t * pLen,
        uint8_t *pData)
{
    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(charHdl,
            LEDServiceCharHandles,
            LED_NUM_ATTR_SUPPORTED);
    uint8_t isValid = 0;

    /* Find settings for the characteristic to be read. */
    switch (PROFILE_ID_CHAR(charID))
    {
    case LED_CHAR0:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = sizeof(ls_LED0Val);
            memcpy(pData, ls_LED0Val, sizeof(ls_LED0Val));
            isValid = 1;
            break;

            /* Other considerations for LED0 can be inserted here */
        default:
            break;
        }
        break;

    default:
        break;
    }

    if (isValid)
    {
        return (SNP_SUCCESS);
    }

    /* Unable to find handle - set len to 0 and return error code */
    *pLen = 0;
    return (SNP_UNKNOWN_ATTRIBUTE);
}

/*******************************************************************************
 * @fn      LEDService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  BLE_PROFILE_SUCCESS, blePending or Failure
 ******************************************************************************/
volatile uint32_t rgb = 0;
static uint8_t LEDService_WriteAttrCB(void *context, uint16_t conn,
        uint16_t charHdl, uint16_t len, uint8_t *pData)
{
    uint8_t status = SNP_UNKNOWN_ATTRIBUTE;
    uint8_t notifyApp = PROFILE_UNKNOWN_CHAR;
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;
    uint32_t nrgb = 0;

    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(charHdl,
            LEDServiceCharHandles,
            LED_NUM_ATTR_SUPPORTED);

    /* Find settings for the characteristic to be read. */
    switch (PROFILE_ID_CHAR(charID))
    {
      case LED_CHAR0:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            if (len == sizeof(ls_LED0Val))
            {
                memcpy(ls_LED0Val, pData, sizeof(ls_LED0Val));
                status = SNP_SUCCESS;
                notifyApp = LS_LED0_ID;
            }
            break;

        /* Other considerations for LED1 can be inserted here */
        default:
            break;
        }
        break;

      default:
        break;
    }

    /* If a characteristic value changed then callback function to notify
       application of change */
    if ((notifyApp != PROFILE_UNKNOWN_CHAR)
            && ledServiceCallbacks->charChangeCB != NULL)
    {
        ledServiceCallbacks->charChangeCB(notifyApp);

        // 同时发送到MQTT的主题里面。
        queueElement.event = 6; // PUBSLISH_BLE_RGB;
        queueElement.msgPtr = ls_LED0Val;
        queueElement.topLen = sizeof(ls_LED0Val);
        /* 把按键值发到mqtt的接收队列里去。                             */

        nrgb = ls_LED0Val[0] << 24 | ls_LED0Val[1] << 16 | ls_LED0Val[2];
        if(nrgb != rgb){
            if (MQTT_SendMsgToQueue(&queueElement))
            {
                mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                                      NULL);
                MQTT_SendMsgToQueue(&queueElement);
            }
        }

        rgb = nrgb;
    }

    return status;
}

/*******************************************************************************
 * @fn      LED_processSNPEventCB
 *
 * @brief   This is a callback operating in the NPI task. It will be invoked
 *          whenever an event is received from the SNP that this profile has
 *          registered for
 *
 * @param   event  - event mask
 * @param   pValue - pointer event struct
 *
 * @return  status
 ******************************************************************************/
static void LED_processSNPEventCB(uint16_t event, snpEventParam_t *param)
{
    switch (event)
    {
    case SNP_CONN_EST_EVT:
    {
    }
        break;

    case SNP_CONN_TERM_EVT:
    {
    }
        break;

    default:
        break;
    }
}
