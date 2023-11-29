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
#include "profile_util.h"
#include "button_service.h"
#include "uart_term.h"

/*******************************************************************************
 *                                   MACROS
 ******************************************************************************/
#define BUTTON_NUM_ATTR_SUPPORTED 2

/*******************************************************************************
 *                                CONSTANTS
 ******************************************************************************/
#define BUTTON_DESCR       "Button Service"

/*******************************************************************************
 *                                  GLOBAL TYPEDEFS
 ******************************************************************************/
/* Button_Service Service UUID */
static uint8_t ButtonServiceUUID[SNP_128BIT_UUID_SIZE] =
 { TI_BASE_UUID_128(BUTTON_SERVICE_UUID) };

/* BUTTON0 UUID */
static uint8_t bs_BUTTON0UUID[SNP_128BIT_UUID_SIZE] =
{ TI_BASE_UUID_128(BS_BUTTON0_UUID) };

/* BUTTON1 UUID */
static uint8_t bs_BUTTON1UUID[SNP_128BIT_UUID_SIZE] =
{ TI_BASE_UUID_128(BS_BUTTON1_UUID) };

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
static BLEProfileCallbacks_t *dataServiceCallbacks;
static uint8_t cccdFlag = 0;
static uint16_t connHandle = 0;

/* Used for log messages */
extern Display_Handle displayOut;

/*******************************************************************************
 *                              Profile Attributes - TYPEDEFS
 ******************************************************************************/
static SAP_Service_t    ButtonService;
static SAP_CharHandle_t ButtonServiceCharHandles[BUTTON_NUM_ATTR_SUPPORTED];

/* Profile Service attribute */
static UUIDType_t ButtonServiceDecl =
{ SNP_128BIT_UUID_SIZE, ButtonServiceUUID };

/* Characteristic "BUTTON0" Value variable */
static uint8_t bs_BUTTON0Val[BS_BUTTON0_LEN] = {0};

/* Characteristic "BUTTON1" Value variable */
static uint8_t bs_BUTTON1Val[BS_BUTTON1_LEN] = {0};

static SAP_UserCCCDAttr_t ButtonDataCCCD =
{ SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE };

static SAP_Char_t ButtonAttrTable[BUTTON_NUM_ATTR_SUPPORTED] =
{
    /* Button 0 Characteristic */
    {
        { SNP_128BIT_UUID_SIZE, bs_BUTTON0UUID }, /* UUID */
        SNP_GATT_PROP_NOTIFICATION,               /* Properties */
        SNP_GATT_PERMIT_READ,                     /* Permissions */
        0,                                        /* User Description */
        &ButtonDataCCCD                           /* CCCD */
    },
    /* Button 1 Characteristic */
    {
        { SNP_128BIT_UUID_SIZE, bs_BUTTON1UUID }, /* UUID */
        SNP_GATT_PROP_NOTIFICATION,               /* Properties */
        SNP_GATT_PERMIT_READ,                     /* Permissions */
        0,                                        /* User Description */
        &ButtonDataCCCD                           /* CCCD */
    },
};

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void Button_processSNPEventCB(uint16_t event, snpEventParam_t *param);
static uint8_t ButtonService_ReadAttrCB(void *context, uint16_t conn,
        uint16_t charHdl, uint16_t offset, uint16_t size, uint16_t * pLen,
        uint8_t *pData);
static uint8_t ButtonService_CCCDIndCB(void *context, uint16_t conn,
        uint16_t cccdHdl, uint8_t type, uint16_t value);

/*******************************************************************************
 *                                 PUBLIC FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      ButtonService_addService
 *
 * @brief   Initializes the ButtonService service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 ******************************************************************************/
uint8_t ButtonService_addService(void)
{
    /* Register to receive Connection Established Events */
    SAP_registerEventCB(Button_processSNPEventCB,
                            SNP_CONN_EST_EVT | SNP_CONN_TERM_EVT);

    /* Build Service to register with NP */
    ButtonService.serviceUUID = ButtonServiceDecl;
    ButtonService.serviceType = SNP_PRIMARY_SERVICE;
    ButtonService.charTableLen = BUTTON_NUM_ATTR_SUPPORTED;
    ButtonService.charTable = ButtonAttrTable;
    ButtonService.context = NULL;
    ButtonService.charReadCallback = ButtonService_ReadAttrCB;
    ButtonService.charWriteCallback = NULL;
    ButtonService.cccdIndCallback = ButtonService_CCCDIndCB;
    ButtonService.charAttrHandles = ButtonServiceCharHandles;

    UART_PRINT("ButtonService: Registered service, %d attributes\r\n",
            BUTTON_NUM_ATTR_SUPPORTED);

    /* Service is set up, register with GATT server on the SNP. */
    return SAP_registerService(&ButtonService);
}

/*******************************************************************************
 * @fn      ButtonService_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  BLE_PROFILE_SUCCESS or BLE_PROFILE_ALREADY_IN_REQ_MODE
 ******************************************************************************/
uint8_t ButtonService_registerAppCBs(BLEProfileCallbacks_t *appCallbacks)
{
    if (dataServiceCallbacks == NULL)
    {
        if (appCallbacks != NULL)
        {
            dataServiceCallbacks = appCallbacks;
            UART_PRINT("ButtonService: registered callbacks to application\r\n");
        } else
        {
            UART_PRINT("ButtonService:  null given for application callbacks.\r\n");
            return BLE_PROFILE_FAILURE;
        }
        return BLE_PROFILE_SUCCESS;
    }

    return BLE_PROFILE_ALREADY_IN_REQ_MODE;
}

/*******************************************************************************
 * @fn      ButtonService_setParameter
 *
 * @brief   Set a ButtonService parameter.
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
uint8_t ButtonService_setParameter(uint8_t param, uint16_t len, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;
    uint8_t *pAttrVal;
    uint16_t valMinLen;
    uint16_t valMaxLen;
    uint8_t sendNotiInd = false;
    snpNotifIndReq_t localReq;

    switch (PROFILE_ID_CHAR(param))
    {
    case BUTTON_CHAR0:
        pAttrVal = bs_BUTTON0Val;
        valMinLen = BS_BUTTON0_LEN_MIN;
        valMaxLen = BS_BUTTON0_LEN;
        sendNotiInd = true;
        localReq.attrHandle = ProfileUtil_getHdlFromCharID(BS_BUTTON0_ID,
                ButtonServiceCharHandles,
                BUTTON_NUM_ATTR_SUPPORTED);
        break;

    case BUTTON_CHAR1:
        pAttrVal = bs_BUTTON1Val;
        valMinLen = BS_BUTTON1_LEN_MIN;
        valMaxLen = BS_BUTTON1_LEN;
        sendNotiInd = true;
        localReq.attrHandle = ProfileUtil_getHdlFromCharID(BS_BUTTON1_ID,
                ButtonServiceCharHandles,
                BUTTON_NUM_ATTR_SUPPORTED);
        break;

    default:
        ret = BLE_PROFILE_INVALIDPARAMETER;
        break;
    }

    /* Check bounds, update value and send notification or indication
     * if possible.
     */
    if (len <= valMaxLen && len >= valMinLen)
    {
        memcpy(pAttrVal, value, len);

        /* Initialize Request */
        localReq.connHandle = connHandle;
        localReq.pData = pAttrVal;
        localReq.authenticate = 0;
        
        if (sendNotiInd)
        {
            /* Check for whether a notification or indication should be sent.
             both flags should never be allowed to be set by NWP */
            if (cccdFlag & SNP_GATT_CLIENT_CFG_NOTIFY)
            {
                localReq.type = SNP_SEND_NOTIFICATION;
                SNP_RPC_sendNotifInd(&localReq, sizeof(uint8_t));
            } else if (cccdFlag & SNP_GATT_CLIENT_CFG_INDICATE)
            {
                localReq.type = SNP_SEND_INDICATION;
                SNP_RPC_sendNotifInd(&localReq, sizeof(uint8_t));
            }
        }
    }
    else
    {
        ret = BLE_PROFILE_INVALID_RANGE;
    }

    return ret;
}

/*******************************************************************************
 * @fn      ButtonService_getParameter
 *
 * @brief   Get a ButtonService parameter
 *
 * @param   param - Profile parameter IDz
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  uint8_t
 ******************************************************************************/
uint8_t ButtonService_getParameter(uint8_t param, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;
    switch (PROFILE_ID_CHAR(param))
    {
    default:
        ret = BLE_PROFILE_INVALIDPARAMETER;
        break;
    }
    return ret;
}

/*******************************************************************************
 * @fn      ButtonService_ReadAttrCB
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
static uint8_t ButtonService_ReadAttrCB(void *context,
        uint16_t connectionHandle, uint16_t charHdl, uint16_t offset,
        uint16_t size, uint16_t * pLen, uint8_t *pData)
{
    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(charHdl,
            ButtonServiceCharHandles,
            BUTTON_NUM_ATTR_SUPPORTED);
    uint8_t isValid = 0;

    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    /* Find settings for the characteristic to be read. */
    switch (PROFILE_ID_CHAR(charID))
    {
    case BUTTON_CHAR0:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = sizeof(bs_BUTTON0Val);
            memcpy(pData, bs_BUTTON0Val, sizeof(bs_BUTTON0Val));
            isValid = 1;
            break;

            /* Other considerations for BUTTON0 can be inserted here */
        default:
            break;
        }
        break;

    case BUTTON_CHAR1:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = sizeof(bs_BUTTON1Val);
            memcpy(pData, bs_BUTTON1Val, sizeof(bs_BUTTON1Val));
            isValid = 1;
            break;

            /* Other considerations for BUTTON1 can be inserted here */
        default:
            break;
        }
        break;

    default:
        break;
    }

    if (isValid)
    {
        return SNP_SUCCESS;
    }

    /* Unable to find handle - set len to 0 and return error code */
    *pLen = 0;
    return SNP_UNKNOWN_ATTRIBUTE;
}

/*******************************************************************************
 * @fn      ButtonService_CCCDIndCB
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
uint8_t ButtonService_CCCDIndCB(void *context, uint16_t connectionHandle,
        uint16_t cccdHdl, uint8_t type, uint16_t value)
{
    uint8_t status = SNP_UNKNOWN_ATTRIBUTE;
    uint8_t notifyApp = PROFILE_UNKNOWN_CHAR;
    uint8_t charID;


    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    /* Get characteristic from handle */
    charID = ProfileUtil_getCharIDFromHdl(cccdHdl, ButtonServiceCharHandles,
    BUTTON_NUM_ATTR_SUPPORTED);

    switch (PROFILE_ID_CHAR(charID))
    {
    case BUTTON_CHAR0:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* Set Global cccd Flag which will be used to to gate Indications
               or notifications when SetParameter() is called */
            cccdFlag = value;
            notifyApp = charID;
            status = SNP_SUCCESS;
            break;
        default:
            /* Should not receive VALUE || USERDESC || FORMAT */
            break;
        }
        break;

    case BUTTON_CHAR1:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* Set Global cccd Flag which will be used to to gate Indications
               or notifications when SetParameter() is called */
            cccdFlag = value;
            notifyApp = charID;
            status = SNP_SUCCESS;
            break;
        default:
            /* Should not receive VALUE || USERDESC || FORMAT */
            break;
        }
        break;

    default:
        /* No other Characteristics have CCCB attributes */
        break;
    }

    /* If a characteristic value changed then callback function to notify
     * application of change */
    if ((notifyApp != PROFILE_UNKNOWN_CHAR) && dataServiceCallbacks
            && dataServiceCallbacks->cccdUpdateCB)
    {
        dataServiceCallbacks->cccdUpdateCB(notifyApp, value);

    }

    return (status);
}

/*******************************************************************************
 * @fn      Button_processSNPEventCB
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
static void Button_processSNPEventCB(uint16_t event, snpEventParam_t *param)
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
