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
#include <ti/display/Display.h>
#include <ti/sap/snp.h>
#include <ti/sap/snp_rpc.h>
#include <ti/sap/snp_rpc_synchro.h>
#include <ti/sap/sap.h>
#include "profile_util.h"
#include "data_service.h"
#include <string.h>
#include <math.h>



#include "uart_term.h"

/*******************************************************************************
 *                                   MACROS
 ******************************************************************************/
#define DATA_NUM_ATTR_SUPPORTED 5

/*******************************************************************************
 *                                  GLOBAL TYPEDEFS
 ******************************************************************************/
/* Data_Service Service UUID */
static uint8_t DataServiceUUID[SNP_128BIT_UUID_SIZE] =
    {TI_BASE_UUID_128(DATA_SERVICE_UUID)};

/* String UUID */
static uint8_t ds_StringUUID[SNP_128BIT_UUID_SIZE] =
    {TI_BASE_UUID_128(DS_STRING_UUID)};

/* Stream UUID */
static uint8_t ds_StreamUUID[SNP_128BIT_UUID_SIZE] =
    {TI_BASE_UUID_128(DS_STREAM_UUID)};

/* HMC5883L UUID */
static uint8_t ds_HMC5883L_UUID[SNP_128BIT_UUID_SIZE] =
    {TI_BASE_UUID_128(DS_HMC5883L_UUID)};

/* DHT11 UUID */
static uint8_t ds_DHT11_UUID[SNP_128BIT_UUID_SIZE] =
    {TI_BASE_UUID_128(DS_DHT11_UUID)};

/* BMP180 UUID */
static uint8_t ds_BMP180_UUID[SNP_128BIT_UUID_SIZE] =
    {TI_BASE_UUID_128(DS_BMP180_UUID)};
/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
static BLEProfileCallbacks_t *dataServiceCallback;
static uint8_t cccdFlag = 0;
static uint16_t connHandle = 0;

/* Used for log messages */
extern Display_Handle displayOut;

/*******************************************************************************
 *                              Profile Attributes - TYPEDEFS
 ******************************************************************************/
static SAP_Service_t DataService;
static SAP_CharHandle_t DataServiceCharHandles[DATA_NUM_ATTR_SUPPORTED];

/* Profile Service attribute */
static UUIDType_t DataServiceDecl =
    {SNP_128BIT_UUID_SIZE, DataServiceUUID};

/* Characteristic "String" Value variable */
static uint8_t ds_StringVal[DS_STRING_LEN] = {0};

/* Length of data in characteristic "String" Value variable,
 * initialized to minimal size.
 */
static uint16_t ds_StringValLen = DS_STRING_LEN_MIN;

/* Characteristic "Stream" Value variable */
static uint8_t ds_StreamVal[DS_STREAM_LEN] = {0};

/* Characteristic "Stream" Value variable */
static uint8_t ds_hmc5883lVal[DS_HMC5883L_LEN] = {0};
static uint16_t ds_hmc5883lValLen = 0;

/* dht11 value variable */
static uint8_t ds_dht11Val[DS_DHT11_LEN] = {0};
static uint16_t ds_dht11ValLen = 0;

/* bmp180 value variable */
static uint8_t ds_bmp180Val[DS_BMP180_LEN] = {0};
static uint16_t ds_bmp180ValLen = 0;

/* Length of data in characteristic "Stream" Value variable,
 * initialized to minimal size.
 */
static uint16_t ds_StreamValLen = DS_STREAM_LEN_MIN;

/*******************************************************************************
 *                              Profile Attributes - TABLE
 ******************************************************************************/
static SAP_UserCCCDAttr_t DataCCCD =
    {SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE};

static SAP_Char_t DataAttrTable[DATA_NUM_ATTR_SUPPORTED] =
    {
        /* String Characteristic */
        {
            {SNP_128BIT_UUID_SIZE, ds_StringUUID},        /* UUID */
            SNP_GATT_PROP_READ | SNP_GATT_PROP_WRITE,     /* Properties */
            SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE, /* Permissions */
        },

        /* Stream Characteristic */
        {
            {SNP_128BIT_UUID_SIZE, ds_StreamUUID},                  /* UUID */
            SNP_GATT_PROP_NOTIFICATION | SNP_GATT_PROP_WRITE_NORSP, /* Properties */
            SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE,           /* Permissions */
            0,                                                      /* User Description */
            &DataCCCD                                               /* CCCD */
        },

        /* Stream Characteristic */
        {
            {SNP_128BIT_UUID_SIZE, ds_HMC5883L_UUID},                  /* UUID */
            SNP_GATT_PROP_NOTIFICATION | SNP_GATT_PROP_WRITE_NORSP, /* Properties */
            SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE,           /* Permissions */
            0,                                                      /* User Description */
            &DataCCCD                                               /* CCCD */
        },
        /* Stream Characteristic */
        {
            {SNP_128BIT_UUID_SIZE, ds_DHT11_UUID},                  /* UUID */
            SNP_GATT_PROP_NOTIFICATION | SNP_GATT_PROP_WRITE_NORSP, /* Properties */
            SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE,           /* Permissions */
            0,                                                      /* User Description */
            &DataCCCD                                               /* CCCD */
        },
        /* Stream Characteristic */
        {
            {SNP_128BIT_UUID_SIZE, ds_BMP180_UUID},                  /* UUID */
            SNP_GATT_PROP_NOTIFICATION | SNP_GATT_PROP_WRITE_NORSP, /* Properties */
            SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE,           /* Permissions */
            0,                                                      /* User Description */
            &DataCCCD                                               /* CCCD */
        },
};

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void Data_processSNPEventCB(uint16_t event, snpEventParam_t *param);
static uint8_t DataService_ReadAttrCB(void *context, uint16_t conn,
                                      uint16_t charHdl, uint16_t offset, uint16_t size, uint16_t *pLen,
                                      uint8_t *pData);
static uint8_t DataService_WriteAttrCB(void *context, uint16_t conn,
                                       uint16_t charHdl, uint16_t len, uint8_t *pData);
static uint8_t DataService_CCCDIndCB(void *context, uint16_t conn,
                                     uint16_t cccdHdl, uint8_t type, uint16_t value);

/*******************************************************************************
 *                                 PUBLIC FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      DataService_addService
 *
 * @brief   Initializes the DataService service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 ******************************************************************************/
uint8_t DataService_addService(void)
{
    /* Register to receive Connection Established Events */
    SAP_registerEventCB(Data_processSNPEventCB,
                        SNP_CONN_EST_EVT | SNP_CONN_TERM_EVT);

    /* Build Service to register with NP */
    DataService.serviceUUID = DataServiceDecl;
    DataService.serviceType = SNP_PRIMARY_SERVICE;
    DataService.charTableLen = DATA_NUM_ATTR_SUPPORTED;
    DataService.charTable = DataAttrTable;
    DataService.context = NULL;
    DataService.charReadCallback = DataService_ReadAttrCB;
    DataService.charWriteCallback = DataService_WriteAttrCB;
    DataService.cccdIndCallback = DataService_CCCDIndCB;
    DataService.charAttrHandles = DataServiceCharHandles;

    UART_PRINT("DataService: Registered service, %d attributes\r\n",
               DATA_NUM_ATTR_SUPPORTED);

    /* Service is set up, register with GATT server on the SNP. */
    return SAP_registerService(&DataService);
}

/*******************************************************************************
 * @fn      DataService_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  BLE_PROFILE_SUCCESS or BLE_PROFILE_ALREADY_IN_REQ_MODE
 ******************************************************************************/
uint8_t DataService_registerAppCBs(BLEProfileCallbacks_t *appCallbacks)
{
    if (dataServiceCallback == NULL)
    {
        if (appCallbacks != NULL)
        {
            dataServiceCallback = appCallbacks;
            UART_PRINT("DataService: Registered callbacks to application.\r\n");
        }
        else
        {
            UART_PRINT("DataService: nullptr given for application callbacks.\r\n");
            return (BLE_PROFILE_FAILURE);
        }
        return (BLE_PROFILE_SUCCESS);
    }

    return (BLE_PROFILE_ALREADY_IN_REQ_MODE);
}

/*******************************************************************************
 * @fn      DataService_setParameter
 *
 * @brief   Set a DataService parameter.
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
uint8_t DataService_setParameter(uint8_t param, uint16_t len, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;
    uint8_t *pAttrVal;
    uint16_t *pValLen;
    uint16_t valMinLen;
    uint16_t valMaxLen;
    uint8_t sendNotiInd = false;
    snpNotifIndReq_t localReq;

    switch (PROFILE_ID_CHAR(param))
    {
    case DATA_STRING_CHAR0:
        pAttrVal = ds_StringVal;
        pValLen = &ds_StringValLen;
        valMinLen = DS_STRING_LEN_MIN;
        valMaxLen = DS_STRING_LEN;
        break;

    case DATA_STREAM_CHAR1:
        pAttrVal = ds_StreamVal;
        pValLen = &ds_StreamValLen;
        valMinLen = DS_STREAM_LEN_MIN;
        valMaxLen = DS_STREAM_LEN;
        sendNotiInd = true;
        localReq.attrHandle = ProfileUtil_getHdlFromCharID(DS_STREAM_ID,
                                                           DataServiceCharHandles,
                                                           DATA_NUM_ATTR_SUPPORTED);
        break;

    case DATA_HMC5883L_CHAR2:
        pAttrVal = ds_hmc5883lVal;
        pValLen = &ds_hmc5883lValLen;
        valMinLen = DS_STREAM_LEN_MIN;
        valMaxLen = DS_HMC5883L_LEN;
        sendNotiInd = true;
        localReq.attrHandle = ProfileUtil_getHdlFromCharID(DS_HMC5883L_ID,
                                                           DataServiceCharHandles,
                                                           DATA_NUM_ATTR_SUPPORTED);
        break;

    case DATA_DHT11_CHAR3:
        pAttrVal = ds_dht11Val;
        pValLen = &ds_dht11ValLen;
        valMinLen = DS_STREAM_LEN_MIN;
        valMaxLen = DS_DHT11_LEN;
        sendNotiInd = true;
        localReq.attrHandle = ProfileUtil_getHdlFromCharID(DS_DHT11_ID,
                                                           DataServiceCharHandles,
                                                           DATA_NUM_ATTR_SUPPORTED);
        break;
    case DATA_BMP180_CHAR4:
        pAttrVal = ds_bmp180Val;
        pValLen = &ds_bmp180ValLen;
        valMinLen = DS_STREAM_LEN_MIN;
        valMaxLen = DS_BMP180_LEN;
        sendNotiInd = true;
        localReq.attrHandle = ProfileUtil_getHdlFromCharID(DS_BMP180_ID,
                                                           DataServiceCharHandles,
                                                           DATA_NUM_ATTR_SUPPORTED);
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
        *pValLen = len;

        if (sendNotiInd)
        {
            /* Initialize Request */
            localReq.connHandle = connHandle;
            localReq.pData = pAttrVal;
            localReq.authenticate = 0;

            /* Check for whether a notification or indication should be sent.
           Both flags should never be allowed to be set by NWP */
            if (cccdFlag & SNP_GATT_CLIENT_CFG_NOTIFY)
            {
                localReq.type = SNP_SEND_NOTIFICATION;
                SNP_RPC_sendNotifInd(&localReq, len);
            }
            else if (cccdFlag & SNP_GATT_CLIENT_CFG_INDICATE)
            {
                localReq.type = SNP_SEND_INDICATION;
                SNP_RPC_sendNotifInd(&localReq, len);
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
 * @fn      DataService_getParameter
 *
 * @brief   Get a DataService parameter
 *
 * @param   param - Profile parameter IDz
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  uint8_t
 ******************************************************************************/
uint8_t DataService_getParameter(uint8_t param, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;
    switch (PROFILE_ID_CHAR(param))
    {
    case DATA_STRING_CHAR0:
        memcpy(value, ds_StringVal, ds_StringValLen);
        break;

    case DS_STREAM_ID:
        memcpy(value, ds_StreamVal, ds_StreamValLen);
        break;

    case DS_HMC5883L_ID:
        memcpy(value, ds_hmc5883lVal, ds_hmc5883lValLen);
        break;

    case DS_DHT11_ID:
        memcpy(value, ds_dht11Val, ds_dht11ValLen);
        break;

    case DS_BMP180_ID:
        memcpy(value, ds_bmp180Val, ds_bmp180ValLen);
        break;

    default:
        ret = BLE_PROFILE_INVALIDPARAMETER;
        break;
    }
    return ret;
}

/******************************************************************************
 * @fn      DataService_ReadAttrCB
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
static uint8_t DataService_ReadAttrCB(void *context, uint16_t connectionHandle,
                                      uint16_t charHdl, uint16_t offset, uint16_t size, uint16_t *pLen,
                                      uint8_t *pData)
{
    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(charHdl,
                                                  DataServiceCharHandles,
                                                  DATA_NUM_ATTR_SUPPORTED);
    uint8_t isValid = 0;

    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    /* Find settings for the characteristic to be read. */
    switch (PROFILE_ID_CHAR(charID))
    {
    case DATA_STRING_CHAR0:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = MIN(size, ds_StringValLen);
            memcpy(pData, ds_StringVal, *pLen);
            isValid = 1;
            break;

            /* Other considerations for String can be inserted here */
        default:
            break;
        }
        break;

    case DATA_STREAM_CHAR1:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = MIN(size, ds_StreamValLen);
            memcpy(pData, ds_StreamVal, *pLen);
            isValid = 1;
            break;

            /* Other considerations for stream can be inserted here */
        default:
            break;
        }
        break;

    case DATA_HMC5883L_CHAR2:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = MIN(size, ds_hmc5883lValLen);
            memcpy(pData, ds_hmc5883lVal, *pLen);
            isValid = 1;
            break;

            /* Other considerations for stream can be inserted here */
        default:
            break;
        }
        break;
    case DATA_DHT11_CHAR3:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = MIN(size, ds_dht11ValLen);
            memcpy(pData, ds_dht11Val, *pLen);
            isValid = 1;
            break;

            /* Other considerations for stream can be inserted here */
        default:
            break;
        }
        break;

    case DATA_BMP180_CHAR4:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *pLen = MIN(size, ds_bmp180ValLen);
            memcpy(pData, ds_bmp180Val, *pLen);
            isValid = 1;
            break;

            /* Other considerations for stream can be inserted here */
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
 * @fn      DataService_WriteAttrCB
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
static uint8_t DataService_WriteAttrCB(void *context, uint16_t connectionHandle,
                                       uint16_t charHdl, uint16_t len, uint8_t *pData)
{
    uint8_t status = SNP_UNKNOWN_ATTRIBUTE;
    uint8_t notifyApp = PROFILE_UNKNOWN_CHAR;
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;


    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(charHdl,
                                                  DataServiceCharHandles,
                                                  DATA_NUM_ATTR_SUPPORTED);

    /* Find settings for the characteristic to be read. */
    switch (PROFILE_ID_CHAR(charID))
    {
    case DATA_STRING_CHAR0:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            memcpy(ds_StringVal, pData, len);
            status = SNP_SUCCESS;
            notifyApp = DS_STRING_ID;
            ds_StringValLen = len;
            break;

        /* Other considerations for LED0 can be inserted here */
        default:
            break;
        }
        break;

    case DATA_STREAM_CHAR1:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            DataService_setParameter(DS_STREAM_ID, len, pData);
            notifyApp = DS_STREAM_ID;
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
     * application of change
     */
    if ((notifyApp != PROFILE_UNKNOWN_CHAR) && dataServiceCallback && dataServiceCallback->charChangeCB)
    {
        dataServiceCallback->charChangeCB(notifyApp);
        if(DS_STREAM_ID == notifyApp){
            // 如果是APP里写入的文本,同时发送到MQTT的主题里面。
           queueElement.event = 5; // PUSLISH_BLE_DATA;
           queueElement.msgPtr = ds_StreamVal;
           queueElement.topLen = len;
           /* 把按键值发到mqtt的接收队列里去。                             */
           if (MQTT_SendMsgToQueue(&queueElement))
           {
               mq_receive(g_PBQueue, (char *)&queueElemRecv, sizeof(struct msgQueue),
                          NULL);
               MQTT_SendMsgToQueue(&queueElement);
           }
        }


    }

    return status;
}

/*******************************************************************************
 * @fn      DataService_CCCDIndCB
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
uint8_t DataService_CCCDIndCB(void *context, uint16_t connectionHandle,
                              uint16_t cccdHdl, uint8_t type, uint16_t value)
{
    uint8_t status = SNP_UNKNOWN_ATTRIBUTE;
    uint8_t notifyApp = PROFILE_UNKNOWN_CHAR;

    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(cccdHdl,
                                                  DataServiceCharHandles,
                                                  DATA_NUM_ATTR_SUPPORTED);

    switch (PROFILE_ID_CHAR(charID))
    {
    case DATA_HMC5883L_CHAR2:
    case DATA_DHT11_CHAR3:
    case DATA_BMP180_CHAR4:
    case DATA_STREAM_CHAR1:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* Set global cccd flag which will be used to to gate indications
             or notifications when SetParameter() is called */
            cccdFlag = value;
            notifyApp = charID;
            status = SNP_SUCCESS;
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }

    /* If a characteristic value changed then callback function to notify
     * application of change
     */
    if ((notifyApp != PROFILE_UNKNOWN_CHAR) && dataServiceCallback && dataServiceCallback->cccdUpdateCB)
    {
        dataServiceCallback->cccdUpdateCB(notifyApp, value);
    }

    return status;
}

/*******************************************************************************
 * @fn      Data_processSNPEventCB
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
static void Data_processSNPEventCB(uint16_t event, snpEventParam_t *param)
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
