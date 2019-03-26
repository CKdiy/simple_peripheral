/******************************************************************************

 @file  devinfoservice.c

 @brief This file contains the Device Information service.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2012-2018, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "osal.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"

#include "devinfoservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Device information service
CONST uint8 devInfoServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DEVINFO_SERV_UUID), HI_UINT16(DEVINFO_SERV_UUID)
};

// System ID
CONST uint8 devInfoBtAddressUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(BTADDRESS_UUID), HI_UINT16(BTADDRESS_UUID)
};

// Model Number String
CONST uint8 devInfoFwReleaseTimeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(FWRELEASETIME_UUID), HI_UINT16(FWRELEASETIME_UUID)
};

// Serial Number String
CONST uint8 devInfoSerialNumberUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SERIAL_NUMBER_UUID), HI_UINT16(SERIAL_NUMBER_UUID)
};

// Firmware Revision String
CONST uint8 devInfoFirmwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(FIRMWARE_REV_UUID), HI_UINT16(FIRMWARE_REV_UUID)
};

// Hardware Revision String
CONST uint8 devInfoHardwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HARDWARE_REV_UUID), HI_UINT16(HARDWARE_REV_UUID)
};

// Manufacture Date String
CONST uint8 devInfoManufactureDateUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MANUFACTUREDATE_UUID), HI_UINT16(MANUFACTUREDATE_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

extern void* memcpy(void *dest, const void *src, size_t len);

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * Profile Attributes - variables
 */

// Device Information Service attribute
static CONST gattAttrType_t devInfoService = { ATT_BT_UUID_SIZE, devInfoServUUID };

// MAC Address characteristic
static uint8 devInfoBtAddressProps = GATT_PROP_READ;
static uint8 devInfoBtAddress[DEVINFO_BTADDRESS_LEN] = {0, 0, 0, 0, 0, 0};

// Firmware Revision Release Time characteristic
static uint8 devInfoFwReleaseTimeProps = GATT_PROP_READ;
static uint8 devInfoFwReleaseTime[11] = "2019-04-01";

// Serial Number String characteristic
static uint8 devInfoSerialNumberProps = GATT_PROP_READ;
static uint8 devInfoSerialNumber[13] = "a02c2f151b15";

// Firmware Revision String characteristic
static uint8 devInfoFirmwareRevProps = GATT_PROP_READ;
static uint8 devInfoFirmwareRev[15] = "FMVERSION_0001";

// Hardware Revision String characteristic
static uint8 devInfoHardwareRevProps = GATT_PROP_READ;
static uint8 devInfoHardwareRev[15] = "HWVERSION_0001";

// Manufacture Date String characteristic
static uint8 devInfoManufactureDateProps = GATT_PROP_READ;
static uint8 devInfoManufactureDate[11] = "2019-04-01";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t devInfoAttrTbl[] =
{
  // Device Information Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&devInfoService                  /* pValue */
  },

    // Device MAC Address Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoBtAddressProps
    },

      // Device MAC Address Value 
      {
        { ATT_BT_UUID_SIZE, devInfoBtAddressUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoBtAddress
      },

    // Firmware Revision Release Time Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoFwReleaseTimeProps
    },

      // Firmware Revision Release Time Value
      {
        { ATT_BT_UUID_SIZE, devInfoFwReleaseTimeUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoFwReleaseTime
      },

    // Serial Number String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSerialNumberProps
    },

      // Serial Number Value
      {
        { ATT_BT_UUID_SIZE, devInfoSerialNumberUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSerialNumber
      },

    // Firmware Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoFirmwareRevProps
    },

      // Firmware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoFirmwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoFirmwareRev
      },

    // Hardware Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoHardwareRevProps
    },

      // Hardware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoHardwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoHardwareRev
      },

    // Manufacture Date String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoManufactureDateProps
    },

      // Manufacture Date String Value
      {
        { ATT_BT_UUID_SIZE, devInfoManufactureDateUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoManufactureDate
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Device Info Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t devInfoCBs =
{
  devInfo_ReadAttrCB, // Read callback function pointer
  NULL,               // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      DevInfo_AddService
 *
 * @brief   Initializes the Device Information service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t DevInfo_AddService( void )
{
  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( devInfoAttrTbl,
                                      GATT_NUM_ATTRS( devInfoAttrTbl ),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &devInfoCBs );
}

/*********************************************************************
 * @fn      DevInfo_SetParameter
 *
 * @brief   Set a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
     case DEVINFO_BTADDRESS:
      if ( len == DEVINFO_BTADDRESS_LEN )
      {
        memcpy(devInfoBtAddress, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_FWRELEASETIME:
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoFwReleaseTime, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoFwReleaseTime, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    case DEVINFO_SERIAL_NUMBER:
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoSerialNumber, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoSerialNumber, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_FIRMWARE_REV:
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoFirmwareRev, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoFirmwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_HARDWARE_REV:
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoHardwareRev, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoHardwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_MANUFACTUREDATE:
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoManufactureDate, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoManufactureDate, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      DevInfo_GetParameter
 *
 * @brief   Get a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case DEVINFO_BTADDRESS:
      memcpy(value, devInfoBtAddress, sizeof(devInfoBtAddress));
      break;

    case DEVINFO_FWRELEASETIME:
      memcpy(value, devInfoFwReleaseTime, DEVINFO_STR_ATTR_LEN);
      break;
	  
    case DEVINFO_SERIAL_NUMBER:
      memcpy(value, devInfoSerialNumber, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_FIRMWARE_REV:
      memcpy(value, devInfoFirmwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_HARDWARE_REV:
      memcpy(value, devInfoHardwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_MANUFACTUREDATE:
      memcpy(value, devInfoManufactureDate, DEVINFO_STR_ATTR_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          devInfo_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  if ( offset >= DEVINFO_STR_ATTR_LEN )
  {
    status = ATT_ERR_ATTR_NOT_LONG ;
  }
  else if( pAttr->type.len ==  ATT_BT_UUID_SIZE)
  {
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid)
    {
      case BTADDRESS_UUID:
		*pLen = sizeof(devInfoBtAddress);
        VOID memcpy(pValue, pAttr->pValue, *pLen);    
        break;
		
      case FWRELEASETIME_UUID:
		*pLen = sizeof(devInfoFwReleaseTime);
        VOID memcpy(pValue, pAttr->pValue, *pLen);    
        break;
		
      case SERIAL_NUMBER_UUID:
		*pLen = sizeof(devInfoSerialNumber);
        VOID memcpy(pValue, pAttr->pValue, *pLen);    
        break;
		
      case FIRMWARE_REV_UUID:
		*pLen = sizeof(devInfoFirmwareRev);
        VOID memcpy(pValue, pAttr->pValue, *pLen);    
        break;
		
      case HARDWARE_REV_UUID:
		*pLen = sizeof(devInfoHardwareRev);
        VOID memcpy(pValue, pAttr->pValue, *pLen);    
        break;
		
      case MANUFACTUREDATE_UUID:
		*pLen = sizeof(devInfoManufactureDate);
        VOID memcpy(pValue, pAttr->pValue, *pLen);    
        break;	 

      default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/
