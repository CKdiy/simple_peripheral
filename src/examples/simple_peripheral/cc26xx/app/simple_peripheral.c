/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2013-2018, Texas Instruments Incorporated
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

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
   
#include <ti/drivers/UART.h>
   
#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"
#include "hal_uart.h"

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/Watchdog.h>
#include <inc/hw_wdt.h>

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board_key.h"

#include "board.h"

#include "simple_peripheral.h"
   
#include "snv.h"
#include "ibeaconcfg.h"
#include "auxadc.h"

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               60000 //60s

#define SBP_PERIODIC_EVT_PERIOD_10MIN         600000 //10min

#define SBP_PERIODIC_EVT_PERIOD_2s            2000 //2s

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008
#define SBP_SYSTEM_RESTAR_EVT                 0x0010
#define SBP_CLEAR_WDT_EVT                     0x0020

#define DEFAULT_UART_AT_TEST_LEN              4
#define DEFAULT_UART_AT_CMD_LEN               49
#define DEFAULT_UART_AT_RSP_LEN               6
/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
PIN_Config adcPinTable[] = {
    Board_DK_ADC   | PIN_INPUT_EN  | PIN_PULLUP, 	
	PIN_TERMINATE    
};

PIN_Config uart0PinTable[] = {
    Board_UART_RX   | PIN_INPUT_EN  | PIN_PULLUP, 	
	Board_UART_TX   | PIN_INPUT_EN  | PIN_PULLUP, 
	PIN_TERMINATE    
};

static PIN_State adcPinState;
static PIN_Handle adcPin;
static PIN_State uart0PinState;
static PIN_Handle uart0Pin;
/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct sysRestarClock;
static Clock_Struct clearWdtClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];
ibeaconinf_config_t ibeaconInf_Config;
// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  0x03,
  GAP_ADTYPE_16BIT_COMPLETE,
  0xF0,0xFF,
  0x0A,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'B','e','e','L','i','n','k','e','r',
  0x09,
  GAP_ADTYPE_SERVICE_DATA,
  0x78,0x25,      //UUID
  0x27,0x14,
  0x36,0xC5,     //major minor
  0x81,0x01      //电池电量
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  0x1A, 0xFF, 
  0x4C, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
  0x02,       // ID
  0x15,       //Length of the remaining payload
  0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1, //Location UUID
  0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25,
  0x27, 0x14, // Major number 
  0x36, 0xCD, // Minor number 
  0xB5 //(-75dB)
};

const uint8_t D_FRT[10] ={'2','0','1','9','-','0','4','-','0','1'};                //固件发布日期 必须与设备信息一致 
const uint8_t D_FR[14]={'F','M','V','E','R','S','I','O','N','_','0','0','0','1'}; //固件版本      必须与设备信息一致 
const uint8_t D_CKey[16]={0xDE,0x48,0x2B,0x1C,0x22,0x1C,0x6C,0x30,0x3C,0xF0,0x50,0xEB,0x00,0x20,0xB0,0xBD}; //与生产软件配合使用

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Beelinker";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

extern volatile uint8_t rx_buff_header,rx_buff_tailor;
static uint8_t rxbuff[RX_BUFF_SIZE];

#ifdef IWDG_ENABLE 
//Watchdog_Params params;
Watchdog_Handle watchdog;
#endif
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD
static void SimpleBLEPeripheral_uart0Task(void);
static void uart0BoardReciveCallback(UART_Handle handle, void *buf, size_t count);
void SimpleBLEPeripheral_BleParameterGet(void);
void SimpleBLEPeripheral_LowPowerMgr(void);
uint8_t str_Compara( uint8_t *ptr1, uint8_t *ptr2, uint8_t len);

#ifdef IWDG_ENABLE 
void wdtInitFxn(void);
#endif
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{
  uint16_t adcvalue = 0;  
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);
  
  Util_constructClock(&sysRestarClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD_10MIN, 0, false, SBP_SYSTEM_RESTAR_EVT);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
  
  Nvram_Init();
  SimpleBLEPeripheral_BleParameterGet();
  adcvalue = adc_OneShot_Read() + 117;
  scanRspData[23] = adcvalue & 0xFF;
  scanRspData[24] = adcvalue >> 8;
  
  if( ibeaconInf_Config.atFlag != (0xFF - 1) )
  	Open_uart0( uart0BoardReciveCallback );
  else
  {
#ifdef IWDG_ENABLE
    wdtInitFxn();
	Util_constructClock(&clearWdtClock, SimpleBLEPeripheral_clockHandler,
                         SBP_PERIODIC_EVT_PERIOD_2s, 0, false, SBP_CLEAR_WDT_EVT);	
	
	Util_startClock(&clearWdtClock);
#endif
  	Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);
	Power_releaseConstraint(PowerCC26XX_IDLE_PD_DISALLOW);
  }
  
  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
	
	if(ibeaconInf_Config.initFlag != 0xFF)
	{
		memcpy(&scanRspData[19], &ibeaconInf_Config.majorValue[0], sizeof(uint32_t));
		memcpy(&advertData[9], &ibeaconInf_Config.uuidValue, DEFAULT_UUID_LEN);
		memcpy(&advertData[9 + DEFAULT_UUID_LEN], &ibeaconInf_Config.majorValue, sizeof(uint32_t));
	}
	
    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }
	
  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    uint8_t txpower = HCI_EXT_TX_POWER_0_DBM;
	  
    if(ibeaconInf_Config.initFlag != 0xFF)
	{
	  switch(ibeaconInf_Config.txInterval)
	  {		
		case 1: advInt = 1400;                            //875ms
		    break;			
		case 2: advInt = DEFAULT_ADVERTISING_INTERVAL*5;  //500ms
		    break;			
		case 3: advInt = DEFAULT_ADVERTISING_INTERVAL*3;  //300ms
		    break;			
		case 4: advInt = 400;                             //250ms
		    break;			
		case 5: advInt = DEFAULT_ADVERTISING_INTERVAL*2;  //200ms
		    break;			
		case 10: advInt = DEFAULT_ADVERTISING_INTERVAL;   //100
		    break;			
		case 20: advInt = DEFAULT_ADVERTISING_INTERVAL/2; //50ms
	        break;			
		case 30: advInt = 48;                             //30ms test
	        break;	
		case 50: advInt = 32;                             //20ms test
	        break;			
	    default: advInt = DEFAULT_ADVERTISING_INTERVAL * 3;
		    break;       
	  }
	  
	  switch(ibeaconInf_Config.txPower)
	  {
		case 0: txpower = HCI_EXT_TX_POWER_0_DBM;         //0dbm                       
		    break;	  
		case 1: txpower = HCI_EXT_TX_POWER_1_DBM;         //1dbm                     
		    break;
		case 2: txpower = HCI_EXT_TX_POWER_2_DBM;         //2dbm                      
		    break;
		case 3: txpower = HCI_EXT_TX_POWER_3_DBM;         //3dbm                   
		    break;
		case 4: txpower = HCI_EXT_TX_POWER_4_DBM;         //4dbm                  
		    break;			
		case 5: txpower = HCI_EXT_TX_POWER_5_DBM;         //5dbm                 
		    break;			
		case 6: txpower = HCI_EXT_TX_POWER_MINUS_3_DBM;   //-3dbm                        
		    break;			
		case 7: txpower = HCI_EXT_TX_POWER_MINUS_6_DBM;   //-6dbm 
		    break;	
		case 8: txpower = HCI_EXT_TX_POWER_MINUS_12_DBM;  //-12dbm  
		    break;	
		case 9: txpower = HCI_EXT_TX_POWER_MINUS_21_DBM;  //-21dbm  
		    break;				
	    default: txpower = HCI_EXT_TX_POWER_0_DBM; 
		    break; 			
	  } 
	}
	
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
	
    HCI_EXT_SetTxPowerCmd(txpower);
  }
 
  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD_ONCHIP
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE

#ifndef FEATURE_OAD_ONCHIP
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1[] = {0x34,0x12}; 
    uint8_t hw[14] ={'H','W','V','E','R','S','I','O','N','_','0','0','0','1'}; 
	
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint16_t),
                               charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, DEFAULT_UUID_LEN,
                               &ibeaconInf_Config.uuidValue[0]);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &ibeaconInf_Config.txPower);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint16_t),
                               &adcvalue);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, sizeof(uint16_t),
                               &ibeaconInf_Config.majorValue[0]);	
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR6, sizeof(uint16_t),
                               &ibeaconInf_Config.minorValue[0]);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR7, sizeof(uint8_t),
                               &ibeaconInf_Config.txInterval);	
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR60, sizeof(uint8_t),
                               &ibeaconInf_Config.Rxp);
	
    memcpy(&hw[10], &ibeaconInf_Config.hwvr[0], sizeof(uint32_t));
    DevInfo_SetParameter(DEVINFO_HARDWARE_REV, sizeof(hw), hw);
    DevInfo_SetParameter(DEVINFO_MANUFACTUREDATE, 10, &ibeaconInf_Config.mDate[0]);
  }
  
  if( ibeaconInf_Config.atFlag == (0xFF - 1) )
  	SimpleBLEPeripheral_LowPowerMgr();

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD_ONCHIP

  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
  
  Util_startClock(&sysRestarClock);
  
  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  HCI_LE_ReadMaxDataLenCmd();
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }
	
    if (events & SBP_PERIODIC_EVT)
    {
      events &= ~SBP_PERIODIC_EVT;
      GAPRole_TerminateConnection();
    }
    else if(events & SBP_SYSTEM_RESTAR_EVT)
    {
        events &= ~SBP_SYSTEM_RESTAR_EVT;
        HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);
    }
	else if( events & SBP_CLEAR_WDT_EVT )
	{
	    events &= ~SBP_CLEAR_WDT_EVT;
	    Util_startClock(&clearWdtClock);
#ifdef IWDG_ENABLE 
	    Watchdog_clear(watchdog);
#endif		
	}
	
	if( ibeaconInf_Config.atFlag != (0xFF - 1) )
		SimpleBLEPeripheral_uart0Task();

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            {
              AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            }
            break;
      
          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      ;
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        DevInfo_SetParameter(DEVINFO_BTADDRESS, B_ADDR_LEN, ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
		  ;
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        }

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      break;

    default:
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t newValue[16];

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);
      break;
	  
    case SIMPLEPROFILE_CHAR2:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR2, &newValue);
      break;	  

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);
      break;

    case SIMPLEPROFILE_CHAR4:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR4, &newValue);
      break;	  

    case SIMPLEPROFILE_CHAR5:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR5, &newValue);
      break;	  

    case SIMPLEPROFILE_CHAR6:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR6, &newValue);
      break;	  

    case SIMPLEPROFILE_CHAR7:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR7, &newValue);
      break;	  
	  
    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD_ONCHIP
}

#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_uart0Task
 *
 * @brief   Uart0 data analysis.
 *
 * @param   ...
 *
 * @return  none
 */
static void SimpleBLEPeripheral_uart0Task(void)
{
	uint8_t heard;
	uint8_t length;
	uint8_t restart;
	uint8_t res;
	uint8_t datebuf[DEFAULT_UART_AT_CMD_LEN];
	uint8_t mac[6];
	uint8_t date[10];
	
	restart = FALSE;
	heard = rx_buff_header;
	if( rx_buff_tailor < heard )
	{
	    length = heard - rx_buff_tailor;
		
		if( length <= DEFAULT_UART_AT_CMD_LEN)
			memcpy( datebuf, (void *)&rxbuff[rx_buff_tailor], length );
		else
		{
			rx_buff_tailor = heard;
			return;		
		}
	}
	else if( rx_buff_tailor > heard)
	{
	    length = RX_BUFF_SIZE - rx_buff_tailor + heard;
		
		if(length <= DEFAULT_UART_AT_CMD_LEN)
		{
		    res =  RX_BUFF_SIZE - rx_buff_tailor;
			memcpy( datebuf, (void *)&rxbuff[rx_buff_tailor], res );
			memcpy( &datebuf[RX_BUFF_SIZE - rx_buff_tailor], (void *)rxbuff,  heard );
		}
		else
		{
			rx_buff_tailor = heard;
			return;
		}
	}
	else
	  return;	
	
	if( DEFAULT_UART_AT_TEST_LEN == length )
	{
		if( str_Compara( datebuf, "AT\r\n", 4 ) )
		{
			Uart0_Write("OK\r\n", 4);	
		}	
	}
	else if( DEFAULT_UART_AT_CMD_LEN == length )
	{
		if( str_Compara( datebuf, "AT+1=", 5) )
		{
			//uuid
			memcpy( &ibeaconInf_Config.uuidValue[0], &datebuf[5], DEFAULT_UUID_LEN );
			//major & minor
			memcpy( &ibeaconInf_Config.majorValue[0], &datebuf[21], sizeof(uint32_t) );
			//hwvr
			memcpy( &ibeaconInf_Config.hwvr[0], &datebuf[35], sizeof(uint32_t) );
			//txPower
			memcpy( &ibeaconInf_Config.txPower, &datebuf[39], sizeof(uint8_t) );
			switch( ibeaconInf_Config.txPower )
			{
				case 1: ibeaconInf_Config.Rxp = 0xAB; break;
				case 3: ibeaconInf_Config.Rxp = 0xB5; break;
				case 5: ibeaconInf_Config.Rxp = 0xBD; break;
				case 7: ibeaconInf_Config.Rxp = 0xC0; break;
				default : ibeaconInf_Config.Rxp = 0xB5;
			}
			//txInterval
			memcpy( &ibeaconInf_Config.txInterval, &datebuf[40], sizeof(uint8_t) );
			//mdate
			memcpy(&ibeaconInf_Config.mDate[0], &datebuf[25], 10);
			
			/***** 用于通过生产软件 ***********/
			memcpy(date, &datebuf[25], sizeof(date));
			memcpy(mac, &datebuf[41], sizeof(mac));
			memset(datebuf, 0, sizeof(datebuf));
			memcpy(datebuf, mac, 6);
			datebuf[6] = ibeaconInf_Config.txPower;
			datebuf[7] = ibeaconInf_Config.txInterval;
			memcpy(&datebuf[8], &ibeaconInf_Config.majorValue[0], 4);
			memcpy(&datebuf[12], &ibeaconInf_Config.uuidValue[0], 16);
			memcpy(&datebuf[28], date, 10);
			datebuf[38] = ibeaconInf_Config.Rxp;
			memcpy(&datebuf[39], &ibeaconInf_Config.hwvr[0], 4); 
			
			Uart0_Write("OK+1\r\n", 6);
			
			Ble_WriteNv_Inf( BLE_NVID_CUST_START + 1, datebuf);		
		}
	}
	else if( DEFAULT_UART_AT_RSP_LEN == length )
	{
		if( str_Compara( datebuf, "AT+?\r\n", 6) )
		{
		    memset(datebuf, 0, sizeof(datebuf));
			Ble_ReadNv_Inf(BLE_NVID_CUST_START + 1, datebuf);
			
			Uart0_Write( "OK+", 3 );	
			Uart0_Write( &datebuf[4], 43);
			Uart0_Write( D_FRT, 10);
			Uart0_Write( &D_FR[10], 4);
			Uart0_Write( D_CKey, 16);
			
			ibeaconInf_Config.atFlag = 0xFF -1;	
			
			Ble_WriteNv_Inf( BLE_NVID_CUST_START, &ibeaconInf_Config.txPower);
			
			restart = TRUE;
		}
	}
	else
	{
	  rx_buff_tailor = heard;
	  return;
	}
	
	rx_buff_tailor = heard;
	
	if( TRUE == restart )
	{
		HCI_EXT_ResetSystemCmd(HCI_EXT_RESET_SYSTEM_HARD);		
	}
}

/*********************************************************************
 * @fn      uart0BoardReciveCallback
 *
 * @brief   Uart0 .
 *
 * @param   ...
 *
 * @return  none
 */
void uart0BoardReciveCallback(UART_Handle handle, void *buf, size_t count)
{
    uint8_t tempcnt;
	uint8_t *ptr;
	uint8_t rxlen;
	
	ptr = (uint8_t *)buf;
	rxlen = count;
	if( rx_buff_header + rxlen >= RX_BUFF_SIZE )
	{
		tempcnt = RX_BUFF_SIZE - rx_buff_header; 
		memcpy((void *)&rxbuff[rx_buff_header], ptr, tempcnt );
		ptr += tempcnt;
		tempcnt = rxlen - tempcnt;
		rx_buff_header = 0;
		memcpy( (void *)&rxbuff[rx_buff_header], ptr, tempcnt );
		rx_buff_header += tempcnt;
	}
	else 
	{
		memcpy( (void *)&rxbuff[rx_buff_header], ptr, rxlen );
		rx_buff_header += rxlen;
	}
	
    UART_read(handle, buf, UART0_RECEICE_BUFF_SIZE);
	
    Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_BleParameterGet
 *
 * @brief    Get Ble Parameter.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLEPeripheral_BleParameterGet(void)
{
  	uint32_t crc;
	
	snvinf_t *ptr = (snvinf_t *)rxbuff; //reuse  
    
	if( Ble_ReadNv_Inf( BLE_NVID_DEVINF_START, (uint8_t *)ptr) == 0 )
	{
		crc = crc32(0, &ptr->ibeaconinf_config.txPower, sizeof(ibeaconinf_config_t));
		if( crc == ptr->crc32)
		  memcpy(&ibeaconInf_Config.txPower, &ptr->ibeaconinf_config.txPower, sizeof(ibeaconinf_config_t));
		else
		  ibeaconInf_Config.initFlag = 0xFF;
	}
	else
		ibeaconInf_Config.initFlag = 0xFF;
	
	memset( (void *)rxbuff, 0, sizeof(rxbuff));
}

void SimpleBLEPeripheral_LowPowerMgr(void)
{
	if(adcPin)
	  PIN_close(adcPin); 
    adcPin = PIN_open(&adcPinState, adcPinTable);
	  
	if(uart0Pin)
		PIN_close(uart0Pin);   
	uart0Pin = PIN_open(&uart0PinState, uart0PinTable); 
}

/****************Watch Dog Functions*********************************/
#ifdef IWDG_ENABLE 
void wdtCallback(UArg handle) 
{
    while(1);
}

void wdtInitFxn(void) 
{
	uint32_t  reloadValue;
	
	Watchdog_Params wp;
	
	Watchdog_init();
	Watchdog_Params_init(&wp);
	wp.callbackFxn    = (Watchdog_Callback)wdtCallback;
	wp.debugStallMode = Watchdog_DEBUG_STALL_ON;
	wp.resetMode      = Watchdog_RESET_ON;
 
	watchdog = Watchdog_open(CC2650_WATCHDOG0, &wp);
	reloadValue = Watchdog_convertMsToTicks(watchdog, 3500000);
	
	if( reloadValue != 0)
	    Watchdog_setReload(watchdog, reloadValue); //  (WDT runs always at 48MHz/32)
}
#endif
/********************************************************************/

/*********************************************************************
 * @fn      str_Compara
 *
 * @brief   Compare the array.
 *
 * @param   ..
 *
 * @return  0 Or 1
 */
uint8_t str_Compara( uint8_t *ptr1, uint8_t *ptr2, uint8_t len) 
{
	uint8_t i;
	
	for( i=0; i<len; i++ )
	{
		if( ptr1[i] != ptr2[i] )
			return 0;  	
	}
	
	return 1;  
}
/*********************************************************************
*********************************************************************/
