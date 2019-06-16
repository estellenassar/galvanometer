/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/PIN.h>
#include <ti/mw/display/Display.h>

#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

// Stack headers
#include <hci_tl.h>
#include <gap.h>
#include <gatt.h>
#include <gapgattserver.h>
#include <gattservapp.h>
#include <osal_snv.h>
#include <gapbondmgr.h>
#include <peripheral.h>
#include <icall_apimsg.h>

#include <devinfoservice.h>

#include "util.h"

#include "Board.h"
#include "project_zero.h"

// Bluetooth Developer Studio services
#include "adc_service.h"
#include "ranger_service.h"

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                      000000

// Task configuration
#define PRZ_TASK_PRIORITY                     1

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   800
#endif

// Internal Events for RTOS application
#define PRZ_STATE_CHANGE_EVT                  0x0001
#define PRZ_CHAR_CHANGE_EVT                   0x0002
#define PRZ_PERIODIC_EVT                      0x0004
#define PRZ_CONN_EVT_END_EVT                  0x0008

/*********************************************************************
 * TYPEDEFS
 */
// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
  uint32   numComparison;
} passcode_req_t;


/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // No scan response data provided.
  0x00 // Placeholder to keep the compiler happy.
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) or general
  // discoverable mode (advertises indefinitely), depending
  // on the DEFAULT_DISCOVERY_MODE define.
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // complete name
  15,
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'A', 'D', 'C', ' ', 'R', 'a', 'n', 'g', 'e', 'r', ' ', 'B', 'L', 'E'

};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "ADC Ranger BLE";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;


/* Pin driver handles */
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 & Board_LED1 are off.
 */
PIN_Config ledPinTable[] = {
  Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

// Global display handle
Display_Handle dispHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ProjectZero_init( void );
static void ProjectZero_taskFxn(UArg a0, UArg a1);

static void user_processApplicationMessage(app_msg_t *pMsg);
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);

static void ProjectZero_sendAttRsp(void);
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg);
static void ProjectZero_freeAttRsp(uint8_t status);

static void user_processGapStateChangeEvt(gaprole_States_t newState);
static void user_gapStateChangeCB(gaprole_States_t newState);
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison);
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status);

// Generic callback handlers for value changes in services.
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

// Task context handlers for generated services.
static void user_AdcService_CfgChangeHandler(char_data_t *pCharData);
static void user_RangerService_CfgChangeHandler(char_data_t *pCharData);

// Task handler for sending notifications.
static void user_updateCharVal(char_data_t *pCharData);

// Utility functions
static char *Util_getLocalNameStr(const uint8_t *data);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t user_gapRoleCBs =
{
  user_gapStateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t user_bondMgrCBs =
{
  user_gapBondMgr_passcodeCB, // Passcode callback
  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */

// ADC Service callback handler.
// The type AdcServiceCBs_t is defined in adc_service.h
static AdcServiceCBs_t user_Adc_ServiceCBs =
{
  .pfnChangeCb    = NULL, // No writable chars in ADC Service, so no change handler.
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

// Ranger Service callback handler.
// The type RangerServiceCBs_t is defined in ranger_service.h
static RangerServiceCBs_t user_Ranger_ServiceCBs =
{
  .pfnChangeCb    = NULL, // No writable chars in Ranger Service, so no change handler.
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void ProjectZero_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = przTaskStack;
  taskParams.stackSize = PRZ_TASK_STACK_SIZE;
  taskParams.priority = PRZ_TASK_PRIORITY;

  Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ProjectZero_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages via ICall to Stack.
  ICall_registerApp(&selfEntity, &sem);

  Log_info0("Initializing the user task, hardware, BLE stack and services.");

  // Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Initialize queue for application messages.
  // Note: Used to transfer control to application thread from e.g. interrupts.
  Queue_construct(&applicationMsgQ, NULL);
  hApplicationMsgQ = Queue_handle(&applicationMsgQ);

  // ******************************************************************
  // Hardware initialization
  // ******************************************************************

  // Open LED pins
  ledPinHandle = PIN_open(&ledPinState, ledPinTable);
  if(!ledPinHandle) {
    Log_error0("Error initializing board LED pins");
    Task_exit();
  }

  // Create the debounce clock objects for Button 0 and Button 1
  Clock_Params clockParams;
  Clock_Params_init(&clockParams);

  // ******************************************************************
  // BLE Stack initialization
  // ******************************************************************

  // Setup the GAP Peripheral Role Profile
  uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up

  // By setting this to zero, the device will go into the waiting state after
  // being discoverable. Otherwise wait this long [ms] before advertising again.
  uint16_t advertOffTime = 0; // miliseconds

  // Set advertisement enabled.
  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                       &initialAdvertEnable);

  // Configure the wait-time before restarting advertisement automatically
  GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                       &advertOffTime);

  // Initialize Scan Response data
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);

  // Initialize Advertisement data
  GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

  Log_info1("Name in advertData array: \x1b[33m%s\x1b[0m",
            (IArg)Util_getLocalNameStr(advertData));

  // Set advertising interval
  uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

  // Set duration of advertisement before stopping in Limited adv mode.
  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds

  // ******************************************************************
  // BLE Bond Manager initialization
  // ******************************************************************
  uint32_t passkey = 0; // passkey "000000"
  uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  uint8_t mitm = TRUE;
  uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8_t bonding = TRUE;

  GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                          &passkey);
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

  // ******************************************************************
  // BLE Service initialization
  // ******************************************************************

  // Add services to GATT server
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

  // Set the device name characteristic in the GAP Profile
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Add services to GATT server and give ID of this task for Indication acks.
  AdcService_AddService(selfEntity);
  RangerService_AddService(selfEntity);

  // Register callbacks with the generated services that
  // can generate events (writes received) to the application
  AdcService_RegisterAppCBs(&user_Adc_ServiceCBs);
  RangerService_RegisterAppCBs(&user_Ranger_ServiceCBs);

  // Placeholder variable for characteristic intialization
  uint8_t initVal[40] = {0};

  // Initalization of characteristics in Adc_Service that can provide data.
  AdcService_SetParameter(AS_VALUE_ID, AS_VALUE_LEN, initVal);

  // Initalization of characteristics in Ranger_Service that can provide data.
  RangerService_SetParameter(RS_DISTANCE_ID, RS_DISTANCE_LEN, initVal);

  // Start the stack in Peripheral mode.
  VOID GAPRole_StartDevice(&user_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&user_bondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // ******************************************************************
  // SC task init
  // ******************************************************************

  SC_init();
}


/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  ProjectZero_init();

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

      // Check if we got a signal because of a stack message
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for event flags received (event signature 0xffff)
          if (pEvt->signature == 0xffff)
          {
            // Event received when a connection event is completed
            if (pEvt->event_flag & PRZ_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              ProjectZero_sendAttRsp();
            }
          }
          else // It's a message from the stack and not an event.
          {
            // Process inter-task message
            safeToDealloc = ProjectZero_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // Process messages sent from another task or another context.
      while (!Queue_empty(hApplicationMsgQ))
      {
        app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

        // Process application-layer message probably sent from ourselves.
        user_processApplicationMessage(pMsg);

        // Free the received message.
        ICall_free(pMsg);
      }
    }
  }
}


/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */
static void user_processApplicationMessage(app_msg_t *pMsg)
{
  char_data_t *pCharData = (char_data_t *)pMsg->pdu;

  switch (pMsg->type)
  {
    case APP_MSG_SERVICE_WRITE: /* Message about received value write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {

      }
      break;

    case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
      /* Call different handler per service */
      switch(pCharData->svcUUID) {
        case ADC_SERVICE_SERV_UUID:
          user_AdcService_CfgChangeHandler(pCharData);
          break;
        case RANGER_SERVICE_SERV_UUID:
          user_RangerService_CfgChangeHandler(pCharData);
          break;
      }
      break;

    case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
      user_updateCharVal(pCharData);
      break;

    case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
      user_processGapStateChangeEvt( *(gaprole_States_t *)pMsg->pdu );
      break;

    case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
      {
        passcode_req_t *pReq = (passcode_req_t *)pMsg->pdu;
        Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
                  (IArg)(pReq->uiInputs?"Sending":"Displaying"),
                  DEFAULT_PASSCODE);
        // Send passcode response.
        GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
      }
      break;
    case APP_MSG_SC_CTRL_READY:
      SC_processCtrlReady();
      break;

    case APP_MSG_SC_TASK_ALERT:
      SC_processTaskAlert();
      break;

    case APP_MSG_SC_EXEC_RANGER:
      SC_execRanger();
      break;
  }
}


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void user_processGapStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
        Log_info1("GAP is started. Our address: \x1b[32m%s\x1b[0m", (IArg)cstr_ownAddress);
      }
      break;

    case GAPROLE_ADVERTISING:
      Log_info0("Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
        Log_info1("Connected. Peer address: \x1b[32m%s\x1b[0m", (IArg)cstr_peerAddress);
       }
      break;

    case GAPROLE_CONNECTED_ADV:
      Log_info0("Connected and advertising");
      break;

    case GAPROLE_WAITING:
      Log_info0("Disconnected / Idle");
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      Log_info0("Connection timed out");
      break;

    case GAPROLE_ERROR:
      Log_info0("Error");
      break;

    default:
      break;
  }
}


/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_AdcService_CfgChangeHandler(char_data_t *pCharData)
{
  // Cast received data to uint16, as that's the format for CCCD writes.
  uint16_t configValue = *(uint16_t *)pCharData->data;
  char *configValString;

  // Determine what to tell the user
  switch(configValue)
  {
  case GATT_CFG_NO_OPERATION:
    configValString = "Noti/Ind disabled";
    break;
  case GATT_CLIENT_CFG_NOTIFY:
    configValString = "Notifications enabled";
    break;
  case GATT_CLIENT_CFG_INDICATE:
    configValString = "Indications enabled";
    break;
  }

  switch (pCharData->paramID)
  {
    case AS_VALUE_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"ADC Service",
                (IArg)"Stream",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;
  }
}


/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_RangerService_CfgChangeHandler(char_data_t *pCharData)
{
  // Cast received data to uint16, as that's the format for CCCD writes.
  uint16_t configValue = *(uint16_t *)pCharData->data;
  char *configValString;

  // Determine what to tell the user
  switch(configValue)
  {
  case GATT_CFG_NO_OPERATION:
    configValString = "Noti/Ind disabled";
    break;
  case GATT_CLIENT_CFG_NOTIFY:
    configValString = "Notifications enabled";
    break;
  case GATT_CLIENT_CFG_INDICATE:
    configValString = "Indications enabled";
    break;
  }

  switch (pCharData->paramID)
  {
    case RS_DISTANCE_ID:
      Log_info3("CCCD Change msg: %s %s: %s",
                (IArg)"Ranger Service",
                (IArg)"Stream",
                (IArg)configValString);
      // -------------------------
      // Do something useful with configValue here. It tells you whether someone
      // wants to know the state of this characteristic.
      // ...
      break;
  }
}


/*
 * @brief   Process an incoming BLE stack message.
 *
 *          This could be a GATT message from a peer device like acknowledgement
 *          of an Indication we sent, or it could be a response from the stack
 *          to an HCI message that the user application sent.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            Log_info0("HCI Command Complete Event received");
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


/*
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    Log_warning1("Outgoing RF FIFO full. Re-schedule transmission of msg with opcode 0x%02x",
      pMsg->method);

    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   PRZ_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      ProjectZero_freeAttRsp(FAILURE);

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

    // Log the opcode of the message that caused the violation.
    Log_error1("Flow control violated. Opcode of offending ATT msg: 0x%02x",
      pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Log_info1("MTU Size change: %d bytes", pMsg->msg.mtuEvt.MTU);
  }
  else
  {
    // Got an expected GATT message from a peer.
    Log_info1("Recevied GATT Message. Opcode: 0x%02x", pMsg->method);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}




/*
 *  Application error handling functions
 *****************************************************************************/

/*
 * @brief   Send a pending ATT response message.
 *
 *          The message is one that the stack was trying to send based on a
 *          peer request, but the response couldn't be sent because the
 *          user application had filled the TX queue with other data.
 *
 * @param   none
 *
 * @return  none
 */
static void ProjectZero_sendAttRsp(void)
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
      ProjectZero_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Log_warning2("Retrying message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
  }
}

/*
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void ProjectZero_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Log_info2("Sent message with opcode 0x%02x. Attempt %d",
        pAttRsp->method, rspTxRetry);
    }
    else
    {
      Log_error2("Gave up message with opcode 0x%02x. Status: %d",
        pAttRsp->method, status);

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


/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/


/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/**
 * Callback from GAP Role indicating a role state change.
 */
static void user_gapStateChangeCB(gaprole_States_t newState)
{
  Log_info1("(CB) GAP State change: %d, Sending msg to app.", (IArg)newState);
  user_enqueueRawAppMsg( APP_MSG_GAP_STATE_CHANGE, (uint8_t *)&newState, sizeof(newState) );
}

/*
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode?
 * @param   uiOutputs  - display passcode?
 * @param   numComparison - numeric comparison value
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                       uint8_t uiInputs, uint8_t uiOutputs, uint32 numComparison)
{
  passcode_req_t req =
  {
    .connHandle = connHandle,
    .uiInputs = uiInputs,
    .uiOutputs = uiOutputs,
    .numComparison = numComparison
  };

  // Defer handling of the passcode request to the application, in case
  // user input is required, and because a BLE API must be used from Task.
  user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *)&req, sizeof(req));
}

/*
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB(uint16_t connHandle, uint8_t state,
                                        uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {
    Log_info0("Pairing started");
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      Log_info0("Pairing completed successfully.");
    }
    else
    {
      Log_error1("Pairing failed. Error: %02x", status);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
     Log_info0("Re-established pairing from stored bond info.");
    }
  }
}

/**
 * Callback handler for characteristic value changes in services.
 */
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len )
{
  // See the service header file to compare paramID with characteristic.
  Log_info2("(CB) Characteristic value change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
                          pValue, len);
}

/**
 * Callback handler for characteristic configuration changes in services.
 */
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len )
{
  Log_info2("(CB) Char config change: svc(0x%04x) paramID(%d). "
            "Sending msg to app.", (IArg)svcUuid, (IArg)paramID);
  user_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid,
                          paramID, pValue, len);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/


/*
 *  Callbacks from Hwi-context
 *****************************************************************************/


/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief  Generic message constructor for characteristic data.
 *
 *         Sends a message to the application for handling in Task context where
 *         the message payload is a char_data_t struct.
 *
 *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
 *         APP_MSG_SERVICE_CFG, and functions running in another context than
 *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
 *         make the user Task loop invoke user_updateCharVal function for them.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @param  connHandle    GAP Connection handle of the relevant connection
 * @param  serviceUUID   16-bit part of the relevant service UUID
 * @param  paramID       Index of the characteristic in the service
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
                                     uint16_t connHandle,
                                     uint16_t serviceUUID, uint8_t paramID,
                                     uint8_t *pValue, uint16_t len )
{
  // Called in Stack's Task context, so can't do processing here.
  // Send message to application message queue about received data.
  uint16_t readLen = len; // How much data was written to the attribute

  // Allocate memory for the message.
  // Note: The pCharData message doesn't have to contain the data itself, as
  //       that's stored in a variable in the service implementation.
  //
  //       However, to prevent data loss if a new value is received before the
  //       service's container is read out via the GetParameter API is called,
  //       we copy the characteristic's data now.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + sizeof(char_data_t) +
                                  readLen );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    char_data_t *pCharData = (char_data_t *)pMsg->pdu;
    pCharData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
    pCharData->paramID = paramID;
    // Copy data from service now.
    memcpy(pCharData->data, pValue, readLen);
    // Update pCharData with how much data we received.
    pCharData->dataLen = readLen;
    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(sem);
  }
}

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData,
                                  uint16_t len)
{
  // Allocate memory for the message.
  app_msg_t *pMsg = ICall_malloc( sizeof(app_msg_t) + len );

  if (pMsg != NULL)
  {
    pMsg->type = appMsgType;

    // Copy data into message
    memcpy(pMsg->pdu, pData, len);

    // Enqueue the message using pointer to queue node element.
    Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
    // Let application know there's a message.
    Semaphore_post(sem);
  }
}


/*
 * @brief  Convenience function for updating characteristic data via char_data_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void user_updateCharVal(char_data_t *pCharData)
{
  bStatus_t (*setParamFxn)(uint8_t, uint16_t, void*) = NULL;
  switch(pCharData->svcUUID) {
  case    ADC_SERVICE_SERV_UUID: setParamFxn = AdcService_SetParameter;    break;
  case RANGER_SERVICE_SERV_UUID: setParamFxn = RangerService_SetParameter; break;
  }

  if (setParamFxn != NULL) {
    setParamFxn(pCharData->paramID, pCharData->dataLen, pCharData->data);
  }
}


/*
 * @brief
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param
 */
void user_toggleLED(uint8_t n)
{
  static uint8_t toggle0 = 0, toggle1 = 0;
  switch(n) {
  case 0: PIN_setOutputValue(ledPinHandle, Board_LED0, (toggle0 ^= 1)); break;
  case 1: PIN_setOutputValue(ledPinHandle, Board_LED1, (toggle1 ^= 1)); break;
  }
}


/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr(const uint8_t *data) {
  uint8_t nuggetLen = 0;
  uint8_t nuggetType = 0;
  uint8_t advIdx = 0;

  static char localNameStr[32] = { 0 };
  memset(localNameStr, 0, sizeof(localNameStr));

  for (advIdx = 0; advIdx < 32;) {
    nuggetLen = data[advIdx++];
    nuggetType = data[advIdx];
    if ( (nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE ||
          nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31) {
      memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
      break;
    } else {
      advIdx += nuggetLen;
    }
  }

  return localNameStr;
}

/*********************************************************************
*********************************************************************/
