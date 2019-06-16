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

#ifndef PROJECTZERO_H
#define PROJECTZERO_H


/*********************************************************************
 * INCLUDES
 */

#include <xdc/std.h>

#include <ti/sysbios/knl/Queue.h>


/*********************************************************************
*  EXTERNAL VARIABLES
*/


/*********************************************************************
 * TYPEDEFS
 */

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written     */
  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed  */
  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value    */
  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed      */
  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing */
  APP_MSG_SC_TASK_ALERT,       /* Sensor Controller generated Task Alert      */
  APP_MSG_SC_CTRL_READY,       /* Sensor Controller generated Ctrl Ready      */
  APP_MSG_SC_EXEC_RANGER,      /* Sensor Controller execute ranger task once  */
} app_msg_types_t;

// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;


/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * MACROS
 */

 // Bitvector macro
#define BV(n)               (1 << (n))

// Clock tick conversion
#define TICK_TO_MS(tick)    ((tick) * Clock_tickPeriod / 1000)
#define MS_TO_TICK(ms)      ((ms) * 1000 / Clock_tickPeriod)


/*********************************************************************
 * FUNCTIONS
 */

// Task creation function for the Simple BLE Peripheral.
void ProjectZero_createTask(void);
void user_enqueueRawAppMsg(app_msg_types_t appMsgType, uint8_t *pData, uint16_t len );
void user_enqueueCharDataMsg(app_msg_types_t appMsgType, uint16_t connHandle,
                             uint16_t serviceUUID, uint8_t paramID,
                             uint8_t *pValue, uint16_t len);
void user_toggleLED(uint8_t n);

// SC Task
void SC_init(void);
void SC_processCtrlReady(void);
void SC_processTaskAlert(void);
void SC_execRanger(void);


/*********************************************************************
*********************************************************************/

#endif /* PROJECTZERO_H */
