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

#include <xdc/std.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file
#include <xdc/runtime/Log.h>

#include <ti/sysbios/knl/Clock.h>

#include "custom_fmt.h"

// Sensor Controller Interface
#include "scif.h"

// BLuetooth Developer Studio
#include "adc_service.h"
#include "ranger_service.h"

#include "project_zero.h"


/*********************************************************************
 * GLOBAL VARIABLES
 */

// Ranger task Clock struct and last tick value
static uint32_t     g_rangerLastTick = 0;
static Clock_Struct g_rangerClock;

/*********************************************************************
 * LOCAL FUNCTION DECLARATIONS
 */
// HWI
static void SC_ctrlReadyHwiCb(void);
static void SC_taskAlertHwiCb(void);

// SWI
static void SC_rangerClockSwiFxn(UArg a0);

// TASK
static void SC_processAdc(void);
static void SC_processRanger(void);


/*********************************************************************
 * HWI CALLBACKS
 */

/*
 * @brief   Callback from Scif driver on Control READY interrupt.
 *
 *          Signals main task with empty msg APP_MSG_SC_CTRL_READY.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SC_ctrlReadyHwiCb(void)
{
    // Signal main loop
    user_enqueueRawAppMsg(APP_MSG_SC_CTRL_READY, NULL, 0);
} // SC_ctrlReadyHwiCb


/*
 * @brief   Callback from Scif driver on Task ALERT interrupt.
 *
 *          Signals main task with empty msg APP_MSG_SC_TASK_ALERT.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SC_taskAlertHwiCb(void)
{
    // Signal main loop
    user_enqueueRawAppMsg(APP_MSG_SC_TASK_ALERT, NULL, 0);
} // SC_taskAlertHwiCb


/*********************************************************************
 * SWI CALLBACKS
 */

/*
 * @brief   Callback from Clock module on timeout.
 *
 *          Stores current clock tick, and signals main task with
 *          empty msg APP_MSG_SC_EXEC_RANGER.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SC_rangerClockSwiFxn(UArg a0)
{
    // Store current tick
    g_rangerLastTick = Clock_getTicks();

    // Signal main loop
    user_enqueueRawAppMsg(APP_MSG_SC_EXEC_RANGER, NULL, 0);
} // SC_rangerClockSwiFxn


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*
 * @brief   Processing function for the ADC SC task.
 *
 *          Is called whenever the APP_MSG_SC_TASK_ALERT msg is sent
 *          and ADC SC task has generated an alert.
 *
 *          Retrieves ADC value from SC, and calculates and sets new period and
 *          timeout for Clock object.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SC_processAdc(void)
{
    // Retrieve ADC value and saturate at 3000
    uint16_t adcValue = scifTaskData.adc.output.adcValue;
    adcValue = (adcValue > 3000) ? 3000 : adcValue;

    // Calculate new Ranger period
    #define MAXVAL 1000
    #define MINVAL 333
    uint32_t newRangerPeriod_ms = (MAXVAL - MINVAL) * adcValue / 3000 + MINVAL;

    uint32_t currRangerExec_ms = TICK_TO_MS(Clock_getTicks() - g_rangerLastTick);
    uint32_t newTimeout_ms = (currRangerExec_ms < newRangerPeriod_ms)
                        ? newRangerPeriod_ms - currRangerExec_ms
                        : 0;

    // Set new clock period and timeout
    Clock_Handle hRangerClock = Clock_handle(&g_rangerClock);
    Clock_stop(hRangerClock);
    Clock_setPeriod(hRangerClock, MS_TO_TICK(newRangerPeriod_ms));
    Clock_setTimeout(hRangerClock, MS_TO_TICK(newTimeout_ms));
    Clock_start(hRangerClock);

    // Notify the change to the BLE service
    char pLine[20];
    itoaAppendStr(pLine, newRangerPeriod_ms, "ms");
    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0,
                            ADC_SERVICE_SERV_UUID, AS_VALUE_ID,
                            (uint8_t *)pLine, strlen(pLine));

    // Toggle LED0 (Red) on LP
    user_toggleLED(0);
} // SC_processAdc


/*
 * @brief   Processing function for the Ranger SC task.
 *
 *          Is called whenever the APP_MSG_SC_TASK_ALERT msg is sent
 *          and Ranger SC task has generated an alert.
 *
 *          Retrieves both high and low tdc values from SC, extends into 32-bit,
 *          converts into cm, and prints out the value on Log.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SC_processRanger(void)
{
    // Convert readings to distance measured and print it
//    uint16_t tdcValueL = scifTaskData.ranger.output.tdcValueL;
//    uint16_t tdcValueH = scifTaskData.ranger.output.tdcValueH;
//    uint32_t tdcValue = (tdcValueH << 16) | tdcValueL; // Build TDC value

//    uint32_t tof_us = tdcValue / 48; // Convert from TDC value to time of flight [us]
//    uint32_t rangerValue_cm = tof_us * 429 / 25000; // Convert from time of flight [us] to distance [cm]

    // Notify the change to the BLE service
//    char pLine[20];
//    itoaAppendStr(pLine, rangerValue_cm, "cm");
//    user_enqueueCharDataMsg(APP_MSG_UPDATE_CHARVAL, 0,
//                            RANGER_SERVICE_SERV_UUID, RS_DISTANCE_ID,
//                            (uint8_t *)pLine, strlen(pLine));

    // Toggle LED1 (Green) on LP
//    user_toggleLED(1);
} // SC_processRanger


/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*
 * @brief   Called before main loop.
 *
 *          Initializes Scif driver, registers callbacks, configures RTC, and
 *          starts SC tasks.
 *
 * @param   None.
 *
 * @return  None.
 */
void SC_init(void)
{
    // Insert default params
    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
    // Set period to 0 ms
    clockParams.period = 0;
    // Initialize the clock object / Clock_Struct previously added globally.
    Clock_construct(&g_rangerClock,        // global clock struct
                    SC_rangerClockSwiFxn,  // callback from clock
                    0,                     // Initial delay before first timeout
                    &clockParams);         // clock parameters

    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(SC_ctrlReadyHwiCb);
    scifOsalRegisterTaskAlertCallback(SC_taskAlertHwiCb);
    scifInit(&scifDriverSetup);

    uint32_t rtcHz = 3; // 3Hz RTC
    scifStartRtcTicksNow(0x00010000 / rtcHz);

    // Configure SC Tasks here, if any

    // Start Sensor Controller
    scifStartTasksNbl(BV(SCIF_ADC_TASK_ID));

    Log_info0("scTask initialization done");
} // SC_init


/*
 * @brief   Processing function for the APP_MSG_SC_CTRL_READY event.
 *
 *          Is called from main loop whenever the APP_MSG_SC_CTRL_READY msg is
 *          sent.
 *
 *          Currently does nothing.
 *
 * @param   None.
 *
 * @return  None.
 */
void SC_processCtrlReady(void)
{
  // Do nothing

} // SC_processCtrlReady


/*
 * @brief   Processing function for the APP_MSG_SC_TASK_ALERT event.
 *
 *          Is called from main loop whenever the APP_MSG_SC_TASK_ALERT msg is
 *          sent.
 *
 *          Checks which SC tasks are active, and calls their corresponding
 *          processing function. Also clears and ACKs the interrupts to the
 *          Scif driver.
 *
 * @param   None.
 *
 * @return  None.
 */
void SC_processTaskAlert(void)
{
    // Clear the ALERT interrupt source
    scifClearAlertIntSource();

    // Do SC Task processing here
    // Get the alert events
    uint32_t bvAlertEvents = scifGetAlertEvents();

    // Check which task called and do process
    if (bvAlertEvents & BV(SCIF_ADC_TASK_ID)) {
        SC_processAdc();
    }
//    if (bvAlertEvents & BV(SCIF_RANGER_TASK_ID)) {
//        SC_processRanger();
//    }

    // Acknowledge the ALERT event
    scifAckAlertEvents();
} // SC_processTaskAlert


/*
 * @brief   Processing function for the APP_MSG_SC_EXEC_RANGER event.
 *
 *          Is called from main loop whenever the APP_MSG_SC_EXEC_RANGER msg is
 *          sent.
 *
 *          Executes the Ranger SC task once.
 *
 * @param   None.
 *
 * @return  None.
 */
void SC_execRanger(void)
{
    // Check Ranger Task is not already active
    uint16_t bvActive = scifGetActiveTaskIds();
//    if (bvActive & BV(SCIF_RANGER_TASK_ID)) {
//        return;
//    }
    // Execute Task once
//    scifResetTaskStructs(BV(SCIF_RANGER_TASK_ID), 0);
//    scifExecuteTasksOnceNbl(BV(SCIF_RANGER_TASK_ID));
} // SC_execRanger
