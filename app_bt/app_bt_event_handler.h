/*******************************************************************************
 * File Name: app_bt_event_handler.h
 *
 * Description:
 * This file contains the interfaces for application procedure to handle the
 * Bluetooth events in Bluetooth Management callback. The Bluetooth Management
 * callback acts like a Finite State Machine (FSM) for the SoC.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/
#ifndef __APP_BT_TASK_H__
#define __APP_BT_TASK_H__

/*******************************************************************************
 *                               Includes
 *******************************************************************************/

#include "cycfg_gatt_db.h"
#include "cycfg_gap.h"
#include "cycfg_bt_settings.h"
#include "cyabs_rtos.h"

#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"

#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_l2c.h"

#include <task.h>
#include "timers.h"

/* Bitflags for LE secure pairing keys IO capabilities event */
#define PAIRING_CAPS_KEYS_FLAG      \
        (BTM_LE_KEY_PENC | BTM_LE_KEY_PID)
/* Key Size for LE secure pairing key IO capabilities event */
#define PAIRING_CAPS_KEY_SIZE       (16u)

/*******************************************************************************
 *                           Global Variables
 *******************************************************************************/
/* Status variable for connection ID */
extern uint16_t app_bt_conn_id;

/*******************************************************************************
 *                           Function Prototypes
 *******************************************************************************/
/* Callback function for Bluetooth stack management type events */
wiced_bt_dev_status_t
app_bt_event_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data);

#endif // __APP_BT_TASK_H__