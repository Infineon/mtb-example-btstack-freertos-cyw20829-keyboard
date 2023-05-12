/*******************************************************************************
 * File Name: app_bt_hid.h
 *
 * Description:
 * This file contains the BLE HID task to send reports on Keyboard events and
 * battery change events to the connected HID Host.
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

#ifndef __APP_BT_HID_H__
#define __APP_BT_HID_H__

/*******************************************************************************
 *                                Include Headers
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "cy_utils.h"
#include "app_hid_codes.h"
#include "app_keyscan.h"

/*******************************************************************************
 *                                Constants
 ******************************************************************************/

/* Message type to differentiate Keyscan msg ,cc and Battery level msg */
#define KS_MSG_TYPE                             ((uint8_t)1u)
#define BATT_MSG_TYPE                           ((uint8_t)2u)
#define CC_MSG_TYPE                             ((uint8_t)3u)

/* hid queue size */
#define HID_MSG_Q_SZ                            ((uint8_t)100u)
/* hid queue item size */
#define HID_MSG_Q_ITEM_SZ                       (sizeof(struct hid_rpt_msg))

#define TASK_MAX_WAIT  1000 // Delay in ms for HID task notify wait

/*******************************************************************************
 *                          Variable Declarations
 ******************************************************************************/

/* State of the application */
typedef enum
{
    UNPAIRED_ON,                    /* Unpaired state when the Device is just powered up */
    PAIRED_ON,                      /* Paired state when the Device is just powered up */
    UNPAIRED_ADVERTISING,           /* Unpaired Undirected advertisement state in which the device is not bonded and making it discoverable for the hosts */
    PAIRED_ADVERTISING_KNOWN_HOST,  /* Paired Undirected advertisement state in which the device has the bond information to connect again to the host */
    PAIRED_ADVERTISING_ANY_HOST,    /* Paired Undirected advertisement state in which the device is bonded but making it discoverable for the other new hosts */
    UNPAIRED_IDLE,                  /* Unpaired Sleep state in which advertisement has timed out due to no interest from any new hosts */
    PAIRED_IDLE,                    /* Paired Sleep state in which advertisement has timed out due to no interest from any new hosts or connected host */
    CONNECTED_NON_ADVERTISING,      /* Paired Connected state in which the device is not interested in advertising */
    CONNECTED_ADVERTISING,          /* Paired Connected state in which the device is advertising to get paired to a new host */
    PASS_KEY_REQUEST,               /* State when user needs to enter authentication PIN */
} app_ble_state_t;
/* Task and Queue Handles of  Bluetooth LE HID Keyboard Application  */
extern TaskHandle_t ble_task_h;
/* Queue for sending KS events and battery reports to Bluetooth LE Task */
extern QueueHandle_t hid_rpt_q;
/* Current Application state */
extern app_ble_state_t current_app_state;

/*******************************************************************************
 *                          Function Declarations
 ******************************************************************************/

void app_ble_task(void* pvParameters);

void app_clear_ble_hid_kbd_buffer(void);

void app_clear_hid_cc_buffer(void);

void app_bt_hid_update_buffer(uint8_t u8HIDcode ,uint8_t u8updownflag );

app_ble_state_t app_bt_hid_get_device_state(void);

void app_bt_hid_update_device_state(app_ble_state_t app_state);

#endif 

/* __APP_BT_HID_H__ */
