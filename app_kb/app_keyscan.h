/*******************************************************************************
* File Name: app_keyscan.h
*
* Description: This file consists of the function definitions that are
*              necessary for developing Keyscan/keyboard use cases.
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

#ifndef __APP_KEYSCAN_H_
#define __APP_KEYSCAN_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "cy_keyscan.h"
#include "cy_sysint.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include "cycfg_peripherals.h"
#include "app_hid_codes.h"
#include "app_config.h"

#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"


/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
#define TICKS_TO_WAIT           ((uint8_t)10)
#define MAX_KEYS                ((uint8_t)78)   

/* key press and release flags */
#define KEY_PRESSED                             ((uint8_t)0u)
#define KEY_RELEASED                            ((uint8_t)1u)

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/* Keyscan Keycode value and press/released flag */
struct ks_code
{
    uint8_t up_down_flag;
    uint16_t key_code;    
};

/* HID Message type : Keyscan Message or Battery level */
union msg
{
    struct ks_code ks;
    uint8_t batt_level;
};

/* HID Message and HID Message type */
struct hid_rpt_msg
{
    uint8_t msg_type;
    uint8_t send_msg;
    uint16_t report_type;
    union msg data;
};

/* enum for keyscan status */
typedef enum app_keyscan_status_t
{
    KEYSCAN_SUCCESS = 0,
    KEYSCAN_FAILURE
}app_keyscan_status_t;

/* task handle for the keyscan task */
extern TaskHandle_t keyscan_task_h;

/* variable used for deepsleep call back function */
extern uint8_t key_state_cnt;

/* Timer handle for pairing mode switch */
extern TimerHandle_t pairing_mode_timer;

extern uint8_t app_ghost_detected;  

extern uint8_t deepsleep_hold;

extern uint8_t fn_key_hold;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

void keyscan_task(void *args);

void app_send_kbd_report();

void app_send_cc_report(uint16_t report_type);

cy_en_syspm_status_t syspm_ks_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                    cy_en_syspm_callback_mode_t mode);

bool channel_select_button_pressed(void);

void app_reset_key_cnt(void);

void app_peer_capabilities_ble_request(wiced_bt_management_evt_data_t *p_event_data);

void app_peer_passkey_request(wiced_bt_management_evt_data_t *p_event_data);

#endif
