/*******************************************************************************
 * File Name: ble_hid.c
 *
 * Description:
 * This file contains the BLE HID task to send reports on keyscan and
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

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "app_bt_hid.h"
#include "app_bt_advert.h"
#include "app_bt_bonding.h"
#include "app_bt_event_handler.h"
#include "app_handler.h"
#include "app_config.h"
#include "app_batmon.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
#define MAX_SIMULTANEOUS_KEYS           ((uint8_t)6)
#define KBD_REPORT_SIZE                 ((uint8_t)8)
#define KBD_REPORT_BUFF_SIZE            ((uint8_t)10)
#define CC_REPORT_BUFF_SIZE             ((uint8_t)5)
#define CC_REPORT_SIZE                  ((uint8_t)1)
#define SEND_BT_HID_NOTIFICATION


/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

/* Task handle of Bluetooth LE HID keybord Application  */
TaskHandle_t ble_task_h;

/* Queue for sending KS events and battery reports to Bluetooth LE Task */
QueueHandle_t hid_rpt_q;

/* Timer handle for triggering connection parameter update */
TimerHandle_t conn_param_update_timer;

/* Timer handle for stopping advertisement */
TimerHandle_t adv_stop_timer;

/* buffer to store hid kbd codes to be sent to host */
uint8_t ble_hid_kbd_buffer[MAX_SIMULTANEOUS_KEYS] = {0};

/* buffer to store hid cc codes to be sent to host */
uint8_t ble_hid_cc_buffer[CC_REPORT_SIZE] = {0};

/* variable to store the current index of ble_hid_kbd_buffer */
uint8_t ble_hid_kbd_buffer_index = 0;

/* variable to store the bit position of modifier keys e.g CTL/ALT/SHIT etc */
uint8_t ble_hid_kbd_modifier = 0;

/* buffer used to send kbd report to bt stack */
uint8_t app_hid_kbd_report[KBD_REPORT_BUFF_SIZE][KBD_REPORT_SIZE];

/* buffer used to send cc report to bt stack */
uint8_t app_hid_cc_report[CC_REPORT_BUFF_SIZE][CC_REPORT_SIZE];

/* peer address to be used for passkey authentication */
extern wiced_bt_device_address_t peer_bd_addr;

/* Current Application state */
app_ble_state_t current_app_state = UNPAIRED_ON;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_bt_keyscan_activity_handler(uint16_t hid_code,
                                            uint8_t up_down_flag,
                                            uint8_t msg_type,
                                            uint16_t report_type,
                                            uint8_t send_msg);

static void app_bt_delete_from_ble_hid_kbd_buffer(uint8_t *buf, uint8_t hid_code);

static void app_bt_passkey_authentication(uint16_t hid_code, uint8_t up_down_flag);

static void app_bt_adv_stop_timer_cb(TimerHandle_t cb_params);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/**
 *  Function name:
 *  app_bt_hid_get_device_state
 *
 *  Function Description:
 *  @brief Function to get the current device state
 *
 *  @param    void
 *
 *  @return   app_ble_state_t: Current BLE state of the application
 */
inline app_ble_state_t app_bt_hid_get_device_state(void)
{
    return current_app_state;
}

/**
 *  Function name:
 *  app_bt_hid_update_device_state
 *
 *  Function Description:
 *  @brief Function to update the current device state
 *
 *  @param    app_ble_state_t: Current BLE state of the application
 *
 *  @return   void
 */
inline void app_bt_hid_update_device_state(app_ble_state_t app_state)
{
    current_app_state = app_state;
}

/*******************************************************************************
 * Function Name: app_bt_passkey_authentication
 ********************************************************************************
 * Summary:
 * Handles the passkey authentication
 *
 * Parameters:
 *  hid_code          : hid code sent from keyscan task
 *  up_down_flag      : indicates key press or release , press = 0 , release = 1
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_bt_passkey_authentication(uint16_t hid_code, uint8_t up_down_flag)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    /* variables to verify pass key authentication */
    static uint32_t passkey = 0;
    static uint32_t pow_result = 1;
    static int8_t count = 5;
    static uint8_t power = 5;

    /* Send Pass Key to the Host device during pairing authentication */
    if (up_down_flag == KEY_PRESSED)
    {
        key_state_cnt = 1;
        /*handle backspace for incorrect key press */
        if (hid_code == HID_BACKSPACE)
        {
            if (count < 5)
            {
                passkey = passkey / pow_result;
                passkey = passkey / 10;
                passkey = passkey * pow_result * 10;
                pow_result = pow_result * 10;
                count++;
            }
        }
        /* process only numbers 0 to 9, since pass key is always numbers*/
        if ((hid_code >= HID_1) && (hid_code <= HID_0))
        {
            /* converting the hid code to equivalent numbers from 0 to 9 */
            hid_code = hid_code - 29;
            if (hid_code == 10)
            {
                hid_code = 0;
            }
            if (count >= 0)
            {
                /* generate 6 digit number from individual numbers */
                power = count;
                pow_result = 1;
                while (power != 0)
                {
                    pow_result *= 10;
                    --power;
                }
                passkey = passkey + (pow_result * hid_code);
                count--;
            }
        }
        if (hid_code == HID_ENTER)
        {
            /* send pass key to bt stack */
            wiced_bt_dev_pass_key_req_reply(result, peer_bd_addr, passkey);
            /* reset variables */
            count = 5;
            pow_result = 1;
            passkey = 0;
            key_state_cnt = 0;
        }
    }
}

/*******************************************************************************
 * Function Name: app_bt_hid_update_buffer
 ********************************************************************************
 * Summary:
 *  adds or removes hid code from the hid buffers for standard keyboard hid codes
 *
 * Parameters:
 *  hid_code          : hid code sent from keyscan task
 *  up_down_flag      : indicates key press or release , press = 0 , release = 1
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_bt_hid_update_buffer(uint8_t hid_code, uint8_t u8updownflag)
{
    uint8_t index = 0;
    bool hid_already_present = false;

    if (hid_code != 0xFF)
    {
        if (u8updownflag == KEY_PRESSED)
        {
            if ((hid_code >= HID_LEFTCTRL) && (hid_code <= HID_RIGHTMETA))
            {
                /* set the bit position for modifier keys */
                ble_hid_kbd_modifier |= ((uint8_t)1 << (hid_code & 0x0F));
            }
            else
            {
                /* update the kbd buffer */
                if (ble_hid_kbd_buffer_index < MAX_SIMULTANEOUS_KEYS)
                {
                    /* check if hid code is already present */
                    for (index = 0; index < MAX_SIMULTANEOUS_KEYS; index++)
                    {
                        if (ble_hid_kbd_buffer[index] == hid_code)
                        {
                            hid_already_present = true;
                            break;
                        }
                    }
                    /* add element to buffer only if hid code is not present already */
                    if (hid_already_present == false)
                    {
                        ble_hid_kbd_buffer[ble_hid_kbd_buffer_index] = hid_code;
                        ble_hid_kbd_buffer_index = (ble_hid_kbd_buffer_index + 1);
                    }
                }
            }
        }
        else
        {
            /* its a key release event , so remove the hid code */
            if ((hid_code >= HID_LEFTCTRL) && (hid_code <= HID_RIGHTMETA))
            {
                /* reset the corresponding modifier bit */
                ble_hid_kbd_modifier &= ~((uint8_t)1 << (hid_code & 0x0F));
            }
            else
            {
                /* delete hid code from the buffer */
                app_bt_delete_from_ble_hid_kbd_buffer(&ble_hid_kbd_buffer[0], hid_code);
            }
        }
    }
#ifdef DEBUG_ENABLE_LED_ON_KEYPRESS
    if (ble_hid_kbd_buffer_index > 0)
    {
        cyhal_gpio_write(STATUS_LED, CYBSP_LED_STATE_ON);
    }
    else
    {
        cyhal_gpio_write(STATUS_LED, CYBSP_LED_STATE_OFF);
    }
#endif
}

/*******************************************************************************
 * Function Name: app_bt_delete_from_ble_hid_kbd_buffer
 ********************************************************************************
 * Summary:
 *  removes hid code from the hid buffers for standard keyboard hid codes
 *
 * Parameters:
 *  buf              : pointer to hid code buffer
 *  hid_code         : hid code to be removed
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_bt_delete_from_ble_hid_kbd_buffer(uint8_t *buf, uint8_t hid_code)
{
    uint8_t index = 0;
    uint8_t position = 0;
    bool key_found = false;

    /* check if hid code is available in buffer */
    for (index = 0; index < ble_hid_kbd_buffer_index; index++)
    {
        if (buf[index] == hid_code)
        {
            key_found = true;
            break;
        }
        position++;
    }

    /* remove if hid code is available in the buffer */
    if (key_found)
    {
        for (index = position; index < (ble_hid_kbd_buffer_index - 1); index++)
        {
            buf[index] = buf[index + 1];
        }

        if (ble_hid_kbd_buffer_index)
        {
            ble_hid_kbd_buffer_index--;
        }
        buf[ble_hid_kbd_buffer_index] = 0;
    }
}

/*******************************************************************************
 * Function Name: app_handle_battery_low
 ********************************************************************************
 * Summary:
 *  Function to handle battery low condition
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void app_handle_battery_low(void)
{
    /* If connected, Disconnect from current host */
    if (app_bt_conn_id != 0)
    {
        (void)wiced_bt_gatt_disconnect(app_bt_conn_id);
    }

    /* Stop ongoing adv if any */
    app_bt_adv_stop();

    /* LED Blink indication before Hibernation */
    app_led_update_blink_period(LED_BLINK_RATE_MS);

    /* wait for 2 seconds */
    vTaskDelay(2000);

    /* Turn OFF LED */
    app_status_led_off();

    /* Enter Hibernation state */
    Cy_SysPm_SystemEnterHibernate();
}

/*******************************************************************************
 * Function Name: app_conn_param_update_timer_cb
 ********************************************************************************
 * Summary:
 *  Timer cb to trigger connection parameter update
 *
 * Parameters:
 *  cb_param: Argument to cb
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void app_conn_param_update_timer_cb(TimerHandle_t cb_params)
{
    if ((app_bt_conn_id != 0) &&
        (conn_param_updated_flag == FALSE))
    {
        wiced_bool_t conn_update_status = 0;

        printf("sending conn param request\r\n");
        conn_update_status = wiced_bt_l2cap_update_ble_conn_params(
            peer_bd_addr,
            MIN_CI,
            MAX_CI,
            SLAVE_LATENCY,
            SUPERVISION_TO);

        if (0 != conn_update_status)
        {
            printf("Connection parameter update successful\r\n");
        }

        /* Trigger Connection Param Update request from HID device after a delay */
        if (pdFAIL == xTimerStart(conn_param_update_timer, TIMER_MIN_WAIT))
        {
            printf("Failed to start connection parameter update Timer\r\n");
        }
    }
}

/**
 *  Function name:
 *  app_bt_adv_stop_timer_cb
 *
 *  Function Description:
 *  @brief Timer cb to stop ongoing BLE adv.
 *
 *  @param    cb_param: Argument to cb
 *
 *  @return   void
 */
static void app_bt_adv_stop_timer_cb(TimerHandle_t cb_params)
{
    /* Stop ongoing adv if any */
    app_bt_adv_stop();
}

/**
 *  Function name:
 *  app_send_batt_report
 *
 *  Function Description:
 *  @brief Function to send battery level percentage
 *
 *  @param    uint8_t: battery_percentage
 *
 *  @return   void
 */
void app_send_batt_report(uint8_t battery_percentage)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    app_bas_battery_level[0] = battery_percentage;

    /* Check the cccd */
    if (app_bas_battery_level_client_char_config[0] == GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        /* send the Battery HID report data */
        gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                             HDLC_BAS_BATTERY_LEVEL_VALUE,
                                                             app_bas_battery_level_len,
                                                             app_bas_battery_level,
                                                             NULL);
        if (WICED_BT_GATT_SUCCESS == gatt_status)
        {
            printf("Battery level Notification sent\r\n\r\n");
        }
    }
}

/*******************************************************************************
 * Function Name: app_send_kbd_report
 ********************************************************************************
 * Summary:
 *  send keyboard report to bt stack
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_send_kbd_report()
{
    wiced_bt_gatt_status_t gatt_status;
    uint32_t ulNotifiedValue;
    static uint8_t rpt_buf_idx = 0;

    if (app_hids_kbd_in_report_client_char_config[0] == 0x01)
    {
        app_hid_kbd_report[rpt_buf_idx][0] = ble_hid_kbd_modifier;
        app_hid_kbd_report[rpt_buf_idx][1] = 0;
        app_hid_kbd_report[rpt_buf_idx][2] = ble_hid_kbd_buffer[0];
        app_hid_kbd_report[rpt_buf_idx][3] = ble_hid_kbd_buffer[1];
        app_hid_kbd_report[rpt_buf_idx][4] = ble_hid_kbd_buffer[2];
        app_hid_kbd_report[rpt_buf_idx][5] = ble_hid_kbd_buffer[3];
        app_hid_kbd_report[rpt_buf_idx][6] = ble_hid_kbd_buffer[4];
        app_hid_kbd_report[rpt_buf_idx][7] = ble_hid_kbd_buffer[5];

#ifdef SEND_BT_HID_NOTIFICATION
        gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                             HDLC_HIDS_KBD_IN_REPORT_VALUE,
                                                             app_hids_kbd_in_report_len,
                                                             app_hid_kbd_report[rpt_buf_idx],
                                                             NULL);
        if (gatt_status != WICED_BT_GATT_SUCCESS)
        {
            xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_HIDS_KBD_IN_REPORT_VALUE,
                                                                 app_hids_kbd_in_report_len,
                                                                 app_hid_kbd_report[rpt_buf_idx],
                                                                 NULL);
        }
        rpt_buf_idx = ((rpt_buf_idx + 1) % KBD_REPORT_BUFF_SIZE);
#endif
    }
}

/*******************************************************************************
 * Function Name: app_send_cc_report
 ********************************************************************************
 * Summary:
 *  send cc report to bt stack
 *
 * Parameters:
 *  pvParameters        : default parameter of freertos
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_send_cc_report(uint16_t report_type)
{
    uint32_t ulNotifiedValue;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    static uint8_t rpt_buf_cc_idx = 0;

    if (app_hids_cc_in_report_client_char_config[0] == 0x01)
    {
        app_hids_cc_in_report[0] = ble_hid_cc_buffer[0];
        app_hid_cc_report[rpt_buf_cc_idx][0] = app_hids_cc_in_report[0];

#ifdef SEND_BT_HID_NOTIFICATION
        gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                             report_type,
                                                             app_hids_cc_in_report_len,
                                                             app_hid_cc_report[rpt_buf_cc_idx],
                                                             NULL);
        if (gatt_status != WICED_BT_GATT_SUCCESS)
        {
            xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 report_type,
                                                                 app_hids_cc_in_report_len,
                                                                 app_hid_cc_report[rpt_buf_cc_idx],
                                                                 NULL);
        }
        rpt_buf_cc_idx = ((rpt_buf_cc_idx + 1) % 5);
#endif
        if (WICED_BT_GATT_SUCCESS == gatt_status)
        {
            printf("CC Notification sent\n");
        }
    }
}

/*******************************************************************************
 * Function Name: app_clear_ble_hid_kbd_buffer
 ********************************************************************************
 * Summary:
 *  clear the ble_hid_kbd_buffer. This is useful when ghost key is detected
 *
 * Parameters:
 *  pvParameters        : default parameter of freertos
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_clear_ble_hid_kbd_buffer(void)
{
    ble_hid_kbd_buffer[0] = 0;
    ble_hid_kbd_buffer[1] = 0;
    ble_hid_kbd_buffer[2] = 0;
    ble_hid_kbd_buffer[3] = 0;
    ble_hid_kbd_buffer[4] = 0;
    ble_hid_kbd_buffer[5] = 0;
    ble_hid_kbd_buffer_index = 0;
}

/*******************************************************************************
 * Function Name: app_clear_hid_cc_buffer
 ********************************************************************************
 * Summary:
 *  clear data in the cc buffer
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_clear_hid_cc_buffer(void)
{
    ble_hid_cc_buffer[0] = 0;
}

/*******************************************************************************
 * Function Name: app_bt_keyscan_activity_handler
 ********************************************************************************
 * Summary:
 * Handles the keycodes and sends to bt stack , processes pass key authentication
 * starts advertisements based on key press
 *
 * Parameters:
 *  hid_code          : hid code sent from keyscan task
 *  up_down_flag      : indicates key press or release , press = 0 , release = 1
 *  msg_type          : consumer hid msg or standard keyboard msg
 *  report_type       : kbd report or cc report
 *  send_msg          : flag to send message to BT stack or not
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_bt_keyscan_activity_handler(uint16_t hid_code, uint8_t up_down_flag, uint8_t msg_type, uint16_t report_type, uint8_t send_msg)
{
    switch (current_app_state)
    {
        case PAIRED_ON:
            /* Pairing Succesful, GATT connection is not yet done */
            printf("Device is in PAIRED_ON state\r\n");
            /* Start advertisements */
            app_bt_adv_start();
            break;

        case UNPAIRED_ADVERTISING:
            /* App is already advertising */
            printf("Device is in UNPAIRED_ADVERTISING state\r\n");
            break;

        case PAIRED_ADVERTISING_KNOWN_HOST:
            /* App is already advertising */
            printf("Device is in PAIRED_ADVERTISING_KNOWN_HOST state\r\n");
            break;

        case PAIRED_ADVERTISING_ANY_HOST:
            /* App is already advertising */
            printf("Device is in PAIRED_ADVERTISING_ANY_HOST state\r\n");
            break;

        case UNPAIRED_IDLE:
            /* When woke up from sleep */
            printf("Device is in UNPAIRED_IDLE state\r\n");
            /* Start advertisements to any host */
            app_bt_adv_start();
            break;

        case PAIRED_IDLE:
            /* When woke up from sleep */
            printf("Device is in PAIRED_IDLE state\r\n");
            /* Start advertisements to known host */
            app_bt_adv_start();
            break;

    case CONNECTED_NON_ADVERTISING:
        /* Device is paired and connected. Send data to connected device */
        printf("Device is in CONNECTED_NON_ADVERTISING state\r\n");
        if (msg_type == KS_MSG_TYPE)
        {
            /* add or remove hid code in the buffer */
            app_bt_hid_update_buffer(hid_code, up_down_flag);

            /* send ble message only if the send_msg flag is set to 1 */
            if (send_msg == 1)
            {
                /* send report to bt stack */
                app_send_kbd_report();
                /* update this flag for deep sleep transition */
                key_state_cnt = ble_hid_kbd_buffer_index;
                if (key_state_cnt == 0)
                {
                    /* make key_state_cnt as non zero , to avoid disabling MFO in deepsleep
                     * this is required because modifier keys are not tracked by 'ble_hid_kbd_buffer_index'
                     */
                    if (ble_hid_kbd_modifier != 0)
                    {
                        key_state_cnt = 1;
                    }
                }
                send_msg = 0;
            }
        }
        else if (msg_type == CC_MSG_TYPE)
        {
            /* its a CC message , update the buffer accordingly */
            if (up_down_flag == KEY_PRESSED)
            {
                ble_hid_cc_buffer[0] = hid_code & 0xFF;
            }
            else
            {
                ble_hid_cc_buffer[0] = 0;
            }
            /* send CC report to bt stack */
            app_send_cc_report(report_type);
        }
        break;

    case CONNECTED_ADVERTISING:
        /* When the device is put into pairing mode for a new device to
         * pair other than the connected device
         */
        printf("Device is in CONNECTED_ADVERTISING state\r\n");
        break;

    case UNPAIRED_ON:
        /* Device is ON but the stack/hardware is not yet initialized */
        printf("Device is in UNPAIRED_ON state\r\n");
        break;

    case PASS_KEY_REQUEST:
        app_bt_passkey_authentication(hid_code, up_down_flag);
        printf("Device is in Pass Key state\r\n");
        break;

    default:
        printf("ERROR: Unknown Remote state\r\rn");
    }
}

/*******************************************************************************
 * Function Name: app_ble_task
 ********************************************************************************
 * Summary:
 *  task to handle ble notifications
 *
 * Parameters:
 *  pvParameters        : default parameter of freertos
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_ble_task(void *pvParameters)
{
    struct hid_rpt_msg rpt_msg;
    BaseType_t xResult = pdFAIL;

    /* Create timer to trigger connection parameter update */
    conn_param_update_timer = xTimerCreate("Conn Param Update Timer",
                                           pdMS_TO_TICKS(CONN_PARAM_UPDATE_TIMER_DELAY),
                                           pdFALSE,
                                           NULL,
                                           app_conn_param_update_timer_cb);

    /* Initialize timer for directed adv */
    adv_stop_timer = xTimerCreate("Directed Adv Timer",
                                  pdMS_TO_TICKS(RECONNECTION_ADV_TIMEOUT_MS),
                                  pdFALSE,
                                  NULL,
                                  app_bt_adv_stop_timer_cb);
    while (TRUE)
    {
        /* Block until a command is received */
        xResult = xQueueReceive(hid_rpt_q, &(rpt_msg), portMAX_DELAY);
        if (xResult != pdPASS)
        {
            continue;
        }

        /* Msg from Keyscan task */
        if (rpt_msg.msg_type == KS_MSG_TYPE || rpt_msg.msg_type == CC_MSG_TYPE)
        {
            app_bt_keyscan_activity_handler(rpt_msg.data.ks.key_code, rpt_msg.data.ks.up_down_flag, rpt_msg.msg_type, rpt_msg.report_type, rpt_msg.send_msg);
        }

        /* Msg from Battery monitor task */
        if (rpt_msg.msg_type == BATT_MSG_TYPE)
        {
            app_send_batt_report(rpt_msg.data.batt_level);
        }

        /* Low battery handle */
        if (app_bas_battery_level[0] == 1)
        {
            /* clearing the buffers before entering hibernate */
            app_clear_ble_hid_kbd_buffer();
            app_clear_hid_cc_buffer();

            /* clear the notifications if already sent , buffers are cleared in the above step*/
            app_send_kbd_report();
            app_send_cc_report(HDLC_HIDS_CC_IN_REPORT_VALUE);

            /* goto hibernate */
            app_handle_battery_low();
        }
    }
}
