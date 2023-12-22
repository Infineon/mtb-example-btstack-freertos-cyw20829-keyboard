/*******************************************************************************
 * File Name: app_keyscan.c
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

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "app_keyscan.h"
#include "app_bt_hid.h"
#include "app_bt_event_handler.h"
#include "app_bt_advert.h"
#include "app_handler.h"
#include "app_keyboard_types.h"
#include "app_config.h"

/*******************************************************************************
 *                              DEFINES
 ******************************************************************************/
/* MACROS to define the min and max keycodes expected */
#define KEY_CODE_MIN            ((uint8_t)(4))
#define KEY_CODE_MAX            ((uint8_t)(143))

#define SEND_BLE_MSG            ((uint8_t)(1))

#define OS_INVALID              ((uint8_t)(0xFF))
#define CHANNEL_INVALID         ((uint8_t)(0xFF))

/*******************************************************************************
 *                              GLOBAL VARIABLES
 ******************************************************************************/
/* Keyscan task handle */
TaskHandle_t keyscan_task_h;

/* Keyscan context */
cy_stc_keyscan_context_t context;

/* Flag to check if there are any events pending from keyscan */
bool events_pending;

/* Keep track of key state - used in deepsleep call back */
uint8_t key_state_cnt = 0;

/* Flag to check whether or not to disable MFO*/
uint8_t deepsleep_hold;

/* Flag to check whether or not to disable MFO based on Function key state*/
uint8_t fn_key_hold = 0;

/* Flags to check whether ghost/rollover key detection*/
uint8_t app_ghost_detected = 0;
uint8_t app_prev_ghost_detected = 0;

/* structure to hold ble hid data for transmit */
typedef struct tx_data
{
    bool send_tx_data;
    uint8_t key_code;
    uint8_t up_down_flag;
    uint8_t msg_type;
    uint8_t reportType;
    uint16_t hid_code;
} tx_data_t;
tx_data_t ble_tx_data_struct;

/* structure to hold data on special function keys */
spl_fun_struct_t function_keys_struct;

/* timer for ghost/rollover release */
TimerHandle_t keyscan_ghost_timer;

/* timer to disconnect device after DISCONNECTION_TIMEOUT_MS */
TimerHandle_t ble_disconnection_timer;

/* keyscan interrupt config */
const cy_stc_sysint_t keyscan_irq_cfg =
    {
        /* .intrSrc */ keyscan_interrupt_IRQn,
        /* .intrPriority */ 7UL};

/* WDT object */
extern cyhal_wdt_t wdt_obj;

/*******************************************************************************
 *                              ENUM
 ******************************************************************************/

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_ks_keyscan_intHandler(void);

static void app_ks_key_detected_callback(void);

static int app_ks_configure_keyscan(void);

static void app_ks_evt_notif_enable(uint32_t key_notify);

static void app_ks_send_msg_to_hid_msg_q(uint16_t key_code,
                                  uint8_t up_down_flag,
                                  uint8_t msg_type,
                                  uint16_t report_type,
                                  uint8_t send_msg);

static void app_ks_update_function_key(tx_data_t *ble_tx_data_struct);

static void app_ks_handle_function_keys(tx_data_t *ble_tx_data_struct);

static void app_ks_handle_standard_keys(tx_data_t *ble_tx_data_struct);

static app_keyscan_status_t app_keyscan_handler_init(void);

static app_keyscan_status_t app_keyscan_interrupt_init(void);

static uint16_t app_ks_keycode_to_hidcode(uint8_t map_table_size,
                                    KeycodeMap_t *key_code_table,
                                    uint8_t key_code);

static uint16_t app_ks_find_special_fn_hid_code(uint8_t map_table_size,
                                        Keycode_SplFn_Map_t *key_code_table,
                                        uint8_t key_code);

static void app_ks_idle_disconnection_timer_cb(TimerHandle_t cb_params);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * Function Name: app_ks_key_detected_callback
 ********************************************************************************
 * Summary:
 *  This is a capabilities request event. Set the IO capabilities of keyboard
 *
 * @param p_event_data:  Pointer to Bluetooth LE management event
 *                                        structures
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_peer_capabilities_ble_request(wiced_bt_management_evt_data_t *p_event_data)
{
    /* Set the IO capabilities */
    p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
            BTM_IO_CAPABILITIES_KEYBOARD_ONLY;
    p_event_data->pairing_io_capabilities_ble_request.oob_data =
            BTM_OOB_NONE;
    /* LE sec bonding */
    p_event_data->pairing_io_capabilities_ble_request.auth_req =
            (BTM_LE_AUTH_REQ_SC | BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM);
    p_event_data->pairing_io_capabilities_ble_request.init_keys =
            PAIRING_CAPS_KEYS_FLAG;
    p_event_data->pairing_io_capabilities_ble_request.resp_keys =
            PAIRING_CAPS_KEYS_FLAG;
    p_event_data->pairing_io_capabilities_ble_request.max_key_size =
            PAIRING_CAPS_KEY_SIZE;
}

/*******************************************************************************
 * Function Name: app_peer_passkey_request
 ********************************************************************************
 * Summary:
 *  This is passkey event. Copy the peer bd_addr to notify passkey to the peer.
 *
 * @param p_event_data:  Pointer to Bluetooth LE management event
 *                                        structures
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_peer_passkey_request(wiced_bt_management_evt_data_t *p_event_data)
{
    memcpy(peer_bd_addr , p_event_data->paired_device_link_keys_request.bd_addr , 6);
    app_bt_hid_update_device_state(PASS_KEY_REQUEST);
}

/*******************************************************************************
 * Function Name: app_ks_key_detected_callback
 ********************************************************************************
 * Summary:
 *  This is an interrupt callback which will be executed based on the
 *  Keyscan interrupt triggers.
 *
 * Parameters:
 *
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_ks_key_detected_callback(void)
{
    /* Send Event Notification to Keyscan task */
    app_ks_evt_notif_enable(0);
}

/*******************************************************************************
 * Function Name: app_reset_key_cnt
 ********************************************************************************
 * Summary:
 *  Reset the key_state_cnt , by default it starts with value 1, make it 0 to
 * disable MFO during deepsleep upon power up.
 *
 * Parameters:
 *
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void app_reset_key_cnt(void)
{
    /* reset this buffer to disable MFO in deepsleep */
    key_state_cnt = 0;
}

/*******************************************************************************
 * Function Name: app_ks_configure_keyscan
 ********************************************************************************
 * Summary:
 *  Configures keyscan with 8 rows and 18 columns, for more information, refer
 *  device configurator.
 *
 * Parameters:
 *
 *
 * Return:
 *  Status of Keyscan initialisation
 *
 *******************************************************************************/
int app_ks_configure_keyscan(void)
{
    cy_en_ks_status_t ks_status;
    ks_status = Cy_Keyscan_Init(MXKEYSCAN, &keyscan_0_config, &context);
    if (ks_status != CY_KEYSCAN_SUCCESS)
    {
        printf("Keyscan Initialization failed \r\n");
        return KEYSCAN_FAILURE;
    }

    ks_status = Cy_Keyscan_Register_Callback(app_ks_key_detected_callback, &context);
    if (ks_status != CY_KEYSCAN_SUCCESS)
    {
        printf("Keyscan register event notification failed. \r\n");
        return KEYSCAN_FAILURE;
    }

    Cy_Keyscan_ClearInterrupt(MXKEYSCAN, MXKEYSCAN_INTR_ALL);
    ks_status = Cy_Keyscan_SetInterruptMask(MXKEYSCAN, MXKEYSCAN_INTR_FIFO_THRESH_DONE);
    if (ks_status != CY_KEYSCAN_SUCCESS)
    {
        printf("Keyscan set interrupt mask failed. \r\n");
        return KEYSCAN_FAILURE;
    }
    return KEYSCAN_SUCCESS;
}

/*******************************************************************************
 * Function Name: app_ks_keyscan_intHandler
 ********************************************************************************
 * Summary:
 *  This is an interrupt callback which will be executed based on the
 *  Keyscan interrupt triggers.
 *
 * Parameters:
 *
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_ks_keyscan_intHandler(void)
{
    cy_en_ks_status_t status;
    uint32_t int_status;

    Cy_Keyscan_GetInterruptMaskedStatus(MXKEYSCAN, &int_status);
    if (MXKEYSCAN_INTR_FIFO_THRESH_DONE == int_status)
    {
        status = Cy_Keyscan_Interrupt_Handler(MXKEYSCAN, &context);
        if (CY_KEYSCAN_SUCCESS != status)
        {
            printf("Keyscan interrupt handler failed \r\n");
        }
        deepsleep_hold = 0;
    }
    else
    {
        deepsleep_hold = 1;
        Cy_Keyscan_ClearInterrupt(MXKEYSCAN, MXKEYSCAN_INTR_KEY_EDGE_DONE);
    }
}

/*******************************************************************************
 * Function Name: app_keyscan_handler_init
 ********************************************************************************
 * Summary:
 *  keyscan initialisation and registers call back
 *
 * Parameters:
 *
 *
 * Return:
 *  status of keyscan operation
 *
 *******************************************************************************/
static app_keyscan_status_t app_keyscan_handler_init(void)
{
    /* Configure Keymatrix size and debounce filters */
    if (KEYSCAN_SUCCESS != app_ks_configure_keyscan())
    {
        printf("Keyscan Config failed \r\n");
    }
    else
    {
        printf("Keyscan Config is successful \r\n");
    }

    return KEYSCAN_SUCCESS;
}

/*******************************************************************************
 * Function Name: app_keyscan_interrupt_init
 ********************************************************************************
 * Summary:
 *  Initialise interrupt for keyscan
 *
 * Parameters:
 *
 *
 * Return:
 *  status of keyscan operation
 *
 *******************************************************************************/
static app_keyscan_status_t app_keyscan_interrupt_init(void)
{
    cy_en_sysint_status_t sysStatus;

    /* Hook the interrupt service routine and enable the interrupt */
    sysStatus = Cy_SysInt_Init(&keyscan_irq_cfg, app_ks_keyscan_intHandler);
    if (CY_SYSINT_SUCCESS != sysStatus)
    {
        return KEYSCAN_FAILURE;
    }

    /* Enable interrupt in NVIC. */
    NVIC_EnableIRQ(keyscan_irq_cfg.intrSrc);

    return KEYSCAN_SUCCESS;
}

/*******************************************************************************
 * Function Name: app_ks_evt_notif_enable
 ********************************************************************************
 * Summary:
 *  This function is invoked when a Keyscan button interrupt is triggered
 *
 * Parameters:
 *
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_ks_evt_notif_enable(uint32_t key_notify)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (pdTRUE == xTaskNotifyFromISR(
                      keyscan_task_h,             /* Handle of the task being notified */
                      key_notify,                 /* Used to update the notification value of the target task */
                      eSetValueWithOverwrite,     /* The Target task receives the event and is unconditionally set to ulvalue. */
                      &xHigherPriorityTaskWoken)) /* This value will be set to pdTRUE when sending */
    {
        /* printf("key evt Notified to task\r\n"); */
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
 * Function Name: app_ks_evt_notif_enable
 ********************************************************************************
 * Summary:
 *  Pushes data to the hid queue. This is will be processed by the ble task
 *
 * Parameters:
 *  key_code     : hid code of the button activated
 *  up_down_flag : up -> key release -> 1 , down -> key press -> 0
 *  msg_type     : consumer hid msg or standard keyboard msg
 *  report_type  : kbd report or cc report
 *  send_msg     : flag to send message to BT stack or not
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_ks_send_msg_to_hid_msg_q(uint16_t key_code, uint8_t up_down_flag, uint8_t msg_type, uint16_t report_type, uint8_t send_msg)
{
    struct hid_rpt_msg ks_msg;
    ks_msg.msg_type = msg_type;
    ks_msg.report_type = report_type;
    ks_msg.send_msg = send_msg;
    ks_msg.data.ks.key_code = key_code;
    ks_msg.data.ks.up_down_flag = up_down_flag;
    if (pdPASS != xQueueSend(hid_rpt_q, &ks_msg, TICKS_TO_WAIT))
    {
        printf("Failed to send msg from KS to HID rpt Queue\r\n");
    }
}

/*******************************************************************************
 * Function Name: app_keyscan_ghost_timer_cb
 ********************************************************************************
 * Summary:
 *  Call back for ghost/rollover timer . Used to release the rollover event.
 *
 * Parameters:
 *  cb_params    : not used
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_keyscan_ghost_timer_cb(TimerHandle_t cb_params)
{
    if (app_ghost_detected == 0)
    {
        app_ks_send_msg_to_hid_msg_q(HID_ERR_OVF, KEY_RELEASED, KS_MSG_TYPE, HDLC_HIDS_KBD_IN_REPORT_VALUE, SEND_BLE_MSG);
        app_prev_ghost_detected = 0;
    }
}

/*******************************************************************************
 * Function Name: app_ks_idle_disconnection_timer_cb
 ********************************************************************************
 * Summary:
 *  Call back for disconnection timer.Disconnects after DISCONNECTION_TIMEOUT_MS
 *
 * Parameters:
 *  cb_params    : not used
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_ks_idle_disconnection_timer_cb(TimerHandle_t cb_params)
{
    if (app_bt_conn_id != 0)
    {
        if (CY_RSLT_SUCCESS != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            return;
        }
    }
}

/*******************************************************************************
 * Function Name: app_ks_update_function_key
 ********************************************************************************
 * Summary:
 * Updates the current status of function key
 *
 * Parameters:
 *  ble_tx_data_struct          : struct containing key tx information
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_ks_update_function_key(tx_data_t *ble_tx_data_struct)
{
    /* Set or Reset flag for Function key press */
    if (ble_tx_data_struct->key_code == KEYCODE_FN)
    {
        function_keys_struct.fn_pressed = ble_tx_data_struct->up_down_flag ? false : true;

        fn_key_hold = function_keys_struct.fn_pressed;

        if (ble_tx_data_struct->up_down_flag == KEY_RELEASED)
        {
            /* Clear the CC buffer to release any already sent notifications */
            app_clear_hid_cc_buffer();

            /* reset the channel selection key status */
            function_keys_struct.channel_key_pressed = false;
        }
    }
}

/*******************************************************************************
 * Function Name: app_ks_keycode_to_hidcode
 ********************************************************************************
 * Summary:
 * Returns the HID code  corresponding to a keyscan key code.
 *
 * Parameters:
 *  map_table_size       : size of the table to check
 *  key_code_table       : the mapping table which contains HID and Keycode
 *  key_code             : key_code for which the HID code needs to be identifed
 *
 * Return:
 *  returns the identified HID code
 *
 *******************************************************************************/
static uint16_t app_ks_keycode_to_hidcode(uint8_t map_table_size, KeycodeMap_t *key_code_table, uint8_t key_code)
{
    uint8_t index = 0;
    uint16_t return_hid_code = HID_INVALID;
    for (index = 0; index < map_table_size; index++)
    {
        if (key_code_table[index].key_code == key_code)
        {
            return_hid_code = key_code_table[index].hid_code;
            break;
        }
    }
    return return_hid_code;
}

/*******************************************************************************
 * Function Name: app_ks_find_special_fn_hid_code
 ********************************************************************************
 * Summary:
 * Returns HID code corresponding to keyscan key_code for special function keys.
 *
 * Parameters:
 *  map_table_size       : size of the table to check
 *  key_code_table       : the mapping table which contains HID and Keycode
 *  key_code             : key_code for which the HID value needs to be identifed
 *
 * Return:
 *  returns the identified HID value
 *
 *******************************************************************************/
static uint16_t app_ks_find_special_fn_hid_code(uint8_t map_table_size, Keycode_SplFn_Map_t *key_code_table, uint8_t key_code)
{
    uint8_t index = 0;
    uint16_t return_hid_code = HID_INVALID;

    /* find the hid code from the table */
    for (index = 0; index < map_table_size; index++)
    {
        if (key_code_table[index].key_code == key_code)
        {
            return_hid_code = key_code_table[index].os_mode_report[function_keys_struct.os_mode].hid_code;
            function_keys_struct.report_type = key_code_table[index].os_mode_report[function_keys_struct.os_mode].report_type;
            break;
        }
    }

    return return_hid_code;
}

/*******************************************************************************
 * Function Name: app_ks_handle_os_mode
 ********************************************************************************
 * Summary:
 * Handles the OS mode selection
 *
 * Parameters:
 *  u8keycode            : key_code to check whether mode/channel handling is required
 *  up_down_flag         : indicates key press or release , press = 0 , release = 1
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static uint8_t app_ks_handle_os_mode(uint8_t u8keycode, uint8_t up_down_flag)
{
    uint8_t index = 0;
    uint8_t os_mode = OS_INVALID;
    uint8_t status = false;

    /* find the os mode info from the mapping table */
    for (index = 0; index < OSMODE_SELECT_KEYS_SIZE; index++)
    {
        if (os_mode_map[index].key_code == u8keycode)
        {
            os_mode = (uint8_t)os_mode_map[index].os_mode;
            function_keys_struct.os_mode = os_mode;
            break;
        }
    }

    /* save the OS mode in flash */
    if ((os_mode != OS_INVALID) && (up_down_flag == 0))
    {
        status = true;
        app_bt_bond_save_os_mode(os_mode);
    }

    return status;
}

/*******************************************************************************
 * Function Name: app_ks_handle_channel_selection
 ********************************************************************************
 * Summary:
 * Handles the BLE channel switching
 *
 * Parameters:
 *  u8keycode          : key_code to check whether mode/channel handling is required
 *  up_down_flag       : indicates key press or release , press = 0 , release = 1
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static uint8_t app_ks_handle_channel_selection(uint8_t u8keycode, uint8_t up_down_flag)
{
    uint8_t index = 0;
    uint8_t channel_no = CHANNEL_INVALID;
    uint8_t status = false;

    /* find the channel number from the mapping table */
    for (index = 0; index < CHANNEL_SELECT_KEYS_SIZE; index++)
    {
        if (device_channel_map[index].key_code == u8keycode)
        {
            channel_no = (uint8_t)device_channel_map[index].channel;
            break;
        }
    }

    if ((channel_no != CHANNEL_INVALID) && (up_down_flag == KEY_PRESSED))
    {
#if (ENABLE_NUMERIC_KEY_CHANNEL_SWITCH == true)
        /* update the channel selection key status */
        function_keys_struct.channel_key_pressed = true;

        /*start pairing mode timer */
        xTimerStart(pairing_mode_timer, TIMER_MAX_WAIT);
#endif

        /* switch channel */
        app_bt_adv_bond_index_switch(channel_no);
    }
    else if (up_down_flag == KEY_RELEASED)
    {
#if (ENABLE_NUMERIC_KEY_CHANNEL_SWITCH == true)
        /* update the channel selection key status */
        function_keys_struct.channel_key_pressed = false;
#endif
    }
    return status;
}

bool channel_select_button_pressed(void)
{
    return function_keys_struct.channel_key_pressed;
}

/*******************************************************************************
 * Function Name: app_ks_handle_function_keys
 ********************************************************************************
 * Summary:
 * handle and process keycodes for function key combinations
 *
 * Parameters:
 *  ble_tx_data_struct   : struct containing keyscan data
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_ks_handle_function_keys(tx_data_t *ble_tx_data_struct)
{
    uint16_t hid_code;
    if (ble_tx_data_struct->key_code != KEYCODE_FN)
    {
        if (app_ks_handle_os_mode(ble_tx_data_struct->key_code, ble_tx_data_struct->up_down_flag) || app_ks_handle_channel_selection(ble_tx_data_struct->key_code, ble_tx_data_struct->up_down_flag))
        {
            /* its either change in os mode or change in device channel
             * so we dont need to send any hid codes
             */
            ble_tx_data_struct->send_tx_data = false;
        }
        else
        {
            /* we have received a valid key press for sending
             *  check if its standard or special function key
             */
            hid_code = app_ks_find_special_fn_hid_code(SPL_FN_ALL_KEYS_SIZE, KeyCode_Special_Function_Map, ble_tx_data_struct->key_code);

            /* if is not HID_INVALID , then its a special function key */
            if (hid_code != HID_INVALID)
            {
                ble_tx_data_struct->send_tx_data = true;
                ble_tx_data_struct->hid_code = hid_code;
                ble_tx_data_struct->reportType = function_keys_struct.report_type;
                if (function_keys_struct.report_type == HDLC_HIDS_CC_IN_REPORT_VALUE)
                {
                    ble_tx_data_struct->msg_type = CC_MSG_TYPE;
                }
                else
                {
                    ble_tx_data_struct->msg_type = KS_MSG_TYPE;
                }
            }
            else
            {
                /* check if its a normal key and send that */
                hid_code = app_ks_keycode_to_hidcode(MAX_KEYS, KeyCode_HID_Map, ble_tx_data_struct->key_code);
                if (hid_code != HID_INVALID)
                {
                    ble_tx_data_struct->send_tx_data = true;
                    ble_tx_data_struct->hid_code = hid_code;
                    ble_tx_data_struct->msg_type = KS_MSG_TYPE;
                    ble_tx_data_struct->reportType = HDLC_HIDS_KBD_IN_REPORT_VALUE;
                }
            }
        }
    }
}

/*******************************************************************************
 * Function Name: app_ks_handle_standard_keys
 ********************************************************************************
 * Summary:
 * handle and process keycodes for standard key combinations
 *
 * Parameters:
 *  ble_tx_data_struct   : struct containing keyscan data
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void app_ks_handle_standard_keys(tx_data_t *ble_tx_data_struct)
{
    ble_tx_data_struct->hid_code = app_ks_keycode_to_hidcode(MAX_KEYS, KeyCode_HID_Map, ble_tx_data_struct->key_code);
    ble_tx_data_struct->send_tx_data = true;
    ble_tx_data_struct->reportType = HDLC_HIDS_KBD_IN_REPORT_VALUE;
    ble_tx_data_struct->msg_type = KS_MSG_TYPE;
}

/*******************************************************************************
 * Function Name: app_ks_keycode_valid
 ********************************************************************************
 * Summary:
 * Checks whether a given key code is valid or not.
 *
 * Parameters:
 *  key_code   : key code to be validated
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static bool app_ks_keycode_valid(uint8_t key_code)
{
    if ((key_code >= KEY_CODE_MIN) && (key_code <= KEY_CODE_MAX))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*******************************************************************************
 * Function Name: keyscan_task
 *******************************************************************************
 * Summary:
 *  Task that prints the key_code, row and columns of the keyscan activity
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void keyscan_task(void *args)
{
    cy_en_ks_status_t app_ks_result;
    cy_stc_key_event kevent;
    cy_en_ks_status_t status;
    uint32_t ulNotifiedValue;
    uint8_t send_bt_msg = 0;
    uint8_t key_code;
    uint8_t up_down_flag;

    /* Initialize NVIC interrupt for Keyscan */
    app_ks_result = app_keyscan_interrupt_init();
    if (0 != app_ks_result)
    {
        printf("Keyscan Interrupt init failed \r\n");
    }

    /* Initialize and Handle Keyscan Hardware configuration */
    app_ks_result = app_keyscan_handler_init();

    /* Create timer for handling ghost/rollover event */
    keyscan_ghost_timer = xTimerCreate("Ghost Timer",
                                       50,
                                       pdFALSE,
                                       NULL,
                                       app_keyscan_ghost_timer_cb);

    /* Create timer to trigger Disconnection if keyboard idle for sometime */
    ble_disconnection_timer = xTimerCreate("Disconnection Timer",
                                           pdMS_TO_TICKS(DISCONNECTION_TIMEOUT_MS),
                                           pdFALSE,
                                           NULL,
                                           app_ks_idle_disconnection_timer_cb);
    /* Start of task loop */
    while (1)
    {
        /* wait for notifications from keyscan interrupt */
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);

        Cy_Keyscan_EventsPending(MXKEYSCAN, &events_pending, &context);

        while (events_pending)
        {
            /* get the next available event in keyscan buffer */
            status = Cy_Keyscan_GetNextEvent(MXKEYSCAN, &kevent, &context);
            if (status != CY_KEYSCAN_SUCCESS)
            {
                printf("Keyscan get next event failed \r\n");
            }

            /* Start Disconnection timer */
            if (pdFAIL == xTimerStart(ble_disconnection_timer, TIMER_MAX_WAIT))
            {
                printf("Failed to start Disconnection Timer\r\n");
            }

            /* Check value of key_code for the corresponding key being pressed */
            key_code = kevent.keyCode;
            up_down_flag = kevent.upDownFlag;

            /* store data in the ble structure */
            ble_tx_data_struct.key_code = kevent.keyCode;
            ble_tx_data_struct.up_down_flag = kevent.upDownFlag;
            ble_tx_data_struct.send_tx_data = false;
            ble_tx_data_struct.hid_code = HID_INVALID;

            if ((key_code == KEYSCAN_KEYCODE_ROLLOVER) || ((key_code == KEYSCAN_KEYCODE_GHOST)))
            {
                if (app_prev_ghost_detected == 0)
                {
                    /* send rollover event */
                    app_ks_send_msg_to_hid_msg_q(HID_ERR_OVF, 0, KS_MSG_TYPE, HDLC_HIDS_KBD_IN_REPORT_VALUE, SEND_BLE_MSG);
                }
                /* clear buffer since keyscan keeps sending events till ghost is cleared */
                app_clear_ble_hid_kbd_buffer();
                /* reset the ghost timer every time we see a ghost */
                xTimerReset(keyscan_ghost_timer, 10U);
                app_ghost_detected = 1;
                app_prev_ghost_detected = 1;
            }
            else if (key_code == KEYSCAN_KEYCODE_END_OF_SCAN_CYCLE)
            {
                app_ghost_detected = 0;
                Cy_Keyscan_EventsPending(MXKEYSCAN, &events_pending, &context);
                continue;
            }
            else
            {
                /* retrieve the OS mode save from flash data*/
                function_keys_struct.os_mode = bondinfo.os_mode[bondindex];

                printf("Keycode detected is :%u \r\n", key_code);

                /* do further processing only if valid key code is received */
                if (true == app_ks_keycode_valid(key_code))
                {
                    /* check whether function key is pressed */
                    app_ks_update_function_key(&ble_tx_data_struct);
                    if (function_keys_struct.fn_pressed)
                    {
                        /* process function key combinations */
                        app_ks_handle_function_keys(&ble_tx_data_struct);
                    }
                    else
                    {
                        /* standard key code is received */
                        app_ks_handle_standard_keys(&ble_tx_data_struct);
                    }
                    if ((ble_tx_data_struct.send_tx_data) && (ble_tx_data_struct.hid_code != HID_INVALID))
                    {
                        /* send hid codes only after ghost is cleared
                         *  setting send_bt_msg to 1 will send the hid code to lower layers
                         */
                        if (app_ghost_detected == 0)
                        {
                            send_bt_msg = 1;
                        }
                        else
                        {
                            /* update buffer upon every valid key event */
                            if (ble_tx_data_struct.reportType == KS_MSG_TYPE)
                            {
                                /* update on the KS buffer , since CC buffer has only one element */
                                app_bt_hid_update_buffer(ble_tx_data_struct.hid_code, up_down_flag);
                            }
                            send_bt_msg = 0;
                        }
                        /* send the data to hid queue for further processing */
                        app_ks_send_msg_to_hid_msg_q(ble_tx_data_struct.hid_code, ble_tx_data_struct.up_down_flag, ble_tx_data_struct.msg_type, ble_tx_data_struct.reportType, send_bt_msg);
                    }
                }
            }
            Cy_Keyscan_EventsPending(MXKEYSCAN, &events_pending, &context);
        }
    }
}
/* END OF FILE */
