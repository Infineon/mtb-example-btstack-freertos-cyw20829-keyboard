/*******************************************************************************
 * File Name: app_bt_event_handler.c
 *
 * Description:
 * This file contains the starting point of Bluetooth LE Keyboard application.
 * The wiced_bt_stack_init() registers for Bluetooth events in this main function.
 * The Bluetooth Management callback manages the Bluetooth events and the
 * application developer can customize the functionality and behavior depending on
 * the Bluetooth events. The Bluetooth Management callback acts like a
 * Finite State Machine (FSM) for the SoC.
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
 *                               Includes
 *******************************************************************************/
#include "app_bt_event_handler.h"
#include "app_bt_advert.h"
#include "app_handler.h"
#include "app_bt_bonding.h"
#include "app_bt_hid.h"

#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"

#include "cyhal_wdt.h"
#ifdef ENABLE_OTA
#include "app_ota_context.h"
#endif
/*******************************************************************************
 *                                Macros
 *******************************************************************************/


/*******************************************************************************
 *                               Global Variables
 *******************************************************************************/
/* Status variable for connection ID */
uint16_t app_bt_conn_id;

static uint8_t reset_bond_data = 0;
static uint16_t conn_interval = 0;
static uint16_t conn_slave_latency = 0;
static uint16_t conn_supervision_timeout = 0;

extern TimerHandle_t conn_param_update_timer;
extern uint8_t conn_param_update_retry;
#ifdef ENABLE_OTA
extern gatt_write_req_buf_t write_buff;
#endif
/*******************************************************************************
 *                           Function Prototypes
 *******************************************************************************/
/* This function initializes Bluetooth sub procedures */
static void app_bt_init(void);

/* This function initializes GATT DB and registers callback for GATT events */
static void app_bt_gatt_db_init(void);
bool ota_started = 0;
/*******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/**
 * Function Name:
 * app_bt_event_management_callback
 *
 * Function Description:
 * @brief This is a Bluetooth stack event handler function to receive management events
 *  from the Bluetooth LE stack and process as per the application.
 *
 * @param event:  Bluetooth LE event code of one byte length
 * @param p_event_data:  Pointer to Bluetooth LE management event
 *                                        structures
 *
 * @return wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 */
wiced_result_t
app_bt_event_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t       bt_dev_status       = WICED_SUCCESS;
    wiced_bt_ble_advert_mode_t *p_adv_mode          = NULL;
    wiced_bt_dev_ble_pairing_info_t *p_pairing_info = NULL;
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    printf("\nBluetooth Management Event: %d,\t", event);
    printf(app_bt_util_get_btm_event_name(event));
    printf("\r\n");

    switch (event)
    {

        case BTM_ENABLED_EVT:
            /* Perform application-specific initialization */
            app_bt_init();
            /* Registering callback for system power management */
            create_deep_sleep_cb();
            create_deep_sleep_ram_cb();
            create_deep_sleep_ram_dbg_diable_cb();

            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            /* Advertisement State Changed */
            p_adv_mode = &p_event_data->ble_advert_state_changed;
            printf("Advertisement state changed to %s\n",
                   app_bt_util_get_btm_advert_mode_name(*p_adv_mode));

            app_bt_adv_state_handler(*p_adv_mode);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            /* Need to compare with BT-SDK remote here for this event */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:

            app_peer_capabilities_ble_request(p_event_data);

            /* Reset the peer bd addr and use new address got when pairing complete */
            memset(peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));

            break;

        case BTM_PIN_REQUEST_EVT:
            /* HID Keyboard Device is not capable */
            break;

        case BTM_PASSKEY_REQUEST_EVT:            
            app_peer_passkey_request(p_event_data);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* Paired Device Link Keys Request */
            printf("Paired Device Link keys Request Event for device ");
            app_bt_util_print_bd_address((uint8_t*)(p_event_data->paired_device_link_keys_request.bd_addr));
            /* Need to search to see if the BD_ADDR we are looking for is in Flash. If not, we return WICED_BT_ERROR and the stack */
            /* will generate keys and will then call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they can be stored */

            /* This call will return WICED_BT_SUCCESS if the device is found else returns WICED_BT_ERROR */
            bt_dev_status = app_bt_bond_check_device_info(p_event_data->paired_device_link_keys_request.bd_addr);

            if (bt_dev_status == WICED_BT_SUCCESS)
            {
                /* Save the peer address for later use */
                memcpy(peer_bd_addr,
                       p_event_data->paired_device_link_keys_request.bd_addr,
                       sizeof(wiced_bt_device_address_t));
                printf("Peer Device BD ADDR: ");
                app_bt_util_print_bd_address(peer_bd_addr);

                /* Fetch the link keys saved previously */
                app_bt_bond_get_device_link_keys(&(p_event_data->paired_device_link_keys_request));
            }
            else
            {
                printf("Device Link Keys not found in the database! \n");
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Save the link keys of paired device to Non volatile storage */
            printf("Paired Device Key Update \r\n");

            rslt = app_bt_bond_save_device_link_keys(&(p_event_data->paired_device_link_keys_update));

            if (CY_RSLT_SUCCESS == rslt)
            {
                printf("Successfully Bonded to ");
                app_bt_util_print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
            }
            else
            {
                printf("Failed to bond! \r\n");
            }

            /* The security keys can be printed here */

            break;

        case BTM_PAIRING_COMPLETE_EVT:
            /* Received Pairing Complete Event */
            p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

            if (SMP_SUCCESS == p_pairing_info->reason) /* Bonding successful */
            {
                /* Save the peer address for later use */
                memcpy(peer_bd_addr,
                       p_event_data->pairing_complete.bd_addr,
                       sizeof(wiced_bt_device_address_t));
                /* Print Bond information stats once a new device is paired.
                 (pairing complete event) */
                printf("Successfully Bonded to: ");
                app_bt_util_print_bd_address(p_event_data->pairing_complete.bd_addr);

                /* Update Num of bonded devices in slot data*/
                app_bt_bond_update_slot_data();

                /* Update current application state for state machine handling */
                app_bt_hid_update_device_state(CONNECTED_NON_ADVERTISING);

                /* Print bond info */
                app_bt_bond_print_info_stats();
            }
            else
            {
                if(SMP_CONFIRM_VALUE_ERR == p_pairing_info->reason)
                {
                    app_bt_adv_start_any_host();
                }
                printf("Bonding failed. Reason for failure: ");
                printf(app_bt_util_get_pairing_status_name(p_pairing_info->reason));
                printf("\r\n");
                /* Delete host info and update bonded list */
            }

            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Save the identity keys to the NV storage */
            printf("Local Identity Key Update\n");
            rslt = app_bt_bond_save_local_identity_key(p_event_data->local_identity_keys_update);
            if (CY_RSLT_SUCCESS != rslt)
            {
                bt_dev_status = WICED_BT_ERROR;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Retrieve the identity keys from the NV storage */

            if (reset_bond_data)
            {
                app_bt_bond_delete_info();
                app_bt_bond_update_data();
            }

            /*
             * If the key type is Identity keys; throw WICED_ERROR to cause the
             * BT stack to generate new keys and then call
             * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that the keys can be stored
             * */
            printf("Local Identity Key Request\r\n");
            /*Read Local Identity Resolution Keys*/
            rslt = app_bt_bond_read_local_identity_keys();
            if (CY_RSLT_SUCCESS == rslt)
            {
                memcpy(&(p_event_data->local_identity_keys_request),
                       &(identity_keys),
                       sizeof(wiced_bt_local_identity_keys_t));
                app_bt_util_print_byte_array(&identity_keys, sizeof(wiced_bt_local_identity_keys_t));
                bt_dev_status = WICED_BT_SUCCESS;
            }
            else
            {
                bt_dev_status = WICED_BT_ERROR;
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            /* Encryption Status Change */
            printf("Encryption Status event for: ");
            app_bt_util_print_bd_address(p_event_data->encryption_status.bd_addr);
            printf("Encryption Status event result: %d \r\n",
                   p_event_data->encryption_status.result);

            /* Check if the bond data of the device that got connected is already present */
            if (!memcmp(peer_bd_addr,
                        p_event_data->encryption_status.bd_addr,
                        sizeof(wiced_bt_device_address_t)))
            {
                /* Update current application state for state machine handling */
                app_bt_hid_update_device_state(CONNECTED_NON_ADVERTISING);

                /* Restore the CCCD stored earlier */
                app_bt_bond_restore_cccd();
                /* Enable all IN Report notifications. The HID Host will automatically
                 * enable all CCCDs. On some cases, the HID device is done independently
                 * for example chromecast. So Use it for debug purposes only.
                 */
                //          app_enable_all_cccds();
            }

            /* Trigger Connection Param Update request from HID device after a delay */
            if (pdFAIL == xTimerStart(conn_param_update_timer, TIMER_MAX_WAIT))
            {
                printf("Failed to start Connection parameter update Timer\r\n");
            }

            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:

            printf("Remote Connection parameter update status:%d,\r\n"
                   "Remote Connection Interval: %d,\r\n"
                   "Remote Connection Latency: %d,\r\n"
                   "Remote Connection Timeout: %d\r\n",
                   p_event_data->ble_connection_param_update.status,
                   p_event_data->ble_connection_param_update.conn_interval,
                   p_event_data->ble_connection_param_update.conn_latency,
                   p_event_data->ble_connection_param_update.supervision_timeout);

            conn_interval = p_event_data->ble_connection_param_update.conn_interval;
            conn_slave_latency = p_event_data->ble_connection_param_update.conn_latency;
            conn_supervision_timeout = p_event_data->ble_connection_param_update.supervision_timeout;

            /* Check if connection parameters are within the required range */
            if ((p_event_data->ble_connection_param_update.conn_interval < MIN_CI) ||
                    (p_event_data->ble_connection_param_update.conn_interval > MAX_CI))
            {
                if(++conn_param_update_retry <= CONN_PARAM_UPDATE_RETRY_COUNT)
                {
                    rslt = CY_RSLT_TYPE_ERROR;
                }
            }

            /* Connection Parameter Update request from HID device */
            if (rslt == CY_RSLT_SUCCESS)
            {
                /* Stop Connection parameter update timer */
                conn_param_update_retry = 0;
                conn_param_updated_flag = TRUE;
                xTimerStop(conn_param_update_timer, TIMER_MAX_WAIT);
            }
            else
            {
                /* Start Connection parameter update timer */
                conn_param_updated_flag = FALSE;
                if(!ota_started)
                {
                    xTimerStart(conn_param_update_timer, TIMER_MAX_WAIT);
                }
            }


            break;

        case BTM_SECURITY_FAILED_EVT:
            /* Handle pairing Failure */
            printf("Pairing Failed\r\n");
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            printf("BTM_BLE_PHY_UPDATE_EVT,\r\n "
                    "PHY Tx value is: %d, \r\n"
                    "PHY Rx value is: %d \r\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;

        case BTM_DISABLED_EVT:
            /* Bluetooth Controller and Host Stack Disabled */
            printf("BTM_DISABLED_EVT\r\n");
            break;

        case BTM_BLE_DATA_LENGTH_UPDATE_EVENT:
            printf("BTM_BLE_DATA_LENGTH_UPDATE_EVENT, \r\n"
                    "Max tx octets is :%d ,\r\n"
                    "Max rx octets is :%d \r\n",
                    p_event_data->ble_data_length_update_event.max_tx_octets,
                    p_event_data->ble_data_length_update_event.max_rx_octets);
            break;

        default:
            printf("\nUnhandled Bluetooth Management Event: %d\r\n", event);
            break;
    }

    return (bt_dev_status);
}

/**
 * Function Name:
 * app_bt_init
 *
 * Function Description:
 * @brief This Function initializes the Bluetooth functions such as GATT DB
 * initialization, Bonding info and advertisement
 *
 * @param void
 *
 * @return void
 */
static void app_bt_init(void)
{
    wiced_bt_device_address_t local_bda = { 0 };
#ifdef ENABLE_OTA
    memset(&write_buff, 0x00, sizeof(gatt_write_req_buf_t));
#endif

    /* GATT DB Initialization */
    app_bt_gatt_db_init();

    /* Retrieve Bond information from NV Storage to Bondinfo structure in RAM */
    /* The​ ​value​ ​of​ ​the​ ​​Client​ ​Characteristic​ ​Configuration​ ​​descriptor​ ​is​ ​persistent​
     * ​for​ ​bonded devices​ ​when​ ​not​ ​in​ ​a​ ​connection
     */
    /* CCCD needs to be restored */
    app_bt_bond_restore_data();

    /* Prints the Number of Bonded devices and free slots */
    app_bt_bond_print_info_stats();

    /* Print the Bond data(BD_ADDR, Link Keys & Identity keys) from EmEEPROM */
    app_bt_bond_print_data();

    /* Generate unique BD Address if not present */
    if (CY_RSLT_SUCCESS != app_bt_bond_get_local_bd_addr(local_bda))
    {
        /* Generate the bd address to be used for pairing a new host */
        app_bt_util_generate_bd_address(local_bda);

        /* Save the bd address to flash */
        app_bt_bond_update_local_bd_addr(local_bda);
    }
    /* Set the local bd address */
    wiced_bt_set_local_bdaddr(local_bda, BLE_ADDR_RANDOM);

    printf("\nBluetooth Device Address: ");
    app_bt_util_print_bd_address(local_bda);

    /* Start Advertisement or re-connect */
    app_bt_adv_start();
}

/**
 * Function Name:
 * app_bt_gatt_db_init
 *
 * Function Description:
 * @brief Initialize the Bluetooth LE GATT Database with the necessary services and
 * characteristics and register GATT callback function.
 *
 * @param void
 *
 * @return void
 *
 */
static void app_bt_gatt_db_init(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    /* Initialize GATT Database */
    if (WICED_BT_GATT_SUCCESS != wiced_bt_gatt_db_init(gatt_database,
                                                       gatt_database_len,
                                                       NULL))
    {
        printf("\r\n GATT DB Initialization not successful\r\n");
    }

    /* Register with Bluetoth stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_bt_gatt_event_handler);

    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        printf("\nGATT event gatt_status:\t");
        printf(app_bt_util_get_gatt_status_name(gatt_status));
        printf("\r\n");
    }
}
