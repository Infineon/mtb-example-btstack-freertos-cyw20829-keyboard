/*******************************************************************************
 * File Name: app_bt_advert.c
 *
 * Description: This File provides the implementations necessary for Bluetooth
 * Advertisements.
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
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"

#include "app_bt_advert.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_hid.h"

#include "app_handler.h"

/*******************************************************************************
 *                      MACROS / VARIABLE DEFINITIONS
 ******************************************************************************/
#define NON_DISCOVERABLE_MODE_ADV   (BTM_BLE_BREDR_NOT_SUPPORTED)
#define DISCOVERABLE_MODE_ADV       (BTM_BLE_BREDR_NOT_SUPPORTED|BTM_BLE_LIMITED_DISCOVERABLE_FLAG)

/* Timer handle for stopping advertisement */
extern TimerHandle_t adv_stop_timer;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
/* This function sets the advertisement data */
static void app_bt_adv_set_data(uint8_t adv_flag);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 *  Function name:
 *  app_bt_adv_start
 *
 *  Function Description:
 *  @brief    Function used to start connectable advertisements once
 *  BTM_ENABLED_EVT event occurs in Bluetooth management callback
 *
 *  @param    void
 *
 *  @return   void
 */
void app_bt_adv_start(void)
{
    /* Stop ongoing adv if any */
    app_bt_adv_stop();

    /* Check if Bond Information is present */
    if (CY_RSLT_SUCCESS == app_bt_bond_check_info())
    {
        /* Bonded Device found */
        app_bt_hid_update_device_state(PAIRED_ADVERTISING_KNOWN_HOST);
        app_bt_adv_start_known_host();
    }
    else
    {
        /* No Bonded Device */
        /* Allow new devices to bond */
        app_bt_hid_update_device_state(UNPAIRED_ADVERTISING);
        app_bt_adv_start_any_host();
    }
}

/**
 * Function Name:
 * app_bt_adv_set_data
 *
 * Function Description:
 * @brief   Function used to set LE Advertisement Data
 *
 * @param   void
 *
 * @return  void
 */
static void app_bt_adv_set_data(uint8_t adv_flag)
{
    uint8_t cy_bt_adv_packet_elem_0[1] = { adv_flag };

    /* Set the adv flag */
    cy_bt_adv_packet_data[0].p_data = (uint8_t*)cy_bt_adv_packet_elem_0;

    /* Set Advertisement data */
    if (WICED_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(NUM_ADV_ELEM, cy_bt_adv_packet_data))
    {
        printf("Setting advertisement data Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_state_handler
 *
 *  Function Description:
 *  @brief Application State change handler based on current advertisement state.
 *
 *  @param    current_adv_mode:  Current Advertisement mode of the application
 *
 *  @return   void
 */
void app_bt_adv_state_handler(wiced_bt_ble_advert_mode_t current_adv_mode)
{
    switch (current_adv_mode)
    {
        case BTM_BLE_ADVERT_OFF:
            /* If advertisements was turned off due to time out after disconnection,
             * then the previous state is PAIRED_ADVERTISING_ANY_HOST
             */
            if (app_bt_conn_id == 0)
            {
                if ((app_bt_hid_get_device_state() == PAIRED_ADVERTISING_ANY_HOST) ||
                        (app_bt_hid_get_device_state() == PAIRED_ADVERTISING_KNOWN_HOST))
                {
                    app_bt_hid_update_device_state(PAIRED_IDLE);
                }
                else if (app_bt_hid_get_device_state() == UNPAIRED_ADVERTISING)
                {
                    app_bt_hid_update_device_state(UNPAIRED_IDLE);
                }
            }
            else
            {
                app_bt_hid_update_device_state(CONNECTED_NON_ADVERTISING);
            }

            /* Stop LED blinking */
            app_status_led_stop_blinking();
            break;
        case BTM_BLE_ADVERT_DIRECTED_HIGH:
        case BTM_BLE_ADVERT_DIRECTED_LOW:
        case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
        case BTM_BLE_ADVERT_UNDIRECTED_LOW:
        case BTM_BLE_ADVERT_NONCONN_HIGH:
        case BTM_BLE_ADVERT_NONCONN_LOW:
        case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
            /* Start LED blinking for adv indication */
            app_status_led_start_blinking();
            break;
        default:
            printf("ERROR: Unknown advertisement state\r\n");
            break;
    }

}

/**
 *  Function name:
 *  app_bt_adv_start_known_host
 *
 *  Function Description:
 *  @brief This Function starts undirected Bluetooth LE advertisement for reconnection to known host
 *
 *  @param   void
 *
 *  @return  void
 */
void app_bt_adv_start_known_host(void)
{
    printf("app_bt_start_adv_known_host\r\n");

    wiced_bt_device_address_t local_bdaddr;

    /* Set Advertisement Data */
    app_bt_adv_set_data(NON_DISCOVERABLE_MODE_ADV);

    /* Start timer to stop undirected adv after required duration */
    if (pdFAIL == xTimerStart(adv_stop_timer, TIMER_MIN_WAIT))
    {
        printf("Failed to start Advertisement stop Timer\r\n");
    }

    /* Update LED blink period for directed adv indication */
    app_led_update_blink_period(RECONNECTION_ADV_LED_BLINK_TIME_MS);

    /* Disable pairing mode */
    wiced_bt_set_pairable_mode(FALSE, TRUE);

    /* Update the local address used for connecting to the host */
    app_bt_bond_get_local_bd_addr(local_bdaddr);
    wiced_bt_set_local_bdaddr(local_bdaddr, BLE_ADDR_RANDOM);

    /* Start Undirected LE Advertisements */
    if (WICED_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                       BLE_ADDR_PUBLIC,
                                                       NULL))
    {
        printf("Starting undirected Bluetooth LE advertisements Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_start_any_host
 *
 *  Function Description:
 *  @brief This Function starts undirected Bluetooth LE advertisement for pairing to new host
 *
 *  @param  void
 *
 *  @return void
 */
void app_bt_adv_start_any_host(void)
{
    printf("app_bt_start_adv_any_host\r\n");

    wiced_bt_device_address_t local_bdaddr;

    /* Set Advertisement Data */
    app_bt_adv_set_data(DISCOVERABLE_MODE_ADV);
    /* Start timer to stop adv after required duration */
    if (pdFAIL == xTimerStop(adv_stop_timer, TIMER_MIN_WAIT))
    {
        printf("Failed to stop Advertisement stop Timer\r\n");
    }
    /* Update LED blink period for pairing mode undirected adv indication */
    app_led_update_blink_period(PAIRING_MODE_ADV_LED_BLINK_TIME_MS);

    /* Enable pairing mode */
    wiced_bt_set_pairable_mode(TRUE, FALSE);

    /* Update the local address to be used for pairing a new host */
    app_bt_bond_get_new_bd_addr(local_bdaddr);
    wiced_bt_set_local_bdaddr(local_bdaddr, BLE_ADDR_RANDOM);

    /* Start Undirected LE Advertisements */
    if (WICED_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                       BLE_ADDR_PUBLIC,
                                                       NULL))
    {
        printf("Starting undirected Bluetooth LE advertisements Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_stop
 *
 *  Function Description:
 *  @brief  Function used to stop ongoing Bluetooth LE advertisement
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_adv_stop(void)
{
    printf("app_bt_stop_adv\r\n");

    if (WICED_SUCCESS != wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF,
                                                       BLE_ADDR_PUBLIC,
                                                       NULL))
    {
        printf("Stop Bluetooth LE advertisements Failed\r\n");
    }
}

/**
 *  Function name:
 *  app_bt_adv_pairing_mode_switch
 *
 *  Function Description:
 *  @brief This Function is used to switch device to pairing mode
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_adv_pairing_mode_switch(void)
{
    /* If in connected state, disconnect from current host */
    if (app_bt_conn_id != 0)
    {
        if (CY_RSLT_SUCCESS != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            return;
        }
    }

    /* Stop ongoing adv if any */
    app_bt_adv_stop();

    /* update current state to any host adv */
    current_app_state = PAIRED_ADVERTISING_ANY_HOST;

    if (!app_bt_conn_id)
    {
        /* Start undirected adv for pairing to new host device if not in connected state, else start from disconnect callback */
        app_bt_adv_start_any_host();
    }
}

/**
 *  Function name:
 *  app_bt_adv_bond_index_switch
 *
 *  Function Description:
 *  @brief This Function is used to change the bond info slot
 *
 *  @param    void
 *
 *  @return    void
 */
void app_bt_adv_bond_index_switch(uint8_t device_channel)
{
    /* If in connected state, disconnect from current host */
    if (app_bt_conn_id != 0)
    {
        if (CY_RSLT_SUCCESS != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            return;
        }
    }

    /* Stop ongoing adv if any */
    app_bt_adv_stop();

    /*Update current bond index */
    app_bt_bond_update_index(device_channel);

    /* update current state to any host adv */
    if (CY_RSLT_SUCCESS == app_bt_bond_check_info())
    {
        current_app_state = PAIRED_ADVERTISING_KNOWN_HOST;
    }
    else
    {
        current_app_state = PAIRED_ADVERTISING_ANY_HOST;
    }

    if (!app_bt_conn_id)
    {
        /* Start adv if not in connected state, else start from disconnect callback */
        app_bt_adv_start();
    }
}
