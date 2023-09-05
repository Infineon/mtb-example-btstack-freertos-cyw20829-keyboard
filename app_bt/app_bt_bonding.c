/******************************************************************************
 * File Name: app_bt_bonding.c
 *
 * Description: This is the source code for Bluetooth bonding implementation using
 *              kv-store library.
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
 *                               Header Files
 *******************************************************************************/
#include <inttypes.h>
#include "cybsp.h"
#include "cyhal.h"
#include "cy_pdl.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gatt_db.h"
#include "wiced_bt_stack.h"
#include "mtb_kvstore.h"
#include "app_bt_utils.h"
#include "app_bt_bonding.h"
#include "app_handler.h"

/*******************************************************************************
 *                              Macro Definitions
 ******************************************************************************/
/*
 * Position refers to the bit position from the LSB.
 * Position starts from 0 to N-1.
 * N refers to size of the variable. Byte has N = 8.
 * x refers to cccd_flags variable
 */
#define SET_CCCD_BIT(x, pos)    (x |= (0x01 << (pos)))
#define CLEAR_CCCD_BIT(x, pos)  (x &= ~(0x01 << (pos)))
#define IS_THIS_BIT_SET(x, pos) (x & (0x01 << (pos)))

/*******************************************************************************
 *                              Variable Definitions
 ******************************************************************************/

/* Local Identity Key */
wiced_bt_local_identity_keys_t identity_keys;
/* Structure containing bonding info of peer devices */
tBondInfo bondinfo = { 0 };
/* This is the index for the link keys, identity keys, cccd and privacy mode
 * of the host we are currently bonded to */
uint8_t bondindex = 0;

extern uint16_t app_bt_conn_id;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static uint8_t app_bt_bond_find_cccd_bit(uint16_t attr_handle);
static cy_rslt_t app_bt_bond_update_new_bd_addr(uint8_t *device_addr);
static wiced_result_t app_bt_bond_delete_device_info(uint8_t index);
static wiced_result_t app_bt_bond_add_devices_to_address_resolution_db(void);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 *              Peer Device and Bond Info Management APIs
 ******************************************************************************/

/**
 * Function Name:
 * app_bt_bond_restore_data
 *
 * Function Description:
 * @brief  This function restores the bond information from the Flash
 *
 * @param   void
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the restoration was successful,
 * an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_restore_data(void)
{
    /* Read and restore contents of Serial flash */
    uint32_t data_size = sizeof(bondinfo);
    cy_rslt_t rslt = mtb_kvstore_read(  &kvstore_obj,
                                        "bondinfo",
                                        (uint8_t *)&bondinfo,
                                        &data_size);
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("Bond data not present in the flash!\r\n");
        return rslt;
    }

    /* Get the channel index of last connected device */
    bondindex = bondinfo.slot_data[LAST_CONNECT_INDEX];

    /* Add devices to address resolution db */
    rslt = app_bt_bond_add_devices_to_address_resolution_db();

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_update_slot_data
 *
 * Function Description:
 * @brief  This function updates the slot data in the Flash
 *
 * @param  void
 *
 * @return cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 * an error code otherwise.
 */
cy_rslt_t app_bt_bond_update_slot_data(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    /* Increment number of bonded devices and save them in Flash */
    if (BOND_MAX > bondinfo.slot_data[NUM_BONDED])
    {
        /* Increment only if the bonded devices are less than BOND_MAX */
        bondinfo.slot_data[NUM_BONDED]++;
    }

    /* Save the bd address to flash */
    app_bt_bond_update_local_bd_addr(bondinfo.next_local_bd_addr);

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_update_data
 *
 * Function Description:
 * @brief This function updates the bond information in the Flash
 *
 * @param   void
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_update_data(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    rslt = mtb_kvstore_write(&kvstore_obj, "bondinfo", (uint8_t*)&bondinfo, sizeof(bondinfo));

    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Flash Write Error,Error code: %" PRIu32 "\r\n", rslt);
    }

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_check_info
 *
 * Function Description:
 * @brief This function fetches checks if bond info is present in the current index selected
 *
 * @param   uint8_t: Bond device count
 *
 * @return  void
 *
 */
uint8_t app_bt_bond_check_info(void)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    wiced_bt_device_address_t local_bda = { 0 };

    /* Check if bond info is present */
    if (memcmp(bondinfo.link_keys[bondindex].bd_addr, local_bda, sizeof(wiced_bt_device_address_t)))
    {
        rslt = CY_RSLT_SUCCESS;
    }

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_get_info_count
 *
 * Function Description:
 * @brief   This function fetches the count of bonded devices from bond information in the Flash
 *
 * @param   void
 *
 * @return  uint8_t: Bond device count
 *
 */
uint8_t app_bt_bond_get_info_count(void)
{
    return bondinfo.slot_data[NUM_BONDED];
}

/**
 * Function Name:
 * app_bt_bond_get_new_bd_addr
 *
 * Function Description:
 * @brief   This function fetches the new bd address to be used from bond information in the Flash
 *
 * @param   device_addr: Pointer to device address
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_get_new_bd_addr(uint8_t *device_addr)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    wiced_bt_device_address_t local_bda = { 0 };

    if (memcmp(bondinfo.next_local_bd_addr, local_bda, sizeof(wiced_bt_device_address_t)))
    {
        rslt = CY_RSLT_SUCCESS;

        memcpy((uint8_t *)(device_addr),
               bondinfo.next_local_bd_addr,
               sizeof(wiced_bt_device_address_t));
    }

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_get_local_bd_addr
 *
 * Function Description:
 * @brief This function fetches the bd address used currently from bond information in the Flash
 *
 * @param   device_addr: Pointer to device address
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_get_local_bd_addr(uint8_t *device_addr)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    wiced_bt_device_address_t local_bda = { 0 };

    printf("app_bt_get_local_bd_addr \r\n");

    /* Check if local bd address is present */
    if (memcmp(bondinfo.local_bd_addr[bondindex], local_bda, sizeof(wiced_bt_device_address_t)))
    {
        rslt = CY_RSLT_SUCCESS;

        memcpy((uint8_t *)(device_addr),
               &bondinfo.local_bd_addr[bondindex],
               sizeof(wiced_bt_device_address_t));

        printf("Local bdaddress present for the slot\r\n");
    }
    else if (CY_RSLT_SUCCESS == app_bt_bond_get_new_bd_addr(device_addr))
    {
        printf("Using new bd address for the slot\r\n");
        /* Using new bd address for the slot */
        rslt = CY_RSLT_SUCCESS;
    }

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_update_local_bd_addr
 *
 * Function Description:
 * @brief This function updates the local bd address used in bond information in the Flash
 *
 * @param   device_addr: Pointer to device address
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_update_local_bd_addr(uint8_t *device_addr)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    memcpy(&bondinfo.local_bd_addr[bondindex],
           (uint8_t *)(device_addr),
           sizeof(wiced_bt_device_address_t));

    /* Update the local bd address to be used for pairing new device */
    app_bt_bond_update_new_bd_addr(device_addr);

    /* Save updated bondinfo to flash */
    rslt = app_bt_bond_update_data();

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_update_new_bd_addr
 *
 * Function Description:
 * @brief This function updates the local bd address used in bond information in the Flash
 *
 * @param   device_addr: Pointer to device address
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 *              an error code otherwise.
 *
 */
static cy_rslt_t app_bt_bond_update_new_bd_addr(uint8_t *device_addr)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    wiced_bt_device_address_t local_bda = { 0 };

    /* Check if local bd address is present */
    if (!memcmp(bondinfo.next_local_bd_addr, local_bda, sizeof(wiced_bt_device_address_t)))
    {
        memcpy(&bondinfo.next_local_bd_addr,
               (uint8_t *)(device_addr),
               sizeof(wiced_bt_device_address_t));
    }
    else
    {
        /* Update the local bd address to be used for pairing new device */
        bondinfo.next_local_bd_addr[5] += 1;
    }

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_update_index
 *
 * Function Description:
 * @brief This function updates the current bond index information in the Flash
 *
 * @param   void
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_update_index(uint8_t device_no)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    bondindex = device_no;
    bondinfo.slot_data[LAST_CONNECT_INDEX] = bondindex;

    /* Save updated bondinfo to flash */
    rslt = app_bt_bond_update_data();

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_delete_info
 *
 * Function Description:
 * @brief  This function removes all bond information in the device
 *
 * @param  void
 *
 * @return  cy_rslt_t: CY_RSLT_SUCCESS if the deletion was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_delete_info(void)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    if (app_bt_conn_id != 0)
    {
        /* Disconnect the peer device before deleting the bondinfo */
        if (0 != wiced_bt_gatt_disconnect(app_bt_conn_id))
        {
            printf("Disconnect failed\r\n");
        };
    }

    for (uint8_t i = 0; i < bondinfo.slot_data[NUM_BONDED]; i++)
    {
        wiced_result_t result = app_bt_bond_delete_device_info(i);
        if (WICED_BT_SUCCESS != result)
        {
            rslt = CY_RSLT_TYPE_ERROR;
            printf("Delete bond info failed\r\n");
            return rslt;
        }
    }

    /* Remove bonding information in RAM */
    memset(&bondinfo, 0, sizeof(bondinfo));

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_delete_device_info
 *
 * Function Description:
 * @brief  This function deletes the bond information of the device from the RAM
 *         and address resolution database.
 *
 * @param  index: Index of the device whose data is to be deleted
 *
 * @return  wiced_result_t: WICED_BT_SUCCESS if the deletion was successful,
 *                   an error code otherwise.
 *
 */
static wiced_result_t app_bt_bond_delete_device_info(uint8_t index)
{
    wiced_result_t result = WICED_BT_SUCCESS;

    /* Remove from the bonded device list after disconnect only */
    result = wiced_bt_dev_delete_bonded_device(bondinfo.link_keys[index].bd_addr);
    if (WICED_BT_SUCCESS != result)
    {
        printf("Delete bond info failed\r\n");
        return result;
    }

    /* Remove link keys from address resolution database */
    result = wiced_bt_dev_remove_device_from_address_resolution_db(&(bondinfo.link_keys[index]));
    if (WICED_BT_SUCCESS != result)
    {
        printf("Delete addr resolution DB failed\r\n");
        return result;
    }

    return result;
}

/**
 * Function Name:
 * app_bt_bond_check_device_info
 *
 * Function Description:
 * @brief This function checks if the provided bd_addr is present in the current bond devices list index
 *
 * @param  bd_addr: pointer to the address of the device to be searched
 *
 * @return wiced_result_t: WICED_BT_SUCCESS if device found,
 *            else returns  WICED_BT_ERROR to indicate the device was not found.
 *
 */
wiced_result_t app_bt_bond_check_device_info(uint8_t *bd_addr)
{
    wiced_result_t bt_dev_status = WICED_BT_ERROR;

    if (0 == memcmp(&(bondinfo.link_keys[bondindex].bd_addr),
                    bd_addr,
                    sizeof(wiced_bt_device_address_t)))
    {
        printf("Found device in the current bond index!\r\n");
        bt_dev_status = WICED_BT_SUCCESS;
    }

    return bt_dev_status;
}

/**
 * Function Name:
 * app_bt_bond_add_devices_to_address_resolution_db
 *
 * Function Description:
 * @brief This function adds the bonded devices to address resolution database
 *
 * @param void
 *
 * @return void
 *
 */
static wiced_result_t app_bt_bond_add_devices_to_address_resolution_db(void)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    /* Copy in the keys and add them to the address resolution database */
    for (uint8_t i = 0; i < bondinfo.slot_data[NUM_BONDED]; i++)
    {
        /* Add device to address resolution database */
        result = wiced_bt_dev_add_device_to_address_resolution_db(&bondinfo.link_keys[i]);
        result = wiced_bt_ble_set_privacy_mode(bondinfo.link_keys[i].bd_addr,
                                               bondinfo.link_keys[i].key_data.ble_addr_type,
                                               BTM_BLE_PRIVACY_MODE_DEVICE);

        if (WICED_BT_SUCCESS == result)
        {
            printf("Device added to address resolution database: ");
            app_bt_util_print_bd_address((uint8_t*)&bondinfo.link_keys[i].bd_addr);
        }
        else
        {
            printf("Error adding device to address resolution database, Error Code %d \n", result);
        }
    }

    return result;
}

/*******************************************************************************
 *              Security Keys Management APIs
 ******************************************************************************/
/**
 * Function Name:
 * app_bt_bond_get_device_link_keys
 *
 * Function Description:
 * @brief This function fetches the peer device link keys in bond info
 *
 * @param link_key: link keys of the peer device.
 *
 * @return cy_rslt_t: CY_RSLT_SUCCESS if the fetch was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_get_device_link_keys(wiced_bt_device_link_keys_t *link_key)
{
    cy_rslt_t rslt = CY_RSLT_SUCCESS;

    memcpy(link_key,
           &bondinfo.link_keys[bondindex],
           sizeof(wiced_bt_device_link_keys_t));

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_save_device_link_keys
 *
 * Function Description:
 * @brief This function saves peer device link keys to the Flash
 *
 * @param link_key: Save link keys of the peer device.
 *
 * @return cy_rslt_t: CY_RSLT_SUCCESS if the save was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_save_device_link_keys(wiced_bt_device_link_keys_t *link_key)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;

    memcpy(&bondinfo.link_keys[bondindex],
           (uint8_t *)(link_key),
           sizeof(wiced_bt_device_link_keys_t));

    /* Save updated bondinfo to flash */
    rslt = app_bt_bond_update_data();

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_save_local_identity_key
 *
 * Function Description:
 * @brief This function saves local device identity keys to the Flash
 *
 * @param id_key: Local identity keys to store in the flash.
 *
 * @return cy_rslt_t: CY_RSLT_SUCCESS if the save was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_save_local_identity_key(wiced_bt_local_identity_keys_t id_key)
{
    memcpy( &identity_keys,
            (uint8_t *)&(id_key),
            sizeof(wiced_bt_local_identity_keys_t));
    cy_rslt_t rslt = mtb_kvstore_write( &kvstore_obj,
                                        "local_irk",
                                        (uint8_t *)&identity_keys,
                                        sizeof(wiced_bt_local_identity_keys_t));
    if (CY_RSLT_SUCCESS == rslt)
    {
        printf("Local identity Keys saved to Flash \r\n");
    }
    else
    {
        printf("Flash Write Error,Error code: %" PRIu32 "\r\n", rslt);
    }

    return rslt;
}

/**
 * Function Name:
 * app_bt_bond_read_local_identity_keys
 *
 * Function Description:
 * @brief This function reads local device identity keys from the Flash
 *
 * @param void
 *
 * @return cy_rslt_t: CY_RSLT_SUCCESS if the read was successful,
 *              an error code otherwise.
 *
 */
cy_rslt_t app_bt_bond_read_local_identity_keys(void)
{
    uint32_t data_size = sizeof(identity_keys);
    cy_rslt_t rslt = mtb_kvstore_read(&kvstore_obj,
                                      "local_irk",
                                      (uint8_t *)&identity_keys,
                                      &data_size);
    if (rslt != CY_RSLT_SUCCESS)
    {
        printf("New Keys need to be generated! \r\n");
    }
    else
    {
        printf("Identity keys are available in the database.\r\n");
        printf("Local identity keys read from Flash: \r\n");
    }
    return rslt;
}

/*******************************************************************************
 *              CCCD Management APIs
 ******************************************************************************/

/**
 * Function Name:
 * app_bt_bond_find_cccd_bit
 *
 * Function Description:
 * @brief This function finds the bit position of the respective CCCD attribute
 * handle
 *
 * @param   attr_handle: CCCD attribute handle
 *
 * @return  uint8_t: bit position from LSB
 */
static uint8_t app_bt_bond_find_cccd_bit(uint16_t attr_handle)
{
    uint8_t position = 0;
    switch (attr_handle)
    {
        case HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG:
            position = 0;
            break;

        case HDLD_HIDS_KBD_IN_REPORT_CLIENT_CHAR_CONFIG:
            position = 1;
            break;

        case HDLD_HIDS_CC_IN_REPORT_CLIENT_CHAR_CONFIG:
            position = 2;
            break;

        default:
            printf("Unknown CCCD Attr Handle: 0x%x \r\n", attr_handle);
            break;
    }

    return position;
}

/**
 * Function Name:
 * app_bt_bond_modify_cccd_in_nv_storage
 *
 * Function Description:
 * @brief This function writes 0 or 1 the respective bits for a particular CCCD
 * and writes to NV Storage
 *
 * @param attr_handle: attr_handle of the CCCD
 * @param p_val: Pointer to the CCCD attribute
 *
 * @return void
 */
void app_bt_bond_modify_cccd_in_nv_storage(uint16_t attr_handle, uint8_t *p_val)
{

    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;
    /* Position refers to the Bit position from the LSB */
    uint8_t position = 0;
    position = app_bt_bond_find_cccd_bit(attr_handle);

    // printf("bondinfo.cccd_flags Value Before: 0x%x\r\n", bondinfo.cccd_flags[bondindex]);
    if (0x0001 == *p_val)
    {
        printf("Notification is enabled\r\n");
        SET_CCCD_BIT(bondinfo.cccd_flags[bondindex], position);
    }
    else if (0x0000 == *p_val)
    {
        printf("Notification is disabled\r\n");
        CLEAR_CCCD_BIT(bondinfo.cccd_flags[bondindex], position);
    }
    // printf("bondinfo.cccd_flags Value After: 0x%x\r\n", bondinfo.cccd_flags[bondindex]);

    rslt = mtb_kvstore_write(&kvstore_obj,
                            "bondinfo",
                            (uint8_t *)&bondinfo,
                            sizeof(bondinfo));

    if (rslt == CY_RSLT_TYPE_ERROR)
    {
        printf("KVStor write error\r\n");
    }
}

/**
 * Function Name:
 * app_bt_bond_restore_cccd
 *
 * Function Description:
 * @brief This function restores the bondinfo cccd_flags values to the actual GATT DB attribute
 *
 * @param void
 *
 * @return cy_rslt_t: CY_RSLT_SUCCESS if the update was successful,
 *              an error code otherwise.
 */
cy_rslt_t app_bt_bond_restore_cccd(void)
{
    for (uint8_t i = 0; i < NUM_OF_CCCD; i++)
    {
        if (IS_THIS_BIT_SET(bondinfo.cccd_flags[bondindex], i))
        {
            printf("Position %d is set\r\n", i);
            if( i == 0)
            {
                app_bas_battery_level_client_char_config[0] = 0x01;
            }
            else if (i == 1)
            {
                app_hids_kbd_in_report_client_char_config[0] = 0x01;
            }
            else
            {
                app_hids_cc_in_report_client_char_config[0] = 0x01;
            }
        }
    }

    printf("Found the device. Revoking CCCD status\r\n");
    printf("cccd_flags value: 0x%x \r\n", bondinfo.cccd_flags[bondindex]);

    return TRUE;
}

/*******************************************************************************
 *              Helper APIs
 ******************************************************************************/
/**
 * Function Name:
 * app_bt_bond_print_data
 *
 * Function Description:
 * @brief This function prints the bond data stored in the Flash
 *
 * @param void
 *
 * @return void
 *
 */
void app_bt_bond_print_data(void)
{
    for (uint8_t i = 0; i < bondinfo.slot_data[NUM_BONDED]; i++)
    {
        printf("Slot: %d", i+1);
        printf(" Device Bluetooth Address: ");
        app_bt_util_print_bd_address(bondinfo.link_keys[i].bd_addr);
        printf("Link Keys: \n");
        app_bt_util_print_byte_array(&(bondinfo.link_keys[i].key_data), sizeof(wiced_bt_device_sec_keys_t));
        printf("\n");
    }
}

/**
 * Function Name:
 * app_bt_bond_print_info_stats
 *
 * Function Description:
 * @brief Prints the status of bonding information in the NV storage.
 *
 * @param void
 *
 * @return void
 */
void app_bt_bond_print_info_stats(void)
{
    printf( "\r\nNumber of bonded devices: %d, "
            "\r\nNumber of slots free: %d\r\n",
            bondinfo.slot_data[NUM_BONDED],
            (BOND_MAX - bondinfo.slot_data[NUM_BONDED]) );
}

/**
* Function Name:
* app_bt_bond_save_os_mode
*
* Function Description:
* @brief This function saves os mode 
*
* @param os_mode: windows , ios or android
*
* @return cy_rslt_t: CY_RSLT_SUCCESS if the save was successful,
*              an error code otherwise.
*
*/
cy_rslt_t app_bt_bond_save_os_mode(uint8_t os_mode)
{
    cy_rslt_t rslt = CY_RSLT_TYPE_ERROR;

    bondinfo.os_mode[bondindex] = os_mode;

    rslt = mtb_kvstore_write(&kvstore_obj,
                            "bondinfo",
                            (uint8_t *)&bondinfo,
                            sizeof(bondinfo));
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Flash Write Error,Error code: %" PRIu32 "\r\n", rslt );
    }
    return rslt;
}
