/******************************************************************************
 * File Name:   app_bt_bonding.h
 *
 * Description: This is the header file for the Bluetooth Bonding API Interface.
 *
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
#ifndef __APP_BT_BONDING_H_
#define __APP_BT_BONDING_H_

/*******************************************************************************
 *                           Header Files
 *******************************************************************************/
#include "wiced_bt_stack.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg_bt_settings.h"

/*******************************************************************************
 *                           Macro Definitions
 *******************************************************************************/
/* Max number of bonded devices */
#define  BOND_MAX                            (3u)
#define  NUM_OF_SLOTS                        (2u)
#define  NUM_OF_CCCD                         (3u)
/* LE Key Size : 16 */
#define  KEY_SIZE_MAX                        (0x10)

/* enum for bondinfo structure */
enum
{
    NUM_BONDED,
    LAST_CONNECT_INDEX
};
/*******************************************************************************
 *                          Global Variables
 ******************************************************************************/
/* Structure to store info that goes into serial flash -
 * it holds the number of bonded devices, remote keys and local keys
 */
typedef struct
{
    uint8_t slot_data[NUM_OF_SLOTS];
    uint8_t os_mode[BOND_MAX];
    wiced_bt_device_link_keys_t link_keys[BOND_MAX];
    wiced_bt_device_address_t local_bd_addr[BOND_MAX];
    wiced_bt_device_address_t next_local_bd_addr;
    /* Variable to store CCCD Data
        Position 0 - bas_in_cccd
        Position 1 - kbd_in_cccd
        Position 2 - cc_in_cccd
    */
    uint8_t cccd_flags[BOND_MAX];
} tBondInfo;

/* Structure containing bonding info of peer devices */
extern tBondInfo bondinfo;

/* Local Identity Key */
extern wiced_bt_local_identity_keys_t identity_keys;

/* Array Index of the current peer device */
extern uint8_t bondindex;

/*******************************************************************************
 *                      Function Prototypes
 ******************************************************************************/

/* Peer Device and Bond Info Management APIs */
cy_rslt_t app_bt_bond_restore_data(void);
cy_rslt_t app_bt_bond_update_slot_data(void);
cy_rslt_t app_bt_bond_update_data(void);
uint8_t app_bt_bond_check_info(void);
uint8_t app_bt_bond_get_info_count(void);
cy_rslt_t app_bt_bond_get_new_bd_addr(uint8_t *device_addr);
cy_rslt_t app_bt_bond_get_local_bd_addr(uint8_t *device_addr);
cy_rslt_t app_bt_bond_update_local_bd_addr(uint8_t *device_addr);
cy_rslt_t app_bt_bond_update_index(uint8_t device_no);
cy_rslt_t app_bt_bond_delete_info(void);
wiced_result_t app_bt_bond_check_device_info(uint8_t *bd_addr);

/* Security Keys Management APIs */
cy_rslt_t app_bt_bond_get_device_link_keys(wiced_bt_device_link_keys_t *link_key);
cy_rslt_t app_bt_bond_save_device_link_keys(wiced_bt_device_link_keys_t *link_key);
cy_rslt_t app_bt_bond_save_local_identity_key(wiced_bt_local_identity_keys_t id_key);
cy_rslt_t app_bt_bond_read_local_identity_keys(void);

/* CCCD Management APIs */
void app_bt_bond_modify_cccd_in_nv_storage(uint16_t attr_handle, uint8_t *p_val);
cy_rslt_t app_bt_bond_restore_cccd(void);

/* Helper APIs */
void app_bt_bond_print_data(void);
void app_bt_bond_print_info_stats(void);

/* NVM storage APIs */
cy_rslt_t app_bt_bond_save_os_mode(uint8_t os_mode);
cy_rslt_t app_bt_save_channel(uint8_t channel);

#endif // __APP_BT_BONDING_H_

/* [] END OF FILE */
