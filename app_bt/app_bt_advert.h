/*******************************************************************************
 * File Name: app_bt_advert.h
 *
 * Description: This File provides the interfaces necessary for Bluetooth LE
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

#ifndef __APP_BT_ADVERT_H__
#define __APP_BT_ADVERT_H__
/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>

#include "cycfg_gap.h"
#include "FreeRTOS.h"
#include "timers.h"

/*******************************************************************************
 *                      MACROS / VARIABLE DEFINITIONS
 ******************************************************************************/
/* Number of advertisement frames */
#define NUM_ADV_ELEM            (CY_BT_ADV_PACKET_DATA_SIZE)

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
/* This function initializes advertisement data and pairable mode */
void app_bt_adv_start(void);

/* This function handles the application state based on the advertisement state change */
void app_bt_adv_state_handler(wiced_bt_ble_advert_mode_t current_adv_mode);

/* This Function starts undirected Bluetooth LE advertisement for reconnection to known host */
void app_bt_adv_start_known_host(void);

/* This Function starts undirected Bluetooth LE advertisement for pairing to new host */
void app_bt_adv_start_any_host(void);

/* This Function stops ongoing Bluetooth LE advertisement */
void app_bt_adv_stop(void);

/* This Function is used to switch device to pairing mode */
void app_bt_adv_pairing_mode_switch(void);

/* This Function is used to switch the current bond index */
void app_bt_adv_bond_index_switch(uint8_t device_channel);

/* This Function stops ongoing Bluetooth LE advertisement */
void app_bt_adv_stop(void);


#endif // __APP_BT_ADVERT_H__