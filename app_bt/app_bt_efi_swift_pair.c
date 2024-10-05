/*******************************************************************************
 * File Name: app_bt_efi_swift_pair.c
 *
 * Description: This File provides the implementations necessary for Bluetooth
 * Advertisements.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "app_bt_advert.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_hid.h"

#include "app_handler.h"

 /*******************************************************************************
  *                      MACROS / VARIABLE DEFINITIONS
  ******************************************************************************/
  /* Number of advertisement frames */
#define NUM_ADV_ELEM            (CY_BT_ADV_PACKET_DATA_SIZE)

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/


 /*******************************************************************************
  *                              FUNCTION DEFINITIONS
  ******************************************************************************/

  /**
   * Function Name:
   * app_bt_efi_swift_pair_set_adv_data
   *
   * Function Description:
   * @brief   Function used to set LE Advertisement Data for swift pairing 
   *
   * @param   adv_flag : Adv mode flag
   *          is_cool_down_adv : True is case of cool down phase , False otherwise
   *
   * @return  void
   */
static void app_bt_efi_swift_pair_set_adv_data(uint8_t adv_flag , bool is_cool_down_adv)
{
    uint8_t cy_bt_adv_packet_elem_0[1] = { adv_flag };

    /* Set the adv flag */
    cy_bt_adv_packet_data[0].p_data = (uint8_t*)cy_bt_adv_packet_elem_0;

    /* Set Advertisement data */
    if (WICED_SUCCESS != wiced_bt_ble_set_raw_advertisement_data((is_cool_down_adv ? (NUM_ADV_ELEM - 1) : (NUM_ADV_ELEM)), cy_bt_adv_packet_data))
    {
        printf("Setting advertisement data Failed\r\n");
    }
}

/**
 * Function name:
 * app_bt_efi_swift_pair_start_adv
 *
 * Function Description:
 * @brief    Function used to start connectable advertisements for swift pairing once
 * BTM_ENABLED_EVT event occurs in Bluetooth management callback
 *
 * @param    adv_flag : Adv mode flag
 *           is_cool_down_adv : True is case of cool down phase , False otherwise
 *
 * @return   void
 */
void app_bt_efi_swift_pair_start_adv(uint8_t adv_flag ,bool is_cool_down_adv)
{
    /* Set Advertisement Data */
    app_bt_efi_swift_pair_set_adv_data(adv_flag, is_cool_down_adv );

    /* Start Undirected LE Advertisements */
    if (WICED_SUCCESS != wiced_bt_start_advertisements((is_cool_down_adv ? (BTM_BLE_ADVERT_UNDIRECTED_LOW) : (BTM_BLE_ADVERT_UNDIRECTED_HIGH)),
                                                       BLE_ADDR_PUBLIC,
                                                       NULL))
    {
        printf("Starting undirected Bluetooth LE advertisements Failed\r\n");
    }
}
