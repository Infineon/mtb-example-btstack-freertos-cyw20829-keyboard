/*******************************************************************************
 * File Name: app_bt_gatt_handler.h
 *
 * Description: This file consists of the function prototypes that are
 *              necessary for developing the BLE applications with GATT Server
 *              callbacks.
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

#ifndef __APP_BT_GATT_HANDLER_H__
#define __APP_BT_GATT_HANDLER_H__

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "wiced_bt_gatt.h"
#include "wiced_bt_l2c.h"
#include "wiced_memory.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
/* The error code for invalid  attribute index in attribute table */
#define INVALID_ATT_TBL_INDEX                   (255u)

/* Connection Parameters */
/* Minimum Connection Interval */
#define MIN_CI  ( (app_gap_peripheral_preferred_connection_parameters[1] << 8) \
                | (app_gap_peripheral_preferred_connection_parameters[0]) )
/* Maximum Connection Interval */
#define MAX_CI  ( (app_gap_peripheral_preferred_connection_parameters[3] << 8) \
                | (app_gap_peripheral_preferred_connection_parameters[2]) )
/* Slave Latency */
#define SLAVE_LATENCY   ( (app_gap_peripheral_preferred_connection_parameters[5] << 8) \
                        | (app_gap_peripheral_preferred_connection_parameters[4]) )
/* Supervision Timeout */
#define SUPERVISION_TO  ( (app_gap_peripheral_preferred_connection_parameters[7] << 8) \
                        | (app_gap_peripheral_preferred_connection_parameters[6]) )

#define CONN_PARAM_UPDATE_TIMER_DELAY   1000
#define CONN_PARAM_UPDATE_RETRY_COUNT   10

/*******************************************************************************
 *                              VARIABLES
 ******************************************************************************/

/* A Global variable to check the status of this device if it is
 * connected to any peer devices*/
extern uint16_t app_bt_conn_id;

/* Number of GATT congestions during audio transfer */
extern uint16_t num_of_congestions;

/* connection parameter updated flag to avoid requesting the connection
 * parameters more than once
 */
extern uint8_t conn_param_updated_flag;

/* Negotiated MTU Size */
extern uint16_t preferred_mtu_size;

/* BD Address of the connected Peer device */
extern wiced_bt_device_address_t peer_bd_addr;
/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
wiced_bt_gatt_status_t
app_bt_gatt_attr_write_handler(wiced_bt_gatt_event_data_t *p_req, uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_bt_gatt_attr_read_handler( uint16_t conn_id,
                            wiced_bt_gatt_opcode_t opcode,
                            wiced_bt_gatt_read_t *p_read_req,
                            uint16_t len_req,
                            uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_bt_gatt_read_by_type_handler(uint16_t conn_id,
                                wiced_bt_gatt_opcode_t opcode,
                                wiced_bt_gatt_read_by_type_t *p_read_data,
                                uint16_t len_requested,
                                uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_bt_gatt_connection_status_change_cb(wiced_bt_gatt_connection_status_t *p_conn_status);

wiced_bt_gatt_status_t
app_bt_gatt_attr_request_cb(wiced_bt_gatt_event_data_t *p_data, uint16_t *p_error_handle);

wiced_bt_gatt_status_t
app_bt_gatt_event_handler(wiced_bt_gatt_evt_t event,
                          wiced_bt_gatt_event_data_t *p_event_data);

wiced_bt_gatt_status_t app_bt_gatt_set_attr_value( uint8_t attr_handle_index,
                                                uint8_t *p_val,
                                                uint16_t len,
                                                wiced_bt_gatt_opcode_t opcode);

uint8_t app_bt_gatt_get_index_by_handle(uint16_t attr_handle);

void app_bt_gatt_disable_all_cccds(void);

typedef void (*pfn_free_buffer_t)(uint8_t*);

#endif      /* __APP_BT_GATT_HANDLER_H__ */

/* [] END OF FILE */
