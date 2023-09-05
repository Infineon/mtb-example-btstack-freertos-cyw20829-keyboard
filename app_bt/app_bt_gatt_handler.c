/*******************************************************************************
 * File Name: app_bt_gatt_handler.c
 *
 * Description: This file consists of the function defintions that are
 *              necessary for developing the Bluetooth LE applications with GATT
 *              Server callbacks.
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

#include "cybt_platform_trace.h"
#include "wiced_bt_ble.h"

#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"
#include "app_bt_advert.h"
#include "app_bt_hid.h"
#include "app_handler.h"
#include "app_batmon.h"

#ifdef ENABLE_OTA
#include "app_ota_context.h"
#include "app_serial_flash.h"
#include "cy_ota_platform.h"
#include "cy_ota_api.h"

extern ota_app_context_t ota_app;
gatt_write_req_buf_t write_buff;
#endif

/*******************************************************************************
 *                      VARIABLE DEFINITIONS
 ******************************************************************************/
/* This variable tracks the number of congestion events during BLE data transfer */
uint16_t num_of_congestions = 0;

/* Flag to check for GATT congestion */
uint8_t is_gatt_congested;

/* Update this flag once connection parameter update is successful */
uint8_t conn_param_updated_flag = FALSE;
uint8_t conn_param_update_retry = 0;

/* MTU size negotiated between local and peer device */
uint16_t preferred_mtu_size = CY_BT_MTU_SIZE;

wiced_bt_device_address_t peer_bd_addr = { 0 };
extern TaskHandle_t ble_task_h;
extern TimerHandle_t ble_disconnection_timer;
extern TimerHandle_t conn_param_update_timer;
extern bool ota_started;



/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/

static void app_bt_gatt_free_buffer(uint8_t *p_event_data);

static uint8_t* app_bt_gatt_alloc_buffer(uint16_t len);

static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id,
                                                                 wiced_bt_gatt_opcode_t opcode,
                                                                 wiced_bt_gatt_read_multiple_req_t *p_read_req,
                                                                 uint16_t len_requested,
                                                                 uint16_t *p_error_handle);
static gatt_db_lookup_table_t *app_bt_gatt_find_by_handle(uint16_t handle);

extern wiced_result_t BTM_SetDataChannelPDULength(wiced_bt_device_address_t bd_addr, uint16_t tx_pdu_length);

#ifdef ENABLE_OTA
static wiced_bt_gatt_status_t app_bt_prepare_write_handler(uint16_t conn_id,
                                                           wiced_bt_gatt_opcode_t opcode,
                                                           wiced_bt_gatt_write_req_t *p_req,
                                                           uint16_t *p_error_handle);

static wiced_bt_gatt_status_t app_bt_execute_write_handler(wiced_bt_gatt_event_data_t *p_req, uint16_t *p_error_handle);
#endif

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 * Function Name:
 * app_bt_gatt_free_buffer
 *
 * Function Description:
 * @brief   This function frees up the buffer memory
 *
 * @param   p_data: Pointer to the buffer to be free
 *
 * @return  void
 */
static void app_bt_gatt_free_buffer(uint8_t *p_event_data)
{
    wiced_bt_free_buffer(p_event_data);
}

/**
 * Function Name:
 * app_bt_gatt_alloc_buffer
 *
 * Function Description:
 * @brief   This function allocates the memory from BT buffer pool.
 *
 * @param   len : Length of buffer to be allocated
 *
 * @return  uint8_t * : Pointer to the start address of allocated memory
 */
static uint8_t* app_bt_gatt_alloc_buffer(uint16_t len)
{
    uint8_t *p_mem = (uint8_t*)wiced_bt_get_buffer(len);
    if (!p_mem)
    {
        printf("OOM\r\n");
        CY_ASSERT(0);
    }

    return p_mem;
}

/**
 * Function Name:
 * app_bt_gatt_event_handler
 *
 * Function Description:
 * @brief   This Function handles the all the GATT events - GATT Event Handler
 *
 * @param   event:            Bluetooth LE GATT event type
 * @param   p_event_data:     Pointer to Bluetooth LE GATT event data
 *
 * @return  wiced_bt_gatt_status_t:  Bluetooth LE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_event_handler(wiced_bt_gatt_evt_t event,
                         wiced_bt_gatt_event_data_t *p_event_data)
{

    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    pfn_free_buffer_t pfn_free;

    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;
    uint16_t error_handle = 0;

    /*
     * Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event parameters to the callback function
     */
    switch (event)
    {

        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = app_bt_gatt_connection_status_change_cb(&p_event_data->connection_status);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = app_bt_gatt_attr_request_cb(p_event_data, &error_handle);

            if(gatt_status != WICED_BT_GATT_SUCCESS)
            {
               wiced_bt_gatt_server_send_error_rsp(p_attr_req->conn_id,
                                                   p_attr_req->opcode,
                                                   error_handle,
                                                   gatt_status);
            }

            break;

        case GATT_OPERATION_CPLT_EVT:
            printf("GATT_OPERATION_CPLT_EVT\r\n");
            break;

        case GATT_CONGESTION_EVT:
            num_of_congestions++;
            is_gatt_congested = (p_event_data->congestion.congested) ? true : false;

            if (!is_gatt_congested)
            {
                xTaskNotify(ble_task_h, 0, eNoAction);
            }

            break;

        case GATT_GET_RESPONSE_BUFFER_EVT:
            printf("GATT_GET_RESPONSE_BUFFER_EVT len_req %d \r\n",
                   p_event_data->buffer_request.len_requested);

            p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_gatt_alloc_buffer(
                    p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void*)app_bt_gatt_free_buffer;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to
             * free it.
             */
            if (pfn_free)
            {
                pfn_free(p_event_data->buffer_xmitted.p_app_data);
            }
            break;

        default:
            printf("ERROR: Unhandled GATT event: %d \r\n", event);
            break;

    }

    if (WICED_BT_GATT_SUCCESS != gatt_status)
    {
        printf("\nGATT event status: %d \t", gatt_status);
        //printf("\nFor the ATT handle: 0x%x\r\n", p_attr_req->data.read_req.handle);
        printf("\r\n");
    }

    return gatt_status;
}

/**
 * Function Name:
 * app_bt_gatt_connection_status_change_cb
 *
 * Function Description:
 * @brief  The callback function is invoked when GATT_CONNECTION_STATUS_EVT occurs
 *         in GATT Event handler function
 *
 * @param   p_conn_status:     Pointer to Bluetooth LE GATT connection status
 *
 * @return  wiced_bt_gatt_status_t:  Bluetooth LE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_connection_status_change_cb(wiced_bt_gatt_connection_status_t *p_conn_status)
{

    wiced_result_t gatt_status = WICED_BT_GATT_SUCCESS;

    if ((p_conn_status->connected) && (0 == app_bt_conn_id))
    {
        BTM_SetDataChannelPDULength(p_conn_status->bd_addr, CY_BT_RX_PDU_SIZE);

        /* Device has connected */
        printf("Connected to BDA:\r\n");
        app_bt_util_print_bd_address(p_conn_status->bd_addr);
        printf("Connection ID: '%d'\r\n", p_conn_status->conn_id);
        printf("Peer device addr type : %d\r\n", p_conn_status->addr_type);

        app_bt_conn_id = p_conn_status->conn_id;
#ifdef ENABLE_OTA
        ota_app.bt_conn_id = p_conn_status->conn_id;
        memcpy(ota_app.bt_peer_addr, p_conn_status->bd_addr, BD_ADDR_LEN);
#endif

        /* Start Disconnection timer */
        if (pdFAIL == xTimerStart(ble_disconnection_timer, TIMER_MAX_WAIT))
        {
            printf("Failed to start Disconnection Timer\r\n");
        }

        /* Stop LED Blink */
        app_status_led_stop_blinking();

        /* reset the key count buffer */
        app_reset_key_cnt();
        Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP);

    }
    else
    {
        /* Device has disconnected */
        printf("Disconnected to BDA:\r\n");
        app_bt_util_print_bd_address(p_conn_status->bd_addr);
        printf("Connection ID: '%d'\r\n", p_conn_status->conn_id);
        printf("\nReason for disconnection: %d \t", p_conn_status->reason);
        printf(app_bt_util_get_gatt_disconn_reason_name(p_conn_status->reason));
        printf("\r\n");

        /* Handle the disconnection */
        app_bt_conn_id = 0;
        /* Reset flag to start connection param update again on next connection event */
        conn_param_updated_flag = FALSE;
        conn_param_update_retry = 0;

        /* Stop Disconnection timer */
        if (pdFAIL == xTimerStop(ble_disconnection_timer, TIMER_MAX_WAIT))
        {
            printf("Failed to stop Disconnection Timer\r\n");
        }
        /* Stop Connection parameter update timer */
        if (pdFAIL == xTimerStop(conn_param_update_timer, TIMER_MAX_WAIT))
        {
            printf("Failed to stop Connection param update Timer\r\n");
        }


        /* Stop LED Blink */
        app_status_led_off();

        /*
         * Reset the CCCD value so that on a reconnect CCCD (notifications)
         * will be off. For HID Bonded devices, CCCDs has to be retained.
         */
        app_bt_gatt_disable_all_cccds();

        /* clear the notifications buffer */
        app_clear_ble_hid_kbd_buffer();
        app_clear_hid_cc_buffer();

        key_state_cnt = 0;

        /* Update the state to IDLE and wait for user events to trigger adv again */
        if (app_bt_hid_get_device_state() == PAIRED_ADVERTISING_ANY_HOST)
        {
            app_bt_adv_start_any_host();
        }
        else if (app_bt_hid_get_device_state() == PAIRED_ADVERTISING_KNOWN_HOST)
        {
            app_bt_adv_start_known_host();
        }
        else if (0 != app_bt_bond_get_info_count())
        {
            /* Update the state to IDLE and wait for user events to trigger adv again */
            app_bt_hid_update_device_state(PAIRED_IDLE);
        }
        else
        {
            /* Update the state to IDLE and wait for user events to trigger adv again */
            app_bt_hid_update_device_state(UNPAIRED_IDLE);
        }
        Cy_SysPm_SetDeepSleepMode(CY_SYSPM_MODE_DEEPSLEEP_RAM);

    }

    return gatt_status;
}

/**
 * Function Name:
 * app_bt_gatt_attr_request_cb
 *
 * Function Description:
 * @brief  The callback function is invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs
 *         in GATT Event handler function. GATT Server Event Callback function.
 *
 * @param   p_data:   GATT Request attribute  Pointer to GATT attribute Request structure
 *
 * @return  wiced_bt_gatt_status_t:  BLE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_attr_request_cb(wiced_bt_gatt_event_data_t *p_data, uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_data->attribute_request;

    switch (p_attr_req->opcode)
    {

        case GATT_REQ_FIND_TYPE_VALUE:
        case GATT_REQ_READ_BY_TYPE:
            gatt_status = app_bt_gatt_read_by_type_handler(p_attr_req->conn_id,
                                                        p_attr_req->opcode,
                        (wiced_bt_gatt_read_by_type_t*)&p_attr_req->data.read_req,
                                                        p_attr_req->len_requested,
                                                        p_error_handle);
            break;

        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
        case GATT_REQ_READ_BY_GRP_TYPE:
            gatt_status = app_bt_gatt_attr_read_handler(p_attr_req->conn_id,
                                                     p_attr_req->opcode,
                             (wiced_bt_gatt_read_t*)&p_attr_req->data.read_req,
                                                     p_attr_req->len_requested,
                                                     p_error_handle);
            break;

        case GATT_REQ_READ_MULTI:
        case GATT_REQ_READ_MULTI_VAR_LENGTH:
            gatt_status = app_bt_gatt_req_read_multi_handler(p_attr_req->conn_id,
                                                             p_attr_req->opcode,
                                                             &p_attr_req->data.read_multiple_req,
                                                             p_attr_req->len_requested,
                                                             p_error_handle);
            break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            gatt_status = app_bt_gatt_attr_write_handler(p_data, p_error_handle);
            if ((p_attr_req->opcode == GATT_REQ_WRITE) && (gatt_status == WICED_BT_GATT_SUCCESS))
            {
                wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                    p_attr_req->opcode,
                                                    p_attr_req->data.write_req.handle);
            }
            break;

        case GATT_REQ_PREPARE_WRITE:
#ifdef ENABLE_OTA
            printf("GATT_REQ_PREPARE_WRITE \r\n");
                    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "  %s() GATT_REQ_PREPARE_WRITE\n", __func__);
                    gatt_status = app_bt_prepare_write_handler(p_attr_req->conn_id,
                                                          p_attr_req->opcode,
                                                          &p_attr_req->data.write_req,
                                                          p_error_handle);
                    if((p_attr_req->opcode == GATT_REQ_PREPARE_WRITE) &&  (gatt_status != WICED_BT_GATT_SUCCESS))
                    {
                        cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "\n\n== Sending Prepare write error response...\n");
                    }
#else
                    gatt_status = WICED_BT_GATT_SUCCESS;
#endif
            break;

        case GATT_REQ_EXECUTE_WRITE:

#ifdef ENABLE_OTA
            printf("GATT_REQ_EXECUTE_WRITE \r\n");
                    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "  %s() GATTS_REQ_TYPE_WRITE_EXEC\n", __func__);
                    gatt_status = app_bt_execute_write_handler(p_data, p_error_handle);
                    if((p_attr_req->opcode == GATT_REQ_EXECUTE_WRITE) &&  (gatt_status == WICED_BT_GATT_SUCCESS))
                    {
                        cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "== Sending execute write success response...\n");
                        wiced_bt_gatt_server_send_execute_write_rsp(p_attr_req->conn_id, p_attr_req->opcode);
                        gatt_status = WICED_BT_GATT_SUCCESS;
                    }
                    else
                    {
                        cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "== Sending execute write error response...\n");
                    }
#else
                     wiced_bt_gatt_server_send_execute_write_rsp(p_attr_req->conn_id, p_attr_req->opcode);
#endif
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_REQ_MTU:
            printf("\rClient MTU Req: %d\r\n", p_attr_req->data.remote_mtu);
            preferred_mtu_size = CY_BT_MTU_SIZE <= (p_attr_req->data.remote_mtu) ?
                                    CY_BT_MTU_SIZE : (p_attr_req->data.remote_mtu);
            gatt_status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                                            p_attr_req->data.remote_mtu,
                                                            preferred_mtu_size);
            printf("MTU Response status %d\r\n", gatt_status);
            break;

        case GATT_HANDLE_VALUE_CONF:           /* Value confirmation */
#ifdef ENABLE_OTA
            printf("GATT_HANDLE_VALUE_CONF \r\n");
            cy_ota_agent_state_t ota_lib_state;
            cy_ota_get_state(ota_app.ota_context, &ota_lib_state);
            if ( (ota_lib_state == CY_OTA_STATE_OTA_COMPLETE) &&        /* Check if we completed the download before rebooting */
                 (ota_app.reboot_at_end != 0) )
            {
                cy_rtos_delay_milliseconds(1000);
                Cy_SysPm_TriggerXRes();
            }
            else
            {
                cy_ota_agent_stop(&ota_app.ota_context);    /* Stop OTA */
            }
#endif
            break;

        case GATT_HANDLE_VALUE_NOTIF:
            break;

        case GATT_RSP_ERROR:
            printf("GATT Response Error\r\n");
            break;

        default:
            printf("ERROR: Unhandled GATT Connection Request case: %d\r\n", p_attr_req->opcode);
            break;
    }

    return gatt_status;
}

/**
 * Function Name:
 * app_bt_gatt_attr_write_handler
 *
 * Function Description:
 * @brief  The function is invoked when GATTS_REQ_TYPE_WRITE is received from the
 *         client device and is invoked GATT Server Event Callback function. This
 *         handles "Write Requests" received from Client device.
 *
 * @param   p_req:   Pointer to BLE GATT write request
 *
 * @return  wiced_bt_gatt_status_t:  BLE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_attr_write_handler(wiced_bt_gatt_event_data_t *p_req, uint16_t *p_error_handle)
{

#ifdef ENABLE_OTA
    cy_rslt_t   result;
#endif
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_req->attribute_request;
    wiced_bt_gatt_write_req_t       *p_write_req = &p_req->attribute_request.data.write_req;
    uint8_t index = 0;
    CY_ASSERT(p_req != NULL);

    *p_error_handle = p_write_req->handle;

    index = app_bt_gatt_get_index_by_handle(p_write_req->handle);
#ifdef ENABLE_OTA

           switch(p_write_req->handle)
           {
           /*
            * If write request is for the OTA FW upgrade service, pass it to the
            * library to process
            */
           case HDLD_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_CONTROL_POINT_CLIENT_CHAR_CONFIG:
               ota_app.bt_config_descriptor = p_write_req->p_val[0];
               gatt_status = WICED_BT_GATT_SUCCESS;
               break;

           case HDLC_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_CONTROL_POINT_VALUE:
               switch(p_write_req->p_val[0])
               {
               case CY_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD:
                   ota_app.connection_type = CY_OTA_CONNECTION_BLE;
                   ota_started = 1;
                   xTimerStop(conn_param_update_timer, TIMER_MAX_WAIT);
                   result = init_ota(&ota_app);
                   if (result != CY_RSLT_SUCCESS)
                   {
                       gatt_status= WICED_BT_GATT_ERROR;
                       break;
                   }
                   printf("Preparing to download the image \r\n");
                   result = cy_ota_ble_download_prepare(ota_app.ota_context, ota_app.bt_conn_id, ota_app.bt_config_descriptor);
                   if (result != CY_RSLT_SUCCESS)
                   {
                       gatt_status= WICED_BT_GATT_ERROR;
                       break;
                   }
                   gatt_status = WICED_BT_GATT_SUCCESS;
                   break;

               case CY_OTA_UPGRADE_COMMAND_DOWNLOAD:
                   result = cy_ota_ble_download(ota_app.ota_context, p_req, ota_app.bt_conn_id, ota_app.bt_config_descriptor);
                   if (result != CY_RSLT_SUCCESS)
                   {
                       gatt_status = WICED_BT_GATT_ERROR;
                       break;
                   }
                   gatt_status = WICED_BT_GATT_SUCCESS;
                   break;

               case CY_OTA_UPGRADE_COMMAND_VERIFY:
                   result = cy_ota_ble_download_verify(ota_app.ota_context, p_req, ota_app.bt_conn_id);
                   if (result != CY_RSLT_SUCCESS)
                   {
                       gatt_status = WICED_BT_GATT_ERROR;
                       break;
                   }
                   gatt_status = WICED_BT_GATT_SUCCESS;
                   break;

               case CY_OTA_UPGRADE_COMMAND_ABORT:
                   ota_started = 0;
                   result = cy_ota_ble_download_abort(&ota_app.ota_context);
                   gatt_status = WICED_BT_GATT_SUCCESS;
                   break;
               }
               break;

           case HDLC_OTA_FW_UPGRADE_SERVICE_OTA_UPGRADE_DATA_VALUE:
               printf("application downloading... \r\n");
               result = cy_ota_ble_download_write(ota_app.ota_context, p_req);
               if (result != CY_RSLT_SUCCESS)
               {
                   gatt_status = WICED_BT_GATT_ERROR;
                   break;
               }
               gatt_status = WICED_BT_GATT_SUCCESS;
               break;

           default:
                      /* Handle normal (non-OTA) indication confirmation requests here */
                      /* Attempt to perform the Write Request */
                      return app_bt_gatt_set_attr_value(index,
                                                   p_write_req->p_val,
                                                    p_write_req->val_len,
                                                    p_attr_req->opcode);
                 }

#else
                  if (INVALID_ATT_TBL_INDEX != index)
                   {
                       /* Validate the length of the attribute and write to the attribute */
                       if (WICED_BT_GATT_SUCCESS != app_bt_gatt_set_attr_value(index,
                                                                            p_write_req->p_val,
                                                                            p_write_req->val_len,
                                                                            p_attr_req->opcode))
                       {
                           printf("WARNING: GATT set attr status 0x%x\r\n", gatt_status);
                       }

                   }
                   else
                   {
                       gatt_status = WICED_BT_GATT_INVALID_HANDLE;
                       printf("Invalid ATT TBL Index : %d\r\n", index);
                   }
#endif
    return (gatt_status);
}

/**
 * Function Name:
 * app_bt_gatt_attr_read_handler
 *
 * Function Description:
 * @brief  The function is invoked when GATTS_REQ_TYPE_READ is received from the
 *         client device and is invoked by GATT Server Event Callback function.
 *         This handles "Read Requests" received from Client device
 *
 * @param   conn_id:  Connection id
 * @param   opcode:   GATT Opcode
 * @param   p_read_req: Pointer to BLE GATT read request
 * @param   len_req: Length of the attribute requested by the Peer device
 *
 * @return  wiced_bt_gatt_status_t:  BLE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_attr_read_handler( uint16_t conn_id,
                            wiced_bt_gatt_opcode_t opcode,
                            wiced_bt_gatt_read_t *p_read_req,
                            uint16_t len_req,
                            uint16_t *p_error_handle)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;
    uint16_t valid_end_handle = HDLD_HIDS_KBD_OUT_REPORT_REPORT_REFERENCE;
    uint8_t index = 0;
    uint16_t attr_len_to_copy, to_send;
    uint8_t *from;

    *p_error_handle = p_read_req->handle;

    // printf("Reading Handle: 0x%x\r\n", (unsigned int)p_read_req->handle);
    index = app_bt_gatt_get_index_by_handle((p_read_req->handle));
    if (INVALID_ATT_TBL_INDEX != index)
    {
        /* Validate the length of the attribute */
        attr_len_to_copy = app_gatt_db_ext_attr_tbl[index].cur_len;

        if (p_read_req->offset >= app_gatt_db_ext_attr_tbl[index].cur_len)
        {
            return WICED_BT_GATT_INVALID_OFFSET;
        }

        to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
        from = app_gatt_db_ext_attr_tbl[index].p_data + p_read_req->offset;
        /* Send the attribute response */
        gatt_status = wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */

    }
    else if (HDLS_GAP == p_read_req->handle)
    {
        gatt_status = wiced_bt_gatt_server_send_read_handle_rsp(conn_id,
                                                                opcode,
                                                                0,
                                                                NULL,
                                                                NULL);
        gatt_status = WICED_BT_GATT_SUCCESS;
    }
    else if (valid_end_handle > p_read_req->handle)
    {
        printf("Invalid ATT TBL Index : %d\r\n", index);
        gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    }

    return (gatt_status);

}

/**
 * Function Name:
 * app_bt_gatt_find_by_handle
 *
 * Function Description:
 * @brief  The function is used to find attribute location by handle
 *
 * @param   handle: GATT handle to look up
 *
 * @return  gatt_db_lookup_table_t: pointer to location containing handle data
 */
static gatt_db_lookup_table_t* app_bt_gatt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/**
 * Function Name:
 * app_bt_gatt_read_by_type_handler
 *
 * Function Description:
 * @brief   This function handles the GATT read by type request events from the stack
 *
 * @param   conn_id: Connection ID
 * @param   opcode: BLE GATT request type opcode
 * @param   p_read_data: Pointer to read request containing the handle to read
 * @param   len_requested: Length requested
 *
 * @return wiced_bt_gatt_status_t: GATT result
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_read_by_type_handler(uint16_t conn_id,
                                wiced_bt_gatt_opcode_t opcode,
                                wiced_bt_gatt_read_by_type_t *p_read_data,
                                uint16_t len_requested,
                                uint16_t *p_error_handle)
{
    uint16_t    attr_handle = p_read_data->s_handle;
    uint8_t     *p_rsp = app_bt_gatt_alloc_buffer(len_requested);
    uint8_t     pair_len = 0;
    uint8_t     index = 0;
    int         used = 0;
    int         filled = 0;

    printf("len_requested %d \r\n", len_requested);
    if (p_rsp == NULL)
    {
        printf("len_requested %d \r\n", len_requested);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_data->e_handle,
                                                        &p_read_data->uuid);
        *p_error_handle = attr_handle;

        if (attr_handle == 0)
            break;

        index = app_bt_gatt_get_index_by_handle(attr_handle);
        if (INVALID_ATT_TBL_INDEX != index)
        {
            if ((used + app_gatt_db_ext_attr_tbl[index].cur_len) > len_requested)
            {
                break;
            }
            printf("attr_handle %x \r\n", attr_handle);
            filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used,
                                                                  len_requested - used,
                                                                  &pair_len,
                                                                  attr_handle,
                                                                  app_gatt_db_ext_attr_tbl[index].cur_len,
                                                                  app_gatt_db_ext_attr_tbl[index].p_data);
            printf("filled %d\r\n", filled);
            if (filled == 0)
            {
                printf("No data is filled\r\n");
                break;
            }
            used += filled;
        }
        else
        {
            printf("free buffer resp %x \r\n", p_read_data->s_handle );
            app_bt_gatt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }
        /* Increment starting handle for next search to one past current */
        attr_handle++;
    } // End of adding the data to the stream

    if (used == 0)
    {
        printf("free buffer resp 1 %x \r\n", p_read_data->s_handle );
        app_bt_gatt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp( conn_id,
                                                opcode,
                                                pair_len,
                                                used,
                                                p_rsp,
                            (wiced_bt_gatt_app_context_t)app_bt_gatt_free_buffer);

    return WICED_BT_GATT_SUCCESS;

}

/**
 * Function Name:
 * app_bt_gatt_req_read_multi_handler
 *
 * Function Description:
 * @brief   This function handles read multi request from peer device
 *
 * @param   conn_id: Connection ID
 * @param   opcode: BLE GATT request type opcode
 * @param   p_read_data: Pointer to read request containing the handle to read
 * @param   len_requested: Length of data requested
 *
 * @return wiced_bt_gatt_status_t: BLE GATT status
 *
 */
static wiced_bt_gatt_status_t app_bt_gatt_req_read_multi_handler(uint16_t conn_id,
                                                                 wiced_bt_gatt_opcode_t opcode,
                                                                 wiced_bt_gatt_read_multiple_req_t *p_read_req,
                                                                 uint16_t len_requested,
                                                                 uint16_t *p_error_handle)
{
    gatt_db_lookup_table_t *puAttribute;
    uint8_t *p_rsp = app_bt_gatt_alloc_buffer(len_requested);
    int used = 0;
    int xx;
    uint16_t handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

    if (p_rsp == NULL)
    {
        printf("Insufficient resource\r\n");
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);
        *p_error_handle = handle;

        if ((puAttribute = app_bt_gatt_find_by_handle(handle)) == NULL)
        {
            app_bt_gatt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used,
                                                                    len_requested - used,
                                                                    puAttribute->handle,
                                                                    puAttribute->cur_len,
                                                                    puAttribute->p_data);
            if (!filled)
            {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp, (void *)app_bt_gatt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function Name:
 * app_bt_gatt_set_attr_value
 *
 * Function Description:
 * @brief  The function is invoked by app_bt_gatt_attr_write_handler to set a value
 *         to GATT DB.
 *
 * @param attr_handle_index:  Attribute handle's index in Attribute table.
 * @param p_val: Pointer to BLE GATT write request value
 * @param len:   length of GATT write request
 * @param opcode: Opcode from the peer device
 *
 * @return wiced_bt_gatt_status_t  BLE GATT status
 */
wiced_bt_gatt_status_t app_bt_gatt_set_attr_value(uint8_t attr_handle_index,
                                        uint8_t *p_val,
                                        uint16_t len,
                                        wiced_bt_gatt_opcode_t opcode)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    if ((app_gatt_db_ext_attr_tbl[attr_handle_index].max_len) >= len)
    {
        if (NULL != memcpy(app_gatt_db_ext_attr_tbl[attr_handle_index].p_data, p_val, len))
        {
            if( WICED_BT_GATT_SUCCESS != wiced_bt_gatt_server_send_write_rsp(
                                        app_bt_conn_id,
                                        opcode,
                app_gatt_db_ext_attr_tbl[attr_handle_index].handle))
            {
                printf("WARNING: GATT Write resp status 0x%x\r\n", gatt_status);
            }

            switch (app_gatt_db_ext_attr_tbl[attr_handle_index].handle)
            {
                case HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG:
                    printf("Battery IN report ");
                    app_bt_bond_modify_cccd_in_nv_storage(
                            app_gatt_db_ext_attr_tbl[attr_handle_index].handle, p_val);
                    break;

            case HDLD_HIDS_KBD_IN_REPORT_CLIENT_CHAR_CONFIG:
                printf("KBD IN report ");
                    app_bt_bond_modify_cccd_in_nv_storage(
                    app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                    p_val);
                    break;

            case HDLD_HIDS_CC_IN_REPORT_CLIENT_CHAR_CONFIG:
                printf("CC IN report ");
                    app_bt_bond_modify_cccd_in_nv_storage(
                    app_gatt_db_ext_attr_tbl[attr_handle_index].handle,
                    p_val);
                    break;

                case HDLC_HIDS_HID_CONTROL_POINT_VALUE:
                    printf("HID Control point att value : 0x%x\r\n",
                           (unsigned int)app_hids_hid_control_point);
                    break;

                default:
                    break;

            } // Switch case for different ATT handles

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            printf("memcpy failed\r\n");
        }
    }
    else
    {
        // Check the len of the attribute
        printf("Length to be written %d\r\n", len);
        gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
    }

    return (gatt_status);
}

/**
 * Function Name:
 * app_bt_gatt_get_index_by_handle
 *
 * Function Description:
 * @brief This function returns the corresponding index for the respective
 * attribute handle from the attribute table.
 *
 * @param attr_handle: 16-bit attribute handle for the characteristics and descriptors
 * @return uint8_t: The index of the valid attribute handle otherwise
 *         INVALID_ATT_TBL_INDEX
 */
uint8_t app_bt_gatt_get_index_by_handle(uint16_t attr_handle)
{

    uint16_t left = 0;
    uint16_t right = app_gatt_db_ext_attr_tbl_size;

    while (left <= right)
    {
        uint16_t mid = left + (right - left) / 2;

        if (app_gatt_db_ext_attr_tbl[mid].handle == attr_handle)
        {
            return mid;
        }

        if (app_gatt_db_ext_attr_tbl[mid].handle < attr_handle)
        {
            left = mid + 1;
        }
        else
        {
            right = mid - 1;
        }
    }

    return INVALID_ATT_TBL_INDEX;

}

/**
 * Function Name:
 * app_bt_gatt_disable_all_cccds
 *
 * Function Description:
 * @brief This function is used to disable all CCCD
 *
 * @param void
 *
 * @return void
 */
void app_bt_gatt_disable_all_cccds(void)
{
    app_bas_battery_level_client_char_config[0] = 0x00;
    app_hids_kbd_in_report_client_char_config[0] = 0x00;
    app_hids_cc_in_report_client_char_config[0] = 0x00;

    printf("All Notifications are disabled\r\n");
}
#ifdef ENABLE_OTA
/**
 * Function Name:
 * app_bt_prepare_write_handler
 *
 * Function Description:
 * @brief   This function reassemble the fragmented packets 
 *
 * @param   conn_id: Connection ID
 * @param   opcode: BLE GATT request type opcode
 * @param   p_req: Pointer to read request containing the handle to read
 *
 * @return wiced_bt_gatt_status_t: GATT result
 *
 */
static wiced_bt_gatt_status_t app_bt_prepare_write_handler(uint16_t conn_id,
                                                           wiced_bt_gatt_opcode_t opcode,
                                                           wiced_bt_gatt_write_req_t *p_req,
                                                           uint16_t *p_error_handle)
{
    if(write_buff.in_use == false)
    {
        memset(&(write_buff.value[0]), 0x00, CY_BT_MTU_SIZE);
        write_buff.written = 0;
        write_buff.in_use = true;
        write_buff.handle = 0;
    }

    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "%s() handle : 0x%x (%d)\n", __func__, p_req->handle, p_req->handle);
    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "     offset : 0x%x\n", p_req->offset);
    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "     p_val  : %p\n",   p_req->p_val);
    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "     val_len: 0x%x\n", p_req->val_len);

    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "val_len = %d \n", p_req->val_len);

    *p_error_handle = p_req->handle;

    /** store the data  */
    if(write_buff.written == p_req->offset)
    {
        int remaining = CY_BT_MTU_SIZE - write_buff.written;
        int to_write = p_req->val_len;

        if (remaining >= to_write)
        {
            memcpy( (void*)((uint32_t)(&(write_buff.value[0]) + write_buff.written)), p_req->p_val, to_write);

            /* send success response */
            cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "== Sending prepare write success response...\n");
            wiced_bt_gatt_server_send_prepare_write_rsp(conn_id, opcode, p_req->handle,
                                                        p_req->offset, to_write,
                                                        &(write_buff.value[write_buff.written]), NULL);
            write_buff.written += to_write;
            write_buff.handle = p_req->handle;
            cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "    Total val_len: %d\n", write_buff.written);
            return WICED_BT_GATT_SUCCESS;
        }
        else
        {
            cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "remaining >= to_write error...\n");
            return WICED_BT_GATT_ERROR;
        }
    }
    else
    {
        cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "write_buff.written != p_req->offset...\n");
    }

    return WICED_BT_GATT_ERROR;
}

/**
 * Function Name:
 * app_bt_execute_write_handler
 *
 * Function Description:
 * @brief   This function writes the fragmented packets 
 *
 * @param   p_req: Pointer to read request containing the handle to read
 *
 * @return wiced_bt_gatt_status_t: GATT result
 *
 */
static wiced_bt_gatt_status_t app_bt_execute_write_handler(wiced_bt_gatt_event_data_t *p_req, uint16_t *p_error_handle)
{
    wiced_bt_gatt_write_req_t       *p_write_req;
    wiced_bt_gatt_status_t          status = WICED_BT_GATT_SUCCESS;

    CY_ASSERT(p_req != NULL);

    p_write_req = &p_req->attribute_request.data.write_req;

    *p_error_handle = p_write_req->handle;

    CY_ASSERT(p_req != NULL);

    if(write_buff.in_use == false)
    {
        cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "write_buff.inuse is false returning error...\n");
        return WICED_BT_GATT_ERROR;
    }

    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_NOTICE, "Execute Write with %d bytes\n", write_buff.written);

    p_write_req->handle = write_buff.handle;
    p_write_req->offset = 0;
    p_write_req->p_val = &(write_buff.value[0]);
    p_write_req->val_len = write_buff.written;

    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "%s() handle : 0x%x (%d)\n", __func__, p_write_req->handle, p_write_req->handle);
    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "     offset : 0x%x\n", p_write_req->offset);
    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "     p_val  : %p\n",   p_write_req->p_val);
    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "     val_len: 0x%x\n", p_write_req->val_len);

    status = app_bt_gatt_attr_write_handler(p_req,p_error_handle);
    if (status != WICED_BT_GATT_SUCCESS)
    {
        cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "app_bt_write_handler() failed....\n");
    }

    write_buff.in_use = false;

    return status;
}
#endif

/* [] END OF FILE */
