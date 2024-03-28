/******************************************************************************
 * File Name: app_serial_flash.h
 *
 * Description: This file contains block device function implementations
 *              required by kv-store library
 *
 * Related Document: See README.md
 *
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

#ifndef __APP_SERIAL_FLASH_H_
#define __APP_SERIAL_FLASH_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "mtb_kvstore.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/
#define UNIQUE_FLASH_ID_ADDR    (0x4B)
#define JEDEC_ID_ADDR           (0x9F)

/*******************************************************************************
 *                          Global Variables
 ******************************************************************************/
/* Object for kvstore library */
extern mtb_kvstore_t kvstore_obj;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
/* QSPI and driver Init API for Non-volatile storage */

void app_flash_kv_store_init(void);
void app_flash_bd_init(mtb_kvstore_bd_t *device);
void app_flash_memory_power_down(void);
void app_flash_memory_power_up(void);
void app_flash_smif_disable(void);
void app_flash_smif_enable(void);
cy_rslt_t app_flash_bd_get_flash_id(void *context, uint32_t addr, uint32_t length, uint8_t *buf);

#endif //__APP_SERIAL_FLASH_H_
