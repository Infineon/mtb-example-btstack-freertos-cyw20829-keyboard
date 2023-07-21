/*******************************************************************************
 * File Name: app_handler.h
 *
 * Description: This file consists of the function prototypes that are
 *              necessary for developing the HAL applications
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

#ifndef __APP_HANDLER_H_
#define __APP_HANDLER_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/

#ifdef STACK_INSIDE_FREE_RTOS
#include "cybt_platform_config.h"
#include "cybt_platform_trace.h"
#endif
#include "app_bt_bonding.h"

#include "app_led.h"
#include "app_button.h"
#include "app_serial_flash.h"
#include "app_config.h"
#include "app_sleep.h"
#include "app_keyscan.h"

#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"
#include "app_bt_advert.h"


/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

#if (ENABLE_LOGGING == false)
#define printf(fmt, ...)        (void)0
#endif

#define TIMER_MAX_WAIT  pdMS_TO_TICKS(30U) // Max ms wait time in timer task
#define TIMER_MIN_WAIT  pdMS_TO_TICKS(0U) // Min ms wait time in timer task
/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

#endif // __APP_HANDLER_H_
