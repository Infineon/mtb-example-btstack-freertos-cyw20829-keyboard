/******************************************************************************
 * File Name:   app_button.h
 *
 * Description: This file consists of the function prototypes that are
 *              necessary for developing push button use cases.
 *
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

#ifndef __APP_BUTTON_H_
#define __APP_BUTTON_H_

/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include "cy_sysint.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "cycfg_pins.h"
#include "app_bt_hid.h"
#include "cyhal_gpio.h"

/*******************************************************************************
 *                      MACROS / VARIABLE DEFINITIONS
 ******************************************************************************/
/* Priority for GPIO Button Interrupt */
#define GPIO_INTERRUPT_PRIORITY     (7u)


/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void app_button_init(void);
void app_button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

#endif      /* __APP_BUTTON_H */

/* [] END OF FILE */
