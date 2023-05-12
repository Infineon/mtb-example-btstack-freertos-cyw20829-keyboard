/*******************************************************************************
 * File Name: app_button.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for developing push button use cases.
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
#include "app_button.h"
#include "app_keyscan.h"
#include "app_bt_event_handler.h"
#include "app_handler.h"
#include "timers.h"
#include "app_bt_advert.h"
#include "app_config.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/* Timer handle for pairing mode switch */
TimerHandle_t pairing_mode_timer;

/* Timer handle for button debounce handle */
TimerHandle_t button_debounce_timer;

/* Structure for GPIO interrupt */
cyhal_gpio_callback_data_t app_button_isr_data =
    {
        .callback = app_button_interrupt_handler,
        .callback_arg = NULL};

extern volatile bool key_press_detected;

/* Task handle to send BLE GATT notifications */
extern TaskHandle_t ble_task_h;

/*******************************************************************************
 *                              Function prototype
 ******************************************************************************/

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

/**
 *  Function name:
 *  app_button_interrupt_handler
 *
 *  Function Description:
 *  @brief    Button interrupt handler
 *
 *  @param    void *handler_arg (unused), cyhal_gpio_irq_event_t (unused)
 *
 *  @return   void
 */
void app_button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken;

    xTimerStartFromISR(button_debounce_timer, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 *  Function name:
 *  app_button_debounce_timer_cb
 *
 *  Function Description:
 *  @brief    This function is used to update current button states after debounce timeout
 *
 *  @param    TimerHandle_t
 *
 *  @return   void
 */
void app_button_debounce_timer_cb(TimerHandle_t cb_params)
{
    if (!cyhal_gpio_read(PAIR_BUTTON))
    {
        key_press_detected = 1;
    }

    if (key_press_detected)
    {
         if (pdFAIL == xTimerStart(pairing_mode_timer, TIMER_MIN_WAIT))
         {
             printf("Failed to start debounce Timer\r\n");
         }
    }
}

/**
 *  Function name:
 *  app_button_pair_mode_switch_timer_cb
 *
 *  Function Description:
 *  @brief    Timer cb to start pairing mode advertisement
 *
 *  @param    TimerHandle_t
 *
 *  @return   void
 */
void app_button_pair_mode_switch_timer_cb(TimerHandle_t cb_params)
{
    if ((!cyhal_gpio_read(PAIR_BUTTON))
#if (ENABLE_NUMERIC_KEY_CHANNEL_SWITCH == true)
        || (channel_select_button_pressed() == true)
#endif
    )
    {
        app_bt_adv_pairing_mode_switch();
    }
}

/*                      GPIO Button Manager APIs                              */
/**
 * @brief Initialize the user button with interrupt configured for falling edge.
 *
 */
void app_button_init(void)
{
    /* Initialize the user Pair button */
    if (CY_RSLT_SUCCESS != cyhal_gpio_init(PAIR_BUTTON,
                                           CYHAL_GPIO_DIR_INPUT,
                                           CYHAL_GPIO_DRIVE_PULLUP,
                                           CYBSP_BTN_OFF))
    {
        printf("Pair Button Init failed\n");
    }

    /* Configure GPIO interrupt on Pair Button */
    cyhal_gpio_register_callback(PAIR_BUTTON,
                                 &app_button_isr_data);

    /* Enable GPIO Interrupt on both edge for Pair Button */
    cyhal_gpio_enable_event(PAIR_BUTTON,
                            CYHAL_GPIO_IRQ_BOTH,
                            GPIO_INTERRUPT_PRIORITY,
                            TRUE);

    /* Create timer for handling button debounce */
    button_debounce_timer = xTimerCreate("Button Debounce Timer",
                                         pdMS_TO_TICKS(PAIR_BUTTON_DEBOUNCE_MS),
                                         pdFALSE,
                                         NULL,
                                         app_button_debounce_timer_cb);

    /* Create timer for handling pairing mode switch */
    pairing_mode_timer = xTimerCreate("Pairing Mode Timer",
                                      PAIRING_MODE_SWITCH_DELAY_MS,
                                      pdFALSE,
                                      NULL,
                                      app_button_pair_mode_switch_timer_cb);
}
