/*******************************************************************************
 * File Name: app_led.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for developing LED and push button use cases.
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

#include "app_led.h"
#include "app_handler.h"
#include "app_batmon.h"
#include "timers.h"

/*******************************************************************************
 *                              CONSTANTS
 ******************************************************************************/

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
/*                  LED and User Button Macros and variables                  */

static TimerHandle_t led_blink_timer;
static TimerHandle_t power_on_led_ind_timer;
static bool led_blinking = false;
static bool led_state = false;
static bool power_on_led_ind = true;

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
/*                      Status LED APIs                                       */

/**
 *  Function name:
 *  app_status_led_on
 *
 *  Function Description:
 *  @brief    Function to Turn ON status LED
 *
 *  @param    void
 *
 *  @return   void
 */
void app_status_led_on(void)
{
    /* Turn On Status LED */
    cyhal_gpio_write(STATUS_LED, CYBSP_LED_STATE_ON);
    led_state = true;
}

/**
 *  Function name:
 *  app_status_led_off
 *
 *  Function Description:
 *  @brief    Function to Turn OFF status LED
 *
 *  @param    void
 *
 *  @return   void
 */
void app_status_led_off(void)
{
    /* Turn Off Status LED */
    cyhal_gpio_write(STATUS_LED, CYBSP_LED_STATE_OFF);
    led_state = false;
}

/**
 *  Function name:
 *  app_led_blink_timer_cb
 *
 *  Function Description:
 *  @brief    Timer cb for LED blinking
 *
 *  @param    TimerHandle_t
 *
 *  @return   void
 */
static void app_led_blink_timer_cb(TimerHandle_t cb_params)
{
    if (led_blinking)
    {
        /* Check to halt LED blinking till power ON Indication is done */
        if (!power_on_led_ind)
        {
            led_state = !led_state;
            cyhal_gpio_toggle(STATUS_LED);
        }

        /* Low battery LED Indication */
        if (low_battery == true)
        {
            if (led_state == true)
            {
                app_led_update_blink_period(LOW_BATT_LED_BLINK_ON_TIME_MS);
            }
            else
            {
                app_led_update_blink_period(LOW_BATT_LED_BLINK_OFF_TIME_MS);
            }
        }
        
        if (pdFAIL == xTimerStart(led_blink_timer, TIMER_MIN_WAIT))
        {
            printf("Failed to start LED blink Timer\r\n");
        }
    }
}

/**
 *  Function name:
 *  app_status_led_start_blinking
 *
 *  Function Description:
 *  @brief    Function to start LED blinking
 *
 *  @param    TimerHandle_t
 *
 *  @return   void
 */
void app_status_led_start_blinking(void)
{
    if (!led_blinking)
    {
        led_blinking = true;
        
         /* Check to halt LED blinking till power ON Indication is done */
        if (!power_on_led_ind)
        {
            led_state = !led_state;
            cyhal_gpio_toggle(STATUS_LED);
        }

        /* Low battery LED Indication */
        if (low_battery == true)
        {
            if (led_state == true)
            {
                app_led_update_blink_period(LOW_BATT_LED_BLINK_ON_TIME_MS);
            }
            else
            {
                app_led_update_blink_period(LOW_BATT_LED_BLINK_OFF_TIME_MS);
            }
        }
        
        if (pdFAIL == xTimerStart(led_blink_timer, TIMER_MAX_WAIT))
        {
            printf("Failed to start LED blink Timer\r\n");
        }
    }
}

/**
 *  Function name:
 *  app_status_led_stop_blinking
 *
 *  Function Description:
 *  @brief    Function to stop LED blinking
 *
 *  @param    TimerHandle_t
 *
 *  @return   void
 */
void app_status_led_stop_blinking(void)
{
    if ((led_blinking) && (low_battery == false))
    {
        led_blinking = false;

        if (!power_on_led_ind)
        {
            app_status_led_off();
        }
    }
}


/**
 *  Function name:
 *  app_led_update_blink_period
 *
 *  Function Description:
 *  @brief    Function to update LED blink period
 *
 *  @param    uint32_t
 *
 *  @return   void
 */
void app_led_update_blink_period(uint32_t time_period)
{
    xTimerChangePeriod(led_blink_timer, pdMS_TO_TICKS(time_period), TIMER_MIN_WAIT);
}

/**
 *  Function name:
 *  app_led_power_on_timer_cb
 *
 *  Function Description:
 *  @brief    Timer cb for Power ON LED indication
 *
 *  @param    TimerHandle_t
 *
 *  @return   void
 */
void app_led_power_on_timer_cb(TimerHandle_t cb_params)
{
    power_on_led_ind = false;

    /* If connected, turn OFF LED after power ON indication,
     * else it will be turned OFF when advertisement stops */
    if (!led_blinking)
    {
        app_status_led_off();
    }
}

/**
 *  Function name:
 *  app_status_led_init
 *
 *  Function Description:
 *  @brief    Function to initialize status LED
 *
 *  @param    void
 *
 *  @return   void
 */
void app_status_led_init(void)
{
     /* Initialize with LED OFF state */
     cyhal_gpio_init(STATUS_LED,
                     CYHAL_GPIO_DIR_OUTPUT,
                     CYHAL_GPIO_DRIVE_STRONG,
                     CYBSP_LED_STATE_OFF);

     /* Initialize timer for blinking */
     led_blink_timer = xTimerCreate("LED Blink Timer",
                                pdMS_TO_TICKS(PAIRING_MODE_ADV_LED_BLINK_TIME_MS),
                                pdFALSE,
                                NULL ,
                                app_led_blink_timer_cb);
    /* Timer init failed. Stop program execution */
    if (NULL == led_blink_timer)
    {
        printf("LED blink Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    
    /* initialize timer for power on */
     power_on_led_ind_timer = xTimerCreate("Power ON LED Ind Timer",
                                pdMS_TO_TICKS(POWER_UP_LED_ON_TIME_MS),
                                pdFALSE,
                                NULL ,
                                app_led_power_on_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == power_on_led_ind_timer)
    {
        printf("Power ON LED Indication Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }
    /* Start Power ON LED Indication */
    app_status_led_on();

    if (pdFAIL == xTimerStart(power_on_led_ind_timer, TIMER_MAX_WAIT))
    {
        printf("Failed to start power ON LED indication Timer\r\n");
    }
}
