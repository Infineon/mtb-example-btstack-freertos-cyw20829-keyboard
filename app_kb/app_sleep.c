/*******************************************************************************
 * File Name: app_sleep.c
 *
 * Description: This file consists of the function definitions that are
 *              necessary for handling device sleep
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
#include "FreeRTOS.h"
#include "task.h"
#include "cyabs_rtos.h"
#if defined(CY_USING_HAL)
#include "cyhal.h"
#endif

#include "app_sleep.h"
#include "app_serial_flash.h"
#include "app_keyscan.h"
#include "app_handler.h"
#include "cybsp_smif_init.h"

// This is included to allow the user to control the idle task behavior via the configurator
// System->Power->RTOS->System Idle Power Mode setting.
#if defined(COMPONENT_BSP_DESIGN_MODUS) || defined(COMPONENT_CUSTOM_DESIGN_MODUS)
#include "cycfg.h"
#endif

/*******************************************************************************
 *                              Macro Definitions
 *******************************************************************************/
#define pdTICKS_TO_MS(xTicks)    ( ( ( TickType_t ) ( xTicks ) * 1000u ) / configTICK_RATE_HZ )

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/
#if defined(CY_USING_HAL) && (configUSE_TICKLESS_IDLE != 0)
static cyhal_lptimer_t *_timer = NULL;
#endif

static bool deep_sleep = true;

static uint8_t deep_sleep_cnt;

cy_stc_syspm_callback_params_t syspm_cpu_sleep_params;
cy_stc_syspm_callback_params_t syspm_deep_sleep_params;
cy_stc_syspm_callback_params_t syspm_hibernate_params;

// uint8_t deepsleep_hold;

#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
/* WDT object */
extern cyhal_wdt_t wdt_obj;
#endif
/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
cy_en_syspm_status_t
syspm_cpu_sleep_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);

cy_en_syspm_status_t
syspm_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                 cy_en_syspm_callback_mode_t mode);

cy_en_syspm_status_t
syspm_hibernate_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode);

cy_stc_syspm_callback_t syspm_cpu_sleep_cb_handler =
{
    syspm_cpu_sleep_cb,
    CY_SYSPM_SLEEP,
    0u,
    &syspm_cpu_sleep_params,
    NULL,
    NULL,
    255
};

cy_stc_syspm_callback_t syspm_deep_sleep_cb_handler =
{
    syspm_ds_cb,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &syspm_deep_sleep_params,
    NULL,
    NULL,
    253
};

cy_stc_syspm_callback_t syspm_hibernate_handler =
{
    syspm_hibernate_cb,
    CY_SYSPM_HIBERNATE,
    0u,
    &syspm_hibernate_params,
    NULL,
    NULL,
    255
};

/**
 * Function name:
 * syspm_cpu_sleep_cb
 *
 * Function Description:
 * @brief Cpu Sleep Callback Function
 *
 * @param callbackParams: Pointer to cy_stc_syspm_callback_params_t
 * @param mode: cy_en_syspm_callback_mode_t
 *
 * @return cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
syspm_cpu_sleep_cb( cy_stc_syspm_callback_params_t *callbackParams,
                cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
            /* Stop watchdog timer */
            cyhal_wdt_kick(&wdt_obj);
            cyhal_wdt_stop(&wdt_obj);
#endif
            //Disable SMIF
            cybsp_smif_disable();

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        {
            //Enable SMIF
            cybsp_smif_enable();
#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
            /* Start watchdog timer */
            cyhal_wdt_start(&wdt_obj);
            cyhal_wdt_kick(&wdt_obj);
#endif
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/**
 *  Function name:
 *  syspm_ds_cb
 *
 *  Function Description:
 *  @brief DeepSleep Callback Function
 *
 *  @param callbackParams: Pointer to cy_stc_syspm_callback_params_t
 *  @param mode: cy_en_syspm_callback_mode_t
 *
 *  @return cy_en_syspm_status_t: CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t syspm_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                 cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
            /* Stop watchdog timer */
            cyhal_wdt_kick(&wdt_obj);
            cyhal_wdt_stop(&wdt_obj);
#endif
            if((key_state_cnt == 0) && (!deepsleep_hold) && (0==app_ghost_detected) && (fn_key_hold == false))
            {
                Cy_Keyscan_SetInterruptMask(MXKEYSCAN,  MXKEYSCAN_INTR_KEY_EDGE_DONE);
                Cy_SysClk_MfoEnable(false);
            }
            else
            {
                Cy_SysClk_MfoEnable(true);
            }
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after exiting the low power mode */
        {
#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
            /* Start watchdog timer */
            cyhal_wdt_start(&wdt_obj);
            cyhal_wdt_kick(&wdt_obj);
#endif
            Cy_Keyscan_SetInterruptMask(MXKEYSCAN, MXKEYSCAN_INTR_FIFO_THRESH_DONE);
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/**
 *  Function name:
 *  syspm_hibernate_cb
 *
 *  Function Description:
 *  @brief Hibernate Callback Function
 *
 *  @param callbackParams: Pointer to cy_stc_syspm_callback_params_t
 *  @param mode: cy_en_syspm_callback_mode_t
 *
 *  @return cy_en_syspm_status_t: CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
syspm_hibernate_cb(cy_stc_syspm_callback_params_t *callbackParams,
                              cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;
    CY_UNUSED_PARAMETER(callbackParams);

    switch(mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_BEFORE_TRANSITION:
        /* Performs the actions to be done before entering the low power mode */
        {
#if (ENABLE_WDT == true) && (ENABLE_LOGGING == false)
            /* Stop watchdog timer */
            cyhal_wdt_kick(&wdt_obj);
            cyhal_wdt_stop(&wdt_obj);
#endif
            // Disable SMIF
            cybsp_smif_disable();

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        default:
            break;
    }

    return retVal;
}
CY_SECTION_RAMFUNC_END

/**
 * Function Name:
 * create_cpu_sleep_cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for CPU Sleep mode
 *
 * @param  void
 *
 * @return void
 */
void create_cpu_sleep_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_cpu_sleep_cb_handler);
}

/**
 * Function Name:
 * create_deep_sleep_cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for Deep Sleep mode
 *
 * @param  void
 *
 * @return void
 */
void create_deep_sleep_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_deep_sleep_cb_handler);
}

/**
 * Function Name:
 * create_hibernate_cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for Hibernate mode
 *
 * @param  void
 *
 * @return void
 */
void create_hibernate_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_hibernate_handler);
}

/**
 * Function Name:
 * deep_sleep_enable
 *
 * Function Description:
 * @brief This function is used to set the MCUSS sleep mode for tickless to either
 *  System DeepSleep or CPU Sleep
 *
 * @param en: To enable / disable deepsleep during Active Peripheral operation
 *
 * @return void
 */
void deep_sleep_enable(bool en)
{
    if (en)
    {
        if (deep_sleep_cnt)
        {
            deep_sleep_cnt--;
        }
    }
    else
    {
        deep_sleep_cnt++;
    }

    if (deep_sleep_cnt)
        deep_sleep = false;
    else
        deep_sleep = true;
}

#if defined(CY_USING_HAL) && (configUSE_TICKLESS_IDLE != 0)
//--------------------------------------------------------------------------------------------------
// vApplicationSleep
//
/** User defined tickless idle sleep function.
 *
 * Provides a implementation for portSUPPRESS_TICKS_AND_SLEEP macro that allows
 * the device to attempt to deep-sleep for the idle time the kernel expects before
 * the next task is ready. This function disables the system timer and enables low power
 * timer that can operate in deep-sleep mode to wake the device from deep-sleep after
 * expected idle time has elapsed.
 *
 * @param[in] xExpectedIdleTime     Total number of tick periods before
 *                                  a task is due to be moved into the Ready state.
 */
__attribute__((section(".text.os_in_ram")))
void vApplicationSleep(TickType_t xExpectedIdleTime)
{

    uint32_t actual_sleep_ms = 0;
    static cyhal_lptimer_t timer;

    if (NULL == _timer)
    {
        cy_rslt_t result = cyhal_lptimer_init(&timer);
        if (result == CY_RSLT_SUCCESS)
        {
            _timer = &timer;
        }
        else
        {
            CY_ASSERT(false);
        }
    }

    if (NULL != _timer)
    {
        /* Disable interrupts so that nothing can change the status of the RTOS while
         * we try to go to sleep or deep-sleep.
         */
        uint32_t status = cyhal_system_critical_section_enter();
        eSleepModeStatus sleep_status = eTaskConfirmSleepModeStatus();

        if (sleep_status != eAbortSleep)
        {
            // By default, the device will deep-sleep in the idle task unless if the device
            // configurator overrides the behaviour to sleep in the System->Power->RTOS->System
            // Idle Power Mode setting.

            #if defined (CY_CFG_PWR_SYS_IDLE_MODE)
            // If the system needs to operate in active mode the tickless mode should not be used in
            // FreeRTOS
            CY_ASSERT(CY_CFG_PWR_SYS_IDLE_MODE != CY_CFG_PWR_MODE_ACTIVE);
            if (deep_sleep)
            {
                deep_sleep =
                    ((CY_CFG_PWR_SYS_IDLE_MODE & CY_CFG_PWR_MODE_DEEPSLEEP) ==
                     CY_CFG_PWR_MODE_DEEPSLEEP);
                #endif
            }
            uint32_t sleep_ms = pdTICKS_TO_MS(xExpectedIdleTime);
            cy_rslt_t result;
            if (deep_sleep)
            {
                // Adjust the deep-sleep time by the sleep/wake latency if set.
#if defined(CY_CFG_PWR_DEEPSLEEP_LATENCY)
                    if (sleep_ms > CY_CFG_PWR_DEEPSLEEP_LATENCY)
                    {
                        sleep_ms -= CY_CFG_PWR_DEEPSLEEP_LATENCY;
                        result = cyhal_syspm_tickless_deepsleep(_timer, sleep_ms, &actual_sleep_ms);
                    }
                    else
                    {
                        result = CY_RTOS_TIMEOUT;
                    }
#else // defined(CY_CFG_PWR_DEEPSLEEP_LATENCY)
                result = cyhal_syspm_tickless_deepsleep(_timer, sleep_ms, &actual_sleep_ms);
#endif // defined(CY_CFG_PWR_DEEPSLEEP_LATENCY)
            }
            else
            {
                result = cyhal_syspm_tickless_sleep(_timer, sleep_ms, &actual_sleep_ms);
            }

            if (result == CY_RSLT_SUCCESS)
            {
                // If you hit this assert, the latency time (CY_CFG_PWR_DEEPSLEEP_LATENCY) should
                // be increased. This can be set though the Device Configurator, or by manually
                // defining the variable.
                CY_ASSERT(actual_sleep_ms <= pdTICKS_TO_MS(xExpectedIdleTime));
                vTaskStepTick(pdMS_TO_TICKS(actual_sleep_ms));
            }
        }

        cyhal_system_critical_section_exit(status);
    }
}

#endif // defined(CY_USING_HAL) && (configUSE_TICKLESS_IDLE != 0)
