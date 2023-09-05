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
#include "cy_retarget_io.h"

/*******************************************************************************
 *                              Macro Definitions
 *******************************************************************************/
#define pdTICKS_TO_MS(xTicks)    ( ( ( TickType_t ) ( xTicks ) * 1000u ) / configTICK_RATE_HZ )

/*******************************************************************************
 *                              Global Variables
 ******************************************************************************/

cy_stc_syspm_callback_params_t syspm_deep_sleep_params;
cy_stc_syspm_callback_params_t syspm_deep_sleep_ram_params;
cy_stc_syspm_callback_params_t syspm_deep_sleep_ram_dbg_diable_params;
extern cy_stc_syspm_warmboot_entrypoint_t syspmBspDeepSleepEntryPoint;

extern cyhal_gpio_callback_data_t app_button_isr_data;

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/
cy_en_syspm_status_t
app_syspm_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                 cy_en_syspm_callback_mode_t mode);

cy_en_syspm_status_t
app_syspm_ds_ram_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                 cy_en_syspm_callback_mode_t mode);


cy_en_syspm_status_t
app_syspm_dbg_diable_dsram_cb(cy_stc_syspm_callback_params_t *callbackParams,
                                    cy_en_syspm_callback_mode_t mode);

cy_stc_syspm_callback_t syspm_deep_sleep_cb_handler =
{
    app_syspm_ds_cb,
    CY_SYSPM_DEEPSLEEP,
    0u,
    &syspm_deep_sleep_params,
    NULL,
    NULL,
    253
};

cy_stc_syspm_callback_t syspm_deep_sleep_ram_cb_handler =
{
    app_syspm_ds_ram_cb,
    CY_SYSPM_DEEPSLEEP_RAM,
    0u,
    &syspm_deep_sleep_ram_params,
    NULL,
    NULL,
    250
};

cy_stc_syspm_callback_t syspm_deep_sleep_ram_dbg_diable_cb_handler =
{
    app_syspm_dbg_diable_dsram_cb,
    CY_SYSPM_DEEPSLEEP_RAM,
    0u,
    &syspm_deep_sleep_ram_dbg_diable_params,
    NULL,
    NULL,
    0
};
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
cy_en_syspm_status_t app_syspm_ds_cb(cy_stc_syspm_callback_params_t *callbackParams,
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
 *  syspm_ds_cb
 *
 *  Function Description:
 *  @brief DeepSleep Callback Function
 *
 *  @param callbackParams: Pointer to cy_stc_syspm_callback_params_t
 *  @param mode: cy_en_syspm_callback_mode_t
 *
 *  @return cy_en_syspm_status_t: CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 *
 */

CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t app_syspm_ds_ram_cb(cy_stc_syspm_callback_params_t *callbackParams,
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
            Cy_Keyscan_SetInterruptMask(MXKEYSCAN, MXKEYSCAN_INTR_FIFO_THRESH_DONE);
            cyhal_gpio_free(PAIR_BUTTON);

            cyhal_gpio_init(PAIR_BUTTON,
                            CYHAL_GPIO_DIR_INPUT,
                            CYHAL_GPIO_DRIVE_PULLUP,
                            CYBSP_BTN_OFF);


            /* Configure GPIO interrupt on Pair Button */
            cyhal_gpio_register_callback(PAIR_BUTTON,
                                         &app_button_isr_data);

            /* Enable GPIO Interrupt on both edge for Pair Button */
            cyhal_gpio_enable_event(PAIR_BUTTON,
                                    CYHAL_GPIO_IRQ_BOTH,
                                    GPIO_INTERRUPT_PRIORITY,
                                    TRUE);
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
 * app_syspm_ks_ds_cb
 *
 * Function Description:
 * @brief DeepSleep Callback Function
 *
 * @param callbackParams Pointer to cy_stc_syspm_callback_params_t
 * @param mode cy_en_syspm_callback_mode_t
 *
 * @return cy_en_syspm_status_t CY_SYSPM_SUCCESS or CY_SYSPM_FAIL
 */
CY_SECTION_RAMFUNC_BEGIN
cy_en_syspm_status_t
app_syspm_dbg_diable_dsram_cb( cy_stc_syspm_callback_params_t *callbackParams,
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
            Cy_Syslib_SetWarmBootEntryPoint((uint32_t*)&syspmBspDeepSleepEntryPoint, false);
            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_DS_WFI_TRANSITION:
        {

            retVal = CY_SYSPM_SUCCESS;
        }
        break;

        case CY_SYSPM_AFTER_TRANSITION:
        /* Performs the actions to be done after entering the low power mode */
        {
#if ENABLE_LOGGING
            if(Cy_SysLib_IsDSRAMWarmBootEntry())
                {
                   cy_retarget_io_deinit();
                   cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,115200);
                }
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
 * create_deep_sleep_ram_cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for Deep Sleep ram mode
 *
 * @param  void
 *
 * @return void
 */
void create_deep_sleep_ram_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_deep_sleep_ram_cb_handler);
}
/**
 * Function Name:
 * create_deep_sleep_ram_dbg_diable_cb
 *
 * Function Description:
 * @brief Creates a syspm Callback for Deep Sleep ram mode
 *
 * @param  void
 *
 * @return void
 */
void create_deep_sleep_ram_dbg_diable_cb(void)
{
    Cy_SysPm_RegisterCallback(&syspm_deep_sleep_ram_dbg_diable_cb_handler);
}
