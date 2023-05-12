/******************************************************************************
 * File Name: app_batmon.c
 *
 * Description: This files contains the function definition of DC monitoring
 * of the battery
 *
 * Related Document: See Readme.md
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
 ******************************************************************************/

/*******************************************************************************
 *                               Includes
 *******************************************************************************/
#include "cy_pdl.h"
#include "cyhal.h"
#include "cyhal_syspm.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "app_bt_hid.h"
#include "app_batmon.h"
#include "app_handler.h"

/*******************************************************************************
 *                               Macro Definitions
 *******************************************************************************/
#define NO_OF_DC_SAMPLES                    (8)
#define DC_START                            (1)
#define DC_STOP                             (0)

/*******************************************************************************
 *                               Global Variables
 *******************************************************************************/
bool low_battery = false;
uint16_t batmon_samples[NO_OF_DC_SAMPLES];
uint8_t dc_sample_cnt = 0;
uint32_t batmon_dc_avg = 0;

/* Initial State of ADC driver set to INIT */
volatile adc_driver_state_t adc_drv_state = ADC_INIT;
/* Timer handle for battery voltage read */
TimerHandle_t batmon_timer;
/* Battery monitoring task handle */
TaskHandle_t batmon_task_h;

/* ADCMIC interrupt configuration parameters */
const cy_stc_sysint_t ADCMIC_IRQ_cfg = { .intrSrc = (IRQn_Type)adcmic_0_IRQ,
                                         .intrPriority = 7 };

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
void app_batmon_timer_cb(TimerHandle_t cb_params);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/**
 * Function Name:
 * app_batmon_adc_dc_cap_start
 *
 * Function Description:
 * @brief It enable the ADC DC monitoring.
 *
 * @param void
 *
 * @return void
 */
static void app_batmon_adc_dc_cap_start(void)
{
    if (adc_drv_state == ADC_IDLE)
    {
        Cy_ADCMic_SetInterruptMask(adcmic_0_HW, CY_ADCMIC_INTR_DC);
        Cy_ADCMic_ClearInterrupt(adcmic_0_HW, CY_ADCMIC_INTR);
        Cy_ADCMic_Enable(adcmic_0_HW);
        Cy_ADCMic_EnableTimer(adcmic_0_HW);
        adc_drv_state = ADC_DC_MON_ON;

        /* Do not allow the device to go to DS */
        deep_sleep_enable(false);
    }

}

/**
 * Function Name:
 * app_batmon_adcmic_dc_intr_handler
 *
 * Function Description:
 * @brief ADC DC Measurement ISR handler.
 *  On Every ISR, one DC measurement is read and stored.
 *  Once the Number of DC samples are read for averaging,
 *  it sends notification battery monitoring task for further processing.
 *
 * @param void
 *
 * @return void
 */
static void app_batmon_adcmic_dc_intr_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    uint32_t intr_status;
    Cy_ADCMic_DisableTimer(adcmic_0_HW);

    /* Clear the DC interrupt */
    intr_status = Cy_ADCMic_GetInterruptStatusMasked(adcmic_0_HW);
    Cy_ADCMic_ClearInterrupt(adcmic_0_HW, intr_status);

    /* Don't start the timer for Next measurement if ADC driver state is changed
     * to IDLE */
    if (adc_drv_state == ADC_IDLE)
        return;

    batmon_samples[dc_sample_cnt] = Cy_ADCMic_GetDcResult(adcmic_0_HW);
    dc_sample_cnt++;

    /* Stop Measurement on receiving the number of Defined ADC DC Samples */
    if (dc_sample_cnt >= NO_OF_DC_SAMPLES)
    {
        dc_sample_cnt = 0;

        /* Stop the conversion */
        Cy_ADCMic_Disable(adcmic_0_HW);
        Cy_ADCMic_SetInterruptMask(adcmic_0_HW, 0);

        /* Set the ADC Driver state to IDLE once DC measurement is done */
        adc_drv_state = ADC_IDLE;

        xTaskNotifyFromISR(batmon_task_h, 0, eNoAction, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        Cy_ADCMic_EnableTimer(adcmic_0_HW);
    }
}

/**
 * Function Name:
 * app_batmon_timer_cb
 *
 * Function Description:
 * @brief Timer cb to start ADC DC monitoring.
 *
 * @param cb_param: Argument to cb
 *
 * @return void
 */
void app_batmon_timer_cb(TimerHandle_t cb_params)
{
    printf("batmon_timer_cb\r\n");

    /* Check if it is initial battery level read */
    if (adc_drv_state == ADC_INIT)
    {
        /* Update the state to idle and change the battery read interval */
        adc_drv_state = ADC_IDLE;
        xTimerChangePeriod(batmon_timer, pdMS_TO_TICKS(BATT_LVL_UPDATE_INTERVAL_MS), TIMER_MIN_WAIT);
    }

    (void)cb_params;
    app_batmon_adc_dc_cap_start();
}

/**
 * Function Name:
 * app_batmon_init
 *
 * Function Description:
 * @brief Initialize battery monitoring using ADC
 *
 * @param void
 *
 * @return void
 *
 */
static void app_batmon_init(void)
{
    /* Register the interrupt handler of ADCMIC Irq */
    Cy_SysInt_Init(&ADCMIC_IRQ_cfg, app_batmon_adcmic_dc_intr_handler);
    NVIC_ClearPendingIRQ(ADCMIC_IRQ_cfg.intrSrc);
    NVIC_EnableIRQ(ADCMIC_IRQ_cfg.intrSrc);

    /* Initialize the ADCMic for for DC monitoring */
    if (CY_ADCMIC_SUCCESS != Cy_ADCMic_Init(adcmic_0_HW, &adcmic_0_config, CY_ADCMIC_DC))
    {
        CY_ASSERT(0);
    }

    /* Create a Periodic timer for Battery Monitoring */
    batmon_timer = xTimerCreate("Battery Monitoring Timer",
                                  pdMS_TO_TICKS(BATT_LVL_INIT_READ_DELAY_MS),
                                  pdTRUE,
                                  NULL,
                                  app_batmon_timer_cb);

    /* Timer init failed. Stop program execution */
    if (NULL == batmon_timer)
    {
        printf("Battery Monitoring Timer Initialization has failed! \r\n");
        CY_ASSERT(0);
    }

    if (pdPASS != xTimerStart(batmon_timer, TIMER_MAX_WAIT))
    {
        printf("Failed to start batmon timer!\r\n");
        CY_ASSERT(0);
    }
    printf("batmon timer started!\r\n");
}

/**
 * Function Name:
 * app_batmon_send_msg_to_hid_msg_q
 *
 * Function Description:
 * @brief Sends battery capacity level to HID queue
 *
 * @param batt_level: Battery capacity level
 *
 * @return void
 */
static void app_batmon_send_msg_to_hid_msg_q(uint8_t batt_level)
{
    struct hid_rpt_msg batmon_msg;
    batmon_msg.msg_type = BATT_MSG_TYPE;
    batmon_msg.data.batt_level = batt_level;

    if (pdPASS != xQueueSend(hid_rpt_q, &batmon_msg, TASK_MAX_WAIT))
    {
        printf("Failed to send msg from BM to HID rpt Queue\r\n");
    }
}

/**
 * Function Name:
 * app_batmon_task
 *
 * Function Description:
 * @brief Battery Monitoring Task Handler:
 *  This task function creates timer and processes the ADC measurement data,
 *  converts it to the battery capacity level
 *
 * @param arg Not used
 *
 * @return void
 */
void app_batmon_task(void *arg)
{
    uint32_t ulNotifiedValue = 0;
    uint16_t batt_level_mv = 0;
    uint8_t batt_cap = 0;
    uint8_t batt_cap_prev = 100;
    int i = 0;

    app_batmon_init();

    while (1)
    {
        /* Block until a command is received */
        xTaskNotifyWait(0, 0, &ulNotifiedValue, portMAX_DELAY);
        batmon_dc_avg = 0;

        /* Averaging of DC measurement data */
        for (i = 0; i < NO_OF_DC_SAMPLES; i++)
        {
            batmon_dc_avg = batmon_dc_avg + batmon_samples[i];
        }
        batmon_dc_avg = batmon_dc_avg / NO_OF_DC_SAMPLES;

        /* Get the DC data to MV and convert it to battery capacity */
        batt_level_mv = Cy_ADCMic_CountsTo_mVolts((uint16_t)batmon_dc_avg, adcmic_0_config.dcConfig->context);
        if (batt_level_mv >= BATT_LVL_100_MV)
        {
            batt_cap = 100;
        }
        else if (batt_level_mv > BATT_LVL_10_MV)
        {
            batt_level_mv = BATT_LVL_100_MV - batt_level_mv;
            batt_cap = 100 - ((batt_level_mv * 100) / BATT_MV_10_CAP);
        }
        else
        {
            if (batt_level_mv <= BATT_LVL_1_MV)
            {
                batt_cap = 1;
            }
            else
            {
                batt_level_mv = BATT_LVL_10_MV - batt_level_mv;
                batt_cap = 10 - ((batt_level_mv * 100) / BATT_MV_1_CAP);
            }
        }

        /* Send the battery value to HID queue */
        if (batt_cap <= batt_cap_prev || batt_cap <= 5)
        {
            batt_cap_prev = batt_cap;

            app_batmon_send_msg_to_hid_msg_q(batt_cap);
        }

        /* Low Battery voltage check and indication */
        if (batt_cap <= LOW_BATT_VOLTAGE_PERCENT)
        {
            low_battery = true;
            app_status_led_start_blinking();
        }

        /* Allow the device to go to DS */
        deep_sleep_enable(true);
    }
}

/* [] END OF FILE */
