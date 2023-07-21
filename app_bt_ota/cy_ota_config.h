/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Cypress OTA API abstracts underlying network and
 * platform support for Over The Air updates.
 */
 /**
 * \addtogroup group_cy_ota Infineon Over The Air (OTA) API
 * \{
 * \defgroup group_ota_config OTA User Configurations
 */
/**
 *  Customer defines for the OTA library
 *
 **********************************************************************/

#ifndef CY_OTA_CONFIG_H__
#define CY_OTA_CONFIG_H__ 1

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initial time for checking for OTA updates.
 *
 * This is used to start the timer for the initial OTA update check after calling cy_ota_agent_start().
 */
#define CY_OTA_INITIAL_CHECK_SECS           (10)            /* 10 seconds. */

#ifdef __cplusplus
    }
#endif

#endif /* CY_OTA_CONFIG_H__ */

/** \} group_cy_ota */
