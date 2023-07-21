/******************************************************************************
* File Name: serial_flash.h
*
* Description:
*
********************************************************************************
* Copyright 2020-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef SOURCE_SERIAL_FLASH_H_
#define SOURCE_SERIAL_FLASH_H_

#include "cy_result_mw.h"

/* OTA API */
#include "cy_ota_api.h"
#include "cy_pdl.h"
#include "cyhal.h"

/** The function or operation is not supported on the target or the memory */
#define CY_RSLT_SERIAL_FLASH_ERR_UNSUPPORTED (cy_rslt_t)(CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_LIB_SERIAL_FLASH, 1))
/** The Serial Flash not initialized */
#define CY_RSLT_SERIAL_FLASH_ERR_NOT_INITED (cy_rslt_t)(CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_LIB_SERIAL_FLASH, 2))
/** Parameters passed to a function are invalid */
#define CY_RSLT_SERIAL_FLASH_ERR_BAD_PARAM   (cy_rslt_t)(CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_LIB_SERIAL_FLASH, 3))
/** A previously initiated read operation is not yet complete */
#define CY_RSLT_SERIAL_FLASH_ERR_READ_BUSY   (cy_rslt_t)(CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_LIB_SERIAL_FLASH, 4))
/** A DMA error occurred during read transfer */
#define CY_RSLT_SERIAL_FLASH_ERR_DMA         (cy_rslt_t)(CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_LIB_SERIAL_FLASH, 5))
/** Read abort failed. QSPI block is busy. */
#define CY_RSLT_SERIAL_FLASH_ERR_QSPI_BUSY   (cy_rslt_t)(CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_BOARD_LIB_SERIAL_FLASH, 6))

cy_rslt_t ota_mem_init(void);
cy_rslt_t ota_mem_read(cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len);
cy_rslt_t ota_mem_write(cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len);
cy_rslt_t ota_mem_erase(cy_ota_mem_type_t mem_type, uint32_t addr, size_t len);
size_t ota_mem_get_prog_size(cy_ota_mem_type_t mem_type, uint32_t addr);
size_t ota_mem_get_erase_size(cy_ota_mem_type_t mem_type, uint32_t addr);

#endif /* SOURCE_SERIAL_FLASH_H_ */
