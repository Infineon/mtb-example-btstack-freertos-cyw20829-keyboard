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

/*
 *  Flash operation callback implementation for OTA update.
 */

/* Header file includes */
#include <stdio.h>
#include <assert.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_log.h"
#include "serial_flash.h"

/**********************************************************************************************************************************
 * local defines
 **********************************************************************************************************************************/

// SMIF slot from which the memory configuration is picked up - fixed to 0 as
// the driver supports only one device
#define MEM_SLOT                                    (0u)

/* Set it high enough for the sector erase operation to complete */
#define MEMORY_BUSY_CHECK_RETRIES                   (750ul)
#define _CYHAL_QSPI_DESELECT_DELAY                  (7UL)

/* cyhal_qspi_init() succeeded */
#define FLAG_HAL_INIT_DONE                          (0x01lu << 0)

#define IS_FLAG_SET(mask)                           (status_flags & (mask))
#define SET_FLAG(mask)                              (status_flags |= (mask))
#define CLEAR_FLAG(mask)                            (status_flags &= ~(mask))

/* QSPI bus frequency set to 50 Mhz */
#define QSPI_BUS_FREQUENCY_HZ                       (50000000lu)

#define TIMEOUT_1_MS                                (1000lu)

#define CY_SMIF_BASE_MEM_OFFSET                     CY_XIP_BASE

#ifdef CY_XIP_SMIF_MODE_CHANGE

/*
 * IMPORTANT NOTE. Do not add calls to non-RAM resident routines
 * between TURN_OFF_XIP and TURN_ON_XIP calls or you will break
 * XIP environments.
 */

#define PRE_SMIF_ACCESS_TURN_OFF_XIP \
                    uint32_t interruptState;                            \
                    interruptState = Cy_SysLib_EnterCriticalSection();  \
                    while(Cy_SMIF_BusyCheck(SMIF0));    \
                    (void)Cy_SMIF_SetMode(SMIF0, CY_SMIF_NORMAL);

#define POST_SMIF_ACCESS_TURN_ON_XIP \
                    while(Cy_SMIF_BusyCheck(SMIF0));    \
                    (void)Cy_SMIF_SetMode(SMIF0, CY_SMIF_MEMORY);   \
                    Cy_SysLib_ExitCriticalSection(interruptState);


#else
#define PRE_SMIF_ACCESS_TURN_OFF_XIP
#define POST_SMIF_ACCESS_TURN_ON_XIP
#endif

/**********************************************************************************************************************************
 * local variables & data
 **********************************************************************************************************************************/

static cy_stc_smif_context_t ota_QSPI_context;
static volatile uint32_t     status_flags;

/* Default QSPI configuration */
const cy_stc_smif_config_t ota_SMIF_config =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = _CYHAL_QSPI_DESELECT_DELAY,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INVERTED_FEEDBACK_CLK,
#else
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INV_INTERNAL_CLK,
#endif
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR,
};



#ifdef READBACK_SMIF_WRITE_TEST
/* Used for testing the write functionality */
static uint8_t read_back_test[1024];
#endif

/**********************************************************************************************************************************
 * Internal Functions
 **********************************************************************************************************************************/

/*******************************************************************************
* Function Name: IsMemoryReady
****************************************************************************//**
*
* Polls the memory device to check whether it is ready to accept new commands or
* not until either it is ready or the retries have exceeded the limit.
*
* \param memConfig
* memory device configuration
*
* \return Status of the operation.
* CY_SMIF_SUCCESS          - Memory is ready to accept new commands.
* CY_SMIF_EXCEED_TIMEOUT - Memory is busy.
*
*******************************************************************************/
static cy_en_smif_status_t IsMemoryReady(cy_stc_smif_mem_config_t const *memConfig)
{
    uint32_t retries = 0;
    bool isBusy;

    do
    {
        isBusy = Cy_SMIF_Memslot_IsBusy(SMIF0, (cy_stc_smif_mem_config_t* )memConfig, &ota_QSPI_context);
        Cy_SysLib_Delay(5);
        retries++;
    }while(isBusy && (retries < MEMORY_BUSY_CHECK_RETRIES));

    return (isBusy ? CY_SMIF_EXCEED_TIMEOUT : CY_SMIF_SUCCESS);
}
/*******************************************************************************
* Function Name: IsQuadEnabled
****************************************************************************//**
*
* Checks whether QE (Quad Enable) bit is set or not in the configuration
* register of the memory.
*
* \param memConfig
* Memory device configuration
*
* \param isQuadEnabled
* This parameter is updated to indicate whether Quad mode is enabled (true) or
* not (false). The value is valid only when the function returns
* CY_SMIF_SUCCESS.
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
static cy_en_smif_status_t IsQuadEnabled(cy_stc_smif_mem_config_t const *memConfig, bool *isQuadEnabled)
{
    cy_en_smif_status_t status;
    uint8_t readStatus = 0;
    uint32_t statusCmd = memConfig->deviceCfg->readStsRegQeCmd->command;
    uint8_t maskQE = (uint8_t) memConfig->deviceCfg->stsRegQuadEnableMask;

    status = Cy_SMIF_Memslot_CmdReadSts(SMIF0, memConfig, &readStatus, statusCmd, &ota_QSPI_context);

    *isQuadEnabled = false;
    if(CY_SMIF_SUCCESS == status)
    {
        /* Check whether Quad mode is already enabled or not */
        *isQuadEnabled = (maskQE == (readStatus & maskQE));
    }

    return status;
}


/*******************************************************************************
* Function Name: EnableQuadMode
****************************************************************************//**
*
* This function sets the QE (QUAD Enable) bit in the external memory
* configuration register to enable Quad SPI mode.
*
* \param memConfig
* Memory device configuration
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
static cy_en_smif_status_t EnableQuadMode(cy_stc_smif_mem_config_t const *memConfig)
{
    cy_en_smif_status_t status;

    /* Send Write Enable to external memory */
    status = Cy_SMIF_Memslot_CmdWriteEnable(SMIF0, smifMemConfigs[0], &ota_QSPI_context);

    if(CY_SMIF_SUCCESS == status)
    {
        status = Cy_SMIF_Memslot_QuadEnable(SMIF0, (cy_stc_smif_mem_config_t* )memConfig, &ota_QSPI_context);

        if(CY_SMIF_SUCCESS == status)
        {
            /* Poll memory for the completion of operation */
            status = IsMemoryReady(memConfig);
        }
    }

    return status;
}


static uint32_t ota_smif_get_memory_size(void)
{
    uint32_t size = 0;

    if (SMIF0 != NULL)
    {
        size = smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->memSize;
    }

    return size;
}

/**********************************************************************************************************************************
 * External Functions
 **********************************************************************************************************************************/
cy_rslt_t ota_mem_init( void )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#if defined(OTA_USE_EXTERNAL_FLASH)
    cy_rslt_t smif_status = CY_SMIF_BAD_PARAM;    /* Does not return error if SMIF Quad fails */
    bool QE_status = false;

    cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() \n", __func__);

    /* pre-access to SMIF */
    PRE_SMIF_ACCESS_TURN_OFF_XIP;

#if (defined (CYW20829A0LKML) || defined (CYW20829B0LKML))
    /* SMIF is already initialized for 20829 so we are only initializing the
     * SMIF base address and the context variables.
     */
    smif_status = Cy_SMIF_Init(SMIF0, &ota_SMIF_config, TIMEOUT_1_MS, &ota_QSPI_context);
    if (smif_status != CY_SMIF_SUCCESS)
    {
        result = smif_status;
        goto _bail;
    }
#endif

    /* Set up SMIF */
    Cy_SMIF_SetDataSelect(SMIF0, smifMemConfigs[0]->slaveSelect, smifMemConfigs[0]->dataSelect);
    Cy_SMIF_Enable(SMIF0, &ota_QSPI_context);

    /* Map memory device to memory map */
    smif_status = Cy_SMIF_Memslot_Init(SMIF0, &smifBlockConfig, &ota_QSPI_context);
    if (smif_status != CY_SMIF_SUCCESS)
    {
        result = smif_status;
        goto _bail;
    }

    /* Even after SFDP enumeration QE command is not initialized */
    /* So, it should be 1.0 device */
    if ((smifMemConfigs[0]->deviceCfg->readStsRegQeCmd->command == 0) ||                        /* 0 - if configurator generated code */
        (smifMemConfigs[0]->deviceCfg->readStsRegQeCmd->command == CY_SMIF_NO_COMMAND_OR_MODE)) /* 0xFF's if SFDP enumerated          */
    {
        smif_status = Cy_SMIF_MemInitSfdpMode(SMIF0,
                                              smifMemConfigs[0],
                                              CY_SMIF_WIDTH_QUAD,
                                              CY_SMIF_SFDP_QER_1,
                                              &ota_QSPI_context);
        if (smif_status != CY_SMIF_SUCCESS)
        {
            result = smif_status;
            goto _bail;
        }
    }

    smif_status = IsQuadEnabled(smifMemConfigs[0], &QE_status);
    if(smif_status != CY_RSLT_SUCCESS)
    {
        result = CY_RSLT_TYPE_ERROR;
    }

    /* If not enabled, enable quad mode */
    if(!QE_status)
    {
        /* Enable Quad mode */
        smif_status = EnableQuadMode(smifMemConfigs[0]);
        if(smif_status != CY_RSLT_SUCCESS)
        {
            result = CY_RSLT_TYPE_ERROR;
        }
    }

    SET_FLAG(FLAG_HAL_INIT_DONE);

  _bail:
    /* post-access to SMIF */
    POST_SMIF_ACCESS_TURN_ON_XIP;
#endif

    return result;
}

cy_rslt_t ota_mem_read( cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        (void)result;
        printf( "%s() READ not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;
        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* pre-access to SMIF */
            PRE_SMIF_ACCESS_TURN_OFF_XIP;

            cy_smif_result = (cy_rslt_t)Cy_SMIF_MemRead(SMIF0, smifBlockConfig.memConfig[MEM_SLOT],
                    addr, data, len, &ota_QSPI_context);
            /* post-access to SMIF */
            POST_SMIF_ACCESS_TURN_ON_XIP;
        }

        if (cy_smif_result != CY_SMIF_SUCCESS)
        {
            cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "%s() FAILED cy_smif_result: 0x%x\n", __func__, cy_smif_result);
        }

        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
    }
    else
    {
        printf( "%s() READ not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

cy_rslt_t ota_mem_write( cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        (void)result;
        printf( "%s() Write not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        //printf( "%s() WRITE length 0x%08lx bytes to offset: 0x%08lx \n", __func__, length, offset);

        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* pre-access to SMIF */
            PRE_SMIF_ACCESS_TURN_OFF_XIP;

            cy_smif_result = Cy_SMIF_MemWrite(SMIF0, smifBlockConfig.memConfig[MEM_SLOT],
                    addr, data, len, &ota_QSPI_context);

            /* post-access to SMIF */
            POST_SMIF_ACCESS_TURN_ON_XIP;
        }
        else
        {
            cy_smif_result = CY_RSLT_SERIAL_FLASH_ERR_NOT_INITED;
        }

    #ifdef READBACK_SMIF_WRITE_TEST
        if (cy_smif_result == CY_SMIF_SUCCESS)
        {
            if (ota_mem_read(CY_OTA_MEM_TYPE_EXTERNAL_FLASH, addr, read_back_test, ((16 < length) ? 16 : len)) == CY_RSLT_SUCCESS)
            {
                int i;
                for (i=0; (i<16 && i<len); i++)
                {
                    if(data[i] != read_back_test[i])
                    {
                        printf( "MISMATCH: i:%d 0x%02x  != 0x%02x\n", i, data[i], read_back_test[i]);
                        result  = -1;
                    }
                }
            }
        }
    #endif

        if (cy_smif_result != CY_SMIF_SUCCESS)
        {
            printf("%s() FAILED cy_en_smif_result: [0x%X]\n", __func__, (unsigned int)cy_smif_result);
        }

        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
    }
    else
    {
        printf( "%s() Write not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

cy_rslt_t ota_mem_erase( cy_ota_mem_type_t mem_type, uint32_t addr, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        (void)result;
        printf( "%s() Erase not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* pre-access to SMIF */
            PRE_SMIF_ACCESS_TURN_OFF_XIP;

            // If the erase is for the entire chip, use chip erase command
            if ((addr == 0u) && (len == ota_smif_get_memory_size()))
            {
                cy_smif_result = Cy_SMIF_MemEraseChip(SMIF0,
                                                    smifBlockConfig.memConfig[MEM_SLOT],
                                                    &ota_QSPI_context);
            }
            else
            {
                // Cy_SMIF_MemEraseSector() returns error if (addr + length) > total flash size or if
                // addr is not aligned to erase sector size or if (addr + length) is not aligned to
                // erase sector size.
                /* Make sure the base offset is correct */
                uint32_t erase_size;
                uint32_t diff;
                erase_size = ota_mem_get_erase_size(CY_OTA_MEM_TYPE_EXTERNAL_FLASH, addr);
                diff = addr & (erase_size - 1);
                addr -= diff;
                len += diff;
                /* Make sure the length is correct */
                len = (len + (erase_size - 1)) & ~(erase_size - 1);
                cy_smif_result = Cy_SMIF_MemEraseSector(SMIF0,
                                                      smifBlockConfig.memConfig[MEM_SLOT],
                                                      addr, len, &ota_QSPI_context);
            }

            /* post-access to SMIF */
            POST_SMIF_ACCESS_TURN_ON_XIP;
        }
        else
        {
            return CY_RSLT_SERIAL_FLASH_ERR_NOT_INITED;
        }

        if (cy_smif_result != CY_SMIF_SUCCESS)
        {
            printf( "%s() FAILED cy_en_smif_result: [0x%X]\n", __func__, (unsigned int)cy_smif_result);
        }
        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
    }
    else
    {
        printf( "%s() Erase not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

size_t ota_mem_get_prog_size ( cy_ota_mem_type_t mem_type, uint32_t addr )
{
    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        return 0;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        uint32_t    program_size = 0;
        (void)addr; /* Hybrid parts not yet supported */
        /* pre-access to SMIF is not needed, as we are just reading data from RAM */
        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            if (SMIF0 != NULL)
            {
                program_size = smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->programSize;
            }
        }
        /* post-access to SMIF  is not needed, as we are just reading data from RAM */

        return program_size;
    }
    else
    {
        return 0;
    }
}

size_t ota_mem_get_erase_size ( cy_ota_mem_type_t mem_type, uint32_t addr )
{
    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
        return 0;
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
        uint32_t                            erase_sector_size = 0;
        cy_stc_smif_hybrid_region_info_t*   hybrid_info = NULL;
        cy_en_smif_status_t                 smif_status;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        /* pre-access to SMIF is not needed, as we are just reading data from RAM */
        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* Cy_SMIF_MemLocateHybridRegion() does not access the external flash, just data tables from RAM  */
            smif_status = Cy_SMIF_MemLocateHybridRegion(smifBlockConfig.memConfig[MEM_SLOT], &hybrid_info, addr);

            if (CY_SMIF_SUCCESS != smif_status)
            {
                erase_sector_size = (size_t)smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->eraseSize;
            }
            else
            {
                erase_sector_size = (size_t)hybrid_info->eraseSize;
            }
        }
        /* post-access to SMIF is not needed, as we are just reading data from RAM */

        return erase_sector_size;
    }
    else
    {
        return 0;
    }
}
